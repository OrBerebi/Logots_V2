#!/usr/bin/env python3
"""audio_on_demand — the robot's ears. Event-driven; asleep by default.

    wake word ("Hey Jarvis" for now) → record until ~1 s silence → utterance
    → Gemma (audio in, actions out) → served on GET :8788/latest_action

The mirror of the frame API: sensor reality flows GUI→consumers on :8787;
decisions flow back to the actuator side on :8788 (see docs/v2_architecture/
action_api.md). Poll it and dedupe on `action_id`, exactly like latest_frame.

Modes
  live (default)  audio pulled from the frame API on :8787 (the robot's mic)
  --wav FILE      same pipeline fed from a wav — our equivalent of SIM mode

Brains
  --brain gemma      local Gemma 4 (E4B); ~16 GB, first call loads the model
  --brain llamacpp   local Gemma 4 E2B GGUF via a persistent llama-server (Jetson path)
  --brain mock       keyword rules on the wav filename — pipeline tests, no model

Examples
  python src/audio_on_demand.py --wav experiments/audio_on_demand/recordings/tts_water_ficus.wav --brain mock --once
  python src/audio_on_demand.py --brain gemma            # live, on the robot
"""
from __future__ import annotations

import argparse
import atexit
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import threading
import time
import urllib.error
import urllib.request
import wave
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import sounddevice as sd
import soundfile as sf

# ── Tunables ──────────────────────────────────────────────────────────────────
AUDIO_RATE      = 16000   # Hz — the system-wide mic rate
CHUNK           = 1280    # samples = 80 ms, openWakeWord's native step
WAKE_THRESHOLD  = 0.5     # wake-word score above this = triggered
SILENCE_RMS     = 300     # int16 RMS below this counts as silence
SILENCE_S       = 1.0     # this much silence closes the capture
MAX_UTTERANCE_S = 28.0    # hard cap (Gemma's audio ceiling is 30 s)
ACTION_PORT     = 8788
MODEL_ID        = "google/gemma-4-E4B-it"

# TTS for `speak` actions — local Piper voice, played out the robot's I2S amp.
PIPER_VOICE = os.environ.get("PIPER_VOICE", os.path.expanduser("~/models/piper/en_US-lessac-medium.onnx"))
# "demixer" — the named ALSA PCM in /etc/asound.conf (plug + dmix over hw:APE,0, the real
# I2S/MAX98357A hardware). Named explicitly, not "default": a remote-desktop session (e.g.
# NoMachine) can run its own PulseAudio server and hijack the ALSA "default" device, silently
# redirecting playback to the *client* machine's speakers instead of the robot's. The raw
# "hw:1,0" device works too but has no rate conversion (Piper's 22050 Hz output plays back
# sped-up/high-pitched on the hardware's fixed 48000 Hz rate) — "demixer"'s "plug" layer
# handles that automatically, same as "default" used to before NoMachine intercepted it.
SPEAKER_DEVICE = os.environ.get("SPEAKER_DEVICE", "demixer")

# llama.cpp brain (Jetson path): a persistent llama-server subprocess, GPU-offloaded
# (fixed by the r36.5 JetPack upgrade — see PLAN_llamacpp_gpu_offload.md), loads the
# ~4.3 GB model+mmproj once and stays warm for the process's life, so decide() is a
# fast HTTP call instead of a fresh process + full model load each utterance.
# Override via env if llama.cpp or the GGUFs live somewhere else.
LLAMACPP_BIN     = os.environ.get("LLAMACPP_BIN", os.path.expanduser("~/llama.cpp/build/bin/llama-server"))
LLAMACPP_MODEL   = os.environ.get("LLAMACPP_MODEL", os.path.expanduser("~/models/gemma-4-E2B-it-qat-q4_0-gguf/gemma-4-E2B_q4_0-it.gguf"))
LLAMACPP_MMPROJ  = os.environ.get("LLAMACPP_MMPROJ", os.path.expanduser("~/models/gemma-4-E2B-it-qat-q4_0-gguf/gemma-4-E2B-it-mmproj.gguf"))
LLAMACPP_CTX     = 4096
LLAMACPP_N_PREDICT = 1000  # generous: this GGUF's reasoning trace length varies a lot before the JSON verdict
LLAMACPP_HOST    = os.environ.get("LLAMACPP_HOST", "127.0.0.1")
LLAMACPP_PORT    = int(os.environ.get("LLAMACPP_PORT", "8789"))       # distinct from 8787/8788
LLAMACPP_GPU_LAYERS = int(os.environ.get("LLAMACPP_GPU_LAYERS", "999"))
LLAMACPP_MEDIA_DIR  = os.environ.get("LLAMACPP_MEDIA_DIR", "/tmp/logots_llamacpp_media")
LLAMACPP_STARTUP_TIMEOUT_S = 120   # one-time model load off disk; generous but bounded

# ── The action schema (the Or↔Asaf contract; keep in sync with action_api.md) ─
ACTIONS = {
    "approach_plant": ("id",),
    "return_to_base": (),
    "scan_room":      (),
    "initiation":     (),
    "inspect_plant":  ("id",),
    "capture_photo":  ("subject",),
    "water_plant":    ("id", "ml"),
    "speak":          ("text",),
    "flag_issue":     ("sev", "msg"),
    "daily_summary":  (),
    "wait_until":     ("next",),
}

BRAIN_PROMPT = f"""You are the decision layer of a plant-care robot. You just heard a
person speak (the clip begins with the wake phrase — ignore it). Choose ONE action.

Actions and their args: {json.dumps({a: list(k) for a, k in ACTIONS.items()})}

Reply with STRICT JSON only: {{"action": "...", "args": {{...}}}}.
If the request does not map to any action other than speak, reply with
{{"action": "speak", "args": {{"text": "<briefly say you can't do that and list what you can do>"}}}}.
If the person asked a question, answer it with speak."""


# ── Ears: wake word + VAD state machine ───────────────────────────────────────
class Ears:
    """Feed 80 ms int16 chunks; returns a finished utterance (int16) or None."""

    def __init__(self, wake_model: str = "hey_jarvis"):
        print(f"[ears] loading wake-word model ({wake_model}) …", flush=True)
        from openwakeword.model import Model      # lazy: heavy-ish import
        # tflite-runtime (openwakeword's default backend) is compiled against NumPy 1.x
        # and crashes under NumPy 2.x ("_ARRAY_API not found") — force the onnx backend,
        # which works fine under either, since callers of this class run under both
        # (logots: NumPy 2.x, logots-audio: NumPy 1.x).
        self._oww = Model(wakeword_models=[wake_model], inference_framework="onnx")
        print("[ears] wake-word model loaded", flush=True)
        self._wake = wake_model
        self._buf: list[np.ndarray] = []
        self.capturing = False
        self._silent_chunks = 0

    def feed(self, chunk: np.ndarray):
        if not self.capturing:
            score = self._oww.predict(chunk)[self._wake]
            if score >= WAKE_THRESHOLD:
                self.capturing = True
                self._buf = [chunk]               # utterance includes the wake tail
                self._silent_chunks = 0
            return None

        self._buf.append(chunk)
        rms = float(np.sqrt(np.mean(chunk.astype(np.float64) ** 2)))
        self._silent_chunks = self._silent_chunks + 1 if rms < SILENCE_RMS else 0
        done = (self._silent_chunks * CHUNK / AUDIO_RATE >= SILENCE_S
                or len(self._buf) * CHUNK / AUDIO_RATE >= MAX_UTTERANCE_S)
        if not done:
            return None

        return self.flush()

    def flush(self):
        """Force-close the current capture (end of a wav = end of speech)."""
        if not self.capturing:
            return None
        utterance = np.concatenate(self._buf)
        self.capturing, self._buf = False, []
        self._oww.reset()                          # clear the detector's history
        return utterance


# ── Brains ────────────────────────────────────────────────────────────────────
class MockBrain:
    """Filename-keyword rules — proves the pipeline with no model loaded."""

    def decide(self, wav_path: str) -> dict:
        name = os.path.basename(wav_path).lower()
        if "water" in name:
            return {"action": "water_plant", "args": {"id": "ficus_1", "ml": 50}}
        if "inspect" in name or "look" in name:
            return {"action": "inspect_plant", "args": {"id": "ficus_1"}}
        return {"action": "speak", "args": {"text": "(mock) I can't do that."}}


class GemmaBrain:
    """Local Gemma 4: utterance audio in → one action out. Lazy-loads."""

    def __init__(self, model_id: str = MODEL_ID):
        self.model_id = model_id
        self._pipe = None

    def _pipeline(self):
        if self._pipe is None:
            import torch
            from transformers import pipeline
            device = "mps" if torch.backends.mps.is_available() else \
                     ("cuda" if torch.cuda.is_available() else "cpu")
            print(f"[brain] loading {self.model_id} on {device} …", flush=True)
            self._pipe = pipeline(task="any-to-any", model=self.model_id,
                                  dtype=torch.bfloat16, device=device)
        return self._pipe

    def decide(self, wav_path: str) -> dict:
        messages = [
            {"role": "system", "content": [{"type": "text", "text": BRAIN_PROMPT}]},
            {"role": "user", "content": [{"type": "audio", "audio": wav_path}]},
        ]
        out = self._pipeline()(text=messages, max_new_tokens=128)
        return _parse_action(_assistant_text(out))


class LlamaCppBrain:
    """Local Gemma 4 E2B via a persistent llama-server (Jetson path).

    GPU-offloaded (--gpu-layers; the r36.5 JetPack upgrade fixed the
    CUDA/nvmap allocator bug that used to OOM this on GPU — see
    PLAN_llamacpp_gpu_offload.md). The server is spawned once, lazily (or
    eagerly from main()), and stays resident for the process's lifetime, so
    decide() is a fast HTTP call against an already-warm model rather than a
    fresh process + full model load. Still uses --jinja for the GGUF's own
    chat template and --temp 0 for reproducibility; --reasoning-format none
    keeps the <channel|>-delimited reasoning trace inline in the response
    text, the same shape the old CLI stdout had, so the existing
    strip-and-parse logic is unchanged.
    """

    def __init__(self, binary: str = LLAMACPP_BIN, model: str = LLAMACPP_MODEL,
                 mmproj: str = LLAMACPP_MMPROJ, host: str = LLAMACPP_HOST,
                 port: int = LLAMACPP_PORT, media_dir: str = LLAMACPP_MEDIA_DIR):
        self.binary, self.model, self.mmproj = binary, model, mmproj
        self.host, self.port, self.media_dir = host, port, media_dir
        self._proc = None
        self._atexit_registered = False

    def _ensure_server(self):
        if self._proc is not None and self._proc.poll() is None:
            return  # already running
        os.makedirs(self.media_dir, exist_ok=True)
        cmd = [
            self.binary,
            "-m", self.model,
            "--mmproj", self.mmproj,
            "--host", self.host, "--port", str(self.port),
            "-c", str(LLAMACPP_CTX),
            "-n", str(LLAMACPP_N_PREDICT),
            "--temp", "0",
            "--gpu-layers", str(LLAMACPP_GPU_LAYERS),
            "--jinja",
            "--reasoning-format", "none",
            "--reasoning", "off",  # this GGUF's chain-of-thought adds ~300 tokens (~10s) for no
                                    # accuracy gain on this schema-constrained task; off cuts to ~1-2s
            "--media-path", self.media_dir,
        ]
        print(f"[brain] starting llama-server on {self.host}:{self.port} …", flush=True)
        self._proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if not self._atexit_registered:
            atexit.register(self._terminate)
            self._atexit_registered = True

        health_url = f"http://{self.host}:{self.port}/health"
        deadline = time.time() + LLAMACPP_STARTUP_TIMEOUT_S
        while time.time() < deadline:
            if self._proc.poll() is not None:
                raise RuntimeError(f"llama-server exited during startup (code {self._proc.returncode})")
            try:
                with urllib.request.urlopen(health_url, timeout=2) as r:
                    if r.status == 200:
                        print("[brain] llama-server ready", flush=True)
                        return
            except (urllib.error.URLError, OSError):
                pass
            time.sleep(0.5)
        raise RuntimeError(f"llama-server didn't become ready within {LLAMACPP_STARTUP_TIMEOUT_S}s")

    def _terminate(self):
        if self._proc and self._proc.poll() is None:
            try:
                self._proc.terminate()
                self._proc.wait(timeout=5)
            except Exception:
                try:
                    self._proc.kill()
                except Exception:
                    pass

    def decide(self, wav_path: str) -> dict:
        self._ensure_server()
        # llama-server only resolves file:// URLs under --media-path; the live pipeline's
        # tempfile already lands there (main()'s dir=scratch_dir), but decide() shouldn't
        # assume that of every caller (e.g. tests passing an arbitrary path directly).
        fname = os.path.basename(wav_path)
        media_target = os.path.join(self.media_dir, fname)
        copied = os.path.abspath(wav_path) != os.path.abspath(media_target)
        if copied:
            shutil.copy(wav_path, media_target)
        body = {
            "messages": [
                {"role": "system", "content": BRAIN_PROMPT},
                {"role": "user", "content": [
                    {"type": "input_audio", "input_audio": {"url": f"file://{fname}"}},
                ]},
            ],
            "temperature": 0,
            "max_tokens": LLAMACPP_N_PREDICT,
            "reasoning_format": "none",
        }
        req = urllib.request.Request(
            f"http://{self.host}:{self.port}/v1/chat/completions",
            data=json.dumps(body).encode(),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        try:
            with urllib.request.urlopen(req, timeout=30) as r:
                resp = json.loads(r.read())
            out = resp["choices"][0]["message"]["content"]
        except Exception as e:
            print(f"[brain] llama-server request failed: {e}", flush=True)
            out = ""
        finally:
            if copied:
                try:
                    os.unlink(media_target)
                except OSError:
                    pass
        reply = out.rsplit("<channel|>", 1)[-1]  # drop the reasoning trace, keep the verdict
        return _parse_action(reply)


def _assistant_text(out) -> str:
    if isinstance(out, list) and out:
        out = out[0]
    gen = out.get("generated_text", out) if isinstance(out, dict) else out
    if isinstance(gen, list) and gen:
        content = gen[-1].get("content", gen[-1]) if isinstance(gen[-1], dict) else gen[-1]
        if isinstance(content, list):
            return "\n".join(b.get("text", "") for b in content if isinstance(b, dict)).strip()
        return str(content).strip()
    return str(gen).strip()


def _parse_action(reply: str) -> dict:
    """Model text → validated {action, args}; anything malformed → spoken refusal."""
    m = re.search(r"\{.*\}", reply, re.DOTALL)
    try:
        d = json.loads(m.group(0)) if m else {}
    except json.JSONDecodeError:
        d = {}
    action, args = d.get("action"), d.get("args", {})
    if action in ACTIONS and isinstance(args, dict) and set(args) <= set(ACTIONS[action]):
        return {"action": action, "args": args}
    return {"action": "speak",
            "args": {"text": "I couldn't map that to something I can do."}}


# ── Mouth: TTS for `speak` actions, played out the robot's I2S amp ────────────
_piper_voice = None


def _get_piper_voice():
    global _piper_voice
    if _piper_voice is None:
        from piper import PiperVoice
        print(f"[mouth] loading Piper voice ({PIPER_VOICE}) …", flush=True)
        _piper_voice = PiperVoice.load(PIPER_VOICE)
    return _piper_voice


def speak(text: str):
    """Synthesize text and play it out the speaker. Never raises — a TTS hiccup
    shouldn't crash the daemon or block the action already published."""
    try:
        voice = _get_piper_voice()
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp:
            path = tmp.name
        with wave.open(path, "wb") as wav_file:
            voice.synthesize_wav(text, wav_file)
        data, fs = sf.read(path)
        os.unlink(path)
        sd.play(data, fs, device=SPEAKER_DEVICE)
        sd.wait()
    except Exception as e:
        print(f"[mouth] speak failed: {e}", flush=True)


# ── Action server (:8788) — the mirror of FrameServer ─────────────────────────
class ActionServer:
    def __init__(self, port: int = ACTION_PORT, sim_mode: bool = False):
        self._latest = None
        self._next_id = 1
        self._lock = threading.Lock()
        outer = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path != "/latest_action":
                    self.send_error(404); return
                with outer._lock:
                    payload = outer._latest
                if payload is None:
                    self.send_error(503, "no action yet"); return
                body = json.dumps(payload).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def log_message(self, *a):  # quiet
                pass

        self._httpd = ThreadingHTTPServer(("localhost", port), Handler)
        threading.Thread(target=self._httpd.serve_forever, daemon=True).start()
        self._sim = sim_mode
        print(f"[server] GET http://localhost:{port}/latest_action", flush=True)

    def publish(self, decision: dict) -> dict:
        from datetime import datetime
        with self._lock:
            payload = {"action_id": self._next_id,
                       "ts": datetime.now().isoformat(),
                       **decision, "sim_mode": self._sim}
            self._latest, self._next_id = payload, self._next_id + 1
        return payload


# ── Audio sources ─────────────────────────────────────────────────────────────
def wav_chunks(path: str, realtime: bool = False):
    """Stream a wav as 80 ms int16 chunks, as if it were the live mic."""
    audio, sr = sf.read(path, dtype="int16")
    if audio.ndim > 1:
        audio = audio[:, 0]
    assert sr == AUDIO_RATE, f"expected {AUDIO_RATE} Hz wav, got {sr}"
    for i in range(0, len(audio) - CHUNK + 1, CHUNK):
        yield audio[i:i + CHUNK]
        if realtime:
            time.sleep(CHUNK / AUDIO_RATE)


def api_chunks(host: str = "localhost", port: int = 8787):
    """Live mode: pull the mic stream from the frame API, dedupe on frame_id."""
    print(f"[ears] connecting to frame API on {host}:{port} …", flush=True)
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from logots_api import get_latest_frame
    last, leftover = None, np.zeros(0, dtype=np.int16)
    last_error = None
    while True:
        try:
            fr = get_latest_frame(host=host, port=port, decode_image=False)
        except Exception as e:
            err = str(e)
            if err != last_error:  # surface it, but don't spam identical repeats
                print(f"[ears] frame API poll failed: {err}", flush=True)
                last_error = err
            time.sleep(0.25); continue
        if last_error is not None:
            print("[ears] frame API poll recovered", flush=True)
            last_error = None
        if fr["frame_id"] != last:
            if last is None:
                print("[ears] receiving live audio", flush=True)
            last = fr["frame_id"]
            floats = np.asarray(fr["audio_samples"], dtype=np.float32)
            pcm = np.concatenate([leftover, (floats * 32767).astype(np.int16)])
            n = (len(pcm) // CHUNK) * CHUNK
            for i in range(0, n, CHUNK):
                yield pcm[i:i + CHUNK]
            leftover = pcm[n:]
        time.sleep(0.03)


# ── Main loop ─────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(description="audio_on_demand — wake word → Gemma → action API")
    ap.add_argument("--wav", help="replay a wav instead of the live frame API (SIM equivalent)")
    ap.add_argument("--realtime", action="store_true", help="pace wav replay at real time")
    ap.add_argument("--brain", choices=["gemma", "llamacpp", "mock"], default="gemma")
    ap.add_argument("--once", action="store_true", help="exit after the first action")
    ap.add_argument("--port", type=int, default=ACTION_PORT)
    args = ap.parse_args()

    ears = Ears()
    brain = {"gemma": GemmaBrain, "llamacpp": LlamaCppBrain, "mock": MockBrain}[args.brain]()
    scratch_dir = None
    if args.brain == "llamacpp":
        os.makedirs(LLAMACPP_MEDIA_DIR, exist_ok=True)
        scratch_dir = LLAMACPP_MEDIA_DIR
        brain._ensure_server()  # pay the one-time model load now, not on the first real utterance
    server = ActionServer(port=args.port, sim_mode=bool(args.wav))
    source = wav_chunks(args.wav, args.realtime) if args.wav else api_chunks()
    print(f"[ears] asleep — waiting for the wake word ({'wav' if args.wav else 'live'} mode)", flush=True)

    def handle(utterance) -> dict:
        print(f"[ears] utterance closed ({len(utterance)/AUDIO_RATE:.1f} s) → brain", flush=True)
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False, dir=scratch_dir) as tmp:
            sf.write(tmp.name, utterance, AUDIO_RATE)
            t0 = time.time()
            decision = brain.decide(tmp.name)
        os.unlink(tmp.name)
        payload = server.publish(decision)
        print(f"[action] #{payload['action_id']}  {payload['action']}({payload['args']})  "
              f"[{time.time()-t0:.1f} s]", flush=True)
        if payload["action"] == "speak":
            speak(payload["args"]["text"])
        return payload

    for chunk in source:
        utterance = ears.feed(chunk)
        if ears.capturing and len(ears._buf) == 1:
            print("[ears] wake word heard — recording …", flush=True)
        if utterance is None:
            continue
        handle(utterance)
        if args.once:
            return

    # wav exhausted: EOF means the speech is over — flush any open capture
    utterance = ears.flush()
    if utterance is not None:
        handle(utterance)
        if args.once:
            return
    if args.wav:                       # keep serving for pollers
        print("[ears] wav finished — still serving; Ctrl+C to quit", flush=True)
        while True:
            time.sleep(1)


if __name__ == "__main__":
    main()
