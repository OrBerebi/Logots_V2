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
  --brain gemma   local Gemma 4 (E4B); ~16 GB, first call loads the model
  --brain mock    keyword rules on the wav filename — pipeline tests, no model

Examples
  python src/audio_on_demand.py --wav experiments/audio_on_demand/recordings/tts_water_ficus.wav --brain mock --once
  python src/audio_on_demand.py --brain gemma            # live, on the robot
"""
from __future__ import annotations

import argparse
import json
import os
import re
import sys
import tempfile
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
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
        from openwakeword.model import Model      # lazy: heavy-ish import
        self._oww = Model(wakeword_models=[wake_model])
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
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from logots_api import get_latest_frame
    last, leftover = None, np.zeros(0, dtype=np.int16)
    while True:
        try:
            fr = get_latest_frame(host=host, port=port, decode_image=False)
        except Exception:
            time.sleep(0.25); continue
        if fr["frame_id"] != last:
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
    ap.add_argument("--brain", choices=["gemma", "mock"], default="gemma")
    ap.add_argument("--once", action="store_true", help="exit after the first action")
    ap.add_argument("--port", type=int, default=ACTION_PORT)
    args = ap.parse_args()

    ears = Ears()
    brain = GemmaBrain() if args.brain == "gemma" else MockBrain()
    server = ActionServer(port=args.port, sim_mode=bool(args.wav))
    source = wav_chunks(args.wav, args.realtime) if args.wav else api_chunks()
    print(f"[ears] asleep — waiting for the wake word ({'wav' if args.wav else 'live'} mode)", flush=True)

    def handle(utterance) -> dict:
        print(f"[ears] utterance closed ({len(utterance)/AUDIO_RATE:.1f} s) → brain", flush=True)
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp:
            sf.write(tmp.name, utterance, AUDIO_RATE)
            t0 = time.time()
            decision = brain.decide(tmp.name)
        os.unlink(tmp.name)
        payload = server.publish(decision)
        print(f"[action] #{payload['action_id']}  {payload['action']}({payload['args']})  "
              f"[{time.time()-t0:.1f} s]", flush=True)
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
