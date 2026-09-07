#!/usr/bin/env python3
"""initiation — cold-start onboarding: survey the room, learn the plants, write memory.

    read knowledge/ (empty on first run) → Gemma decides `initiation` is needed
    → publish the decision on :8788 → survey (motion by the actuator side; in SIM
    the recording *is* the survey) → collect one pass of experiences (1 fps frames
    + pose from the frame API) → Gemma identifies the plants → write knowledge/*.md

Run it exactly like audio_on_demand.py — the GUI must be serving :8787
(SIM replaying a recording, or the live robot):

    python src/initiation.py                      # one full cycle, then exits
    python src/initiation.py --knowledge-dir experiments/initiation_e/knowledge

The knowledge layer written (see docs/v2_architecture/report.md):
    guidelines.md      input — seeded with the trigger rule if missing
    roster.md          created: one line per plant found
    plants/<id>.md     created: per-plant profile
    species/<name>.md  created: care knowledge (from the model, offline for now)
    room_map.md        created: pose + camera angle where each plant was seen
    journal.md         appended: what initiation did
    current_state.md   rewritten: the robot's "now"

Scope: initiation today means "survey from where you stand" — one fixed motion
request (rotate in place, pan the camera) that the actuator side executes; in SIM
the recording stands in for it. Moving through the house (multi-room initiation)
does not exist yet anywhere — it needs navigation on a trusted map.

Brain: auto-selected like the GUI's voice assistant — llama.cpp/E2B when the
llama-server binary exists (the robot, and any machine mirroring it), else the
transformers/E4B pipeline. Override with VOICE_BRAIN=llamacpp|gemma.
"""
from __future__ import annotations

import argparse
import io
import json
import os
import re
import sys
import time
from collections import deque
from datetime import datetime

import numpy as np
from PIL import Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from logots_api import get_latest_frame
from audio_on_demand import (ACTIONS, ActionServer, ACTION_PORT, MODEL_ID,
                             LlamaCppBrain, LLAMACPP_BIN, LLAMACPP_N_PREDICT)

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_KNOWLEDGE = os.path.join(REPO, "knowledge")
SURVEY_MAX_S   = 90     # cap on collection (live robot has no loop wrap to stop at)
SURVEY_FRAMES  = 8      # frames handed to Gemma, sampled evenly across the pass

GUIDELINES = """# Guidelines

You are Logots, a home plant-care robot. Core rules:

1. If the roster is empty (no plants known), run `initiation`: survey the room,
   identify every plant, and record each one in memory.
2. Never invent knowledge about a plant you have not seen.
3. Prefer doing nothing (`wait_until`) over acting on uncertainty.
"""

DECIDE_PROMPT = """You are the decision layer of a plant-care robot.

Guidelines:
{guidelines}

Current knowledge state: {state}

Actions and their args: {actions}

Choose ONE action. Reply with STRICT JSON only: {{"action": "...", "args": {{...}}}}."""

IDENTIFY_PROMPT = """You are a plant-care robot surveying a room. These frames were taken
during one survey pass; frame captions give the body pose and camera angle at capture.

Identify each DISTINCT plant you can see (the same plant may appear in several frames —
list it once). For each, give:
- id: short snake_case name (e.g. "ficus_1")
- species: choose from the SPECIES CATALOG below — pick the entry whose visual
  description best matches what you see (use "other" only if nothing fits at all)
- confidence: how sure you are of the species — "low", "medium" or "high"
- seen_in_frame: the frame number where it is clearest
- description: one sentence — appearance, pot, surroundings
- care: one sentence — watering interval and light needs for that species

Reply with STRICT JSON only: a list of objects with exactly those keys.
If you see no plants at all, reply [] — do not invent any."""


# ── Gemma (text + images in, JSON out) ────────────────────────────────────────
class VisionBrain:
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

    def ask(self, text: str, images: list | None = None, max_new_tokens: int = 1024) -> str:
        content = [{"type": "image", "image": im} for im in (images or [])]
        content.append({"type": "text", "text": text})
        out = self._pipeline()(text=[{"role": "user", "content": content}],
                               max_new_tokens=max_new_tokens)
        if isinstance(out, list) and out:
            out = out[0]
        gen = out.get("generated_text", out) if isinstance(out, dict) else out
        if isinstance(gen, list) and gen:
            c = gen[-1].get("content", gen[-1]) if isinstance(gen[-1], dict) else gen[-1]
            if isinstance(c, list):
                return "\n".join(b.get("text", "") for b in c if isinstance(b, dict)).strip()
            return str(c).strip()
        return str(gen).strip()


class LlamaCppVisionBrain(LlamaCppBrain):
    """Text+images through Darab's llama-server — his class untouched, extended
    here (our file) with an ask() so the same server that hears can also see.
    Images go through the server's --media-path mechanism, like his audio does."""

    def ask(self, text: str, images: list | None = None, max_new_tokens: int = LLAMACPP_N_PREDICT) -> str:
        import shutil, urllib.request
        self._ensure_server()
        os.makedirs(self.media_dir, exist_ok=True)
        names = []
        for i, im in enumerate(images or []):
            name = f"initiation_{i}.jpg"
            im.save(os.path.join(self.media_dir, name), "JPEG", quality=90)
            names.append(name)
        content = [{"type": "image_url", "image_url": {"url": f"file://{n}"}} for n in names]
        content.append({"type": "text", "text": text})
        body = {"messages": [{"role": "user", "content": content}],
                "temperature": 0, "max_tokens": max_new_tokens,
                "reasoning_format": "none"}
        req = urllib.request.Request(f"http://{self.host}:{self.port}/v1/chat/completions",
                                     data=json.dumps(body).encode(),
                                     headers={"Content-Type": "application/json"}, method="POST")
        try:
            with urllib.request.urlopen(req, timeout=180) as r:
                out = json.loads(r.read())["choices"][0]["message"]["content"]
        finally:
            for n in names:
                try: os.unlink(os.path.join(self.media_dir, n))
                except OSError: pass
        return out.rsplit("<channel|>", 1)[-1].strip()   # same reasoning-trace strip as his


def pick_brain():
    """Same auto-select convention as Darab's GUI: llama.cpp when his server
    binary exists (Jetson — and now this Mac too), else transformers. Override
    with VOICE_BRAIN=llamacpp|gemma."""
    choice = os.environ.get("VOICE_BRAIN", "auto")
    if choice == "llamacpp" or (choice == "auto" and os.path.exists(LLAMACPP_BIN)):
        print("[brain] using LlamaCppVisionBrain (llama.cpp — same as the robot)", flush=True)
        return LlamaCppVisionBrain()
    print("[brain] using VisionBrain (transformers)", flush=True)
    return VisionBrain()


def _json_block(reply: str):
    m = re.search(r"\[.*\]|\{.*\}", reply, re.DOTALL)
    if not m:
        raise ValueError(f"no JSON in model reply: {reply[:200]!r}")
    block = m.group(0)
    try:
        return json.loads(block)
    except json.JSONDecodeError:
        pass
    # cheap repairs for the model's common slips, then one more try
    repaired = block.replace("“", "'").replace("”", "'").replace("’", "'")
    repaired = re.sub(r",\s*([}\]])", r"\1", repaired)              # trailing commas
    repaired = re.sub(r'(?<=[A-Za-z0-9])"(?=[A-Za-z0-9 ])', "'", repaired)  # inner quotes
    try:
        return json.loads(repaired)
    except json.JSONDecodeError as e:
        raise ValueError(f"unparseable JSON from model ({e}); raw reply:\n{reply}") from e


# ── Knowledge layer ───────────────────────────────────────────────────────────
def knowledge_state(kdir: str) -> str:
    roster = os.path.join(kdir, "roster.md")
    if not os.path.exists(roster):
        return "roster: does not exist yet — 0 plants known; all memory files empty"
    n = sum(1 for line in open(roster) if line.startswith("| ") and "plant id" not in line and "---" not in line)
    return f"roster: {n} plants known"


def seed_guidelines(kdir: str) -> None:
    os.makedirs(os.path.join(kdir, "plants"), exist_ok=True)
    os.makedirs(os.path.join(kdir, "species"), exist_ok=True)
    path = os.path.join(kdir, "guidelines.md")
    if not os.path.exists(path):
        open(path, "w").write(GUIDELINES)


def write_knowledge(kdir: str, plants: list[dict], sightings: list[dict]) -> list[str]:
    """Write every document initiation owns. Returns the paths written."""
    now = datetime.now().isoformat(timespec="seconds")
    written = []

    def w(rel: str, text: str, mode: str = "w"):
        path = os.path.join(kdir, rel)
        with open(path, mode) as f:
            f.write(text)
        written.append(rel)

    rows = "\n".join(f"| {p['id']} | {p['species']} | frame {p['seen_in_frame']} |"
                     for p in plants)
    w("roster.md", f"# Plant roster\n\nInitiated {now}.\n\n"
                   f"| plant id | species | first seen |\n|---|---|---|\n{rows}\n")

    for p in plants:
        w(f"plants/{p['id']}.md",
          f"# {p['id']}\n\n- species: {p['species']} (confidence: {p.get('confidence', 'unstated')})\n"
          f"- first seen: {now}\n"
          f"- description: {p['description']}\n- last watered: never (just initiated)\n")

    for sp in sorted({p["species"] for p in plants}):
        care = next(p["care"] for p in plants if p["species"] == sp)
        w(f"species/{re.sub(r'[^a-z0-9]+', '_', sp)}.md",
          f"# {sp}\n\n- care: {care}\n- source: model knowledge (offline); verify online later\n")

    sight = "\n".join(f"| {s['id']} | ({s['pos_x']:.2f}, {s['pos_y']:.2f}) | "
                      f"{s['heading']:.0f}° | pan {s['pan_angle']}° |" for s in sightings)
    w("room_map.md", f"# Room map\n\nPose is dead-reckoned (drifts); origin = survey start.\n\n"
                     f"| plant id | body (x, y) m | heading | camera |\n|---|---|---|---|\n{sight}\n")

    w("journal.md", f"- {now} — initiation: surveyed the room, found "
                    f"{len(plants)} plant(s): {', '.join(p['id'] for p in plants) or 'none'}\n", "a")

    w("current_state.md", f"# Current state\n\n## now\n- {now}: initiation complete\n"
                          f"- plants known: {len(plants)}\n## pending\n- first care round not yet planned\n")
    return written


# ── Survey collection (frames + pose from the frame API) ──────────────────────
def collect_pass(max_s: float = SURVEY_MAX_S) -> list[dict]:
    """One survey's experiences at 1 fps: {sec, image, pos_x, pos_y, heading, pan_angle}.

    In SIM the recording loops — a backward frame_id jump = pass complete (sync to the
    first jump, stop at the next). On the live robot ids only climb: collect max_s."""
    rows, last, synced = [], None, False
    bucket_sec, bucket = None, None
    t0 = time.time()
    while time.time() - t0 < max_s + 45:            # +45: sync wait headroom in SIM
        try:
            fr = get_latest_frame(decode_image=True)
        except Exception:
            time.sleep(0.25); continue
        if fr["frame_id"] != last:
            if last is not None and fr["frame_id"] < last:   # loop wrap
                if not synced:
                    synced, t0 = True, time.time()           # start of a clean pass
                    print("[survey] synced to recording start", flush=True)
                else:
                    break                                    # full pass collected
            if synced or fr["sim_mode"] is False:            # live robot: no sync needed
                sec = int(datetime.fromisoformat(fr["timestamp"]).timestamp())
                if sec != bucket_sec:
                    if bucket is not None:
                        rows.append(bucket)
                    bucket_sec = sec
                    bucket = {"sec": sec, "image": fr["image"],
                              "pos_x": fr.get("pos_x", 0.0), "pos_y": fr.get("pos_y", 0.0),
                              "heading": fr.get("heading", 0.0), "pan_angle": fr["pan_angle"]}
                if not fr["sim_mode"] and time.time() - t0 >= max_s:
                    break
            last = fr["frame_id"]
        time.sleep(0.03)
    if bucket is not None:
        rows.append(bucket)
    return rows


# ── Main cycle ────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(description="initiation — survey the room, write knowledge/")
    ap.add_argument("--knowledge-dir", default=os.environ.get("KNOWLEDGE_DIR", DEFAULT_KNOWLEDGE))
    ap.add_argument("--survey-s", type=float, default=SURVEY_MAX_S)
    args = ap.parse_args()

    kdir = args.knowledge_dir
    seed_guidelines(kdir)
    state = knowledge_state(kdir)
    print(f"[knowledge] {kdir}\n[knowledge] state: {state}", flush=True)

    brain = pick_brain()

    # 1. The decision — Gemma chooses, given guidelines + state. Not hard-coded.
    reply = brain.ask(DECIDE_PROMPT.format(
        guidelines=open(os.path.join(kdir, "guidelines.md")).read(),
        state=state,
        actions=json.dumps({a: list(k) for a, k in ACTIONS.items()})))
    decision = _json_block(reply)
    print(f"[decide] Gemma chose: {decision}", flush=True)

    # 2. Publish it on the action API (the actuator side executes motion; in SIM
    #    nobody polls and the recording already contains the survey movement).
    try:
        server = ActionServer(port=ACTION_PORT)
        server.publish(decision)
    except OSError:
        print(f"[server] :{ACTION_PORT} already taken (GUI voice assistant?) — "
              f"decision printed only", flush=True)
    if decision.get("action") != "initiation":
        print("[decide] not initiation — nothing to do; exiting", flush=True)
        return
    print("[survey] motion request: rotate slowly in place, camera pan sweeping "
          "30–150° (actuator side's job — ignored in SIM)", flush=True)

    # 3. Collect one survey pass of experiences.
    rows = collect_pass(args.survey_s)
    print(f"[survey] collected {len(rows)} experiences (1 fps)", flush=True)
    if not rows:
        print("[survey] no experiences — is the GUI serving :8787?", flush=True)
        return

    # 4. Identify the plants from sampled frames.
    idx = np.linspace(0, len(rows) - 1, min(SURVEY_FRAMES, len(rows))).round().astype(int)
    picked = [rows[i] for i in sorted(set(idx))]
    captions = "\n".join(
        f"frame {n}: body at ({r['pos_x']:.2f}, {r['pos_y']:.2f}) heading {r['heading']:.0f}°, "
        f"camera pan {r['pan_angle']}°" for n, r in enumerate(picked))
    t0 = time.time()
    prompt = IDENTIFY_PROMPT + "\n\nFrame captions:\n" + captions
    catalog = os.path.join(kdir, "species_catalog.md")
    if os.path.exists(catalog):
        prompt += "\n\nSPECIES CATALOG:\n" + open(catalog).read()
    images = [Image.fromarray(r["image"]) for r in picked]
    try:
        plants = _json_block(brain.ask(prompt, images=images))
    except ValueError as e:
        print(f"[identify] invalid JSON, retrying once — {str(e)[:120]}", flush=True)
        plants = _json_block(brain.ask(
            prompt + "\n\nIMPORTANT: output ONLY valid JSON. Do not use any quotation "
                     "marks inside string values.", images=images))
    print(f"[identify] {len(plants)} plant(s) in {time.time()-t0:.1f}s: "
          f"{[p.get('id') for p in plants]}", flush=True)

    # 5. Write the knowledge layer; sightings use the pose of each plant's clearest frame.
    sightings = []
    for p in plants:
        # tolerate loose model output: "frame 4" / "4" / missing → a frame index
        m = re.search(r"\d+", str(p.get("seen_in_frame", 0)))
        n = min(int(m.group(0)) if m else 0, len(picked) - 1)
        p["seen_in_frame"] = n                      # normalized for the roster row
        p.setdefault("id", f"plant_{len(sightings)+1}")
        p.setdefault("species", "unknown")
        p.setdefault("description", "(none given)")
        p.setdefault("care", "(none given)")
        r = picked[n]
        sightings.append({"id": p["id"], "pos_x": r["pos_x"], "pos_y": r["pos_y"],
                          "heading": r["heading"], "pan_angle": r["pan_angle"]})
    written = write_knowledge(kdir, plants, sightings)
    print("[knowledge] wrote:", flush=True)
    for relpath in written:
        print(f"  - {relpath}", flush=True)

    # 6. Integrity check.
    n_plant_files = len(os.listdir(os.path.join(kdir, "plants")))
    ok = n_plant_files == len(plants) and os.path.exists(os.path.join(kdir, "journal.md"))
    print(f"[check] roster={len(plants)} plant_files={n_plant_files} journal=yes "
          f"→ {'CONSISTENT' if ok else 'MISMATCH'}", flush=True)
    print(f"[knowledge] new state: {knowledge_state(kdir)}", flush=True)


if __name__ == "__main__":
    main()
