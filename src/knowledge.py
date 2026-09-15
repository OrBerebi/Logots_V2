#!/usr/bin/env python3
"""knowledge — the robot's memory: reading state and writing the .md files.

Maps to the "knowledge/ · .md files" box in docs/v2_architecture/architecture.svg.

    guidelines.md      authored input — seeded with the core rules if missing
    roster.md          one line per plant known
    plants/<id>.md     per-plant profile
    species/<name>.md  care knowledge (from the model, offline for now)
    room_map.md        pose where each plant was seen
    journal.md         appended: what each function run did
    current_state.md   rewritten: the robot's "now"
"""
from __future__ import annotations

import os
import re
from datetime import datetime

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_KNOWLEDGE = os.path.join(REPO, "knowledge")

GUIDELINES = """# Guidelines

You are Logots, a home plant-care robot. Core rules:

1. If the roster is empty (no plants known), the `initiation` function runs:
   survey the surroundings, identify every plant, and record each one in memory.
2. Never invent knowledge about a plant you have not seen.
3. Prefer doing nothing over acting on uncertainty.
"""


def state(kdir: str) -> str:
    roster = os.path.join(kdir, "roster.md")
    if not os.path.exists(roster):
        return "roster: does not exist yet — 0 plants known; all memory files empty"
    n = sum(1 for line in open(roster)
            if line.startswith("| ") and "plant id" not in line and "---" not in line)
    return f"roster: {n} plants known"


def roster_empty(kdir: str) -> bool:
    return "0 plants" in state(kdir) or "does not exist" in state(kdir)


def seed(kdir: str) -> None:
    os.makedirs(os.path.join(kdir, "plants"), exist_ok=True)
    os.makedirs(os.path.join(kdir, "species"), exist_ok=True)
    path = os.path.join(kdir, "guidelines.md")
    if not os.path.exists(path):
        open(path, "w").write(GUIDELINES)


def guidelines(kdir: str) -> str:
    return open(os.path.join(kdir, "guidelines.md")).read()


SPECIES_RULE = ('the common species name, from what you actually see. If you '
                'cannot tell from the current view, do NOT guess — use '
                'approach_plant and inspect_plant to get a closer look first; '
                'answer "unknown" only if you still cannot tell up close')


def normalize_plants(plants: list) -> list[dict]:
    """Tolerate loose model output; guarantee every key write() needs."""
    out = []
    for i, p in enumerate(plants):
        if not isinstance(p, dict):
            continue
        p.setdefault("id", f"plant_{i + 1}")
        p.setdefault("species", "unknown")
        p.setdefault("confidence", "unstated")
        p.setdefault("description", "(none given)")
        p.setdefault("care", "(none given)")
        out.append(p)
    return out


def write(kdir: str, plants: list[dict], sightings: list[dict], journal_line: str) -> list[str]:
    """Write every document a function run produces. Returns the paths written."""
    now = datetime.now().isoformat(timespec="seconds")
    written = []

    def w(rel: str, text: str, mode: str = "w"):
        with open(os.path.join(kdir, rel), mode) as f:
            f.write(text)
        written.append(rel)

    rows = "\n".join(f"| {p['id']} | {p['species']} | {now} |" for p in plants)
    w("roster.md", f"# Plant roster\n\nInitiated {now}.\n\n"
                   f"| plant id | species | first seen |\n|---|---|---|\n{rows}\n")

    for p in plants:
        w(f"plants/{p['id']}.md",
          f"# {p['id']}\n\n- species: {p['species']} (confidence: {p['confidence']})\n"
          f"- first seen: {now}\n"
          f"- description: {p['description']}\n- last watered: never (just initiated)\n")

    for sp in sorted({p["species"] for p in plants}):
        care = next(p["care"] for p in plants if p["species"] == sp)
        w(f"species/{re.sub(r'[^a-z0-9]+', '_', sp.lower())}.md",
          f"# {sp}\n\n- care: {care}\n- source: model knowledge (offline); verify online later\n")

    sight = "\n".join(f"| {s['id']} | ({s['pos_x']:.2f}, {s['pos_y']:.2f}) | "
                      f"{s['heading']:.0f}° | pan {s['pan_angle']}° |" for s in sightings)
    w("room_map.md", f"# Room map\n\nPose is dead-reckoned (drifts); origin = run start.\n\n"
                     f"| plant id | body (x, y) m | heading | camera |\n|---|---|---|---|\n{sight}\n")

    w("journal.md", f"- {now} — {journal_line}\n", "a")

    w("current_state.md", f"# Current state\n\n## now\n- {now}: initiation complete\n"
                          f"- plants known: {len(plants)}\n## pending\n- first care round not yet planned\n")
    return written


def check(kdir: str, plants: list[dict]) -> bool:
    n_files = len(os.listdir(os.path.join(kdir, "plants")))
    ok = n_files == len(plants) and os.path.exists(os.path.join(kdir, "journal.md"))
    print(f"[check] roster={len(plants)} plant_files={n_files} journal=yes "
          f"→ {'CONSISTENT' if ok else 'MISMATCH'}", flush=True)
    return ok
