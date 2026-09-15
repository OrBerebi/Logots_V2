#!/usr/bin/env python3
"""functions — orchestration: multi-step routines that drive Gemma one action at a time.

Maps to the "functions" box in docs/v2_architecture/architecture.svg.

A *function* (initiation, watering) is a loop:
    build prompt → Gemma picks ONE action → publish it on :8788 → wait for the
    actuator side's "done" → append the result to the prompt → prompt again,
    until Gemma calls `finish`. The growing prompt is the routine's memory.

`inspect_plant` and `finish` are handled on our side (mart last row / end of
loop); every other action is published for the actuator side to execute.

Completion feedback: the loop waits for POST /action_done (see actions.py —
proposed contract, Darab's side does not send it yet). While the GUI replays a
recording (frame API reports sim_mode), the wait is stubbed after a few seconds
with the mart's current pose, so the loop mechanics can run end-to-end without
the robot.

Run (the GUI must be serving :8787):

    python src/functions.py                 # trigger check → run what's needed
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from datetime import datetime

from PIL import Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import knowledge
import mrt_experience
from actions import ACTIONS, ACTION_ARG_SCHEMAS, ACTIONS_MD, open_endpoint
from mrt_reflective import pick_brain, ask_json

RUNS_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "runs")

MAX_STEPS   = 12    # step budget per function run — a stuck loop must end
DONE_WAIT_S = 120   # how long to wait for the actuator side's completion signal
STUB_DONE_S = 6     # replay only: fake the completion after this many seconds

# Constrained decoding (llama.cpp), built from knowledge/actions.md: only the
# listed actions are generatable, and each must carry exactly its declared args
# with their types and ranges (e.g. approach_plant's left/right PWM). The
# VALUES stay entirely the LLM's decision.
ACTION_SCHEMA = {
    "anyOf": [
        {"type": "object",
         "properties": {"action": {"const": name}, "args": ACTION_ARG_SCHEMAS[name]},
         "required": ["action", "args"]}
        for name in ACTIONS
    ]
}

STEP_PROMPT = """You are Logots, a plant-care robot, running the function "{name}".
Goal: {goal}

Guidelines:
{guidelines}

Knowledge state: {state}

{actions}

Rules: choose exactly ONE action per reply. `approach_plant` is executed by the
robot's actuator side; you will be told when it is done. `inspect_plant` is how
you look: it fetches a fresh camera view — always inspect after a motion
completes, and never repeat an action already marked done in HISTORY without
inspecting first. When the goal is achieved, call `finish`.
{finish_rule}

HISTORY of this run so far:
{history}

Current view (image attached): pos ({px:.2f}, {py:.2f}), heading {hdg:.0f}°, camera pan {pan}°
Reply with STRICT JSON only: {{"action": "...", "args": {{...}}}}"""


def wait_for_done(endpoint, action_id: int) -> dict | None:
    """Block until the actuator side reports the action done (POST /action_done).
    On a replay (sim_mode from the frame API) the signal is stubbed — the real
    one is the ask to Darab. Returns the report, or None on timeout."""
    t0 = time.time()
    while time.time() - t0 < DONE_WAIT_S:
        if endpoint:
            report = endpoint.completion(action_id)
            if report:
                return report
        row = mrt_experience.last_row()
        if row["sim_mode"] and time.time() - t0 >= STUB_DONE_S:
            print(f"[functions] completion stubbed after {STUB_DONE_S}s "
                  f"(replay — the real signal is the actuator side's job)", flush=True)
            return {"action_id": action_id, "stub": True,
                    "pos_x": row["pos_x"], "pos_y": row["pos_y"],
                    "heading": row["heading"]}
        time.sleep(0.5)
    return None


def run_function(name: str, goal: str, finish_rule: str, brain, endpoint,
                 kdir: str, max_steps: int = MAX_STEPS):
    """The loop. Returns (finish_decision | None, history, last_view).
    The full prompt chain is written to runs/<name>_<ts>.log for review."""
    os.makedirs(RUNS_DIR, exist_ok=True)
    log_path = os.path.join(RUNS_DIR, f"{name}_{datetime.now():%Y%m%d_%H%M%S}.log")
    log = open(log_path, "w")
    print(f"[functions] prompt log: {os.path.relpath(log_path)}", flush=True)

    history: list[str] = []
    view = mrt_experience.last_row()
    for step in range(1, max_steps + 1):
        prompt = STEP_PROMPT.format(
            name=name, goal=goal,
            guidelines=knowledge.guidelines(kdir),
            state=knowledge.state(kdir),
            actions=ACTIONS_MD,
            finish_rule=finish_rule,
            history="\n".join(f"| {h}" for h in history) or "| (nothing yet — this is the first step)",
            px=view["pos_x"], py=view["pos_y"], hdg=view["heading"], pan=view["pan_angle"])
        decision = ask_json(brain, prompt, images=[Image.fromarray(view["image"])],
                            schema=ACTION_SCHEMA)
        action, args = decision.get("action"), decision.get("args", {})
        log.write(f"{'=' * 74}\nSTEP {step} — PROMPT (1 image attached: the current view)\n"
                  f"{'=' * 74}\n{prompt}\n\n---- STEP {step} — GEMMA'S DECISION ----\n"
                  f"{json.dumps(decision)}\n\n")
        log.flush()
        print(f"[functions] step {step}: {action}({json.dumps(args)})", flush=True)

        if action == "finish":
            return decision, history, view
        if action == "inspect_plant":       # Asaf-side: look — fetch the mart's last row
            view = mrt_experience.last_row()
            history.append(f"inspect_plant({json.dumps(args)}) done → fresh view attached, "
                           f"pos ({view['pos_x']:.2f}, {view['pos_y']:.2f}), "
                           f"heading {view['heading']:.0f}°")
            continue
        if action not in ACTIONS:
            history.append(f"{action} is NOT a valid action — choose only from the list")
            continue

        payload = endpoint.publish({"action": action, "args": args}) if endpoint \
            else {"action_id": step}          # :8788 taken — decision printed only
        report = wait_for_done(endpoint, payload["action_id"])
        if report:
            history.append(f"{action}({json.dumps(args)}) done, pos "
                           f"({report.get('pos_x', 0):.2f}, {report.get('pos_y', 0):.2f}) "
                           f"— call inspect_plant to see the result")
        else:
            history.append(f"{action}({json.dumps(args)}) — no completion signal "
                           f"(timeout); decide how to continue")
    print(f"[functions] step budget ({max_steps}) exhausted without finish", flush=True)
    return None, history, view


# ── The functions themselves ──────────────────────────────────────────────────
def initiation(brain, endpoint, kdir: str, max_steps: int = MAX_STEPS):
    """Cold start: look around, learn the plants, write all memory docs."""
    goal = ("Your memory is empty. Identify every distinct plant you can see and "
            "record it. Use approach_plant and inspect_plant to see better if needed.")
    finish_rule = (
        'When you finish, `finish` args MUST include "plants": a list with one object '
        'per distinct plant seen, each with exactly these keys: "id" (short snake_case), '
        '"species" (' + knowledge.SPECIES_RULE + '), "confidence" ("low"/"medium"/"high"), '
        '"description" (one sentence), "care" (one sentence: watering + light). '
        'If you see no plants at all, use an empty list — do not invent any.')

    decision, history, view = run_function("initiation", goal, finish_rule,
                                           brain, endpoint, kdir, max_steps)
    if decision is None:
        print("[functions] initiation did not finish — nothing written", flush=True)
        return

    plants = knowledge.normalize_plants(decision.get("args", {}).get("plants", []))
    sightings = [{"id": p["id"], "pos_x": view["pos_x"], "pos_y": view["pos_y"],
                  "heading": view["heading"], "pan_angle": view["pan_angle"]}
                 for p in plants]                       # pose of the final view (v1 limitation)
    journal = (f"initiation (loop, {len(history) + 1} steps): "
               f"{', '.join(p['id'] for p in plants) or 'no plants found'}")
    written = knowledge.write(kdir, plants, sightings, journal)
    print("[knowledge] wrote:", flush=True)
    for rel in written:
        print(f"  - {rel}", flush=True)
    knowledge.check(kdir, plants)
    print(f"[knowledge] new state: {knowledge.state(kdir)}", flush=True)


def watering(brain, endpoint, kdir: str, **_):
    """Planned: approach each plant on the roster → inspect_plant → water. Not built yet."""
    print("[functions] watering: not implemented yet", flush=True)


FUNCTIONS = {"initiation": initiation, "watering": watering}


# ── Trigger + entry point ─────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(description="functions — run the routine the robot needs now")
    ap.add_argument("--knowledge-dir", default=os.environ.get("KNOWLEDGE_DIR", knowledge.DEFAULT_KNOWLEDGE))
    ap.add_argument("--max-steps", type=int, default=MAX_STEPS)
    args = ap.parse_args()

    kdir = args.knowledge_dir
    knowledge.seed(kdir)
    print(f"[knowledge] {kdir}\n[knowledge] state: {knowledge.state(kdir)}", flush=True)

    # The trigger rule (guidelines rule 1): empty roster → initiation.
    if knowledge.roster_empty(kdir):
        print("[functions] roster empty → running initiation", flush=True)
        brain = pick_brain()
        endpoint = open_endpoint()
        initiation(brain, endpoint, kdir, max_steps=args.max_steps)
    else:
        print("[functions] roster has plants — nothing scheduled "
              "(watering not implemented yet)", flush=True)


if __name__ == "__main__":
    main()
