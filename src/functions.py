#!/usr/bin/env python3
"""functions — orchestration: multi-step routines that drive Gemma one action at a time.

Maps to the "functions" box in docs/v2_architecture/architecture.svg.

A *function* (initiation, watering) is a loop:
    build prompt → Gemma picks ONE action → publish it on :8788 → wait for the
    actuator side's "done" → append the result to the prompt → prompt again,
    until Gemma calls `finish`. The growing prompt is the routine's memory.

`inspect` and `finish` are handled on our side (mart last row / end of
loop); every other action is published for the actuator side to execute.

Completion feedback: after publishing an actuator action, the loop waits for
POST /action_done (Or's ActionReader sends it after driving). There is NO fake
completion anywhere: if nothing reports done — a replayed recording, a robot
with dead motors — the wait times out and the truth ("no completion signal;
position unchanged") goes into the history for the LLM to cope with.

Run (the GUI must be serving :8787):

    python src/functions.py                 # trigger check → run what's needed
    python src/functions.py --done-wait 15  # shorter completion timeout (replay tests)
    python src/functions.py --speak         # speak each step's "thought" out loud
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

try:
    # Reuses the exact same Piper/"demixer" TTS path as the voice assistant
    # (see CLAUDE.md "Voice assistant") — self-contained, not dependent on
    # logots_ui.py running. Lazily loads the Piper voice on first call.
    from audio_on_demand import speak as _speak
except Exception as _e:                     # pragma: no cover - missing audio deps
    _speak, _speak_import_error = None, _e

RUNS_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "runs")

MAX_STEPS   = 12    # step budget per function run — a stuck loop must end
DONE_WAIT_S = 120   # how long to wait for the actuator side's completion signal

# Constrained decoding (llama.cpp), built from knowledge/actions.md: only the
# listed actions are generatable, each with exactly its declared args (types,
# ranges — e.g. approach_plant's left/right PWM). "thought" comes first so the
# model reasons toward the goal before committing to an action; it is passed to
# the next step's prompt (goal-oriented chaining) and rides along in the :8788
# payload so the actuator side may speak it. The VALUES are the LLM's decision.
def build_step_schema(finish_args_schema: dict | None = None) -> dict:
    """The decision schema; a function may pass a strict schema for finish's
    args so its results (e.g. initiation's plant list) cannot be omitted."""
    variants = []
    for name in ACTIONS:
        args = ACTION_ARG_SCHEMAS[name]
        if name == "finish" and finish_args_schema:
            args = finish_args_schema
        variants.append({"type": "object",
                         "properties": {"thought": {"type": "string"},
                                        "action": {"const": name},
                                        "args": args},
                         "required": ["thought", "action", "args"]})
    return {"anyOf": variants}


ACTION_SCHEMA = build_step_schema()

STEP_PROMPT = """{identity}

You work in GOAL-ORIENTED STEPS:
read what has happened so far, think toward the goal, and choose ONE action.
Your "thought" is passed to your next step — write it as a note to your future
self about where you stand on the goal.

{goal_block}

ACTIONS — the only actions that exist:
{actions}

STEPS SO FAR — you are at step {step} of {max_steps}; the run is cut off after
step {max_steps}, so achieve the goal before then:
{history}

RULES:
- Choose exactly ONE action per reply.
- If repeating an action is not changing what you see or know, stop repeating —
  choose finish and explain why in the summary.
{finish_rule}

Knowledge state: {state}

{view_block}
Reply with STRICT JSON only: {{"thought": "...", "action": "...", "args": {{...}}}}"""

VIEW_NONE = """YOU HAVE NOT LOOKED YET — no image attached, you have no visual data."""

VIEW_LAST = """THE IMAGE ATTACHED is what you saw when you last inspected (step {n}), from
pos ({px:.2f}, {py:.2f}), heading {hdg:.0f}°, camera pan {pan}°. The world may have
changed since then; only inspect fetches what is there now."""

OBSERVE_PROMPT = """You are Logots, a plant-care robot. You just inspected your
surroundings — the attached image is what you see right now, from pos
({px:.2f}, {py:.2f}), heading {hdg:.0f}°.

In one sentence, describe what you can see. If a plant is visible, give your
best species guess from its visible features, and how confident you are; if no
plant is visible, say so in the observation and use "none" as the guess.
Reply with STRICT JSON only: {{"observation": "...", "species_guess": "...",
"confidence": "low"|"medium"|"high"}}"""

OBSERVE_SCHEMA = {
    "type": "object",
    "properties": {
        "observation":   {"type": "string"},
        "species_guess": {"type": "string"},
        "confidence":    {"enum": ["low", "medium", "high"]},
    },
    "required": ["observation", "species_guess", "confidence"],
}


def wait_for_done(endpoint, action_id: int, timeout: float = DONE_WAIT_S) -> dict | None:
    """Block until the actuator side reports the action done (POST /action_done).
    No fake completions: if nothing reports — replay, dead motors — this times
    out and returns None, and the loop records the truth."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        if endpoint:
            report = endpoint.completion(action_id)
            if report:
                return report
        time.sleep(0.5)
    return None


def run_function(name: str, goal_block: str, finish_rule: str, brain, endpoint,
                 kdir: str, max_steps: int = MAX_STEPS,
                 finish_args_schema: dict | None = None,
                 done_wait: float = DONE_WAIT_S,
                 speak_thoughts: bool = False):
    """The loop. Returns (finish_decision | None, history, last_view).
    The full prompt chain is written to runs/<name>_<ts>.log for review."""
    os.makedirs(RUNS_DIR, exist_ok=True)
    log_path = os.path.join(RUNS_DIR, f"{name}_{datetime.now():%Y%m%d_%H%M%S}.log")
    log = open(log_path, "w")
    print(f"[functions] prompt log: {os.path.relpath(log_path)}", flush=True)

    history: list[str] = []
    view, view_step = None, None      # no data until the LLM inspects
    for step in range(1, max_steps + 1):
        view_block = VIEW_NONE if view is None else VIEW_LAST.format(
            n=view_step, px=view["pos_x"], py=view["pos_y"],
            hdg=view["heading"], pan=view["pan_angle"])
        prompt = STEP_PROMPT.format(
            identity=knowledge.identity(kdir),
            goal_block=goal_block, step=step, max_steps=max_steps,
            state=knowledge.state(kdir),
            actions=ACTIONS_MD,
            finish_rule=finish_rule,
            history="\n".join(f"| {h}" for h in history) or "| (nothing yet — this is the first step)",
            view_block=view_block)
        decision = ask_json(brain, prompt,
                            images=[] if view is None else [Image.fromarray(view["image"])],
                            schema=build_step_schema(finish_args_schema))
        thought = decision.get("thought", "")
        action, args = decision.get("action"), decision.get("args", {})
        log.write(f"{'=' * 74}\nSTEP {step} — PROMPT (image attached only if the LLM has inspected)\n"
                  f"{'=' * 74}\n{prompt}\n\n---- STEP {step} — GEMMA'S DECISION ----\n"
                  f"{json.dumps(decision)}\n\n")
        log.flush()
        print(f"[functions] step {step}: {action}({json.dumps(args)})"
              f"\n[functions]   thought: {thought[:100]}", flush=True)
        entry = f'{step} · thought: "{thought}" → {action}'

        if speak_thoughts and thought:
            _speak(thought)     # blocks until playback finishes — heard before the action runs

        if action == "finish":
            return decision, history, view
        if action == "inspect":
            # Asaf-side: look AND judge — fresh mart row, then Gemma reads it;
            # the result goes into the history so every inspect adds
            # information, not just pixels.
            view, view_step = mrt_experience.last_row(), step
            obs = ask_json(brain, OBSERVE_PROMPT.format(
                px=view["pos_x"], py=view["pos_y"],
                hdg=view["heading"]), images=[Image.fromarray(view["image"])],
                schema=OBSERVE_SCHEMA)
            log.write(f"---- STEP {step} — INSPECT RESULT ----\n{json.dumps(obs)}\n\n")
            log.flush()
            print(f"[functions]   saw: {obs.get('species_guess')} "
                  f"({obs.get('confidence')}) — {str(obs.get('observation'))[:80]}", flush=True)
            history.append(f'{entry} → saw: "{obs.get("observation", "")}" — species guess: '
                           f'{obs.get("species_guess", "?")}, confidence: {obs.get("confidence", "?")}')
            continue

        old = view or mrt_experience.last_row()
        old_x, old_y = old["pos_x"], old["pos_y"]
        payload = endpoint.publish(decision) if endpoint \
            else {"action_id": step}          # :8788 taken — decision printed only
        report = wait_for_done(endpoint, payload["action_id"], done_wait)
        if report:
            history.append(f"{entry}({json.dumps(args)}) → done, moved "
                           f"({old_x:.2f}, {old_y:.2f}) → ({report.get('pos_x', 0):.2f}, "
                           f"{report.get('pos_y', 0):.2f})")
        else:
            now = mrt_experience.last_row()
            history.append(f"{entry}({json.dumps(args)}) → NO completion signal after "
                           f"{done_wait:.0f}s; position unchanged "
                           f"({now['pos_x']:.2f}, {now['pos_y']:.2f}) — the action did "
                           f"not happen; decide how to continue")
            print(f"[functions]   no completion signal after {done_wait:.0f}s", flush=True)
    print(f"[functions] step budget ({max_steps}) exhausted without finish", flush=True)
    return None, history, view


# ── The functions themselves ──────────────────────────────────────────────────
def initiation(brain, endpoint, kdir: str, max_steps: int = MAX_STEPS,
               done_wait: float = DONE_WAIT_S, speak_thoughts: bool = False):
    """Cold start: look around, learn the plants, write all memory docs."""
    goal_block = knowledge.function_goal(kdir, "initiation")
    finish_rule = (
        '- When you finish, `finish` args MUST include "plants": a list with one object '
        'per distinct plant seen, each with exactly these keys: "id" (short snake_case), '
        '"species" (' + knowledge.SPECIES_RULE + '), "confidence" ("low"/"medium"/"high"), '
        '"description" (one sentence), "care" (one sentence: watering + light). '
        'If you see no plants at all, use an empty list — do not invent any.')

    finish_args = {
        "type": "object",
        "properties": {
            "summary": {"type": "string"},
            "plants": {"type": "array", "items": {
                "type": "object",
                "properties": {"id": {"type": "string"}, "species": {"type": "string"},
                               "confidence": {"enum": ["low", "medium", "high"]},
                               "description": {"type": "string"}, "care": {"type": "string"}},
                "required": ["id", "species", "confidence", "description", "care"]}},
        },
        "required": ["summary", "plants"],
    }
    decision, history, view = run_function("initiation", goal_block, finish_rule,
                                           brain, endpoint, kdir, max_steps,
                                           finish_args_schema=finish_args,
                                           done_wait=done_wait,
                                           speak_thoughts=speak_thoughts)
    if decision is None:
        print("[functions] initiation did not finish — nothing written", flush=True)
        return

    plants = knowledge.normalize_plants(decision.get("args", {}).get("plants", []))
    pose = view or mrt_experience.last_row()
    sightings = [{"id": p["id"], "pos_x": pose["pos_x"], "pos_y": pose["pos_y"],
                  "heading": pose["heading"], "pan_angle": pose["pan_angle"]}
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
    """Planned: approach each plant on the roster → inspect → water. Not built yet."""
    print("[functions] watering: not implemented yet", flush=True)


FUNCTIONS = {"initiation": initiation, "watering": watering}


# ── Voice: "Hey Jarvis" → utterance → one action ─────────────────────────────
VOICE_PROMPT = """{identity}

A person just spoke to you — their words are the attached audio. Choose ONE of
your ACTIONS in response. To answer in words, use the speak action — its text
is played on your speaker.

ACTIONS — the only actions that exist:
{actions}

Knowledge state: {state}
Reply with STRICT JSON only: {{"action": "...", "args": {{...}}}}"""

# Voice decisions use the actions.md output structure exactly: action + args,
# schema-enforced like every other Gemma call (no free text, no thought field).
VOICE_SCHEMA = {
    "anyOf": [
        {"type": "object",
         "properties": {"action": {"const": name}, "args": ACTION_ARG_SCHEMAS[name]},
         "required": ["action", "args"]}
        for name in ACTIONS
    ]
}


def _say(text: str):
    print(f'[voice] say: "{text}"', flush=True)
    if _speak is not None:
        _speak(text)


def handle_utterance(utterance, brain, endpoint, kdir: str, done_wait: float, log):
    """One utterance → one free Gemma reply → judged by shape:
    a valid action JSON runs that action; anything else is spoken aloud."""
    import soundfile as sf
    import tempfile
    from audio_on_demand import AUDIO_RATE, LLAMACPP_MEDIA_DIR

    os.makedirs(LLAMACPP_MEDIA_DIR, exist_ok=True)
    with tempfile.NamedTemporaryFile(suffix=".wav", delete=False, dir=LLAMACPP_MEDIA_DIR) as tmp:
        sf.write(tmp.name, utterance, AUDIO_RATE)
        prompt = VOICE_PROMPT.format(identity=knowledge.identity(kdir),
                                     actions=ACTIONS_MD, state=knowledge.state(kdir))
        try:
            decision = ask_json(brain, prompt, schema=VOICE_SCHEMA, audio=tmp.name)
        except ValueError as e:
            decision = {"action": "speak",
                        "args": {"text": "Sorry, I could not form a decision."}}
            print(f"[voice] undecodable reply — {str(e)[:100]}", flush=True)
    os.unlink(tmp.name)
    log.write(f"{'=' * 74}\nUTTERANCE at {datetime.now().isoformat(timespec='seconds')} "
              f"({len(utterance) / AUDIO_RATE:.1f}s audio attached)\n{prompt}\n\n"
              f"---- DECISION ----\n{json.dumps(decision)}\n\n")
    log.flush()
    if decision.get("action") not in ACTIONS:
        decision = {"action": "speak", "args": {"text": "Sorry, I could not form a decision."}}
    print(f"[voice] decision: {decision['action']}({json.dumps(decision['args'])})", flush=True)

    action, args = decision["action"], decision["args"]
    if action == "speak":
        _say(str(args.get("text", "")))
    elif action == "finish":                               # no loop running — just report
        _say(str(args.get("summary", "Nothing to finish right now.")))
    elif action == "inspect":
        view = mrt_experience.last_row()
        obs = ask_json(brain, OBSERVE_PROMPT.format(
            px=view["pos_x"], py=view["pos_y"], hdg=view["heading"]),
            images=[Image.fromarray(view["image"])], schema=OBSERVE_SCHEMA)
        log.write(f"---- INSPECT RESULT ----\n{json.dumps(obs)}\n\n"); log.flush()
        _say(f"{obs.get('observation', '')} My guess: {obs.get('species_guess', 'unknown')}, "
             f"confidence {obs.get('confidence', 'low')}.")
    else:                                                  # actuator action → the contract
        payload = endpoint.publish(decision) if endpoint else {"action_id": 0}
        report = wait_for_done(endpoint, payload["action_id"], done_wait)
        _say("Done." if report else
             f"I tried, but got no completion signal after {done_wait:.0f} seconds.")
    log.write(f"---- EXECUTED ----\n{json.dumps(decision)}\n\n"); log.flush()


def listen(brain, endpoint, kdir: str, done_wait: float, wav: str | None = None):
    """Ears → Gemma → one action per utterance. Live from the frame API, or a
    wav replay for testing (same semantics as audio_on_demand --wav)."""
    from audio_on_demand import Ears, api_chunks, wav_chunks

    os.makedirs(RUNS_DIR, exist_ok=True)
    log_path = os.path.join(RUNS_DIR, f"voice_{datetime.now():%Y%m%d_%H%M%S}.log")
    log = open(log_path, "w")
    print(f"[voice] log: {os.path.relpath(log_path)}", flush=True)

    ears = Ears()
    source = wav_chunks(wav) if wav else api_chunks()
    print(f"[voice] asleep — waiting for the wake word "
          f"({'wav replay' if wav else 'live'} mode)", flush=True)
    for chunk in source:
        utterance = ears.feed(chunk)
        if ears.capturing and len(ears._buf) == 1:
            print("[voice] wake word heard — recording …", flush=True)
        if utterance is not None:
            handle_utterance(utterance, brain, endpoint, kdir, done_wait, log)
    utterance = ears.flush()                               # wav EOF closes the capture
    if utterance is not None:
        handle_utterance(utterance, brain, endpoint, kdir, done_wait, log)


# ── Trigger + entry point ─────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(description="functions — run the routine the robot needs now")
    ap.add_argument("--knowledge-dir", default=os.environ.get("KNOWLEDGE_DIR", knowledge.DEFAULT_KNOWLEDGE))
    ap.add_argument("--max-steps", type=int, default=MAX_STEPS)
    ap.add_argument("--done-wait", type=float, default=DONE_WAIT_S,
                    help="seconds to wait for the actuator side's completion signal")
    ap.add_argument("--speak", action="store_true",
                    help="speak each step's 'thought' aloud on the robot's speaker "
                         "(same Piper/\"demixer\" TTS path as the voice assistant)")
    ap.add_argument("--listen", action="store_true",
                    help="voice mode: wake word → utterance → one action "
                         "(identity + the 4-action contract; free reply = spoken)")
    ap.add_argument("--wav", help="with --listen: replay a wav instead of live audio")
    args = ap.parse_args()

    if args.speak and _speak is None:
        print(f"[functions] --speak requested but audio_on_demand couldn't be imported "
              f"({_speak_import_error}) — continuing without speech", flush=True)
        args.speak = False

    kdir = args.knowledge_dir
    knowledge.seed(kdir)
    print(f"[knowledge] {kdir}\n[knowledge] state: {knowledge.state(kdir)}", flush=True)

    if args.listen:
        brain = pick_brain()
        endpoint = open_endpoint()
        listen(brain, endpoint, kdir, done_wait=args.done_wait, wav=args.wav)
        return

    # The trigger rule (guidelines rule 1): empty roster → initiation.
    if knowledge.roster_empty(kdir):
        print("[functions] roster empty → running initiation", flush=True)
        brain = pick_brain()
        endpoint = open_endpoint()
        initiation(brain, endpoint, kdir, max_steps=args.max_steps,
                  done_wait=args.done_wait, speak_thoughts=args.speak)
    else:
        print("[functions] roster has plants — nothing scheduled "
              "(watering not implemented yet)", flush=True)


if __name__ == "__main__":
    main()
