# The functions layer — 2026-09-14

## Context

Until now Asaf's side had two working pieces: **mrt_experience** (the mart — one
row per second, frame + dead-reckoned pose, last 30 s in RAM, fed by Or's frame
API on :8787) and a one-shot **initiation** script that surveyed a recording,
identified the plant, and wrote the knowledge/*.md files. The old architecture
diagram showed data flowing mart → Gemma → actions, with no way for the LLM to
act in steps.

## The issues

- **No functions layer.** "Initiation" and "watering" are multi-step routines,
  but the code treated initiation as one shot — and even listed it as an
  *action*, which it is not.
- **No loop between the LLM and the actions.** When an action finished (e.g.
  the robot reached the plant), nothing told the LLM — so it could never decide
  the *next* step based on the result.

## The new architecture

See the updated `docs/v2_architecture/architecture.svg`. The new **functions**
box sits between the mart and Gemma. The solution:

- A **function** (initiation; watering next) is a **loop**: prompt Gemma →
  Gemma picks ONE action → the action executes → the result is appended to the
  prompt → Gemma is asked again — until it calls `finish`.
- The prompt **accumulates**: every completed step is added as text, so each
  Gemma call knows the full history. Only the newest camera view is attached
  as an image (context limit: 4096 tokens).
- The action set is **closed and enforced**: `knowledge/actions.md` defines the
  actions, their args, types, and ranges. A JSON schema is built from that file
  and given to llama-server — the model *cannot generate* an action outside the
  list or an approach command without motor values.
- `approach_plant` args are Or's real protocol: `left_pwm`, `right_pwm`
  (−255..255) and `duration_s` — the LLM decides the values, the actuator side
  executes them.
- `inspect_plant` is served on Asaf's side: it fetches the newest mart row
  (fresh frame + pose) so the LLM can look before deciding.
- Every run writes its full prompt chain to `runs/<function>_<timestamp>.log`
  for review.

### Code map (matches the diagram boxes)

| diagram box | file |
|---|---|
| functions (entry point) | `src/functions.py` — run with `python src/functions.py` |
| mrt_experience | `src/mrt_experience.py` |
| mrt_reflective (Gemma) | `src/mrt_reflective.py` |
| ACTIONS contract | `src/actions.py` + `knowledge/actions.md` |
| knowledge/ .md files | `src/knowledge.py` |
| ears | `src/audio_on_demand.py` (unchanged) |

## Mock-up: one initiation run on the robot

```
$ python src/functions.py                ← the frame API must be up

[functions]      knowledge/ is empty (no roster) → start the "initiation" function
[functions]      PROMPT 1 → Gemma: goal + guidelines + actions.md + current view
[mrt_reflective] Gemma → {"action": "inspect_plant", "args": {"id": "plant_1"}}
[mrt_experience] fresh frame + pose attached to the next prompt

[functions]      PROMPT 2 = PROMPT 1 + "| inspect_plant done |"
[mrt_reflective] Gemma → {"action": "approach_plant",
                          "args": {"id": "plant_1", "left_pwm": 100,
                                   "right_pwm": 100, "duration_s": 2}}
[actions]        published on Asaf's API (:8788) — Or's side polls it and drives
[functions]      waiting for completion…  ► the piece Or's side reports back

[functions]      PROMPT 3 = PROMPT 2 + "| approach_plant done, pos (0.8, 0.1) |"
[mrt_reflective] Gemma → {"action": "finish", "args": {"summary": "...",
                          "plants": [{"id": "...", "species": "...", ...}]}}
[knowledge]      writes roster, plants/, species/, room_map, journal, current_state
```

## The actual run and why it did not finish

We ran the loop against a recording replayed by the GUI. The machinery worked
end to end: the empty roster triggered initiation, every decision was a valid
action, and in step 2 the model produced a complete motor command on its own
(`left_pwm 100, right_pwm 100, duration_s 2`). But the model never called
`finish`: it kept inspecting. The reason is the test setup, not the loop — a
recording cannot respond to approach commands, the plant never gets closer, and
the model is instructed not to name a species it cannot see well. (For the same
reason we removed the species catalog: on distant footage it had produced a
confidently *wrong* species, while the bare model honestly said "unknown" —
the right answer is to get closer, which is exactly what `approach_plant` is
for.) The loop can only be validated fully on the real robot.

## Why Asaf's API could not serve the decisions

Both directions are supposed to work the same way: Or serves frames on :8787,
Asaf reads; Asaf serves decisions on :8788, Or reads. Tonight the decisions
were printed instead of served, because the robot program (`logots_ui.py`)
already occupies :8788 — the voice assistant it embeds (from the shared
`audio_on_demand.py`) starts its own copy of the action server on the same
port, with no way to turn it off. Two copies of Asaf's API cannot share the
address.

## Asks for Or

1. **Free :8788** — make the embedded voice-assistant action server optional
   (one small change in `logots_ui.py`), so the functions layer can serve
   Asaf's API.
2. **Completion signal** — when an action finishes, report "action N done" (+
   pose). Proposed mechanism to agree on together; recommendation: a field on
   your :8787 API that Asaf polls, mirroring how frames already flow.
3. **The reader** — code on your side that polls :8788 and executes
   `approach_plant`'s `left_pwm / right_pwm / duration_s` on the motors.
