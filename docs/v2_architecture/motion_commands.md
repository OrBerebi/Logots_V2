# Motion commands: traverse, rotate, servo

*Written 2026-11-10 by Or (Jetson + Arduino side). For Asaph, who wires these into the LLM
side. **None of this has run on the robot yet** — see "Known issues" in `CLAUDE.md` for the
first-run checklist.*

The LLM controls the robot's motion with three commands. Each is **sent once**; the robot
carries it out by itself, stops by itself, and reports back once. There is nothing to
stream, time or stop from the LLM side.

| Command | Argument | Meaning |
|---|---|---|
| `traverse` | `cm` | Drive straight. `+` = forward, `-` = backward. `-30` = 30 cm backwards. |
| `rotate` | `degrees` | Turn on the spot. `+` = left, `-` = right. `-45` = 45° to the right. |
| `servo` | `pan`, `tilt` | Point the camera. `pan`: 0 = front, `+` = left. `tilt`: 0 = level, `+` = up. `(45, 10)` = 45° left, 10° up. Absolute, not relative. |

## How to send one

Exactly like `approach_plant` today: publish a decision on `:8788` (`ActionsEndpoint.publish`),
then wait for the completion (`wait_for_done`). `ActionReader` in `src/logots_ui.py` picks it
up from `GET /latest_action` and answers with `POST /action_done`.

```json
{"action": "traverse", "args": {"cm": -30}}
{"action": "rotate",   "args": {"degrees": -45}}
{"action": "servo",    "args": {"pan": 45, "tilt": 10}}
```

Suggested `- args:` lines for `knowledge/actions.md` (your file; nothing was added there):

```
traverse:  cm:int(-200..200)
rotate:    degrees:int(-180..180)
servo:     pan:int(-45..45), tilt:int(-90..90)
```

## Ranges

| Argument | Accepted | Outside the range |
|---|---|---|
| `cm` | −200 … +200 | limited to ±200, reported in `problem` (there is no obstacle detection) |
| `degrees` | −180 … +180 | limited to ±180, reported in `problem` |
| `pan` | −45 … +45 | limited to ±45 (same cap as the GUI slider), reported in `problem` |
| `tilt` | −90 … +90 | limited to what the servo can reach, reported in `problem` |

Numbers may be floats; servo angles are rounded to whole degrees.

## The completion report

Every report has the pose after the command, as before:

```json
{"action_id": 1794306249655, "pos_x": -0.16, "pos_y": -0.19, "heading": 39.4, ...}
```

plus, per command:

| Command | Extra field | What it is |
|---|---|---|
| `traverse` | `moved_cm` | distance actually driven, from the wheel encoders (signed) |
| `rotate` | `turned_deg` | angle actually turned, from the IMU (signed) |
| `servo` | `pan`, `tilt` | the angles actually set (after any limiting) |

and **`problem`** (a string) only when the command did not go exactly as asked: it was
limited, stopped by a person (SPACE / STOP in the GUI), timed out, or could not run (Arduino
not connected, old firmware). The numbers in the report are always what really happened,
so a stopped `traverse` reports the centimetres it covered before stopping.

How long to wait: a traverse takes about 6 s per metre, a rotation a few seconds, a servo
move under a second. The existing `DONE_WAIT_S` = 120 s covers all of them.

## What you need to handle on the LLM side

- **The step history only shows position.** `run_function()` writes
  `moved (x, y) → (x, y)` after an actuator action. A `rotate` or `servo` doesn't change
  position, so it reads as "nothing changed", and the prompt's rule ("stop when your actions
  produce no change") tells the model to give up. Show `heading` (from the report) for
  `rotate`, `pan`/`tilt` for `servo`, and `problem` when present.
- **Heading convention**: `heading` grows when the robot turns left, the same sign as
  `rotate`. `rotate: +30` raises `heading` by about 30.
- **Tilt in the data is still the raw servo angle.** `pan_angle` in the frame API / mart
  rows is already signed like the command. `tilt_angle` there is the raw servo angle
  (0…180, level = 90): signed tilt = `tilt_angle − 90`. Only the `servo` command and its
  report use signed tilt.
- **`approach_plant` still works** (PWM left/right for a duration), so nothing breaks before
  you switch. It is the only way left to make the wheels do anything other than a straight
  traverse or an on-the-spot rotation; once you no longer use it, tell Or and it can go.
- **Sim mode**: commands are ignored, as `approach_plant` is today, so they time out.

## Expected accuracy (estimates until tested)

- `traverse`: the encoders were within ~2% of a tape measure on 2026-10-03. Heading during
  the drive is held from the IMU.
- `rotate`: aimed at ±1.5° (the IMU is checked after the turn and a correction turn is made
  if needed).
- `servo`: whole degrees; which raw direction is "up" still has to be confirmed on the robot.
