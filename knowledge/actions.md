# Actions — the Or↔Asaf contract

The closed set of atomic actions the LLM may choose. A function (initiation,
watering) is a loop over these. This file is the single source of truth: the
prompt shows it to the LLM verbatim, and the decoding schema is built from it —
the action name AND its args (names, types, ranges) are enforced at generation
time, so a decision always carries exactly the values the executor needs.

Arg syntax: `name:type` — `string`, `int(min..max)`, `number(min..max)`;
a trailing `...` means the action may carry extra free-form result fields.
"executed by": **actuator** = Or's side (`ActionReader` polls GET :8788,
drives the motors, reports done via POST :8788/action_done) · **data** =
Asaf's side, served inside the loop.

## inspect
- executed by: data
- what: look — fetches the newest data row (camera frame + position) and
  returns your reading of it (observation, species guess, confidence)
- when: whenever you want current data — your attached image only updates
  when you inspect
- output: {"action": "inspect", "args": {}}

## approach_plant
- executed by: actuator
- args: id:string, left_pwm:int(-255..255), right_pwm:int(-255..255), duration_s:number(0.1..10)
- what: drive toward the plant — YOU output the exact motor command: left/right
  wheel PWM (positive = forward, equal values = straight) and seconds to run
- physics (measured on the robot): PWM below ~65 does not move the wheels at
  all (a silent no-op); at PWM 150 real forward speed is ~0.165 m/s, so to
  cover N metres use duration_s ≈ N / 0.165 at PWM 150
- when: you want a closer or better view than your last look gave you
- output: {"action": "approach_plant", "args": {"id": "...", "left_pwm": 150, "right_pwm": 150, "duration_s": 2.0}}

## speak
- executed by: actuator
- args: text:string
- what: say the text out loud
- when: the human should hear something right now
- output: {"action": "speak", "args": {"text": "..."}}

## finish
- executed by: loop
- args: summary:string, ...
- what: end the running function and deliver its results
- when: the goal is achieved (e.g. classified with medium/high confidence) — or
  continuing clearly leads nowhere; then explain why in the summary
- output: {"action": "finish", "args": {"summary": "...", ...function results}}
