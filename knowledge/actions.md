# Actions — the Or↔Asaf contract

The closed set of atomic actions the LLM may choose. A function (initiation,
watering) is a loop over these. This file is the single source of truth: the
prompt shows it to the LLM verbatim, and the decoding schema is built from it —
the action name AND its args (names, types, ranges) are enforced at generation
time, so a decision always carries exactly the values the executor needs.

Arg syntax: `name:type` — `string`, `int(min..max)`, `number(min..max)`;
a trailing `...` means the action may carry extra free-form result fields.

"executed by": **actuator** = Darab's side (polls GET :8788, drives the motors
via his Arduino protocol, reports done via POST /action_done) · **data** =
Asaf's side, served inside the loop.

| action | args | executed by | description |
|---|---|---|---|
| approach_plant | id:string, left_pwm:int(-255..255), right_pwm:int(-255..255), duration_s:number(0.1..10) | actuator | drive toward the plant — YOU output the exact motor command: left/right wheel PWM (positive = forward) and how many seconds to run it |
| inspect_plant | id:string | data | look: fetch a fresh camera view (frame + pose) of where you are now |
| speak | text:string | actuator | say the text out loud |
| finish | summary:string, ... | loop | end the running function; args carry its results |
