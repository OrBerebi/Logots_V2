# Action API — manual

*Asaf → Or. This is the channel through which my side hands you transformed data:
decisions. It is the mirror of your frame API — reality flows to me on `:8787`,
decisions flow back to you on `:8788`. Same pattern both ways: HTTP on localhost,
JSON, poll + dedupe.*

**Status: implemented and running.** The server lives inside `src/audio_on_demand.py`
(my first producer — voice command → Gemma → action). A complete working demo, with a
real run baked in, is `experiments/audio_on_demand/audio_on_demand.ipynb`.

---

## 1. The endpoint

```
GET http://localhost:8788/latest_action
```

Returns the most recent decision, always. A real response from today's run:

```json
{
  "action_id": 3,
  "ts": "2026-07-27T23:44:12.807517",
  "action": "speak",
  "args": { "text": "I'm sorry, I can't dance for you. I can approach a plant, scan the room, or water a plant." },
  "sim_mode": true
}
```

| field | meaning |
|---|---|
| `action_id` | 1-based, consecutive. **Your dedupe key — execute each id once.** |
| `ts` | when the decision was made (ISO 8601) |
| `action` | one of the schema below, always validated before publishing |
| `args` | the action's arguments (schema below) |
| `sim_mode` | `true` when the producer is replaying a recording (my SIM equivalent) |

Semantics — identical to your `latest_frame`:

- **Poll and dedupe on `action_id`.** Polling faster than decisions arrive returns the
  same action again. Decisions are sparse (seconds-to-minutes apart), so your existing
  20 Hz loop rate is far more than enough.
- **`503`** until the first decision exists (mirrors your server).
- Localhost only, no auth — same trust model as `:8787`.

## 2. Client — your pattern, inverted

The consumer loop is your own `api_demo.py` pattern:

```python
import json, time, urllib.request

last_id = None
while True:
    try:
        with urllib.request.urlopen("http://localhost:8788/latest_action", timeout=2) as r:
            a = json.loads(r.read())
        if a["action_id"] != last_id:          # new decision — execute exactly once
            last_id = a["action_id"]
            execute(a["action"], a["args"])    # your actuator dispatch
    except OSError:
        pass                                   # 503 / producer not up yet
    time.sleep(0.05)
```

## 3. The action schema

| action | args | executed by |
|---|---|---|
| `approach_plant` | `id` | motors |
| `return_to_base` | — | motors |
| `scan_room` | — | motors + pan/tilt |
| `initiation` | — | motors + pan/tilt |
| `inspect_plant` | `id` | pan/tilt |
| `capture_photo` | `subject` | pan/tilt |
| `water_plant` | `id`, `ml` | pump |
| `speak` | `text` | speaker (TTS later; text for now) |
| `flag_issue` | `sev`, `msg` | — (my side: notify human) |
| `daily_summary` | — | — (my side) |
| `wait_until` | `next` ISO ts | — (nobody moves) |

Every decision is published, including ones with no physical effect — one stream, full
transparency. You execute the rows whose *executed by* touches your hardware and ignore
the rest. Malformed model output never reaches you: anything that doesn't validate
against this schema is replaced by a `speak` explaining the failure.

## 4. The first producer: voice commands (`audio_on_demand`)

What feeds the API today — the robot's ears:

```
asleep → wake word → record until ~1 s silence → utterance → local Gemma 4 → action on :8788
```

Run it (conda env `gemma-lab` — needs `openwakeword`, `soundfile`, and the gated Gemma
weights via `huggingface-cli login`; see `experiments/gemma_investigation/README.md`):

```bash
# replay a wav through the pipeline — my equivalent of your SIM mode:
python src/audio_on_demand.py --wav experiments/audio_on_demand/recordings/tts_combined.wav --brain gemma

# live on the robot — audio comes from YOUR frame API on :8787:
python src/audio_on_demand.py --brain gemma
```

Measured (Mac, M-series): model load ~49 s once, then **~2–4 s per utterance** warm.
Latest three-scenario run: *"water the ficus"* → `water_plant({id: ficus, ml: 100})`;
*"how are my plants doing?"* → `daily_summary` (a previous run answered with `speak` —
ambiguous requests vary run to run); *"dance for me"* → `speak` (a refusal listing the
robot's real capabilities).

**To see it yourself:** open `experiments/audio_on_demand/audio_on_demand.ipynb` —
it records the scenarios, launches this service, and polls it with the client above;
the outputs of a real run are baked in.

## 5. Honest status — what's tested and what isn't

- ✅ wav mode end-to-end (wake → VAD → Gemma → API), on the Mac.
- ✅ **Live mode end-to-end through both APIs**, on the Mac: Asaf spoke *"Hey Jarvis,
  please water the ficus"* into a session recorded with **your** GUI (⚫ REC), replayed
  it in SIM, and the service — pulling audio live from your `:8787` — woke, understood,
  and published `water_plant({id: Ficus, ml: 100})` on `:8788`. Recording:
  `recordings/session_20260727_234304/`. Still to do on the robot itself: same test
  with the I2S mic.
- ⚠️ Wake phrase is pretrained **"Hey Jarvis"** for now; custom **"Hi Robot"** needs a
  small trained model (planned).
- ⚠️ Action args are not yet grounded — Gemma invents `ml` and answers about plant
  health it can't know. The `.md` knowledge layer (roster, journal, species) fixes
  this; it's the next component on my side.
- ⚠️ **Does Gemma E4B fit the Jetson's 8 GB?** Everything above is measured on the Mac.
  If you've already run Gemma on the robot, tell me what fits.

## 6. Running this on the robot — what's still missing

"Say Hey Jarvis to the robot and it works" is **not yet true**. What it takes:

- **Two processes on the Jetson:** your GUI (owns the mic, serves `:8787`) + my service
  (`python src/audio_on_demand.py --brain gemma`). The audio path is your API — no new
  wiring.
- **Publishing is not executing.** `:8788` only announces the decision; nothing moves
  until **your actuator loop polls it and executes** (the §2 client). That integration
  is yours and doesn't exist yet.
- Deployment gaps, in order:
  1. **Does Gemma fit?** E4B is ~16 GB in bf16; the Jetson has 8 GB unified. Likely
     needs a quantized variant — the blocker. What Gemma, if any, runs on it today?
  2. A Python env on the Jetson with `transformers` 5.x + `openwakeword` (onnxruntime,
     ARM64) + `soundfile` — the ARM64 sibling of my `gemma-lab`.
  3. The gated Gemma weights (`huggingface-cli login`) on the robot.
  4. An on-robot wake-word test with the I2S mic — sensitivity there is unproven.

## 7. Open for v2 of this contract

**Acknowledgment.** I currently learn nothing about execution: did watering happen, did
the approach fail. Proposal: you expose `GET :8787/action_status`
(`{action_id, status: running|done|failed, detail}`), or POST results back to me. My
journal wants "watered ficus ✓", not "asked to water" — and a `failed` lets the LLM
retry or flag.

*Not in this contract:* navigation itself (how `approach_plant` finds the plant) —
motion control is yours; I supply the target and, from the map, its stored position.
