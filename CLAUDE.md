# Logots Robot — Project Guide for Claude

## What this is
A home plant-monitoring and care robot ("PlantSitter"). Runs on a Jetson Orin Nano.
The GUI (`logots_ui.py`) is the single control surface — it shows live sensor data and sends motor/servo commands to an Arduino over I2C.

## Working directory
All work happens in the git repo, which lives on two machines:
- **Jetson (robot)**: `/home/logots/Desktop/Logots_V2/` — do not edit files in the old `/home/logots/Desktop/logots/` directory.
- **MacBook (dev, sim mode)**: `/Users/orberebi/Documents/GitHub/Logots_V2/`

## How to run
```bash
conda run -n logots python src/logots_ui.py
```
- Conda env: `logots` (Python 3.10, NumPy 2.x) — `environment.yml` works on both Jetson and macOS
- Always run through conda — system Python is missing deps and system OpenCV is incompatible (NumPy 1.x vs 2.x)
- On the Mac there is no hardware: sensors show error/unavailable, Arduino stays DISCONNECTED — use **Sim mode** with a recording CSV

## Platform
- **Hardware**: Jetson Orin Nano, JetPack 6.2.2 (L4T R36.5), ARM64 (upgraded 2026-08-02 from R36.4.7 — see Known issues)
- **Remote access**: NoMachine at 192.168.68.114:4000. Virtual display is `:1001.0`
- **Storage**: NVMe nvme0n1p1 (500GB Kingston). SD card removed.
- **Shutdown timeout**: systemd set to 5s (`/etc/systemd/system.conf`)

## Repo structure
```
Logots_V2/
├── README.md
├── CLAUDE.md
├── environment.yml          — conda env spec (python 3.10, numpy 2.x)
├── recordings/              — session CSVs written here (gitignored)
│   └── session_YYYYMMDD_HHMMSS/
│       └── session_YYYYMMDD_HHMMSS.csv
└── src/
    ├── logots_ui.py         — main GUI (all sensors + motor control + recording + sim mode)
    ├── logots_api.py        — HTTP client for the frame API (get_latest_frame)
    ├── api_demo.py          — toy example: video+audio playback via the API
    ├── pinout.txt           — full 40-pin header wiring reference
    └── firmware/
        └── logots_motor_control/
            └── logots_motor_control.ino  — Arduino firmware
```

## Key files
| File | Purpose |
|---|---|
| `src/logots_ui.py` | Main GUI — all sensors + motor control + recording + sim mode + frame API server |
| `src/logots_api.py` | Client module for the frame API — `get_latest_frame()` |
| `src/api_demo.py` | Toy example using the API: video + synced audio playback |
| `src/audio_on_demand.py` | Wake word → local LLM → action → TTS; also runs in-process inside `logots_ui.py` (see "Voice assistant") |
| `src/firmware/logots_motor_control/logots_motor_control.ino` | Arduino firmware |
| `src/pinout.txt` | Full 40-pin header wiring reference |
| `environment.yml` | Conda environment spec |

## Hardware wiring

### I2C buses (40-pin header)
| Bus | Kernel device | Jetson pins | Used for |
|---|---|---|---|
| i2c8 (jetson-io label) | /dev/i2c-7 | Pin 3 (SDA), Pin 5 (SCL) | IMU (MPU-9250, 0x68) |
| gen1 | /dev/i2c-1 | Pin 27 (SDA), Pin 28 (SCL) | Arduino (0x08) |

**Level shifter required on pins 27/28**: Jetson is 3.3V, Arduino is 5V.
LV side → Jetson (LV=3.3V from Pin 1), HV side → Arduino (HV=5V).

### I2S audio (pins shared between mic and amp)
- Pin 12: SCLK, Pin 35: FS, Pin 38: DIN (mic), Pin 40: DOUT (amp)
- Mic: 3.3V power. Amp: 5V power.

### Camera
- IMX219-160 fisheye CSI on CAM0 port
- Device tree overlay enabled via `jetson-io.py` (NEVER edit extlinux.conf manually)

## Arduino firmware protocol
- I2C slave address: `0x08` on `/dev/i2c-1`
- Message format sent by GUI: `"{left_pwm},{right_pwm},{pan_angle},{tilt_angle}\n"`
  - left/right PWM: -255 to +255, positive = forward — on the wire too, and everywhere in
    `logots_ui.py` (joystick, keyboard, `ActionReader`/`approach_plant`, `PositionEstimator`,
    the recording CSV). Each wheel's hardware direction is corrected in the firmware's
    **calibration block** (`LEFT_MOTOR_DIR`/`RIGHT_MOTOR_DIR`), per motor. (Until 2026-10-03
    `_send_motors()` flipped both signs globally; that was removed when the motors were rewired
    to M1/M4, since one wheel can be reversed relative to the other.) If a wheel spins the wrong
    way, flip its `*_MOTOR_DIR` and reflash.
  - pan: -90 to +90 degrees, **0 = camera facing front, + = left** (since 2026-10-03; the horn
    was re-seated so servo 90 = front). The firmware maps it to a servo angle with
    `PAN_CENTER_DEG`/`PAN_DIR` from its calibration block and starts at front on power-up.
  - tilt: 0 to 180 degrees
- **Encoder read-back**: a 10-byte I2C read from `0x08` returns `[0xE5][int32 left][int32 right]
  [sum8 of bytes 0..8]`, little-endian, read by `_read_encoders()` each real tick and shown as
  `EL`/`ER` in the side panel (`----` = no reading). Counts are cumulative 4x-quadrature ticks,
  positive = forward (`LEFT_ENC_DIR`/`RIGHT_ENC_DIR`). Decoded by a pin-change ISR on A0–A3 in
  the firmware. Display only for now; not in the CSV or frame API, and `PositionEstimator`
  doesn't use them yet. A failed or bad read blanks the readout but never disconnects.
- Motor driver: HW-130 (L293D, Adafruit Motor Shield v1 clone, AFMotor.h), M1=right, M4=left
- Encoders: JGA25-370 Hall encoders, right on A0/A1, left on A2/A3 (full wiring: `src/pinout.txt`)
- Pan servo: Arduino pin 10. Tilt servo: Arduino pin 9. Both MG90S.
- Serial debug at 9600 baud: prints `OK  L=X R=X PAN=X TILT=X` per command, plus
  `ENC L=X R=X` every 500 ms while the counts are changing
- Flash from MacBook with Arduino IDE (no Linux ARM64 build exists for IDE 2.x). Compile check
  from the terminal: `"/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli" compile --fqbn arduino:avr:uno --libraries ~/Library/Arduino15/libraries src/firmware/logots_motor_control`

## Camera pipeline
Always requires `EGL_PLATFORM=surfaceless` for headless/NoMachine use:
```
nvarguscamerasrc sensor-id=0
  ! video/x-raw(memory:NVMM),width=640,height=480,framerate=15/1   # CAMERA_FPS
  ! nvvidconv
  ! video/x-raw,format=BGRx
  ! videoconvert
  ! video/x-raw,format=BGR
  ! filesink location=/tmp/logots_camera.fifo
```
Frames are read from the FIFO in `CameraReader` thread. PIL (not cv2) used for display.

## GUI layout
```
  LOGOTS ROBOT CONTROL          ⬤ MIC: asleep  ⬤ LLM: ready  FPS:8.2/10  ⬤ DISCONNECTED
┌─────────────────────┬─────────────────────┐
│  DRIVE & CAM        │  IMU ORIENTATION    │
│  joystick + pan/    │  3D Madgwick AHRS   │
│  tilt sliders +     │  YPR display        │
│  position mini-map  │                     │
├─────────────────────┼─────────────────────┤
│  AUDIO INPUT        │  VIDEO FEED         │
│  waveform + RMS +   │  live IMX219 feed   │
│  ACTION (last voice │                     │
│  command decided)   │                     │
└─────────────────────┴─────────────────────┘
  L +000  R +000  PAN:+00°  TLT:090°  X+0.00 Y+0.00  HDG:090°  ⌖ POS  LOOP  ▶ SIM  ⚫ REC  ■ STOP  OFF
```
- Position mini-map (bottom of DRIVE & CAM, redesigned 2026-10-03, 200×200px): **robot-centered**
  — the heading arrow stays fixed at the canvas center, and the background (1m gridlines +
  trail) pans underneath it as the robot moves, like a radar display, instead of the old
  auto-zoom-to-fit-the-whole-trail behavior. Fixed scale (`POS_MAP_PX_PER_M`=28, ~±3.6m visible
  each direction) so the gridlines are meaningful, labeled distance marks from the session/
  recording origin (brighter axis line + a small circle marker when the origin is in view) —
  older trail points scroll off-canvas as the robot moves on, which is expected for this
  local-area style. `⌖ POS` in the status bar zeros the estimate (origin = here).
- **Encoder counts (`EL`/`ER`) are read internally (feed `PositionEstimator`) but are no
  longer shown in the GUI** (removed 2026-10-03, once the position feature built on them was
  validated) — internal/debug data, not something the user needs to see.
- **`OFF` button (2026-10-03)**: closes the GUI application only — does **not** shut down the
  Jetson. Confirms first, flushes any active recording, sends a final motor-stop, then exits.
- Header `⬤ MIC:`/`⬤ LLM:` indicators and the AUDIO INPUT panel's `ACTION` row are the voice
  assistant's status — see "Voice assistant" below. All status glyphs use the same plain `⬤`
  dingbat as the connection indicator (not emoji — the deployed GUI's font has no color-emoji
  support, confirmed 2026-08-16; don't reach for 🎤/🧠-style pictographic emoji anywhere in this
  file, use `⬤`/`▶`/`⚫`/`■`/`⌖`/`⌨` instead).
- Keyboard: W/S = forward/back, A/D = turn, SPACE = stop
- Arduino auto-connects on startup, retries every 3s if lost

## Recording feature

Press **⚫ REC** to start recording; press again to stop. Output:
```
recordings/session_YYYYMMDD_HHMMSS/session_YYYYMMDD_HHMMSS.csv
```

### CSV schema (staging layer — one row per capture tick, `TARGET_FPS` default 10 Hz)
| Column | Type | Description |
|---|---|---|
| `frame_id` | int | 0-based counter per session |
| `timestamp` | ISO 8601 | wall-clock time |
| `frame_data` | base64 str | **color** 640×640 JPEG, base64-encoded; `""` if no camera (sessions recorded before 2026-07-14 are grayscale — sim playback handles both) |
| `yaw` | JSON array | all IMU yaw readings (°) since last frame |
| `pitch` | JSON array | all IMU pitch readings (°) since last frame |
| `roll` | JSON array | all IMU roll readings (°) since last frame |
| `audio_samples` | JSON array | all mic samples since last frame (~1600 floats at 10 FPS) |
| `left_pwm` | int | left motor command, –255…+255 |
| `right_pwm` | int | right motor command, –255…+255 |
| `pan_angle` | int | camera pan, –90…+90° (0 = front, + = left). Recordings made before 2026-10-03 hold raw servo degrees 0…180 from the old horn position; sim replays them as-is, without converting |
| `tilt_angle` | int | tilt servo, 0…180° |
| `pos_x` | float | estimated body X position (m), origin = recording start; see position note |
| `pos_y` | float | estimated body Y position (m), origin = recording start |
| `heading` | float | body heading (°) used for the estimate = IMU yaw at that tick |

> **Position is dead-reckoned from wheel encoders + IMU heading (2026-10-03), not measured
> odometry.** `PositionEstimator` computes each tick's forward distance from the two wheels'
> encoder count deltas (`LEFT_COUNTS_PER_REV`=3286, `RIGHT_COUNTS_PER_REV`=3208, wheel
> diameter 2.78in — all bench-measured) averaged together, with direction from the IMU yaw
> (not from the encoders — the two wheels' counts-per-rev already differ by ~2%, not precise
> enough for differential heading on top of the IMU). It falls back to the old
> `K_V·(left_pwm+right_pwm)/2` PWM model (`ROBOT_MAX_SPEED_MPS`, still an uncalibrated guess)
> only on a tick where the encoder read-back fails. Real distance accuracy now depends on the
> bench measurements above and on the drivetrain not slipping, rather than on PWM calibration.
> Columns are optional: recordings made before this feature lack them and still replay
> (origin/0.0); recordings made before 2026-10-03 hold PWM-model positions, not encoder-based.

### Architecture notes
- Each capture tick builds a snapshot trio under `self._frame_lock`: `latest_frame` (CSV-shaped dict), `latest_frame_bgr` (BGR numpy image or None), and `latest_decoded` (parsed IMU tuples + audio floats for the widgets). All GUI monitoring widgets render from this snapshot in both Real and Sim modes.
- `IMUReader.drain_samples()` and `AudioReader.drain_samples()` atomically swap their internal buffers, so all readings since the last frame are captured as arrays (not just the latest snapshot).
- `RecordingManager` writer thread handles frame encoding (PIL BGR→RGB→640×640→JPEG→base64) and CSV writes off the main thread.
- Frame encoding uses PIL, not cv2, to stay compatible with NumPy 2.x.
- `csv.field_size_limit` is raised at module import — color base64 fields exceed the 128 KB default.
- `PositionEstimator` (module-level, near `IMUReader`) integrates X/Y each `_tick_real`; the DRIVE & CAM panel shows a top-down mini-map (`_position_map`/`_draw_pos_map`) and the status bar shows an `X/Y/HDG` readout + a `⌖ POS` reset button. In Sim mode the map/readout replay the recorded `pos_*` columns. `RecordingManager.CORE_FIELDNAMES` (the original 11 columns) is what `SimPlayer` requires, so adding columns never breaks playback of older CSVs.

## Sim mode

The **▶ SIM** button in the status bar toggles Real/Sim. Entering Sim opens a file dialog for a session CSV; `SimPlayer` then replays it row-by-row inside the same capture `_loop`, rebuilding the snapshot trio as if the data were sampled live. Details:
- **Playback is paced by the CSV's timestamps** (real-time replay), so it is correct regardless of the rate a given recording actually achieved. `SimPlayer.next_frame()` returns `SimPlayer.WAIT` while the current frame should be held.
- The joystick knob and pan/tilt sliders animate from the recorded values; user input to those controls is blocked during sim, and pre-sim pan/tilt is restored on exit (so servos don't jump when real sends resume).
- Works on any machine (macOS included) — only needs the `logots` conda env and a recording CSV. Hardware readers keep running but are ignored (stopping `IMUReader` would force a ~10 s recalibration per toggle); I2C sends and reconnects are skipped.
- **LOOP** checkbox: wrap at end-of-file vs freeze on last frame (`SIM ended`).
- REC is disabled during sim (auto-stopped when entering); malformed CSV rows are skipped and counted; canceling the file dialog stays in Real mode.
- Status labels (IMU/audio/video) show `SIM`; header shows `⬤ SIM MODE`; motor labels show the CSV's recorded values.

## Frame API (for downstream processing)

`FrameServer` inside the GUI serves `GET http://localhost:8787/latest_frame` (JSON) in both modes. `frame_data` is base64 color JPEG — in Real mode it's encoded lazily per request (cached by `frame_id`) so the capture tick never pays for it; the four array fields are real JSON arrays; `pos_x`/`pos_y` (m) and `heading` (°) carry the dead-reckoned body position; `sim_mode` (bool) is included. Client helper:

```python
from logots_api import get_latest_frame   # src/logots_api.py
frame = get_latest_frame()                # adds frame['image']: 640×640×3 uint8 RGB numpy
```

## Functions-layer actuation (executing Asaf's decisions on :8788)

Asaf's `src/functions.py` (see `docs/v2_architecture/functions_layer_report.md`) runs a
Gemma decision loop and publishes each chosen action via his `src/actions.py`
`ActionsEndpoint`, which serves `GET :8788/latest_action` and `POST :8788/action_done` — the
completion contract `functions.py`'s `wait_for_done()` actually blocks on (up to `DONE_WAIT_S`
= 120 s). `ActionReader(threading.Thread)` in `logots_ui.py` (added 2026-09-15, addressing
that report's asks #2/#3) closes the loop from this side:

- **The reader**: polls `GET :8788/latest_action` every 0.2 s, dedupes on `action_id`. When the
  action is `approach_plant`, it drives the motors exactly the way the joystick does — sets
  `gui.left_pwm`/`right_pwm` from the action's args (clamped to ±255) for `duration_s` seconds,
  then zeros them; `_send_motors()`'s normal per-tick I2C send does the rest. Blocked while
  `sim_mode` is on. Tolerates `:8788` not being up yet (functions.py not running) — just keeps
  polling silently.
- **Completion signal**: after driving, `ActionReader` reads the live pose off
  `gui._pos_est` (`.x`/`.y`/`.heading`) and does `POST :8788/action_done` with
  `{"action_id", "pos_x", "pos_y", "heading"}` — exactly the shape `actions.py`'s
  `ActionsEndpoint.do_POST` expects, landing in its `_done` dict that
  `endpoint.completion(action_id)` reads. This is the real contract (checked directly against
  `actions.py` before wiring it up) — not the `:8787`-frame-field idea floated earlier, which
  doesn't satisfy what `functions.py` actually waits on.
- **Port note**: `ActionReader` is a client only (`urllib.request` against `:8788`) — it doesn't
  bind a port itself, so `functions.py`'s own `ActionsEndpoint` (which does bind `:8788`) needs
  that port free, which is why the voice assistant's action server is off by default now (see
  "Voice assistant" below).
- **Not yet done**: only `approach_plant` is actuated; other actions (`inspect_plant`, `finish`,
  `speak`, …) are Asaf's-side or no-ops here by design per the report. Concurrent manual
  joystick/keyboard driving while `ActionReader` is also driving is unguarded (last write wins)
  — fine for now since the two aren't expected to run at once, revisit if that changes.

## Note for Asaph — 2026-10-03 changes relevant to the LLM/functions layer

Nothing in the `:8787`/`:8788` API *shapes* changed — `pos_x`/`pos_y`/`heading` are still the
same fields, same units. What changed is how trustworthy they are, plus two numbers worth
baking into `approach_plant` reasoning if you ever have the LLM pick `left_pwm`/`right_pwm`/
`duration_s` to hit a target distance:

- **`pos_x`/`pos_y` are now real metric distance, not a rough guess.** Previously
  `PositionEstimator` modelled forward speed from commanded PWM only (`ROBOT_MAX_SPEED_MPS` was
  an uncalibrated placeholder) — explicitly *not* something to trust for a real distance
  threshold. As of today it integrates actual wheel-encoder counts (bench-measured
  counts/rev + wheel diameter), and was live-validated against a physical tape measurement:
  **~2% accuracy** on a ~1m drive. If `inspect_plant`'s data-side reasoning (or any future
  action) wants to use "how far have I moved since my last observation" as a real signal —
  e.g. "I drove 0.4m and the plant still isn't centered, something's wrong" — that number is
  now actually meaningful, where before it wasn't.
- **Reference numbers for duration_s math**, from live floor tests (not the no-load bench
  numbers — those overstate real-world speed): at **PWM 150, forward speed ≈ 0.16-0.17 m/s**
  under real floor load. **Minimum PWM to move at all is ≈65** — anything below that and the
  wheels don't turn (stiction), so a chosen `left_pwm`/`right_pwm` below ~65 will silently do
  nothing for the commanded `duration_s`. If you ever want the LLM (or a wrapper around it) to
  convert "move roughly N metres" into a PWM+duration pair, `duration_s ≈ N / 0.165` at
  PWM≈150 is a reasonable starting point — not precise (floor friction varies), but much
  better than guessing.
- **Equal `left_pwm`=`right_pwm` now actually drives straight**, or close to it. Found today
  that even matched PWM was curving 30-40° over a ~6s drive under real floor load (a real
  motor/floor asymmetry, not a decoding issue) — added a gyro-assisted correction
  (`_straight_trim()` in `logots_ui.py`, transparent to any caller) that cuts that to roughly
  1-1.4°/s residual. `knowledge/actions.md`'s `approach_plant` doc already says "equal values =
  straight" — that description is now much closer to physically true than it was before today.
- **No action needed on your side unless you want to use the above** — this is a
  heads-up, not a breaking change. The `:8787` frame API and `:8788` action contract are
  unchanged; existing `functions.py`/`mrt_reflective.py` code keeps working exactly as before.
- Unrelated to today, still open and still yours: **Open bug #2** (`inspect_plant` decoding
  loop, see "Known issues" below) — not touched this session.

## Voice assistant ("Hey Jarvis" → local LLM → spoken action)

`src/audio_on_demand.py` (wake word → `Ears` → `LlamaCppBrain` → action → `speak`) now runs
**in-process inside `logots_ui.py`** via a `VoiceAssistant(threading.Thread)`, started
automatically on GUI launch — one command line (`python src/logots_ui.py`), no separate
terminal, no separate conda env. It reuses `audio_on_demand.py`'s classes unmodified (`Ears`,
`LlamaCppBrain`, `ActionServer`, `speak`, `api_chunks`) and pulls audio the same way any
external client would — polling this GUI's own frame API on `:8787` — so the module stays a
correct standalone script too (`python src/audio_on_demand.py --brain llamacpp`) if ever needed.

- **Status in the GUI**: header `⬤ LLM:` (loading/ready/error) and `⬤ MIC:`
  (asleep/listening/thinking/speaking), plus an `ACTION` row in the AUDIO INPUT panel showing
  the last decided action (e.g. `water_plant(id=ficus, ml=250)`). All driven by
  `VoiceAssistant.stage`/`.last_action`, polled each GUI tick in `_update_voice_widgets()`.
- **Latency (2026-08-16 fix, ~20-30s → ~1-2s/utterance)**: `LlamaCppBrain` used to spawn a
  fresh `llama-mtmd-cli` process per utterance (full model reload every time) and was still
  forced CPU-only. It now runs a **persistent `llama-server`** (spawned once, GPU-offloaded,
  health-checked via `/health`) and `decide()` is just an HTTP call. A second fix was needed on
  top of that: this GGUF's chain-of-thought reasoning trace added ~300 tokens (~10s) per call
  for no accuracy benefit on this schema-constrained task — `--reasoning off` on the server
  cut that to the final ~1-2s. This also closes out the old CPU-only/GPU-OOM workaround
  documented below.
- **TTS**: local **Piper** (`pip install piper-tts`; voice model `en_US-lessac-medium` at
  `~/models/piper/`, ~60 MB, one-time download from `rhasspy/piper-voices` on Hugging Face).
  Chosen over espeak-ng (its CLI isn't installed, needs `sudo apt install`) — Piper is pure
  pip + `onnxruntime`, no sudo, prebuilt aarch64 wheels.
- **Speaker output device — do not use `sd.play(data, fs)` with no device on this Jetson
  under NoMachine.** A NoMachine remote-desktop session runs its own private PulseAudio server
  (`PULSE_SERVER` env var points at a `~/.nx/devices/.../audio/native.socket`) that silently
  hijacks ALSA's `default` device, redirecting playback to the **client** machine's speakers
  (e.g. a connected MacBook) instead of the robot's. Fix: `speak()` targets the named ALSA PCM
  `"demixer"` explicitly (`SPEAKER_DEVICE` in `audio_on_demand.py`) — defined in
  `/etc/asound.conf` as `plug` (auto rate-convert) + `dmix` (software mixing) over the real
  `hw:APE,0` hardware, bypassing Pulse entirely. The raw `hw:1,0` device works too but has *no*
  rate conversion (Piper's 22050 Hz output plays back sped-up/high-pitched on the hardware's
  fixed 48000 Hz rate) — always go through `"demixer"`, never the bare `hw:` device, for
  playback. If testing playback manually outside the GUI, `unset PULSE_SERVER` first or you'll
  hear it on the wrong machine and wonder why.
- **Actual hardware fault found this session**: after ruling out software/OS causes (code,
  gain, ALSA routing, kernel driver errors — even a *pure* `speaker-test` sine tone was
  distorted), it turned out to be a **loose physical wire** on the amp side. If speaker output
  is ever loud/garbled/unintelligible again and doesn't respond to digital volume changes,
  check the physical DIN/DOUT wiring (pin 38 → mic SD, pin 40 → amp DIN — these are the only
  wires that differ between mic and amp; 12/35 are correctly shared) before assuming it's a
  driver/config regression.
- **openwakeword needs the ONNX backend, not its default TFLite one**: `tflite-runtime` is
  compiled against NumPy 1.x and crashes (`_ARRAY_API not found`) under NumPy 2.x — breaks in
  the `logots` env (NumPy 2.x) but not `logots-audio` (NumPy 1.x, used during earlier
  standalone testing). Fixed by forcing `Ears`'s `Model(..., inference_framework="onnx")` in
  `audio_on_demand.py` — works under both NumPy versions, and is also required for macOS (see
  below), so this is the permanent setting, not a Jetson-only patch. Also: openwakeword's model
  weights (`.onnx`/`.tflite` files) live inside each conda env's own `site-packages/openwakeword/`
  and don't carry over between envs — run `python -c "from openwakeword.utils import
  download_models; download_models()"` once per new env that needs it (the pip package alone
  doesn't include them).
- **`environment.yml`** now includes `soundfile`, `openwakeword`, `piper-tts` (added
  2026-08-16) alongside the existing `sounddevice`. The Jetson-specific piece —
  `~/llama.cpp/build/bin/llama-server` plus the downloaded GGUF/mmproj/Piper files under
  `~/models/` — is **not** part of the repo or `environment.yml`; see "Can Asaph run this on
  his Mac?" below for what that means off-robot.
- **Brain auto-selection (2026-08-16)**: `VoiceAssistant` picks `LlamaCppBrain` if
  `~/llama.cpp/build/bin/llama-server` exists on disk, else falls back to `GemmaBrain`
  (transformers, MPS/CUDA/CPU) — so the same GUI code works on the Jetson (fast path) and a
  MacBook (Mac-native path) with zero config. Override with `VOICE_BRAIN=llamacpp|gemma` env
  var if needed. `GemmaBrain.ensure_ready()` forces its lazy model load at startup (mirrors
  `LlamaCppBrain._ensure_server()`) so a missing `torch`/`transformers` install surfaces
  immediately as `⬤ LLM: error`, not silently on the first utterance.
- **`SPEAKER_DEVICE` is now platform-aware**: defaults to `"demixer"` only on Linux; elsewhere
  (macOS) it's `None`, meaning "use the system's default output device" — `sounddevice`'s
  normal behavior, so Piper's audio plays out the Mac's actual speakers instead of erroring on
  a nonexistent ALSA device name.

### Can Asaph run this on his Mac?
**Sim mode: yes, unaffected, always has been.** `VoiceAssistant` degrades gracefully on any
startup failure (missing binary, missing deps) — caught, sets `⬤ LLM:`/`⬤ MIC:` to `error`,
thread exits cleanly, nothing else in the GUI (Sim mode, sensors, frame API) is affected.

**The real voice feature (live mic → LLM → spoken reply), with his own Mac hardware: also yes,
as of the 2026-08-16 brain-auto-selection fix — with one manual step.** `GemmaBrain` needs
`torch` + `transformers`, deliberately **not** added to the shared `environment.yml` (adding
them would force a heavy, Jetson-risky install — generic PyPI `torch` doesn't have Jetson/CUDA
support, unlike NVIDIA's special Jetson wheels, and the Jetson doesn't need `GemmaBrain` at all
since `LlamaCppBrain` already covers it there). So on his Mac, after the normal
`conda env create -f environment.yml`, he additionally needs:
```bash
pip install torch transformers
```
in the `logots` env — after that, launching `python src/logots_ui.py` in Real mode (not Sim)
will auto-select `GemmaBrain`, capture from his MacBook's real mic (`AudioReader` uses
`sounddevice`'s system default, no Jetson-specific code there), decide actions with Gemma 4 on
MPS, and speak `speak` actions out his Mac's real speakers (`SPEAKER_DEVICE` fallback above).
Without that `pip install` step, he'll just see `⬤ LLM: error` with a clear message naming the
missing dependency — harmless, same graceful-degradation path as before.
- `openwakeword`/`piper-tts`/`sounddevice`/`soundfile` all install fine on macOS (piper-tts
  ships `macosx_11_0_arm64` wheels; openwakeword's `tflite-runtime` dep is Linux-only per its
  own package metadata, so macOS falls back to the onnx backend anyway — same one we now force
  everywhere, so no behavior difference to fix later).
- This is the same `GemmaBrain` already documented in `docs/v2_architecture/action_api.md`
  (~2-4s/utterance warm) — nothing new about the model itself, just that `VoiceAssistant` can
  now reach it automatically instead of him needing a separate script/env.

**Startup steps for Asaph, first run after this update:**
```bash
cd /Users/orberebi/Documents/GitHub/Logots_V2
git pull origin main
conda env update -n logots -f environment.yml --prune   # picks up openwakeword/piper-tts/soundfile
conda run -n logots python src/logots_ui.py
```
- **Sim mode**: click **▶ SIM**, pick any session CSV — unaffected by any of today's changes,
  works with the steps above alone.
- **Real voice feature (his own Mac mic/speakers), optional**: additionally run
  `pip install torch transformers` in the `logots` env first, per "Can Asaph run this on his
  Mac?" above, then launch in Real mode (not Sim). Without that extra install, the header just
  shows `⬤ LLM: error` — harmless, not a bug, and everything else in the GUI still works fine.

## Git setup
- Remote: `https://github.com/OrBerebi/Logots_V2.git`
- Credentials stored in `~/.git-credentials` via `git credential.helper store`
- Git identity: `Or Berebi <or.berebi1@gmail.com>`
- To push: `git -C /home/logots/Desktop/Logots_V2 push origin main`
- Arduino IDE 2.x has no Linux ARM64 build — flash firmware from MacBook only

## System tweaks (already applied)
- gnome-terminal copy/paste remapped to `Ctrl+C` / `Ctrl+V` via gsettings
- systemd `DefaultTimeoutStopSec=5s` for fast headless shutdown

## Critical rules
1. **Never manually edit `/boot/extlinux/extlinux.conf`** — always use `jetson-io.py`. Manual edits brick the boot.
   **After any `nvidia-l4t-*` package upgrade, re-check it**: an `apt dist-upgrade` across L4T versions can
   silently reset `DEFAULT` back to `primary` (losing the custom `JetsonIO` boot entry → camera stops working,
   `/dev/video0` disappears) even though the actual overlay files in `/boot` are untouched. Fix by rerunning
   `jetson-io.py` → "Configure for compatible hardware" and reselecting the camera module — this rewrites
   `extlinux.conf`'s `DEFAULT` correctly without hand-editing it. Confirmed happening on the 2026-08-02
   r36.4.7→r36.5 upgrade (see Known issues).
2. **Camera always needs `EGL_PLATFORM=surfaceless`** — DISPLAY=:0 and DISPLAY=:1001.0 both fail for nvarguscamerasrc.
3. **Don't use system cv2 from conda** — it's compiled for NumPy 1.x and will crash with conda's NumPy 2.x.
4. **Arduino I2C is always bus 1** — confirmed with `i2cdetect -y -r 1`, shows 0x08.
5. **Always work in the git repo** — `/home/logots/Desktop/Logots_V2/`. The old `logots/` directory is archived.
6. **Never play audio via ALSA `default`/no-device under a NoMachine session** — NoMachine runs
   its own PulseAudio server per session and silently redirects `default` playback to the
   *client* machine's speakers, not the robot's. Always target the named ALSA PCM `"demixer"`
   explicitly (see "Voice assistant" below) and `unset PULSE_SERVER` before any manual ALSA
   testing (`speaker-test`, `aplay`, etc.) from a NoMachine terminal.

## Known issues / next steps
- **`action_id` collision across separate `functions.py` runs skipped actuation silently —
  FIXED 2026-09-15** (found testing Asaf's `39038cf` "goal-oriented prompt chaining" patch with
  Or, same session): `ActionsEndpoint._next_id` in `src/actions.py` started at `1` fresh every
  time `functions.py` was launched as a new process, but `ActionReader` (in `logots_ui.py`)
  lives inside the long-running GUI process and keeps its `_seen_id` dedup state across every
  `functions.py` invocation. Reproduced live: first `functions.py` run's `approach_plant` got
  `action_id=1` and drove fine; a second `functions.py` run (same GUI still up) also started its
  own counter at `1` for its first actuator action — `ActionReader` saw `action_id=1` again,
  treated it as already-handled, and silently skipped driving. `wait_for_done()` then correctly
  waited the full `DONE_WAIT_S=120s`, got nothing, and (per the same patch's new honest-completion
  behavior) told the LLM the truth ("no completion signal; position unchanged") instead of
  hanging or faking it — so the *symptom* looked like the script being stuck for 2 minutes, but
  the loop itself was working correctly; the actuation call underneath it was just silently
  dropped. Not something Asaf's patch introduced — it was a gap in the actuation wiring from the
  earlier `51f17fb` commit. **Fix**: `ActionsEndpoint._next_id` (src/actions.py) is now seeded
  from `int(time.time() * 1000)` instead of `1`, so ids stay unique across process restarts while
  still monotonically increasing within one run. Verified live immediately after the fix: two
  back-to-back `functions.py` runs against the same still-running GUI (no restart in between)
  both drove `approach_plant` cleanly, no timeout.
- **`functions.py --speak` added 2026-09-15**: each step's `"thought"` is now spoken aloud
  through the robot's speaker via the same Piper/`"demixer"` TTS path as the voice assistant
  (`audio_on_demand.speak()`, imported directly — self-contained, doesn't need `logots_ui.py`'s
  `VoiceAssistant` running). Opt-in flag, off by default so normal/scheduled runs don't pay the
  Piper load + playback latency per step. Verified live: audible and clear on the robot's
  speaker during a full initiation run.
- **Functions-layer actuation (2026-09-15) — live-tested against the real robot today, mixed
  results. Confirmed working, plus two open bugs — one for Or/hardware, one for Asaf/decoding:**

  **Confirmed working end-to-end:** `ActionReader` in `logots_ui.py` polls Asaf's
  `:8788/latest_action`, drives `approach_plant` on the real motors, and reports completion via
  `POST :8788/action_done` — the real contract implemented in his `src/actions.py`
  (`ActionsEndpoint`/`wait_for_done()`), not the `:8787`-frame-field idea floated first (checked
  against `actions.py` and corrected before testing). One full live run drove the robot, posted
  completion, and `functions.py` received and logged it correctly. Also fixed today:
  `LlamaCppBrain._ensure_server()` (`audio_on_demand.py`) now detects and reuses an
  already-running `llama-server` on `:8789` instead of trying to spawn a second GPU-loaded one
  when `functions.py`'s `LlamaCppVisionBrain` starts — needed since the GUI's embedded voice
  assistant already has one up.

  **Open bug #1 (hardware/Or's side) — motor direction, CLOSED 2026-10-03:** motors rewired to
  M1=right/M4=left; the right wheel came up reversed on the new wiring (bench W-test: left
  encoder climbed, right fell). Fixed at the hardware layer by swapping the two leads on the
  right motor's connector, not firmware or software — kept `+PWM = forward` true at every layer
  instead of adding an asymmetric correction in code (see the bench-calibration entry below for
  the full reasoning and the considered-then-reverted `_send_motors()` approach). Retested: both
  `EL`/`ER` climb positive on W. The old PWM-100-turns-right symptom and the original
  `_send_motors()` global sign flip (from the pre-rewiring single-M1/M4 era) are both
  historical at this point — see the "Stiff right motor" entry below for the follow-up bench
  test that found the turning behavior was near-PWM-threshold stiction, not a hardware
  asymmetry at running speed.

  **Open bug #2 (decoding — for Asaf) — the decision loop gets stuck on `inspect_plant`,
  reproduced 6/6 times today:** across substantially different conditions — robot far from the
  plant, robot moved close to the plant, `knowledge/actions.md`'s `approach_plant` wording
  varied and then reverted to original, GUI freshly relaunched, and even **sim mode** (where
  each `inspect_plant` call gets a genuinely different replayed camera frame) — `functions.py`
  reliably calls `inspect_plant` for all 12 steps of its budget and never calls `approach_plant`
  or `finish`. Since the image content clearly varies across these runs yet the decision doesn't,
  this looks like a decoding issue rather than a vision/scene issue. Leading theory: both the
  `llama-server` launch (`--temp 0` in `audio_on_demand.py`) and the per-request payload
  (`"temperature": 0` in `mrt_reflective.py`'s `ask_json()`) use fully greedy/zero-temperature
  decoding, deliberately, per an existing comment ("for reproducibility") — small models under
  greedy decoding are known to fall into repetition loops once a pattern (2+ identical
  `inspect_plant` steps) appears in their own growing prompt history. The one run that *did*
  call `approach_plant` did so at step 3, before such a pattern had "set in" — consistent with
  this theory but not proof. NOT changed — `temp=0` was a deliberate design choice and touches
  shared decoding config also used by the voice assistant, so this needs Asaf's call, not a
  unilateral edit from this side. Options worth considering: a small non-zero temperature or a
  repeat-penalty on `llama-server`; a much larger `--max-steps` to check whether it's a
  slow-breaking loop rather than a permanent one; or an explicit prompt-side rule (in
  `knowledge/guidelines.md` or the `STEP_PROMPT` in `functions.py`) against repeating
  `inspect_plant` without new information.

  **To reproduce today's testing:** GUI running, then
  `conda run -n logots python src/functions.py --knowledge-dir <fresh-empty-dir>` (a
  non-empty/default `knowledge/` dir with plants already on the roster makes it skip
  immediately — see `src/knowledge.py`'s `roster_empty()`). Decision log lands at
  `runs/initiation_<timestamp>.log` (gitignored).

  `knowledge/actions.md` is back at its original content (gitignored, untracked — not part of
  this commit). `git status` shows `src/logots_ui.py`, `src/audio_on_demand.py`, `CLAUDE.md`
  modified — reviewed and committed today.
- **Pan/tilt servo random twitch fixed (2026-09-03, commit `c36d94e`)**: servos made small,
  seemingly random jumps every ~0.5s even with unchanged target angles. Root cause was in
  `src/firmware/logots_motor_control/logots_motor_control.ino` — the I2C `onReceive` ISR
  (`receiveEvent()`) ran `sscanf` and several `Serial.print()` calls directly inside the
  interrupt. AVR interrupts don't nest by default, so this held Timer1's compare-match
  interrupt — which the `Servo` library depends on to end each pulse at the correct
  microsecond — blocked long enough to occasionally stretch a pulse, showing up as a twitch.
  Fixed by making the ISR only buffer bytes and set a flag; `parseMessage()` (the
  sscanf/Serial.print work) now runs from `loop()` instead, outside interrupt context. If
  servo/motor jitter reappears, check first whether new code was added *inside* an ISR —
  anything slow in an AVR ISR reintroduces this exact class of bug. Requires reflashing from
  the MacBook (Arduino IDE) to take effect; user confirmed fixed after flashing.
- **Voice assistant done (2026-08-16), diff not yet committed**: wake word → local LLM →
  spoken action, fully wired into `logots_ui.py` as a background thread — see "Voice assistant"
  section above for the full write-up (latency fix, TTS, speaker-routing gotcha, the physical
  wiring fault found, openwakeword's ONNX-backend requirement, Mac/Sim-mode compatibility).
  `git status` currently shows `src/audio_on_demand.py`, `src/logots_ui.py`, `environment.yml`
  modified and `PLAN_llamacpp_gpu_offload.md` untracked — review and commit when ready.
  `PLAN_llamacpp_gpu_offload.md` (repo root) has the detailed before/after latency numbers.
- **Post-upgrade regressions found and fixed (2026-08-02):** the r36.4.7→r36.5 upgrade reset the boot
  loader's `DEFAULT` to `primary` (see Critical rules #1) — broke the camera, fixed via `jetson-io.py`.
  Separately, the mic/speaker went silent (`sd.InputStream` reads exact `0.0` even with real input) —
  **this one is unrelated to the OS upgrade**: comparing `jetson-io.py`'s live pin config against the
  reference screenshot at `/home/logots/Desktop/logots/header_pinouts.png` (May 28) showed pins 12/35/38/40
  (`i2s2_sclk/fs/din/dout`) as `unused`, while the currently-active custom overlay
  (`/boot/jetson-io-hdr40-user-custom.dtbo`) is dated Jun 4 — one day *after* the last known-good mic
  recording (`logots/logots_unified_test.wav`, Jun 3). Best guess: a manual pin edit on Jun 4 (likely when
  adding the Arduino `i2c2` pins) wasn't incremental and dropped the `i2s2` group. Fixed by re-adding
  `i2s2` via `jetson-io.py` → "Configure header pins manually"; user confirmed all sensors working after.
  If audio drops out again, check that live config against `header_pinouts.png` first.
- Camera has pink/IR hue — missing IR cut filter on IMX219-160 fisheye. Need M12 IR cut filter hardware.
- Robot is assembled: motors and servos are physically connected to the Arduino and the I2C command flow drives them. **2026-10-03**: the drive motors' built-in JGA25-370 encoders are now wired up, and the motors were rewired to M1=right, M4=left. The right motor is noticeably stiffer to turn by hand, a likely cause of Open bug #1's drift to the right. The Arduino is now powered from the 12V battery through a 9V buck converter, and the shield takes 12V on EXT_PWR (see `src/pinout.txt` §6; the shield's PWR jumper must be off). Encoder counts are read back to the GUI (see "Arduino firmware protocol"). **Task 2 done 2026-10-03**: `PositionEstimator` now uses them in place of the PWM speed model (see "Recording feature" position note above).
- **Bench calibration (2026-10-03), done on the Jetson**: the firmware from commit `fd843bb` is
  flashed, calibration block still at its defaults (`LEFT_MOTOR_DIR=+1`, `RIGHT_MOTOR_DIR=+1`,
  `PAN_DIR=+1`, `PAN_CENTER_DEG=90`) — no reflash was needed:
  - **encoder signs: DONE.** `LEFT_ENC_DIR = +1`, `RIGHT_ENC_DIR = -1` (mirror-mounted motor).
    Confirmed after reflash: both wheels count up when turned forward by hand (Serial Monitor).
  - **motor directions: DONE, fixed in hardware not firmware.** Wheels-raised W test showed left
    climbing (+9523) and right falling (−7419) — right wheel reversed. Rather than flip
    `RIGHT_MOTOR_DIR` (which would've meant a Mac reflash), the two leads on the right motor's
    connector were swapped by hand. Retested: both `EL`/`ER` climb positive on W. Firmware
    `*_MOTOR_DIR` constants and `_send_motors()` are both untouched/symmetric — considered,
    then reverted, a software-side flip in `_send_motors()` in favor of this hardware fix, since
    it keeps `+PWM = forward` true at every layer instead of carrying an asymmetric correction
    in code. Note the encoder's `RIGHT_ENC_DIR=-1` mirror-mount correction is unaffected — it's
    on the separate Hall-sensor signal path, not the motor power leads.
  - **pan: DONE, no change needed.** GUI slider 0 = front, +45 = left — confirmed correct
    against `PAN_DIR=+1`/`PAN_CENTER_DEG=90` defaults.
  - **Coast-down asymmetry observed (expected, not a bug)**: releasing W, the left wheel coasts
    briefly before stopping while the right stops dead. Firmware's `controlMotor()` calls
    identical `RELEASE` on both motors (L293D has no active brake, just floats the terminals) —
    confirmed symmetric in code. The difference is mechanical: the stiff right motor (below)
    has enough friction to stop immediately; the looser left motor coasts on momentum.
- **Pan GUI-limited to ±45° (2026-10-03, `src/logots_ui.py` `_cam_sliders()`)**: slider range
  narrowed from the firmware's full ±90° to ±45° for now (labels `L 45°`/`R 45°`). Firmware
  `PAN_DIR`/`PAN_CENTER_DEG` still accept the full range — this is a GUI-only cap, no
  calibration-block change, revert by widening the slider's `top`/`bot` back to 90.
- **Counts per wheel revolution: MEASURED 2026-10-03** (hand-turn test, 5 full turns each,
  wheels on the ground): left 27907→44337 (Δ16430 / 5 = **3286 counts/rev**), right
  21805→37847 (Δ16042 / 5 = **3208 counts/rev**) — the two wheels agree within ~2.4%, consistent
  with the earlier half-turn tests' "low thousands" estimate. `encLeft`/`encRight` are
  `volatile int32_t` in RAM (no EEPROM) — they reset on any Arduino power-cycle/reset/reflash,
  *not* on GUI reconnect or Sim-mode toggling; at ~3.2k counts/rev, `int32_t` wraps only after
  roughly 670,000 wheel revolutions, not a practical concern. **Task 2 done 2026-10-03**: fed
  into `PositionEstimator` (with wheel diameter 2.78in, bench-measured) in place of the PWM
  speed model — see "Recording feature" position note and `src/logots_ui.py`'s
  `PositionEstimator`.
- **Encoder-based `PositionEstimator` validated live on the floor (2026-10-03)**: drove the
  robot via the real `ActionReader`/`:8788` `approach_plant` path (not a side-channel — the
  same contract `functions.py` uses) for two ~1m straight-PWM (150/150) runs. Model's computed
  straight-line start→end displacement vs. a physical tape measurement: run 1 wasn't measured
  (no start mark); run 2 — model said **108.3 cm**, measured (average of the two wheels'
  tracks, left 101cm/right 111cm, since the path curved — see below) **106 cm** — **~2%
  high**, a good match given the hand-measured wheel diameter. No constant changes made; this
  is validation, not recalibration.
  - **New finding — floor-load curving, not seen on the raised-wheel bench test**: both runs,
    at equal L=R=150 PWM the whole time, curved substantially instead of driving straight
    (heading drifted ~33-40° over ~6s). The bench test (wheels raised, no load) found the two
    motors roughly speed-matched (ratio 0.958) — this says there's a real asymmetry that only
    shows up under the wheels' actual floor load (weight distribution, tire contact, or the
    stiff right motor behaving differently loaded vs. free-spinning). The position *model*
    handled this correctly regardless (it integrates heading every tick, so a curved path is
    still tracked accurately — confirmed by the measurement match above) but the robot itself
    doesn't drive straight on an equal-PWM command. Worth a future closed-loop straight-drive
    fix (e.g. trim one side's PWM, or steer off the gyro) if driving straight matters for a
    task — not done here, out of scope for this session.
- **Gyro-assisted straight-line trim, added 2026-10-03** (`src/logots_ui.py`,
  `_straight_trim()`/`STRAIGHT_KP`/`STRAIGHT_KI`/`STRAIGHT_I_MAX`/`STRAIGHT_MAX_TRIM`): fixes
  the curving above in software rather than chasing the mechanical cause. Called from
  `_send_motors()` — the single choke point every drive command passes through (joystick,
  keyboard, and `ActionReader`'s autonomous `approach_plant`) — so it applies automatically
  everywhere, with no caller changes. Whenever the commanded PWM is equal and nonzero (a
  "drive straight" command), it locks the IMU yaw at that moment and each tick applies a PI
  correction (P on the current heading error, I on the error accumulated over time, to kill
  the P-only controller's steady-state residual) by trimming one wheel down and the other up
  — **only on the bytes actually sent over I2C**; `self.left_pwm`/`right_pwm` (what
  `PositionEstimator`'s PWM fallback, the CSV, the frame API and `ActionReader` all see) stay
  exactly as commanded. Any turn (unequal PWM) or stop passes through untouched and drops the
  lock, so a fresh straight segment always starts clean.
  - **Live-tuned on the floor, 2026-10-03**: uncorrected drift was ~5.6°/s (see the curving
    finding above). First sign guess for which wheel to trim was backwards (positive feedback
    — a 4s drive swung heading +83° instead of holding it); flipped, `STRAIGHT_KP=3.0`
    (P-only) then held drift to ~1.4°/s, reproduced twice (+5.6°, +5.45° over separate 4s
    runs). Tried doubling to `KP=6.0` to tighten further — went unstable instead (+133° in 4s,
    worse than uncorrected) — reverted. Added a small integral term instead
    (`STRAIGHT_KI=0.4`, anti-windup clamped at `STRAIGHT_I_MAX=40`) without touching `KP`,
    which got a 4s run down to +3.87° without instability; the integral needs more time to
    fully wind up, so longer drives should tighten it further. Current settings are a
    deliberate compromise (user confirmed "good as it is") — **don't raise `STRAIGHT_KP`
    without adding tick-by-tick error/trim logging first**, since the instability may be
    partly the IMU being noisier while actually driving (motor electrical
    noise/vibration) than at rest — confirmed rock-solid stationary (0.04° drift over 7s) —
    which a higher P gain would amplify.
  - This does not make driving perfectly straight (small residual lean remains, visually
    confirmed — "right wheel slightly in front of the left") but is a 4-5x+ improvement over
    uncorrected, and works transparently for autonomous `approach_plant` calls exactly as it
    does for manual joystick/keyboard driving.
- **Encoder-based `PositionEstimator` + straight-line trim validated together on real
  autonomous drives, 2026-10-03**: drove the robot via the actual `ActionReader`/`:8788`
  `approach_plant` contract (not a side-channel), with camera frames pulled before each drive
  purely as a safety check (never for distance measurement — physical tape/tile measurements
  were used for that, per the user's instruction). One ~1m run: model computed 108.3cm
  straight-line displacement; physical measurement (average of the two wheels' tracks, which
  differed — 101cm/111cm — since the path curved) was 106cm, a **~2% match**. Confirms the
  bench-measured wheel diameter/counts-per-rev are accurate enough for this model, and that
  the position math correctly integrates a curved (not just straight) path.
- **GUI FPS regression found and fixed, 2026-10-03** — the position map redesign (above)
  initially dropped the live FPS readout from its normal ~10/10 to ~5/10, then ~7.8/10 after a
  partial fix, confirmed fully fixed (~9.7/10) after the real cause was found. In order:
  1. `IMUReader`'s sampling loop (`src/logots_ui.py`) had **no rate limit at all** — no
     `time.sleep()` anywhere in its `while` loop — so it span as fast as the I2C bus allowed
     (likely 1000s of Hz), continuously pegging a CPU core. Capped to ~200Hz
     (`time.sleep(0.005)`), still far faster than the Madgwick filter or the 10Hz capture tick
     need. This alone only took FPS from ~5.3 to ~7.8 — a real fix (this loop looks
     pre-existing, not new today) but not the dominant cause of today's specific regression.
  2. **The actual dominant cause**: `_draw_pos_map()` called `cv.delete('all')` then rebuilt
     every grid line/label/trail/arrow from scratch each call — and this runs inside `_loop()`,
     the same function the FPS readout times. Benchmarked in isolation: `canvas.create_text()`
     specifically cost ~4ms *per call* on this system (not line or polyline items — those were
     cheap, ~0.3-2ms for the same count) — 16 grid labels recreated every tick cost ~65ms,
     nearly the entire 100ms/tick budget, on its own explaining almost all of the FPS drop.
     **Fixed** by rewriting `_position_map()`/`_draw_pos_map()` to pre-create a fixed pool of
     canvas items once (10 gridlines + 10 labels per axis, trail, arrow, dot, origin marker)
     and update them in place each redraw via `coords()`/`itemconfig()` (toggling `state`
     'normal'/'hidden' for the currently-unused pool slots) instead of delete+recreate —
     benchmarked at ~0.3ms for the same 16-label update, a ~200x improvement. No functional
     change to what's drawn, confirmed via screenshot (grid, trail, and arrow all render
     identically to the delete/recreate version, just far cheaper).
  - **Lesson if this class of bug shows up again**: on this system, `tkinter.Canvas`
    `create_*` calls (especially `create_text`) are fine for one-time setup but far too slow
    to call repeatedly inside a hot per-tick loop — always pre-create canvas items once and
    update them via `coords()`/`itemconfig()`/`state` instead.
- **Position map layout bugs found while diagnosing the above, both fixed, 2026-10-03**: the
  200px map (enlarged from 118px — see the map redesign entry above) was being **clipped**, not
  failing to draw — found via `gnome-screenshot` on the Jetson's `:1001.0` display (raising the
  right window first with a small `python-xlib` snippet, since multiple windows can occupy that
  display under NoMachine). `p_ctrl` (the DRIVE & CAM panel, `_main_panels()`) has a
  hard-coded `width=LW,height=TH` with `pack_propagate(False)`, so its content doesn't auto-grow
  the panel — it just silently overflows past the fixed boundary and gets cut off. The map's
  column (pan/tilt sliders + the enlarged map, beside the 244px-wide joystick) needed both more
  height and more width than the old `TH=310`/`LW=375` had. Grown to `TH=430`/`LW=480` (confirmed
  by screenshot: full grid -3..3 both axes, origin crosshair, and the heading arrow all visible).
  `p_audio` shares `LW` and `p_imu` shares `TH` for grid symmetry, so they're a little roomier
  now too — cosmetic only, no functional change there. Also enlarged the heading arrow itself
  (14px/width 2 → 24px/width 3, `arrowshape=(10,12,5)`) since it was hard to spot against the
  bigger canvas even once it was actually visible.
- **Stiff right motor — quantified and closed out (2026-10-03)**: ran a direct I2C bench script
  (bypassing the GUI, same protocol as `_send_motors()`/`_read_encoders()`) to ramp PWM per
  wheel and time encoder counts, wheels raised:
  - **Minimum starting PWM: 65 for both wheels** (right showed a little creep — 4-7 counts — at
    PWM 35-55 before properly turning at 65; left jumped cleanly straight to 65).
  - **Speed at PWM 150, both wheels, 2s**: left 2847 counts/s, right 2973 counts/s — ratio 0.958
    (right if anything slightly *faster*, roughly matched within ~4%).
  - **Conclusion**: no real sustained speed/torque asymmetry between the wheels at running
    PWM. The earlier PWM-100 right-turn drift and the coast-down asymmetry (left coasts, right
    stops dead) are both more likely a near-threshold stiction effect (PWM 100 is close to the
    65 minimum, where small per-wheel differences in exactly when static friction breaks can
    swing the heading) than a hardware defect. **Next**, now that direction is fixed in
    hardware too: retest straight-line driving at a higher PWM (200+) — if it still drifts
    there, look at alignment/weight distribution instead of the motors.
- Staging layer CSV + sim mode + frame API done (Asaph can develop off-robot against `logots_api.get_latest_frame()`); transformation + mart + decision layers not yet written.
- Color recording not yet exercised on the Jetson (grayscale→color change verified on Mac only) — record a short session next time on the robot and confirm the JPEGs are RGB.
- The capture loop is **drift-compensated**: it waits `PERIOD_MS` minus the time the tick's work took, so the actual rate tracks `TARGET_FPS` (default **10 Hz**, set in `logots_ui.py`) as long as the per-tick work fits inside the period. The header shows a live `FPS:actual/target` readout (green within 10% of target, amber below) — run on the Jetson and, if it can't hold green, set `TARGET_FPS` just under the sustained value. The camera runs at `CAMERA_FPS` (default 15, down from the sensor's 30) since the loop only keeps the latest frame; if `nvarguscamerasrc` rejects that framerate for its sensor mode, raise it or drop frames downstream with a `videorate` element. Recordings are timestamped, so sim playback is unaffected by the exact rate.
- **`ROBOT_MAX_SPEED_MPS` is now a fallback-only constant (2026-10-03)**: `PositionEstimator`'s primary model uses encoder counts for distance (see "Recording feature" position note), so this PWM-speed guess only matters on the rare tick where the encoder read-back fails. Calibrating it is no longer a blocker for `pos_x`/`pos_y` accuracy — if it's ever worth tightening, drive a known distance at full PWM for a known time and set the constant to `distance/time`. Position still assumes no wheel slip and drifts with IMU yaw drift over long sessions.
