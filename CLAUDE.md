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
  - left/right PWM: -255 to +255
  - pan/tilt angles: 0 to 180 degrees
- Motor driver: Adafruit Motor Shield (AFMotor.h), channels 3=left, 4=right
- Pan servo: Arduino pin 10. Tilt servo: Arduino pin 9. Both MG90S.
- Serial debug at 9600 baud: prints `OK  L=X R=X PAN=X TILT=X` per command
- Flash from MacBook with Arduino IDE (no Linux ARM64 build exists for IDE 2.x)

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
  L +000  R +000  PAN:090°  TLT:090°  X+0.00 Y+0.00  HDG:090°  ⌖ POS  LOOP  ▶ SIM  ⚫ REC  ■ STOP
```
- Position mini-map (bottom of DRIVE & CAM): top-down trail of the dead-reckoned body
  position with a heading arrow; `⌖ POS` in the status bar zeros the estimate (origin = here).
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
| `pan_angle` | int | pan servo, 0…180° |
| `tilt_angle` | int | tilt servo, 0…180° |
| `pos_x` | float | estimated body X position (m), origin = recording start; see position note |
| `pos_y` | float | estimated body Y position (m), origin = recording start |
| `heading` | float | body heading (°) used for the estimate = IMU yaw at that tick |

> **Position is a dead-reckoning estimate, not measured odometry.** There are no wheel
> encoders, so `PositionEstimator` models forward speed as `K_V·(left_pwm+right_pwm)/2`
> with direction from the IMU yaw, integrated per tick. Accuracy depends on the
> `ROBOT_MAX_SPEED_MPS` constant (currently a guess — calibrate on the robot). Columns are
> optional: recordings made before this feature lack them and still replay (origin/0.0).

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
- Robot is assembled: motors and servos are physically connected to the Arduino and the I2C command flow drives them. The drive motors are simple **non-feedback** motors (no encoders / no velocity readback) — a feedback-capable drivetrain is planned for the next body iteration.
- Staging layer CSV + sim mode + frame API done (Asaph can develop off-robot against `logots_api.get_latest_frame()`); transformation + mart + decision layers not yet written.
- Color recording not yet exercised on the Jetson (grayscale→color change verified on Mac only) — record a short session next time on the robot and confirm the JPEGs are RGB.
- The capture loop is **drift-compensated**: it waits `PERIOD_MS` minus the time the tick's work took, so the actual rate tracks `TARGET_FPS` (default **10 Hz**, set in `logots_ui.py`) as long as the per-tick work fits inside the period. The header shows a live `FPS:actual/target` readout (green within 10% of target, amber below) — run on the Jetson and, if it can't hold green, set `TARGET_FPS` just under the sustained value. The camera runs at `CAMERA_FPS` (default 15, down from the sensor's 30) since the loop only keeps the latest frame; if `nvarguscamerasrc` rejects that framerate for its sensor mode, raise it or drop frames downstream with a `videorate` element. Recordings are timestamped, so sim playback is unaffected by the exact rate.
- **Calibrate `ROBOT_MAX_SPEED_MPS`** (in `logots_ui.py`) on the robot (motors are now connected): drive a known distance at full PWM for a known time and set the constant to `distance/time`. Until then `pos_x`/`pos_y` are directionally right (heading is real IMU data) but not metrically accurate, and they assume motors track commands (no encoder feedback). Position also drifts with IMU yaw drift over long sessions.
