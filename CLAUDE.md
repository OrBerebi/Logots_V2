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
- **Hardware**: Jetson Orin Nano, JetPack 6 (L4T R36.4.7), ARM64
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
  ! video/x-raw(memory:NVMM),width=640,height=480,framerate=30/1
  ! nvvidconv
  ! video/x-raw,format=BGRx
  ! videoconvert
  ! video/x-raw,format=BGR
  ! filesink location=/tmp/logots_camera.fifo
```
Frames are read from the FIFO in `CameraReader` thread. PIL (not cv2) used for display.

## GUI layout
```
┌─────────────────────┬─────────────────────┐
│  DRIVE & CAM        │  IMU ORIENTATION    │
│  joystick + pan/    │  3D Madgwick AHRS   │
│  tilt sliders +     │  YPR display        │
│  position mini-map  │                     │
├─────────────────────┼─────────────────────┤
│  AUDIO INPUT        │  VIDEO FEED         │
│  waveform + RMS     │  live IMX219 feed   │
└─────────────────────┴─────────────────────┘
  L +000  R +000  PAN:090°  TLT:090°  X+0.00 Y+0.00  HDG:090°  ⌖ POS  LOOP  ▶ SIM  ⚫ REC  ■ STOP
```
- Position mini-map (bottom of DRIVE & CAM): top-down trail of the dead-reckoned body
  position with a heading arrow; `⌖ POS` in the status bar zeros the estimate (origin = here).
- Keyboard: W/S = forward/back, A/D = turn, SPACE = stop
- Arduino auto-connects on startup, retries every 3s if lost

## Recording feature

Press **⚫ REC** to start recording; press again to stop. Output:
```
recordings/session_YYYYMMDD_HHMMSS/session_YYYYMMDD_HHMMSS.csv
```

### CSV schema (staging layer — one row per 20 Hz tick)
| Column | Type | Description |
|---|---|---|
| `frame_id` | int | 0-based counter per session |
| `timestamp` | ISO 8601 | wall-clock time |
| `frame_data` | base64 str | **color** 640×640 JPEG, base64-encoded; `""` if no camera (sessions recorded before 2026-07-14 are grayscale — sim playback handles both) |
| `yaw` | JSON array | all IMU yaw readings (°) since last frame |
| `pitch` | JSON array | all IMU pitch readings (°) since last frame |
| `roll` | JSON array | all IMU roll readings (°) since last frame |
| `audio_samples` | JSON array | all mic samples since last frame (~800 floats at 20 FPS) |
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
- Each 20 Hz tick builds a snapshot trio under `self._frame_lock`: `latest_frame` (CSV-shaped dict), `latest_frame_bgr` (BGR numpy image or None), and `latest_decoded` (parsed IMU tuples + audio floats for the widgets). All GUI monitoring widgets render from this snapshot in both Real and Sim modes.
- `IMUReader.drain_samples()` and `AudioReader.drain_samples()` atomically swap their internal buffers, so all readings since the last frame are captured as arrays (not just the latest snapshot).
- `RecordingManager` writer thread handles frame encoding (PIL BGR→RGB→640×640→JPEG→base64) and CSV writes off the main thread.
- Frame encoding uses PIL, not cv2, to stay compatible with NumPy 2.x.
- `csv.field_size_limit` is raised at module import — color base64 fields exceed the 128 KB default.
- `PositionEstimator` (module-level, near `IMUReader`) integrates X/Y each `_tick_real`; the DRIVE & CAM panel shows a top-down mini-map (`_position_map`/`_draw_pos_map`) and the status bar shows an `X/Y/HDG` readout + a `⌖ POS` reset button. In Sim mode the map/readout replay the recorded `pos_*` columns. `RecordingManager.CORE_FIELDNAMES` (the original 11 columns) is what `SimPlayer` requires, so adding columns never breaks playback of older CSVs.

## Sim mode

The **▶ SIM** button in the status bar toggles Real/Sim. Entering Sim opens a file dialog for a session CSV; `SimPlayer` then replays it row-by-row inside the same 20 Hz `_loop`, rebuilding the snapshot trio as if the data were sampled live. Details:
- **Playback is paced by the CSV's timestamps** (real-time replay). The recorder's actual tick rate is ~10.5 Hz on the Jetson (tick work + `after(50)`), so fixed-rate replay would run ~1.9× fast. `SimPlayer.next_frame()` returns `SimPlayer.WAIT` while the current frame should be held.
- The joystick knob and pan/tilt sliders animate from the recorded values; user input to those controls is blocked during sim, and pre-sim pan/tilt is restored on exit (so servos don't jump when real sends resume).
- Works on any machine (macOS included) — only needs the `logots` conda env and a recording CSV. Hardware readers keep running but are ignored (stopping `IMUReader` would force a ~10 s recalibration per toggle); I2C sends and reconnects are skipped.
- **LOOP** checkbox: wrap at end-of-file vs freeze on last frame (`SIM ended`).
- REC is disabled during sim (auto-stopped when entering); malformed CSV rows are skipped and counted; canceling the file dialog stays in Real mode.
- Status labels (IMU/audio/video) show `SIM`; header shows `⬤ SIM MODE`; motor labels show the CSV's recorded values.

## Frame API (for downstream processing)

`FrameServer` inside the GUI serves `GET http://localhost:8787/latest_frame` (JSON) in both modes. `frame_data` is base64 color JPEG — in Real mode it's encoded lazily per request (cached by `frame_id`) so the 20 Hz tick never pays for it; the four array fields are real JSON arrays; `pos_x`/`pos_y` (m) and `heading` (°) carry the dead-reckoned body position; `sim_mode` (bool) is included. Client helper:

```python
from logots_api import get_latest_frame   # src/logots_api.py
frame = get_latest_frame()                # adds frame['image']: 640×640×3 uint8 RGB numpy
```

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
2. **Camera always needs `EGL_PLATFORM=surfaceless`** — DISPLAY=:0 and DISPLAY=:1001.0 both fail for nvarguscamerasrc.
3. **Don't use system cv2 from conda** — it's compiled for NumPy 1.x and will crash with conda's NumPy 2.x.
4. **Arduino I2C is always bus 1** — confirmed with `i2cdetect -y -r 1`, shows 0x08.
5. **Always work in the git repo** — `/home/logots/Desktop/Logots_V2/`. The old `logots/` directory is archived.

## Known issues / next steps
- Camera has pink/IR hue — missing IR cut filter on IMX219-160 fisheye. Need M12 IR cut filter hardware.
- Motors and servos not yet physically connected to Arduino — I2C command flow confirmed working, hardware wiring pending.
- Staging layer CSV + sim mode + frame API done (Asaph can develop off-robot against `logots_api.get_latest_frame()`); transformation + mart + decision layers not yet written.
- Color recording not yet exercised on the Jetson (grayscale→color change verified on Mac only) — record a short session next time on the robot and confirm the JPEGs are RGB.
- The 20 Hz loop actually achieves ~10.5 Hz on the Jetson (tick work + `after(50)` re-arm). Recordings are timestamped so sim playback is unaffected, but worth knowing for downstream timing assumptions.
- **Calibrate `ROBOT_MAX_SPEED_MPS`** (in `logots_ui.py`) on the robot once motors are connected: drive a known distance at full PWM for a known time and set the constant to `distance/time`. Until then `pos_x`/`pos_y` are directionally right (heading is real IMU data) but not metrically accurate, and they assume motors track commands (no encoder feedback). Position also drifts with IMU yaw drift over long sessions.
