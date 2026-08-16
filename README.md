# Logots V2 — PlantSitter

An autonomous home plant-monitoring and care robot. Logots roams your home, keeps an eye on your plants, streams live camera and audio, and lets you drive it remotely and inspect any plant up close using a pan/tilt camera tower — all from a single GUI.

Built around a **Jetson Orin Nano** running JetPack 6, with an **Arduino** handling motor and servo control over I2C.

---

## Hardware

| Component | Part | Notes |
|---|---|---|
| Compute | Jetson Orin Nano (8GB) | JetPack 6, ARM64 |
| Camera | IMX219-160 CSI fisheye | CAM0 port |
| Microphone | INMP441 | I2S2 input |
| Amplifier | MAX98357A | I2S2 output |
| IMU | Grove IMU 9DOF V2.2 (MPU-9250) | I2C, pins 3/5 |
| Motor controller | Arduino + Adafruit Motor Shield v1 | I2C via level shifter, pins 27/28 |
| Drive motors | 2× DC motors | Left = ch3, Right = ch4 |
| Camera servos | 2× MG90S | Pan = pin 10, Tilt = pin 9 |
| Level shifter | BSS138 bidirectional (4-ch) | Jetson 3.3V ↔ Arduino 5V |
| Storage | 500GB Kingston NVMe | nvme0n1p1 |

For full wiring details see [`src/pinout.txt`](src/pinout.txt).

---

## Software setup

### 1. Clone the repo

```bash
git clone https://github.com/OrBerebi/Logots_V2.git
cd Logots_V2
```

### 2. Create the conda environment

```bash
conda env create -f environment.yml
conda activate logots
```

> Requires [Miniconda](https://docs.conda.io/en/latest/miniconda.html) installed on the Jetson.
> Always run through conda — the system Python lacks required packages and the system OpenCV is incompatible with NumPy 2.x.

### 3. Enable device tree overlays (one-time)

The camera and I2S audio require device tree overlays. **Never edit `/boot/extlinux/extlinux.conf` manually** — use `jetson-io.py`:

```bash
sudo python3 /opt/nvidia/jetson-io/jetson-io.py
```

Enable:
- `HDR40 User Custom` → configure I2S2 pins for audio
- `Configure Jetson 24pin CSI Connector` → `Camera IMX219-A`

Reboot after saving.

### 4. Flash the Arduino firmware

Arduino IDE 2.x has no Linux ARM64 build — flash from a Mac or Windows machine:

1. Install [Arduino IDE](https://www.arduino.cc/en/software) on your laptop
2. Install the **Adafruit Motor Shield** library: `Sketch → Include Library → Manage Libraries → search "AFMotor"`
3. Open `src/firmware/logots_motor_control/logots_motor_control.ino`
4. Select your board and USB port, then upload

### 5. Set up the voice assistant (optional)

The GUI can listen for a wake word, understand a spoken request with a local LLM, and speak
the answer back — see [Voice assistant](#voice-assistant) below for how it works. It runs
automatically once these one-time pieces are in place:

**Piper TTS voice** (needed on any machine, ~60 MB, one-time download):
```bash
mkdir -p ~/models/piper
curl -L -o ~/models/piper/en_US-lessac-medium.onnx \
  https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/lessac/medium/en_US-lessac-medium.onnx
curl -L -o ~/models/piper/en_US-lessac-medium.onnx.json \
  https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/lessac/medium/en_US-lessac-medium.onnx.json
```

**Wake-word models** (needed on any machine, one-time — the pip package doesn't bundle them):
```bash
conda run -n logots python -c "from openwakeword.utils import download_models; download_models()"
```

**On the Jetson**, the LLM ("brain") runs locally via a [llama.cpp](https://github.com/ggml-org/llama.cpp)
`llama-server` build with CUDA enabled, plus a Gemma 4 E2B GGUF (weights + mmproj file) under
`~/models/`. Build llama.cpp per its own instructions (`-DGGML_CUDA=ON`) and point
`LLAMACPP_BIN`/`LLAMACPP_MODEL`/`LLAMACPP_MMPROJ` (env vars, see `src/audio_on_demand.py`) at
your build if it lives somewhere other than `~/llama.cpp/build/bin/llama-server` and
`~/models/gemma-4-E2B-it-qat-q4_0-gguf/`.

**On a Mac** (or anywhere without that Jetson-specific build), the GUI automatically falls back
to a Hugging Face `transformers` pipeline instead — just add:
```bash
pip install torch transformers
```
to the same `logots` env. No further setup: the correct brain is auto-selected at runtime.

---

## Running the GUI

```bash
conda run -n logots python src/logots_ui.py
```

For headless / NoMachine sessions the GUI connects automatically. No display variable needs to be set — the camera pipeline uses `EGL_PLATFORM=surfaceless` internally.

---

## GUI overview

```
  LOGOTS ROBOT CONTROL          ⬤ MIC: asleep  ⬤ LLM: ready  FPS:8.2/10  ⬤ CONNECTED
┌─────────────────────┬─────────────────────┐
│  DRIVE & CAM        │  IMU ORIENTATION    │
│  joystick + pan/    │  3D Madgwick AHRS   │
│  tilt sliders +     │  YPR display        │
│  position mini-map  │                     │
├─────────────────────┼─────────────────────┤
│  AUDIO INPUT        │  VIDEO FEED         │
│  waveform + RMS +   │  live IMX219 feed   │
│  last voice action  │                     │
└─────────────────────┴─────────────────────┘
  L:+000  R:+000  PAN:090°  TLT:090°  X+0.00 Y+0.00  HDG:090°  ⌖ POS   LOOP ▶ SIM  ⚫ REC  ■ STOP
```

The header's `⬤ MIC:`/`⬤ LLM:` indicators and the AUDIO INPUT panel's `ACTION` line show the
[voice assistant](#voice-assistant)'s live status.

A **position mini-map** at the bottom of the DRIVE & CAM panel shows a top-down trail of where the robot thinks its body is (dead-reckoned from motor commands + IMU heading), with an `X/Y/HDG` readout in the status bar and a `⌖ POS` button to reset the origin to "here". See the position note below.

**Keyboard shortcuts:** `W/S` = forward/back · `A/D` = turn · `SPACE` = stop

The GUI auto-connects to the Arduino on startup and retries every 3 seconds if the connection drops.

The header shows a live **`FPS:actual/target`** readout (green when within 10% of target, amber below). The capture/recording rate is `TARGET_FPS` (default **10 Hz**) and the camera runs at `CAMERA_FPS` (default **15**, down from the sensor's 30) — both are constants at the top of `src/logots_ui.py`. The loop is drift-compensated, so the actual rate tracks the target as long as the Jetson can keep up.

---

## Sim mode (no robot needed)

Press **▶ SIM** and pick a session CSV (the output of the **⚫ REC** feature). The GUI replays the recording row-by-row exactly as if the data were sampled live: video, IMU orientation, audio waveform, joystick, sliders, and motor/servo values all come from the file. Playback is paced by the recording's own timestamps, so it runs in real time. The **LOOP** checkbox chooses whether playback wraps around at the end of the file or freezes on the last frame. Press **⏹ REAL** to return to live sensors.

Sim mode runs on any machine — no Jetson, no hardware. On a MacBook:

```bash
conda env create -f environment.yml
conda activate logots
python src/logots_ui.py        # then press SIM and pick a recording CSV
```

---

## Voice assistant

Say **"Hey Jarvis"** followed by a request — e.g. *"water the ficus"*, *"how are my plants
doing?"* — and the robot listens, decides what to do with a local LLM, and speaks its answer
back out loud. No extra process or terminal: it starts automatically with the GUI (see
[setup](#5-set-up-the-voice-assistant-optional) above for the one-time model downloads).

- **Status**: the header shows `⬤ MIC:` (asleep → listening → thinking → speaking) and
  `⬤ LLM:` (loading → ready), and the AUDIO INPUT panel's `ACTION` line shows the last thing it
  decided, e.g. `water_plant(id=ficus, ml=250)`.
- **The LLM ("brain") is auto-selected**: the Jetson runs a fast, GPU-offloaded local
  `llama-server` (~1-2s per response); a Mac with no such build falls back to a Hugging Face
  `transformers` pipeline (~2-4s per response) using the same real mic and speakers. Nothing to
  configure — whichever is available on the machine just gets used.
- **On NoMachine**: if you hear the response on your *laptop's* speakers instead of the robot's,
  that's NoMachine's remote-desktop audio redirection, not a bug — run `unset PULSE_SERVER`
  before launching the GUI from a NoMachine terminal.

---

## Frame API

In both Real and Sim modes the GUI serves its latest sensor snapshot over HTTP on port **8787**. From any script or notebook (same conda env, GUI running):

```python
import sys; sys.path.append('path/to/Logots_V2/src')
from logots_api import get_latest_frame

frame = get_latest_frame()
frame['image']           # 640×640×3 uint8 RGB numpy array (None if no camera)
frame['yaw']             # list of yaw readings (°) since the previous frame
frame['audio_samples']   # list of mic samples (~1600 per frame at 10 FPS)
frame['left_pwm']        # motor/servo commands: left_pwm, right_pwm, pan_angle, tilt_angle
frame['pos_x']           # estimated body position (m): pos_x, pos_y, and heading (°)
```

> **Position (`pos_x`/`pos_y`/`heading`) is a dead-reckoning estimate, not measured odometry** — the
> drive motors have no encoders. Forward speed is modelled from the commanded PWM and direction from the
> IMU yaw, so distances are only as accurate as the `ROBOT_MAX_SPEED_MPS` calibration constant in
> `logots_ui.py`. Recordings made before this feature simply omit the columns.

The raw endpoint is `GET http://localhost:8787/latest_frame` (JSON), if you'd rather not use the helper.

For a complete working example — video playback with synchronized audio — run the GUI (sim or real), then in a second terminal:

```bash
python src/api_demo.py
```

---

## Arduino I2C protocol

Messages are sent from the Jetson to the Arduino once per capture tick (`TARGET_FPS`, default 10 Hz):

```
"{left_pwm},{right_pwm},{pan_angle},{tilt_angle}\n"
```

| Field | Range | Description |
|---|---|---|
| left_pwm | -255 to +255 | Left motor speed/direction |
| right_pwm | -255 to +255 | Right motor speed/direction |
| pan_angle | 0 to 180 | Camera pan (rotation) |
| tilt_angle | 0 to 180 | Camera tilt (elevation) |

The Arduino echoes each received command to Serial at 9600 baud:
```
OK  L=200 R=200 PAN=90 TILT=45
```

---

## Remote access

NoMachine at `192.168.68.114:4000` — provides a full remote desktop on the Jetson.

---

## Known issues

- Camera has a pink/IR hue — the IMX219-160 fisheye has no IR cut filter. Fix: M12 IR cut filter (hardware).
- Drive motors have no encoders (open-loop), so the `pos_x`/`pos_y` estimate is directional but not metrically accurate until `ROBOT_MAX_SPEED_MPS` is calibrated. A feedback drivetrain is planned for the next body iteration.
- A NoMachine remote-desktop session runs its own audio server and can silently redirect the voice assistant's spoken replies to your laptop's speakers instead of the robot's — run `unset PULSE_SERVER` before launching the GUI from a NoMachine terminal if that happens.
