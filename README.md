# Logots V2 — PlantSitter

A home plant-monitoring and care robot. Logots moves around your home, keeps an eye on your plants, streams live camera and audio, and inspects plants up close with a pan/tilt camera tower. You can talk to it ("Hey Jarvis …") or move it from a single GUI with three commands: traverse, rotate and servo.

Built around a **Jetson Orin Nano** running JetPack 6, with an **Arduino** handling motor and servo control over I2C.

---

## How it fits together

One command starts two processes:

- **The GUI** (`src/logots_ui.py`) owns the sensors and motors. It serves the latest sensor snapshot on **`:8787`** and carries out motion commands.
- **The listener** (`src/functions.py --listen`) owns the decisions. It hears the wake word, asks a local Gemma model (`llama-server` on `:8789`) what to do, and publishes the chosen action on **`:8788`**, where the GUI picks it up and reports back when done.
- **Memory** lives in `knowledge/`: the robot's identity, the list of allowed actions, and what it has learned about your plants.

Design notes and the full action contract: [`docs/v2_architecture/`](docs/v2_architecture/).

---

## Hardware

| Component | Part | Notes |
|---|---|---|
| Compute | Jetson Orin Nano (8GB) | JetPack 6, ARM64 |
| Camera | IMX219-160 CSI fisheye | CAM0 port |
| Microphone | INMP441 | I2S2 input |
| Amplifier | MAX98357A | I2S2 output |
| IMU | Grove IMU 9DOF V2.2 (MPU-9250) | I2C, pins 3/5 |
| Motor controller | Arduino Uno + HW-130 shield (Adafruit Motor Shield v1 clone) | I2C address 0x08 via level shifter, pins 27/28 |
| Drive motors | 2× JGA25-370 DC gear motors with Hall encoders | M1 = right, M4 = left; encoders right A0/A1, left A2/A3 |
| Camera servos | 2× MG90S | Pan = pin 10, Tilt = pin 9 |
| Level shifter | BSS138 bidirectional (4-ch) | Jetson 3.3V ↔ Arduino 5V |
| Power | 12V battery pack | 12V to the shield's EXT_PWR (motors), 9V buck converter to the Arduino |
| Storage | 500GB Kingston NVMe | nvme0n1p1 |

Full wiring, including the power rules: [`src/pinout.txt`](src/pinout.txt).

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

> Requires [Miniconda](https://docs.conda.io/en/latest/miniconda.html). The same `environment.yml` works on the Jetson and on macOS. To bring an existing env up to date: `conda env update -n logots -f environment.yml`.
> Always run through conda: the system Python lacks required packages and the system OpenCV is incompatible with NumPy 2.x.

### 3. Configure the Jetson's pins (one-time)

The camera, IMU, Arduino and audio all need device tree overlays. **Never edit `/boot/extlinux/extlinux.conf` by hand**; use `jetson-io.py`:

```bash
sudo /opt/nvidia/jetson-io/jetson-io.py
```

- **Configure Jetson 40pin Header → Configure header pins manually**, and enable all four groups: `i2c8` (pins 3/5, IMU), `i2c2` (pins 27/28, Arduino), `i2s2` (pins 12/35/38/40, mic + amp), `uarta` (pins 8/10). Saving rewrites the whole pin set, so a group left out is switched off. The known-good screen: [`docs/robot1_system/header_pinouts.png`](docs/robot1_system/header_pinouts.png).
- **Configure Jetson 24pin CSI Connector** → `Camera IMX219-A`.

Reboot after saving.

> After any `nvidia-l4t-*` package upgrade, check that the camera still works. An upgrade can reset the boot entry and the camera disappears. Rerun `jetson-io.py` and select the camera again to fix it.

### 4. Flash the Arduino firmware

Arduino IDE 2.x has no Linux ARM64 build, so flash from a Mac or Windows machine:

1. Install [Arduino IDE](https://www.arduino.cc/en/software) on your laptop
2. Install the **Adafruit Motor Shield** library: `Sketch → Include Library → Manage Libraries → search "AFMotor"`
3. Open `src/firmware/logots_motor_control/logots_motor_control.ino` (`motion.h` in the same folder is picked up automatically)
4. Select the board (Arduino Uno) and USB port, then upload

If a wheel spins the wrong way or the camera doesn't face front at pan 0, fix it in the calibration block at the top of the `.ino` (`LEFT_MOTOR_DIR`/`RIGHT_MOTOR_DIR`, `PAN_CENTER_DEG`/`PAN_DIR`) and reflash.

### 5. Set up the voice assistant

The listener needs three one-time pieces. Without them the GUI still works and shows `⬤ LLM: error`.

**The LLM**: a [llama.cpp](https://github.com/ggml-org/llama.cpp) `llama-server` build (on the Jetson with CUDA, `-DGGML_CUDA=ON`), plus the Gemma 4 E2B GGUF and its mmproj file. Default locations:

```
~/llama.cpp/build/bin/llama-server
~/models/gemma-4-E2B-it-qat-q4_0-gguf/gemma-4-E2B_q4_0-it.gguf
~/models/gemma-4-E2B-it-qat-q4_0-gguf/gemma-4-E2B-it-mmproj.gguf
```

Elsewhere, set `LLAMACPP_BIN`, `LLAMACPP_MODEL` and `LLAMACPP_MMPROJ` (see `src/audio_on_demand.py`). Voice needs this llama.cpp build on every machine, the Mac included. Without it the listener falls back to a `transformers` model that can't take audio, so it never hears what you say.

**Piper TTS voice** (~60 MB):
```bash
mkdir -p ~/models/piper
curl -L -o ~/models/piper/en_US-lessac-medium.onnx \
  https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/lessac/medium/en_US-lessac-medium.onnx
curl -L -o ~/models/piper/en_US-lessac-medium.onnx.json \
  https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/lessac/medium/en_US-lessac-medium.onnx.json
```

**Wake-word models** (once per conda env; the pip package doesn't include them):
```bash
conda run -n logots python -c "from openwakeword.utils import download_models; download_models()"
```

---

## Running

```bash
conda run -n logots python src/logots_ui.py
```

This starts the whole robot: the GUI, then the voice listener as its own process. The model takes about a minute to load on the Jetson. No display variable is needed under NoMachine; the camera pipeline sets `EGL_PLATFORM=surfaceless` itself.

> **The robot may start driving by itself.** If it doesn't know any plants yet, the listener runs *initiation* as soon as the model has loaded: it looks around and drives toward plants to learn them, for up to 12 steps, and can't hear you until it's done. Clear the floor the first time. The `CLOSE GUI` button stops everything.

To start the GUI without the listener (no voice, no initiation), for example to run `functions.py` by hand or to test motion:

```bash
LISTEN=0 conda run -n logots python src/logots_ui.py
```

Only one listener can run at a time; a second one exits with a message.

---

## GUI overview

```
  LOGOTS ROBOT CONTROL          ⬤ MIC: asleep  ⬤ LLM: ready  FPS:9.8/10  ⬤ CONNECTED
┌─────────────────────┬─────────────────────┐
│  DRIVE & CAM        │  IMU ORIENTATION    │
│  traverse/rotate +  │  3D Madgwick AHRS   │
│  pan/tilt sliders,  │  YPR display        │
│  map below (full w) │                     │
├─────────────────────┼─────────────────────┤
│  AUDIO INPUT        │  VIDEO FEED         │
│  waveform + RMS +   │  live IMX219 feed   │
│  GAIN slider +      │                     │
│  last voice action  │                     │
└─────────────────────┴─────────────────────┘
  L:+000  R:+000  PAN:+00°  TLT:+00°  X+0.00 Y+0.00  HDG:090°  ⌖ POS  LOOP  ▶ SIM  ⚫ REC  ■ STOP  CLOSE GUI
```

**The robot is moved by three commands only.** There is no free driving:

- **Traverse** (cm, + = forward): `FWD 50`/`FWD 10`/`BWD 10`/`BWD 50`, or type a distance (up to ±200 cm) and `GO`
- **Rotate** (degrees, + = left): `L 90°`/`L 15°`/`R 15°`/`R 90°`, or type an angle (up to ±180°) and `GO`
- **Servo** (pan, tilt): the two sliders. Pan 0 = front, + = left (±45°); tilt 0 = level, + = up

Each press sends one command; the robot carries it out and stops by itself. The badge in the panel title shows what the wheels are doing and who is moving the robot (you or the LLM). While the LLM is moving it, the buttons are ignored; `SPACE` and `■ STOP` still cancel.

**Keyboard:** `W/S` = traverse 10 cm forward/back · `A/D` = rotate 15° left/right · `SPACE` = stop

The **position mini-map** shows a top-down trail of where the robot is, worked out from the wheel encoders and the IMU heading (about 2% error over 1 m in a tape-measure test). The map is centered on the robot with 1 m gridlines. `⌖ POS` sets the origin to "here".

`CLOSE GUI` closes the GUI and the listener; it does not shut down the Jetson. The GUI connects to the Arduino on startup and retries every 3 seconds if the connection drops. The header's `FPS:actual/target` turns amber when the loop can't keep up with `TARGET_FPS` (10 Hz, set at the top of `src/logots_ui.py`).

---

## Sim mode (no robot needed)

Press **▶ SIM** and pick a session CSV (the output of **⚫ REC**). The GUI replays the recording as if it were live: video, IMU, audio waveform, sliders and motor values all come from the file, paced by the recording's own timestamps. **LOOP** chooses whether playback wraps or freezes on the last frame. Press **⏹ REAL** to return to live sensors.

Sim mode runs on any machine, no hardware needed:

```bash
conda env create -f environment.yml
LISTEN=0 conda run -n logots python src/logots_ui.py     # then press SIM and pick a recording CSV
```

(`LISTEN=0` skips the model load. Without it the listener runs too and hears the replayed audio, but drives it decides aren't carried out in Sim.)

---

## Voice assistant

Say **"Hey Jarvis"** followed by a request, for example *"go closer to the plant"* or *"what plant is this?"*. The listener picks one action from the closed list in [`knowledge/actions.md`](knowledge/actions.md) and carries it out:

| Action | What it does |
|---|---|
| `inspect` | look through the camera and describe what it sees |
| `approach_plant` | drive toward a plant |
| `speak` | say something out loud |
| `finish` | end the running routine and report |

- **Status**: the header shows `⬤ MIC:` (asleep → listening → thinking → acting → speaking, or `initiation`) and `⬤ LLM:` (loading → ready). The AUDIO INPUT panel's `ACTION` line shows the last decision, e.g. `approach_plant(id=unknown, left_pwm=150, right_pwm=150, duration_s=1.5)`.
- **While it handles a request it isn't listening**, so it can't hear you during a drive.
- **On NoMachine**: if you hear the reply on your *laptop's* speakers instead of the robot's, that's NoMachine's audio redirection. Run `unset PULSE_SERVER` before launching the GUI from a NoMachine terminal.

---

## APIs

**Frame API (`:8787`).** In both Real and Sim modes the GUI serves its latest sensor snapshot. From any script or notebook (same conda env, GUI running):

```python
import sys; sys.path.append('path/to/Logots_V2/src')
from logots_api import get_latest_frame

frame = get_latest_frame()
frame['image']           # 640×640×3 uint8 RGB numpy array (None if no camera)
frame['yaw']             # list of yaw readings (°) since the previous frame
frame['audio_samples']   # list of mic samples (~1600 per frame at 10 FPS), GAIN already applied
frame['mic_gain_db']     # that gain, in dB (the GAIN slider in AUDIO INPUT)
frame['left_pwm']        # motor/servo commands: left_pwm, right_pwm, pan_angle, tilt_angle
frame['pos_x']           # body position (m): pos_x, pos_y, and heading (°)
```

The raw endpoint is `GET http://localhost:8787/latest_frame` (JSON). Position comes from the wheel encoders and the IMU heading; it assumes the wheels don't slip and drifts slowly with the IMU's yaw. A full example, video playback with synchronized audio: run the GUI, then `python src/api_demo.py`.

**Action API (`:8788`).** The listener publishes actions on `GET :8788/latest_action`; the GUI carries out `approach_plant` and the three motion commands (`traverse`, `rotate`, `servo`) and answers on `POST :8788/action_done` with where the robot ended up. Spec for the motion commands: [`docs/v2_architecture/motion_commands.md`](docs/v2_architecture/motion_commands.md).

---

## Arduino I2C protocol

The Jetson sends a drive message to `0x08` on `/dev/i2c-1` once per capture tick (~10 Hz):

```
"{left_pwm},{right_pwm},{pan_angle},{tilt_angle}\n"
```

| Field | Range | Description |
|---|---|---|
| left_pwm | -255 to +255 | Left motor, + = forward |
| right_pwm | -255 to +255 | Right motor, + = forward |
| pan_angle | -90 to +90 | Camera pan, 0 = front, + = left (the GUI limits it to ±45) |
| tilt_angle | 0 to 180 | Raw tilt servo angle (the GUI's signed tilt is converted on the Jetson) |

For traverse and rotate, the Jetson sends one `R` (rotate) or `T` (traverse) message, and the Arduino runs the wheels to an encoder-count target and stops by itself. A 13-byte read from `0x08` returns both encoder counts and the motion's status. The firmware stops the motors if no message arrives for 500 ms. Formats and details: the header of `src/firmware/logots_motor_control/logots_motor_control.ino`, and `motion.h` next to it. Firmware older than 2026-11-10 has no motion support, and the GUI then refuses traverse/rotate with "Arduino needs the new firmware".

The Arduino echoes each drive message to Serial at 9600 baud: `OK  L=200 R=200 PAN=0 TILT=90`.

---

## Testing motion

`src/motion_live_test.py` plays the LLM's part: it sends each motion command over `:8788` and checks the result against the IMU, encoders and servo angles. Start the GUI with `LISTEN=0`, then:

```bash
conda run --no-capture-output -n logots python src/motion_live_test.py --stage raised  # wheels off the floor
conda run --no-capture-output -n logots python src/motion_live_test.py --stage floor   # 1 m clear all round, ~3 min
conda run --no-capture-output -n logots python src/motion_live_test.py --stage tape    # asks you for tape-measure readings
```

Each stage prints PASS/FAIL per check and saves a report under `runs/`. It can't stop a motion itself; keep a hand on `SPACE` in the GUI.

---

## Remote access

NoMachine, port 4000:

- `logots-desktop.local` on any network (robot #1)
- `192.168.55.1` over the USB-C cable, which works whatever network the robot is on

Setting up a new Jetson (WiFi, NoMachine permissions, pin config): see "New Jetson setup" in [`CLAUDE.md`](CLAUDE.md).

---

## Repo map

```
src/
  logots_ui.py          GUI: sensors, motors, recording, sim mode, :8787 frame server
  functions.py          the listener / decision layer (--listen), routines like initiation
  actions.py            the :8788 action endpoint
  audio_on_demand.py    wake word, llama-server management, Piper speech
  logots_api.py         client for the frame API
  motion_live_test.py   live test of traverse / rotate / servo
  firmware/logots_motor_control/   Arduino firmware (.ino + motion.h)
  pinout.txt            wiring reference
knowledge/              prompts and the robot's memory (only the prompt files are tracked)
docs/                   architecture notes, test reports, robot #1's system files
recordings/             session CSVs from ⚫ REC (not tracked)
```

`CLAUDE.md` has the full engineering notes: calibration history, known bugs and how they were found.

---

## Known issues

- **Traverse, rotate and servo have not run on the robot yet.** The tuning values in `src/logots_ui.py` (`SPIN_*`, `ROT_*`, `TRAV_*`, `TILT_*`) are starting guesses; the first-run checklist is in `CLAUDE.md` under "Known issues".
- **Initiation has no limit on driving yet.** In one off-robot test it chose to drive 8 times in a row. There is no obstacle detection, so watch it and keep `CLOSE GUI` within reach.
- Position trusts the IMU's yaw, which drifts slowly over a long session.
- The camera has a pink/IR hue: the IMX219-160 fisheye has no IR cut filter. Fix: an M12 IR cut filter (hardware).
