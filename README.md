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

---

## Running the GUI

```bash
conda run -n logots python src/logots_ui.py
```

For headless / NoMachine sessions the GUI connects automatically. No display variable needs to be set — the camera pipeline uses `EGL_PLATFORM=surfaceless` internally.

---

## GUI overview

```
┌─────────────────────┬─────────────────────┐
│  DRIVE & CAM        │  IMU ORIENTATION    │
│  joystick + pan/    │  3D Madgwick AHRS   │
│  tilt sliders       │  YPR display        │
├─────────────────────┼─────────────────────┤
│  AUDIO INPUT        │  VIDEO FEED         │
│  waveform + RMS     │  live IMX219 feed   │
└─────────────────────┴─────────────────────┘
  L: +0000  R: +0000  PAN:090°  TLT:090°   ■ STOP
```

**Keyboard shortcuts:** `W/S` = forward/back · `A/D` = turn · `SPACE` = stop

The GUI auto-connects to the Arduino on startup and retries every 3 seconds if the connection drops.

---

## Arduino I2C protocol

Messages are sent from the Jetson to the Arduino at 20 Hz:

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
- Motors and servos not yet physically wired to the Arduino — I2C command flow is confirmed working.
