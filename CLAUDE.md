# Logots Robot — Project Guide for Claude

## What this is
A home plant-monitoring and care robot ("PlantSitter"). Runs on a Jetson Orin Nano.
The GUI (`logots_ui.py`) is the single control surface — it shows live sensor data and sends motor/servo commands to an Arduino over I2C.

## How to run
```bash
conda run -n logots python /home/logots/Desktop/Logots_V2/src/logots_ui.py
```
- Conda env: `logots` (Python 3.10, NumPy 2.2.5)
- Always run through conda — system Python is missing deps and system OpenCV is incompatible (NumPy 1.x vs 2.x)

## Platform
- **Hardware**: Jetson Orin Nano, JetPack 6 (L4T R36.4.7), ARM64
- **Remote access**: NoMachine at 192.168.68.114:4000. Virtual display is `:1001.0`
- **Storage**: NVMe nvme0n1p1 (500GB Kingston). SD card removed.
- **Shutdown timeout**: systemd set to 5s (`/etc/systemd/system.conf`)

## Key files
| File | Purpose |
|---|---|
| `src/logots_ui.py` | Main GUI — all sensors + motor control |
| `logots_motor_control/logots_motor_control.ino` | Arduino firmware |
| `pinout.txt` | Full 40-pin header wiring reference |
| `camera_test.sh` | Capture 10 test frames from IMX219 to Desktop |
| `imu_test.py` | Standalone IMU test |

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
│  tilt sliders       │  YPR display        │
├─────────────────────┼─────────────────────┤
│  AUDIO INPUT        │  VIDEO FEED         │
│  waveform + RMS     │  live IMX219 feed   │
└─────────────────────┴─────────────────────┘
  L: +0000  R: +0000  PAN:090°  TLT:090°   ■ STOP
```
- Keyboard: W/S = forward/back, A/D = turn, SPACE = stop
- Arduino auto-connects on startup, retries every 3s if lost

## Critical rules
1. **Never manually edit `/boot/extlinux/extlinux.conf`** — always use `jetson-io.py`. Manual edits brick the boot.
2. **Camera always needs `EGL_PLATFORM=surfaceless`** — DISPLAY=:0 and DISPLAY=:1001.0 both fail for nvarguscamerasrc.
3. **Don't use system cv2 from conda** — it's compiled for NumPy 1.x and will crash with conda's NumPy 2.x.
4. **Arduino I2C is always bus 1** — confirmed with `i2cdetect -y -r 1`, shows 0x08.

## Known issues / next steps
- Camera has pink/IR hue — missing IR cut filter on IMX219-160 fisheye. Need M12 IR cut filter hardware.
- Motors and servos not yet physically connected to Arduino — I2C command flow confirmed working, hardware wiring pending.
- No unified SensorFrame / MotorCommand schema yet — main robot loop not written.
