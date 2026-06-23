# Logots V1 — System Guide

A complete reference for the first version of the Logots cat-sitter robot. This document is written so that an engineer could **rebuild the robot from scratch**: it names the boards, wiring, firmware sketches, Python modules, key functions, models, and constants — without reproducing the code itself.

The system follows a layered (medallion-style) data pipeline:

```
Sensory Input → Staging → Transformation → Mart → Decision → Execution → (back to Hardware)
```

---

## 1. Overview & Purpose

Logots is an AI-powered cat-sitter robot. Four ESP32-based boards stream raw sensor data (camera, microphone, IMU, motor state) over WiFi to a Python backend running on a Mac. The backend turns those raw streams into structured "experiences," runs two layers of decision-making over them (fast rules + a slower LLM), and sends motor commands back to the robot to approach, retreat, gaze at, or play with a cat.

The guiding design idea: **treat robot perception as a data pipeline.** Every stage has a name and a persisted output, so the whole run can be inspected after the fact like a database, not a black box. Entry point is `run.py`, which launches a Tkinter GUI (`gui_controller.py`).

---

## 2. Hardware

The robot is four independent microcontroller boards plus the host computer.

| Subsystem | Board | Sensor/Actuator | Link |
|---|---|---|---|
| Vision | ESP32-CAM | OV2640 camera, 640×480 JPEG | HTTP `/capture` on port 80 |
| Audio (in) | ESP32 + INMP441 | I²S MEMS microphone, 16 kHz mono | TCP port 12345 |
| IMU | ESP32 + Grove 9DOF | yaw/pitch/roll stream | TCP port 12345 |
| Motors + Audio (out) | ESP32 → Arduino Uno + L293D | 2 DC wheels, 1 arm servo, I²S speaker | TCP port 12345 (motors), 12346 (audio playback) |

Key hardware facts:
- **PWM range** ~60–150; arm servo 0–180°. Cruise PWM is 150.
- The **motor ESP32 also drives the speaker** (MAX98357A I²S amp) — audio-out and motor commands share that board on two different ports (12345 / 12346).
- The ESP32↔Arduino link is I²C; the ESP32 forwards `left,right,arm` commands to the Arduino which drives the L293D.
- **Power is the main fragility:** the ESP32-CAM browns out and drops off WiFi under a sagging supply. Symptom is fast→slow→offline `/capture` responses over minutes. Power-cycling recovers it.
- Each board joins WiFi with hardcoded SSID/password in its firmware; the host reaches them by fixed IP.

---

## 3. From Hardware to Data (the ingest boundary)

This is the inbound half of the system: firmware on each board emits a stream, and a dedicated Python producer thread pulls it into a shared buffer.

### Firmware (Arduino/ESP32 sketches, `src/logots/firmware/`)
- `audio/esp32_audio/esp32_audio.ino` — reads INMP441 over I²S, streams raw 16-bit PCM over TCP.
- `imu/imu_ypr_v6/imu_ypr_v6.ino` — reads the Grove 9DOF, prints yaw/pitch/roll lists per frame over TCP.
- `video/ESP32_CameraWebServer/ESP32_CameraWebServer.ino` — standard camera web server exposing `/capture` (JPEG) and `/control` (resolution).
- `motors/.../esp32_motor_and_audio_out.ino` — accepts `left,right,arm` motor commands on port 12345, forwards to the Arduino over I²C; also runs an I²S audio-playback server on port 12346.

### Python ingest (`recording_module.py`)
- **Producer threads**, one per stream: `stream_audio`, `stream_imu`, `stream_motors`, and `VideoStreamProducer` (a `threading.Thread` subclass that fetches `/capture` at `FPS=4`).
- **`ThreadSafeBuffer`** — the shared nervous system. Producers append raw frames; the pipeline consumes them. Holds per-stream lists (`visual`, `imu`, `motor`, `audio_pipeline`), rolling context, and the persisted "master" result tables. Access is guarded by a lock; CSV writes by a second `csv_lock`.
- **`configure_camera`** sets the camera framesize/quality at startup via `/control`.
- The whole thing is started by **`run_data_collection(duration, stop_event, debug)`**, which spawns every producer, the pipeline consumer, the transcription producer, the reflective-decision thread, and the audio-playback consumer, then waits on `stop_event`.

Conceptually this is the "Sensory Input" box of the diagram becoming the "Staging" tables.

---

## 4. Full Data Pipeline

The pipeline runs in `pipeline_consumer` → `process_chunk`, which fires once per **12-frame chunk** (`MART_WINDOW_SIZE = 12`, ~3 s at 4 fps). Each chunk flows through the layers below. Global cadence constants: `FPS = 4`, `AUDIO_SAMPLE_RATE = 16000`.

### 4.1 Staging (`stg_*`)
Raw, untransformed capture as it left the sensors: `stg_visual_data`, `stg_audio_data`, `stg_imu_data`, `stg_motor_data`. This is the ground truth before any model runs — the chunk that `get_chunk_if_ready` hands to `process_chunk`.

### 4.2 Transformation (`trans_*`, in `transformation_mart_pipeline.py`)
Each modality is converted from raw signal into features. One function per transform:

- **`transform_visual`** → `trans_visual_cat_detection`. Runs **YOLOv8m** (`yolov8m.pt`, COCO class 15 = cat, confidence threshold `VISUAL_CONF_THR = 0.10`) on each frame; outputs cat centroid, bounding-box area, confidence. Model loaded once via `get_visual_model` (MPS, falls back to CPU).
- **`transform_audio`** → `trans_audio_features`. Runs the **AST** model (`mit/ast-finetuned-audioset-10-10-0.4593`, via `get_audio_model`) on a 3-second rolling buffer to classify cat-meow vs. human-voice vs. motor noise; outputs `is_cat_voice`, `is_human_voice`, `meow_loudness`. Includes a motor-noise veto.
- **`trans_audio_transcribe`** → speech text. Runs **Whisper tiny** (Apple **MLX** build `mlx-community/whisper-tiny`, `get_whisper_model`) on a rolling window; emits the `voice_transcription` string + a sequence id. Runs in its own `audio_transcription_producer` thread.
- **`transform_imu`** → `trans_imu_features`. From yaw/pitch/roll: `compute_rotation_speed`, `compute_movement_intensity`, `compute_balance_state`, `compute_cat_interaction`, `compute_is_rest`. Yaw is unwrapped first (`unwrap_yaw`).
- **`transform_motor`** → `trans_motor_data`. `compute_motor_vectors` turns left/right PWM into thrust + rotation velocity vectors.

### 4.3 Mart (`mrt_experiences`)
**`build_mrt_experiences`** joins all four transformed modalities into one unified experience table. Each row is one visual frame anchored to the last `N_FRAMES = 12` frames of audio/IMU/motor context. This `mrt_experiences` table is the **single source of truth** the decision layers read from; it also carries `voice_transcription` and the cat-motion/robot-motion deltas. (`mart_play_kpis` summarizes play-interaction stats.)

### 4.4 Decision
Two paths read the mart and both emit into a shared `BRAIN_COMMAND_QUEUE`.

- **Reactive — `mrt_immediate_decisions`** (`decision_engine.py`). `build_immediate_decisions` runs the `RULE_REGISTRY` over each experience row and keeps the highest-priority hit:
  - `rule_safety_stop` (priority 99) → `back_off` on high movement intensity
  - `rule_voice_command` (50) → keyword match on the transcription
  - `rule_cat_greeting` (10) → `get_closer` on cat + meow
  - `rule_cat_gaze` (5) → `center_gaze` on any cat
  Action shapes come from **`decision_definitions`** (`DECISION_DEFINITIONS`: `def_get_closer`, `def_back_off`, `def_play_arm`, `def_center_gaze`, `def_play_audio_bark`), resolved by `get_action_parameters`. Fires every chunk (~3 s).
- **Reflective — `mrt_reflective_decisions`** (`mrt_reflective_decisions.py`). A daemon thread (`reflective_decision_loop`) that every `DEFAULT_INTERVAL = 20` s reads the last `EXPERIENCE_WINDOW = 80` rows of the mart, compresses them to text, and calls **Claude Haiku** (`claude-haiku-4-5-20251001`) for a strategic choice from `VALID_ACTIONS = {get_closer, back_off, play_arm, no_action}`. It adds semantic understanding of natural-language commands the keyword matcher misses, validates the JSON reply, logs every cycle to `recordings/mrt_reflective_log.jsonl`, and degrades gracefully if the API/key is absent.

### 4.5 Execution (`mrt_decisions_to_actions` → `mrt_motor`)
- `build_decisions_to_actions` maps a decision + its parameters into the `mrt_decisions_to_actions` table.
- **`build_mrt_motor`** (`execution_engine.py`) expands each action into a timestamped PWM sequence at `DT = 0.1 s` (10 Hz), using `PWM_CRUISE = 150` and the scale constants `V_SCALE = 0.001`, `R_SCALE = 0.005`. Output is the `mrt_motor` frame table.

### 4.6 Back to Hardware (closing the loop)
The generated 10 Hz `mrt_motor` frames are downsampled to the 4 fps robot loop and pushed onto `BRAIN_COMMAND_QUEUE`. The **`stream_motors`** thread is the arbiter: manual GUI input always wins; otherwise it pops the next brain command; otherwise it idles. It sends `left,right,arm` to the motor ESP32 (port 12345), which relays to the Arduino. Audio actions (`play_audio_*`) take a parallel path: `process_chunk` puts a filename on `AUDIO_PLAYBACK_QUEUE`, and **`audio_playback_consumer`** streams that WAV to the motor ESP32's I²S speaker server on port **12346**. This is the mirror of Chapter 3 — data has become physical action.

---

## 5. Operating the Robot

- **Run:** `python run.py` → Tkinter GUI (`gui_controller.py`). Models preload in a background thread; wait for the green "Ready" status before pressing Start.
- **Manual control:** D-pad + speed/arm sliders send commands via `send_motor`; manual input overrides the brain at all times.
- **Debug vs. Normal mode** (GUI checkbox, default OFF):
  - *Normal* — in-RAM history is capped (mrt/decisions/actions/motor at 200/50/50/200) and every chunk is streamed to `recordings/*.csv` as it's produced; heavy media is skipped; console prints a ~1/min heartbeat. Safe for multi-hour unattended runs.
  - *Debug* — keeps full history in RAM and additionally saves `audio_data.wav`, `stg_visual_data.mp4`, and an annotated GIF; verbose per-chunk logging.
- **Outputs (`recordings/`):** `mrt_experience_data.csv`, `mrt_immediate_decisions.csv`, `mrt_decisions_to_actions.csv`, `mrt_generated_motor.csv`, and `mrt_reflective_log.jsonl` (LLM reasoning trace, every cycle).
- **Dependencies:** `ultralytics` (YOLO), `transformers` (AST), `whisper` / `mlx-whisper` (STT), `torch` (MPS), `anthropic` (Haiku), `python-dotenv` (`.env` holds `ANTHROPIC_API_KEY`), plus `pandas`, `numpy`, `opencv-python`, `pydub`, `scipy`. Runs on Apple Silicon; MPS preferred, CPU fallback automatic.

---

## 6. Limitations & Handoff Notes

- **Camera power/thermal fragility** — the single biggest operational risk. The ESP32-CAM degrades (fast→slow→offline `/capture`) under a weak or warm supply; the video thread fetches with a 1 s timeout and **silently swallows failures**, so a flaky camera produces zero frames and an empty `mrt_experiences` with no error. The pipeline only forms chunks from *visual* frames, so no camera = no experiences = no decisions. Power-cycle to recover.
- **WiFi-dependent** — all four boards and the host must share one network; IPs are hardcoded in `recording_module.py`. Each firmware has the SSID/password baked in, so moving networks means re-flashing.
- **Boards are physically interchangeable-looking** — identify each by its MAC in the esptool output before flashing, and match the Arduino board *profile* to the physical board (WROOM profile for plain boards, AI-Thinker for the camera).
- **Latency floor** — reactive loop ~3 s (chunk size), reflective loop ~20 s + ~1.5 s API round-trip. Not suitable for fast reflexes.
- **Voice rules are keyword-based** at the reactive layer; only the reflective LLM understands free-form phrasing, and only every 20 s.
- **Single-host, single-cat** — perception assumes COCO class 15 and one host machine; no multi-robot or fleet support.

---

*This guide describes V1. The pipeline structure (Staging → Transformation → Mart → Decision → Execution) is the durable contract; individual models and the host hardware are expected to change in V2.*
