#!/usr/bin/env python3
"""
Logots Robot Control — Full Monitoring GUI
Panels: Drive + Arm  |  IMU orientation  |  Audio waveform  |  Video feed
"""

import tkinter as tk
from tkinter import filedialog, messagebox
import atexit
import math
import signal
import sys
import time
import threading
import subprocess
import os
import tempfile
import csv
import queue
import json
import collections
import io
import base64
import struct
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# Base64 color JPEG fields exceed the default 128 KB csv field limit
csv.field_size_limit(min(sys.maxsize, 2**31 - 1))

import numpy as np

import matplotlib
matplotlib.use('TkAgg')
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# ── Optional deps ─────────────────────────────────────────────────────────────
try:
    from smbus2 import SMBus, i2c_msg
    SMBUS2 = True
except ImportError:
    try:
        import smbus
        SMBUS2 = False
    except ImportError:
        SMBUS2 = None

try:
    import sounddevice as sd
    AUDIO_OK = True
except ImportError:
    AUDIO_OK = False


from PIL import Image, ImageTk

try:
    import cv2
    CV2_OK = True
except ImportError:
    CV2_OK = False

# ── Hardware constants ────────────────────────────────────────────────────────
API_PORT      = 8787
ARDUINO_ADDR  = 0x08
ENC_PKT_HDR   = 0xE5   # encoder read-back packet: hdr, int32 left, int32 right, sum8
ENC_PKT_LEN   = 10     # (see requestEvent() in the firmware)
MOT_PKT_LEN   = 13     # the same packet + [mot_seq][mot_state][check] (firmware with motions)
MOT_PKT_SALT  = 0x5A   # check = (mot_seq + mot_state + salt) & 0xFF
# firmware motion states (motion.h); rotate and traverse share them, one motion at a time
MOT_IDLE, MOT_MOVING, MOT_SETTLE, MOT_DONE, MOT_TIMEOUT, MOT_ABORTED = range(6)
IMU_I2C_BUS   = 7
MPU_ADDR      = 0x68
AUDIO_RATE    = 16000
AUDIO_BLOCK   = 512
# Mic gain (2026-11-17), the GAIN slider in AUDIO INPUT. Applied to the live mic in
# AudioReader, so the waveform, the recording, the frame API and the voice listener all
# get the same audio; the frame carries it as mic_gain_db, and the listener scales its
# end-of-speech threshold by it (audio_on_demand.Ears). Saved per machine in SETTINGS_PATH.
MIC_GAIN_DB_MIN = -30
MIC_GAIN_DB_MAX = 20
MIC_HIGHPASS_HZ = 40    # removes the mic's 1-3 Hz drift before the gain (AudioReader)
SETTINGS_PATH   = os.path.expanduser('~/.config/logots/ui_settings.json')

def _load_settings():
    try:
        with open(SETTINGS_PATH) as f: return json.load(f)
    except Exception:
        return {}

def _save_settings(**changes):
    s=_load_settings(); s.update(changes)
    try:
        os.makedirs(os.path.dirname(SETTINGS_PATH),exist_ok=True)
        with open(SETTINGS_PATH,'w') as f: json.dump(s,f,indent=1)
    except Exception as e:
        print(f'[settings] could not save {SETTINGS_PATH}: {e}',flush=True)

# ── Capture loop rate ─────────────────────────────────────────────────────────
# Sensory-capture / recording tick rate. The loop is drift-compensated (it waits
# PERIOD_MS minus however long the tick's work took), so the ACTUAL rate tracks
# TARGET_FPS as long as the per-tick work fits inside PERIOD_MS. 10 FPS (100 ms)
# is sustainable on the Jetson, whose per-tick work is ~45-90 ms.
TARGET_FPS = 10
PERIOD_MS  = int(round(1000 / TARGET_FPS))   # 100 ms per tick

# Camera capture rate. The FIFO reader keeps only the latest frame, so the camera
# only needs to feed the ~10 FPS loop — running it at the sensor's full 30 FPS just
# burns ISP + videoconvert + FIFO bandwidth on frames that get discarded. A little
# headroom above TARGET_FPS keeps the displayed frame fresh. Drop to match if you
# want maximum savings.
CAMERA_FPS = 15

PWR_MGMT_1 = 0x6B;  CONFIG_REG = 0x1A
GYRO_CFG   = 0x1B;  ACCEL_CFG  = 0x1C;  ACCEL_CFG2 = 0x1D
ACCEL_OUT  = 0x3B;  GYRO_OUT   = 0x43
ACCEL_SCALE = 16384.0;  GYRO_SCALE = 131.0

# ── Position estimation ───────────────────────────────────────────────────────
# Primary model (2026-10-03): per-tick forward distance comes from real encoder counts
# (JGA25-370 Hall encoders), not a PWM model — see PositionEstimator below. Direction still
# comes from the IMU yaw (no encoder-based heading — the two wheels' counts-per-rev already
# differ by ~2%, not reliable enough for differential heading on top of the IMU we already
# have). counts-per-rev and wheel diameter are from the 2026-10-03 bench test (5 hand-turns
# per wheel; wheel diameter measured at 2.78in outer).
LEFT_COUNTS_PER_REV  = 3286
RIGHT_COUNTS_PER_REV = 3208
WHEEL_DIAMETER_M      = 2.78 * 0.0254   # = 0.0706 m
WHEEL_CIRCUMFERENCE_M = math.pi * WHEEL_DIAMETER_M

# Fallback model, used only on a tick where the encoder read-back fails (see
# PositionEstimator.update()) — forward speed modelled as proportional to commanded average
# PWM. ROBOT_MAX_SPEED_MPS is still an uncalibrated GUESS; fine for the rare fallback tick,
# not a substitute for the encoder path above.
ROBOT_MAX_SPEED_MPS = 0.30            # forward speed at |PWM|=255 — GUESS, calibrate
K_V = ROBOT_MAX_SPEED_MPS / 255.0     # m/s per PWM unit

# Gyro-assisted straight-line trim (2026-10-03): live floor tests found the robot curves
# ~5-6°/s even at equal left_pwm=right_pwm, despite the two motors measuring roughly
# speed-matched on the (unloaded, wheels-raised) bench test — a real floor-load asymmetry.
# _send_motors() corrects this in software: whenever the commanded PWM is equal and
# nonzero, it locks the IMU yaw at that moment and trims one wheel proportionally to the
# yaw error each tick, on the wire only (self.left_pwm/right_pwm — the commanded intent used
# by PositionEstimator's PWM fallback, the CSV and the frame API — are untouched). The sign
# below (left wheel reduced / right increased for positive error) is a first guess from the
# 2026-10-03 curving direction (right wheel's path measured longer than left's — see
# CLAUDE.md) — if a straight-drive test still curves, or curves the other way, flip the sign
# in _straight_trim().
STRAIGHT_KP       = 3.0   # PWM counts of trim per degree of yaw error. Live-tested 2026-10-03:
                           # KP=3 cut drift from ~5.6°/s (uncorrected) to ~1.4°/s, reproduced
                           # twice. KP=6 was tried to tighten it further and instead went
                           # unstable (133° drift in 4s, worse than uncorrected) — reverted.
                           # Don't raise this without adding tick-by-tick error/trim logging
                           # first to see the oscillation (the IMU may also be noisier while
                           # actually driving — motor electrical noise/vibration — than at rest,
                           # which a higher P gain would amplify).
STRAIGHT_KI       = 0.4   # PWM counts of trim per (degree·second) of accumulated error —
                           # added 2026-10-03 to kill the P-only controller's steady-state
                           # residual (observed: robot visibly not quite straight, right wheel
                           # slightly ahead) without raising KP again.
STRAIGHT_I_MAX    = 40     # anti-windup clamp on the accumulated integral's contribution
STRAIGHT_MAX_TRIM = 60    # clamp, so a large transient error can't stall/reverse a wheel

# Motion commands (2026-11-10): traverse(cm), rotate(degrees), servo(pan, tilt). Each is
# sent once and the robot carries it out by itself. For the two wheel motions the Arduino
# runs the wheels to an encoder-count target and stops (motion.h); Mover below turns
# cm/degrees into counts and uses the IMU for what the encoders can't see: the real angle
# of a rotation (correction passes) and the heading during a traverse (steering).
# Every number here is sent to the Arduino with each command, so tuning never needs a
# reflash. NOT YET RUN ON THE ROBOT: all values marked GUESS are starting points.
# + degrees = left = IMU yaw going up. + cm = forward.
SPIN_COUNTS_PER_DEG = 20.0  # encoder counts of (right-left)/2 per degree of body rotation.
                            # GUESS (assumes ~16 cm between the wheels). Mover learns the
                            # real value from the IMU and prints it: copy it here.
SPIN_DIR          = +1      # flip to -1 if a rotation aborts with "turned the wrong way"
ROT_PWM_FAST      = 150     # PWM far from the target
ROT_PWM_SLOW      = 90      # PWM near the target; the firmware raises it by itself if the
                            # wheels don't move (stiction)
ROT_RAMP_DEG      = 20      # slow down from FAST to SLOW over the last this-many degrees
ROT_BALANCE_K     = 0.5     # PWM of trim per count of difference between the two wheels,
                            # keeps the robot pivoting in place
ROT_TOL_DEG       = 1.5     # IMU error accepted without another correction pass
ROT_MAX_PASSES    = 4       # first pass + corrections
ROT_SETTLE_S      = 0.3     # yaw (rotate) / wheels (traverse) must be still this long
ROT_STILL_DEG     = 0.3     # "still" = yaw changed less than this in one tick
ROT_WRONG_WAY_DEG = 8       # turned this far against the command -> abort (SPIN_DIR wrong)
ROT_TIMEOUT_S     = 20      # a whole rotate request, all passes
ROT_KEY_STEP_DEG  = 15      # one A / D key press
TRAV_KEY_STEP_CM  = 10      # one W / S key press

# Traverse: distance comes from the encoders alone (bench-measured constants above, ~2% on
# the 2026-10-03 tape test), so there is no guess in the cm -> counts conversion.
TRAV_COUNTS_PER_CM = ((LEFT_COUNTS_PER_REV + RIGHT_COUNTS_PER_REV) / 2.0
                      / (WHEEL_CIRCUMFERENCE_M * 100.0))      # ~146 counts of (left+right)/2
TRAV_PWM_FAST     = 150     # ~0.165 m/s on the floor (2026-10-03)
TRAV_PWM_SLOW     = 90
TRAV_RAMP_CM      = 10      # slow down over the last this-many cm
TRAV_BALANCE_K    = 0.5     # PWM of trim per count the wheels are off the wanted difference
TRAV_STEER_K      = 0.15    # GUESS. Each tick the wanted wheel difference moves by this
                            # fraction of the IMU heading error. Higher = straighter but can
                            # weave (the 2026-10-03 lesson with STRAIGHT_KP); 0 = wheels
                            # simply matched, no IMU.
TRAV_STEER_MAX_DEG = 30     # the IMU may bend the wheel difference by at most this much
TRAV_TOL_CM       = 1.0     # further off than this at the end is reported as a problem
TRAV_MAX_CM       = 200     # one traverse is clamped to this: there is no obstacle detection

# Servo command: pan as everywhere else (0 = front, + = left); tilt signed, 0 = level, + = up.
# The wire, the tilt slider, the CSV and the frame API keep the raw tilt servo angle.
TILT_LEVEL_DEG    = 90      # raw tilt servo angle with the camera level. GUESS (the start pose)
TILT_DIR          = +1      # GUESS: +1 if a larger raw angle tilts the camera UP, else -1
SERVO_PAN_MAX     = 45      # same cap as the GUI pan slider
SERVO_TILT_MAX    = 90
SERVO_MS_PER_DEG  = 2       # wait this long per degree of travel before reporting (the
                            # firmware steps 1 degree per ms; this leaves margin)

# ── Palette ───────────────────────────────────────────────────────────────────
C_BG      = '#1e1e2e'
C_PANEL   = '#2a2a3e'
C_BORDER  = '#3a3a5a'
C_TEXT    = '#e0e0f0'
C_SUB     = '#7070a0'
C_BLUE    = '#1a7abf'
C_BLUE_HI = '#3498db'
C_GREEN   = '#27ae60'
C_RED     = '#c0392b'
C_AMBER   = '#e67e22'
# macOS draws tk.Button natively (white) and ignores bg, so light text is unreadable
# there; on the Jetson (X11) the dark bg shows and the light text is right.
C_BTN_TEXT = '#000000' if sys.platform == 'darwin' else C_TEXT

# ── Madgwick AHRS ─────────────────────────────────────────────────────────────
class Madgwick:
    def __init__(self, beta=0.033):
        self.beta = beta
        self.q    = np.array([1.0, 0.0, 0.0, 0.0])

    def update_imu(self, gx, gy, gz, ax, ay, az, dt):
        q = self.q
        gx=math.radians(gx); gy=math.radians(gy); gz=math.radians(gz)
        norm=math.sqrt(ax*ax+ay*ay+az*az)
        if norm==0: return
        ax/=norm; ay/=norm; az/=norm
        _2q0=2*q[0];_2q1=2*q[1];_2q2=2*q[2];_2q3=2*q[3]
        _4q0=4*q[0];_4q1=4*q[1];_4q2=4*q[2];_8q1=8*q[1];_8q2=8*q[2]
        q0q0=q[0]*q[0];q1q1=q[1]*q[1];q2q2=q[2]*q[2];q3q3=q[3]*q[3]
        s0=_4q0*q2q2+_2q2*ax+_4q0*q1q1-_2q1*ay
        s1=_4q1*q3q3-_2q3*ax+4*q0q0*q[1]-_2q0*ay-_4q1+_8q1*q1q1+_8q1*q2q2+_4q1*az
        s2=4*q0q0*q[2]+_2q0*ax+_4q2*q3q3-_2q3*ay-_4q2+_8q2*q1q1+_8q2*q2q2+_4q2*az
        s3=4*q1q1*q[3]-_2q1*ax+4*q2q2*q[3]-_2q2*ay
        norm=math.sqrt(s0*s0+s1*s1+s2*s2+s3*s3)
        if norm>0: s0/=norm;s1/=norm;s2/=norm;s3/=norm
        qD0=0.5*(-q[1]*gx-q[2]*gy-q[3]*gz)-self.beta*s0
        qD1=0.5*( q[0]*gx+q[2]*gz-q[3]*gy)-self.beta*s1
        qD2=0.5*( q[0]*gy-q[1]*gz+q[3]*gx)-self.beta*s2
        qD3=0.5*( q[0]*gz+q[1]*gy-q[2]*gx)-self.beta*s3
        q[0]+=qD0*dt;q[1]+=qD1*dt;q[2]+=qD2*dt;q[3]+=qD3*dt
        self.q=q/math.sqrt(sum(x*x for x in q))

    def get_yaw_pitch_roll(self):
        q=self.q
        yaw  =math.degrees(math.atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3])))
        pitch=math.degrees(math.asin(max(-1,min(1,2*(q[0]*q[2]-q[3]*q[1])))))
        roll =math.degrees(math.atan2(2*(q[0]*q[1]+q[2]*q[3]),1-2*(q[1]*q[1]+q[2]*q[2])))
        return yaw, pitch, roll

# ── IMU reader thread ─────────────────────────────────────────────────────────
class IMUReader(threading.Thread):
    CAL_N = 500

    def __init__(self):
        super().__init__(daemon=True)
        self.roll=self.pitch=self.yaw=0.0
        self.status      = 'starting'
        self.cal_prog    = 0
        self._stop       = threading.Event()
        self._bus        = None
        self._sample_buf  = []
        self._sample_lock = threading.Lock()

    def stop(self): self._stop.set()

    def drain_samples(self):
        with self._sample_lock:
            out, self._sample_buf = self._sample_buf, []
        return out

    def run(self):
        try:
            self._init_hw()
            self._calibrate()
            self.status = 'running'
            filt=Madgwick(); t_prev=time.time()
            while not self._stop.is_set():
                ax,ay,az=self._accel()
                gx,gy,gz=self._gyro()
                gx-=self._bg[0]; gy-=self._bg[1]; gz-=self._bg[2]
                t=time.time(); dt=t-t_prev; t_prev=t
                filt.update_imu(gx,gy,gz,ax,ay,az,dt)
                yaw,pitch,roll=filt.get_yaw_pitch_roll()
                yaw-=self._yo; pitch-=self._po; roll-=self._ro
                yaw=math.fmod(yaw+360,360)
                if pitch>180: pitch-=360
                if pitch<-180: pitch+=360
                if roll>180: roll-=360
                if roll<-180: roll+=360
                self.yaw=yaw; self.pitch=pitch; self.roll=roll
                with self._sample_lock:
                    self._sample_buf.append((yaw, pitch, roll))
                time.sleep(0.005)   # cap ~200Hz (2026-10-03) — this loop had no rate limit at
                # all, spinning as fast as the I2C bus allowed (likely 1000s of Hz) and
                # pegging a CPU core continuously; under the GIL that starves the main GUI
                # thread of scheduling time, which is why the capture tick's actual FPS was
                # sitting well under its 10Hz target even at rest. 200Hz is still far faster
                # than anything downstream needs (the 10Hz capture tick just drains whatever
                # accumulated; the Madgwick filter integrates fine at this rate).
        except Exception as e:
            self.status=f'error: {e}'

    def _init_hw(self):
        b = SMBus(IMU_I2C_BUS) if SMBUS2 else smbus.SMBus(IMU_I2C_BUS)
        self._bus=b
        b.write_byte_data(MPU_ADDR,PWR_MGMT_1,0x00); time.sleep(0.1)
        b.write_byte_data(MPU_ADDR,PWR_MGMT_1,0x01)
        b.write_byte_data(MPU_ADDR,CONFIG_REG, 0x02)
        b.write_byte_data(MPU_ADDR,ACCEL_CFG,  0x00)
        b.write_byte_data(MPU_ADDR,ACCEL_CFG2, 0x04)
        b.write_byte_data(MPU_ADDR,GYRO_CFG,   0x00)
        time.sleep(0.1)

    def _calibrate(self):
        self.status='calibrating'
        N=self.CAL_N
        sx=sy=sz=0.0
        for i in range(N):
            gx,gy,gz=self._gyro(); sx+=gx; sy+=gy; sz+=gz
            self.cal_prog=int(i/N*50)
        self._bg=(sx/N,sy/N,sz/N)
        filt=Madgwick(); t_prev=time.time()
        sy2=sp2=sr2=0.0
        for i in range(N):
            ax,ay,az=self._accel(); gx,gy,gz=self._gyro()
            gx-=self._bg[0]; gy-=self._bg[1]; gz-=self._bg[2]
            t=time.time(); filt.update_imu(gx,gy,gz,ax,ay,az,t-t_prev); t_prev=t
            y,p,r=filt.get_yaw_pitch_roll(); sy2+=y; sp2+=p; sr2+=r
            self.cal_prog=50+int(i/N*50)
        self._yo=sy2/N; self._po=sp2/N; self._ro=sr2/N

    def _s16(self,hi,lo):
        v=(hi<<8)|lo; return v-65536 if v>32767 else v

    def _accel(self):
        d=self._bus.read_i2c_block_data(MPU_ADDR,ACCEL_OUT,6)
        return (self._s16(d[0],d[1])/ACCEL_SCALE,
                self._s16(d[2],d[3])/ACCEL_SCALE,
                self._s16(d[4],d[5])/ACCEL_SCALE)

    def _gyro(self):
        d=self._bus.read_i2c_block_data(MPU_ADDR,GYRO_OUT,6)
        return (self._s16(d[0],d[1])/GYRO_SCALE,
                self._s16(d[2],d[3])/GYRO_SCALE,
                self._s16(d[4],d[5])/GYRO_SCALE)

# ── Position estimator ────────────────────────────────────────────────────────
class PositionEstimator:
    """Dead-reckons body (x, y) in metres from wheel encoders + IMU heading.

    Forward distance per tick is the average of each wheel's measured displacement
    (encoder count delta / that wheel's counts-per-rev · wheel circumference); heading θ
    comes from the IMU yaw, not the encoders (the two wheels' counts-per-rev already differ
    by ~2%, not precise enough for differential heading on top of the IMU we already have).
    Integrated each tick: x += d·cosθ, y += d·sinθ. Pure spins (L=+, R=−) average to ~0 net
    wheel displacement, so heading changes without translation.

    Falls back to the old PWM-speed model (v = k_v·(left_pwm+right_pwm)/2, integrated over
    dt) for any tick where the encoder read-back fails (enc_left/enc_right is None) — e.g. a
    bad I2C read. Once readings resume, the encoder delta is computed against the last good
    count, so no distance is lost across a brief gap; only that one tick's distance comes
    from the (less accurate) PWM model instead of being skipped outright."""

    def __init__(self, k_v=K_V,
                 left_counts_per_rev=LEFT_COUNTS_PER_REV,
                 right_counts_per_rev=RIGHT_COUNTS_PER_REV,
                 wheel_circumference_m=WHEEL_CIRCUMFERENCE_M):
        self.k_v = k_v
        self.left_counts_per_rev    = left_counts_per_rev
        self.right_counts_per_rev   = right_counts_per_rev
        self.wheel_circumference_m  = wheel_circumference_m
        self.reset()

    def reset(self):
        self.x = self.y = self.heading = 0.0
        self._t = None
        self._last_enc_left = self._last_enc_right = None

    def update(self, enc_left, enc_right, left_pwm, right_pwm, yaw_deg, now=None):
        now = now if now is not None else time.monotonic()
        have_enc = enc_left is not None and enc_right is not None
        if self._t is None:                 # first sample: seed time + heading only
            self._t = now; self.heading = yaw_deg
            if have_enc: self._last_enc_left, self._last_enc_right = enc_left, enc_right
            return
        dt = now - self._t; self._t = now
        if dt <= 0 or dt > 0.5: return      # guard stalls / first big gap
        if have_enc and self._last_enc_left is not None:
            dist_l = ((enc_left  - self._last_enc_left)  / self.left_counts_per_rev
                       * self.wheel_circumference_m)
            dist_r = ((enc_right - self._last_enc_right) / self.right_counts_per_rev
                       * self.wheel_circumference_m)
            forward = (dist_l + dist_r) / 2.0
        else:                                # encoder read failed this tick: PWM fallback
            forward = self.k_v * (left_pwm + right_pwm) / 2.0 * dt
        if have_enc:
            self._last_enc_left, self._last_enc_right = enc_left, enc_right
        th = math.radians(yaw_deg)
        self.x += forward * math.cos(th)
        self.y += forward * math.sin(th)
        self.heading = yaw_deg

# ── Motion commands: traverse / rotate / servo ────────────────────────────────
def _wrap180(a):
    return (a + 180) % 360 - 180

def tilt_to_raw(tilt):
    """Signed tilt (0 = level, + = up) -> raw tilt servo angle 0..180."""
    return int(max(0, min(180, round(TILT_LEVEL_DEG + TILT_DIR * tilt))))

def tilt_from_raw(raw):
    return int(round((raw - TILT_LEVEL_DEG) * TILT_DIR))

class Mover:
    """One wheel motion at a time, rotate(degrees) or traverse(cm), stepped once per real
    tick. The request is made once; this carries it through and sets job['event'].

    Rotate, per pass: degrees still to turn -> encoder counts -> one "R,..." message to the
    Arduino, resent every tick until the read-back shows its seq. The Arduino spins to that
    count and stops (motion.h). Once the yaw is still, the turn so far is read off the IMU
    (wheels scrub when spinning, so counts alone are a few percent off) and, if it is more
    than ROT_TOL_DEG from the request, another pass covers the difference. Each large pass
    also measures the real counts-per-degree, which replaces the SPIN_COUNTS_PER_DEG guess
    for the rest of the session.

    Traverse: cm -> counts -> one "T,..." message. The Arduino drives to the count with the
    two wheels matched. While it runs, every tick sends "S,..." = how far apart the wheels
    should be, moved against the IMU heading error, so the heading at the start is held even
    if matched wheels would curve. One pass: for distance the encoders are the reference."""

    def __init__(self, gui):
        self.gui = gui
        self.job = None
        self.last = None             # result of the last finished request
        self.counts_per_deg = SPIN_COUNTS_PER_DEG
        self._seq = None
        self._lock = threading.Lock()

    @property
    def active(self):
        return self.job is not None

    def request(self, kind, amount, source='manual'):
        """kind 'rotate' (degrees, + = left) or 'traverse' (cm, + = forward). Returns the
        job (wait on job['event'], then read job['result']) or None if one is already
        running. Any thread."""
        with self._lock:
            if self.job is not None:
                return None
            self.job = {'kind': kind, 'requested': float(amount), 'source': source,
                        'turned': 0.0, 'moved': 0.0, 'yaw_prev': None, 'phase': 'start',
                        'passes': 0, 'nominal': (0, 0), 't0': time.monotonic(),
                        'timeout': (ROT_TIMEOUT_S if kind == 'rotate'
                                    else 10 + abs(amount) * 0.3),
                        'cancel': None, 'result': None, 'event': threading.Event()}
            return self.job

    def cancel(self, reason='stopped'):
        job = self.job
        if job is not None and not job['cancel']:
            job['cancel'] = reason

    def abort(self, reason):
        """End now without telling the Arduino (it is gone, or its watchdog will stop it)."""
        if self.job is not None:
            self._finish(False, reason)

    def wire_message(self):
        """The motion message to send this tick in place of the drive message, or None."""
        job = self.job
        if job is None:
            return None
        if job['phase'] == 'run' and job['kind'] == 'traverse':
            return f"S,{job['seq']},{int(round(job['steer']))}\n".encode()
        if job['phase'] not in ('send', 'cancel'):
            return None
        letter = 'R' if job['kind'] == 'rotate' else 'T'
        if job['counts'] == 0:
            return f"{letter},{job['seq']},0,0,0,0,0\n".encode()
        if job['kind'] == 'rotate':
            fast, slow, kb = ROT_PWM_FAST, ROT_PWM_SLOW, ROT_BALANCE_K
            ramp = ROT_RAMP_DEG * self.counts_per_deg
        else:
            fast, slow, kb = TRAV_PWM_FAST, TRAV_PWM_SLOW, TRAV_BALANCE_K
            ramp = TRAV_RAMP_CM * TRAV_COUNTS_PER_CM
        return (f"{letter},{job['seq']},{job['counts']},{fast},{slow},"
                f"{int(round(ramp))},{int(round(kb * 100))}\n").encode()

    def step(self, now=None):
        job = self.job
        if job is None:
            return
        g = self.gui
        now = time.monotonic() if now is None else now
        d = 0.0
        imu = g.imu_reader
        if imu is not None and imu.status == 'running':
            yaw = imu.yaw
            if job['yaw_prev'] is not None:
                d = _wrap180(yaw - job['yaw_prev'])
                job['turned'] += d
            job['yaw_prev'] = yaw
        else:
            job['yaw_prev'] = None
        job['nominal'] = (0, 0)
        trav = job['kind'] == 'traverse'
        if trav and job['phase'] != 'start':
            moved = self._moved_cm()
            if moved is not None:
                job['moved'] = moved
        ph = job['phase']
        if ph == 'start':
            if not (g.connected and g.i2c):
                return self._finish(False, 'Arduino not connected')
            if not g.fw_motion:
                return self._finish(False, 'Arduino needs the new firmware (reflash)')
            if job['cancel']:
                return self._finish(False, job['cancel'])
            if trav:
                return self._begin_traverse(now)
            return self._begin_pass(job['requested'], now)
        if not g.connected:
            return self._finish(False, 'Arduino disconnected')
        if ph != 'cancel' and (job['cancel'] or now - job['t0'] > job['timeout']):
            job['cancel'] = job['cancel'] or f"timed out after {job['timeout']:.0f} s"
            job.update(seq=self._next_seq(), counts=0, phase='cancel', t_phase=now)
            return
        if ph == 'cancel':
            if g.fw_mot_seq == job['seq'] or now - job['t_phase'] > 1.0:
                self._finish(False, job['cancel'])
        elif ph == 'send':
            if g.fw_mot_seq == job['seq']:
                job['phase'] = 'run'
        elif ph == 'run':
            st = g.fw_mot_state if g.fw_mot_seq == job['seq'] else None
            if st == MOT_MOVING:
                if trav:
                    job['nominal'] = (job['dir'] * TRAV_PWM_FAST, job['dir'] * TRAV_PWM_FAST)
                else:
                    job['nominal'] = (-job['dir'] * ROT_PWM_FAST, job['dir'] * ROT_PWM_FAST)
            elif st in (MOT_DONE, MOT_TIMEOUT, MOT_ABORTED):
                job.update(fw_end=st, phase='settle', t_phase=now, t_still=now,
                           enc_prev=(g.enc_left, g.enc_right))
            if trav and job['yaw_prev'] is not None:
                # Turned left of the starting heading (turned > 0): ask the wheels for a
                # difference that turns right, and keep adding while the error lasts.
                lim = TRAV_STEER_MAX_DEG * self.counts_per_deg
                job['steer'] = max(-lim, min(lim, job['steer'] - TRAV_STEER_K * job['turned']
                                             * self.counts_per_deg * SPIN_DIR))
        elif ph == 'settle':
            if trav:
                enc = (g.enc_left, g.enc_right)
                if enc != job['enc_prev']:
                    job['enc_prev'] = enc; job['t_still'] = now
            elif abs(d) > ROT_STILL_DEG:
                job['t_still'] = now
            if now - job['t_still'] >= ROT_SETTLE_S or now - job['t_phase'] > 2.0:
                self._end_traverse() if trav else self._evaluate(now)

    def _enc_progress(self):
        g = self.gui
        if g.enc_left is None or g.enc_right is None:
            return None
        return (g.enc_right - g.enc_left) / 2.0

    def _moved_cm(self):
        """Distance since the traverse began, from the encoders (same maths as
        PositionEstimator: each wheel's own counts-per-rev, averaged)."""
        g = self.gui; job = self.job
        if g.enc_left is None or g.enc_right is None or job.get('enc0') is None:
            return None
        l = (g.enc_left  - job['enc0'][0]) / LEFT_COUNTS_PER_REV
        r = (g.enc_right - job['enc0'][1]) / RIGHT_COUNTS_PER_REV
        return (l + r) / 2.0 * WHEEL_CIRCUMFERENCE_M * 100.0

    def _next_seq(self):
        fw = self.gui.fw_mot_seq
        base = self._seq if self._seq is not None else (fw or 0)
        new = base % 255 + 1                 # 1..255
        if new == fw:                        # must differ from the one the Arduino holds
            new = new % 255 + 1
        self._seq = new
        return new

    def _begin_traverse(self, now):
        job = self.job; g = self.gui
        counts = int(round(job['requested'] * TRAV_COUNTS_PER_CM))
        if counts == 0:
            return self._finish(True, '')
        if g.enc_left is None or g.enc_right is None:
            return self._finish(False, 'no encoder reading from the Arduino')
        job['passes'] = 1
        job.update(seq=self._next_seq(), counts=counts, dir=(1 if counts > 0 else -1),
                   enc0=(g.enc_left, g.enc_right), steer=0.0, turned=0.0,
                   phase='send', t_phase=now)

    def _end_traverse(self):
        job = self.job
        off = job['requested'] - job['moved']
        if job['fw_end'] == MOT_TIMEOUT:
            self._finish(False, 'Arduino gave up: wheels not turning?')
        elif job['fw_end'] == MOT_ABORTED:
            self._finish(False, 'Arduino stopped it (lost contact with the Jetson)')
        elif abs(off) > TRAV_TOL_CM:
            self._finish(False, f'stopped {off:+.1f} cm from the target')
        else:
            self._finish(True, '' if job['yaw_prev'] is not None
                         else 'no IMU: wheels matched, heading not checked')

    def _begin_pass(self, rem_deg, now):
        job = self.job
        counts = int(round(rem_deg * self.counts_per_deg)) * SPIN_DIR
        if counts == 0:
            return self._finish(True, '')
        job['passes'] += 1
        job.update(seq=self._next_seq(), counts=counts, dir=(1 if rem_deg > 0 else -1),
                   pass_req=rem_deg, pass_turned0=job['turned'],
                   pass_enc0=self._enc_progress(), phase='send', t_phase=now)

    def _evaluate(self, now):
        job = self.job
        enc0, enc1 = job['pass_enc0'], self._enc_progress()
        left_counts = (enc1 - enc0) * SPIN_DIR if enc0 is not None and enc1 is not None else None
        if job['yaw_prev'] is None:          # no IMU: the wheel encoders are all there is
            if left_counts is not None:
                job['turned'] += left_counts / self.counts_per_deg
            return self._finish(left_counts is not None,
                                'no IMU: angle from the wheel encoders only')
        pass_turn = job['turned'] - job['pass_turned0']
        if abs(pass_turn) >= ROT_WRONG_WAY_DEG and pass_turn * job['pass_req'] < 0:
            return self._finish(False, 'turned the wrong way: flip SPIN_DIR in logots_ui.py')
        if left_counts is not None and abs(pass_turn) >= 10 and left_counts * pass_turn > 0:
            measured = abs(left_counts) / abs(pass_turn)
            self.counts_per_deg = max(0.5 * SPIN_COUNTS_PER_DEG, min(2.0 * SPIN_COUNTS_PER_DEG,
                                      0.5 * self.counts_per_deg + 0.5 * measured))
            print(f"[rotate] pass {job['passes']}: asked {job['pass_req']:+.1f}°, IMU says "
                  f"{pass_turn:+.1f}° for {left_counts:+.0f} counts = {measured:.1f} counts/deg "
                  f"(now using {self.counts_per_deg:.1f}; SPIN_COUNTS_PER_DEG is "
                  f"{SPIN_COUNTS_PER_DEG})", flush=True)
        rem = job['requested'] - job['turned']
        if abs(rem) <= ROT_TOL_DEG:
            self._finish(True, '')
        elif job['fw_end'] == MOT_TIMEOUT:
            self._finish(False, 'Arduino gave up: wheels not turning?')
        elif job['passes'] >= ROT_MAX_PASSES:
            self._finish(False, f"still {rem:+.1f}° off after {job['passes']} passes")
        else:
            self._begin_pass(rem, now)

    def _finish(self, ok, reason):
        job = self.job
        res = {'kind': job['kind'], 'requested': job['requested'],
               'turned': round(job['turned'], 2), 'moved': round(job['moved'], 2),
               'passes': job['passes'], 'ok': ok, 'reason': reason, 't': time.monotonic()}
        job['result'] = res
        self.last = res
        self.job = None
        job['event'].set()
        if job['kind'] == 'rotate':
            did = f"turned {res['turned']:+.1f}° in {job['passes']} pass(es)"
            asked = f"{job['requested']:+.0f}°"
        else:
            did = f"moved {res['moved']:+.1f} cm, heading changed {res['turned']:+.1f}°"
            asked = f"{job['requested']:+.0f} cm"
        print(f"[{job['kind']}] {job['source']} {asked}: {did}"
              + (f" ({reason})" if reason else ''), flush=True)

# ── Audio reader ──────────────────────────────────────────────────────────────
class AudioReader:
    def __init__(self):
        self._stream   = None
        self.status    = 'stopped'
        self._drain_buf  = []
        self._drain_lock = threading.Lock()
        self.gain_db     = 0
        self._gain       = 1.0
        # High-pass BEFORE the gain: the I2S mic's baseline drifts at ~1-3 Hz (see
        # audio_on_demand.VAD_HIGHPASS_HZ). Measured 2026-11-17 on the robot's July
        # recordings: 98% of the energy is below 40 Hz, and at +20 dB (the slider's max) the
        # drift alone peaks at 0.24-0.30 of full scale without the high-pass, 0.07-0.15 with it.
        # 40 Hz is below speech.
        try:
            from scipy.signal import butter, sosfilt
            self._hp=butter(2,MIC_HIGHPASS_HZ,'hp',fs=AUDIO_RATE,output='sos')
            self._sosfilt=sosfilt; self._hp_zi=None
        except Exception as e:      # scipy comes with openwakeword; without it, no high-pass
            print(f'[audio] no high-pass ({e})',flush=True); self._hp=None

    def set_gain_db(self,db):
        self.gain_db=db; self._gain=10.0**(db/20.0)

    def _process(self,x):
        """Live mic block (float32, ±1) -> high-pass, gain, clip."""
        if self._hp is not None:
            if self._hp_zi is None:          # start in steady state at the first sample's level
                from scipy.signal import sosfilt_zi
                self._hp_zi=sosfilt_zi(self._hp)*float(x[0]) if len(x) else None
            if self._hp_zi is not None:
                x,self._hp_zi=self._sosfilt(self._hp,x,zi=self._hp_zi)
        if self._gain!=1.0:
            x=x*self._gain
        return np.clip(x,-1.0,1.0)   # past ±1 the listener's int16 conversion would wrap

    def start(self):
        if not AUDIO_OK:
            self.status='unavailable'; return
        try:
            self._stream=sd.InputStream(samplerate=AUDIO_RATE,channels=1,
                                         blocksize=AUDIO_BLOCK,callback=self._cb)
            self._stream.start(); self.status='running'
        except Exception as e:
            self.status=f'error: {e}'

    def _cb(self,indata,frames,t,status):
        x=self._process(indata[:, 0].astype(np.float64))
        with self._drain_lock:
            self._drain_buf.extend(x.tolist())

    def drain_samples(self):
        with self._drain_lock:
            out, self._drain_buf = self._drain_buf, []
        return out

    def stop(self):
        if self._stream:
            try: self._stream.stop(); self._stream.close()
            except: pass

# ── Action reader (executes functions-layer decisions from :8788) ──────────────
class ActionReader(threading.Thread):
    """Executes the decisions published on :8788 (GET /latest_action, src/actions.py) and
    answers each with one POST /action_done, the contract functions.py's wait_for_done()
    blocks on. The motion commands (2026-11-10) are sent ONCE and the robot carries them
    out by itself:

        traverse  {"cm": -30}              drive straight, + = forward      (Mover)
        rotate    {"degrees": -45}         turn on the spot, + = left       (Mover)
        servo     {"pan": 45, "tilt": 10}  camera: pan + = left, tilt + = up, 0/0 = ahead

    approach_plant (left_pwm/right_pwm/duration_s) is the older PWM-for-a-time action, kept
    so today's functions.py keeps working until it switches to the three above.
    The report always carries pos_x/pos_y/heading, plus moved_cm / turned_deg / pan+tilt,
    and "problem" when the command was limited, cancelled or failed. While a wheel motion
    runs the GUI shows a self-driving badge and ignores its controls; SPACE / STOP cancel
    it, and it still reports. Full spec: docs/v2_architecture/motion_commands.md."""

    POLL_S = 0.2

    def __init__(self, gui, host='localhost', port=8788):
        super().__init__(daemon=True)
        self.gui   = gui
        self.host  = host
        self.port  = port
        self.get_url  = f'http://{host}:{port}/latest_action'
        self.post_url = f'http://{host}:{port}/action_done'
        self._stop_evt = threading.Event()
        self._seen_id  = None

    def stop(self):
        self._stop_evt.set()

    def run(self):
        import urllib.request
        while not self._stop_evt.is_set():
            time.sleep(self.POLL_S)
            try:
                with urllib.request.urlopen(self.get_url, timeout=1.0) as resp:
                    action = json.loads(resp.read())
            except Exception:
                continue  # functions.py not running yet, or no action published yet
            aid = action.get('action_id')
            if aid is None or aid == self._seen_id:
                continue
            self._seen_id = aid
            if self.gui.sim_mode:
                continue
            name = action.get('action')
            if name == 'approach_plant':
                self._drive(action)
            elif name == 'rotate':
                self._move(action, 'rotate', 'degrees', 180.0, 'turned_deg', 'turned')
            elif name == 'traverse':
                self._move(action, 'traverse', 'cm', float(TRAV_MAX_CM), 'moved_cm', 'moved')
            elif name == 'servo':
                self._servo(action)

    def _wait_free(self):
        """A manual rotation step may still be running; let it end first."""
        while self.gui._mover.active and not self._stop_evt.is_set():
            time.sleep(0.1)

    def _report(self, aid, name, **extra):
        import urllib.request
        pos = self.gui._pos_est
        report = {'action_id': aid, 'pos_x': pos.x, 'pos_y': pos.y, 'heading': pos.heading,
                  **extra}
        try:
            req = urllib.request.Request(self.post_url, data=json.dumps(report).encode(),
                                         headers={'Content-Type': 'application/json'}, method='POST')
            urllib.request.urlopen(req, timeout=1.0).close()
            print(f"[actions] {name} #{aid} done, reported pos "
                  f"({pos.x:.2f}, {pos.y:.2f}) heading {pos.heading:.0f}°", flush=True)
        except Exception as e:
            print(f"[actions] failed to POST /action_done for #{aid}: {e}", flush=True)

    def _move(self, action, kind, arg, limit, report_key, result_key):
        args = action.get('args', {})
        try:
            asked = float(args[arg])
        except (KeyError, TypeError, ValueError) as e:
            print(f"[actions] bad {kind} args {args}: {e}", flush=True)
            return
        amount = max(-limit, min(limit, asked))
        aid = action['action_id']
        print(f"[actions] {kind} #{aid} {amount:+.0f}", flush=True)
        self._wait_free()
        job = self.gui._mover.request(kind, amount, 'llm')
        res = {}
        if job is not None:
            job['event'].wait(job['timeout'] + 5)
            res = job['result'] or {}
        extra = {report_key: res.get(result_key, 0.0)}
        problems = [res.get('reason') or ('' if res else f'{kind} did not run')]
        if amount != asked:
            problems.append(f'{arg} limited to ±{limit:.0f}')
        if any(problems):
            extra['problem'] = '; '.join(x for x in problems if x)
        self._report(aid, kind, **extra)

    def _servo(self, action):
        args = action.get('args', {})
        try:
            pan_asked, tilt_asked = float(args['pan']), float(args['tilt'])
        except (KeyError, TypeError, ValueError) as e:
            print(f"[actions] bad servo args {args}: {e}", flush=True)
            return
        g = self.gui
        pan = int(round(max(-SERVO_PAN_MAX, min(SERVO_PAN_MAX, pan_asked))))
        raw = tilt_to_raw(max(-SERVO_TILT_MAX, min(SERVO_TILT_MAX, tilt_asked)))
        tilt = tilt_from_raw(raw)
        aid = action['action_id']
        print(f"[actions] servo #{aid} pan {pan:+d} tilt {tilt:+d}", flush=True)
        travel = max(abs(pan - g.pan_angle), abs(raw - g.tilt_angle))
        g.pan_angle, g.tilt_angle = pan, raw     # sent with the next tick; sliders follow
        g.servo_note = (f'servo ({pan:+d}, {tilt:+d})', time.monotonic())
        # the firmware steps the servos 1 degree per ms; report once they have arrived
        self._stop_evt.wait(0.25 + travel * SERVO_MS_PER_DEG / 1000.0)
        extra = {'pan': pan, 'tilt': tilt}
        problems = []
        if pan != round(pan_asked):
            problems.append(f'pan limited to ±{SERVO_PAN_MAX}')
        if tilt != round(tilt_asked):
            problems.append('tilt limited to what the servo can reach')
        if not (g.connected and g.i2c):
            problems.append('Arduino not connected')
        if problems:
            extra['problem'] = '; '.join(problems)
        self._report(aid, 'servo', **extra)

    def _drive(self, action):
        args = action.get('args', {})
        try:
            l   = max(-255, min(255, int(args['left_pwm'])))
            r   = max(-255, min(255, int(args['right_pwm'])))
            dur = max(0.0, float(args['duration_s']))
        except (KeyError, TypeError, ValueError) as e:
            print(f"[actions] bad approach_plant args {args}: {e}", flush=True)
            return
        aid = action['action_id']
        print(f"[actions] approach_plant #{aid} L={l} R={r} for {dur}s", flush=True)
        g = self.gui
        self._wait_free()
        g._auto_cancel.clear()
        g.auto = {'label': 'approach_plant', 't_end': time.monotonic() + dur}
        g.left_pwm, g.right_pwm = l, r
        stopped = g._auto_cancel.wait(dur)               # SPACE / STOP end it early
        g.left_pwm = g.right_pwm = 0
        g.auto = None
        extra = {'problem': 'stopped by the user before the end'} if stopped else {}
        self._report(aid, 'approach_plant', **extra)

# ── Listener status (what functions.py --listen is doing, from :8788/status) ───
class ListenerStatusReader(threading.Thread):
    """Polls the voice listener's GET :8788/status for the header's LLM/MIC
    indicators and the audio panel's ACTION row. Its own thread: ActionReader
    sleeps through every drive, and the capture tick must never wait on HTTP."""

    POLL_S = 0.2

    def __init__(self, host='localhost', port=8788):
        super().__init__(daemon=True)
        self.url = f'http://{host}:{port}/status'
        self.stage = 'off'       # 'off' = nothing answering on :8788
        self.last_action = None

    def run(self):
        import urllib.request
        while True:
            try:
                with urllib.request.urlopen(self.url, timeout=1.0) as resp:
                    status = json.loads(resp.read())
                self.stage = status.get('stage', 'off')
                self.last_action = status.get('last_action') or self.last_action
            except Exception:
                self.stage = 'off'
            time.sleep(self.POLL_S)

# ── Camera reader ─────────────────────────────────────────────────────────────
class CameraReader(threading.Thread):
    W, H = 640, 480
    FRAME_BYTES = W * H * 3

    def __init__(self):
        super().__init__(daemon=True)
        self._frame = None
        self._lock  = threading.Lock()
        self.status = 'starting'
        self._stop  = threading.Event()
        self._proc  = None

    def stop(self):
        self._stop.set()
        if self._proc:
            try: self._proc.terminate()
            except: pass

    def get_frame(self):
        with self._lock:
            return self._frame.copy() if self._frame is not None else None

    FIFO = '/tmp/logots_camera.fifo'

    def run(self):
        # Use a FIFO so frame bytes are fully separated from gst-launch text output
        if os.path.exists(self.FIFO):
            os.unlink(self.FIFO)
        os.mkfifo(self.FIFO)

        pipeline = (
            f'nvarguscamerasrc sensor-id=0 '
            f'! "video/x-raw(memory:NVMM),width={self.W},height={self.H},framerate={CAMERA_FPS}/1" '
            f'! nvvidconv '
            f'! "video/x-raw,format=BGRx" '
            f'! videoconvert '
            f'! "video/x-raw,format=BGR" '
            f'! filesink location={self.FIFO}'
        )
        env = os.environ.copy()
        env['EGL_PLATFORM'] = 'surfaceless'
        try:
            self._proc = subprocess.Popen(
                f'gst-launch-1.0 -q {pipeline}',
                shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                env=env
            )
            with open(self.FIFO, 'rb') as fifo:
                self.status = 'running'
                while not self._stop.is_set():
                    raw = fifo.read(self.FRAME_BYTES)
                    if len(raw) != self.FRAME_BYTES:
                        break
                    frame = np.frombuffer(raw, dtype=np.uint8).reshape(self.H, self.W, 3)
                    with self._lock:
                        self._frame = frame
        except Exception as e:
            self.status = f'error: {e}'
            return
        self.status = 'stopped'

# ── 3-D board geometry ────────────────────────────────────────────────────────
_BV = np.array([[-0.8,-1.5,-0.1],[0.8,-1.5,-0.1],[0.8,1.5,-0.1],[-0.8,1.5,-0.1],
                [-0.8,-1.5, 0.1],[0.8,-1.5, 0.1],[0.8,1.5, 0.1],[-0.8,1.5, 0.1]])
_BF = [[0,1,2,3],[4,5,6,7],[4,5,1,0],[6,7,3,2],[0,3,7,4],[1,2,6,5]]
_FC = [C_BLUE,C_BLUE,C_GREEN,C_GREEN,C_RED,C_RED]

def _rotmat(r,p,y):
    r=math.radians(r); p=math.radians(p); y=math.radians(y)
    Rx=np.array([[1,0,0],[0,math.cos(r),-math.sin(r)],[0,math.sin(r),math.cos(r)]])
    Ry=np.array([[math.cos(p),0,math.sin(p)],[0,1,0],[-math.sin(p),0,math.cos(p)]])
    Rz=np.array([[math.cos(y),-math.sin(y),0],[math.sin(y),math.cos(y),0],[0,0,1]])
    return Rz@Ry@Rx

# ── Frame encoding ────────────────────────────────────────────────────────────
def _encode_frame(frame_bgr):
    """BGR numpy (H×W×3) → color 640×640 JPEG base64 string."""
    img = Image.fromarray(frame_bgr[:, :, ::-1])
    img = img.resize((640, 640), Image.LANCZOS)
    buf = io.BytesIO()
    img.save(buf, format='JPEG', quality=70)
    return base64.b64encode(buf.getvalue()).decode('utf-8')

def _decode_frame(b64):
    """base64 JPEG string → BGR numpy (H×W×3), or None if empty.
    Handles both old grayscale and new color recordings."""
    if not b64:
        return None
    img = Image.open(io.BytesIO(base64.b64decode(b64))).convert('RGB')
    return np.asarray(img)[:, :, ::-1]

# ── Recording manager ─────────────────────────────────────────────────────────
class RecordingManager:
    # Original columns — the minimal set required for a CSV to count as a Logots
    # recording (SimPlayer checks against this so pre-position recordings still load).
    CORE_FIELDNAMES = [
        'frame_id', 'timestamp', 'frame_data',
        'yaw', 'pitch', 'roll',
        'audio_samples',
        'left_pwm', 'right_pwm', 'pan_angle', 'tilt_angle',
    ]
    # pos_x/pos_y (metres) + heading (deg): dead-reckoning estimate, see PositionEstimator.
    # mic_gain_db: the GAIN slider's value, already applied to audio_samples (2026-11-17).
    FIELDNAMES = CORE_FIELDNAMES + ['pos_x', 'pos_y', 'heading', 'mic_gain_db']

    def __init__(self):
        self._active   = False
        self._queue    = queue.Queue(maxsize=300)
        self._thread   = None
        self._csv_path = None

    @property
    def active(self): return self._active

    def start(self):
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        _repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        out_dir = os.path.join(_repo, 'recordings', f'session_{ts}')
        os.makedirs(out_dir, exist_ok=True)
        self._csv_path = os.path.join(out_dir, f'session_{ts}.csv')
        self._active = True
        self._thread = threading.Thread(target=self._writer, daemon=True)
        self._thread.start()

    def stop(self):
        self._active = False
        self._queue.put(None)
        if self._thread:
            self._thread.join(timeout=15)

    def enqueue(self, frame: dict, frame_bgr):
        if not self._active: return
        try:
            self._queue.put_nowait((dict(frame), frame_bgr))
        except queue.Full:
            pass

    def _writer(self):
        with open(self._csv_path, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=self.FIELDNAMES)
            w.writeheader()
            while True:
                item = self._queue.get()
                if item is None:
                    break
                row, frame_bgr = item
                row['frame_data'] = _encode_frame(frame_bgr) if frame_bgr is not None else ''
                w.writerow(row)
                f.flush()

# ── Sim player ────────────────────────────────────────────────────────────────
class SimPlayer:
    """Replays a recorded session CSV row-by-row, paced by its timestamps
    so playback runs in real time regardless of the rate the recording
    actually achieved."""

    WAIT = 'wait'   # sentinel: current frame should be held, next row not due yet

    def __init__(self, csv_path):
        self.csv_path  = csv_path
        self.loop      = True
        self.row_index = 0
        self.n_skipped = 0
        self._pending    = None   # parsed (frame, decoded, bgr) not yet due
        self._pending_ts = None   # its recorded timestamp (datetime or None)
        try:                       # one-time line count for the SIM %-progress readout
            with open(csv_path, 'r', newline='') as f:
                self.total_rows = max(sum(1 for _ in f) - 1, 0)   # minus header
        except OSError:
            self.total_rows = 0
        self._open()

    def _open(self):
        self._file   = open(self.csv_path, 'r', newline='')
        self._reader = csv.DictReader(self._file)
        fields = self._reader.fieldnames or []
        if not set(RecordingManager.CORE_FIELDNAMES) <= set(fields):
            self._file.close()
            raise ValueError('not a Logots recording CSV')
        self.row_index    = 0     # restart the %-progress readout each pass
        self._wall_start = None   # monotonic time when this pass started
        self._rec_start  = None   # first row's recorded timestamp of this pass

    def next_frame(self):
        """Return the next (frame, decoded, frame_bgr) once its recorded
        timestamp is due, SimPlayer.WAIT while the current frame should be
        held, or None at end-of-file when loop is off. Malformed rows are
        skipped; rows without a parsable timestamp are due immediately."""
        if self._pending is None and not self._advance():
            return None
        if self._pending_ts is not None:
            now = time.monotonic()
            if self._wall_start is None:
                self._wall_start = now
                self._rec_start  = self._pending_ts
            due = (self._pending_ts - self._rec_start).total_seconds()
            if now - self._wall_start < due:
                return SimPlayer.WAIT
        out = self._pending
        self._pending = None
        return out

    def _advance(self):
        """Parse the next valid row into self._pending. False at EOF (no loop)."""
        while True:
            try:
                row = next(self._reader)
            except StopIteration:
                if not self.loop:
                    return False
                self._file.close()
                self._open()          # resets this pass's time origins
                continue
            self.row_index += 1
            try:
                self._pending = self._parse(row)
            except Exception:
                self.n_skipped += 1
                continue
            try:
                self._pending_ts = datetime.fromisoformat(self._pending[0]['timestamp'])
            except Exception:
                self._pending_ts = None
            return True

    def _parse(self, row):
        def arr(key):
            try:    return json.loads(row.get(key) or '[]')
            except Exception: return []
        def num(key, fallback):
            try:    return int(float(row.get(key)))
            except Exception: return fallback
        def fnum(key, fallback):            # float variant, for the position columns
            try:    return float(row.get(key))
            except Exception: return fallback
        yaw, pitch, roll = arr('yaw'), arr('pitch'), arr('roll')
        decoded = {
            'imu':        list(zip(yaw, pitch, roll)),
            'audio':      arr('audio_samples'),
            'left_pwm':   num('left_pwm', 0),
            'right_pwm':  num('right_pwm', 0),
            'pan_angle':  num('pan_angle', 0),
            'tilt_angle': num('tilt_angle', 90),
            'pos_x':      fnum('pos_x', 0.0),   # 0.0 when column absent (old CSVs)
            'pos_y':      fnum('pos_y', 0.0),
            'heading':    fnum('heading', 0.0),
        }
        frame = {
            'frame_id':      num('frame_id', self.row_index - 1),
            'timestamp':     row.get('timestamp') or '',
            'frame_data':    row.get('frame_data') or '',
            'yaw':           row.get('yaw') or '[]',
            'pitch':         row.get('pitch') or '[]',
            'roll':          row.get('roll') or '[]',
            'audio_samples': row.get('audio_samples') or '[]',
            'left_pwm':      decoded['left_pwm'],
            'right_pwm':     decoded['right_pwm'],
            'pan_angle':     decoded['pan_angle'],
            'tilt_angle':    decoded['tilt_angle'],
            'pos_x':         decoded['pos_x'],
            'pos_y':         decoded['pos_y'],
            'heading':       decoded['heading'],
            'mic_gain_db':   fnum('mic_gain_db', 0.0),   # 0 dB for CSVs made before the slider
        }
        return frame, decoded, _decode_frame(frame['frame_data'])

    def close(self):
        try: self._file.close()
        except Exception: pass

# ── Frame API server ──────────────────────────────────────────────────────────
class FrameServer:
    """Stdlib HTTP server exposing the GUI's latest_frame at GET /latest_frame.
    JPEG encoding for real mode happens lazily per request (cached by frame_id)
    so the 20 Hz tick never pays for it."""

    def __init__(self, gui, host='0.0.0.0', port=API_PORT):
        self.gui    = gui
        self._cache = (None, '')   # (frame_id, base64 jpeg)
        self._server = ThreadingHTTPServer((host, port), self._make_handler())
        threading.Thread(target=self._server.serve_forever, daemon=True).start()

    def _payload(self):
        gui = self.gui
        with gui._frame_lock:
            frame = dict(gui.latest_frame)
            bgr   = gui.latest_frame_bgr
            sim   = gui.sim_mode
        if not frame:
            return None
        if not frame.get('frame_data') and bgr is not None:
            fid, b64 = self._cache
            if fid != frame['frame_id']:
                b64 = _encode_frame(bgr)
                self._cache = (frame['frame_id'], b64)
            frame['frame_data'] = b64
        for k in ('yaw', 'pitch', 'roll', 'audio_samples'):
            try:    frame[k] = json.loads(frame.get(k) or '[]')
            except Exception: frame[k] = []
        frame['sim_mode'] = sim
        return frame

    def _make_handler(server_self):
        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *a): pass
            def _send(self, code, obj):
                body = json.dumps(obj).encode('utf-8')
                self.send_response(code)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            def do_GET(self):
                if self.path.split('?')[0] != '/latest_frame':
                    self._send(404, {'error': 'unknown path, use /latest_frame'})
                    return
                try:
                    payload = server_self._payload()
                except Exception as e:
                    self._send(500, {'error': str(e)})
                    return
                if payload is None:
                    self._send(503, {'error': 'no frame yet'})
                else:
                    self._send(200, payload)
        return Handler

# ── Widget helpers ────────────────────────────────────────────────────────────
def _hline(p):
    tk.Frame(p,bg=C_BORDER,height=1).pack(fill='x',padx=16,pady=3)

def _lbl(p,txt,fg=C_SUB,font=('Courier',9),bg=C_BG,**kw):
    return tk.Label(p,text=txt,bg=bg,fg=fg,font=font,**kw)

def _plbl(p,txt,fg=C_SUB,font=('Courier',9),**kw):
    return tk.Label(p,text=txt,bg=C_PANEL,fg=fg,font=font,**kw)

def _vallbl(p,txt,fg=C_BLUE,width=0,**kw):
    kw.setdefault('padx',8); kw.setdefault('pady',4)
    return tk.Label(p,text=txt,bg=C_PANEL,fg=fg,font=('Courier',11,'bold'),
                    width=width,**kw)

def _btn(p,txt,color,cmd,**kw):
    return tk.Button(p,text=txt,bg=color,fg=C_BTN_TEXT,relief='flat',
                     font=('Courier',10,'bold'),padx=12,pady=3,command=cmd,
                     cursor='hand2',activebackground=color,activeforeground=C_BTN_TEXT,**kw)

# ── Main GUI ──────────────────────────────────────────────────────────────────
class RobotControlGUI:
    def __init__(self, root):
        self.root=root
        root.title('Logots — Robot Control')
        root.configure(bg=C_BG)
        root.resizable(False,False)

        self.left_pwm=self.right_pwm=0
        self.pan_angle=0     # -90…+90, 0 = camera facing front, + = left
        self.tilt_angle=90
        self.enc_left=self.enc_right=None   # cumulative encoder counts; None = no reading
        self.fw_motion=False                # the Arduino's firmware has the rotate command
        self.fw_mot_seq=self.fw_mot_state=None   # its last rotate seq / state; None = no reading
        self._mover=Mover(self)         # rotate-by-degrees requests (manual and LLM)
        self.auto=None                      # {'label','t_end'} while ActionReader drives
        self.servo_note=None                # (text, time) of the last servo command, for the badge
        self._auto_cancel=threading.Event() # set by SPACE / STOP to end that drive early
        self._was_rotating=False
        g=_load_settings().get('mic_gain_db',0)
        self._mic_gain_db=int(max(MIC_GAIN_DB_MIN,min(MIC_GAIN_DB_MAX,g if isinstance(g,(int,float)) else 0)))
        self._clip_t=0.0                    # time.monotonic() of the last clipped sample
        self._gain_lbl_state=None
        self._straight_target=None   # locked IMU yaw for the gyro straight-line trim, see _straight_trim()
        self._straight_integral=0.0  # accumulated yaw error for the trim's integral term
        self._straight_t=None        # time.monotonic() of the last _straight_trim() call

        self._pos_est   = PositionEstimator()
        self._pos_trail = collections.deque(maxlen=400)  # (x,y) points for the mini-map

        self._fps_ema     = 0.0    # smoothed actual capture rate (Hz)
        self._last_tick_t = None   # monotonic time of the previous tick start

        self.i2c=None; self.connected=False
        self._last_reconnect=0

        self._rec_manager  = RecordingManager()
        self._rec_frame_id = 0
        self.latest_frame     = {}
        self.latest_frame_bgr = None
        self.latest_decoded   = {'imu': [], 'audio': []}
        self._frame_lock      = threading.Lock()
        self._rms_ema         = 0.0

        self.sim_mode   = False
        self.sim_player = None
        self._sim_ended = False
        self._pre_sim_angles = (90, 90)

        self.imu_reader=None; self.audio_reader=None; self.camera_reader=None
        self._imu_poly=None; self._imu_live=False
        self.action_reader=None
        self.listener=None

        self._build_ui()
        self._bind_keys()
        self._start_sensors()
        try:
            self._frame_server = FrameServer(self)
        except Exception as e:
            print(f'[logots] frame API server not started: {e}')
            self._frame_server = None
        self.action_reader = ActionReader(self); self.action_reader.start()
        self.listener_status = ListenerStatusReader(); self.listener_status.start()
        self._start_listener()
        self.root.protocol('WM_DELETE_WINDOW',self._shutdown)
        # via after(): never interrupt a tick mid-I2C-write with the final motor-stop
        signal.signal(signal.SIGTERM,lambda *_: self.root.after(0,self._shutdown))
        self._loop()

    # ── Build UI ──────────────────────────────────────────────────────────────
    def _build_ui(self):
        self._hdr(); _hline(self.root)
        self._main_panels()

    def _hdr(self):
        f=tk.Frame(self.root,bg=C_BG)
        f.pack(fill='x',padx=16,pady=(12,0))
        _lbl(f,'LOGOTS  ROBOT CONTROL',fg=C_TEXT,font=('Courier',14,'bold')).pack(side='left')
        self.lbl_cs=_lbl(f,'⬤  DISCONNECTED',fg=C_RED,font=('Courier',9,'bold'))
        self.lbl_cs.pack(side='right')
        self.lbl_fps=_lbl(f,'FPS: --',fg=C_SUB,font=('Courier',9,'bold'))
        self.lbl_fps.pack(side='right',padx=(0,16))
        self.lbl_llm=_lbl(f,'⬤  LLM: --',fg=C_SUB,font=('Courier',9,'bold'))
        self.lbl_llm.pack(side='right',padx=(0,16))
        self.lbl_voice=_lbl(f,'⬤  MIC: --',fg=C_SUB,font=('Courier',9,'bold'))
        self.lbl_voice.pack(side='right',padx=(0,16))


    def _main_panels(self):
        f=tk.Frame(self.root,bg=C_BG)
        f.pack(padx=16,pady=4)
        # Original 2×2 grid of big blocks, unchanged in size/height — the former
        # full-width bottom status bar is now a sidebar to its left instead.
        SW=190; LW=480; RW=415; TH=430; BH=220; GAP=6  # LW/TH grown 2026-10-03 (were 375/310):
        # the enlarged 200px position map (was 118px) sits in a column beside the 244px-wide
        # joystick, and together with the pan/tilt sliders above it, that column no longer fit
        # in the old fixed width *or* height — p_ctrl's content was silently being clipped by
        # pack_propagate(False) on both axes, not failing to draw (p_audio shares LW and
        # p_imu shares TH for grid symmetry, so they grow a little too and just get more
        # padding — no layout change needed there)
        p_side  = tk.Frame(f,bg=C_BG,   width=SW,height=TH+GAP+BH)
        p_ctrl  = tk.Frame(f,bg=C_PANEL,width=LW,height=TH)
        p_imu   = tk.Frame(f,bg=C_PANEL,width=RW,height=TH)
        p_audio = tk.Frame(f,bg=C_PANEL,width=LW,height=BH)
        p_video = tk.Frame(f,bg=C_PANEL,width=RW,height=BH)
        for p in (p_side,p_ctrl,p_imu,p_audio,p_video):
            p.pack_propagate(False)
        p_side.grid( row=0,column=0,rowspan=2,padx=(0,GAP),sticky='nsew')
        p_ctrl.grid( row=0,column=1,padx=(0,GAP),pady=(0,GAP),sticky='nsew')
        p_imu.grid(  row=0,column=2,pady=(0,GAP),sticky='nsew')
        p_audio.grid(row=1,column=1,padx=(0,GAP),sticky='nsew')
        p_video.grid(row=1,column=2,sticky='nsew')
        self._side_panel(p_side)
        self._ctrl_panel(p_ctrl)
        self._audio_panel(p_audio)
        self._imu_panel(p_imu)
        self._video_panel(p_video)

    # ── Control panel ─────────────────────────────────────────────────────────
    def _ctrl_panel(self,parent):
        p=parent
        h=tk.Frame(p,bg=C_PANEL); h.pack(fill='x',padx=12,pady=(8,4))
        _plbl(h,'DRIVE  &  CAM',font=('Courier',9,'bold')).pack(side='left')
        # READY, or what the wheels/servos are doing right now (see _update_drive_widgets)
        self.lbl_auto=_plbl(h,'⬤  READY',font=('Courier',9,'bold')); self.lbl_auto.pack(side='right')
        # top half: traverse + rotate (left), pan/tilt sliders (right); bottom half: the map,
        # full width, filling whatever height is left (so font size differences between the
        # Mac and the Jetson can't clip it, see the 2026-10-03 layout entry in CLAUDE.md)
        row=tk.Frame(p,bg=C_PANEL); row.pack(fill='x',padx=10,pady=(0,4))
        lc=tk.Frame(row,bg=C_PANEL); lc.pack(side='left',anchor='n')
        self._motion_controls(lc)
        sl=tk.Frame(row,bg=C_PANEL); sl.pack(side='right',anchor='n')
        self._cam_sliders(sl)
        mp=tk.Frame(p,bg=C_PANEL); mp.pack(fill='both',expand=True,padx=10,pady=(0,10))
        self._position_map(mp)

    def _motion_controls(self,parent):
        """The only wheel controls: TRAVERSE in cm (+ = forward) and ROTATE in degrees
        (+ = left). Step buttons, or type any value and GO. Each sends one command; the
        robot carries it out and stops by itself."""
        def b(p,txt,cmd):
            return tk.Button(p,text=txt,bg=C_BORDER,fg=C_BTN_TEXT,relief='flat',font=('Courier',9,'bold'),
                             padx=4,pady=2,command=cmd,cursor='hand2',
                             activebackground=C_BLUE,activeforeground=C_BTN_TEXT)
        self._move_entries={}
        for kind,title,steps,unit,init,limit in [
                ('traverse','TRAVERSE',[('FWD 50',50),('FWD 10',10),('BWD 10',-10),('BWD 50',-50)],
                 'cm + = fwd','30',TRAV_MAX_CM),
                ('rotate','ROTATE',[('L 90°',90),('L 15°',15),('R 15°',-15),('R 90°',-90)],
                 '°  + = left','30',180)]:
            r1=tk.Frame(parent,bg=C_PANEL); r1.pack(pady=(6,1))
            for txt,v in steps:
                b(r1,txt,lambda k=kind,v=v:self._move_manual(k,v)).pack(side='left',padx=2)
            r2=tk.Frame(parent,bg=C_PANEL); r2.pack(pady=(1,0))
            _plbl(r2,title,font=('Courier',8,'bold'),width=8,anchor='w').pack(side='left')
            var=tk.StringVar(value=init)
            ent=tk.Entry(r2,textvariable=var,width=5,justify='right',
                         bg='#16162a',fg=C_TEXT,insertbackground=C_TEXT,relief='flat',
                         font=('Courier',10,'bold'),highlightthickness=1,
                         highlightbackground=C_BORDER,highlightcolor=C_BLUE)
            ent.pack(side='left',padx=(4,2))
            go=lambda e=None,k=kind:self._move_go(k)
            ent.bind('<Return>',go)
            _plbl(r2,unit,font=('Courier',8),width=11,anchor='w').pack(side='left')
            b(r2,'GO',go).pack(side='left',padx=(2,0))
            self._move_entries[kind]=(var,ent,limit)

    def _move_go(self,kind):
        var,ent,limit=self._move_entries[kind]
        try:
            v=float(var.get().strip().replace('°','').replace('cm',''))
        except ValueError:
            ent.config(highlightbackground=C_RED,highlightcolor=C_RED); return
        ent.config(highlightbackground=C_BORDER,highlightcolor=C_BLUE)
        self.root.focus_set()                 # give W/S/A/D/SPACE back to the robot
        self._move_manual(kind,max(-limit,min(limit,v)))

    def _move_manual(self,kind,amount):
        if self.sim_mode or self.self_driving or amount==0: return
        self._mover.request(kind,amount,'manual')

    @property
    def self_driving(self):
        """The wheels are moving by themselves: a traverse or rotation in progress (from
        the GUI or the LLM), or the LLM's older approach_plant drive. The motion controls
        are ignored meanwhile; SPACE / STOP cancel (_estop)."""
        return self.auto is not None or self._mover.active

    def _cam_sliders(self,parent):
        # The servo parameters (pan, tilt), same meaning as the servo command: pan 0 = front,
        # + = left, -45…+45 (GUI/command cap since 2026-10-03; the firmware accepts ±90);
        # tilt 0 = level, + = up (signed since 2026-11-10; tilt_angle itself stays the raw
        # servo angle for the wire, the CSV and the frame API — see tilt_to_raw()).
        for attr,label,top,bot,init,top_txt,bot_txt in [
                ('pan', 'PAN', SERVO_PAN_MAX, -SERVO_PAN_MAX,0, f'L {SERVO_PAN_MAX}°',f'R {SERVO_PAN_MAX}°'),
                ('tilt','TILT',SERVO_TILT_MAX,-SERVO_TILT_MAX,tilt_from_raw(self.tilt_angle),
                 f'UP {SERVO_TILT_MAX}°',f'DN {SERVO_TILT_MAX}°')]:
            f=tk.Frame(parent,bg=C_PANEL); f.pack(side='left',padx=2)
            _plbl(f,label,font=('Courier',8,'bold')).pack(pady=(4,0))
            _plbl(f,top_txt,font=('Courier',7)).pack()
            var=tk.IntVar(value=init)
            setattr(self,f'{attr}_var',var)
            tk.Scale(f,from_=top,to=bot,variable=var,orient='vertical',
                     length=120,width=16,bg=C_PANEL,fg=C_TEXT,troughcolor='#16162a',
                     activebackground=C_BLUE_HI,highlightthickness=0,bd=0,
                     sliderrelief='flat',sliderlength=20,showvalue=True,
                     command=lambda v,a=attr:self._slider_moved(a,int(float(v)))).pack()
            _plbl(f,bot_txt,font=('Courier',7)).pack(pady=(0,4))

    def _slider_moved(self,attr,v):
        if self.sim_mode: return
        if attr=='pan': self.pan_angle=v
        else:           self.tilt_angle=tilt_to_raw(v)

    # ── Position mini-map ─────────────────────────────────────────────────────
    # Size: fills the lower half of DRIVE & CAM (2026-11-17, was a 200 px square beside the
    # dial); the real size comes from <Configure>, these are only the minimum.
    POS_MAP_MIN_W, POS_MAP_MIN_H = 200, 120
    POS_MAP_PX_PER_M = 28  # fixed scale (unchanged), so a wider map shows more floor
    POS_MAP_N_GRID = 24    # pool size per axis — enough for a 670 px side at this scale

    def _position_map(self,parent):
        _plbl(parent,'POSITION (m)',font=('Courier',7,'bold')).pack(pady=(2,0))
        cv=tk.Canvas(parent,width=self.POS_MAP_MIN_W,height=self.POS_MAP_MIN_H,
                     bg='#16162a',highlightthickness=1,highlightbackground=C_BORDER)
        cv.pack(fill='both',expand=True,pady=(1,0))
        self.pos_cv=cv
        self._map_w,self._map_h=self.POS_MAP_MIN_W,self.POS_MAP_MIN_H
        def resized(e):
            self._map_w,self._map_h=e.width,e.height; self._draw_pos_map()
        cv.bind('<Configure>',resized)
        # Pre-create a fixed pool of canvas items, updated in place every redraw via
        # coords()/itemconfig() instead of delete()+create() — found 2026-10-03 that
        # recreating ~16 canvas text items cost ~65ms total on this system (each
        # create_text() ~4ms), blowing most of the capture tick's 100ms budget and dragging
        # FPS from 10 down to ~5. Updating pre-existing items costs ~0.02ms each instead.
        # Creation order here also fixes the stacking order permanently (grid under trail
        # under arrow/dot), so it never needs re-asserting on redraw.
        N=self.POS_MAP_N_GRID
        self._map_vlines =[cv.create_line(0,0,0,1,fill='#25253a') for _ in range(N)]
        self._map_hlines =[cv.create_line(0,0,1,0,fill='#25253a') for _ in range(N)]
        self._map_vlabels=[cv.create_text(0,0,text='',fill=C_SUB,font=('Courier',6),anchor='nw') for _ in range(N)]
        self._map_hlabels=[cv.create_text(0,0,text='',fill=C_SUB,font=('Courier',6),anchor='nw') for _ in range(N)]
        self._map_origin = cv.create_oval(0,0,0,0,outline=C_SUB,fill='',state='hidden')
        self._map_trail  = cv.create_line(0,0,0,0,fill=C_BLUE,width=1,state='hidden')
        self._map_target = cv.create_line(0,0,0,0,fill=C_AMBER,width=2,dash=(4,3),state='hidden')  # where a rotation will end
        self._map_arrow  = cv.create_line(0,0,0,0,fill=C_GREEN,width=3,arrow='last',arrowshape=(10,12,5))
        self._map_dot    = cv.create_oval(0,0,0,0,fill=C_BLUE_HI,outline='')
        self._draw_pos_map()

    def _draw_pos_map(self):
        """Robot-centered top-down map (2026-10-03 redesign): the heading arrow stays fixed
        at the canvas center; the background (metre gridlines + trail) pans under it as the
        robot moves, like a radar/mini-map display, instead of the whole trail being
        auto-zoomed to fit. Fixed scale (POS_MAP_PX_PER_M) so the gridlines are always
        meaningful distance marks, not a shrinking/growing scale. +X right, +Y up on screen.
        Updates the item pool created in _position_map() in place — see that method's
        comment for why (performance)."""
        cv=self.pos_cv; W,H=self._map_w,self._map_h; PPM=self.POS_MAP_PX_PER_M
        pts=list(self._pos_trail)
        rx,ry = pts[-1] if pts else (0.0,0.0)   # robot's current world position
        heading = self.latest_frame.get('heading',0.0) if self.latest_frame else 0.0
        cxp,cyp=W/2,H/2   # the robot is always drawn here — canvas center never moves
        def to_px(wx,wy):
            return cxp+(wx-rx)*PPM, cyp-(wy-ry)*PPM   # world -> screen, robot-centered
        # metre gridlines, panning with the robot — origin's axis line drawn brighter so you
        # can always tell which way back to the start (session/recording origin) is
        half_x, half_y = (W/2)/PPM, (H/2)/PPM
        xs = list(range(math.floor(rx-half_x), math.ceil(rx+half_x)+1))
        ys = list(range(math.floor(ry-half_y), math.ceil(ry+half_y)+1))
        for i,item in enumerate(self._map_vlines):
            if i<len(xs):
                gx=xs[i]; px,_=to_px(gx,ry)
                cv.coords(item,px,0,px,H); cv.itemconfig(item,fill=(C_SUB if gx==0 else '#25253a'),state='normal')
            else:
                cv.itemconfig(item,state='hidden')
        for i,item in enumerate(self._map_vlabels):
            if i<len(xs) and xs[i]!=0:
                px,_=to_px(xs[i],ry)
                cv.coords(item,px+2,2); cv.itemconfig(item,text=str(xs[i]),state='normal')
            else:
                cv.itemconfig(item,state='hidden')
        for i,item in enumerate(self._map_hlines):
            if i<len(ys):
                gy=ys[i]; _,py=to_px(rx,gy)
                cv.coords(item,0,py,W,py); cv.itemconfig(item,fill=(C_SUB if gy==0 else '#25253a'),state='normal')
            else:
                cv.itemconfig(item,state='hidden')
        for i,item in enumerate(self._map_hlabels):
            if i<len(ys) and ys[i]!=0:
                _,py=to_px(rx,ys[i])
                cv.coords(item,2,py+2); cv.itemconfig(item,text=str(ys[i]),state='normal')
            else:
                cv.itemconfig(item,state='hidden')
        # origin marker (session/recording start), only shown while it's in view
        ox,oy=to_px(0.0,0.0)
        if 0<=ox<=W and 0<=oy<=H:
            cv.coords(self._map_origin,ox-3,oy-3,ox+3,oy+3); cv.itemconfig(self._map_origin,state='normal')
        else:
            cv.itemconfig(self._map_origin,state='hidden')
        # trail polyline, in the same panning frame — older points scroll off-canvas
        # naturally as the robot moves on, newest end always meets the center arrow
        if len(pts)>=2:
            flat=[]
            for wx,wy in pts:
                px,py=to_px(wx,wy); flat+=[px,py]
            cv.coords(self._map_trail,*flat); cv.itemconfig(self._map_trail,state='normal')
        else:
            cv.itemconfig(self._map_trail,state='hidden')
        # heading arrow — always at the fixed canvas center
        th=math.radians(heading)
        ax=cxp+24*math.cos(th); ay=cyp-24*math.sin(th)
        cv.coords(self._map_arrow,cxp,cyp,ax,ay)
        job=self._mover.job
        if job and job['kind']=='rotate' and not self.sim_mode:
            tt=math.radians(heading+job['requested']-job['turned'])
            cv.coords(self._map_target,cxp,cyp,cxp+60*math.cos(tt),cyp-60*math.sin(tt))
            cv.itemconfig(self._map_target,state='normal')
        else:
            cv.itemconfig(self._map_target,state='hidden')
        cv.coords(self._map_dot,cxp-4,cyp-4,cxp+4,cyp+4)

    # ── Audio panel ───────────────────────────────────────────────────────────
    def _audio_panel(self,parent):
        p=parent
        h=tk.Frame(p,bg=C_PANEL); h.pack(fill='x',padx=8,pady=(6,2))
        _plbl(h,'AUDIO INPUT',font=('Courier',9,'bold')).pack(side='left')
        self.lbl_as=_plbl(h,'',font=('Courier',8),fg=C_AMBER); self.lbl_as.pack(side='right')
        self.wav_cv=tk.Canvas(p,width=356,height=90,bg='#16162a',highlightthickness=0)
        self.wav_cv.pack(padx=8,pady=(0,4))
        self._wave_idle()
        br=tk.Frame(p,bg=C_PANEL); br.pack(fill='x',padx=8,pady=(0,4))
        _plbl(br,'RMS',font=('Courier',8),width=5,anchor='w').pack(side='left')
        self.rms_cv=tk.Canvas(br,width=286,height=8,bg='#16162a',highlightthickness=0)
        self.rms_cv.pack(side='left',padx=6)
        # Mic gain, see MIC_GAIN_DB_MIN. Saved when the mouse is released; double-click = 0 dB.
        gr=tk.Frame(p,bg=C_PANEL); gr.pack(fill='x',padx=8,pady=(0,4))
        _plbl(gr,'GAIN',font=('Courier',8),width=5,anchor='w').pack(side='left')
        self.mic_gain_var=tk.IntVar(value=self._mic_gain_db)
        sc=tk.Scale(gr,from_=MIC_GAIN_DB_MIN,to=MIC_GAIN_DB_MAX,variable=self.mic_gain_var,
                    orient='horizontal',length=286,width=10,bg=C_PANEL,fg=C_TEXT,
                    troughcolor='#16162a',activebackground=C_BLUE_HI,highlightthickness=0,bd=0,
                    sliderrelief='flat',sliderlength=14,showvalue=False,
                    command=lambda v:self._mic_gain_moved(int(float(v))))
        sc.pack(side='left',padx=6)
        sc.bind('<ButtonRelease-1>',lambda e:_save_settings(mic_gain_db=self._mic_gain_db))
        sc.bind('<Double-Button-1>',lambda e:(self.mic_gain_var.set(0),self._mic_gain_moved(0),
                                              _save_settings(mic_gain_db=0)))
        self.lbl_gain=_plbl(gr,'',font=('Courier',8,'bold'),fg=C_TEXT); self.lbl_gain.pack(side='left')
        self._update_gain_label()
        ar=tk.Frame(p,bg=C_PANEL); ar.pack(fill='x',padx=8,pady=(0,8))
        _plbl(ar,'ACTION',font=('Courier',8)).pack(side='left',anchor='n')
        self.lbl_voice_action=_plbl(ar,'(no action yet)',font=('Courier',8),fg=C_SUB)
        self.lbl_voice_action.config(wraplength=410,justify='left')   # approach_plant's args run long
        self.lbl_voice_action.pack(side='left',padx=6)

    def _wave_idle(self):
        cv=self.wav_cv; w,h=356,90
        cv.delete('all')
        cv.create_line(0,h//2,w,h//2,fill=C_BORDER)
        cv.create_text(w//2,h//2,text='NO AUDIO',fill=C_SUB,font=('Courier',9))

    def _draw_wave(self,samples):
        cv=self.wav_cv; w,h=356,90; mid=h//2
        cv.delete('all')
        cv.create_line(0,mid,w,mid,fill=C_BORDER)
        n=len(samples)
        if n<2: return
        pts=[]
        for i,s in enumerate(samples):
            pts.extend([i*w/(n-1), mid-float(s)*mid*0.9*8])
        cv.create_line(*pts,fill=C_GREEN,width=1,smooth=True)

    def _mic_gain_moved(self,db):
        self._mic_gain_db=db
        if self.audio_reader: self.audio_reader.set_gain_db(db)
        self._update_gain_label()

    def _update_gain_label(self):
        clip=time.monotonic()-self._clip_t<1.0   # a clipped sample in the last second
        state=(self._mic_gain_db,clip)
        if state==self._gain_lbl_state: return
        self._gain_lbl_state=state
        self.lbl_gain.config(text=f'{self._mic_gain_db:+d} dB'+('  CLIP' if clip else ''),
                             fg=(C_RED if clip else C_TEXT))

    def _draw_rms(self,rms):
        cv=self.rms_cv; cv.delete('all')
        fw=int(min(rms*64,1.0)*286)
        if fw>0:
            c=C_GREEN if rms<0.1 else (C_AMBER if rms<0.3 else C_RED)
            cv.create_rectangle(0,0,fw,8,fill=c,outline='')

    # ── IMU panel ─────────────────────────────────────────────────────────────
    def _imu_panel(self,parent):
        p=parent
        h=tk.Frame(p,bg=C_PANEL); h.pack(fill='x',padx=8,pady=(6,2))
        _plbl(h,'IMU ORIENTATION',font=('Courier',9,'bold')).pack(side='left')
        self.lbl_is=_plbl(h,'starting...',font=('Courier',8),fg=C_AMBER); self.lbl_is.pack(side='right')

        # Calibration placeholder (same footprint as the matplotlib figure)
        self._cal_frame=tk.Frame(p,bg=C_PANEL,width=407,height=240)
        self._cal_frame.pack_propagate(False); self._cal_frame.pack()
        _plbl(self._cal_frame,'Calibrating — keep IMU still',fg=C_AMBER,
              font=('Courier',10)).place(relx=.5,rely=.38,anchor='center')
        self._cal_cv=tk.Canvas(self._cal_frame,width=300,height=10,
                                bg='#16162a',highlightthickness=0)
        self._cal_cv.place(relx=.5,rely=.58,anchor='center')

        # Matplotlib 3D (packed after calibration)
        fig=Figure(figsize=(4.07,2.4),dpi=100,facecolor=C_PANEL)
        self._imu_ax=fig.add_subplot(111,projection='3d')
        ax=self._imu_ax; ax.set_facecolor(C_PANEL)
        for pn in (ax.xaxis.pane,ax.yaxis.pane,ax.zaxis.pane):
            pn.fill=False; pn.set_edgecolor('#333')
        ax.set_xlim(-2.5,2.5); ax.set_ylim(-2.5,2.5); ax.set_zlim(-2.5,2.5)
        for lbl,color in [('X',C_RED),('Y',C_GREEN),('Z',C_BLUE)]:
            getattr(ax,f'set_{lbl.lower()}label')(lbl,color=color,labelpad=2,fontsize=7)
        ax.tick_params(colors='#555',labelsize=6)
        fig.tight_layout(pad=0.4)
        self._imu_fig_cv=FigureCanvasTkAgg(fig,master=p)
        self._imu_wgt=self._imu_fig_cv.get_tk_widget()
        self._imu_wgt.configure(bg=C_PANEL,highlightthickness=0)

        # YPR labels (packed after calibration) — no manual colon-alignment
        # padding (they're separate side-by-side chips, not stacked, so it
        # only wasted width) and tighter internal padx so all three fit.
        self._ypr_frame=tk.Frame(p,bg=C_PANEL)
        self.lbl_r=_vallbl(self._ypr_frame,'Roll: +0.0°',fg=C_RED,padx=5)
        self.lbl_pi=_vallbl(self._ypr_frame,'Pitch: +0.0°',fg=C_GREEN,padx=5)
        self.lbl_y=_vallbl(self._ypr_frame,'Yaw: +0.0°',fg=C_BLUE,padx=5)
        for w in (self.lbl_r,self.lbl_pi,self.lbl_y): w.pack(side='left',padx=1)

    def _imu_go_live(self):
        self._cal_frame.pack_forget()
        self._imu_wgt.pack(padx=8)
        self._ypr_frame.pack(pady=(0,8))
        self._imu_live=True

    def _imu_redraw(self,roll,pitch,yaw):
        ax=self._imu_ax
        if self._imu_poly:
            try: self._imu_poly.remove()
            except: pass
        R=_rotmat(roll,pitch,yaw); v=_BV@R.T
        faces=[[v[i] for i in f] for f in _BF]
        self._imu_poly=Poly3DCollection(faces,facecolors=_FC,
                                         edgecolors='#111',linewidths=0.4,alpha=0.88)
        ax.add_collection3d(self._imu_poly)
        self._imu_fig_cv.draw_idle()

    # ── Video panel ───────────────────────────────────────────────────────────
    def _video_panel(self,parent):
        p=parent
        h=tk.Frame(p,bg=C_PANEL); h.pack(fill='x',padx=8,pady=(6,2))
        _plbl(h,'VIDEO FEED',font=('Courier',9,'bold')).pack(side='left')
        self.lbl_vs=_plbl(h,'no camera',font=('Courier',8),fg=C_AMBER); self.lbl_vs.pack(side='right')
        self.vid_cv=tk.Canvas(p,width=397,height=175,bg='#16162a',highlightthickness=0)
        self.vid_cv.pack(padx=8,pady=(0,8))
        self._vid_placeholder()

        # Label widget for live camera frames (hidden until camera available)
        self._vid_lbl=tk.Label(p,bg='#16162a',borderwidth=0)

    def _vid_placeholder(self):
        cv=self.vid_cv; w,h=397,175; cx,cy=w//2,h//2-10
        cv.create_rectangle(cx-42,cy-24,cx+42,cy+24,outline=C_BORDER,width=2)
        cv.create_oval(cx-15,cy-15,cx+15,cy+15,outline=C_BORDER,width=2)
        cv.create_oval(cx-5,cy-5,cx+5,cy+5,fill=C_BORDER,outline='')
        cv.create_rectangle(cx+30,cy-9,cx+42,cy-3,fill=C_BORDER,outline='')
        cv.create_text(w//2,h//2+30,text='STARTING CAMERA...',
                       fill=C_SUB,font=('Courier',8))

    def display_frame(self,bgr_frame):
        rgb=bgr_frame[:,:,::-1]   # BGR → RGB without cv2
        img=Image.fromarray(rgb).resize((397,175))
        self._photo=ImageTk.PhotoImage(img)
        self.vid_cv.delete('all')
        self.vid_cv.create_image(0,0,anchor='nw',image=self._photo)

    # ── Side panel (was the bottom status bar; now a left column) ──────────────
    def _side_panel(self,parent):
        p=parent
        def pk(w,pady=3):
            w.pack(fill='x',padx=10,pady=pady); return w
        self.lbl_lm =_vallbl(p,'L:  +000',fg=C_BLUE,width=12)
        self.lbl_rm =_vallbl(p,'R:  +000',fg=C_BLUE,width=12)
        # Encoder counts (EL/ER) are read each tick and feed PositionEstimator (see
        # "Arduino firmware protocol"/_read_encoders()) but are no longer shown in the GUI —
        # internal/debug data the user doesn't need to see (removed 2026-10-03 once the
        # position feature built on them was validated).
        self.lbl_pan=_vallbl(p,'PAN:+00°',fg=C_GREEN,width=12)
        self.lbl_tlt=_vallbl(p,'TLT:+00°',fg=C_AMBER,width=12)
        self.lbl_pos=_vallbl(p,'X+0.00 Y+0.00',fg=C_BLUE_HI,width=15)
        self.lbl_hdg=_vallbl(p,'HDG:000°',fg=C_GREEN,width=12)
        pk(self.lbl_lm,(12,3)); pk(self.lbl_rm)
        pk(self.lbl_pan); pk(self.lbl_tlt)
        pk(self.lbl_pos); pk(self.lbl_hdg)
        pk(_btn(p,'⌖  POS',C_SUB,self._reset_pos),(8,4))
        _hline(p)
        self.btn_sim=_btn(p,'▶  SIM',C_SUB,self._toggle_sim); pk(self.btn_sim,(8,4))
        self.sim_loop_var=tk.BooleanVar(value=True)
        pk(tk.Checkbutton(p,text='LOOP',variable=self.sim_loop_var,bg=C_BG,fg=C_SUB,
                          font=('Courier',8),selectcolor=C_PANEL,activebackground=C_BG,
                          activeforeground=C_TEXT,highlightthickness=0),(0,4))
        self.btn_rec=_btn(p,'⚫  REC',C_SUB,self._toggle_rec); pk(self.btn_rec)
        _hline(p)
        pk(_btn(p,'■  STOP',C_RED,self._estop),(8,8))
        _hline(p)
        pk(_btn(p,'CLOSE GUI',C_RED,self._off),(8,8))
        pk(_lbl(p,f'⌨  W/S ±{TRAV_KEY_STEP_CM} cm · A/D ±{ROT_KEY_STEP_DEG}° · SPACE stop',fg=C_SUB,font=('Courier',8),
                justify='center',wraplength=150),(4,4))
        self.lbl_tx=_lbl(p,'',fg=C_SUB,font=('Courier',8),
                         wraplength=150,justify='left',anchor='w')
        pk(self.lbl_tx,(8,4))

    # ── Keyboard ──────────────────────────────────────────────────────────────
    def _bind_keys(self):
        self.root.bind('<KeyPress>',  self._kd)

    def _kd(self,e):
        k=e.keysym.lower()
        if k=='space': self._estop(); return            # always, also while self-driving
        if isinstance(e.widget,tk.Entry): return
        # one step per press; a held key starts the next step when the last one ends
        step={'w':('traverse', TRAV_KEY_STEP_CM),'s':('traverse',-TRAV_KEY_STEP_CM),
              'a':('rotate',   ROT_KEY_STEP_DEG),'d':('rotate',  -ROT_KEY_STEP_DEG)}.get(k)
        if step: self._move_manual(*step)

    def _estop(self):
        """SPACE / STOP: cancels whatever the wheels are doing (a traverse, a rotation, an
        LLM drive). The cancelled action still reports its end pose."""
        self._mover.cancel('stopped by the user')
        self._auto_cancel.set()
        self.left_pwm=self.right_pwm=0

    def _off(self):
        """Closes the GUI app only — does NOT shut down the Jetson itself. Confirms first
        (closing stops the voice listener / frame API / actuation too)."""
        if not messagebox.askyesno('Close GUI', 'Close the robot control GUI?'):
            return
        self._shutdown()

    def _shutdown(self):
        """Every way out (CLOSE GUI, window close, SIGTERM): flushes any active recording, zeros
        and sends a final motor-stop, stops the voice listener, then exits the process."""
        if self._rec_manager.active:
            self._rec_manager.stop()
        if self._mover.active:              # tell the Arduino to stop spinning (its
            self._mover.cancel('GUI closed')   # watchdog would too, 0.5 s later)
            self._mover.step()
        self.left_pwm = self.right_pwm = 0
        self._send_motors()
        self._stop_listener()
        self.root.destroy()
        sys.exit(0)

    def _start_listener(self):
        """One command starts the whole robot: launches the voice listener
        (`functions.py --listen`) as its own process — this one owns sensors/actuators
        and :8787, that one owns decisions and :8788. LISTEN=0 skips it, e.g. for a
        manual functions.py run, which needs :8788 for itself."""
        if os.environ.get('LISTEN','1') == '0':
            print('[voice] listener not started (LISTEN=0)',flush=True)
            return
        script = os.path.join(os.path.dirname(os.path.abspath(__file__)),'functions.py')
        try:
            self.listener = subprocess.Popen([sys.executable,script,'--listen'])
        except OSError as e:
            print(f'[voice] listener not started: {e}',flush=True)
            return
        atexit.register(self._stop_listener)   # also on Ctrl-C or an unhandled exception

    def _stop_listener(self):
        if self.listener is not None and self.listener.poll() is None:
            self.listener.terminate()          # it stops its own llama-server on the way out
            try:
                self.listener.wait(timeout=8)  # llama-server gets up to 5 s of this
            except subprocess.TimeoutExpired:
                self.listener.kill()

    def _reset_pos(self):
        """Zero the position estimate (origin = here). Blocked during sim."""
        if self.sim_mode: return
        self._pos_est.reset(); self._pos_trail.clear(); self._draw_pos_map()

    def _toggle_rec(self):
        if self._rec_manager.active:
            self._rec_manager.stop()
            self._rec_frame_id=0
            self.btn_rec.config(text='⚫  REC',bg=C_SUB)
        else:
            self._pos_est.reset(); self._pos_trail.clear()   # session starts at origin
            self._rec_manager.start()
            self.btn_rec.config(text='🔴  REC',bg=C_RED)

    def _toggle_sim(self):
        if self.sim_mode:                                   # Sim → Real
            self.sim_mode=False; self._sim_ended=False
            if self.sim_player:
                self.sim_player.close(); self.sim_player=None
            # Restore the pre-sim control state so the robot doesn't jump
            # to the last replayed values when real sends resume
            self.pan_angle,self.tilt_angle=self._pre_sim_angles
            self.pan_var.set(self.pan_angle); self.tilt_var.set(tilt_from_raw(self.tilt_angle))
            self.left_pwm=self.right_pwm=0
            self._pos_est.reset(); self._pos_trail.clear()   # real odometry starts fresh
            self.btn_sim.config(text='▶  SIM',bg=C_SUB)
            self.btn_rec.config(state='normal')
            self.lbl_tx.config(text='')
            if self.connected: self.lbl_cs.config(text='⬤  CONNECTED',fg=C_GREEN)
            else:              self.lbl_cs.config(text='⬤  DISCONNECTED',fg=C_RED)
            return
        _repo=os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        path=filedialog.askopenfilename(
            title='Load recording CSV',
            initialdir=os.path.join(_repo,'recordings'),
            filetypes=[('Recording CSV','*.csv'),('All files','*.*')])
        if not path: return                                 # cancelled → stay Real
        try:
            player=SimPlayer(path)
        except Exception as e:
            self.lbl_tx.config(text=f'SIM load failed: {e}'[:44]); return
        if self._rec_manager.active: self._toggle_rec()
        self._estop()                                       # nothing keeps driving into Sim;
        self._mover.abort('Sim mode started')             # the Arduino's watchdog stops a spin
        self._pre_sim_angles=(self.pan_angle,self.tilt_angle)
        self.enc_left=self.enc_right=None   # not replayed
        self.sim_player=player; self.sim_mode=True; self._sim_ended=False
        self._pos_trail.clear()                             # replay draws the recorded path
        self.btn_sim.config(text='⏹  REAL',bg=C_BLUE)
        self.btn_rec.config(state='disabled')
        self.lbl_cs.config(text='⬤  SIM MODE',fg=C_BLUE_HI)

    # ── Arduino I2C ───────────────────────────────────────────────────────────
    def _conn(self):
        try:
            self.i2c=SMBus(1) if SMBUS2 else smbus.SMBus(1)
            if SMBUS2: self.i2c.read_byte(ARDUINO_ADDR)
            else: self.i2c.read_byte_data(ARDUINO_ADDR,0)
            self.connected=True
            self.lbl_cs.config(text='⬤  CONNECTED',fg=C_GREEN)
        except Exception:
            self.i2c=None

    def _disc(self):
        self.connected=False
        if self.i2c:
            try: self.i2c.close()
            except: pass
            self.i2c=None
        self.lbl_cs.config(text='⬤  DISCONNECTED',fg=C_RED)

    def _update_motor_labels(self,l,r,pa,ta):
        self.lbl_lm.config(text=f'L:  {l:+04d}')
        self.lbl_rm.config(text=f'R:  {r:+04d}')
        self.lbl_pan.config(text=f'PAN:{pa:+03d}°')
        self.lbl_tlt.config(text=f'TLT:{tilt_from_raw(ta):+03d}°')   # signed: 0 = level, + = up

    def _straight_trim(self, l, r):
        """Gyro-assisted straight-line correction (PI on yaw error) — see the STRAIGHT_KP/KI
        comments above. Only engages when commanded PWM is equal and nonzero (straight
        forward/backward); any turn (unequal PWM) or stop passes through untouched, drops the
        heading lock, and resets the integral so it doesn't carry over into the next straight
        run."""
        now = time.monotonic()
        if l != r or l == 0 or not self.imu_reader:
            self._straight_target = None
            self._straight_integral = 0.0
            self._straight_t = None
            return l, r
        yaw = self.imu_reader.yaw
        if self._straight_target is None:
            self._straight_target = yaw
            self._straight_integral = 0.0
            self._straight_t = now
            return l, r
        dt = now - self._straight_t; self._straight_t = now
        err = (yaw - self._straight_target + 180) % 360 - 180   # wrap to [-180,180]
        if 0 < dt < 0.5:   # guard stalls/big gaps, same as PositionEstimator
            self._straight_integral = max(-STRAIGHT_I_MAX, min(STRAIGHT_I_MAX,
                                           self._straight_integral + err*dt))
        trim = STRAIGHT_KP*err + STRAIGHT_KI*self._straight_integral
        trim = max(-STRAIGHT_MAX_TRIM, min(STRAIGHT_MAX_TRIM, trim))
        sign = 1 if l > 0 else -1    # reverse correction direction when backing up
        # Sign flipped 2026-10-03 live test: the first guess (l-=trim, r+=trim) was positive
        # feedback — a 4s drive swung heading +83° instead of holding it. This is the opposite.
        l2 = max(-255, min(255, l + int(round(sign*trim))))
        r2 = max(-255, min(255, r - int(round(sign*trim))))
        return l2, r2

    def _send_motors(self):
        l=self.left_pwm; r=self.right_pwm
        pa=self.pan_angle; ta=self.tilt_angle
        self._update_motor_labels(l,r,pa,ta)
        if not(self.connected and self.i2c):
            self.lbl_tx.config(text='(not sent)'); return
        # Positive PWM = forward on the wire too; each wheel's hardware direction is
        # corrected in the firmware's calibration block (LEFT/RIGHT_MOTOR_DIR). l/r are
        # further trimmed by _straight_trim() for the gyro-assisted straight-line correction
        # — self.left_pwm/right_pwm (the commanded intent) stay untouched for everything else
        # (PositionEstimator's PWM fallback, the CSV, the frame API).
        # During a rotation the Arduino owns the wheels: the wire carries 0,0 (or the rotate
        # message itself), while left_pwm/right_pwm hold the nominal spin PWM for display.
        if self._mover.active: l,r = self._straight_trim(0,0)
        else:                    l,r = self._straight_trim(l,r)
        msg=self._mover.wire_message() or f'{l},{r},{pa},{ta}\n'.encode()
        try:
            if SMBUS2:
                for off in range(0,len(msg),32):
                    self.i2c.i2c_rdwr(i2c_msg.write(ARDUINO_ADDR,list(msg[off:off+32])))
            else:
                self.i2c.write_i2c_block_data(ARDUINO_ADDR,0,list(msg))
            self.lbl_tx.config(text=f'TX {msg.decode().strip()}')
        except Exception:
            self._disc()

    def _read_encoders(self):
        # Cumulative wheel-encoder counts read back from the Arduino (smbus2 only).
        # A failed or bad read only blanks the readout — it never disconnects, so an
        # encoder problem can't take motor commands down with it.
        # The same read carries the firmware's rotate status in 3 extra bytes; firmware
        # from before 2026-11-10 sends only the first 10, and the rest fails its check.
        el=er=seq=st=None
        if self.connected and self.i2c and SMBUS2:
            try:
                rd=i2c_msg.read(ARDUINO_ADDR,MOT_PKT_LEN)
                self.i2c.i2c_rdwr(rd)
                pkt=bytes(rd)
                if pkt[0]==ENC_PKT_HDR and sum(pkt[:ENC_PKT_LEN-1])&0xFF==pkt[ENC_PKT_LEN-1]:
                    el,er=struct.unpack('<ii',pkt[1:9])
                    self.fw_motion=((pkt[10]+pkt[11]+MOT_PKT_SALT)&0xFF==pkt[12]
                                    and pkt[11]<=MOT_ABORTED)
                    if self.fw_motion: seq,st=pkt[10],pkt[11]
            except Exception:
                pass
        self.enc_left,self.enc_right=el,er
        self.fw_mot_seq,self.fw_mot_state=seq,st

    # ── Sensor startup ────────────────────────────────────────────────────────
    def _start_sensors(self):
        try:
            self.imu_reader=IMUReader(); self.imu_reader.start()
        except Exception as e:
            self.lbl_is.config(text=f'error: {e}',fg=C_RED)
        self.audio_reader=AudioReader(); self.audio_reader.set_gain_db(self._mic_gain_db)
        self.audio_reader.start()
        self.camera_reader=CameraReader(); self.camera_reader.start()
        self._conn()

    # ── Capture loop (TARGET_FPS, drift-compensated) ──────────────────────────
    def _loop(self):
        t0 = time.monotonic()
        # Actual capture rate = 1 / (time between successive tick starts), smoothed.
        if self._last_tick_t is not None:
            dt = t0 - self._last_tick_t
            if dt > 0:
                inst = 1.0 / dt
                self._fps_ema = inst if self._fps_ema == 0 else 0.2*inst + 0.8*self._fps_ema
        self._last_tick_t = t0

        if self.sim_mode: self._tick_sim()
        else:             self._tick_real()
        self._update_imu_widgets()
        self._update_audio_widgets()
        self._update_video_widgets()
        self._update_position_widgets()
        self._update_drive_widgets()
        if self._rec_manager.active and not self.sim_mode:
            self._rec_manager.enqueue(self.latest_frame, self.latest_frame_bgr)
        self._update_fps_label()
        self._update_voice_widgets()

        # Drift compensation: wait only the remainder of the period, so the actual
        # rate tracks TARGET_FPS instead of (work + fixed delay). If a tick overruns
        # PERIOD_MS we re-arm immediately (min 1 ms) and just run slower that tick.
        elapsed_ms = (time.monotonic() - t0) * 1000.0
        delay = max(1, int(round(PERIOD_MS - elapsed_ms)))
        self.root.after(delay, self._loop)

    def _tick_real(self):
        if not self.connected and time.time()-self._last_reconnect>3:
            self._last_reconnect=time.time()
            self._conn()
        self._mover.step()
        job=self._mover.job
        if job:                                  # nominal PWM, for labels/CSV/sim replay
            self.left_pwm,self.right_pwm=job['nominal']
        elif self._was_rotating:
            self.left_pwm=self.right_pwm=0
        self._was_rotating=job is not None
        self._send_motors()
        self._read_encoders()
        frame_bgr   = self.camera_reader.get_frame()    if self.camera_reader else None
        imu_samples = self.imu_reader.drain_samples()   if self.imu_reader    else []
        audio_samps = self.audio_reader.drain_samples() if self.audio_reader  else []
        yaw_now = (imu_samples[-1][0] if imu_samples
                   else (self.imu_reader.yaw if self.imu_reader else 0.0))
        self._pos_est.update(self.enc_left, self.enc_right,
                              self.left_pwm, self.right_pwm, yaw_now)
        frame = {
            'frame_id':      self._rec_frame_id,
            'timestamp':     datetime.now().isoformat(),
            'frame_data':    '',
            'yaw':           json.dumps([round(s[0],4) for s in imu_samples]),
            'pitch':         json.dumps([round(s[1],4) for s in imu_samples]),
            'roll':          json.dumps([round(s[2],4) for s in imu_samples]),
            'audio_samples': json.dumps([round(v,6) for v in audio_samps]),
            'left_pwm':      self.left_pwm,
            'right_pwm':     self.right_pwm,
            'pan_angle':     self.pan_angle,
            'tilt_angle':    self.tilt_angle,
            'pos_x':         round(self._pos_est.x, 4),
            'pos_y':         round(self._pos_est.y, 4),
            'heading':       round(self._pos_est.heading, 2),
            'mic_gain_db':   self.audio_reader.gain_db if self.audio_reader else 0,
        }
        with self._frame_lock:
            self.latest_frame     = frame
            self.latest_frame_bgr = frame_bgr
            self.latest_decoded   = {'imu': imu_samples, 'audio': audio_samps}
        self._rec_frame_id += 1

    def _tick_sim(self):
        if not self.sim_player: return
        self.sim_player.loop = self.sim_loop_var.get()
        nxt = self.sim_player.next_frame()
        if nxt is None:
            self._sim_ended = True
            self.lbl_tx.config(text='SIM ended')
            return
        if nxt is SimPlayer.WAIT:       # next row not due yet → hold this frame
            return
        self._sim_ended = False
        frame, decoded, frame_bgr = nxt
        with self._frame_lock:
            self.latest_frame     = frame
            self.latest_frame_bgr = frame_bgr
            self.latest_decoded   = decoded
        self._update_motor_labels(decoded['left_pwm'],decoded['right_pwm'],
                                  decoded['pan_angle'],decoded['tilt_angle'])
        # Animate the drive/cam controls from the recorded values
        self.pan_var.set(decoded['pan_angle'])
        self.tilt_var.set(tilt_from_raw(decoded['tilt_angle']))
        sp=self.sim_player
        if sp.total_rows:
            pct=min(100,int(sp.row_index/sp.total_rows*100))
            txt=f'SIM {pct}%'
        else:
            txt=f'SIM row {sp.row_index}'
        if sp.n_skipped: txt+=f' ({sp.n_skipped} skipped)'
        self.lbl_tx.config(text=txt)

    # ── Monitoring widgets (driven by latest_frame snapshot in both modes) ────
    def _update_video_widgets(self):
        if self.sim_mode:
            if self.latest_frame_bgr is not None:
                self.display_frame(self.latest_frame_bgr)
            self.lbl_vs.config(text='SIM ended' if self._sim_ended else 'SIM',fg=C_BLUE_HI)
            return
        if not self.camera_reader: return
        st=self.camera_reader.status
        if st=='running':
            if self.latest_frame_bgr is not None:
                self.display_frame(self.latest_frame_bgr)
                self.lbl_vs.config(text='live',fg=C_GREEN)
        elif st=='stopped':
            self.lbl_vs.config(text='stopped',fg=C_AMBER)
        elif 'error' in str(st):
            self.lbl_vs.config(text=st[:28],fg=C_RED)

    def _update_imu_widgets(self):
        if self.sim_mode:
            self.lbl_is.config(text='SIM ended' if self._sim_ended else 'SIM',fg=C_BLUE_HI)
            if not self._imu_live: self._imu_go_live()
            self._imu_show_last()
            return
        if not self.imu_reader: return
        st=self.imu_reader.status
        if st=='calibrating':
            prog=self.imu_reader.cal_prog
            self.lbl_is.config(text=f'calibrating {prog}%',fg=C_AMBER)
            cv=self._cal_cv; cv.delete('all')
            cv.create_rectangle(0,0,int(prog*3),10,fill=C_BLUE,outline='')
        elif st=='running':
            self.lbl_is.config(text='running',fg=C_GREEN)
            if not self._imu_live: self._imu_go_live()
            self._imu_show_last()
        elif 'error' in str(st):
            self.lbl_is.config(text=st,fg=C_RED)

    def _imu_show_last(self):
        samples=self.latest_decoded.get('imu') or []
        if not samples: return                 # nothing new this tick → hold pose
        y,p,r=samples[-1]
        self._imu_redraw(r,p,y)
        self.lbl_r.config( text=f'Roll: {r:+.1f}°')
        self.lbl_pi.config(text=f'Pitch: {p:+.1f}°')
        self.lbl_y.config( text=f'Yaw: {y:+.1f}°')

    def _update_audio_widgets(self):
        if self.sim_mode:
            self.lbl_as.config(text='SIM ended' if self._sim_ended else 'SIM',fg=C_BLUE_HI)
            self._draw_audio()
            return
        if not self.audio_reader: return
        st=self.audio_reader.status
        if st=='running':
            self.lbl_as.config(text='live',fg=C_GREEN)
            self._draw_audio()
        elif 'error' in str(st):
            self.lbl_as.config(text=st[:28],fg=C_RED)
        elif st=='unavailable':
            self.lbl_as.config(text='sounddevice not installed',fg=C_RED)

    def _draw_audio(self):
        samples=self.latest_decoded.get('audio') or []
        if samples:
            arr=np.asarray(samples,dtype=float)
            if np.max(np.abs(arr))>=0.999: self._clip_t=time.monotonic()   # hit the ±1 clip
            arr=np.clip(arr,-0.5,0.5)  # reject glitches
            raw=float(np.sqrt(np.mean(arr**2)))
            self._rms_ema=0.25*raw+0.75*self._rms_ema   # smooth out spikes
            self._draw_wave(arr[-512:])
        else:
            self._draw_wave(np.zeros(512))
        self._draw_rms(self._rms_ema)
        self._update_gain_label()

    def _update_position_widgets(self):
        """Read pos_x/pos_y/heading from the current snapshot (Real: live estimate,
        Sim: recorded values) and refresh the readout + mini-map."""
        fr=self.latest_frame or {}
        x=fr.get('pos_x',0.0); y=fr.get('pos_y',0.0); hdg=fr.get('heading',0.0)
        if not self._pos_trail or (x,y)!=self._pos_trail[-1]:
            self._pos_trail.append((x,y))
        self.lbl_pos.config(text=f'X{x:+05.2f} Y{y:+05.2f}')
        self.lbl_hdg.config(text=f'HDG:{int(hdg)%360:03d}°')
        self._draw_pos_map()

    def _update_drive_widgets(self):
        """The 'who is moving the robot' badge, the dial following the wheels (knob =
        commanded PWM, ring = rotation progress) and the sliders following servo commands."""
        now=time.monotonic(); job=self._mover.job; auto=self.auto; last=self._mover.last
        note=self.servo_note
        if self.sim_mode:
            txt,fg='SIM REPLAY',C_BLUE_HI
        elif job:
            rot=job['kind']=='rotate'; unit='°' if rot else ' cm'
            done=job['turned'] if rot else job['moved']
            who=(f"SELF-DRIVING  {job['kind']}" if job['source']=='llm'
                 else ('ROTATING' if rot else 'TRAVERSING'))
            txt=f"{who} {job['requested']:+.0f}{unit} · {abs(job['requested']-done):.0f}{unit} to go"
            fg=C_AMBER
        elif auto:
            txt,fg=f"SELF-DRIVING  {auto['label']} · {max(0.0,auto['t_end']-now):.1f} s left",C_AMBER
        elif note and now-note[1]<3:
            txt,fg=f'SELF-DRIVING  {note[0]}',C_AMBER
        elif last and now-last['t']<5:           # how the last motion ended, for a few seconds
            if not last['ok']:            txt,fg=f"{last['kind']}: {last['reason']}"[:52],C_RED
            elif last['kind']=='rotate':  txt,fg=f"turned {last['turned']:+.1f}°",C_GREEN
            else:                         txt,fg=f"moved {last['moved']:+.1f} cm",C_GREEN
        else:
            txt,fg='READY',C_SUB
        self.lbl_auto.config(text=f'⬤  {txt}',fg=fg)
        if not self.sim_mode:
            if self.pan_var.get()!=self.pan_angle: self.pan_var.set(self.pan_angle)
            t=tilt_from_raw(self.tilt_angle)
            if self.tilt_var.get()!=t: self.tilt_var.set(t)

    def _update_fps_label(self):
        fps = self._fps_ema
        ok  = fps >= 0.9 * TARGET_FPS           # green when within 10% of target
        self.lbl_fps.config(text=f'FPS:{fps:4.1f}/{TARGET_FPS}',
                            fg=(C_GREEN if ok else C_AMBER))

    def _listener_stage(self):
        stage = self.listener_status.stage
        if stage == 'off' and self.listener is not None:   # ours, but nothing answering on :8788
            return 'loading' if self.listener.poll() is None else 'error'
        return stage

    def _update_voice_widgets(self):
        stage = self._listener_stage()
        llm_text, llm_fg = {
            'off':       ('LLM: off',     C_SUB),
            'loading':   ('LLM: loading', C_AMBER),
            'error':     ('LLM: error',   C_RED),
        }.get(stage, ('LLM: ready', C_GREEN))
        self.lbl_llm.config(text=f'⬤  {llm_text}', fg=llm_fg)

        voice_text, voice_fg = {
            'off':       ('MIC: off',       C_SUB),
            'loading':   ('MIC: loading',   C_AMBER),
            'error':     ('MIC: error',     C_RED),
            'asleep':    ('MIC: asleep',    C_SUB),
            'listening': ('MIC: listening', C_GREEN),
            'thinking':  ('MIC: thinking',  C_AMBER),
            'acting':    ('MIC: acting',    C_AMBER),
            'speaking':  ('MIC: speaking',  C_BLUE_HI),
            'initiation':('MIC: initiation',C_AMBER),   # busy with the routine, not listening
        }.get(stage, (f'MIC: {stage}', C_SUB))
        self.lbl_voice.config(text=f'⬤  {voice_text}', fg=voice_fg)

        last_action = self.listener_status.last_action
        if last_action:
            self.lbl_voice_action.config(text=last_action[:120], fg=C_GREEN)


# ── Entry point ───────────────────────────────────────────────────────────────
if __name__=='__main__':
    root=tk.Tk()
    app=RobotControlGUI(root)
    root.mainloop()
