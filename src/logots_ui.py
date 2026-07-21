#!/usr/bin/env python3
"""
Logots Robot Control — Full Monitoring GUI
Panels: Drive + Arm  |  IMU orientation  |  Audio waveform  |  Video feed
"""

import tkinter as tk
from tkinter import filedialog
import math
import sys
import time
import threading
import subprocess
import os
import csv
import queue
import json
import collections
import io
import base64
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
IMU_I2C_BUS   = 7
MPU_ADDR      = 0x68
AUDIO_RATE    = 16000
AUDIO_BLOCK   = 512

PWR_MGMT_1 = 0x6B;  CONFIG_REG = 0x1A
GYRO_CFG   = 0x1B;  ACCEL_CFG  = 0x1C;  ACCEL_CFG2 = 0x1D
ACCEL_OUT  = 0x3B;  GYRO_OUT   = 0x43
ACCEL_SCALE = 16384.0;  GYRO_SCALE = 131.0

# ── Position estimation ───────────────────────────────────────────────────────
# PWM dead-reckoning: forward speed is modelled as proportional to the commanded
# average PWM, direction taken from the IMU yaw. This is a MODEL, not measured
# odometry (no wheel encoders) — accuracy depends on calibrating ROBOT_MAX_SPEED_MPS
# on the real robot with the motors connected.
ROBOT_MAX_SPEED_MPS = 0.30            # forward speed at |PWM|=255 — GUESS, calibrate
K_V = ROBOT_MAX_SPEED_MPS / 255.0     # m/s per PWM unit

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
    """Dead-reckons body (x, y) in metres from commanded PWM + IMU heading.

    Forward speed v = k_v · (left_pwm + right_pwm)/2; heading θ from IMU yaw.
    Integrated each tick: x += v·cosθ·dt, y += v·sinθ·dt. Pure spins (L=+, R=−)
    average to ~0 forward speed, so heading changes without translation."""

    def __init__(self, k_v=K_V):
        self.k_v = k_v
        self.reset()

    def reset(self):
        self.x = self.y = self.heading = 0.0
        self._t = None

    def update(self, left_pwm, right_pwm, yaw_deg, now=None):
        now = now if now is not None else time.monotonic()
        if self._t is None:                 # first sample: seed time + heading only
            self._t = now; self.heading = yaw_deg; return
        dt = now - self._t; self._t = now
        if dt <= 0 or dt > 0.5: return      # guard stalls / first big gap
        v  = self.k_v * (left_pwm + right_pwm) / 2.0
        th = math.radians(yaw_deg)
        self.x += v * math.cos(th) * dt
        self.y += v * math.sin(th) * dt
        self.heading = yaw_deg

# ── Audio reader ──────────────────────────────────────────────────────────────
class AudioReader:
    def __init__(self):
        self._stream   = None
        self.status    = 'stopped'
        self._drain_buf  = []
        self._drain_lock = threading.Lock()

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
        with self._drain_lock:
            self._drain_buf.extend(indata[:, 0].tolist())

    def drain_samples(self):
        with self._drain_lock:
            out, self._drain_buf = self._drain_buf, []
        return out

    def stop(self):
        if self._stream:
            try: self._stream.stop(); self._stream.close()
            except: pass

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
            f'! "video/x-raw(memory:NVMM),width={self.W},height={self.H},framerate=30/1" '
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
    FIELDNAMES = CORE_FIELDNAMES + ['pos_x', 'pos_y', 'heading']

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
        self._open()

    def _open(self):
        self._file   = open(self.csv_path, 'r', newline='')
        self._reader = csv.DictReader(self._file)
        fields = self._reader.fieldnames or []
        if not set(RecordingManager.CORE_FIELDNAMES) <= set(fields):
            self._file.close()
            raise ValueError('not a Logots recording CSV')
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
            'pan_angle':  num('pan_angle', 90),
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

def _vallbl(p,txt,fg=C_BLUE,width=0):
    return tk.Label(p,text=txt,bg=C_PANEL,fg=fg,font=('Courier',11,'bold'),
                    padx=8,pady=4,width=width)

def _btn(p,txt,color,cmd,**kw):
    return tk.Button(p,text=txt,bg=color,fg=C_TEXT,relief='flat',
                     font=('Courier',10,'bold'),padx=12,pady=3,command=cmd,
                     cursor='hand2',activebackground=color,activeforeground=C_TEXT,**kw)

# ── Main GUI ──────────────────────────────────────────────────────────────────
class RobotControlGUI:
    JOY_R = 108
    JOY_T = 20

    def __init__(self, root):
        self.root=root
        root.title('Logots — Robot Control')
        root.configure(bg=C_BG)
        root.resizable(False,False)

        self.joy_x=self.joy_y=0.0
        self.joy_dragging=False
        self.keys_held=set()
        self.left_pwm=self.right_pwm=0
        self.pan_angle=90
        self.tilt_angle=90

        self._pos_est   = PositionEstimator()
        self._pos_trail = collections.deque(maxlen=400)  # (x,y) points for the mini-map

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

        self._build_ui()
        self._bind_keys()
        self._start_sensors()
        try:
            self._frame_server = FrameServer(self)
        except Exception as e:
            print(f'[logots] frame API server not started: {e}')
            self._frame_server = None
        self._loop()

    # ── Build UI ──────────────────────────────────────────────────────────────
    def _build_ui(self):
        self._hdr(); _hline(self.root)
        self._main_panels()
        _hline(self.root)
        self._status_bar()

    def _hdr(self):
        f=tk.Frame(self.root,bg=C_BG)
        f.pack(fill='x',padx=16,pady=(12,0))
        _lbl(f,'LOGOTS  ROBOT CONTROL',fg=C_TEXT,font=('Courier',14,'bold')).pack(side='left')
        self.lbl_cs=_lbl(f,'⬤  DISCONNECTED',fg=C_RED,font=('Courier',9,'bold'))
        self.lbl_cs.pack(side='right')


    def _main_panels(self):
        f=tk.Frame(self.root,bg=C_BG)
        f.pack(padx=16,pady=4)
        # Fixed-size 2×2 grid so all panels align perfectly
        LW=375; RW=415; TH=310; BH=220; GAP=6
        p_ctrl  = tk.Frame(f,bg=C_PANEL,width=LW,height=TH)
        p_imu   = tk.Frame(f,bg=C_PANEL,width=RW,height=TH)
        p_audio = tk.Frame(f,bg=C_PANEL,width=LW,height=BH)
        p_video = tk.Frame(f,bg=C_PANEL,width=RW,height=BH)
        for p in (p_ctrl,p_imu,p_audio,p_video):
            p.pack_propagate(False)
        p_ctrl.grid( row=0,column=0,padx=(0,GAP),pady=(0,GAP),sticky='nsew')
        p_imu.grid(  row=0,column=1,pady=(0,GAP),sticky='nsew')
        p_audio.grid(row=1,column=0,padx=(0,GAP),sticky='nsew')
        p_video.grid(row=1,column=1,sticky='nsew')
        self._ctrl_panel(p_ctrl)
        self._audio_panel(p_audio)
        self._imu_panel(p_imu)
        self._video_panel(p_video)

    # ── Control panel ─────────────────────────────────────────────────────────
    def _ctrl_panel(self,parent):
        p=parent
        _plbl(p,'DRIVE  &  CAM',font=('Courier',9,'bold')).pack(pady=(8,4))
        row=tk.Frame(p,bg=C_PANEL); row.pack(padx=10,pady=(0,8))
        self._joystick(row)
        rc=tk.Frame(row,bg=C_PANEL); rc.pack(side='left')   # sliders on top, map below
        sl=tk.Frame(rc,bg=C_PANEL); sl.pack(side='top')
        self._cam_sliders(sl)
        self._position_map(rc)

    def _joystick(self,parent):
        sz=(self.JOY_R+14)*2
        self.joy_cv=tk.Canvas(parent,width=sz,height=sz,bg=C_PANEL,highlightthickness=0)
        self.joy_cv.pack(side='left',padx=(4,10))
        self._draw_joy()
        self.joy_cv.bind('<ButtonPress-1>',  self._jp)
        self.joy_cv.bind('<B1-Motion>',      self._jm)
        self.joy_cv.bind('<ButtonRelease-1>',self._jr)

    def _draw_joy(self):
        cv=self.joy_cv; R=self.JOY_R; tr=self.JOY_T; cx=cy=R+14
        cv.delete('all')
        cv.create_oval(cx-R,cy-R,cx+R,cy+R,fill='#16162a',outline=C_BORDER,width=2)
        for f in (.5,1.): r2=R*f; cv.create_oval(cx-r2,cy-r2,cx+r2,cy+r2,outline=C_BORDER,fill='')
        cv.create_line(cx-R+8,cy,cx+R-8,cy,fill=C_BORDER)
        cv.create_line(cx,cy-R+8,cx,cy+R-8,fill=C_BORDER)
        mr=R-tr-2; tx=cx+self.joy_x*mr; ty=cy-self.joy_y*mr
        g=tr+7; cv.create_oval(tx-g,ty-g,tx+g,ty+g,fill='#0d2a40',outline='')
        fc=C_BLUE_HI if self.joy_dragging else C_BLUE
        cv.create_oval(tx-tr,ty-tr,tx+tr,ty+tr,fill=fc,outline='#5bb3e8')
        cv.create_oval(tx-4,ty-4,tx+4,ty+4,fill='#cce8ff',outline='')
        for txt,x,y in [('FWD',cx,cy-R+8),('BWD',cx,cy+R-8),('L',cx-R+8,cy),('R',cx+R-8,cy)]:
            cv.create_text(x,y,text=txt,fill=C_SUB,font=('Courier',7))

    def _jp(self,e):
        if self.sim_mode: return
        self.joy_dragging=True;  self._jxy(e.x,e.y)
    def _jm(self,e):
        if self.sim_mode: return
        self._jxy(e.x,e.y)
    def _jr(self,e):
        if self.sim_mode: return
        self.joy_dragging=False; self.joy_x=self.joy_y=0.0
        self._pwm(); self._draw_joy()
    def _jxy(self,mx,my):
        cx=cy=self.JOY_R+14; dx,dy=mx-cx,my-cy
        lim=self.JOY_R-self.JOY_T-2; d=math.hypot(dx,dy)
        if d>lim: dx=dx/d*lim; dy=dy/d*lim
        self.joy_x=dx/lim; self.joy_y=-dy/lim
        self._pwm(); self._draw_joy()

    def _cam_sliders(self,parent):
        for attr,label in [('pan','PAN'),('tilt','TILT')]:
            f=tk.Frame(parent,bg=C_PANEL); f.pack(side='left',padx=2)
            _plbl(f,label,font=('Courier',8,'bold')).pack(pady=(4,0))
            _plbl(f,'180°',font=('Courier',7)).pack()
            var=tk.IntVar(value=90)
            setattr(self,f'{attr}_var',var)
            tk.Scale(f,from_=180,to=0,variable=var,orient='vertical',
                     length=120,width=16,bg=C_PANEL,fg=C_TEXT,troughcolor='#16162a',
                     activebackground=C_BLUE_HI,highlightthickness=0,bd=0,
                     sliderrelief='flat',sliderlength=20,showvalue=True,
                     command=lambda v,a=attr:None if self.sim_mode
                             else setattr(self,f'{a}_angle',int(v))).pack()
            _plbl(f,'0°',font=('Courier',7)).pack(pady=(0,4))

    # ── Position mini-map ─────────────────────────────────────────────────────
    POS_MAP = 118   # canvas size (px) for the top-down trail

    def _position_map(self,parent):
        _plbl(parent,'POSITION (m)',font=('Courier',7,'bold')).pack(pady=(2,0))
        self.pos_cv=tk.Canvas(parent,width=self.POS_MAP,height=self.POS_MAP,
                              bg='#16162a',highlightthickness=1,
                              highlightbackground=C_BORDER)
        self.pos_cv.pack(pady=(1,0))
        self._draw_pos_map()

    def _draw_pos_map(self):
        """Auto-scaled top-down view of the trail; +X right, +Y up, heading arrow."""
        cv=self.pos_cv; S=self.POS_MAP; cv.delete('all')
        pts=list(self._pos_trail)
        # world bounds (always include origin), with a minimum span so a still
        # robot doesn't zoom to infinity
        xs=[p[0] for p in pts]+[0.0]; ys=[p[1] for p in pts]+[0.0]
        xmin,xmax=min(xs),max(xs); ymin,ymax=min(ys),max(ys)
        span=max(xmax-xmin, ymax-ymin, 0.5); pad=span*0.15+1e-6
        cx=(xmin+xmax)/2; cy=(ymin+ymax)/2
        half=span/2+pad
        def to_px(wx,wy):
            px=(wx-cx)/(2*half)*(S-8)+S/2
            py=S/2-(wy-cy)/(2*half)*(S-8)   # +Y up
            return px,py
        # grid crosshair through origin
        ox,oy=to_px(0.0,0.0)
        cv.create_line(0,oy,S,oy,fill='#25253a')
        cv.create_line(ox,0,ox,S,fill='#25253a')
        cv.create_oval(ox-2,oy-2,ox+2,oy+2,outline=C_SUB,fill='')  # origin marker
        # trail polyline
        if len(pts)>=2:
            flat=[]
            for wx,wy in pts:
                px,py=to_px(wx,wy); flat+=[px,py]
            cv.create_line(*flat,fill=C_BLUE,width=1)
        # current position + heading arrow
        if pts:
            wx,wy=pts[-1]; px,py=to_px(wx,wy)
            th=math.radians(self.latest_frame.get('heading',0.0) if self.latest_frame else 0.0)
            ax=px+10*math.cos(th); ay=py-10*math.sin(th)
            cv.create_line(px,py,ax,ay,fill=C_GREEN,width=2,arrow='last')
            cv.create_oval(px-3,py-3,px+3,py+3,fill=C_BLUE_HI,outline='')

    # ── Audio panel ───────────────────────────────────────────────────────────
    def _audio_panel(self,parent):
        p=parent
        h=tk.Frame(p,bg=C_PANEL); h.pack(fill='x',padx=8,pady=(6,2))
        _plbl(h,'AUDIO INPUT',font=('Courier',9,'bold')).pack(side='left')
        self.lbl_as=_plbl(h,'',font=('Courier',8),fg=C_AMBER); self.lbl_as.pack(side='right')
        self.wav_cv=tk.Canvas(p,width=356,height=90,bg='#16162a',highlightthickness=0)
        self.wav_cv.pack(padx=8,pady=(0,4))
        self._wave_idle()
        br=tk.Frame(p,bg=C_PANEL); br.pack(fill='x',padx=8,pady=(0,8))
        _plbl(br,'RMS',font=('Courier',8)).pack(side='left')
        self.rms_cv=tk.Canvas(br,width=286,height=8,bg='#16162a',highlightthickness=0)
        self.rms_cv.pack(side='left',padx=6)

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

        # YPR labels (packed after calibration)
        self._ypr_frame=tk.Frame(p,bg=C_PANEL)
        self.lbl_r=_vallbl(self._ypr_frame,'Roll:  +0.0°',fg=C_RED)
        self.lbl_pi=_vallbl(self._ypr_frame,'Pitch: +0.0°',fg=C_GREEN)
        self.lbl_y=_vallbl(self._ypr_frame,'Yaw:   +0.0°',fg=C_BLUE)
        for w in (self.lbl_r,self.lbl_pi,self.lbl_y): w.pack(side='left',padx=2)

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

    # ── Status bar ────────────────────────────────────────────────────────────
    def _status_bar(self):
        f=tk.Frame(self.root,bg=C_BG); f.pack(fill='x',padx=16,pady=(4,14))
        self.lbl_lm =_vallbl(f,'L:  +000',fg=C_BLUE,width=8)
        self.lbl_rm =_vallbl(f,'R:  +000',fg=C_BLUE,width=8)
        self.lbl_pan=_vallbl(f,'PAN:090°',fg=C_GREEN,width=8)
        self.lbl_tlt=_vallbl(f,'TLT:090°',fg=C_AMBER,width=8)
        self.lbl_pos=_vallbl(f,'X+0.00 Y+0.00',fg=C_BLUE_HI,width=15)
        self.lbl_hdg=_vallbl(f,'HDG:000°',fg=C_GREEN,width=9)
        self.lbl_lm.pack(side='left',padx=(0,5))
        self.lbl_rm.pack(side='left',padx=(0,5))
        self.lbl_pan.pack(side='left',padx=(0,5))
        self.lbl_tlt.pack(side='left',padx=(0,5))
        self.lbl_pos.pack(side='left',padx=(0,5))
        self.lbl_hdg.pack(side='left',padx=(0,5))
        _btn(f,'⌖  POS',C_SUB,self._reset_pos,width=7).pack(side='left',padx=(0,5))
        _btn(f,'■  STOP',C_RED,self._estop).pack(side='right')
        self.btn_rec=_btn(f,'⚫  REC',C_SUB,self._toggle_rec); self.btn_rec.pack(side='right',padx=(0,6))
        self.btn_sim=_btn(f,'▶  SIM',C_SUB,self._toggle_sim,width=7)
        self.btn_sim.pack(side='right',padx=(0,6))
        self.sim_loop_var=tk.BooleanVar(value=True)
        tk.Checkbutton(f,text='LOOP',variable=self.sim_loop_var,bg=C_BG,fg=C_SUB,
                       font=('Courier',8),selectcolor=C_PANEL,activebackground=C_BG,
                       activeforeground=C_TEXT,highlightthickness=0).pack(side='right',padx=(0,6))
        self.lbl_tx=_lbl(f,'',fg=C_SUB,font=('Courier',8),width=26,anchor='e')
        self.lbl_tx.pack(side='right',padx=10)
        _lbl(f,'⌨  W/S  A/D  SPACE',fg=C_SUB,font=('Courier',8)).pack(side='left',padx=10)

    # ── Keyboard ──────────────────────────────────────────────────────────────
    def _bind_keys(self):
        self.root.bind('<KeyPress>',  self._kd)
        self.root.bind('<KeyRelease>',self._ku)

    def _kd(self,e):
        if self.sim_mode: return
        self.keys_held.add(e.keysym.lower());  self._k2j()
    def _ku(self,e):
        if self.sim_mode: return
        self.keys_held.discard(e.keysym.lower()); self._k2j()
    def _k2j(self):
        if 'space' in self.keys_held:
            self.joy_x=self.joy_y=0.0
        else:
            self.joy_y=float('w' in self.keys_held)-float('s' in self.keys_held)
            self.joy_x=float('d' in self.keys_held)-float('a' in self.keys_held)
            self.joy_x=max(-1,min(1,self.joy_x)); self.joy_y=max(-1,min(1,self.joy_y))
        self._pwm(); self._draw_joy()

    # ── Motor mixing ──────────────────────────────────────────────────────────
    def _pwm(self):
        t=self.joy_y; turn=self.joy_x
        l=t+turn; r=t-turn; s=max(abs(l),abs(r),1.0)
        self.left_pwm=int(l/s*255); self.right_pwm=int(r/s*255)

    def _estop(self):
        self.joy_x=self.joy_y=0.0; self.left_pwm=self.right_pwm=0; self._draw_joy()

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
            self.pan_var.set(self.pan_angle); self.tilt_var.set(self.tilt_angle)
            self.joy_x=self.joy_y=0.0; self.left_pwm=self.right_pwm=0
            self._draw_joy()
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
        self._pre_sim_angles=(self.pan_angle,self.tilt_angle)
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
        self.lbl_pan.config(text=f'PAN:{pa:03d}°')
        self.lbl_tlt.config(text=f'TLT:{ta:03d}°')

    def _send_motors(self):
        l=self.left_pwm; r=self.right_pwm
        pa=self.pan_angle; ta=self.tilt_angle
        self._update_motor_labels(l,r,pa,ta)
        if not(self.connected and self.i2c):
            self.lbl_tx.config(text='(not sent)'); return
        msg=f'{l},{r},{pa},{ta}\n'.encode()
        try:
            if SMBUS2:
                for off in range(0,len(msg),32):
                    self.i2c.i2c_rdwr(i2c_msg.write(ARDUINO_ADDR,list(msg[off:off+32])))
            else:
                self.i2c.write_i2c_block_data(ARDUINO_ADDR,0,list(msg))
            self.lbl_tx.config(text=f'TX {msg.decode().strip()}')
        except Exception:
            self._disc()

    # ── Sensor startup ────────────────────────────────────────────────────────
    def _start_sensors(self):
        try:
            self.imu_reader=IMUReader(); self.imu_reader.start()
        except Exception as e:
            self.lbl_is.config(text=f'error: {e}',fg=C_RED)
        self.audio_reader=AudioReader(); self.audio_reader.start()
        self.camera_reader=CameraReader(); self.camera_reader.start()
        self._conn()

    # ── 20 Hz update loop ─────────────────────────────────────────────────────
    def _loop(self):
        if self.sim_mode: self._tick_sim()
        else:             self._tick_real()
        self._update_imu_widgets()
        self._update_audio_widgets()
        self._update_video_widgets()
        self._update_position_widgets()
        if self._rec_manager.active and not self.sim_mode:
            self._rec_manager.enqueue(self.latest_frame, self.latest_frame_bgr)
        self.root.after(50,self._loop)

    def _tick_real(self):
        if not self.connected and time.time()-self._last_reconnect>3:
            self._last_reconnect=time.time()
            self._conn()
        self._send_motors()
        frame_bgr   = self.camera_reader.get_frame()    if self.camera_reader else None
        imu_samples = self.imu_reader.drain_samples()   if self.imu_reader    else []
        audio_samps = self.audio_reader.drain_samples() if self.audio_reader  else []
        yaw_now = (imu_samples[-1][0] if imu_samples
                   else (self.imu_reader.yaw if self.imu_reader else 0.0))
        self._pos_est.update(self.left_pwm, self.right_pwm, yaw_now)
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
        self.tilt_var.set(decoded['tilt_angle'])
        l,r=decoded['left_pwm'],decoded['right_pwm']
        self.joy_y=max(-1.0,min(1.0,(l+r)/510.0))   # inverse of _pwm mixing
        self.joy_x=max(-1.0,min(1.0,(l-r)/510.0))
        self._draw_joy()
        txt=f'SIM row {self.sim_player.row_index}'
        if self.sim_player.n_skipped: txt+=f' ({self.sim_player.n_skipped} skipped)'
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
        self.lbl_r.config( text=f'Roll:  {r:+6.1f}°')
        self.lbl_pi.config(text=f'Pitch: {p:+6.1f}°')
        self.lbl_y.config( text=f'Yaw:   {y:+6.1f}°')

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
            arr=np.clip(np.asarray(samples,dtype=float),-0.5,0.5)  # reject glitches
            raw=float(np.sqrt(np.mean(arr**2)))
            self._rms_ema=0.25*raw+0.75*self._rms_ema   # smooth out spikes
            self._draw_wave(arr[-512:])
        else:
            self._draw_wave(np.zeros(512))
        self._draw_rms(self._rms_ema)

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


# ── Entry point ───────────────────────────────────────────────────────────────
if __name__=='__main__':
    root=tk.Tk()
    app=RobotControlGUI(root)
    root.mainloop()
