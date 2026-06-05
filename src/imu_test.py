#!/usr/bin/env python3
"""
IMU orientation visualizer — MPU-9250 on I2C bus 7
Chip orientation: X right, Y forward, Z up (as printed on board)
Fixes vs v1: hardware DLPF enabled, variable dt in Madgwick, calibration at correct rate.
"""

try:
    import smbus2 as smbus
except ImportError:
    import smbus

import time
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# ── I2C / register constants ──────────────────────────────────────────────────
I2C_BUS     = 7
MPU_ADDR    = 0x68

PWR_MGMT_1  = 0x6B
CONFIG_REG  = 0x1A   # DLPF config
GYRO_CFG    = 0x1B
ACCEL_CFG   = 0x1C
ACCEL_CFG2  = 0x1D   # accel DLPF
ACCEL_OUT   = 0x3B
GYRO_OUT    = 0x43

ACCEL_SCALE = 16384.0   # ±2g  → g
GYRO_SCALE  = 131.0     # ±250 °/s → °/s

IMU_RATE    = 50.0   # used only for calibration progress display

# ── Hardware init ─────────────────────────────────────────────────────────────
bus = smbus.SMBus(I2C_BUS)

def init_mpu():
    bus.write_byte_data(MPU_ADDR, PWR_MGMT_1, 0x00)    # wake up
    time.sleep(0.1)
    bus.write_byte_data(MPU_ADDR, PWR_MGMT_1, 0x01)    # use gyro clock
    bus.write_byte_data(MPU_ADDR, CONFIG_REG,  0x02)   # gyro DLPF  92 Hz BW
    bus.write_byte_data(MPU_ADDR, ACCEL_CFG,   0x00)   # ±2g
    bus.write_byte_data(MPU_ADDR, ACCEL_CFG2,  0x04)   # accel DLPF 20 Hz BW
    bus.write_byte_data(MPU_ADDR, GYRO_CFG,    0x00)   # ±250 °/s
    time.sleep(0.1)

# ── Low-level reads ───────────────────────────────────────────────────────────
def _s16(hi, lo):
    v = (hi << 8) | lo
    return v - 65536 if v > 32767 else v

def read_accel():
    d = bus.read_i2c_block_data(MPU_ADDR, ACCEL_OUT, 6)
    return (_s16(d[0], d[1]) / ACCEL_SCALE,
            _s16(d[2], d[3]) / ACCEL_SCALE,
            _s16(d[4], d[5]) / ACCEL_SCALE)

def read_gyro():
    d = bus.read_i2c_block_data(MPU_ADDR, GYRO_OUT, 6)
    return (_s16(d[0], d[1]) / GYRO_SCALE,
            _s16(d[2], d[3]) / GYRO_SCALE,
            _s16(d[4], d[5]) / GYRO_SCALE)

# ── Madgwick AHRS (IMU mode — no magnetometer) ────────────────────────────────
# Ported from MadgwickAHRS Arduino library, beta=0.1, variable dt
class Madgwick:
    def __init__(self, beta=0.1):
        self.beta = beta
        self.q    = np.array([1.0, 0.0, 0.0, 0.0])

    def update_imu(self, gx, gy, gz, ax, ay, az, dt):
        """gyro in °/s, accel in g, dt in seconds"""
        q = self.q
        gx = math.radians(gx)
        gy = math.radians(gy)
        gz = math.radians(gz)

        norm = math.sqrt(ax*ax + ay*ay + az*az)
        if norm == 0:
            return
        ax /= norm; ay /= norm; az /= norm

        _2q0 = 2.0*q[0]; _2q1 = 2.0*q[1]
        _2q2 = 2.0*q[2]; _2q3 = 2.0*q[3]
        _4q0 = 4.0*q[0]; _4q1 = 4.0*q[1]; _4q2 = 4.0*q[2]
        _8q1 = 8.0*q[1]; _8q2 = 8.0*q[2]
        q0q0 = q[0]*q[0]; q1q1 = q[1]*q[1]
        q2q2 = q[2]*q[2]; q3q3 = q[3]*q[3]

        s0 = _4q0*q2q2 + _2q2*ax + _4q0*q1q1 - _2q1*ay
        s1 = (_4q1*q3q3 - _2q3*ax + 4.0*q0q0*q[1] - _2q0*ay
              - _4q1 + _8q1*q1q1 + _8q1*q2q2 + _4q1*az)
        s2 = (4.0*q0q0*q[2] + _2q0*ax + _4q2*q3q3 - _2q3*ay
              - _4q2 + _8q2*q1q1 + _8q2*q2q2 + _4q2*az)
        s3 = 4.0*q1q1*q[3] - _2q1*ax + 4.0*q2q2*q[3] - _2q2*ay

        norm = math.sqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3)
        if norm > 0:
            s0 /= norm; s1 /= norm; s2 /= norm; s3 /= norm

        qDot0 = 0.5*(-q[1]*gx - q[2]*gy - q[3]*gz) - self.beta*s0
        qDot1 = 0.5*( q[0]*gx + q[2]*gz - q[3]*gy) - self.beta*s1
        qDot2 = 0.5*( q[0]*gy - q[1]*gz + q[3]*gx) - self.beta*s2
        qDot3 = 0.5*( q[0]*gz + q[1]*gy - q[2]*gx) - self.beta*s3

        q[0] += qDot0*dt; q[1] += qDot1*dt
        q[2] += qDot2*dt; q[3] += qDot3*dt

        norm = math.sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3])
        self.q = q / norm

    def get_yaw_pitch_roll(self):
        q = self.q
        yaw   = math.degrees(math.atan2(
                    2.0*(q[0]*q[3] + q[1]*q[2]),
                    1.0 - 2.0*(q[2]*q[2] + q[3]*q[3])))
        pitch = math.degrees(math.asin(
                    max(-1.0, min(1.0, 2.0*(q[0]*q[2] - q[3]*q[1])))))
        roll  = math.degrees(math.atan2(
                    2.0*(q[0]*q[1] + q[2]*q[3]),
                    1.0 - 2.0*(q[1]*q[1] + q[2]*q[2])))
        return yaw, pitch, roll

# ── Calibration at proper 50 Hz pace ─────────────────────────────────────────
def calibrate(filt, samples=500):
    print(f"Step 1/2 — gyro bias ({samples} samples, ~{samples/IMU_RATE:.0f}s) keep IMU still…")
    sx = sy = sz = 0.0
    for i in range(samples):
        t0 = time.time()
        gx, gy, gz = read_gyro()
        sx += gx; sy += gy; sz += gz
        if i % 50 == 0:
            print(f"  {i}/{samples}", end='\r')
    bias = (sx/samples, sy/samples, sz/samples)
    print(f"\n  bias → gx={bias[0]:+.4f}  gy={bias[1]:+.4f}  gz={bias[2]:+.4f}")

    print(f"Step 2/2 — YPR offsets ({samples} samples) keep IMU still…")
    sy2 = sp2 = sr2 = 0.0
    t_prev = time.time()
    for i in range(samples):
        ax, ay, az = read_accel()
        gx, gy, gz = read_gyro()
        gx -= bias[0]; gy -= bias[1]; gz -= bias[2]
        t_now  = time.time()
        dt     = t_now - t_prev
        t_prev = t_now
        filt.update_imu(gx, gy, gz, ax, ay, az, dt)
        y, p, r = filt.get_yaw_pitch_roll()
        sy2 += y; sp2 += p; sr2 += r
        if i % 50 == 0:
            print(f"  {i}/{samples}", end='\r')
    offsets = (sy2/samples, sp2/samples, sr2/samples)
    print(f"\n  offsets → yaw={offsets[0]:+.2f}°  pitch={offsets[1]:+.2f}°  roll={offsets[2]:+.2f}°")
    print("Calibration done.\n")
    return bias, offsets

# ── 3-D board geometry ────────────────────────────────────────────────────────
# Chip is X-right, Y-forward, Z-up — long axis along Y (forward)
BOARD_VERTS = np.array([
    [-0.8, -1.5, -0.1], [ 0.8, -1.5, -0.1],
    [ 0.8,  1.5, -0.1], [-0.8,  1.5, -0.1],
    [-0.8, -1.5,  0.1], [ 0.8, -1.5,  0.1],
    [ 0.8,  1.5,  0.1], [-0.8,  1.5,  0.1],
])

FACES = [
    [0,1,2,3], [4,5,6,7],   # bottom / top   (blue)
    [4,5,1,0], [6,7,3,2],   # front / back   (green  — Y axis ends)
    [0,3,7,4], [1,2,6,5],   # left / right   (red    — X axis ends)
]
FACE_COLORS = ['#1a7abf','#1a7abf', '#27ae60','#27ae60', '#c0392b','#c0392b']

def rot_matrix(r_deg, p_deg, y_deg):
    r = math.radians(r_deg); p = math.radians(p_deg); y = math.radians(y_deg)
    Rx = np.array([[1,0,0],[0,math.cos(r),-math.sin(r)],[0,math.sin(r),math.cos(r)]])
    Ry = np.array([[math.cos(p),0,math.sin(p)],[0,1,0],[-math.sin(p),0,math.cos(p)]])
    Rz = np.array([[math.cos(y),-math.sin(y),0],[math.sin(y),math.cos(y),0],[0,0,1]])
    return Rz @ Ry @ Rx

# ── Matplotlib figure ─────────────────────────────────────────────────────────
BG = '#1e1e2e'
fig = plt.figure(figsize=(9, 7), facecolor=BG)
ax  = fig.add_subplot(111, projection='3d')
ax.set_facecolor(BG)
fig.subplots_adjust(left=0, right=1, bottom=0.12, top=0.92)

for pane in (ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane):
    pane.fill = False; pane.set_edgecolor('#333')

ax.set_xlim(-2.5, 2.5); ax.set_ylim(-2.5, 2.5); ax.set_zlim(-2.5, 2.5)
ax.set_xlabel('X →right',  color='#c0392b', labelpad=6)
ax.set_ylabel('Y →fwd',    color='#27ae60', labelpad=6)
ax.set_zlabel('Z →up',     color='#1a7abf', labelpad=6)
ax.tick_params(colors='#555')
ax.set_title('IMU Orientation  —  MPU-9250  (Madgwick + DLPF)', color='white', fontsize=13, pad=10)

angle_txt = fig.text(
    0.02, 0.03,
    'Roll:  +0.00°\nPitch: +0.00°\nYaw:   +0.00°',
    color='white', fontsize=12, fontfamily='monospace',
    verticalalignment='bottom',
    bbox=dict(boxstyle='round,pad=0.5', facecolor='#2a2a3e', alpha=0.85)
)

poly_col   = [None]
_state     = {}   # holds filter, bias, offsets, t_prev

def animate(_frame):
    t_now  = time.time()
    dt     = t_now - _state['t_prev']
    _state['t_prev'] = t_now

    ax_v, ay_v, az_v = read_accel()
    gx, gy, gz       = read_gyro()
    bx, by, bz       = _state['bias']
    gx -= bx; gy -= by; gz -= bz

    _state['filt'].update_imu(gx, gy, gz, ax_v, ay_v, az_v, dt)
    yaw, pitch, roll = _state['filt'].get_yaw_pitch_roll()

    oy, op, or_  = _state['offsets']
    yaw   -= oy;  pitch -= op;  roll  -= or_

    yaw = math.fmod(yaw + 360.0, 360.0)
    if pitch >  180.0: pitch -= 360.0
    if pitch < -180.0: pitch += 360.0
    if roll  >  180.0: roll  -= 360.0
    if roll  < -180.0: roll  += 360.0

    R     = rot_matrix(roll, pitch, yaw)
    verts = BOARD_VERTS @ R.T
    faces = [[verts[i] for i in f] for f in FACES]

    if poly_col[0] is not None:
        poly_col[0].remove()
    poly_col[0] = Poly3DCollection(faces, facecolors=FACE_COLORS,
                                   edgecolors='#111', linewidths=0.4, alpha=0.88)
    ax.add_collection3d(poly_col[0])
    angle_txt.set_text(f'Roll:  {roll:+7.2f}°\nPitch: {pitch:+7.2f}°\nYaw:   {yaw:+7.2f}°')
    return poly_col[0], angle_txt

# ── Entry point ───────────────────────────────────────────────────────────────
if __name__ == '__main__':
    print("Initialising MPU-9250 — enabling DLPF 41 Hz on gyro + accel…")
    init_mpu()

    filt = Madgwick(beta=0.033)
    bias, offsets = calibrate(filt, samples=500)

    _state['filt']    = filt
    _state['bias']    = bias
    _state['offsets'] = offsets
    _state['t_prev']  = time.time()

    print("Running — close the window to stop.")
    ani = animation.FuncAnimation(
        fig, animate, interval=20, blit=False, cache_frame_data=False
    )
    plt.show()
