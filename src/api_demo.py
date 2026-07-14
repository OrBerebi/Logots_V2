#!/usr/bin/env python3
"""Toy example: play the Logots video + audio stream through the frame API.

This is the minimal pattern for consuming Logots data from your own code.
It only uses the public API (logots_api.get_latest_frame) — no GUI internals.

How to run:
  1. Start the GUI:  python src/logots_ui.py
     (press ▶ SIM and pick a recording CSV, or run live on the robot)
  2. In another terminal, same conda env:  python src/api_demo.py

A window opens playing the camera feed while the mic audio plays through
your speakers. Close the window or Ctrl+C to quit.
"""
import sys
import os
from collections import deque

import numpy as np
import tkinter as tk
from PIL import Image, ImageTk

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from logots_api import get_latest_frame

try:
    import sounddevice as sd
    AUDIO_OK = True
except ImportError:
    AUDIO_OK = False
    print('sounddevice not installed — playing video only')

AUDIO_RATE = 16000   # must match the GUI's microphone config (AUDIO_RATE)
POLL_MS    = 30      # poll a bit faster than the ~20 Hz frame rate
DISPLAY_PX = 480

# ── Audio: buffer incoming samples, feed the sound card from a callback ───────
audio_buf = deque()

def audio_callback(outdata, frames, t, status):
    """Called by the sound card when it needs more samples.
    Plays whatever the API delivered so far; silence on underrun."""
    chunk = [audio_buf.popleft() for _ in range(min(frames, len(audio_buf)))]
    out = np.zeros(frames, dtype='float32')
    out[:len(chunk)] = chunk
    outdata[:, 0] = out

# ── Video: a bare tkinter window with one image label ─────────────────────────
root = tk.Tk()
root.title('Logots API demo')
_black = ImageTk.PhotoImage(Image.new('RGB', (DISPLAY_PX, DISPLAY_PX)))
video_lbl = tk.Label(root, image=_black, bg='black')
video_lbl.pack()
status_var = tk.StringVar(value='waiting for frames…')
tk.Label(root, textvariable=status_var, font=('Courier', 10)).pack(pady=4)

last_frame_id = None

def poll():
    global last_frame_id
    try:
        frame = get_latest_frame()
    except (ConnectionError, RuntimeError) as e:
        status_var.set(str(e)[:70])
        root.after(500, poll)          # GUI not up yet — retry slowly
        return

    # get_latest_frame() returns the *latest* frame, not a queue: polling
    # faster than the GUI produces frames returns the same frame again,
    # so always dedupe on frame_id before processing.
    if frame['frame_id'] != last_frame_id:
        last_frame_id = frame['frame_id']

        if frame['image'] is not None:                       # RGB numpy array
            img = Image.fromarray(frame['image']).resize((DISPLAY_PX, DISPLAY_PX))
            video_lbl._photo = ImageTk.PhotoImage(img)       # keep a reference
            video_lbl.config(image=video_lbl._photo)

        audio_buf.extend(frame['audio_samples'])             # queue for playback

        mode = 'SIM' if frame['sim_mode'] else 'REAL'
        status_var.set(f"{mode}  frame {frame['frame_id']:>5}  "
                       f"imu n={len(frame['yaw'])}  audio n={len(frame['audio_samples'])}")

    root.after(POLL_MS, poll)

stream = None
if AUDIO_OK:
    try:
        stream = sd.OutputStream(samplerate=AUDIO_RATE, channels=1,
                                 callback=audio_callback)
        stream.start()
    except Exception as e:
        print(f'audio output unavailable ({e}) — playing video only')

poll()
try:
    root.mainloop()
except KeyboardInterrupt:
    pass
finally:
    if stream:
        stream.stop(); stream.close()
