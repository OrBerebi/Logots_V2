#!/usr/bin/env python3
"""Client for the Logots frame API.

The Logots GUI (src/logots_ui.py) serves its most recent sensor snapshot over
HTTP on port 8787, in both REAL and SIM modes. Usage from a script/notebook:

    from logots_api import get_latest_frame
    frame = get_latest_frame()
    frame['image']          # 640×640×3 uint8 RGB numpy array (None if no camera)
    frame['yaw']            # list of yaw readings (°) since the previous frame
    frame['audio_samples']  # list of mic samples (~800 per frame at 20 Hz)
"""
import base64
import io
import json
import urllib.error
import urllib.request

import numpy as np
from PIL import Image

DEFAULT_PORT = 8787


def get_latest_frame(host='localhost', port=DEFAULT_PORT, decode_image=True, timeout=2.0):
    """Fetch the most recent frame from the running Logots GUI.

    Returns a dict with keys:
      frame_id (int), timestamp (ISO 8601 str), frame_data (base64 JPEG str),
      yaw / pitch / roll (lists of degrees sampled since the previous frame),
      audio_samples (list of floats), left_pwm, right_pwm (−255…+255),
      pan_angle, tilt_angle (0…180°), sim_mode (bool), and — when
      decode_image is True — image: an H×W×3 uint8 RGB numpy array,
      or None if no camera frame was available.

    Raises ConnectionError if the GUI is not reachable, RuntimeError if the
    GUI is up but has not produced a frame yet.
    """
    url = f'http://{host}:{port}/latest_frame'
    try:
        with urllib.request.urlopen(url, timeout=timeout) as resp:
            frame = json.loads(resp.read().decode('utf-8'))
    except urllib.error.HTTPError as e:
        if e.code == 503:
            raise RuntimeError('Logots GUI is up but has not produced a frame yet') from e
        raise
    except urllib.error.URLError as e:
        raise ConnectionError(
            f'Could not reach {url} — is the Logots GUI running '
            f'with its frame server on port {port}?') from e
    if decode_image:
        b64 = frame.get('frame_data') or ''
        if b64:
            img = Image.open(io.BytesIO(base64.b64decode(b64))).convert('RGB')
            frame['image'] = np.asarray(img)
        else:
            frame['image'] = None
    return frame


if __name__ == '__main__':
    f = get_latest_frame()
    img = f.get('image')
    print(f"frame_id={f['frame_id']}  sim_mode={f['sim_mode']}  "
          f"image={'none' if img is None else img.shape}  "
          f"imu_samples={len(f['yaw'])}  audio_samples={len(f['audio_samples'])}")
