#!/usr/bin/env python3
"""mrt_experience — the mart: raw reality from Darab's frame API (:8787).

Maps to the "mrt_experience" box in docs/v2_architecture/architecture.svg.
One row = one experience: frame + pose at one moment. Nothing derived.

    last_row()      the newest experience — this is what the `refresh` action returns
    collect_pass()  a whole survey pass at 1 fps (used by scan-style steps)

Needs the GUI serving :8787 (live robot, or replaying a recording).
"""
from __future__ import annotations

import os
import sys
import time
from datetime import datetime

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from logots_api import get_latest_frame

SURVEY_MAX_S = 90   # cap on collection (live robot has no loop wrap to stop at)


def _row(fr: dict) -> dict:
    return {"ts": fr["timestamp"], "image": fr["image"],
            "pos_x": fr.get("pos_x", 0.0), "pos_y": fr.get("pos_y", 0.0),
            "heading": fr.get("heading", 0.0), "pan_angle": fr["pan_angle"],
            "sim_mode": fr["sim_mode"], "frame_id": fr["frame_id"]}


def last_row(retries: int = 40, delay: float = 0.25) -> dict:
    """The mart's newest row. Backs the `refresh` action: how the LLM looks *now*."""
    for _ in range(retries):
        try:
            return _row(get_latest_frame(decode_image=True))
        except Exception:
            time.sleep(delay)
    raise RuntimeError("frame API :8787 unreachable — is the GUI running?")


def collect_pass(max_s: float = SURVEY_MAX_S) -> list[dict]:
    """One survey's experiences at 1 fps: {sec, image, pos_x, pos_y, heading, pan_angle}.

    When the GUI replays a recording it loops — a backward frame_id jump = pass
    complete (sync to the first jump, stop at the next). On the live robot ids
    only climb: collect max_s."""
    rows, last, synced = [], None, False
    bucket_sec, bucket = None, None
    t0 = time.time()
    while time.time() - t0 < max_s + 45:            # +45: sync wait headroom on replay
        try:
            fr = get_latest_frame(decode_image=True)
        except Exception:
            time.sleep(0.25); continue
        if fr["frame_id"] != last:
            if last is not None and fr["frame_id"] < last:   # loop wrap
                if not synced:
                    synced, t0 = True, time.time()           # start of a clean pass
                    print("[mart] synced to recording start", flush=True)
                else:
                    break                                    # full pass collected
            if synced or fr["sim_mode"] is False:            # live robot: no sync needed
                sec = int(datetime.fromisoformat(fr["timestamp"]).timestamp())
                if sec != bucket_sec:
                    if bucket is not None:
                        rows.append(bucket)
                    bucket_sec = sec
                    bucket = {"sec": sec, **_row(fr)}
                if not fr["sim_mode"] and time.time() - t0 >= max_s:
                    break
            last = fr["frame_id"]
        time.sleep(0.03)
    if bucket is not None:
        rows.append(bucket)
    return rows
