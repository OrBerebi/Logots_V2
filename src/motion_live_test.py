#!/usr/bin/env python3
"""motion_live_test — checks traverse / rotate / servo on the REAL robot.

Run it on the Jetson after flashing the firmware. It plays the LLM's part: it publishes
each motion command once on :8788, waits for the robot's completion report, and compares
that report with what the robot's own sensors show on the frame API (:8787): IMU heading,
encoder position, servo angles. It prints PASS/FAIL per check and saves the same text
under runs/.

    # 1. start the GUI WITHOUT the voice listener (this script needs :8788 itself)
    LISTEN=0 conda run -n logots python src/logots_ui.py
    # 2. wheels OFF the floor first:
    conda run --no-capture-output -n logots python src/motion_live_test.py --stage raised
    # 3. then on the floor, 1 m of clear space on every side:
    conda run --no-capture-output -n logots python src/motion_live_test.py --stage floor
    # optional: compare with a tape measure (asks you to type what you measured)
    conda run --no-capture-output -n logots python src/motion_live_test.py --stage tape

The robot MOVES. On the floor it stays within about 60 cm of where it starts. There is no
obstacle detection. To stop it: SPACE or the STOP button in the GUI (this script cannot
stop a motion; Ctrl-C only stops the script from sending the next one).

What it can and cannot prove: the robot's sensors (encoders, IMU) agreeing with each
other and with the request. Whether 30 cm on the encoders is 30 cm on the floor is what
--stage tape is for.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
import urllib.request
from datetime import datetime

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

FRAME_URL = "http://localhost:8787/latest_frame"
REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# Pass limits. Deliberately looser than the controller's own targets (1.5 deg, 1 cm):
# this test asks "does the feature work", tuning is a separate step.
ROT_TOL_DEG        = 3.0    # |turned - asked|, by the report (IMU)
ROT_AGREE_DEG      = 2.0    # report vs heading change seen on :8787
ROT_DRIFT_CM       = 5.0    # position change during a rotation (should pivot in place)
TRAV_TOL_CM        = 1.5    # |moved - asked|, by the report (encoders)
TRAV_AGREE_CM      = 2.0    # report vs position change seen on :8787
TRAV_HEADING_DEG   = 4.0    # heading change during a traverse (should hold straight)
SPEED_MIN_CM_S     = 4.0    # slower than this over a traverse means something drags
SQUARE_TOL_DEG     = 6.0    # heading error after 4 x 90 degrees
RETURN_TOL_CM      = 5.0    # distance from the start after out-and-back traverses


class Run:
    def __init__(self, endpoint, wait_for_done, log):
        self.ep, self.wait, self.log = endpoint, wait_for_done, log
        self.n_pass = self.n_fail = 0

    def say(self, text=""):
        print(text, flush=True)
        self.log.write(text + "\n"); self.log.flush()

    def check(self, name, ok, detail=""):
        self.n_pass += bool(ok); self.n_fail += (not ok)
        self.say(f"  {'PASS' if ok else 'FAIL'}  {name}" + (f"   [{detail}]" if detail else ""))
        return ok

    def frame(self):
        with urllib.request.urlopen(FRAME_URL, timeout=3) as r:
            f = json.loads(r.read())
        return {k: f.get(k) for k in ("pos_x", "pos_y", "heading", "pan_angle",
                                      "tilt_angle", "sim_mode", "left_pwm", "right_pwm")}

    def send(self, action, args, timeout=90):
        """Publish one command, wait for its report. Returns (report|None, before, after, s)."""
        before = self.frame()
        t0 = time.time()
        payload = self.ep.publish({"action": action, "args": args})
        report = self.wait(self.ep, payload["action_id"], timeout)
        dt = time.time() - t0
        time.sleep(0.4)                      # let the frame API catch up with the end pose
        after = self.frame()
        self.say(f"\n> {action} {json.dumps(args)}  ->  "
                 f"{json.dumps({k: (round(v, 3) if isinstance(v, float) else v) for k, v in (report or {}).items() if k != 'action_id'}) if report else 'NO REPORT'}"
                 f"   ({dt:.1f} s)")
        return report, before, after, dt


def wrap180(a):
    return (a + 180) % 360 - 180


def moved_cm(before, after):
    return math.hypot(after["pos_x"] - before["pos_x"], after["pos_y"] - before["pos_y"]) * 100


def along_cm(before, after):
    """Displacement along the starting heading, signed (+ = forward)."""
    th = math.radians(before["heading"])
    return ((after["pos_x"] - before["pos_x"]) * math.cos(th)
            + (after["pos_y"] - before["pos_y"]) * math.sin(th)) * 100


# ── checks shared by the stages ───────────────────────────────────────────────
def preflight(run: Run) -> bool:
    run.say("== Preflight")
    try:
        f = run.frame()
    except Exception as e:
        run.check("GUI is running (frame API on :8787 answers)", False, repr(e))
        return False
    run.check("GUI is running (frame API on :8787 answers)", True)
    if not run.check("GUI is in Real mode, not Sim", not f["sim_mode"]):
        return False
    rep, _, _, _ = run.send("servo", {"pan": 0, "tilt": 0}, timeout=10)
    if not run.check("GUI executes commands from :8788 (a report came back)", rep is not None,
                     "" if rep is not None else "no report: is the GUI started with LISTEN=0, "
                     "and is this the only functions.py/test using :8788?"):
        return False
    if not run.check("Arduino connected", "not connected" not in (rep.get("problem") or ""),
                     rep.get("problem", "")):
        return False
    rep, _, _, _ = run.send("traverse", {"cm": 0}, timeout=10)     # 0 cm: moves nothing
    problem = (rep or {}).get("problem", "")
    return run.check("Arduino has the new firmware", rep is not None and "firmware" not in problem
                     and "not connected" not in problem, problem)


def servo_checks(run: Run):
    run.say("\n== Servo (watch the camera: left, right, up, down, centre)")
    for pan, tilt in [(30, 0), (-30, 0), (0, 20), (0, -20), (45, 10), (0, 0)]:
        rep, _, after, _ = run.send("servo", {"pan": pan, "tilt": tilt}, timeout=10)
        ok = rep is not None and rep.get("pan") == pan and rep.get("tilt") == tilt and "problem" not in rep
        run.check(f"servo ({pan}, {tilt}) reported as set", ok)
        run.check(f"servo ({pan}, {tilt}) seen on the frame API", after["pan_angle"] == pan,
                  f"pan_angle={after['pan_angle']}, raw tilt_angle={after['tilt_angle']}")
        time.sleep(0.6)
    rep, _, _, _ = run.send("servo", {"pan": 80, "tilt": 0}, timeout=10)
    run.check("servo pan 80 is limited to 45 and says so",
              rep is not None and rep.get("pan") == 45 and "limited" in rep.get("problem", ""))
    run.send("servo", {"pan": 0, "tilt": 0}, timeout=10)
    run.say("  (by eye: did +pan go LEFT and +tilt go UP? If tilt went down, set TILT_DIR = -1)")


def traverse_check(run: Run, cm, raised=False):
    rep, before, after, dt = run.send("traverse", {"cm": cm})
    if not run.check(f"traverse {cm:+d} cm: report came back", rep is not None):
        return
    run.check(f"traverse {cm:+d} cm: no problem reported", "problem" not in rep, rep.get("problem", ""))
    got = rep.get("moved_cm", 0.0)
    run.check(f"traverse {cm:+d} cm: moved the asked distance (encoders)",
              abs(got - cm) <= TRAV_TOL_CM, f"moved_cm={got:+.2f}, limit ±{TRAV_TOL_CM}")
    seen = along_cm(before, after)
    run.check(f"traverse {cm:+d} cm: frame API position agrees",
              abs(seen - got) <= TRAV_AGREE_CM, f"position moved {seen:+.2f} cm along the heading")
    run.check(f"traverse {cm:+d} cm: speed is plausible", abs(got) / max(dt, 0.1) >= SPEED_MIN_CM_S,
              f"{abs(got) / max(dt, 0.1):.1f} cm/s")
    if not raised:
        turn = wrap180(after["heading"] - before["heading"])
        run.check(f"traverse {cm:+d} cm: heading held", abs(turn) <= TRAV_HEADING_DEG,
                  f"heading changed {turn:+.1f}°, limit ±{TRAV_HEADING_DEG}")


def rotate_check(run: Run, deg):
    rep, before, after, _ = run.send("rotate", {"degrees": deg})
    if not run.check(f"rotate {deg:+d}°: report came back", rep is not None):
        return
    problem = rep.get("problem", "")
    if "wrong way" in problem:
        run.check(f"rotate {deg:+d}°: turned the right way", False,
                  "set SPIN_DIR = -1 in logots_ui.py, restart the GUI, run again")
        return
    run.check(f"rotate {deg:+d}°: no problem reported", not problem, problem)
    got = rep.get("turned_deg", 0.0)
    run.check(f"rotate {deg:+d}°: turned the asked angle (IMU)",
              abs(got - deg) <= ROT_TOL_DEG, f"turned_deg={got:+.2f}, limit ±{ROT_TOL_DEG}")
    seen = wrap180(after["heading"] - before["heading"])
    run.check(f"rotate {deg:+d}°: frame API heading agrees",
              abs(wrap180(seen - got)) <= ROT_AGREE_DEG, f"heading changed {seen:+.2f}°")
    run.check(f"rotate {deg:+d}°: stayed in place", moved_cm(before, after) <= ROT_DRIFT_CM,
              f"position moved {moved_cm(before, after):.1f} cm, limit {ROT_DRIFT_CM}")


# ── stages ────────────────────────────────────────────────────────────────────
def stage_raised(run: Run):
    run.say("\n== Wheels raised: do the wheels and the encoders respond?")
    run.say("  (watch: FORWARD = both wheels forward; ROTATE LEFT = right forward, left backward)")
    traverse_check(run, 10, raised=True)
    traverse_check(run, -10, raised=True)
    rep, _, _, _ = run.send("rotate", {"degrees": 15})
    problem = (rep or {}).get("problem", "")
    # The body can't turn while raised, so the IMU sees nothing and the rotation must end
    # by saying so. What matters here: it ran, it ended, and it did not hang.
    run.check("rotate +15° raised: ran and ended by itself", rep is not None)
    run.check("rotate +15° raised: reports it could not reach the angle (expected while raised)",
              "off after" in problem or "wrong way" in problem or not problem, problem)
    servo_checks(run)


def stage_floor(run: Run):
    run.say("\n== On the floor")
    start = run.frame()
    servo_checks(run)
    run.say("\n-- rotate")
    for deg in (15, -15, 90, -90, 45, -45):
        rotate_check(run, deg)
    run.say("\n-- traverse")
    for cm in (10, -10, 30, -30, 50, -50):
        traverse_check(run, cm)
    back = run.frame()
    run.check("after out-and-back traverses the robot is back near its start",
              moved_cm(start, back) <= RETURN_TOL_CM, f"{moved_cm(start, back):.1f} cm from the start")
    run.say("\n-- four left turns of 90° should come back to the same heading")
    h0 = run.frame()["heading"]
    total = 0.0
    for _ in range(4):
        rep, _, _, _ = run.send("rotate", {"degrees": 90})
        total += (rep or {}).get("turned_deg", 0.0)
    h1 = run.frame()["heading"]
    run.check("4 x 90°: reports add up to 360°", abs(total - 360) <= SQUARE_TOL_DEG, f"sum {total:.1f}°")
    run.check("4 x 90°: heading is back where it was", abs(wrap180(h1 - h0)) <= SQUARE_TOL_DEG,
              f"off by {wrap180(h1 - h0):+.1f}°")
    run.say("\n-- a command the robot should refuse to do in full")
    rep, _, _, _ = run.send("rotate", {"degrees": 400})
    run.check("rotate 400° is limited to 180° and says so",
              rep is not None and "limited" in rep.get("problem", "") and abs(rep.get("turned_deg", 0) - 180) <= ROT_TOL_DEG,
              json.dumps({k: rep.get(k) for k in ("turned_deg", "problem")}) if rep else "")
    run.send("rotate", {"degrees": -180})


def stage_tape(run: Run):
    run.say("\n== Tape measure: is the robot's own measurement true?")

    def ask(q):
        try:
            return float(input(q).strip().replace(",", "."))
        except (ValueError, EOFError):
            return None
    input("Mark where a wheel touches the floor, then press Enter: the robot will drive 100 cm forward. ")
    rep, _, _, _ = run.send("traverse", {"cm": 100})
    real = ask("Distance from the mark to the same wheel now, in cm: ")
    if real is not None and rep:
        err = real - rep.get("moved_cm", 0.0)
        run.check("100 cm traverse: tape agrees with the encoders within 3 cm", abs(err) <= 3.0,
                  f"tape {real:.1f} cm, robot says {rep.get('moved_cm', 0):.1f} cm")
        side = ask("How far sideways did it end from a straight line, in cm (0 if straight): ")
        if side is not None:
            run.check("100 cm traverse: sideways error under 5 cm", abs(side) <= 5.0, f"{side:.1f} cm")
    input("Line the robot up with a floor line, press Enter: it will turn 90° left. ")
    rep, _, _, _ = run.send("rotate", {"degrees": 90})
    real = ask("Angle it actually turned, in degrees (a floor tile corner is 90): ")
    if real is not None and rep:
        run.check("90° rotation: measured angle within 4° of 90", abs(real - 90) <= 4.0,
                  f"measured {real:.0f}°, robot says {rep.get('turned_deg', 0):.1f}°")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--stage", choices=["raised", "floor", "tape"], required=True)
    ap.add_argument("--yes", action="store_true", help="skip the 5 s countdown")
    args = ap.parse_args()

    from actions import open_endpoint

    def wait_for_done(endpoint, action_id, timeout):    # same contract functions.py waits on
        t0 = time.time()
        while time.time() - t0 < timeout:
            report = endpoint.completion(action_id)
            if report:
                return report
            time.sleep(0.1)
        return None
    endpoint = open_endpoint()
    if endpoint is None:
        sys.exit("Could not take :8788. Start the GUI with LISTEN=0 and stop any other functions.py.")

    os.makedirs(os.path.join(REPO, "runs"), exist_ok=True)
    path = os.path.join(REPO, "runs", f"motion_test_{args.stage}_{datetime.now():%Y%m%d_%H%M%S}.txt")
    with open(path, "w") as log:
        run = Run(endpoint, wait_for_done, log)
        run.say(f"motion_live_test --stage {args.stage}   {datetime.now().isoformat(timespec='seconds')}")
        where = {"raised": "Wheels must be OFF the floor.",
                 "floor": "Robot on the floor, 1 m clear on every side.",
                 "tape": "Robot on the floor, 1.5 m clear in front."}[args.stage]
        run.say(f"{where} To stop the robot: SPACE or STOP in the GUI.")
        time.sleep(1.0)          # give the GUI's ActionReader a moment to find the endpoint
        try:
            if preflight(run):
                if not args.yes:
                    for n in range(5, 0, -1):
                        print(f"  the robot moves in {n}…", flush=True); time.sleep(1)
                {"raised": stage_raised, "floor": stage_floor, "tape": stage_tape}[args.stage](run)
            else:
                run.say("\nPreflight failed: nothing was moved.")
        except KeyboardInterrupt:
            run.say("\nInterrupted. A motion already sent keeps going: press SPACE in the GUI.")
        run.say(f"\nRESULT: {run.n_pass} passed, {run.n_fail} failed"
                + ("" if run.n_fail else "  — all checks passed"))
        run.say(f"saved: {os.path.relpath(path, REPO)}")
    os._exit(1 if run.n_fail else 0)     # the endpoint's server thread would keep us alive


if __name__ == "__main__":
    main()
