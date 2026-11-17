/*
 * Self-contained wheel motions on encoder counts (added 2026-11-10).
 *
 * The Jetson asks once for a motion as a signed encoder-count target and this
 * code runs the wheels until it is reached, then stops by itself:
 *
 *   MOT_SPIN      rotate on the spot. progress = (right_delta - left_delta) / 2,
 *                 + = left turn. The Jetson then checks the real angle on the IMU
 *                 and sends a correction rotation if needed.
 *   MOT_STRAIGHT  traverse. progress = (right_delta + left_delta) / 2, + = forward.
 *                 The wheels are kept matched; the Jetson steers from the IMU by
 *                 moving `steer`, the wanted (right_delta - left_delta) / 2.
 *
 * Plain C++ on purpose: fixed-width ints, no Arduino calls, time and encoder
 * counts passed in. The same file is compiled on a PC against a motor model to
 * test the logic without the robot.
 */
#ifndef LOGOTS_MOTION_H
#define LOGOTS_MOTION_H

#include <stdint.h>

#define MOT_IDLE     0   // nothing asked since power-up
#define MOT_MOVING   1   // wheels turning toward the target
#define MOT_SETTLE   2   // motors off, waiting for the wheels to stop
#define MOT_DONE     3
#define MOT_TIMEOUT  4   // still not there when the time limit ran out
#define MOT_ABORTED  5   // cancelled (counts = 0) or the watchdog fired

#define MOT_SPIN      0
#define MOT_STRAIGHT  1

#define MOT_STEP_MS       5      // control period
#define MOT_TIMEOUT_BASE_MS 3000 // time limit = base + per-count allowance, capped
#define MOT_TIMEOUT_PER_COUNT_X10 40   // 4 ms per count (= a crawl of 250 counts/s, ~1.7 cm/s)
#define MOT_TIMEOUT_MAX_MS  60000
#define MOT_COAST_MS      30     // cut the motors this much travel early (coast allowance)
#define MOT_STILL_MS      150    // no count change for this long = stopped
#define MOT_TOL_COUNTS    10     // after stopping: further off than this -> touch-up pass
#define MOT_MAX_TOUCHUPS  2
#define MOT_STALL_MS      100    // no progress for this long -> raise PWM
#define MOT_STALL_COUNTS  4      // "progress" = at least this many counts
#define MOT_BOOST_STEP    8      // PWM added per stall period (stiction breakaway)
#define MOT_MAX_TRIM      40     // clamp on the wheel-balance trim (PWM)

struct Motion {
  uint8_t  state;
  uint8_t  seq;        // id of the last command taken; a repeat is ignored
  uint8_t  mode;       // MOT_SPIN / MOT_STRAIGHT
  int32_t  startL, startR;
  int32_t  target;     // counts
  int32_t  steer;      // MOT_STRAIGHT: wanted (right_delta - left_delta)/2, set by the Jetson
  int16_t  fast, slow; // PWM far from / close to the target
  int16_t  ramp;       // slow-down zone before the target (counts)
  int16_t  kb;         // wheel-balance gain, PWM per count x100
  int8_t   dir;        // direction of the current pass: +1 / -1
  uint8_t  touchups;
  int16_t  boost;
  int32_t  rate;       // smoothed |progress| speed, counts/s
  int32_t  lastProg, stallRef;
  uint32_t tStart, tStep, tProgress, tStill, timeoutMs;
  int16_t  outL, outR; // signed wheel PWM to apply while motionActive()
};

static inline int32_t motAbs32(int32_t v) { return v < 0 ? -v : v; }
static inline int16_t motClamp(int32_t v, int16_t lo, int16_t hi) {
  return (int16_t)(v < lo ? lo : (v > hi ? hi : v));
}

static inline void motionInit(Motion &m) {
  m.state = MOT_IDLE; m.seq = 0; m.mode = MOT_SPIN; m.outL = m.outR = 0;
}

static inline bool motionActive(const Motion &m) {
  return m.state == MOT_MOVING || m.state == MOT_SETTLE;
}

static inline int32_t motionProgress(const Motion &m, int32_t encL, int32_t encR) {
  int32_t dl = encL - m.startL, dr = encR - m.startR;
  return m.mode == MOT_SPIN ? (dr - dl) / 2 : (dr + dl) / 2;
}

static inline void motionAbort(Motion &m) {
  if (motionActive(m)) { m.state = MOT_ABORTED; m.outL = m.outR = 0; }
}

// Take a new command. Returns false (and changes nothing) for a repeated seq.
// counts == 0 cancels whatever is running.
static inline bool motionCommand(Motion &m, uint8_t mode, uint8_t seq, int32_t counts,
                                 int16_t fast, int16_t slow, int16_t ramp, int16_t kb,
                                 int32_t encL, int32_t encR, uint32_t now) {
  if (seq == m.seq) return false;
  m.seq = seq;
  m.outL = m.outR = 0;
  if (counts == 0) { m.state = MOT_ABORTED; return true; }
  m.mode = mode;
  m.startL = encL; m.startR = encR;
  m.target = counts;
  m.steer = 0;
  m.fast = motClamp(fast, 0, 255);
  m.slow = motClamp(slow, 0, m.fast);
  m.ramp = ramp < 0 ? 0 : ramp;
  m.kb   = kb < 0 ? 0 : kb;
  m.dir  = counts > 0 ? 1 : -1;
  m.touchups = 0; m.boost = 0; m.rate = 0;
  m.lastProg = m.stallRef = 0;
  m.tStart = m.tStep = m.tProgress = now;
  uint32_t limit = MOT_TIMEOUT_BASE_MS
                 + (uint32_t)motAbs32(counts) * MOT_TIMEOUT_PER_COUNT_X10 / 10;
  m.timeoutMs = limit > MOT_TIMEOUT_MAX_MS ? MOT_TIMEOUT_MAX_MS : limit;
  m.state = MOT_MOVING;
  return true;
}

// Heading correction for the running traverse `seq`; anything else is ignored.
static inline void motionSteer(Motion &m, uint8_t seq, int32_t steer) {
  if (motionActive(m) && m.mode == MOT_STRAIGHT && seq == m.seq) m.steer = steer;
}

// Call every loop() with fresh encoder counts; it paces itself to MOT_STEP_MS.
static inline void motionStep(Motion &m, int32_t encL, int32_t encR, uint32_t now) {
  if (!motionActive(m)) return;
  uint32_t dt = now - m.tStep;
  if (dt < MOT_STEP_MS) return;
  m.tStep = now;

  if (now - m.tStart > m.timeoutMs) {
    m.state = MOT_TIMEOUT; m.outL = m.outR = 0; return;
  }

  int32_t dl = encL - m.startL, dr = encR - m.startR;
  int32_t prog = m.mode == MOT_SPIN ? (dr - dl) / 2 : (dr + dl) / 2;
  int32_t rem  = m.target - prog;

  if (m.state == MOT_SETTLE) {
    m.outL = m.outR = 0;
    if (prog != m.lastProg) { m.lastProg = prog; m.tStill = now; return; }
    if (now - m.tStill < MOT_STILL_MS) return;
    if (motAbs32(rem) > MOT_TOL_COUNTS && m.touchups < MOT_MAX_TOUCHUPS) {
      m.touchups++;
      m.dir = rem > 0 ? 1 : -1;
      m.rate = 0; m.stallRef = prog; m.tProgress = now;
      m.state = MOT_MOVING;
    } else {
      m.state = MOT_DONE;
    }
    return;
  }

  // ── MOT_MOVING ──
  int32_t inst = motAbs32(prog - m.lastProg) * 1000 / (int32_t)dt;   // counts/s
  m.rate = (3 * m.rate + inst) / 4;
  m.lastProg = prog;

  int32_t ahead = rem * m.dir;                       // counts still to go in this pass
  if (ahead <= m.rate * MOT_COAST_MS / 1000) {       // there (or will coast there): stop
    m.outL = m.outR = 0;
    m.state = MOT_SETTLE; m.tStill = now;
    return;
  }

  // not moving -> push harder, a step at a time
  if (motAbs32(prog - m.stallRef) >= MOT_STALL_COUNTS) {
    m.stallRef = prog; m.tProgress = now;
  } else if (now - m.tProgress >= MOT_STALL_MS) {
    if (m.boost < 255) m.boost += MOT_BOOST_STEP;
    m.tProgress = now;
  }

  int32_t mag;
  if (m.touchups > 0)                        mag = m.slow;
  else if (m.ramp <= 0 || ahead >= m.ramp)   mag = m.fast;
  else mag = m.slow + (int32_t)(m.fast - m.slow) * ahead / m.ramp;
  mag += m.boost;

  // Wheel balance. `err` is what should be zero, in counts of (right vs left):
  //   spin:     dl + dr            = twice the robot's forward creep (0 = pure pivot)
  //   straight: (dr - dl) - 2*steer = twice the unwanted turn (0 = on the wanted heading)
  // err > 0 means the right wheel is ahead: slow it, speed up the left. The same
  // rule holds in both directions of both modes (dir flips with the pass).
  int32_t err = m.mode == MOT_SPIN ? (dl + dr) : (dr - dl) - 2 * m.steer;
  int32_t trim = motClamp((int32_t)m.kb * err / 100, -MOT_MAX_TRIM, MOT_MAX_TRIM);
  int16_t magR = motClamp(mag - m.dir * trim, 0, 255);
  int16_t magL = motClamp(mag + m.dir * trim, 0, 255);
  m.outR = (int16_t)(m.dir * magR);
  m.outL = (int16_t)((m.mode == MOT_SPIN ? -m.dir : m.dir) * magL);
}

#endif
