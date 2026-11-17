/*
 * Logots V2 Motor Control Firmware
 * I2C slave 0x08
 * Protocol (Jetson -> Arduino): "{left_pwm},{right_pwm},{pan},{tilt}\n"
 *   left_pwm / right_pwm : -255 to +255 (positive = forward)
 *   pan                  : -90 to +90 degrees (0 = camera facing front, + = left)
 *   tilt                 : 0 to 180 degrees
 *
 * Self-contained motions (Jetson -> Arduino, added 2026-11-10). Sent once; the
 * Arduino runs the wheels to an encoder-count target and stops. See motion.h.
 *   "R,{seq},{counts},{fast},{slow},{ramp},{kb}\n"   rotate on the spot until
 *        (right_delta - left_delta)/2 reaches {counts}; + = left turn
 *   "T,{seq},{counts},{fast},{slow},{ramp},{kb}\n"   traverse straight until
 *        (right_delta + left_delta)/2 reaches {counts}; + = forward
 *   "S,{seq},{steer}\n"   heading correction for the running traverse {seq}: the
 *        wanted (right_delta - left_delta)/2. The Jetson moves it from the IMU.
 *   seq  : 1..255, a repeated seq is ignored (the Jetson resends until it sees it
 *          in the read-back).  counts = 0 cancels.
 *   fast / slow : PWM far from / near the target;  ramp : slow-down zone in counts
 *   kb   : wheel-balance gain, PWM per count x100
 *   While a motion runs, left_pwm/right_pwm of drive messages are ignored.
 *
 * Watchdog: no valid message for 500 ms -> motors stop, a running motion aborts.
 *
 * Read-back (Arduino -> Jetson): a 13-byte I2C read from 0x08 returns
 *   [0xE5][int32 left_count][int32 right_count][sum8 of bytes 0..8]   (bytes 0..9,
 *   unchanged, so a 10-byte read still works) then
 *   [mot_seq][mot_state][(mot_seq + mot_state + 0x5A) & 0xFF]  (one motion at a time,
 *   rotate and traverse share them).
 *   Counts are cumulative quadrature (4x) encoder ticks; positive = forward.
 *
 * Hardware (see src/pinout.txt):
 *   DC motors  : HW-130 / Adafruit Motor Shield v1, M1 (right) and M4 (left)
 *   Encoders   : right A0 (A) / A1 (B), left A2 (A) / A3 (B)
 *   Pan servo  : pin 10
 *   Tilt servo : pin 9
 */

#include <Wire.h>
#include <AFMotor.h>
#include <Servo.h>
#include "motion.h"

// ── Calibration (set from the bench test, then reflash) ─────────────────────
const int PAN_CENTER_DEG  = 90;  // servo angle that faces front (trim after re-seating horn)
const int PAN_DIR         = +1;  // +1 if a larger servo angle turns the camera LEFT, else -1
const int LEFT_MOTOR_DIR  = +1;  // flip to -1 if the left wheel spins backward on +PWM
const int RIGHT_MOTOR_DIR = +1;
const int LEFT_ENC_DIR    = +1;  // set by hand-turn test 2026-10-03: left forward counts up
const int RIGHT_ENC_DIR   = -1;  // right forward counted down (mirror-mounted motor)

// ── Motors ─────────────────────────────────────────────────────────────────
AF_DCMotor motorRight(1);
AF_DCMotor motorLeft(4);

// ── Servos ─────────────────────────────────────────────────────────────────
Servo panServo;
Servo tiltServo;

const int PAN_PIN  = 10;
const int TILT_PIN = 9;

// ── Targets (set from parseMessage) ────────────────────────────────────────
volatile int target_left_pwm  = 0;
volatile int target_right_pwm = 0;
volatile int target_pan_angle  = PAN_CENTER_DEG;   // servo degrees
volatile int target_tilt_angle = 90;

// ── Current servo positions (smoothed in loop) ────────────────────────────
int current_pan_angle  = PAN_CENTER_DEG;
int current_tilt_angle = 90;

unsigned long lastServoMoveTime = 0;
const int SERVO_SPEED_DELAY = 1;  // ms per 1-degree step

// ── I2C receive buffer ────────────────────────────────────────────────────
#define I2C_BUFFER_SIZE 32
char i2cBuffer[I2C_BUFFER_SIZE + 1];
char pendingBuffer[I2C_BUFFER_SIZE + 1];
byte bufferIndex = 0;
volatile bool messageReady = false;

// ── Encoders ──────────────────────────────────────────────────────────────
// A0-A3 are PC0-PC3 = PCINT8-11, all on PCINT1_vect. A4/A5 (I2C) stay out of
// the mask. Polling from loop() would miss edges: every loop iteration calls
// AFMotor run(), which bit-bangs the shield's shift register.
volatile int32_t encLeft  = 0;
volatile int32_t encRight = 0;
uint8_t prevR, prevL;   // last 2-bit (B<<1)|A state, touched only by the ISR after setup()

// index = (prev << 2) | curr  ->  -1 / 0 / +1 step (0 for no change or a skipped state)
const int8_t QDEC[16] = {0, +1, -1, 0,  -1, 0, 0, +1,  +1, 0, 0, -1,  0, -1, +1, 0};

#define ENC_PKT_HDR 0xE5
#define ENC_PKT_LEN 13   // 10-byte encoder packet + mot_seq, mot_state, check
#define MOT_PKT_SALT 0x5A

// ── Motion (rotate / traverse) + watchdog ───────────────────────────────────────────────────
Motion mot;
bool motWasActive = false;
const unsigned long WATCHDOG_MS = 500;   // the Jetson sends every ~100 ms
unsigned long lastMsgTime = 0;

unsigned long lastEncPrintTime = 0;
int32_t lastPrintedLeft = 0, lastPrintedRight = 0;

// ─────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);
  Serial.println("Logots V2 motor controller ready.");

  motorLeft.setSpeed(0);  motorLeft.run(RELEASE);
  motorRight.setSpeed(0); motorRight.run(RELEASE);

  panServo.attach(PAN_PIN);
  tiltServo.attach(TILT_PIN);
  panServo.write(current_pan_angle);
  tiltServo.write(current_tilt_angle);

  pinMode(A0, INPUT_PULLUP);
  pinMode(A1, INPUT_PULLUP);
  pinMode(A2, INPUT_PULLUP);
  pinMode(A3, INPUT_PULLUP);
  motionInit(mot);

  uint8_t p = PINC;
  prevR = p & 0x03;
  prevL = (p >> 2) & 0x03;
  PCMSK1 |= 0x0F;          // PCINT8-11 = A0-A3
  PCICR  |= _BV(PCIE1);

  Wire.begin(0x08);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
}

void loop() {
  if (messageReady) {
    parseMessage(pendingBuffer);
    messageReady = false;
  }
  // Watchdog: a silent Jetson (crashed or frozen GUI) must not leave the wheels turning.
  if (millis() - lastMsgTime > WATCHDOG_MS) {
    target_left_pwm = 0;
    target_right_pwm = 0;
    motionAbort(mot);
  }

  noInterrupts();
  int32_t encL = encLeft, encR = encRight;
  interrupts();
  motionStep(mot, encL, encR, millis());

  if (motionActive(mot)) {
    controlMotor(motorLeft,  LEFT_MOTOR_DIR  * mot.outL);
    controlMotor(motorRight, RIGHT_MOTOR_DIR * mot.outR);
  } else {
    controlMotor(motorLeft,  LEFT_MOTOR_DIR  * target_left_pwm);
    controlMotor(motorRight, RIGHT_MOTOR_DIR * target_right_pwm);
  }
  if (motWasActive && !motionActive(mot)) {
    Serial.print("MOT end state="); Serial.print(mot.state);
    Serial.print(" progress=");     Serial.print(motionProgress(mot, encL, encR));
    Serial.print(" target=");       Serial.println(mot.target);
  }
  motWasActive = motionActive(mot);
  smoothServoMove();
  printEncoders();
}

// ── Smooth servo stepping ─────────────────────────────────────────────────
void smoothServoMove() {
  if (millis() - lastServoMoveTime < SERVO_SPEED_DELAY) return;
  lastServoMoveTime = millis();

  if (current_pan_angle < target_pan_angle)       current_pan_angle++;
  else if (current_pan_angle > target_pan_angle)  current_pan_angle--;

  if (current_tilt_angle < target_tilt_angle)      current_tilt_angle++;
  else if (current_tilt_angle > target_tilt_angle) current_tilt_angle--;

  panServo.write(current_pan_angle);
  tiltServo.write(current_tilt_angle);
}

// ── Signed pan (0 = front, + = left) -> servo angle ───────────────────────
int panToServo(int pan) {
  return constrain(PAN_CENTER_DEG + PAN_DIR * constrain(pan, -90, 90), 0, 180);
}

// ── Encoder ISR: table lookup + add only ──────────────────────────────────
// Same rule as receiveEvent() below: anything slow in an ISR delays the Servo
// library's Timer1 interrupt and makes the pan/tilt servos twitch.
ISR(PCINT1_vect) {
  uint8_t p = PINC;
  uint8_t r = p & 0x03;
  uint8_t l = (p >> 2) & 0x03;
  encRight += RIGHT_ENC_DIR * QDEC[(prevR << 2) | r]; prevR = r;
  encLeft  += LEFT_ENC_DIR  * QDEC[(prevL << 2) | l]; prevL = l;
}

// ── Serial debug: encoder counts every 500 ms, only when they changed ─────
// Lets the encoders be checked by turning the wheels by hand with no Jetson.
void printEncoders() {
  if (millis() - lastEncPrintTime < 500) return;
  lastEncPrintTime = millis();
  noInterrupts();
  int32_t l = encLeft, r = encRight;
  interrupts();
  if (l == lastPrintedLeft && r == lastPrintedRight) return;
  lastPrintedLeft = l; lastPrintedRight = r;
  Serial.print("ENC L="); Serial.print(l);
  Serial.print(" R=");    Serial.println(r);
}

// ── I2C ISR: buffer bytes only, hand off on newline ───────────────────────
// Keep this ISR as short as possible. sscanf()/Serial.print() used to run
// here directly; that held global interrupts off for long enough to delay
// Timer1's compare-match interrupt (which Servo uses to end each pulse),
// occasionally stretching a pan/tilt pulse and showing up as a random
// twitch even with an unchanged target angle. Parsing now happens in loop().
void receiveEvent(int bytesReceived) {
  while (Wire.available()) {
    char c = Wire.read();
    if (c == '\n') {
      i2cBuffer[bufferIndex] = '\0';
      if (!messageReady) {
        strcpy(pendingBuffer, i2cBuffer);
        messageReady = true;
      }
      bufferIndex = 0;
    } else {
      if (bufferIndex < I2C_BUFFER_SIZE) {
        i2cBuffer[bufferIndex++] = c;
      } else {
        bufferIndex = 0;
      }
    }
  }
}

// ── I2C ISR: master read -> encoder packet ────────────────────────────────
// Runs in the TWI interrupt with interrupts off, so the 32-bit counts are read
// atomically. Builds 13 bytes and returns; nothing else belongs here. Bytes 0..9
// are the original encoder packet; a master that reads only 10 still gets it.
void requestEvent() {
  uint8_t pkt[ENC_PKT_LEN];
  int32_t l = encLeft, r = encRight;
  pkt[0] = ENC_PKT_HDR;
  memcpy(&pkt[1], &l, 4);   // AVR is little-endian
  memcpy(&pkt[5], &r, 4);
  uint8_t sum = 0;
  for (uint8_t i = 0; i < 9; i++) sum += pkt[i];
  pkt[9]  = sum;
  pkt[10] = mot.seq;
  pkt[11] = mot.state;
  pkt[12] = (uint8_t)(mot.seq + mot.state + MOT_PKT_SALT);
  Wire.write(pkt, ENC_PKT_LEN);
}

// ── Parse "R|T,{seq},{counts},{fast},{slow},{ramp},{kb}" and "S,{seq},{steer}" ──
void parseMotion(const char* msg) {
  int seq = 0, fast = 0, slow = 0, ramp = 0, kb = 0;
  long counts = 0;
  if (msg[0] == 'S') {
    if (sscanf(msg + 1, ",%d,%ld", &seq, &counts) == 2) {
      lastMsgTime = millis();
      motionSteer(mot, (uint8_t)seq, counts);
    }
    return;                      // sent every tick while traversing: no Serial print
  }
  int n = sscanf(msg + 1, ",%d,%ld,%d,%d,%d,%d", &seq, &counts, &fast, &slow, &ramp, &kb);
  if (n != 6 || seq < 1 || seq > 255) {
    Serial.print("Parse error: ");
    Serial.println(msg);
    return;
  }
  lastMsgTime = millis();
  noInterrupts();
  int32_t encL = encLeft, encR = encRight;
  interrupts();
  uint8_t mode = msg[0] == 'R' ? MOT_SPIN : MOT_STRAIGHT;
  if (motionCommand(mot, mode, (uint8_t)seq, counts, fast, slow, ramp, kb, encL, encR, millis())) {
    target_left_pwm = 0;     // nothing left over to resume when the motion ends
    target_right_pwm = 0;
    Serial.print(msg[0] == 'R' ? "ROT seq=" : "TRAV seq="); Serial.print(seq);
    Serial.print(" counts="); Serial.println(counts);
  }
}

// ── Parse "{left},{right},{pan},{tilt}" ────────────────────────────────────
void parseMessage(const char* msg) {
  if (msg[0] == 'R' || msg[0] == 'T' || msg[0] == 'S') { parseMotion(msg); return; }
  int lp = 0, rp = 0, pa = 0, ta = 90;
  int n = sscanf(msg, "%d,%d,%d,%d", &lp, &rp, &pa, &ta);
  if (n == 4) {
    lastMsgTime = millis();
    if (!motionActive(mot)) {   // a motion owns the wheels until it ends
      target_left_pwm  = lp;
      target_right_pwm = rp;
    }
    target_pan_angle  = panToServo(pa);
    target_tilt_angle = constrain(ta, 0, 180);
    Serial.print("OK  L="); Serial.print(lp);
    Serial.print(" R=");    Serial.print(rp);
    Serial.print(" PAN=");  Serial.print(pa);
    Serial.print(" TILT="); Serial.println(ta);
  } else {
    Serial.print("Parse error: ");
    Serial.println(msg);
  }
}

// ── Drive a DC motor with signed PWM ─────────────────────────────────────
void controlMotor(AF_DCMotor& motor, int pwm) {
  pwm = constrain(pwm, -255, 255);
  motor.setSpeed(abs(pwm));
  if      (pwm > 0) motor.run(FORWARD);
  else if (pwm < 0) motor.run(BACKWARD);
  else              motor.run(RELEASE);
}
