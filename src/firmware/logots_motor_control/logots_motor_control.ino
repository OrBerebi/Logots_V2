/*
 * Logots V2 Motor Control Firmware
 * I2C slave 0x08
 * Protocol: "{left_pwm},{right_pwm},{pan_angle},{tilt_angle}\n"
 *   left_pwm / right_pwm : -255 to +255
 *   pan_angle / tilt_angle: 0 to 180 degrees
 *
 * Hardware:
 *   DC motors  : Adafruit Motor Shield channels 3 (left) and 4 (right)
 *   Pan servo  : pin 10
 *   Tilt servo : pin 9
 */

#include <Wire.h>
#include <AFMotor.h>
#include <Servo.h>

// ── Motors ─────────────────────────────────────────────────────────────────
AF_DCMotor motorLeft(3);
AF_DCMotor motorRight(4);

// ── Servos ─────────────────────────────────────────────────────────────────
Servo panServo;
Servo tiltServo;

const int PAN_PIN  = 10;
const int TILT_PIN = 9;

// ── Targets (set from ISR) ──────────────────────────────────────────────────
volatile int target_left_pwm  = 0;
volatile int target_right_pwm = 0;
volatile int target_pan_angle  = 90;
volatile int target_tilt_angle = 90;

// ── Current servo positions (smoothed in loop) ────────────────────────────
int current_pan_angle  = 90;
int current_tilt_angle = 90;

unsigned long lastServoMoveTime = 0;
const int SERVO_SPEED_DELAY = 1;  // ms per 1-degree step

// ── I2C receive buffer ────────────────────────────────────────────────────
#define I2C_BUFFER_SIZE 32
char i2cBuffer[I2C_BUFFER_SIZE + 1];
byte bufferIndex = 0;

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

  Wire.begin(0x08);
  Wire.onReceive(receiveEvent);
}

void loop() {
  controlMotor(motorLeft,  target_left_pwm);
  controlMotor(motorRight, target_right_pwm);
  smoothServoMove();
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

// ── I2C ISR: buffer bytes, parse on newline ───────────────────────────────
void receiveEvent(int bytesReceived) {
  while (Wire.available()) {
    char c = Wire.read();
    if (c == '\n') {
      i2cBuffer[bufferIndex] = '\0';
      parseMessage(i2cBuffer);
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

// ── Parse "{left},{right},{pan},{tilt}" ────────────────────────────────────
void parseMessage(const char* msg) {
  int lp = 0, rp = 0, pa = 90, ta = 90;
  int n = sscanf(msg, "%d,%d,%d,%d", &lp, &rp, &pa, &ta);
  if (n == 4) {
    target_left_pwm  = lp;
    target_right_pwm = rp;
    target_pan_angle  = constrain(pa, 0, 180);
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
