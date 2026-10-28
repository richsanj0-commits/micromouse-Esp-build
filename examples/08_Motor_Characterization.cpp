// =========================================================================
// MOTOR CHARACTERIZATION (PWM -> wheel speed), wheels in the air
//   Steps PWM up from low to full, holds each step, measures each wheel's
//   speed from the encoders. Forward sweep, then backward sweep.
//   Start: type "s" + Enter on serial (115200), or press BTN1 (GPIO 13).
//   "x" or BTN2 stops the sweep.
// =========================================================================
#include <Arduino.h>
#include "soc/gpio_struct.h"
#include "robot_config.h"

// --- PIN DEFINITIONS ---
// Motor Driver (TB6612)
#define PWMA 27
#define AIN1 14
#define AIN2 26
#define PWMB 32
#define BIN1 25
#define BIN2 33

// Buttons
#define BTN1 13
#define BTN2 4

const int pwmFreq = 5000;
const int pwmChannelLeft = 0;
const int pwmChannelRight = 1;
const int pwmResolution = 8; // 0-255

// --- SWEEP SETTINGS ---
const int PWM_STEPS[] = {10, 15, 20, 25, 30, 35, 40, 50, 60, 70, 80, 100, 120, 150, 180, 210, 255};
const int N_STEPS = sizeof(PWM_STEPS) / sizeof(PWM_STEPS[0]);
const int SETTLE_MS = 1000;   // let speed settle after each PWM change
const int MEASURE_MS = 2000;  // then count for this long

// --- ENCODERS (4x quadrature) ---
volatile int32_t leftCount = 0;
volatile int32_t rightCount = 0;
volatile uint8_t leftState = 0;
volatile uint8_t rightState = 0;

static const int8_t QDEC[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

static inline uint8_t IRAM_ATTR readAB(uint8_t pinA, uint8_t pinB) {
  uint32_t in = GPIO.in1.val;
  return (((in >> (pinA - 32)) & 1) << 1) | ((in >> (pinB - 32)) & 1);
}

void IRAM_ATTR leftEncoderISR() {
  uint8_t s = readAB(L_ENC_A, L_ENC_B);
  leftCount += QDEC[(leftState << 2) | s] * L_ENC_DIR;
  leftState = s;
}

void IRAM_ATTR rightEncoderISR() {
  uint8_t s = readAB(R_ENC_A, R_ENC_B);
  rightCount += QDEC[(rightState << 2) | s] * R_ENC_DIR;
  rightState = s;
}

void readCounts(int32_t &l, int32_t &r) {
  noInterrupts();
  l = leftCount;
  r = rightCount;
  interrupts();
}

// --- MOTORS (directions verified on the robot) ---
void setupMotors() {
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  ledcSetup(pwmChannelLeft, pwmFreq, pwmResolution);
  ledcSetup(pwmChannelRight, pwmFreq, pwmResolution);

  ledcAttachPin(PWMA, pwmChannelLeft);
  ledcAttachPin(PWMB, pwmChannelRight);
}

void setMotorLeft(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    ledcWrite(pwmChannelLeft, speed);
  } else if (speed < 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    ledcWrite(pwmChannelLeft, -speed);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    ledcWrite(pwmChannelLeft, 0);
  }
}

void setMotorRight(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    ledcWrite(pwmChannelRight, speed);
  } else if (speed < 0) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    ledcWrite(pwmChannelRight, -speed);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    ledcWrite(pwmChannelRight, 0);
  }
}

void stopMotors() {
  setMotorLeft(0);
  setMotorRight(0);
}

bool stopRequested() {
  if (digitalRead(BTN2) == LOW) return true;
  while (Serial.available()) {
    if (Serial.read() == 'x') return true;
  }
  return false;
}

// --- SWEEP ---
// Returns false if stopped early
bool sweep(int dir) {
  Serial.printf("\n=== %s ===\n", dir > 0 ? "FORWARD" : "BACKWARD");
  Serial.println("  PWM |  L counts/s   L rpm |  R counts/s   R rpm |  R/L");
  for (int k = 0; k < N_STEPS; k++) {
    int pwm = PWM_STEPS[k];
    setMotorLeft(dir * pwm);
    setMotorRight(dir * pwm);

    unsigned long t = millis();
    while (millis() - t < SETTLE_MS)
      if (stopRequested()) return false;

    int32_t l0, r0, l1, r1;
    readCounts(l0, r0);
    t = millis();
    while (millis() - t < MEASURE_MS)
      if (stopRequested()) return false;
    readCounts(l1, r1);

    float secs = MEASURE_MS / 1000.0f;
    float cl = (l1 - l0) / secs;
    float cr = (r1 - r0) / secs;
    float rpmL = cl / CPR_LEFT * 60.0f;
    float rpmR = cr / CPR_RIGHT * 60.0f;
    if (fabsf(cl) > 1)
      Serial.printf("  %3d | %10.0f %7.1f | %10.0f %7.1f | %5.3f\n", pwm, cl, rpmL, cr, rpmR, cr / cl);
    else
      Serial.printf("  %3d | %10.0f %7.1f | %10.0f %7.1f |   -\n", pwm, cl, rpmL, cr, rpmR);
  }
  return true;
}

void runCharacterization() {
  Serial.println("\nStarting sweep in 1 s - wheels must be OFF the ground. 'x' / BTN2 = stop");
  delay(1000);
  bool ok = sweep(1);
  stopMotors();
  delay(800);
  if (ok) ok = sweep(-1);
  stopMotors();
  Serial.println(ok ? "\n=== DONE ===\n" : "\n=== STOPPED ===\n");
}

bool buttonPressed(int pin) {
  if (digitalRead(pin) == LOW) {
    delay(50); // Debounce
    if (digitalRead(pin) == LOW) {
      while (digitalRead(pin) == LOW) delay(10);
      return true;
    }
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  setupMotors();
  stopMotors();

  pinMode(L_ENC_A, INPUT);
  pinMode(L_ENC_B, INPUT);
  pinMode(R_ENC_A, INPUT);
  pinMode(R_ENC_B, INPUT);
  leftState = readAB(L_ENC_A, L_ENC_B);
  rightState = readAB(R_ENC_A, R_ENC_B);
  attachInterrupt(digitalPinToInterrupt(L_ENC_A), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(L_ENC_B), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_ENC_A), rightEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_ENC_B), rightEncoderISR, CHANGE);

  pinMode(BTN1, INPUT_PULLUP);
  pinMode(BTN2, INPUT_PULLUP);

  Serial.println("Motor characterization ready: type s (or BTN1) to start, x (or BTN2) to stop");
}

void loop() {
  if (buttonPressed(BTN1)) runCharacterization();
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 's') runCharacterization();
  }
}
