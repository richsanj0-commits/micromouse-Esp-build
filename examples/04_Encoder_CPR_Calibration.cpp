// =========================================================================
// ENCODER CALIBRATION - Counts Per Wheel Revolution (CPR)
// Motors: 12V 600 RPM N20 gear motors with magnetic quadrature encoders
//
// Uses full 4x quadrature decoding (interrupts on BOTH A and B, every edge).
// MEASURED: ~2772 counts per wheel revolution (4x) on both wheels
// (The old 03_Encoder_Test counted only A edges = 2x, so it would show ~1386.)
//
// SERIAL COMMANDS (115200 baud, send with Enter):
//   z        -> zero both counters (same as BTN2 / GPIO 4)
//   <N>      -> e.g. "20" : you turned the wheels N full turns by hand,
//               saves a trial and prints the averaged counts per revolution
//   t / c    -> show trials / clear trials
//   v / v10  -> verify: spins each wheel exactly 1 (or 10) rev using the CPR
//               (v = BTN1 / GPIO 13)  ** lift the robot first **
//   s        -> speed test: full PWM for 2 s, prints wheel RPM
//               ** lift the robot first **
//   p        -> print current counts
// =========================================================================
#include <Arduino.h>
#include "soc/gpio_struct.h"

// --- PIN DEFINITIONS ---
// Motor Driver (TB6612)
#define PWMA 27
#define AIN1 14
#define AIN2 26
#define PWMB 32
#define BIN1 25
#define BIN2 33

// Encoders (GPIO 34-39 are input-only, need external 4.7k pull-ups to 3.3V)
// Verified on robot: left encoder is on 34/35, right on 36/39
#define L_ENC_A 34
#define L_ENC_B 35
#define R_ENC_A 36
#define R_ENC_B 39

// Verified on robot: both counted negative rolling forward -> inverted
#define L_ENC_DIR  1
#define R_ENC_DIR -1

// Buttons
#define BTN1 13   // verify 1 revolution
#define BTN2 4    // zero counters

const int pwmFreq = 5000;
const int pwmChannelLeft = 0;
const int pwmChannelRight = 1;
const int pwmResolution = 8; // 0-255

const int VERIFY_PWM = 70;          // slow speed for 1-rev verify spin
// Measured 2026-10-27: 10 hand turns -> L 27721, R 27723 counts (4x)
const float CPR_LEFT = 2772.1f;
const float CPR_RIGHT = 2772.3f;

float cprLeft = CPR_LEFT;
float cprRight = CPR_RIGHT;

// Repeated hand-turn trials, averaged for the final value
const int MAX_TRIALS = 10;
float trialL[MAX_TRIALS];
float trialR[MAX_TRIALS];
int trialCount = 0;

// --- ENCODER VARIABLES ---
volatile int32_t leftCount = 0;
volatile int32_t rightCount = 0;
volatile uint32_t leftErrors = 0;   // illegal transitions (noise / missed edges)
volatile uint32_t rightErrors = 0;
volatile uint8_t leftState = 0;
volatile uint8_t rightState = 0;

// Quadrature lookup: index = (prevAB << 2) | newAB
// 0 = no move, +1/-1 = one step, 2 = illegal (both bits changed)
static const int8_t QDEC[16] = {
   0, -1,  1,  2,
   1,  0,  2, -1,
  -1,  2,  0,  1,
   2,  1, -1,  0
};

// Fast register read (all encoder pins are GPIO 32-39 -> GPIO.in1)
static inline uint8_t IRAM_ATTR readAB(uint8_t pinA, uint8_t pinB) {
  uint32_t in = GPIO.in1.val;
  return (((in >> (pinA - 32)) & 1) << 1) | ((in >> (pinB - 32)) & 1);
}

// Interrupt Service Routines for Encoders
void IRAM_ATTR leftEncoderISR() {
  uint8_t s = readAB(L_ENC_A, L_ENC_B);
  int8_t d = QDEC[(leftState << 2) | s];
  if (d == 2) leftErrors++;
  else leftCount += d * L_ENC_DIR;
  leftState = s;
}

void IRAM_ATTR rightEncoderISR() {
  uint8_t s = readAB(R_ENC_A, R_ENC_B);
  int8_t d = QDEC[(rightState << 2) | s];
  if (d == 2) rightErrors++;
  else rightCount += d * R_ENC_DIR;
  rightState = s;
}

// --- MOTORS ---
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

// TB6612 short brake: IN1 = IN2 = HIGH, PWM = HIGH
void brakeLeft()  { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, HIGH); ledcWrite(pwmChannelLeft, 255); }
void brakeRight() { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, HIGH); ledcWrite(pwmChannelRight, 255); }

void stopMotors() {
  setMotorLeft(0);
  setMotorRight(0);
}

// --- HELPERS ---
void readCounts(int32_t &l, int32_t &r) {
  noInterrupts();
  l = leftCount;
  r = rightCount;
  interrupts();
}

void zeroCounts() {
  noInterrupts();
  leftCount = 0;
  rightCount = 0;
  leftErrors = 0;
  rightErrors = 0;
  interrupts();
  Serial.println("\n>>> ENCODERS ZEROED - now turn the wheels N full turns, then type N <<<\n");
}

void printCounts() {
  int32_t l, r;
  readCounts(l, r);
  Serial.printf("[Live] L: %7ld  (%.3f rev) | R: %7ld  (%.3f rev) | err L:%lu R:%lu\n",
                (long)l, l / cprLeft, (long)r, r / cprRight,
                (unsigned long)leftErrors, (unsigned long)rightErrors);
}

void printTrials();

void computeCPR(float turns) {
  int32_t l, r;
  readCounts(l, r);
  float cl = fabsf(l) / turns;
  float cr = fabsf(r) / turns;

  Serial.println("\n==================== RESULT ====================");
  Serial.printf("Turns entered : %.2f\n", turns);
  Serial.printf("LEFT  : %ld counts -> %.1f counts/rev (4x)  | %.1f (old 2x code)\n", (long)l, cl, cl / 2);
  Serial.printf("RIGHT : %ld counts -> %.1f counts/rev (4x)  | %.1f (old 2x code)\n", (long)r, cr, cr / 2);
  Serial.printf("Implied gear ratio if 7 PPR: L 1:%.1f  R 1:%.1f | if 14 PPR: L 1:%.1f  R 1:%.1f\n",
                cl / 28.0f, cr / 28.0f, cl / 56.0f, cr / 56.0f);
  if (l <= 0 || r <= 0)
    Serial.println("NOTE: a count is <= 0 -> rolled backwards, or flip L_ENC_DIR / R_ENC_DIR");
  if (leftErrors || rightErrors)
    Serial.println("WARNING: illegal transitions seen -> check wiring / pull-ups / noise");
  Serial.println("================================================\n");

  if (cl < 10 || cr < 10) {
    Serial.println("Trial NOT saved (a wheel did not move).\n");
    return;
  }
  if (trialCount < MAX_TRIALS) {
    trialL[trialCount] = cl;
    trialR[trialCount] = cr;
    trialCount++;
  } else {
    Serial.println("Trial list full - type 'c' to clear. Using existing trials.");
  }
  printTrials();
}

void statsOf(const float *v, int n, float &mean, float &sd) {
  mean = 0;
  for (int i = 0; i < n; i++) mean += v[i];
  mean /= n;
  sd = 0;
  for (int i = 0; i < n; i++) sd += (v[i] - mean) * (v[i] - mean);
  sd = n > 1 ? sqrtf(sd / (n - 1)) : 0;
}

void printTrials() {
  if (trialCount == 0) {
    Serial.println("No trials yet.\n");
    return;
  }
  Serial.println("------------------- TRIALS ---------------------");
  for (int i = 0; i < trialCount; i++)
    Serial.printf("  #%d  L %.1f   R %.1f\n", i + 1, trialL[i], trialR[i]);

  float mL, sL, mR, sR;
  statsOf(trialL, trialCount, mL, sL);
  statsOf(trialR, trialCount, mR, sR);
  Serial.printf("MEAN  L %.1f (+/- %.1f)   R %.1f (+/- %.1f)   over %d trials\n",
                mL, sL, mR, sR, trialCount);
  Serial.println("Paste into code:");
  Serial.printf("  const float CPR_LEFT = %.1ff;\n", mL);
  Serial.printf("  const float CPR_RIGHT = %.1ff;\n", mR);
  Serial.println("------------------------------------------------\n");

  cprLeft = mL;
  cprRight = mR;
  Serial.println("Mean saved for this session. Lift robot and type v10 to verify.\n");
}

// Spin one wheel exactly N revolutions so you can see the tape mark come back.
// More revs = any CPR error is multiplied, so it is easier to see.
void spinRevs(bool left, float revs) {
  float cpr = left ? cprLeft : cprRight;
  int32_t target = (int32_t)(cpr * revs + 0.5f);
  noInterrupts();
  leftCount = 0;
  rightCount = 0;
  interrupts();
  Serial.printf("Spinning %s wheel %.0f rev (target %ld counts)...\n",
                left ? "LEFT" : "RIGHT", revs, (long)target);

  unsigned long t0 = millis();
  if (left) setMotorLeft(VERIFY_PWM); else setMotorRight(VERIFY_PWM);
  while (true) {
    int32_t l, r;
    readCounts(l, r);
    int32_t c = abs(left ? l : r);
    if (c >= target) break;
    if (millis() - t0 > 5000 * revs) { Serial.println("TIMEOUT - no counts? check encoder wiring"); break; }
  }
  if (left) brakeLeft(); else brakeRight();
  delay(300);
  stopMotors();

  int32_t l, r;
  readCounts(l, r);
  int32_t c = left ? l : r;
  Serial.printf("%s stopped at %ld counts (overshoot %+ld = %.1f deg)\n",
                left ? "LEFT" : "RIGHT", (long)c, (long)(abs(c) - target),
                (abs(c) - target) * 360.0f / cpr);
  Serial.println("  Mark AHEAD of pointer (minus overshoot) -> CPR too big; BEHIND -> too small\n");
}

void verifyRevs(float revs) {
  spinRevs(true, revs);
  delay(500);
  spinRevs(false, revs);
}

void speedTest() {
  Serial.println("Speed test: full PWM for 2 s (wheels must be lifted!)");
  delay(1000);
  setMotorLeft(255);
  setMotorRight(255);
  delay(500);                   // let it spin up
  int32_t l0, r0, l1, r1;
  readCounts(l0, r0);
  delay(2000);
  readCounts(l1, r1);
  stopMotors();
  float rpmL = (l1 - l0) / cprLeft / 2.0f * 60.0f;
  float rpmR = (r1 - r0) / cprRight / 2.0f * 60.0f;
  Serial.printf("LEFT  %.0f RPM | RIGHT %.0f RPM (rated 600 @ 12V no-load)\n\n", rpmL, rpmR);
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

  // Every edge on A and B -> full 4x resolution
  attachInterrupt(digitalPinToInterrupt(L_ENC_A), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(L_ENC_B), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_ENC_A), rightEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_ENC_B), rightEncoderISR, CHANGE);

  pinMode(BTN1, INPUT_PULLUP);
  pinMode(BTN2, INPUT_PULLUP);

  Serial.println("==================================================");
  Serial.println(" ENCODER CPR CALIBRATION (4x quadrature)");
  Serial.println(" 1. Put a tape mark on each wheel, press BTN2 (GPIO4) or type z");
  Serial.println(" 2. Roll / turn BOTH wheels FORWARD exactly 10 full turns");
  Serial.println(" 3. Type 10 and press Enter -> counts per revolution");
  Serial.println(" 4. Lift robot, press BTN1 (GPIO13) or type v to verify");
  Serial.println(" Other: s = speed test, p = print counts");
  Serial.println("==================================================");
}

unsigned long lastPrintTime = 0;
int32_t lastL = 0, lastR = 0;

void loop() {
  if (buttonPressed(BTN2)) zeroCounts();
  if (buttonPressed(BTN1)) verifyRevs(1);

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd == "z") zeroCounts();
    else if (cmd.startsWith("v")) verifyRevs(cmd.length() > 1 ? max(1.0f, cmd.substring(1).toFloat()) : 1);
    else if (cmd == "s") speedTest();
    else if (cmd == "p") printCounts();
    else if (cmd == "t") printTrials();
    else if (cmd == "c") { trialCount = 0; Serial.println("Trials cleared.\n"); }
    else if (cmd.length() && cmd.toFloat() > 0) computeCPR(cmd.toFloat());
    else if (cmd.length()) Serial.println("Unknown command. Use z, <turns>, t, c, v, v10, s, p");
  }

  // Live print every 200 ms, only when something moved
  if (millis() - lastPrintTime >= 200) {
    lastPrintTime = millis();
    int32_t l, r;
    readCounts(l, r);
    if (l != lastL || r != lastR) {
      printCounts();
      lastL = l;
      lastR = r;
    }
  }
}
