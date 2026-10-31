// =========================================================================
// MAZE SOLVER - buttons only (no WiFi, no serial commands, no printouts)
// Same logic as examples/11 (BEST, video run) + button fix from examples/13.
//
// Placement (both runs): robot in the start cell facing into the maze,
// BACK TOUCHING the back wall. (Axle is then 58 mm behind the cell centre -
// the first move adds that.) Facing a wall at the start also works.
//
//   BTN1 (GPIO 13) -> explore until an open 2x2 (goal) is found, back to start
//   BTN2 (GPIO 4)  -> fast run start -> goal on the shortest explored path
//                     (during a run BTN2 = stop)
//   Blue LED (GPIO 2) on = press registered / run going
// The map is kept in RAM: power off = explore again.
// =========================================================================
#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include "soc/gpio_struct.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
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
#define LED_PIN 2   // blue LED on the ESP32 board: on = button press registered / run going

// ToF (order: L, LF, F, RF, R)
#define I2C_SDA 21
#define I2C_SCL 22
enum { S_L, S_LF, S_F, S_RF, S_R };
const uint8_t XSHUT_PIN[5] = {23, 19, 18, 17, 16};
const uint8_t ADDR[5]      = {0x30, 0x31, 0x32, 0x33, 0x34};

const int pwmFreq = 5000;
const int pwmChannelLeft = 0;
const int pwmChannelRight = 1;
const int pwmResolution = 8; // 0-255

// --- MAZE GEOMETRY (16 mm walls on a 180 mm grid -> 164 mm corridors) ---
const float START_OFFSET_MM = 58.0f;  // axle behind cell centre with back on the back wall (CAD)

// --- MOTION TUNING (from drive / turn / characterization tests) ---
const int   MAX_PWM      = 150;    // straight speed (90 in the video run, top of the linear motor range)
const int   MIN_PWM      = 45;
const float SLOW_MM      = 60.0f;  // slow-down distance before the end of a move (was 25 at PWM 90)
const float KP_STRAIGHT  = 0.6f;   // PWM per count of L/R encoder difference
const int   BRAKE_LEAD   = 190;    // counts slid after braking (70 at PWM 90; slide grows with speed^2)
const float ACCEL_MM     = 50.0f;  // speed ramps up over the first 50 mm of a move
const int   FRONT_SLOW_RAW = 200;  // front wall closer than this (raw F) -> slow down with distance
const int   TURN_PWM     = 80;
const int   TURN_BRAKE_LEAD = 40;

// --- WALL THRESHOLDS (raw ToF mm; side ones are set by calibration) ---
float kpWall    = 0.3f;   // degrees of steering per mm off-centre (0.15 in the video run - too weak)
float turnTrim  = 1.0f;   // extra turn scale on top of TURN_SCALE (encoder turns only)
const int   SIDE_WALL_RAW = 185;   // L/R raw below this = wall (maze test: walls <= 150, open >= 223)
const int   FRONT_WALL_RAW   = 120; // F raw below this at a cell centre = wall ahead
const int   DIAG_WALL_RAW    = 95;  // LF/RF raw below this at a cell centre = wall ahead
const int   FRONT_ABORT_RAW  = 60;  // unexpected wall this close while moving -> stop
const int   FRONT_STOP_RAW   = 55;  // F raw with the robot at a cell centre facing a wall
const int   DIAG_STOP_RAW    = 70;  // LF+RF average this close while driving = wall ahead (backup for F)
int sideRefL = 91;   // raw L / R with the robot centred (measured; overwritten by calibration)
int sideRefR = 91;

// =========================================================================
// ENCODERS (4x quadrature)
// =========================================================================
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

void zeroCounts() {
  noInterrupts();
  leftCount = 0;
  rightCount = 0;
  interrupts();
}

// =========================================================================
// MOTORS (directions verified on the robot)
// =========================================================================
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

// TB6612 short brake: IN1 = IN2 = HIGH
void brake() {
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, HIGH); ledcWrite(pwmChannelLeft, 255);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, HIGH); ledcWrite(pwmChannelRight, 255);
}

void stopMotors() {
  setMotorLeft(0);
  setMotorRight(0);
}

void brakeStop() {
  brake();
  delay(300);
  stopMotors();
}

// =========================================================================
// TOF SENSORS (continuous mode, read without blocking)
// =========================================================================
VL53L0X tof[5];
bool tofOk[5] = {false};
int tofRaw[5] = {0};   // latest raw mm (8190 = no target)

void initToF() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  for (int i = 0; i < 5; i++) {
    pinMode(XSHUT_PIN[i], OUTPUT_OPEN_DRAIN);
    digitalWrite(XSHUT_PIN[i], LOW);
  }
  delay(50);
  for (int i = 0; i < 5; i++) {
    digitalWrite(XSHUT_PIN[i], HIGH);
    delay(25);
    tof[i].setTimeout(100);
    tofOk[i] = tof[i].init();
    if (tofOk[i]) {
      tof[i].setAddress(ADDR[i]);
      tof[i].setMeasurementTimingBudget(33000);
      tof[i].startContinuous();
    }
    delay(20);
  }
}

// Picks up any new readings; returns immediately
void pollToF() {
  for (int i = 0; i < 5; i++) {
    if (!tofOk[i]) continue;
    if ((tof[i].readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) == 0) continue;
    tofRaw[i] = tof[i].readReg16Bit(VL53L0X::RESULT_RANGE_STATUS + 10);
    tof[i].writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 0x01);
  }
}

// Fresh, averaged readings while standing still
void readToFAveraged(int out[5], int samples = 5) {
  long sum[5] = {0};
  for (int s = 0; s < samples; s++) {
    for (int i = 0; i < 5; i++) {
      if (!tofOk[i]) continue;
      sum[i] += tof[i].readRangeContinuousMillimeters();
    }
  }
  for (int i = 0; i < 5; i++) out[i] = tofOk[i] ? sum[i] / samples : 9999;
}

// =========================================================================
// STOP REQUEST (BTN2 during a run)
// =========================================================================
bool abortFlag = false;

bool checkAbort() {
  if (digitalRead(BTN2) == LOW) abortFlag = true;
  return abortFlag;
}

// =========================================================================
// MOVES
// =========================================================================
// Straight drive (defined after the gyro section, it steers with the gyro)
bool forwardMm(float mm);

// =========================================================================
// GYRO (MPU6050 at 0x68) - measures the real turn angle, wheel slip or not
// =========================================================================
const uint8_t MPU_ADDR = 0x68;
const float GYRO_LSB_PER_DPS = 65.5f;   // +-500 deg/s range
bool gyroOk = false;
uint8_t gyroAxis = 2;     // axis pointing up (found from gravity at start)
float gyroBias = 0;       // raw reading when still
float gyroBrakeDeg = 4.7f; // degrees it keeps turning after braking (tuned: 4 x 90 within 3 deg)
bool useGyroTurns = true;  // turn on the gyro (encoder turns slip 79-95 deg)
int yawSign = -1;          // set from the mounting at start, checked after every turn

// Straight-drive steering
float kpHeading = 3.0f;    // PWM per degree of heading error
// kpWall (above) = degrees of heading change per mm off-centre
const float MAX_WALL_STEER_DEG = 8.0f;  // wall correction never steers more than this (was 5)

void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

bool mpuRead(uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, n) != n) return false;
  for (int i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

float readGyroRaw() {
  uint8_t b[2];
  if (!mpuRead(0x43 + 2 * gyroAxis, b, 2)) return gyroBias;
  return (int16_t)((b[0] << 8) | b[1]);
}

// Robot must be still
void calibrateGyro() {
  if (!gyroOk) return;
  float sum = 0;
  for (int i = 0; i < 400; i++) {
    sum += readGyroRaw();
    delay(2);
  }
  gyroBias = sum / 400;
}

void initGyro() {
  mpuWrite(0x6B, 0x00);  // wake up
  delay(100);
  mpuWrite(0x1A, 0x03);  // low-pass filter ~44 Hz
  mpuWrite(0x1B, 0x08);  // +-500 deg/s
  uint8_t a[6];
  gyroOk = mpuRead(0x3B, a, 6);
  if (!gyroOk) return;   // no gyro -> turns use encoders only
  // The axis that feels gravity is the one pointing up = the turning axis
  int16_t acc[3];
  for (int i = 0; i < 3; i++) acc[i] = (int16_t)((a[2 * i] << 8) | a[2 * i + 1]);
  gyroAxis = 0;
  for (int i = 1; i < 3; i++)
    if (abs(acc[i]) > abs(acc[gyroAxis])) gyroAxis = i;
  // Axis pointing up + counter-clockwise positive -> a right turn reads negative
  yawSign = acc[gyroAxis] > 0 ? -1 : 1;
  calibrateGyro();
}

// --- HEADING (yaw) TRACKING: degrees, right = positive, 0 = start direction ---
float yaw = 0;           // where the robot points now
float targetYaw = 0;     // where it should point (multiple of 90)
unsigned long yawLast = 0;

void updateYaw() {
  unsigned long now = micros();
  if (gyroOk && yawLast != 0)
    yaw += yawSign * (readGyroRaw() - gyroBias) / GYRO_LSB_PER_DPS * (now - yawLast) / 1e6f;
  yawLast = now;
}

void resetYaw() {
  yaw = 0;
  targetYaw = 0;
  yawLast = micros();
}

// Turn in place on encoders only (fallback when there is no gyro)
bool turnEncoders(float deg);

// Turn in place using the gyro. Positive = right. Returns false if stopped.
// Turns to an ABSOLUTE direction (targetYaw += deg), so small errors from
// earlier turns are corrected instead of adding up.
bool turnDegrees(float deg) {
  if (!gyroOk || !useGyroTurns) return turnEncoders(deg);
  targetYaw += deg;
  updateYaw();
  const float startYaw = yaw;
  const float needed = targetYaw - startYaw;      // includes any earlier error
  const int dir = needed >= 0 ? 1 : -1;
  const float stopAt = fabsf(needed) - gyroBrakeDeg;
  zeroCounts();
  unsigned long t0 = millis();
  int retries = 0;
  float turnedAtZero = 0;   // robot angle when the wheel counts were last zeroed

  while (true) {
    if (checkAbort()) { brakeStop(); return false; }
    updateYaw();
    float turned = fabsf(yaw - startYaw);          // works even if yawSign is wrong
    float remaining = stopAt - turned;
    if (remaining <= 0) break;
    if (millis() - t0 > 6000) { brakeStop(); return false; }

    // Wheels spinning far more than the robot turns = a corner caught on a wall
    int32_t l, r;
    readCounts(l, r);
    float encDeg = (dir * l - dir * r) / 2.0f / COUNTS_PER_90_TURN * 90.0f;
    if (encDeg > 1.6f * (turned - turnedAtZero) + 90) {   // normal slip: wheels 100-230 for a 90
      brakeStop();
      if (++retries > 3) return false;
      // Back straight up a little to free the corner, then carry on turning
      zeroCounts();
      unsigned long tb = millis();
      while (millis() - tb < 1000) {
        readCounts(l, r);
        if ((l + r) / 2.0f * MM_PER_COUNT_L < -20) break;
        setMotorLeft(-(MIN_PWM + 10));
        setMotorRight(-(MIN_PWM + 10));
        updateYaw();
        delay(2);
      }
      brakeStop();
      updateYaw();
      zeroCounts();
      turnedAtZero = fabsf(yaw - startYaw);
      continue;
    }

    // Slow down for the last 25 degrees
    int base = TURN_PWM;
    if (remaining < 25) base = MIN_PWM + (TURN_PWM - MIN_PWM) * remaining / 25;

    // Keep both wheels turning the same amount
    int correction = KP_STRAIGHT * (dir * l + dir * r);
    setMotorLeft(dir * (base - correction));
    setMotorRight(-dir * (base + correction));
    delayMicroseconds(500);
  }

  // Brake, but keep measuring so we see the real final angle
  brake();
  unsigned long tb = millis();
  while (millis() - tb < 300) {
    updateYaw();
    delayMicroseconds(500);
  }
  stopMotors();

  // Check the gyro direction: the measured turn must have the commanded sign
  float signedTurn = yaw - startYaw;
  if (fabsf(signedTurn) > 30 && (signedTurn > 0) != (dir > 0)) {
    yawSign = -yawSign;
    yaw = startYaw - signedTurn;
  }
  return true;
}

// Drive straight forward by mm.
// - Gyro holds the heading (targetYaw), so it drives straight even if wheels slip
// - Side walls (either one or both) nudge that heading by a few degrees to
//   bring the robot back to the centre of the corridor
bool forwardMm(float mm) {
  const float target = mm / MM_PER_COUNT_L - BRAKE_LEAD;
  const float total = mm / MM_PER_COUNT_L;
  zeroCounts();
  updateYaw();
  for (int i = 0; i < 5; i++) tofRaw[i] = 0;   // forget readings from the last move (0 = ignored)
  int32_t encOffset = 0;
  unsigned long t0 = millis();
  bool creeping = false;

  while (true) {
    if (checkAbort()) { brakeStop(); return false; }
    pollToF();
    updateYaw();
    int32_t l, r;
    readCounts(l, r);
    float travelled = (l + r) / 2.0f;
    float remaining = target - travelled;
    bool nearEnd = total - travelled < 0.35f * total;

    // Front wall ahead near the end: its distance decides where to stop, not the encoders
    bool frontWall = tofRaw[S_F] > 0 && tofRaw[S_F] < FRONT_WALL_RAW;
    if (nearEnd && frontWall && tofRaw[S_F] <= FRONT_STOP_RAW) break;
    // Backup if F misses the wall (robot slightly angled): both diagonals see it close
    // (at the normal stop point LF+RF average ~75-80; side walls alone give ~130)
    if (nearEnd && tofRaw[S_LF] > 0 && tofRaw[S_RF] > 0 && (tofRaw[S_LF] + tofRaw[S_RF]) / 2 <= DIAG_STOP_RAW) break;
    if (remaining <= 0) {
      // Encoders say done but the front wall is still far -> creep up to it (max 60 mm extra)
      if (frontWall && tofRaw[S_F] > FRONT_STOP_RAW && -remaining * MM_PER_COUNT_L < 60) creeping = true;
      else break;
    }
    // ~5.7 s for one cell, longer for straights
    if (millis() - t0 > 3000 + mm * 15) { brakeStop(); return false; }

    // Unexpected wall ahead (before the last 35% of the move)
    if (!nearEnd && tofRaw[S_F] > 0 && tofRaw[S_F] < FRONT_ABORT_RAW) {
      brakeStop();
      return false;
    }

    float remainingMm = remaining * MM_PER_COUNT_L;
    int base = MAX_PWM;
    if (remainingMm < SLOW_MM) base = MIN_PWM + (MAX_PWM - MIN_PWM) * remainingMm / SLOW_MM;
    // Speed ramp at the start (less wheel slip = no sideways kick)
    float travelledMm = travelled * MM_PER_COUNT_L;
    if (travelledMm < ACCEL_MM) base = min(base, (int)(MIN_PWM + (MAX_PWM - MIN_PWM) * travelledMm / ACCEL_MM));
    // Front wall coming: slow down with its distance, so it never slides into it
    if (tofRaw[S_F] > 0 && tofRaw[S_F] < FRONT_SLOW_RAW)
      base = min(base, (int)(MIN_PWM + (tofRaw[S_F] - FRONT_STOP_RAW) * 0.5f));   // 0.8 in examples/17
    if (creeping) base = MIN_PWM;
    base = max(base, MIN_PWM);

    // Off-centre error in mm, positive = robot is right of centre
    bool leftWall  = tofRaw[S_L] > 0 && tofRaw[S_L] < SIDE_WALL_RAW;
    bool rightWall = tofRaw[S_R] > 0 && tofRaw[S_R] < SIDE_WALL_RAW;
    float offCentre = 0;
    if (leftWall && rightWall) offCentre = ((tofRaw[S_L] - sideRefL) - (tofRaw[S_R] - sideRefR)) / 2.0f;
    // One wall only: half strength (L and R sensors read a little differently)
    else if (leftWall)         offCentre = 0.5f * (tofRaw[S_L] - sideRefL);
    else if (rightWall)        offCentre = -0.5f * (tofRaw[S_R] - sideRefR);

    float correction;
    if (gyroOk && useGyroTurns) {
      // Right of centre -> aim a little left (smaller yaw)
      float steer = constrain(kpWall * offCentre, -MAX_WALL_STEER_DEG, MAX_WALL_STEER_DEG);
      float headingErr = yaw - (targetYaw - steer);   // positive = pointing too far right
      correction = kpHeading * headingErr;              // positive -> turn left
    } else {
      correction = KP_STRAIGHT * ((l - r) - encOffset);
    }
    correction = constrain(correction, -35.0f, 35.0f);   // was 25 at PWM 90; more room at higher speed
    setMotorLeft(base - correction);
    setMotorRight(base + correction);
    delay(2);
  }
  brakeStop();
  updateYaw();
  return true;
}

// Turn in place on encoder counts only (fallback). Positive = right.
bool turnEncoders(float deg) {
  const int dir = deg >= 0 ? 1 : -1;
  const int32_t counts = lroundf(fabsf(deg) / 90.0f * COUNTS_PER_90_TURN * turnTrim);
  const float target = counts - TURN_BRAKE_LEAD;
  zeroCounts();
  unsigned long t0 = millis();

  while (true) {
    if (checkAbort()) { brakeStop(); return false; }
    int32_t l, r;
    readCounts(l, r);
    int32_t pl = dir * l;
    int32_t pr = -dir * r;
    float done = (pl + pr) / 2.0f;
    float remaining = target - done;
    if (remaining <= 0) break;
    if (millis() - t0 > 4000) { brakeStop(); return false; }

    // Gentle start (first 20%) and slow down (last 20%) to avoid wheel slip
    int base = TURN_PWM;
    if (done < counts * 0.2f) base = MIN_PWM + (TURN_PWM - MIN_PWM) * done / (counts * 0.2f);
    if (remaining < counts * 0.2f) base = MIN_PWM + (TURN_PWM - MIN_PWM) * remaining / (counts * 0.2f);
    int correction = KP_STRAIGHT * (pl - pr);
    setMotorLeft(dir * (base - correction));
    setMotorRight(-dir * (base + correction));
    delay(1);
  }

  brake();
  delay(300);
  stopMotors();
  return true;
}

// =========================================================================
// POSITION + WALLS
// =========================================================================
int posX = 0, posY = 0;   // cell, start = (0,0)
int heading = 0;          // 0 = N, 1 = E, 2 = S, 3 = W (start facing N)
bool atStartOffset = true; // robot still at the start placement (back on wall)

struct Walls { bool left, front, right; int raw[5]; };

Walls readWalls() {
  Walls w;
  readToFAveraged(w.raw);
  w.left  = w.raw[S_L] < SIDE_WALL_RAW;
  w.right = w.raw[S_R] < SIDE_WALL_RAW;
  w.front = w.raw[S_F] < FRONT_WALL_RAW ||
            (w.raw[S_LF] < DIAG_WALL_RAW && w.raw[S_RF] < DIAG_WALL_RAW);
  return w;
}

const int DX[4] = {0, 1, 0, -1};   // cell step for N, E, S, W
const int DY[4] = {1, 0, -1, 0};

// Drive n cells straight in one move
bool moveCells(int n) {
  float mm = n * CELL_SIZE_MM + (atStartOffset ? START_OFFSET_MM : 0);
  if (!forwardMm(mm)) return false;
  atStartOffset = false;
  posX += DX[heading] * n;
  posY += DY[heading] * n;
  return true;
}

bool turnRight()  { if (!turnDegrees(90))   return false; heading = (heading + 1) % 4; return true; }
bool turnLeft()   { if (!turnDegrees(-90))  return false; heading = (heading + 3) % 4; return true; }
bool turnAround() { if (!turnDegrees(180))  return false; heading = (heading + 2) % 4; return true; }

void resetPosition() {
  posX = 0; posY = 0; heading = 0; atStartOffset = true;
  resetYaw();
}

// Side sensors with the robot at the start placement (walls left and right)
void calibrate() {
  calibrateGyro();
  resetYaw();
  int raw[5];
  readToFAveraged(raw, 10);
  if (raw[S_L] > 200 || raw[S_R] > 200) return;   // no side walls -> keep old values
  // Centre = middle of the two walls, so a robot placed off-centre still learns the true centre
  sideRefL = (raw[S_L] + raw[S_R]) / 2;
  sideRefR = sideRefL;
}

// =========================================================================
// MAZE MAP + RUNS
// Explore (BTN1): depth-first into unvisited cells (left, front, right
//   first), recording walls, until an open 2x2 block (the goal) is found.
//   Then back to the start on the shortest known path.
// Fast run (BTN2): flood fill over explored cells only, start -> goal,
//   straight cells joined into one move.
// Start for both: start cell, back against the wall, facing into the maze.
// =========================================================================
const int GRID = 32, OFF = 16;   // map covers -16..15 cells around the start
bool visited[GRID][GRID];        // walls of this cell are known
uint8_t wallMap[GRID][GRID];     // bit d set = wall on side d (0 N, 1 E, 2 S, 3 W)
uint8_t cameFrom[GRID * GRID];   // direction moved into each new cell (for backing up)
int16_t dist[GRID][GRID];        // flood fill: cells to the target, -1 = unreachable
bool goalFound = false;
int goalX = 0, goalY = 0;        // bottom-left cell of the 2x2 goal

bool inGrid(int x, int y) { return x >= -OFF && y >= -OFF && x < GRID - OFF && y < GRID - OFF; }
bool isVisited(int x, int y) { return inGrid(x, y) && visited[x + OFF][y + OFF]; }

void setWall(int x, int y, int d) {
  wallMap[x + OFF][y + OFF] |= 1 << d;
  int nx = x + DX[d], ny = y + DY[d];
  if (inGrid(nx, ny)) wallMap[nx + OFF][ny + OFF] |= 1 << ((d + 2) % 4);
}

// Can we drive from (x,y) to its neighbour on side d? Only between explored cells.
bool canGo(int x, int y, int d) {
  return isVisited(x, y) && isVisited(x + DX[d], y + DY[d]) && !(wallMap[x + OFF][y + OFF] & (1 << d));
}

// Record the walls of the current cell (first visit only)
void recordWalls(const Walls &w, bool wallBehind) {
  if (isVisited(posX, posY)) return;
  visited[posX + OFF][posY + OFF] = true;
  if (w.left)  setWall(posX, posY, (heading + 3) % 4);
  if (w.front) setWall(posX, posY, heading);
  if (w.right) setWall(posX, posY, (heading + 1) % 4);
  if (wallBehind) setWall(posX, posY, (heading + 2) % 4);
}

// Open 2x2 block with (x,y) bottom-left: all 4 explored, no walls inside, not the start
bool isOpenBlock(int x, int y) {
  if ((x == 0 || x == -1) && (y == 0 || y == -1)) return false;
  return canGo(x, y, 1) && canGo(x, y, 0) && canGo(x + 1, y, 0) && canGo(x, y + 1, 1);
}

// Openings in the 8 outer sides of the 2x2 block (all 4 cells explored -> all sides known)
int blockOpenings(int x, int y) {
  const int cx[4] = {x, x + 1, x, x + 1}, cy[4] = {y, y, y + 1, y + 1};
  const int outA[4] = {2, 2, 0, 0}, outB[4] = {3, 1, 3, 1};   // outer sides of each cell
  int n = 0;
  for (int i = 0; i < 4; i++) {
    if (!(wallMap[cx[i] + OFF][cy[i] + OFF] & (1 << outA[i]))) n++;
    if (!(wallMap[cx[i] + OFF][cy[i] + OFF] & (1 << outB[i]))) n++;
  }
  return n;
}

// Goal = open 2x2 with exactly ONE way in (like a real micromouse goal).
// The first open 2x2 of any kind is kept as a backup goal.
bool backupFound = false;
int backupX = 0, backupY = 0;

bool checkGoal() {
  for (int ox = -1; ox <= 0; ox++)
    for (int oy = -1; oy <= 0; oy++) {
      int x = posX + ox, y = posY + oy;
      if (!isOpenBlock(x, y)) continue;
      if (blockOpenings(x, y) == 1) { goalFound = true; goalX = x; goalY = y; return true; }
      if (!backupFound) { backupFound = true; backupX = x; backupY = y; }
    }
  return false;
}

// Flood fill from the target: dist = cells to go, over explored cells only
void flood(bool toGoal) {
  static int8_t qx[GRID * GRID], qy[GRID * GRID];
  int head = 0, tail = 0;
  for (int i = 0; i < GRID; i++) for (int j = 0; j < GRID; j++) dist[i][j] = -1;
  auto seed = [&](int x, int y) { dist[x + OFF][y + OFF] = 0; qx[tail] = x; qy[tail++] = y; };
  if (toGoal) { seed(goalX, goalY); seed(goalX + 1, goalY); seed(goalX, goalY + 1); seed(goalX + 1, goalY + 1); }
  else seed(0, 0);
  while (head < tail) {
    int x = qx[head], y = qy[head++];
    for (int d = 0; d < 4; d++) {
      int nx = x + DX[d], ny = y + DY[d];
      if (!canGo(x, y, d) || dist[nx + OFF][ny + OFF] >= 0) continue;
      dist[nx + OFF][ny + OFF] = dist[x + OFF][y + OFF] + 1;
      qx[tail] = nx; qy[tail++] = ny;
    }
  }
}

// Turn to an absolute direction (0..3) and drive n cells
bool goDir(int d, int n = 1) {
  int t = (d - heading + 4) % 4;
  if (t != 0 && atStartOffset) {           // at the start the axle is behind centre:
    if (!forwardMm(START_OFFSET_MM)) return false;   // drive to the centre first, then turn
    atStartOffset = false;
  }
  bool ok = true;
  if (t == 1) ok = turnRight();
  else if (t == 2) ok = turnAround();
  else if (t == 3) ok = turnLeft();
  return ok && moveCells(n);
}

// Drive down the flood fill to its target, joining straight cells into one move
bool followFlood() {
  if (dist[posX + OFF][posY + OFF] < 0) return false;   // no known path
  while (dist[posX + OFF][posY + OFF] > 0) {
    int cur = dist[posX + OFF][posY + OFF];
    int best = -1;
    for (int i = 0; i < 4 && best < 0; i++) {
      int d = (heading + i) % 4;   // straight on first = fewer turns
      if (canGo(posX, posY, d) && dist[posX + DX[d] + OFF][posY + DY[d] + OFF] == cur - 1) best = d;
    }
    if (best < 0) return false;   // path lost
    int n = 1, x = posX + DX[best], y = posY + DY[best];
    while (dist[x + OFF][y + OFF] > 0 && canGo(x, y, best) &&
           dist[x + DX[best] + OFF][y + DY[best] + OFF] == dist[x + OFF][y + OFF] - 1) {
      n++; x += DX[best]; y += DY[best];
    }
    if (!goDir(best, n)) return false;
  }
  return true;
}

void exploreRun() {
  abortFlag = false;
  resetPosition();
  calibrate();
  memset(visited, 0, sizeof(visited));
  memset(wallMap, 0, sizeof(wallMap));
  goalFound = false;
  backupFound = false;
  int depth = 0, moves = 0;

  while (true) {   // no move limit: runs until the goal is found or the maze is fully explored
    Walls w = readWalls();
    // Start facing a wall (walls L, F, R) -> the opening must be behind
    bool startBoxed = moves == 0 && w.left && w.front && w.right;
    recordWalls(w, moves == 0 && !startBoxed);   // start cell: back against a wall (normally)
    if (checkGoal()) break;

    // Open neighbours not yet visited, in order left, front, right
    int next = -1;
    const int rel[3] = {3, 0, 1};
    const bool wall[3] = {w.left, w.front, w.right};
    for (int i = 0; i < 3 && next < 0; i++) {
      int d = (heading + rel[i]) % 4;
      if (!wall[i] && inGrid(posX + DX[d], posY + DY[d]) && !isVisited(posX + DX[d], posY + DY[d])) next = d;
    }
    if (startBoxed) {
      atStartOffset = false;   // not backed onto a wall, so no start offset to drive first
      next = (heading + 2) % 4;
    }

    bool ok;
    if (next >= 0) {
      cameFrom[depth++] = next;
      ok = goDir(next);
    } else if (depth > 0) {
      int back = (cameFrom[--depth] + 2) % 4;
      ok = goDir(back);
    } else {
      // Whole maze explored and back at the start: no one-way-in 2x2 found,
      // so use the first open 2x2 seen (if any) as the goal
      if (backupFound) { goalFound = true; goalX = backupX; goalY = backupY; }
      return;
    }
    if (!ok) return;   // stopped
    moves++;
  }

  // Goal found -> back to start on the shortest known path
  flood(false);
  followFlood();
}

void fastRun() {
  abortFlag = false;
  if (!goalFound) return;   // explore first (BTN1)
  resetPosition();
  calibrate();
  flood(true);
  followFlood();
}

// =========================================================================
// BUTTONS
// =========================================================================
bool buttonPressed(int pin) {
  if (digitalRead(pin) == LOW) {
    delay(50); // Debounce
    if (digitalRead(pin) == LOW) {
      digitalWrite(LED_PIN, HIGH);   // press registered
      while (digitalRead(pin) == LOW) delay(10);
      return true;
    }
  }
  return false;
}

void setup() {
  // Motors + 5 sensors starting together can dip the supply
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  for (int i = 0; i < 5; i++) {
    pinMode(XSHUT_PIN[i], OUTPUT_OPEN_DRAIN);
    digitalWrite(XSHUT_PIN[i], LOW);
  }

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
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  delay(300);
  initToF();
  initGyro();
}

void loop() {
  updateYaw();
  if (buttonPressed(BTN1)) { digitalWrite(LED_PIN, HIGH); delay(1000); exploreRun(); digitalWrite(LED_PIN, LOW); }
  if (buttonPressed(BTN2)) { digitalWrite(LED_PIN, HIGH); delay(1000); fastRun(); digitalWrite(LED_PIN, LOW); }   // while a run is going BTN2 = stop
}
