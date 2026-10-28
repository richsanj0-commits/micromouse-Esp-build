// =========================================================================
// MAZE TEST - cell-by-cell moves, wall reading, left-hand wall follower
//
// Placement: robot in the start cell facing the opening, BACK TOUCHING the
// back wall, centred left/right by eye. (Axle is then 58 mm behind the
// cell centre - the first move adds that.)
//
//   BTN1 (GPIO 13) -> calibrate side sensors, then AUTO run (left-hand rule)
//                     until back in the start cell (max 30 moves)
//   BTN2 (GPIO 4)  -> stop (also 'x' over WiFi)
//
// WiFi terminal (port 23) / USB commands:
//   g = auto run     c = calibrate only (robot at start position)
//   f = 1 cell fwd   r / l = turn right / left 90   b = turn 180
//   w = read walls   s = live sensor values once    p = reset position to start
// =========================================================================
#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <Wire.h>
#include <VL53L0X.h>
#include "soc/gpio_struct.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "robot_config.h"

// --- WIFI ---
#include "wifi_secrets.h"  // local only, see wifi_secrets.example.h
const char* WIFI_SSID = WIFI_SSID_VALUE;
const char* WIFI_PASS = WIFI_PASS_VALUE;
const char* HOSTNAME  = "micromouse";

WiFiServer telnetServer(23);
WiFiClient telnet;

// Print to USB serial and to the WiFi terminal (if connected)
void logf(const char *fmt, ...) {
  char buf[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  Serial.print(buf);
  if (telnet && telnet.connected()) telnet.print(buf);
}

void setupWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(HOSTNAME);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("WiFi: connecting to %s", WIFI_SSID);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 8000) {
    delay(250);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    MDNS.begin(HOSTNAME);
    telnetServer.begin();
    telnetServer.setNoDelay(true);
    Serial.printf("\nWiFi OK: %s  (telnet port 23, %s.local)\n",
                  WiFi.localIP().toString().c_str(), HOSTNAME);
  } else {
    Serial.println("\nWiFi not found - buttons still work, output on USB only");
  }
}

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

// ToF (order: L, LF, F, RF, R)
#define I2C_SDA 21
#define I2C_SCL 22
enum { S_L, S_LF, S_F, S_RF, S_R };
const uint8_t XSHUT_PIN[5] = {23, 19, 18, 17, 16};
const uint8_t ADDR[5]      = {0x30, 0x31, 0x32, 0x33, 0x34};
const char*   NAME[5]      = {"L", "LF", "F", "RF", "R"};

const int pwmFreq = 5000;
const int pwmChannelLeft = 0;
const int pwmChannelRight = 1;
const int pwmResolution = 8; // 0-255

// --- MAZE GEOMETRY (16 mm walls on a 180 mm grid -> 164 mm corridors) ---
const float START_OFFSET_MM = 58.0f;  // axle behind cell centre with back on the back wall (CAD)

// --- MOTION TUNING (from drive / turn / characterization tests) ---
const int   MAX_PWM      = 90;
const int   MIN_PWM      = 45;
const float SLOW_MM      = 25.0f;
const float KP_STRAIGHT  = 0.6f;   // PWM per count of L/R encoder difference
const int   BRAKE_LEAD   = 70;     // counts slid after braking at MAX_PWM
const int   TURN_PWM     = 80;
const int   TURN_BRAKE_LEAD = 40;

// --- WALL THRESHOLDS (raw ToF mm; side ones are set by calibration) ---
const float KP_WALL      = 0.8f;   // PWM per mm of centring error
const int   SIDE_WALL_MARGIN = 60; // wall if raw < calibrated centre value + this
const int   FRONT_WALL_RAW   = 120; // F raw below this at a cell centre = wall ahead
const int   DIAG_WALL_RAW    = 95;  // LF/RF raw below this at a cell centre = wall ahead
const int   FRONT_ABORT_RAW  = 60;  // unexpected wall this close while moving -> stop
int sideRefL = 85;   // raw L / R with the robot centred (overwritten by calibration)
int sideRefR = 85;

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
    Serial.printf("ToF %-2s %s\n", NAME[i], tofOk[i] ? "OK" : "FAIL");
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
// STOP REQUEST (BTN2 or 'x' over WiFi / USB)
// =========================================================================
bool abortFlag = false;

bool checkAbort() {
  if (digitalRead(BTN2) == LOW) abortFlag = true;
  if (Serial.available() && Serial.peek() == 'x') { Serial.read(); abortFlag = true; }
  if (telnet && telnet.connected() && telnet.available() && telnet.peek() == 'x') {
    telnet.read();
    abortFlag = true;
  }
  return abortFlag;
}

// =========================================================================
// MOVES
// =========================================================================
// Drive straight forward by mm. Uses the side walls to stay centred when both
// are seen, otherwise keeps the wheel counts equal. Returns false if stopped.
bool forwardMm(float mm) {
  const float target = mm / MM_PER_COUNT_L - BRAKE_LEAD;
  const float total = mm / MM_PER_COUNT_L;
  zeroCounts();
  int32_t encOffset = 0;      // L-R difference to hold when not using walls
  bool usingWalls = false;
  unsigned long t0 = millis();

  while (true) {
    if (checkAbort()) { brakeStop(); logf("STOP requested\n"); return false; }
    pollToF();
    int32_t l, r;
    readCounts(l, r);
    float travelled = (l + r) / 2.0f;
    float remaining = target - travelled;
    if (remaining <= 0) break;
    if (millis() - t0 > 5000) { brakeStop(); logf("TIMEOUT in forward move\n"); return false; }

    // Unexpected wall ahead (before the last 35% of the move)
    if (total - travelled > 0.35f * total && tofRaw[S_F] > 0 && tofRaw[S_F] < FRONT_ABORT_RAW) {
      brakeStop();
      logf("STOP: unexpected wall ahead (F raw %d)\n", tofRaw[S_F]);
      return false;
    }

    float remainingMm = remaining * MM_PER_COUNT_L;
    int base = MAX_PWM;
    if (remainingMm < SLOW_MM) base = MIN_PWM + (MAX_PWM - MIN_PWM) * remainingMm / SLOW_MM;

    // Centring: positive error = robot is right of centre -> steer left
    bool leftWall  = tofRaw[S_L] < sideRefL + SIDE_WALL_MARGIN;
    bool rightWall = tofRaw[S_R] < sideRefR + SIDE_WALL_MARGIN;
    float correction;
    if (leftWall && rightWall) {
      float err = (tofRaw[S_L] - sideRefL) - (tofRaw[S_R] - sideRefR);
      correction = KP_WALL * err / 2.0f;
      usingWalls = true;
    } else {
      if (usingWalls) { encOffset = l - r; usingWalls = false; }
      correction = KP_STRAIGHT * ((l - r) - encOffset);
    }
    correction = constrain(correction, -25.0f, 25.0f);
    setMotorLeft(base - correction);
    setMotorRight(base + correction);
    delay(2);
  }
  brakeStop();
  return true;
}

// Turn in place. Positive = right. Returns false if stopped.
bool turnDegrees(float deg) {
  const int dir = deg >= 0 ? 1 : -1;
  const int32_t counts = lroundf(fabsf(deg) / 90.0f * COUNTS_PER_90_TURN);
  const float target = counts - TURN_BRAKE_LEAD;
  zeroCounts();
  unsigned long t0 = millis();

  while (true) {
    if (checkAbort()) { brakeStop(); logf("STOP requested\n"); return false; }
    int32_t l, r;
    readCounts(l, r);
    int32_t pl = dir * l;
    int32_t pr = -dir * r;
    float remaining = target - (pl + pr) / 2.0f;
    if (remaining <= 0) break;
    if (millis() - t0 > 4000) { brakeStop(); logf("TIMEOUT in turn\n"); return false; }

    int base = TURN_PWM;
    if (remaining < counts * 0.2f) base = MIN_PWM + (TURN_PWM - MIN_PWM) * remaining / (counts * 0.2f);
    int correction = KP_STRAIGHT * (pl - pr);
    setMotorLeft(dir * (base - correction));
    setMotorRight(-dir * (base + correction));
    delay(2);
  }
  brakeStop();
  return true;
}

// =========================================================================
// POSITION + WALLS
// =========================================================================
int posX = 0, posY = 0;   // cell, start = (0,0)
int heading = 0;          // 0 = N, 1 = E, 2 = S, 3 = W (start facing N)
bool atStartOffset = true; // robot still at the start placement (back on wall)
const char HEAD[4] = {'N', 'E', 'S', 'W'};

struct Walls { bool left, front, right; int raw[5]; };

Walls readWalls() {
  Walls w;
  readToFAveraged(w.raw);
  w.left  = w.raw[S_L] < sideRefL + SIDE_WALL_MARGIN;
  w.right = w.raw[S_R] < sideRefR + SIDE_WALL_MARGIN;
  w.front = w.raw[S_F] < FRONT_WALL_RAW ||
            (w.raw[S_LF] < DIAG_WALL_RAW && w.raw[S_RF] < DIAG_WALL_RAW);
  return w;
}

void printWalls(const Walls &w) {
  logf("Cell (%d,%d) facing %c | walls L:%d F:%d R:%d | raw L %d LF %d F %d RF %d R %d\n",
       posX, posY, HEAD[heading], w.left, w.front, w.right,
       w.raw[S_L], w.raw[S_LF], w.raw[S_F], w.raw[S_RF], w.raw[S_R]);
}

bool moveOneCell() {
  float mm = CELL_SIZE_MM + (atStartOffset ? START_OFFSET_MM : 0);
  if (!forwardMm(mm)) return false;
  atStartOffset = false;
  static const int DX[4] = {0, 1, 0, -1};
  static const int DY[4] = {1, 0, -1, 0};
  posX += DX[heading];
  posY += DY[heading];
  return true;
}

bool turnRight()  { if (!turnDegrees(90))   return false; heading = (heading + 1) % 4; return true; }
bool turnLeft()   { if (!turnDegrees(-90))  return false; heading = (heading + 3) % 4; return true; }
bool turnAround() { if (!turnDegrees(180))  return false; heading = (heading + 2) % 4; return true; }

void resetPosition() {
  posX = 0; posY = 0; heading = 0; atStartOffset = true;
  logf("Position reset: (0,0) facing N, at start placement\n");
}

// Side sensors with the robot at the start placement (walls left and right)
void calibrate() {
  int raw[5];
  readToFAveraged(raw, 10);
  logf("Calibration raw: L %d LF %d F %d RF %d R %d\n", raw[S_L], raw[S_LF], raw[S_F], raw[S_RF], raw[S_R]);
  if (raw[S_L] > 200 || raw[S_R] > 200) {
    logf("CALIBRATION FAILED: no side wall seen (L %d, R %d) - keeping old values\n", raw[S_L], raw[S_R]);
    return;
  }
  sideRefL = raw[S_L];
  sideRefR = raw[S_R];
  logf("Side refs: L %d, R %d (wall if raw < ref + %d)\n", sideRefL, sideRefR, SIDE_WALL_MARGIN);
}

// =========================================================================
// AUTO RUN: left-hand wall follower until back in the start cell
// =========================================================================
void autoRun() {
  abortFlag = false;
  resetPosition();
  calibrate();
  logf("\n=== AUTO RUN (left-hand rule) ===\n");
  unsigned long t0 = millis();

  for (int moves = 0; moves < 30; moves++) {
    Walls w = readWalls();
    printWalls(w);

    bool ok;
    if (!w.left)       { logf("-> turn LEFT + forward\n");  ok = (atStartOffset || turnLeft()) && moveOneCell(); }
    else if (!w.front) { logf("-> forward\n");              ok = moveOneCell(); }
    else if (!w.right) { logf("-> turn RIGHT + forward\n"); ok = (atStartOffset || turnRight()) && moveOneCell(); }
    else               { logf("-> dead end, turn 180 + forward\n"); ok = (atStartOffset || turnAround()) && moveOneCell(); }
    if (!ok) { logf("=== RUN STOPPED ===\n\n"); return; }

    if (posX == 0 && posY == 0) {
      logf("Back in the start cell after %d moves, %.1f s\n", moves + 1, (millis() - t0) / 1000.0f);
      printWalls(readWalls());
      logf("=== RUN DONE ===\n\n");
      return;
    }
  }
  logf("=== 30 moves reached, stopping ===\n\n");
}

// =========================================================================
// COMMANDS
// =========================================================================
String inputLine;

bool readCommand(String &cmd) {
  if (telnetServer.hasClient()) {
    if (telnet && telnet.connected()) telnet.stop();
    telnet = telnetServer.available();
    logf("WiFi terminal connected. g=auto c=calib f=fwd r/l=turn b=180 w=walls s=sensors p=reset x=stop\n");
  }
  while (Serial.available() || (telnet && telnet.available())) {
    char c = Serial.available() ? Serial.read() : telnet.read();
    if (c == 13 || c == 10) {  // Enter (CR / LF)
      if (inputLine.length() == 0) continue;
      cmd = inputLine;
      cmd.trim();
      cmd.toLowerCase();
      inputLine = "";
      return true;
    }
    if (isPrintable(c) && inputLine.length() < 20) inputLine += c;
  }
  return false;
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
  // Motors + WiFi + 5 sensors starting together can dip the supply
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  for (int i = 0; i < 5; i++) {
    pinMode(XSHUT_PIN[i], OUTPUT_OPEN_DRAIN);
    digitalWrite(XSHUT_PIN[i], LOW);
  }

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

  delay(300);
  initToF();
  setupWiFi();
  Serial.println("Maze test ready: BTN1 = auto run, BTN2 = stop");
}

void loop() {
  if (buttonPressed(BTN1)) { delay(1000); autoRun(); }

  String cmd;
  if (readCommand(cmd)) {
    abortFlag = false;
    if (cmd == "g") autoRun();
    else if (cmd == "c") calibrate();
    else if (cmd == "f") { if (moveOneCell()) printWalls(readWalls()); }
    else if (cmd == "r") { if (turnRight())  printWalls(readWalls()); }
    else if (cmd == "l") { if (turnLeft())   printWalls(readWalls()); }
    else if (cmd == "b") { if (turnAround()) printWalls(readWalls()); }
    else if (cmd == "w") printWalls(readWalls());
    else if (cmd == "s") {
      int raw[5];
      readToFAveraged(raw);
      logf("raw L %d LF %d F %d RF %d R %d\n", raw[S_L], raw[S_LF], raw[S_F], raw[S_RF], raw[S_R]);
    }
    else if (cmd == "p") resetPosition();
    else if (cmd == "x") logf("Nothing running\n");
    else logf("Unknown. g=auto c=calib f=fwd r/l=turn b=180 w=walls s=sensors p=reset x=stop\n");
  }
}
