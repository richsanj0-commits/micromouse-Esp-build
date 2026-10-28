// =========================================================================
// TURN TEST (90 deg on the spot)
//   BTN1 (GPIO 13) -> turn RIGHT 90 deg
//   BTN2 (GPIO 4)  -> turn LEFT 90 deg
// Waits 1 s after the press so your hand is clear, then turns, keeping
// both wheels equal, slows down near the end and brakes.
//
// WIRELESS: joins the "alienware" hotspot. Output goes to USB serial AND to
// a WiFi terminal on port 23 (no cable needed):
//   pio device monitor --port socket://micromouse.local:23
// Commands over WiFi or USB (type + Enter):
//   r = right 90   l = left 90   a<deg> = any angle (a-180 = left 180)   1/2 = fwd/back 2000 counts
// =========================================================================
#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
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

// Accept a new WiFi terminal and read commands from USB or WiFi
String inputLine;

bool readCommand(String &cmd) {
  if (telnetServer.hasClient()) {
    if (telnet && telnet.connected()) telnet.stop();
    telnet = telnetServer.available();
    logf("WiFi terminal connected. Commands: 1 = fwd 2000, 2 = back 2000, c<counts>, d<mm>, t\n");
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

// --- DRIVE TUNING ---
const int   MAX_PWM   = 90;    // cruise speed
const int   MIN_PWM   = 45;    // slowest speed that still moves the robot
const float SLOW_MM   = 25.0f; // start slowing down this far before the target
const float KP_STRAIGHT = 0.6f; // PWM per count of left/right difference

// --- ENCODERS (4x quadrature, same as calibration sketch) ---
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

// --- DRIVE ---
const int BRAKE_LEAD = 70;  // counts the robot slides after braking -> brake this early

// Drive both wheels the same number of counts (negative = backwards)
void driveCounts(int32_t counts) {
  const int dir = counts >= 0 ? 1 : -1;
  const float target = abs(counts) - BRAKE_LEAD;
  logf("\nDriving %s %ld counts (%.1f mm)...\n", dir > 0 ? "FORWARD" : "BACKWARD",
       (long)abs(counts), abs(counts) * MM_PER_COUNT_L);
  zeroCounts();
  unsigned long t0 = millis();

  while (true) {
    int32_t l, r;
    readCounts(l, r);
    l *= dir;   // work in "progress" counts, positive in the driving direction
    r *= dir;

    // Safety: a wheel going the wrong way means a motor is wired the other way
    if (millis() - t0 > 300 && (l < -50 || r < -50)) {
      stopMotors();
      logf("STOPPED: %s wheel runs the wrong way - swap that motor's direction\n",
                    l < -50 ? "LEFT" : "RIGHT");
      return;
    }

    float travelled = (l + r) / 2.0f;
    float remaining = target - travelled;
    if (remaining <= 0) break;

    if (millis() - t0 > 4000) {
      stopMotors();
      logf("TIMEOUT: robot did not reach the distance (stuck? MIN_PWM too low?)\n");
      return;
    }

    // Slow down near the end
    float remainingMm = remaining * MM_PER_COUNT_L;
    int base = MAX_PWM;
    if (remainingMm < SLOW_MM)
      base = MIN_PWM + (MAX_PWM - MIN_PWM) * remainingMm / SLOW_MM;

    // Keep straight: if left is ahead, slow left / speed up right
    int correction = KP_STRAIGHT * (l - r);
    setMotorLeft(dir * (base - correction));
    setMotorRight(dir * (base + correction));
    delay(2);
  }

  brake();
  delay(300);
  stopMotors();

  int32_t l, r;
  readCounts(l, r);
  logf("Done in %lu ms: L %ld counts | R %ld counts | target %ld (error L %+ld, R %+ld)\n",
       millis() - t0, (long)l, (long)r, (long)counts, (long)(l - counts), (long)(r - counts));
  logf("= L %.1f mm, R %.1f mm. Measure with a ruler and compare.\n\n",
       l * MM_PER_COUNT_L, r * MM_PER_COUNT_R);
}

void driveDistance(float mm) {
  driveCounts(lroundf(mm / MM_PER_COUNT_L));
}

// --- TURN IN PLACE ---
const int TURN_PWM = 80;
const int TURN_BRAKE_LEAD = 40;  // counts each wheel slides after braking

// Positive degrees = turn right (left wheel forward, right wheel backward)
void turnDegrees(float deg) {
  const int dir = deg >= 0 ? 1 : -1;
  const int32_t counts = lroundf(fabsf(deg) / 90.0f * COUNTS_PER_90_TURN);
  const float target = counts - TURN_BRAKE_LEAD;
  logf("\nTurning %s %.0f deg (%ld counts per wheel)...\n", dir > 0 ? "RIGHT" : "LEFT",
       fabsf(deg), (long)counts);
  zeroCounts();
  unsigned long t0 = millis();

  while (true) {
    int32_t l, r;
    readCounts(l, r);
    int32_t pl = dir * l;    // progress of each wheel, positive while turning
    int32_t pr = -dir * r;

    if (millis() - t0 > 300 && (pl < -50 || pr < -50)) {
      stopMotors();
      logf("STOPPED: %s wheel runs the wrong way\n", pl < -50 ? "LEFT" : "RIGHT");
      return;
    }

    float travelled = (pl + pr) / 2.0f;
    float remaining = target - travelled;
    if (remaining <= 0) break;

    if (millis() - t0 > 4000) {
      stopMotors();
      logf("TIMEOUT: turn did not finish\n");
      return;
    }

    // Slow down for the last 20%
    int base = TURN_PWM;
    if (remaining < counts * 0.2f)
      base = MIN_PWM + (TURN_PWM - MIN_PWM) * remaining / (counts * 0.2f);

    // Keep both wheels turning the same amount
    int correction = KP_STRAIGHT * (pl - pr);
    setMotorLeft(dir * (base - correction));
    setMotorRight(-dir * (base + correction));
    delay(2);
  }

  brake();
  delay(300);
  stopMotors();

  int32_t l, r;
  readCounts(l, r);
  float turned = (dir * l - dir * r) / 2.0f / COUNTS_PER_90_TURN * 90.0f;
  logf("Done: L %+ld | R %+ld counts (target %ld each) = %.1f deg\n\n",
       (long)l, (long)r, (long)counts, dir * turned);
}

// Run each motor alone briefly with a "forward" command and report which
// encoder moved and in which direction (forward = positive counts)
void motorCheck() {
  for (int m = 0; m < 2; m++) {
    zeroCounts();
    if (m == 0) setMotorLeft(70); else setMotorRight(70);
    delay(250);
    brake();
    delay(200);
    stopMotors();
    int32_t l, r;
    readCounts(l, r);
    logf("%s motor forward cmd -> L enc %+ld, R enc %+ld\n",
         m == 0 ? "LEFT (A) " : "RIGHT (B)", (long)l, (long)r);
    delay(300);
  }
  logf("Expected: LEFT cmd moves only L (+), RIGHT cmd moves only R (+)\n\n");
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
  // Motors + WiFi starting together can dip the supply
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

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

  setupWiFi();
  Serial.println("Turn test ready: BTN1 = right 90, BTN2 = left 90");
}

void loop() {
  if (buttonPressed(BTN1)) { delay(1000); turnDegrees(90); }
  if (buttonPressed(BTN2)) { delay(1000); turnDegrees(-90); }

  String cmd;
  if (readCommand(cmd)) {
    if (cmd == "r") turnDegrees(90);
    else if (cmd == "l") turnDegrees(-90);
    else if (cmd.startsWith("a") && cmd.substring(1).toFloat() != 0) turnDegrees(cmd.substring(1).toFloat());
    else if (cmd == "1") driveCounts(2000);
    else if (cmd == "2") driveCounts(-2000);
    else if (cmd.startsWith("c") && cmd.substring(1).toInt() != 0) driveCounts(cmd.substring(1).toInt());
    else if (cmd == "t") motorCheck();
    else if (cmd.startsWith("d") && cmd.substring(1).toFloat() > 0) driveDistance(cmd.substring(1).toFloat());
    else logf("Unknown command. Use r, l, a<deg>, 1, 2, c<counts>, d<mm>, t\n");
  }
}
