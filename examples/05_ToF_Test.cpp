// =========================================================================
// TOF SENSOR TEST - 5x VL53L0X (Pololu library)
//
// Boot: brings sensors up one by one via XSHUT, gives each its own I2C
// address, prints an I2C scan and OK/FAIL per sensor.
//
// SERIAL COMMANDS (115200 baud, send with Enter):
//   a        -> average 50 readings per sensor: mean / spread / min / max
//   e<mm>    -> e.g. "e100": a flat wall is 100 mm from a sensor face,
//               prints the error of every sensor vs that distance
//   c f 100  -> calibrate: wall is 100 mm from the F sensor face. Repeat at
//               2-3 distances per sensor -> fitted gain/offset to paste in
//   x f      -> clear calibration points of F       k -> print corrections
//   r        -> re-init all sensors (after fixing a wire)
//   l        -> live stream on / off
//
// Readings: mm from the sensor face. "----" = no target (> ~1.2 m),
// "TOUT" = sensor stopped answering, "FAIL" = did not init.
// =========================================================================
#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// --- PIN DEFINITIONS ---
#define I2C_SDA 21
#define I2C_SCL 22

// Order: L, LF, F, RF, R
const uint8_t XSHUT_PIN[5] = {23, 19, 18, 17, 16};
const uint8_t ADDR[5]      = {0x30, 0x31, 0x32, 0x33, 0x34};
const char*   NAME[5]      = {"L", "LF", "F", "RF", "R"};

const uint32_t TIMING_BUDGET_US = 33000;  // 33 ms = default accuracy
const uint16_t NO_TARGET = 8000;          // library returns ~8190 for "nothing"

VL53L0X sensor[5];
bool ok[5] = {false};

// --- SENSOR SETUP ---
void i2cScan() {
  Serial.print("I2C scan:");
  int found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf(" 0x%02X", a);
      found++;
    }
  }
  Serial.printf("  (%d device%s)\n", found, found == 1 ? "" : "s");
}

void initSensors() {
  // All sensors off
  for (int i = 0; i < 5; i++) {
    pinMode(XSHUT_PIN[i], OUTPUT_OPEN_DRAIN);
    digitalWrite(XSHUT_PIN[i], LOW);
  }
  delay(50);

  // One at a time: wake -> init at 0x29 -> move to its own address
  for (int i = 0; i < 5; i++) {
    digitalWrite(XSHUT_PIN[i], HIGH);
    delay(25);
    sensor[i].setTimeout(200);
    ok[i] = sensor[i].init();
    if (ok[i]) {
      sensor[i].setAddress(ADDR[i]);
      sensor[i].setMeasurementTimingBudget(TIMING_BUDGET_US);
      sensor[i].startContinuous();
    }
    Serial.printf("  %-2s  XSHUT %2d  addr 0x%02X  %s\n",
                  NAME[i], XSHUT_PIN[i], ADDR[i], ok[i] ? "OK" : "FAIL  <-- check wiring / XSHUT");
    delay(20);
  }
  i2cScan();
  Serial.println("  (expect 0x30..0x34 + 0x68 for the MPU6050; 0x29 left over = a sensor stuck at default)\n");
}

// --- CORRECTION: true_mm = GAIN * raw + OFFSET (fitted with the "c" command) ---
float gain[5]   = {1, 1, 1, 1, 1};
float offset[5] = {0, 0, 0, 0, 0};

// Raw sensor value: mm, NO_TARGET, or -1 (fail / timeout)
int readRaw(int i) {
  if (!ok[i]) return -1;
  uint16_t mm = sensor[i].readRangeContinuousMillimeters();
  if (sensor[i].timeoutOccurred()) return -1;
  if (mm >= NO_TARGET) return NO_TARGET;
  return mm;
}

float correct(int i, float raw) {
  return gain[i] * raw + offset[i];
}

// Corrected value for display / use
int readSensor(int i) {
  int raw = readRaw(i);
  if (raw < 0 || raw == NO_TARGET) return raw;
  return max(0, (int)lroundf(correct(i, raw)));
}

void printValue(int i, int v) {
  if (!ok[i])              Serial.printf("%3s: FAIL ", NAME[i]);
  else if (v < 0)          Serial.printf("%3s: TOUT ", NAME[i]);
  else if (v == NO_TARGET) Serial.printf("%3s: ---- ", NAME[i]);
  else                     Serial.printf("%3s:%5d ", NAME[i], v);
}

// --- STATISTICS ---
const int N_SAMPLES = 50;

void averageAll(int wallMm) {
  float sum[5] = {0}, sumSq[5] = {0};
  int mn[5], mx[5], n[5] = {0};
  for (int i = 0; i < 5; i++) { mn[i] = 99999; mx[i] = -1; }

  Serial.printf("Sampling %d readings per sensor... keep still\n", N_SAMPLES);
  for (int s = 0; s < N_SAMPLES; s++) {
    for (int i = 0; i < 5; i++) {
      int v = readRaw(i);
      if (v < 0 || v == NO_TARGET) continue;
      sum[i] += v;
      sumSq[i] += (float)v * v;
      mn[i] = min(mn[i], v);
      mx[i] = max(mx[i], v);
      n[i]++;
    }
  }

  Serial.println("------------------------------------------------------------");
  Serial.println(" sensor    raw   +/-    min   max   valid  corrected" + String(wallMm > 0 ? "   error" : ""));
  for (int i = 0; i < 5; i++) {
    if (n[i] == 0) {
      Serial.printf("  %-3s    %s\n", NAME[i], ok[i] ? "no target / timeouts" : "FAIL");
      continue;
    }
    float mean = sum[i] / n[i];
    float sd = sqrtf(max(0.0f, sumSq[i] / n[i] - mean * mean));
    float corr = correct(i, mean);
    Serial.printf("  %-3s  %6.1f  %4.1f  %5d %5d   %2d/%d   %6.1f", NAME[i], mean, sd, mn[i], mx[i], n[i], N_SAMPLES, corr);
    if (wallMm > 0) Serial.printf("   %+6.1f mm", corr - wallMm);
    Serial.println();
  }
  Serial.println("------------------------------------------------------------\n");
}

// --- CALIBRATION: several (raw, true) points per sensor -> straight-line fit ---
const int MAX_POINTS = 8;
float calRaw[5][MAX_POINTS];
float calTrue[5][MAX_POINTS];
int calCount[5] = {0};

int sensorIndex(const String &s) {
  for (int i = 0; i < 5; i++)
    if (s.equalsIgnoreCase(NAME[i])) return i;
  return -1;
}

void printCorrections() {
  Serial.println("Paste into code:");
  Serial.print("  float gain[5]   = {");
  for (int i = 0; i < 5; i++) Serial.printf("%.4ff%s", gain[i], i < 4 ? ", " : "};\n");
  Serial.print("  float offset[5] = {");
  for (int i = 0; i < 5; i++) Serial.printf("%.1ff%s", offset[i], i < 4 ? ", " : "};\n");
  Serial.println();
}

// "c f 100" -> wall is 100 mm from the F sensor face
void calibratePoint(int i, int trueMm) {
  if (!ok[i]) { Serial.printf("%s not working\n", NAME[i]); return; }
  if (calCount[i] >= MAX_POINTS) { Serial.printf("%s has %d points - 'x %s' to clear\n", NAME[i], MAX_POINTS, NAME[i]); return; }

  Serial.printf("Calibrating %s at %d mm (50 readings, keep still)...\n", NAME[i], trueMm);
  float sum = 0, sumSq = 0;
  int n = 0;
  for (int s = 0; s < N_SAMPLES; s++) {
    int v = readRaw(i);
    if (v < 0 || v == NO_TARGET) continue;
    sum += v;
    sumSq += (float)v * v;
    n++;
  }
  if (n < N_SAMPLES / 2) { Serial.printf("Only %d/%d valid readings - point NOT saved\n\n", n, N_SAMPLES); return; }
  float mean = sum / n;
  float sd = sqrtf(max(0.0f, sumSq / n - mean * mean));

  calRaw[i][calCount[i]] = mean;
  calTrue[i][calCount[i]] = trueMm;
  calCount[i]++;

  Serial.printf("  %s points (true -> raw):\n", NAME[i]);
  for (int k = 0; k < calCount[i]; k++)
    Serial.printf("    %4.0f mm -> %6.1f  (error %+.1f)\n", calTrue[i][k], calRaw[i][k], calRaw[i][k] - calTrue[i][k]);
  Serial.printf("  last point spread +/- %.1f mm\n", sd);

  if (calCount[i] == 1) {
    gain[i] = 1;
    offset[i] = trueMm - mean;
    Serial.printf("  1 point -> offset only: %+.1f mm. Add a 2nd distance to check the gain.\n", offset[i]);
  } else {
    // Least-squares line: true = gain * raw + offset
    float sx = 0, sy = 0, sxx = 0, sxy = 0;
    int m = calCount[i];
    for (int k = 0; k < m; k++) {
      sx += calRaw[i][k]; sy += calTrue[i][k];
      sxx += calRaw[i][k] * calRaw[i][k]; sxy += calRaw[i][k] * calTrue[i][k];
    }
    float den = m * sxx - sx * sx;
    if (fabsf(den) < 1e-3f) { Serial.println("  Points too close together - use distances 50+ mm apart\n"); return; }
    gain[i] = (m * sxy - sx * sy) / den;
    offset[i] = (sy - gain[i] * sx) / m;
    float worst = 0;
    for (int k = 0; k < m; k++) worst = max(worst, fabsf(correct(i, calRaw[i][k]) - calTrue[i][k]));
    Serial.printf("  fit: true = %.4f x raw %+.1f   (worst fit error %.1f mm)\n", gain[i], offset[i], worst);
  }
  Serial.println();
  printCorrections();
}

// --- SERIAL COMMAND INPUT ---
// The PlatformIO monitor sends every key as it is typed, so collect
// characters until Enter instead of using readStringUntil() with a timeout.
String inputLine;

bool readCommand(String &cmd) {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') {
      if (inputLine.length() == 0) continue;
      cmd = inputLine;
      cmd.trim();
      cmd.toLowerCase();
      inputLine = "";
      Serial.println();
      return true;
    }
    if ((c == '\b' || c == 127) && inputLine.length()) {
      inputLine.remove(inputLine.length() - 1);
      Serial.print("\b \b");
    } else if (isPrintable(c) && inputLine.length() < 20) {
      inputLine += c;
      Serial.print(c);  // echo so you can see what you type
    }
  }
  return false;
}

// --- MAIN ---
bool live = true;
unsigned long lastPrint = 0;

void setup() {
  // Stops resets when 5 sensors power up at once
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  // Keep all ToF sensors off during boot
  for (int i = 0; i < 5; i++) {
    pinMode(XSHUT_PIN[i], OUTPUT_OPEN_DRAIN);
    digitalWrite(XSHUT_PIN[i], LOW);
  }

  Serial.begin(115200);
  delay(300);
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  Serial.println("\n==================================================");
  Serial.println(" 5x VL53L0X ToF TEST");
  Serial.println(" Commands: a = average, e100 = error vs 100 mm wall,");
  Serial.println("           c f 100 = calibrate F at 100 mm, k = show corrections");
  Serial.println("           r = re-init, l = live on/off");
  Serial.println("==================================================");
  initSensors();
}

void loop() {
  String cmd;
  if (readCommand(cmd)) {
    if (cmd == "a") averageAll(0);
    else if (cmd.startsWith("e") && cmd.substring(1).toInt() > 0) averageAll(cmd.substring(1).toInt());
    else if (cmd == "e") Serial.println("Give the wall distance, e.g. e100");
    else if (cmd.startsWith("c ")) {
      // c <sensor> <mm>
      String rest = cmd.substring(2);
      rest.trim();
      int sp = rest.indexOf(' ');
      int i = sp > 0 ? sensorIndex(rest.substring(0, sp)) : -1;
      int mm = sp > 0 ? rest.substring(sp + 1).toInt() : 0;
      if (i < 0 || mm <= 0) Serial.println("Use: c <L|LF|F|RF|R> <mm>   e.g. c f 100");
      else calibratePoint(i, mm);
    }
    else if (cmd.startsWith("x ")) {
      String rest = cmd.substring(2);
      rest.trim();
      int i = sensorIndex(rest);
      if (i < 0) Serial.println("Use: x <L|LF|F|RF|R>");
      else { calCount[i] = 0; gain[i] = 1; offset[i] = 0; Serial.printf("%s calibration cleared\n", NAME[i]); }
    }
    else if (cmd == "k") printCorrections();
    else if (cmd == "r") initSensors();
    else if (cmd == "l") { live = !live; Serial.printf("Live %s\n", live ? "ON" : "OFF"); }
    else if (cmd.length()) Serial.println("Unknown command. Use a, e<mm>, c <sensor> <mm>, x <sensor>, k, r, l");
  }

  // Pause the live stream while you are typing so the echo stays readable
  if (live && inputLine.length() == 0 && millis() - lastPrint >= 150) {
    lastPrint = millis();
    Serial.print("[ToF mm] ");
    for (int i = 0; i < 5; i++) printValue(i, readSensor(i));
    Serial.println();
  }
}
