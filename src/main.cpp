#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Wire.h>
#include <VL53L0X.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "dashboard_html.h"

// =========================================================================
// WI-FI STATION CONFIGURATION (HOTSPOT ONLY)
// =========================================================================
const char* WIFI_SSID = "alienware";
const char* WIFI_PASS = "12345678";

WebServer server(80);
String currentNetInfo = "Connecting to alienware...";

// =========================================================================
// PIN DEFINITIONS
// =========================================================================
#define I2C_SDA   21
#define I2C_SCL   22

// XSHUT Shutdown Pins for 5x VL53L0X ToF
#define XSHUT_L   23
#define XSHUT_LF  19
#define XSHUT_F   18
#define XSHUT_RF  17
#define XSHUT_R   16

// I2C Unique Sensor Addresses
#define ADDR_L    0x30
#define ADDR_LF   0x31
#define ADDR_F    0x32
#define ADDR_RF   0x33
#define ADDR_R    0x34

// Quadrature Encoders
#define L_ENC_A   36
#define L_ENC_B   39
#define R_ENC_A   34
#define R_ENC_B   35

// Buttons & Status LED
#define BTN1      4    // Press to zero tick counters
#define BTN2      13
#define LED_PIN   2

// =========================================================================
// GLOBALS & SENSORS
// =========================================================================
VL53L0X sensorL;
VL53L0X sensorLF;
VL53L0X sensorF;
VL53L0X sensorRF;
VL53L0X sensorR;

bool okL  = false;
bool okLF = false;
bool okF  = false;
bool okRF = false;
bool okR  = false;

uint16_t distL  = 9999;
uint16_t distLF = 9999;
uint16_t distF  = 9999;
uint16_t distRF = 9999;
uint16_t distR  = 9999;

// 64-bit atomic tick counters
volatile int64_t leftTicks  = 0;
volatile int64_t rightTicks = 0;

void IRAM_ATTR isrLeftEncoder() {
    int a = digitalRead(L_ENC_A);
    int b = digitalRead(L_ENC_B);
    if (a == b) leftTicks++;
    else        leftTicks--;
}

void IRAM_ATTR isrRightEncoder() {
    int a = digitalRead(R_ENC_A);
    int b = digitalRead(R_ENC_B);
    if (a == b) rightTicks++;
    else        rightTicks--;
}

// =========================================================================
// WIRELESS SERIAL LOG RING BUFFER
// =========================================================================
#define LOG_BUFFER_SIZE 30
struct LogEntry {
    uint32_t seq;
    String message;
};

LogEntry logBuffer[LOG_BUFFER_SIZE];
int logHead = 0;
int logCount = 0;
uint32_t currentLogSeq = 0;

void logMessage(const String& msg) {
    Serial.println(msg);
    currentLogSeq++;
    logBuffer[logHead].seq = currentLogSeq;
    logBuffer[logHead].message = msg;
    logHead = (logHead + 1) % LOG_BUFFER_SIZE;
    if (logCount < LOG_BUFFER_SIZE) {
        logCount++;
    }
}

// =========================================================================
// TOF SENSOR HARDWARE INITIALIZATION
// =========================================================================
void initToFSensors() {
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(100000); // 100kHz standard stable speed

    pinMode(XSHUT_L, OUTPUT_OPEN_DRAIN);
    pinMode(XSHUT_LF, OUTPUT_OPEN_DRAIN);
    pinMode(XSHUT_F, OUTPUT_OPEN_DRAIN);
    pinMode(XSHUT_RF, OUTPUT_OPEN_DRAIN);
    pinMode(XSHUT_R, OUTPUT_OPEN_DRAIN);

    // Keep all sensors in hardware standby
    digitalWrite(XSHUT_L, LOW);
    digitalWrite(XSHUT_LF, LOW);
    digitalWrite(XSHUT_F, LOW);
    digitalWrite(XSHUT_RF, LOW);
    digitalWrite(XSHUT_R, LOW);
    delay(40);

    // 1. Left Sensor
    digitalWrite(XSHUT_L, HIGH);
    delay(25);
    sensorL.setTimeout(200);
    if (sensorL.init()) {
        sensorL.setAddress(ADDR_L);
        sensorL.startContinuous(50);
        okL = true;
        logMessage("[ToF] Left Sensor (0x30): OK");
    } else {
        logMessage("[ToF] Left Sensor: NOT DETECTED");
    }
    delay(20);

    // 2. Left-Front Sensor
    digitalWrite(XSHUT_LF, HIGH);
    delay(25);
    sensorLF.setTimeout(200);
    if (sensorLF.init()) {
        sensorLF.setAddress(ADDR_LF);
        sensorLF.startContinuous(50);
        okLF = true;
        logMessage("[ToF] Left-Front (0x31): OK");
    } else {
        logMessage("[ToF] Left-Front: NOT DETECTED");
    }
    delay(20);

    // 3. Front Sensor
    digitalWrite(XSHUT_F, HIGH);
    delay(25);
    sensorF.setTimeout(200);
    if (sensorF.init()) {
        sensorF.setAddress(ADDR_F);
        sensorF.startContinuous(50);
        okF = true;
        logMessage("[ToF] Front Sensor (0x32): OK");
    } else {
        logMessage("[ToF] Front Sensor: NOT DETECTED");
    }
    delay(20);

    // 4. Right-Front Sensor
    digitalWrite(XSHUT_RF, HIGH);
    delay(25);
    sensorRF.setTimeout(200);
    if (sensorRF.init()) {
        sensorRF.setAddress(ADDR_RF);
        sensorRF.startContinuous(50);
        okRF = true;
        logMessage("[ToF] Right-Front (0x33): OK");
    } else {
        logMessage("[ToF] Right-Front: NOT DETECTED");
    }
    delay(20);

    // 5. Right Sensor
    digitalWrite(XSHUT_R, HIGH);
    delay(25);
    sensorR.setTimeout(200);
    if (sensorR.init()) {
        sensorR.setAddress(ADDR_R);
        sensorR.startContinuous(50);
        okR = true;
        logMessage("[ToF] Right Sensor (0x34): OK");
    } else {
        logMessage("[ToF] Right Sensor: NOT DETECTED");
    }
    delay(20);
}

// =========================================================================
// HTTP SERVER HANDLERS
// =========================================================================
void handleRoot() {
    server.send_P(200, "text/html", DASHBOARD_HTML);
}

void handleTelemetry() {
    uint32_t since = 0;
    if (server.hasArg("since")) {
        since = (uint32_t)server.arg("since").toInt();
    }

    noInterrupts();
    int64_t curL = leftTicks;
    int64_t curR = rightTicks;
    interrupts();

    String json = "{";
    json += "\"uptime_s\":" + String(millis() / 1000) + ",";
    json += "\"net_info\":\"" + currentNetInfo + "\",";

    // 5-ToF Distances (mm)
    json += "\"tof\":{\"l\":" + String(distL) + ",\"lf\":" + String(distLF) + ",\"f\":" + String(distF) + ",\"rf\":" + String(distRF) + ",\"r\":" + String(distR) + "},";
    json += "\"tof_ok\":{\"l\":" + String(okL ? "true" : "false") + ",\"lf\":" + String(okLF ? "true" : "false") + ",\"f\":" + String(okF ? "true" : "false") + ",\"rf\":" + String(okRF ? "true" : "false") + ",\"r\":" + String(okR ? "true" : "false") + "},";

    // Encoders
    json += "\"enc\":{\"left\":" + String((long)curL) + ",\"right\":" + String((long)curR) + "},";
    json += "\"seq\":" + String(currentLogSeq) + ",";

    // Buffered Serial Logs
    json += "\"logs\":[";
    bool first = true;
    for (int i = 0; i < logCount; i++) {
        int idx = (logHead - logCount + i + LOG_BUFFER_SIZE) % LOG_BUFFER_SIZE;
        if (logBuffer[idx].seq > since) {
            if (!first) json += ",";
            first = false;
            String escaped = logBuffer[idx].message;
            escaped.replace("\"", "\\\"");
            json += "\"" + escaped + "\"";
        }
    }
    json += "]}";

    server.send(200, "application/json", json);
}

void handleResetEncoders() {
    noInterrupts();
    leftTicks  = 0;
    rightTicks = 0;
    interrupts();
    logMessage(">>> [WEB] Encoder counters zeroed <<<");
    server.send(200, "text/plain", "OK");
}

void handleBlink() {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    server.send(200, "text/plain", "OK");
}

// =========================================================================
// SETUP & MAIN LOOP
// =========================================================================
bool lastBtnState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

void setup() {
    // Disable brownout detector for clean Wi-Fi boot
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

    // Keep all ToF sensors off immediately during boot
    pinMode(XSHUT_L, OUTPUT_OPEN_DRAIN);
    pinMode(XSHUT_LF, OUTPUT_OPEN_DRAIN);
    pinMode(XSHUT_F, OUTPUT_OPEN_DRAIN);
    pinMode(XSHUT_RF, OUTPUT_OPEN_DRAIN);
    pinMode(XSHUT_R, OUTPUT_OPEN_DRAIN);
    digitalWrite(XSHUT_L, LOW);
    digitalWrite(XSHUT_LF, LOW);
    digitalWrite(XSHUT_F, LOW);
    digitalWrite(XSHUT_RF, LOW);
    digitalWrite(XSHUT_R, LOW);

    Serial.begin(115200);
    delay(300);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    pinMode(L_ENC_A, INPUT);
    pinMode(L_ENC_B, INPUT);
    pinMode(R_ENC_A, INPUT);
    pinMode(R_ENC_B, INPUT);

    pinMode(BTN1, INPUT_PULLUP);
    pinMode(BTN2, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(L_ENC_A), isrLeftEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(R_ENC_A), isrRightEncoder, CHANGE);

    logMessage("==============================================");
    logMessage("  ESP32 MICROMOUSE WIRELESS TELEMETRY SYSTEM  ");
    logMessage("==============================================");

    // Station mode ONLY (Connect to Alienware Hotspot)
    WiFi.mode(WIFI_STA);
    WiFi.setTxPower(WIFI_POWER_11dBm);

    logMessage("[WiFi] Connecting to hotspot: " + String(WIFI_SSID) + "...");
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    // Wait for connection
    while (WiFi.status() != WL_CONNECTED) {
        delay(300);
        Serial.print(".");
    }
    Serial.println();

    String ip = WiFi.localIP().toString();
    currentNetInfo = "Hotspot: " + String(WIFI_SSID) + " • IP: " + ip;
    logMessage("[WiFi] >>> CONNECTED TO " + String(WIFI_SSID) + "! <<<");
    logMessage("[WiFi] Web Dashboard IP: http://" + ip);

    if (MDNS.begin("micromouse")) {
        logMessage("[mDNS] Local Domain: http://micromouse.local");
    }

    // Initialize 5x ToF sensors
    initToFSensors();

    // Start Web Server
    server.on("/", HTTP_GET, handleRoot);
    server.on("/api/telemetry", HTTP_GET, handleTelemetry);
    server.on("/api/reset_encoders", HTTP_POST, handleResetEncoders);
    server.on("/api/blink", HTTP_POST, handleBlink);
    server.begin();

    logMessage("[HTTP] Telemetry server listening on port 80");
    digitalWrite(LED_PIN, LOW);
}

void loop() {
    server.handleClient();

    // Reset button (BTN1) handling
    int btnVal = digitalRead(BTN1);
    if (btnVal != lastBtnState) {
        lastDebounceTime = millis();
    }
    if ((millis() - lastDebounceTime) > debounceDelay) {
        if (btnVal == LOW && lastBtnState == HIGH) {
            noInterrupts();
            leftTicks  = 0;
            rightTicks = 0;
            interrupts();
            logMessage(">>> [BTN1] Hardware Reset: Encoders zeroed <<<");
            digitalWrite(LED_PIN, HIGH);
            delay(80);
            digitalWrite(LED_PIN, LOW);
        }
    }
    lastBtnState = btnVal;

    // Periodic sensor read and telemetry broadcast (~10Hz)
    static unsigned long lastUpdate = 0;
    if (millis() - lastUpdate >= 100) {
        lastUpdate = millis();

        if (okL)  distL  = sensorL.readRangeContinuousMillimeters();
        if (okLF) distLF = sensorLF.readRangeContinuousMillimeters();
        if (okF)  distF  = sensorF.readRangeContinuousMillimeters();
        if (okRF) distRF = sensorRF.readRangeContinuousMillimeters();
        if (okR)  distR  = sensorR.readRangeContinuousMillimeters();

        noInterrupts();
        int64_t curL = leftTicks;
        int64_t curR = rightTicks;
        interrupts();

        char buf[128];
        snprintf(buf, sizeof(buf), "L:%4d LF:%4d F:%4d RF:%4d R:%4d | Enc L:%5ld R:%5ld",
                 distL, distLF, distF, distRF, distR, (long)curL, (long)curR);
        logMessage(String(buf));
    }

    // Automatic Wi-Fi reconnection if hotspot signal drops
    static unsigned long lastWifiCheck = 0;
    if (millis() - lastWifiCheck >= 3000) {
        lastWifiCheck = millis();
        if (WiFi.status() != WL_CONNECTED) {
            WiFi.reconnect();
        }
    }
}
