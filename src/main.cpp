#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// ==========================================
// PIN DEFINITIONS
// ==========================================
#define I2C_SDA 21
#define I2C_SCL 22

#define XSHUT_L  23
#define XSHUT_LF 19
#define XSHUT_F  18
#define XSHUT_RF 17
#define XSHUT_R  16

// Sensor I2C Addresses
#define ADDR_L   0x30
#define ADDR_LF  0x31
#define ADDR_F   0x32
#define ADDR_RF  0x33
#define ADDR_R   0x34

// Sensor objects
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

void initSensors() {
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(100000); // 100kHz standard speed for stable lines

    // Configure all XSHUT as open-drain (high-z when releasing, pull low to reset)
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
    delay(50);

    // 1. Left Sensor (release reset with HIGH / floating input)
    digitalWrite(XSHUT_L, HIGH);
    delay(30);
    sensorL.setTimeout(500);
    if (sensorL.init()) {
        sensorL.setAddress(ADDR_L);
        sensorL.startContinuous(50);
        okL = true;
        Serial.println("Left ToF: OK");
    } else {
        Serial.println("Left ToF: FAILED");
    }
    delay(30);

    // 2. Left-Front Sensor
    digitalWrite(XSHUT_LF, HIGH);
    delay(30);
    sensorLF.setTimeout(500);
    if (sensorLF.init()) {
        sensorLF.setAddress(ADDR_LF);
        sensorLF.startContinuous(50);
        okLF = true;
        Serial.println("Left-Front ToF: OK");
    } else {
        Serial.println("Left-Front ToF: FAILED");
    }
    delay(30);

    // 3. Front Sensor
    digitalWrite(XSHUT_F, HIGH);
    delay(30);
    sensorF.setTimeout(500);
    if (sensorF.init()) {
        sensorF.setAddress(ADDR_F);
        sensorF.startContinuous(50);
        okF = true;
        Serial.println("Front ToF: OK");
    } else {
        Serial.println("Front ToF: FAILED");
    }
    delay(30);

    // 4. Right-Front Sensor
    digitalWrite(XSHUT_RF, HIGH);
    delay(30);
    sensorRF.setTimeout(500);
    if (sensorRF.init()) {
        sensorRF.setAddress(ADDR_RF);
        sensorRF.startContinuous(50);
        okRF = true;
        Serial.println("Right-Front ToF: OK");
    } else {
        Serial.println("Right-Front ToF: FAILED");
    }
    delay(30);

    // 5. Right Sensor
    digitalWrite(XSHUT_R, HIGH);
    delay(30);
    sensorR.setTimeout(500);
    if (sensorR.init()) {
        sensorR.setAddress(ADDR_R);
        sensorR.startContinuous(50);
        okR = true;
        Serial.println("Right ToF: OK");
    } else {
        Serial.println("Right ToF: FAILED");
    }
    delay(30);
}

void setup() {
    // Disable ESP32 brownout detector for USB development/power dips
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

    Serial.begin(115200);
    delay(1000);

    Serial.println("\n--- 5-ToF Sensor Initialization ---");
    initSensors();
}

void loop() {
    uint16_t dL  = okL  ? sensorL.readRangeContinuousMillimeters()  : 9999;
    uint16_t dLF = okLF ? sensorLF.readRangeContinuousMillimeters() : 9999;
    uint16_t dF  = okF  ? sensorF.readRangeContinuousMillimeters()  : 9999;
    uint16_t dRF = okRF ? sensorRF.readRangeContinuousMillimeters() : 9999;
    uint16_t dR  = okR  ? sensorR.readRangeContinuousMillimeters()  : 9999;

    Serial.printf("L: %4d mm | LF: %4d mm | F: %4d mm | RF: %4d mm | R: %4d mm\n",
                  dL, dLF, dF, dRF, dR);

    delay(100);
}
