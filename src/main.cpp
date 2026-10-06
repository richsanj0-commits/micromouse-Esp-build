#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>

// ==========================================
// PIN DEFINITIONS
// ==========================================
#define PWMA 27
#define AIN1 14
#define AIN2 26
#define PWMB 32
#define BIN1 25
#define BIN2 33

#define L_ENC_A 36
#define L_ENC_B 39
#define R_ENC_A 34
#define R_ENC_B 35

#define I2C_SDA 21
#define I2C_SCL 22

#define XSHUT_L  23
#define XSHUT_LF 19
#define XSHUT_F  18
#define XSHUT_RF 17
#define XSHUT_R  16

#define BTN1 4
#define BTN2 13

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

void initSensors() {
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000); // 400kHz I2C speed

    // 1. Reset all sensors by setting XSHUT LOW
    pinMode(XSHUT_L, OUTPUT);
    pinMode(XSHUT_LF, OUTPUT);
    pinMode(XSHUT_F, OUTPUT);
    pinMode(XSHUT_RF, OUTPUT);
    pinMode(XSHUT_R, OUTPUT);

    digitalWrite(XSHUT_L, LOW);
    digitalWrite(XSHUT_LF, LOW);
    digitalWrite(XSHUT_F, LOW);
    digitalWrite(XSHUT_RF, LOW);
    digitalWrite(XSHUT_R, LOW);
    delay(20);

    // 2. Initialize Left Sensor
    digitalWrite(XSHUT_L, HIGH);
    delay(10);
    sensorL.setTimeout(500);
    if (!sensorL.init()) Serial.println("Failed to init Left ToF!");
    sensorL.setAddress(ADDR_L);
    sensorL.startContinuous(50);

    // 3. Initialize Left-Front Sensor
    digitalWrite(XSHUT_LF, HIGH);
    delay(10);
    sensorLF.setTimeout(500);
    if (!sensorLF.init()) Serial.println("Failed to init Left-Front ToF!");
    sensorLF.setAddress(ADDR_LF);
    sensorLF.startContinuous(50);

    // 4. Initialize Front Sensor
    digitalWrite(XSHUT_F, HIGH);
    delay(10);
    sensorF.setTimeout(500);
    if (!sensorF.init()) Serial.println("Failed to init Front ToF!");
    sensorF.setAddress(ADDR_F);
    sensorF.startContinuous(50);

    // 5. Initialize Right-Front Sensor
    digitalWrite(XSHUT_RF, HIGH);
    delay(10);
    sensorRF.setTimeout(500);
    if (!sensorRF.init()) Serial.println("Failed to init Right-Front ToF!");
    sensorRF.setAddress(ADDR_RF);
    sensorRF.startContinuous(50);

    // 6. Initialize Right Sensor
    digitalWrite(XSHUT_R, HIGH);
    delay(10);
    sensorR.setTimeout(500);
    if (!sensorR.init()) Serial.println("Failed to init Right ToF!");
    sensorR.setAddress(ADDR_R);
    sensorR.startContinuous(50);

    Serial.println("All 5 ToF sensors initialized successfully!");
}

void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println("\n--- 5-ToF Sensor Monitor Starting ---");
    initSensors();
}

void loop() {
    uint16_t dL  = sensorL.readRangeContinuousMillimeters();
    uint16_t dLF = sensorLF.readRangeContinuousMillimeters();
    uint16_t dF  = sensorF.readRangeContinuousMillimeters();
    uint16_t dRF = sensorRF.readRangeContinuousMillimeters();
    uint16_t dR  = sensorR.readRangeContinuousMillimeters();

    // Formatted readable output (in mm)
    Serial.printf("L: %4d mm | LF: %4d mm | F: %4d mm | RF: %4d mm | R: %4d mm\n",
                  dL, dLF, dF, dRF, dR);

    delay(100);
}
