#pragma once

#include <Arduino.h>

// =============================================================================
// MICROMOUSE ESP32 38-PIN BOARD PINOUT DEFINITION
// =============================================================================

// -----------------------------------------------------------------------------
// TB6612FNG MOTOR DRIVER PINS
// -----------------------------------------------------------------------------
// Left Motor (Motor A)
#define PWMA 27         // Left Motor PWM Speed Control (ESP32 Pin 11)
#define AIN1 14         // Left Motor Direction 1 (ESP32 Pin 12)
#define AIN2 26         // Left Motor Direction 2 (ESP32 Pin 10)

// Right Motor (Motor B)
#define PWMB 32         // Right Motor PWM Speed Control (ESP32 Pin 7)
#define BIN1 25         // Right Motor Direction 1 (ESP32 Pin 9)
#define BIN2 33         // Right Motor Direction 2 (ESP32 Pin 8)

// -----------------------------------------------------------------------------
// QUADRATURE ENCODERS (With 4.7k Pull-ups to 3.3V)
// -----------------------------------------------------------------------------
// Left Encoder
#define L_ENC_A 36      // Left Encoder Channel 1 (ESP32 Pin 3 - Sensor_VP)
#define L_ENC_B 39      // Left Encoder Channel 2 (ESP32 Pin 4 - Sensor_VN)

// Right Encoder
#define R_ENC_A 34      // Right Encoder Channel 1 (ESP32 Pin 5)
#define R_ENC_B 35      // Right Encoder Channel 2 (ESP32 Pin 6)

// -----------------------------------------------------------------------------
// I2C BUS (Shared across all 5 ToF Sensors & MPU6050)
// -----------------------------------------------------------------------------
#define I2C_SDA 21      // I2C Data (ESP32 Pin 33)
#define I2C_SCL 22      // I2C Clock (ESP32 Pin 36)

// -----------------------------------------------------------------------------
// TIME-OF-FLIGHT (ToF) XSHUT PINS & I2C ADDRESSES
// -----------------------------------------------------------------------------
#define XSHUT_L   23    // Left ToF XSHUT (ESP32 Pin 37)
#define XSHUT_LF  19    // Left-Front ToF XSHUT (ESP32 Pin 31)
#define XSHUT_F   18    // Front ToF XSHUT (ESP32 Pin 30)
#define XSHUT_RF  17    // Right-Front ToF XSHUT (ESP32 Pin 28)
#define XSHUT_R   16    // Right ToF XSHUT (ESP32 Pin 27)

// Dynamic I2C Addresses assigned sequentially after XSHUT activation
#define TOF_ADDR_LEFT        0x30
#define TOF_ADDR_LEFT_FRONT  0x31
#define TOF_ADDR_FRONT       0x32
#define TOF_ADDR_RIGHT_FRONT 0x33
#define TOF_ADDR_RIGHT       0x34

// -----------------------------------------------------------------------------
// IMU (MPU6050)
// -----------------------------------------------------------------------------
#define MPU6050_ADDR 0x68  // AD0 pulled to GND

// -----------------------------------------------------------------------------
// USER BUTTONS (Active LOW - Internal INPUT_PULLUP)
// -----------------------------------------------------------------------------
#define BTN1 4          // Button 1 (ESP32 Pin 26) -> GND
#define BTN2 13         // Button 2 (ESP32 Pin 15) -> GND

// -----------------------------------------------------------------------------
// ONBOARD STATUS LED
// -----------------------------------------------------------------------------
#define LED_PIN 2       // ESP32 Onboard LED (ESP32 Pin 24)
