# Micromouse ESP32 Advanced Build

This repository contains the PlatformIO setup and isolated component test codes for an ESP32-based Micromouse.

## Calibration Data
* **Wheel Diameter:** 43 mm
* **Wheel Circumference:** 135.088 mm
* **Ticks per Revolution:** 1393 ticks
* **Distance per tick:** 0.096976 mm/tick
* **Ticks per mm:** 10.3117 ticks/mm
* **Ticks per Cell (180mm):** ~1856 ticks

## ESP32 Pin Connections

### Left Side (Motors & Encoders)
| Component | Pin | Note |
| :--- | :--- | :--- |
| **TB6612 Motor Driver** |
| PWMA | `GPIO 27` | |
| AIN1 | `GPIO 14` | Left Motor (Logic Inverted in software) |
| AIN2 | `GPIO 26` | |
| PWMB | `GPIO 32` | |
| BIN1 | `GPIO 25` | Right Motor |
| BIN2 | `GPIO 33` | |
| **Encoders** | 
| Left Encoder Phase A | `GPIO 35` | Swapped A/B for positive forward counting |
| Left Encoder Phase B | `GPIO 34` | |
| Right Encoder Phase A | `GPIO 36` | Swapped L/R matching wheels |
| Right Encoder Phase B | `GPIO 39` | |
| **Buttons** |
| Button 2 | `GPIO 13` | (Active LOW, INPUT_PULLUP) |

### Right Side (5x VL53L0X ToF Sensors & MPU6050)
| Component | Pin | Note |
| :--- | :--- | :--- |
| **I2C Bus** |
| SDA | `GPIO 21` | Shared across all ToF + MPU |
| SCL | `GPIO 22` | Shared across all ToF + MPU |
| **ToF XSHUT Pins (for boot sequencing)** |
| Left (0x30) | `GPIO 23` | |
| Left-Front (0x31) | `GPIO 19` | |
| Front (0x32) | `GPIO 18` | |
| Right-Front (0x33)| `GPIO 17` | |
| Right (0x34) | `GPIO 16` | |
| **Buttons** |
| Button 1 | `GPIO 4` | (Active LOW, INPUT_PULLUP) |

## Test Codes
The `examples/` directory contains isolated test scripts:
1. `01_Motor_Test.cpp`: Tests both motors spinning forward and backward via button presses.
2. `02_Sensor_Test.cpp`: Initializes 5x VL53L0X sensors with custom I2C addresses via XSHUT pins and streams data.
3. `03_Encoder_Test.cpp`: Stripped down encoder-only live feed with a button to reset ticks to 0. (Currently active in `src/main.cpp`).
