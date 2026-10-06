# Micromouse ESP32 (38-Pin) Hardware Connection Guide

This document contains the exact pinouts, connection maps, safety rules, and operational guidelines for the ESP32 38-Pin Micromouse.

---

## 1. ESP32 38-Pin Module Pinout Map

### Left Side (Motors, Encoders, Buttons)
| ESP32 Pin | GPIO | Function / Peripheral | Notes |
|:---:|:---:|:---|:---|
| Pin 1 | **3V3** | 3.3V Rail | Supplies encoder pull-ups & TB6612 VCC |
| Pin 2 | **EN** | Empty / Reset | Reset button circuit |
| Pin 3 | **GPIO36** (VP) | Left Encoder Channel 1 (C1) | **Requires 4.7k pull-up to 3.3V** (Input only) |
| Pin 4 | **GPIO39** (VN) | Left Encoder Channel 2 (C2) | **Requires 4.7k pull-up to 3.3V** (Input only) |
| Pin 5 | **GPIO34** | Right Encoder Channel 1 (C1) | **Requires 4.7k pull-up to 3.3V** (Input only) |
| Pin 6 | **GPIO35** | Right Encoder Channel 2 (C2) | **Requires 4.7k pull-up to 3.3V** (Input only) |
| Pin 7 | **GPIO32** | TB6612 PWMB | Right Motor Speed (PWM Channel 1, 20kHz) |
| Pin 8 | **GPIO33** | TB6612 BIN2 | Right Motor Direction 2 |
| Pin 9 | **GPIO25** | TB6612 BIN1 | Right Motor Direction 1 |
| Pin 10 | **GPIO26** | TB6612 AIN2 | Left Motor Direction 2 |
| Pin 11 | **GPIO27** | TB6612 PWMA | Left Motor Speed (PWM Channel 0, 20kHz) |
| Pin 12 | **GPIO14** | TB6612 AIN1 | Left Motor Direction 1 |
| Pin 13 | **GPIO12** | Empty | Strapping pin (leave unconnected) |
| Pin 14 | **GND** | System Ground | TB6612 & Encoders common ground |
| Pin 15 | **GPIO13** | User Button 2 (BTN2) | Pushbutton to GND (`INPUT_PULLUP`) |
| Pin 16 | **GPIO9** | **NEVER USE** | Connected to internal SPI Flash |
| Pin 17 | **GPIO10** | **NEVER USE** | Connected to internal SPI Flash |
| Pin 18 | **GPIO11** | **NEVER USE** | Connected to internal SPI Flash |
| Pin 19 | **5V** | 5V Power Input | Buck converter 5.0V output rail |

---

### Right Side (Sensors, I2C, IMU)
| ESP32 Pin | GPIO | Function / Peripheral | Notes |
|:---:|:---:|:---|:---|
| Pin 38 | **GND** | System Ground | ToF sensors & MPU6050 ground |
| Pin 37 | **GPIO23** | Left ToF XSHUT | Active HIGH enable, default address 0x29 -> changes to **0x30** |
| Pin 36 | **GPIO22** | I2C Clock (SCL) | Shared by all 5 ToF sensors + MPU6050 |
| Pin 35 | **GPIO1** | Empty | Hardware Serial TX (USB Communication) |
| Pin 34 | **GPIO3** | Empty | Hardware Serial RX (USB Communication) |
| Pin 33 | **GPIO21** | I2C Data (SDA) | Shared by all 5 ToF sensors + MPU6050 |
| Pin 32 | **GND** | Spare GND | Auxiliary ground |
| Pin 31 | **GPIO19** | Left-Front ToF XSHUT | Dynamic address -> **0x31** |
| Pin 30 | **GPIO18** | Front ToF XSHUT | Dynamic address -> **0x32** |
| Pin 29 | **GPIO5** | Empty | Available GPIO |
| Pin 28 | **GPIO17** | Right-Front ToF XSHUT | Dynamic address -> **0x33** |
| Pin 27 | **GPIO16** | Right ToF XSHUT | Dynamic address -> **0x34** |
| Pin 26 | **GPIO4** | User Button 1 (BTN1) | Pushbutton to GND (`INPUT_PULLUP`) |
| Pin 25 | **GPIO0** | Empty | Strapping / BOOT button |
| Pin 24 | **GPIO2** | Onboard LED | Built-in Blue LED |
| Pin 23 | **GPIO15** | Empty | Available GPIO |
| Pin 22 | **GPIO8** | **NEVER USE** | Connected to internal SPI Flash |
| Pin 21 | **GPIO7** | **NEVER USE** | Connected to internal SPI Flash |
| Pin 20 | **GPIO6** | **NEVER USE** | Connected to internal SPI Flash |

---

## 2. TB6612FNG Motor Driver Connections

```
           TB6612FNG Module
           ┌──────────────┐
  12.6V ───┤ VM       PWMA├─── GPIO27 (Left Speed)
   3.3V ───┤ VCC      AIN2├─── GPIO26 (Left Dir 2)
   3.3V ───┤ STBY     AIN1├─── GPIO14 (Left Dir 1)
    GND ───┤ GND       AO1├─── Left Motor M1
    GND ───┤ GND       AO2├─── Left Motor M2
           │           BO2├─── Right Motor M2
   GPIO32 ─┤ PWMB      BO1├─── Right Motor M1
   GPIO33 ─┤ BIN2     BIN1├─── GPIO25 (Right Dir 1)
           └──────────────┘
```

### Motor Driver Control Truth Table
| IN1 | IN2 | PWM | Motor State |
|:---:|:---:|:---:|:---|
| HIGH | LOW | Speed | Forward |
| LOW | HIGH | Speed | Backward |
| HIGH | HIGH | Any | **Short Brake** (Dynamic braking via low-side FETs) |
| LOW | LOW | Any | **Stop / Coast** (High impedance freewheel) |

*Note: `STBY` must be pulled to 3.3V (or 5V) to take the chip out of standby mode.*

---

## 3. Quadrature Encoders & Pull-Up Network

The input-only pins (GPIO34, 35, 36, 39) do **not** have internal pull-ups on the ESP32. External 4.7kΩ pull-up resistors to 3.3V are **mandatory**.

```
                ┌── 4.7kΩ ──→ GPIO36 (Left Encoder C1)
                ├── 4.7kΩ ──→ GPIO39 (Left Encoder C2)
3.3V (Pin 1) ───┼── 4.7kΩ ──→ GPIO34 (Right Encoder C1)
                └── 4.7kΩ ──→ GPIO35 (Right Encoder C2)
```

- **Left Encoder Power**: VCC → 5V, GND → GND
- **Right Encoder Power**: VCC → 5V, GND → GND

---

## 4. Time-of-Flight (ToF) Sensors & MPU6050

### Shared I2C Bus:
- **VIN / VCC**: 5V rail
- **GND**: Common GND
- **SDA**: GPIO21
- **SCL**: GPIO22

### Sensor Addressing Scheme:
| Device | XSHUT Pin | Target Address | Default Boot Address |
|:---|:---:|:---:|:---:|
| MPU6050 (IMU) | N/A (AD0 to GND) | `0x68` | `0x68` |
| Left ToF | GPIO23 | `0x30` | `0x29` |
| Left-Front ToF | GPIO19 | `0x31` | `0x29` |
| Front ToF | GPIO18 | `0x32` | `0x29` |
| Right-Front ToF | GPIO17 | `0x33` | `0x29` |
| Right ToF | GPIO16 | `0x34` | `0x29` |

---

## 5. Pre-Power Safety Checklist

> [!CAUTION]
> **DO NOT connect the ESP32 until the following voltage measurements are verified with a multimeter:**

1. **Buck Converter Output**:
   - Must measure **5.00V ± 0.1V**. Never exceed 5.25V.
2. **SDA & SCL Lines**:
   - Must measure **≤ 3.3V**. Under no circumstances should 5V appear on these pins.
3. **Encoder Signal Lines (C1/C2)**:
   - Turn each wheel slowly by hand while measuring voltage at GPIO36, 39, 34, and 35.
   - Signals must toggle cleanly between **0.0V and ~3.3V** (never 5.0V).
4. **Common Ground**:
   - Verify 0Ω continuity between Battery Ground, Buck Output Ground, TB6612 Ground, and ESP32 Ground.
