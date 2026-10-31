# PCB guide - micromouse v2 board

What the custom PCB should fix and include, based on everything tested on the
prototype (perfboard + ESP32 DevKit + TB6612 + 2x N20 + 5x VL53L0X + MPU6050).
Measured values come from `CLAUDE.md` and `include/robot_config.h`.

## 1. Shape (most important)
- Maze cell 180 mm, 16 mm walls -> 164 mm corridor -> **82 mm from cell centre to a wall**.
- Prototype: 84 x 100 mm, axle far back -> front corners ~87 mm from the axle ->
  corners catch walls on in-place turns ("stuck in turn" in the logs).
- New board: **axle at the middle of the robot**, **rounded front** (arc around the axle).
  Keep every point within **65-70 mm of the axle** -> turns can never touch a wall.
- Use the **PCB as the chassis**: motors mounted under the board, battery low and
  centred over the axle (lower centre of gravity, less wheel slip).
- Track width now 75.3 mm, wheel D 34 mm. If these change, update `robot_config.h`.

## 2. MCU
- **ESP32-WROOM-32E** (NOT WROVER: GPIO 16/17 are used for ToF XSHUT, WROVER uses them for PSRAM).
- Simple v1: ESP32 DevKit (38-pin) in female headers - same as today, easy to replace.
- v2: bare module + USB-C + CH340C or CP2102N + auto-reset (2 transistors) + EN/BOOT buttons.

## 3. Pin map (keep = no code change)
| Function | GPIO | Notes |
|---|---|---|
| Motor L PWMA / AIN1 / AIN2 | 27 / 14 / 26 | forward = AIN1 LOW, AIN2 HIGH |
| Motor R PWMB / BIN1 / BIN2 | 32 / 25 / 33 | forward = BIN1 HIGH, BIN2 LOW (see change below) |
| Encoder L A / B | 34 / 35 | input-only pins: **add 10 k pull-ups to 3.3 V** |
| Encoder R A / B | 36 / 39 | input-only pins: **add 10 k pull-ups to 3.3 V** |
| ToF XSHUT L, LF, F, RF, R | 23, 19, 18, 17, 16 | addresses set to 0x30-0x34 at boot |
| I2C SDA / SCL | 21 / 22 | 5x VL53L0X + MPU6050 (0x68) |
| BTN1 (explore) / BTN2 (fast run, stop) | 13 / 4 | to GND, internal pull-up |

Suggested changes (one-line code edits):
- **Battery voltage sense**: all ADC1 pins (32-39) are in use and ADC2 doesn't work with WiFi.
  Move **PWMB 32 -> GPIO 5**, put the battery divider on **GPIO 32**.
  Divider e.g. 100 k / 22 k for 3S (12.6 V -> 2.27 V), 100 k / 47 k for 2S.
- **Status LED on GPIO 2** (boot pin - an LED to GND through a resistor is fine).
- Avoid GPIO 12 (must be low at boot), 0, 15 for anything that pulls them at reset.

## 4. Power
- Battery -> **reverse-polarity P-MOSFET** -> **power switch** -> motors (TB6612 VM).
- **Buck converter to 3.3 V, >= 1 A** for ESP32 (WiFi peaks ~0.5 A) + 5 ToF + MPU.
  No AMS1117 straight from the battery (too hot).
- TB6612: **100-220 uF + 100 nF at VM**, 100 nF at VCC, wide motor traces.
- **Motor ground and logic ground joined at one point** (near the battery connector).
- Battery connector: XT30 (smaller than the XT60 used now).
- Battery type (2S or 3S) to be decided: motors are 12 V N20 300 rpm.

## 5. Sensors
- **VL53L0X** at fixed angles on the board edge, all at the same height:
  L and R at 90 deg, LF and RF at ~45 deg, F straight ahead.
  Side sensors near the axle line. Today L/R read ~49 mm to the walls when centred.
- **I2C pull-ups 2.2-4.7 k** on the board (one bus, 6 devices).
- **MPU6050 flat, at the centre between the wheels**, away from motors (header for GY-521 module).
  Turns use the gyro Z axis - it must be level.

## 6. Connectors and extras
- Motor + encoder: 6-pin JST (M1, M2, GND, VCC 3.3 V, A, B) per motor.
- BTN1 / BTN2 on the board edge, easy to reach (code waits 1 s after the press).
- Reset + BOOT buttons, test points (3.3 V, VBAT, GND, SDA, SCL), optional buzzer.

## 7. After the board is built - check list
1. Encoder counts per wheel turn still 2772 (`examples/04`).
2. Motor and encoder directions (forward = positive counts).
3. ToF all 5 found, side readings when centred (`w` command).
4. Gyro axis = Z, 4 x 90 deg turns end where they started.
5. Update `robot_config.h` with any new geometry.
