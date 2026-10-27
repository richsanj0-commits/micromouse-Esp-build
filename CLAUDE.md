# Micromouse (wall maze solver) - ESP32

PlatformIO project (`esp32dev`, Arduino core 2.x -> `ledcSetup`/`ledcAttachPin` API).
GitHub: https://github.com/richsanj0-commits/micromouse-Esp-build

## Workflow
- `src/main.cpp` is the single active sketch; it gets swapped between test programs.
  On GitHub it is currently the WiFi web telemetry build.
- Standalone test programs live in `examples/` (01 motor, 02 ToF sensors, 03 encoder).
  Before overwriting `src/main.cpp`, make sure its content is saved in `examples/`.
- Build: `pio run` | Upload: `pio run -t upload` | Monitor: 115200 baud.

## Hardware
- ESP32 38-pin, TB6612 driver, 2x N20 12V 600 RPM gear motors with magnetic
  quadrature encoders (measured ~2772 counts/rev at 4x decoding).
- Motors: Left PWMA=27 AIN1=14 AIN2=26 | Right PWMB=32 BIN1=25 BIN2=33
  (forward: left AIN1=LOW/AIN2=HIGH, right BIN1=HIGH/BIN2=LOW).
- Encoders (verified on robot 2026-10-27): L A/B=34/35, R A/B=36/39, both counts
  inverted (DIR = -1) so forward = positive. Input-only pins, external 4.7k pull-ups.
  The repo's web build (`apexweb.cpp`) still has L/R swapped (L=36/39) - fix when merging.
- I2C SDA=21 SCL=22: 5x VL53L0X ToF (XSHUT 23,19,18,17,16; addr 0x30-0x34) + MPU6050.
- Buttons: BTN1=13, BTN2=4 (repo web build uses BTN1=4).
- `apexweb.cpp` + `include/dashboard_html.h`: WiFi web telemetry dashboard.

## Calibration results
- Encoder counts per wheel rev (4x decoding): **L 2772.1, R 2772.3** (10 hand turns, 2026-10-27).
  **FINAL - verified: motor-driven `v` 1-rev spin stops on the tape mark.** 2x (A-edges only) code would see ~1386. Constants live in `include/robot_config.h`; calibration sketch saved as `examples/04_Encoder_CPR_Calibration.cpp`.

## Geometry (from `cad/micromouse_assembly.step`, Fusion export 2026-10-27)
CAD axes: X = right, Y = forward, Z = up. Pivot (axle midpoint) = X 26.35, Y 29.0.
Below: x = right, y = forward, relative to the pivot, in mm.
- Wheels: "Roda 34 mm" -> **D = 34 mm** (CAD value, verify on real tyre), 8.6 wide.
- **Track width (wheel centre to centre) = 75.3 mm**. Overall width 83.9 mm.
- Chassis plate 80.9 x 98.9 mm; axle 24 mm from rear edge, ~75 mm behind nose.
- Max radius from pivot (in-place turn sweep) = 76.0 mm -> 8 mm clearance in a
  168 mm corridor (180 cell - 12 wall) when the pivot is at the cell centre.
- ToF sensors (VL53L0X, ~32 mm above ground):
  - F  : x  -0.8, y +75  facing forward
  - LF : x -20.6, y +65  diagonal ~45 deg left
  - RF : x +20.5, y +65  diagonal ~45 deg right
  - L  : x -34.6, y +42  facing left  -> reads ~49 mm to a wall with the mouse centred in the cell
  - R  : x +34.5, y +43  facing right -> reads ~49 mm to a wall with the mouse centred in the cell
- Ground clearance under plate 8.5 mm. No caster/skid modelled; axle is far back.
- Top plate not modelled (4 pillars 50 mm tall).

## Derived motion constants (CPR 2772, D 34, track 75.3)
- mm per count = pi*34/2772 = 0.03853
- counts per 180 mm cell = 4672
- counts per wheel for a 90 deg in-place turn = (pi*75.3/4)/0.03853 = 1535
