# Micromouse (wall maze solver) - ESP32

PlatformIO, `esp32dev`, Arduino core 2.x. GitHub: richsanj0-commits/micromouse-Esp-build
User prefers small practical steps and short explanations.

## Workflow
- `src/main.cpp` = active sketch (swapped per test). Saved tests in `examples/`:
  01 motor, 02 ToF (old), 03 encoder (old), 04 encoder CPR calibration, 05 ToF test, 06 drive test.
- Shared values: `include/robot_config.h`. Web telemetry build: `apexweb.cpp` (encoder pins there are old).
- WiFi credentials: `include/wifi_secrets.h` (gitignored; template `wifi_secrets.example.h`).
- Upload over USB (COM9 on this PC), then unplug and monitor over WiFi:
  robot joins the "alienware" hotspot, IP 192.168.137.209, port 23
  (`pio device monitor --port socket://192.168.137.209:23`).

## Hardware (all verified on the robot)
- TB6612 + 2x N20 12V 600 RPM with magnetic encoders. Buttons BTN1=13, BTN2=4.
- Motors: L PWMA=27 AIN1=14 AIN2=26, forward = AIN1 LOW/AIN2 HIGH
          R PWMB=32 BIN1=25 BIN2=33, forward = BIN1 HIGH/BIN2 LOW
- Encoders (4x decoding): L A/B=34/35 dir +1, R A/B=36/39 dir -1 -> forward = positive.
- **2772 counts per wheel revolution** (both wheels).
- 5x VL53L0X (L, LF, F, RF, R): XSHUT 23,19,18,17,16, addr 0x30-0x34; MPU6050 0x68. All working.
  Raw ToF reads long (F: +37 mm @ 50, +43 mm @ 100); thresholds to come from start-cell
  calibration in the maze code, no bench calibration.

## Geometry (cad/micromouse_assembly.step)
- Wheel D 34 mm (CAD), track width 75.3 mm, robot 84 x 100 mm, maze cell 180 mm.
- 0.0385 mm/count, ~4672 counts per cell, ~1535 counts per wheel for a 90 deg turn.
- L/R sensors read ~49 mm to the walls when centred in a cell.

## Status (2026-10-27)
- Done: encoders, ToF check, motor directions, drive test (BTN1 fwd / BTN2 back 2000 counts).
- Next: check real distance vs encoders (wheel size), then 90 deg turns, then 1-cell moves.
