# Micromouse (wall maze solver) - ESP32

PlatformIO, `esp32dev`, Arduino core 2.x. GitHub: richsanj0-commits/micromouse-Esp-build
User prefers small practical steps and short explanations.

## Workflow
- `src/main.cpp` = active sketch (swapped per test). Saved tests in `examples/`:
  01 motor, 02 ToF (old), 03 encoder (old), 04 encoder CPR calibration, 05 ToF test,
  06 drive test, 07 turn test, 08 motor characterization, 09 maze test (current main.cpp).
- Shared values: `include/robot_config.h`. Web telemetry build: `apexweb.cpp` (encoder pins there are old).
- WiFi credentials: `include/wifi_secrets.h` (gitignored; template `wifi_secrets.example.h`).
- Upload over USB (COM9 on this PC), then unplug and monitor over WiFi:
  robot joins the "alienware" hotspot, IP 192.168.137.209, port 23
  (`pio device monitor --port socket://192.168.137.209:23`).

## Hardware (all verified on the robot)
- TB6612 + 2x N20 12V 300 RPM (~1:100) with magnetic encoders. Buttons BTN1=13, BTN2=4.
- Motors: L PWMA=27 AIN1=14 AIN2=26, forward = AIN1 LOW/AIN2 HIGH
          R PWMB=32 BIN1=25 BIN2=33, forward = BIN1 HIGH/BIN2 LOW
- Encoders (4x decoding): L A/B=34/35 dir +1, R A/B=36/39 dir -1 -> forward = positive.
- **2772 counts per wheel revolution** (both wheels).
- 5x VL53L0X (L, LF, F, RF, R): XSHUT 23,19,18,17,16, addr 0x30-0x34; MPU6050 0x68. All working.
  Raw ToF reads long (F: +37 mm @ 50, +43 mm @ 100); thresholds to come from start-cell
  calibration in the maze code, no bench calibration.

## Geometry (cad/micromouse_assembly.step)
- Wheel D 34 mm (CAD), track width 75.3 mm, robot 84 x 100 mm, maze cell 180 mm.
- 0.0385 mm/count (distance checked on the floor), ~4672 counts per cell.
- 90 deg turn: TURN_SCALE 1.14 (tuned on the floor) -> ~1750 counts per wheel.
- L/R sensors read ~49 mm to the walls when centred in a cell.

## Motor characterization (wheels in the air, 2026-10-28, repeated with 2 s steps - same)
- Linear up to PWM ~150: ~60 counts/s per PWM, both motors. PWM ~= 4 + counts_per_s / 60.
- L/R matched within 0-2% for PWM 50-150 (R 2-11% faster below 40, 2-8% faster above 180).
- Same forward and backward. Max ~290-318 rpm at PWM 255 = the motors rated 300 rpm.
- In the air PWM 10 sometimes fails to start a wheel, 15+ always starts.
  On the floor PWM 45 is known to move the robot.

## Test maze (user's, 2026-10-28)
- 16 mm wooden walls on a 180 mm grid -> 164 mm corridors. 2 x 3 cells:
  top row 2-3-4, bottom row S | 6-5 (wall between S and 6, wall between 3 and 6).
- Robot centred: ~40 mm side gap, L/R ~47 mm true to the walls, 6 mm clearance on in-place turns,
  nose only ~7 mm from a front wall (F unreliable that close -> LF/RF decide front walls).
- Start placement: back against the back wall -> axle 58 mm behind cell centre (CAD).

## Status (2026-10-28)
- Done: encoders, ToF check, motor directions, straight drive distance, 90 deg turns,
  motor characterization.
- Ready, NOT yet run: examples/09 maze test (BTN1 = left-hand run S..6..S, BTN2 / x = stop).
  Wall thresholds (FRONT_WALL_RAW 120, DIAG_WALL_RAW 95, side ref + 60) are estimates -> check
  the raw values it logs in each cell and adjust.
