# Micromouse (wall maze solver) - ESP32

PlatformIO, `esp32dev`, Arduino core 2.x. GitHub: richsanj0-commits/micromouse-Esp-build
User prefers small practical steps and short explanations.

## Workflow
- `src/main.cpp` = active sketch (swapped per test). Saved tests in `examples/`:
  01 motor, 02 ToF (old), 03 encoder (old), 04 encoder CPR calibration, 05 ToF test,
  06 drive test, 07 turn test, 08 motor characterization, 09 maze test,
  10 maze solver (older, NOT the good one), 11 **BEST maze solver (ran the user's 10x10 in the video
  7-8 times - always use this)**, 12 right-hand priority (failed: hit walls), 13 = 11 + button fix
  (= current main.cpp).
- Shared values: `include/robot_config.h`. Web telemetry build: `apexweb.cpp` (encoder pins there are old).
- WiFi credentials: `include/wifi_secrets.h` (gitignored; template `wifi_secrets.example.h`).
- Upload over USB (COM9 on this PC), then unplug and monitor over WiFi:
  robot joins the PC hotspot (now "Richas-lap", was "alienware"), IP 192.168.137.209, port 23
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
- 90 deg turn: TURN_SCALE 1.14 (tuned on the floor) -> ~1750 counts per wheel. Encoder-only turns
  are NOT repeatable on the maze base (wheel slip: real angle 79-95 deg for the same counts).
- **Turns use the gyro** (MPU6050, Z axis, +-500 dps): gyroBrakeDeg 4.7 -> 4 x 90 within +-3 deg.
  Gyro must be zeroed while the robot is still (done at boot and by `c`; moved robot -> bad drift).
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

## Current maze (2026-10-29)
- User's own 10x10 maze, layout unknown to us (user keeps it secret). Cell given as "16-17 cm"
  (not yet clear if corridor or pitch; code still uses 180 mm pitch). PC hotspot now "Richas-lap".

## Maze solver (examples/10, 2026-10-28) - works on the robot
- BTN1 / `g`: explore (depth-first, prefers left/front/right) and record a wall map until an open
  2x2 block (no inner walls, not the start) is found = goal, then back to start on the shortest known path.
- BTN2 / `fr`: fast run start -> goal, flood fill over explored cells only, straights joined into one move.
  BTN2 during a run = stop. `m` prints the map. Map is in RAM only (power off = explore again).
- Start for both runs: start cell, back against the wall; its facing = "N" on the map.
- Straight moves: gyro heading hold (`kh` 3) + side-wall nudge (`kw` 0.15 deg/mm, max 5 deg).
  Turns to absolute headings (targetYaw), so errors don't add up.
- Side centre ref = average of L and R at the start (robot placed off-centre used to hug a wall).
- Front wall ahead: stop at F raw 55 (creeps up to 60 mm extra) -> fixes distance from wheel slip.
- ToF readings cleared at the start of each move (stale F 53 from the last run used to block the start).
- Wheel slip on turns is large (wheels 100-230 deg for 90 deg) - gyro handles it.
- A hole/gap in the floor makes the wheels spin -> position lost. Keep the floor flat.

## Tests on the user's maze (2026-10-29)
- Pushed solver code (cfd5b5b) ran the user's maze correctly 2 times with BTN1/BTN2 - keep it.
- Seen problems: "stuck in turn" when a corner catches a wall (robot off-centre / placed touching a
  wall; only a few mm clearance on in-place turns). Hand in front -> "unexpected wall ahead" (correct).
- Goal = FIRST open 2x2 (all 4 cells visited, no inner walls, not the start). Other open spaces are
  skipped if found later or not fully visited. Idea: goal = maze centre (10x10 known, start in a corner).
- **BEST code = examples/11** (= solver + start facing a wall -> turn around + stuck turn -> back up
  20 mm and retry). The user uploaded it from main.cpp on 2026-10-29 and it ran perfectly (video).
  2026-10-31: I wrongly uploaded examples/10 -> robot misbehaved. Never swap the base code again.
- examples/13 (2026-10-31): BEST + buttons work right after power-on (WiFi in background, was
  blocking 8 s) + blue LED (GPIO 2) on when a press registers / during a run.

## Status (2026-10-28)
- Done: all hardware, gyro turns, wall following, explore + 2x2 goal + flood-fill fast run.
- Next ideas: faster fast run (higher speed on straights), save map to flash, smooth turns.
- Robot IP on the hotspot changes (seen .209, .238): scan 192.168.137.x port 23 if not found.
