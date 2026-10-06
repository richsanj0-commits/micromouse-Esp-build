# ESP32 38-Pin Micromouse Maze Solver

High-performance autonomous micromouse maze-solving robot powered by the ESP32 (38-pin DevKit module).

- **GitHub Repository**: [richsanj0-commits/micromouse-Esp-build](https://github.com/richsanj0-commits/micromouse-Esp-build)
- **Framework**: PlatformIO + Arduino Core (ESP32)

---

## 📁 Project Structure

```
D:\MazeSolver\
├── .vscode/                 # VS Code workspace settings & IntelliSense
│   ├── c_cpp_properties.json
│   ├── extensions.json
│   └── settings.json
├── docs/                    # Hardware documentation
│   └── PINOUT.md            # Complete connection diagram & safety checklist
├── include/                 # Header files
│   ├── pinout.h             # All GPIO pin definitions & addresses
│   └── motor.h              # Motor driver functions (TB6612)
├── src/                     # Source files
│   ├── main.cpp             # Main program & motor forward/backward test
│   └── motor.cpp            # 20kHz PWM & motor directional control
├── platformio.ini           # PlatformIO board & compiler configuration
└── README.md
```

---

## ⚡ Quick Start with VS Code

1. Open **VS Code**.
2. Select **File > Open Folder...** and choose `D:\MazeSolver`.
3. The **PlatformIO IDE** extension will automatically detect the project and activate the toolbar at the bottom:
   - **Build**: Click the checkmark icon (`✓`) in the status bar (or press `Ctrl+Alt+B`).
   - **Upload**: Connect your ESP32 via micro-USB and click the arrow icon (`→`) (or press `Ctrl+Alt+U`).
   - **Serial Monitor**: Click the plug icon (`🔌`) (or press `Ctrl+Alt+S`) to view real-time diagnostics at **115200 baud**.

---

## 🎮 Motor Forward & Backward Test

The initial firmware includes an interactive motor validation routine with soft dynamic braking.

### 1. Hardware Buttons
- **Button 1 (GPIO4)**: Starts the Automated Test sequence:
  - 1-second arming delay (fast LED flash on GPIO2)
  - Moves **FORWARD** for 1.5 seconds at speed 150/255
  - Dynamic **BRAKE** for 0.5 seconds
  - Moves **BACKWARD** for 1.5 seconds at speed 150/255
  - Dynamic **BRAKE** and stops safely.
- **Button 2 (GPIO13)**: **Emergency Stop** (instantly brakes and idles the motors).

### 2. Interactive Serial Monitor Commands (115200 Baud)
Open the Serial Monitor and send any of the following characters:
- `t` : Run the automated Forward / Backward test cycle
- `f` : Drive continuously **FORWARD**
- `b` : Drive continuously **BACKWARD**
- `l` : Turn in place **LEFT**
- `r` : Turn in place **RIGHT**
- `s` or `[Space]` : Emergency **BRAKE & STOP**
- `+` / `-` : Increase or decrease test speed in steps of 15

---

## 🔌 Hardware Pinout Summary

| Peripheral | Signal | ESP32 GPIO | Description |
|:---|:---:|:---:|:---|
| **TB6612 Left Motor** | PWMA / AIN1 / AIN2 | `27` / `14` / `26` | Speed (20kHz PWM) & Direction |
| **TB6612 Right Motor**| PWMB / BIN1 / BIN2 | `32` / `25` / `33` | Speed (20kHz PWM) & Direction |
| **Encoders** | Left C1, C2 / Right C1, C2 | `36`, `39` / `34`, `35` | Requires 4.7kΩ pull-up to 3.3V |
| **I2C Bus** | SDA / SCL | `21` / `22` | Shared by 5x ToF + MPU6050 |
| **ToF XSHUT** | L / LF / F / RF / R | `23` / `19` / `18` / `17` / `16` | Dynamic sensor addressing |
| **Pushbuttons** | BTN1 / BTN2 | `4` / `13` | Active LOW (`INPUT_PULLUP`) |
| **Status LED** | LED | `2` | Built-in Blue LED |

> Complete wiring schematics and pre-power safety checks are located in [docs/PINOUT.md](docs/PINOUT.md).

---

## 🔄 Git Synchronization

The repository is linked to `https://github.com/richsanj0-commits/micromouse-Esp-build`.
Whenever you want to commit and push changes, ask to update git and it will automatically commit and push your latest code.
