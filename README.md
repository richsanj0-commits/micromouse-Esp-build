# ESP32 38-Pin Micromouse Maze Solver

Simple single-file project setup for ESP32 (38-pin module).

- **GitHub Repository**: [richsanj0-commits/micromouse-Esp-build](https://github.com/richsanj0-commits/micromouse-Esp-build)
- **Active Code**: [`src/main.cpp`](src/main.cpp)
- **Wireless Web Telemetry**: [`apexweb.cpp`](apexweb.cpp)

---

## 📁 Project Structure

```
D:\MazeSolver\
├── apexweb.cpp              # Complete standalone wireless web telemetry setup (alienware hotspot)
├── src/
│   └── main.cpp             # Active PlatformIO build source
├── include/
│   └── dashboard_html.h     # High-tech dark mode responsive telemetry web dashboard
├── platformio.ini           # ESP32 board configuration
└── README.md
```

---

## ⚡ How to Run in VS Code

1. Open **`D:\MazeSolver`** in VS Code.
2. Click **Build** (`✓`) or **Upload** (`→`) on the PlatformIO bottom toolbar.
3. Open **Serial Monitor** at **115200 baud** to see status.

---

## 🔌 Pin Definitions in `src/main.cpp`

- **Motors**: Left (`PWMA=27`, `AIN1=14`, `AIN2=26`), Right (`PWMB=32`, `BIN1=25`, `BIN2=33`)
- **Encoders**: Left (`36`, `39`), Right (`34`, `35`) + 4.7k pull-ups to 3.3V
- **I2C Bus**: `SDA=21`, `SCL=22` (5x ToF + MPU6050)
- **ToF XSHUT**: `23`, `19`, `18`, `17`, `16`
- **Buttons**: `BTN1=4`, `BTN2=13`
