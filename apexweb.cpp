// =========================================================================
// APEXWEB: ESP32 Micromouse Wireless Web Telemetry Setup
// Connects directly to Hotspot 'alienware' (Password: '12345678')
// Hosts real-time 5-ToF Proximity Radar, Quadrature Encoders, & Serial Console
// Accessible at: http://micromouse.local or your hotspot assigned IP
// =========================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Wire.h>
#include <VL53L0X.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// =========================================================================
// EMBEDDED WEB DASHBOARD HTML / CSS / JS (SELF-CONTAINED)
// =========================================================================
const char DASHBOARD_HTML[] PROGMEM = R"rawliteral(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
<title>Micromouse Telemetry</title>
<style>
  :root {
    --bg: #090d16;
    --card-bg: rgba(18, 26, 43, 0.85);
    --card-border: rgba(255, 255, 255, 0.08);
    --text-primary: #f1f5f9;
    --text-muted: #94a3b8;
    --accent-cyan: #06b6d4;
    --accent-blue: #3b82f6;
    --safe: #10b981;
    --warning: #f59e0b;
    --danger: #ef4444;
    --offline: #64748b;
    --radius: 12px;
  }
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body {
    background-color: var(--bg);
    background-image: radial-gradient(circle at 50% 0%, #172554 0%, transparent 60%);
    color: var(--text-primary);
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
    padding: 12px;
    max-width: 1000px;
    margin: 0 auto;
    min-height: 100vh;
  }
  header {
    display: flex;
    justify-content: space-between;
    align-items: center;
    padding: 10px 14px;
    background: var(--card-bg);
    border: 1px solid var(--card-border);
    border-radius: var(--radius);
    margin-bottom: 12px;
    backdrop-filter: blur(10px);
  }
  .title-group { display: flex; align-items: center; gap: 10px; }
  .logo-icon {
    width: 28px; height: 28px;
    background: linear-gradient(135deg, var(--accent-cyan), var(--accent-blue));
    border-radius: 8px;
    display: grid; place-items: center;
    font-weight: 800; font-size: 14px; color: #fff;
  }
  h1 { font-size: 16px; font-weight: 700; letter-spacing: 0.5px; }
  .subtitle { font-size: 11px; color: var(--text-muted); }
  .stats-bar { display: flex; gap: 8px; align-items: center; font-size: 12px; }
  .badge {
    padding: 4px 8px;
    border-radius: 20px;
    font-size: 11px;
    font-weight: 600;
    display: inline-flex;
    align-items: center;
    gap: 5px;
  }
  .badge-live { background: rgba(16, 185, 129, 0.15); color: var(--safe); border: 1px solid rgba(16, 185, 129, 0.3); }
  .badge-live::before { content: ""; width: 6px; height: 6px; background: var(--safe); border-radius: 50%; box-shadow: 0 0 8px var(--safe); animation: pulse 1.5s infinite; }
  .badge-lost { background: rgba(239, 68, 68, 0.15); color: var(--danger); border: 1px solid rgba(239, 68, 68, 0.3); }
  .pill { background: rgba(255, 255, 255, 0.05); padding: 4px 8px; border-radius: 6px; color: var(--text-muted); }
  
  @keyframes pulse { 0%, 100% { opacity: 1; } 50% { opacity: 0.4; } }

  .section-title {
    font-size: 13px;
    text-transform: uppercase;
    letter-spacing: 1px;
    color: var(--text-muted);
    margin: 14px 2px 8px 2px;
    font-weight: 700;
  }

  /* SENSORS GRID */
  .sensors-grid {
    display: grid;
    grid-template-columns: repeat(5, 1fr);
    gap: 8px;
    margin-bottom: 12px;
  }
  @media (max-width: 768px) {
    .sensors-grid { grid-template-columns: repeat(3, 1fr); }
    .sensor-card:nth-child(4) { grid-column: span 1; }
    .sensor-card:nth-child(5) { grid-column: span 2; }
  }
  @media (max-width: 480px) {
    .sensors-grid { grid-template-columns: 1fr 1fr; }
    .sensor-card:nth-child(3) { grid-column: span 2; }
  }

  .sensor-card {
    background: var(--card-bg);
    border: 1px solid var(--card-border);
    border-radius: var(--radius);
    padding: 12px 10px;
    display: flex;
    flex-direction: column;
    justify-content: space-between;
    text-align: center;
    position: relative;
    overflow: hidden;
    transition: transform 0.15s ease, border-color 0.15s ease;
  }
  .sensor-card::before {
    content: "";
    position: absolute;
    top: 0; left: 0; right: 0; height: 3px;
    background: var(--accent-cyan);
    opacity: 0.6;
  }
  .sensor-card.danger::before { background: var(--danger); opacity: 1; }
  .sensor-card.warning::before { background: var(--warning); opacity: 1; }
  .sensor-card.safe::before { background: var(--safe); opacity: 1; }
  .sensor-card.offline::before { background: var(--offline); opacity: 0.5; }

  .sensor-name { font-size: 11px; font-weight: 700; color: var(--text-muted); letter-spacing: 0.5px; }
  .sensor-val {
    font-size: 26px;
    font-weight: 800;
    margin: 6px 0;
    font-variant-numeric: tabular-nums;
  }
  .val-unit { font-size: 11px; font-weight: 500; color: var(--text-muted); margin-left: 2px; }
  .sensor-bar-bg {
    width: 100%;
    height: 6px;
    background: rgba(255, 255, 255, 0.07);
    border-radius: 3px;
    overflow: hidden;
    margin-top: 4px;
  }
  .sensor-bar-fill {
    height: 100%;
    width: 0%;
    border-radius: 3px;
    transition: width 0.1s linear, background-color 0.2s ease;
  }
  .sensor-status { font-size: 10px; margin-top: 4px; font-weight: 600; }

  /* DUAL COLUMN SECTION */
  .telemetry-row {
    display: grid;
    grid-template-columns: 1fr 1.6fr;
    gap: 12px;
    margin-bottom: 12px;
  }
  @media (max-width: 768px) {
    .telemetry-row { grid-template-columns: 1fr; }
  }

  .panel-card {
    background: var(--card-bg);
    border: 1px solid var(--card-border);
    border-radius: var(--radius);
    padding: 14px;
    display: flex;
    flex-direction: column;
  }

  /* ENCODERS */
  .encoder-boxes {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 10px;
    margin-bottom: 12px;
  }
  .enc-box {
    background: rgba(0, 0, 0, 0.25);
    border: 1px solid var(--card-border);
    border-radius: 8px;
    padding: 10px;
    text-align: center;
  }
  .enc-label { font-size: 11px; color: var(--text-muted); font-weight: 600; text-transform: uppercase; }
  .enc-num { font-size: 22px; font-weight: 800; color: var(--accent-cyan); font-variant-numeric: tabular-nums; margin-top: 4px; }
  .btn-row { display: flex; gap: 8px; }
  .btn {
    flex: 1;
    background: rgba(255, 255, 255, 0.08);
    border: 1px solid rgba(255, 255, 255, 0.15);
    color: var(--text-primary);
    padding: 9px 12px;
    border-radius: 8px;
    font-size: 12px;
    font-weight: 600;
    cursor: pointer;
    transition: background 0.15s ease, border-color 0.15s ease;
    display: inline-flex;
    justify-content: center;
    align-items: center;
    gap: 6px;
  }
  .btn:hover { background: rgba(255, 255, 255, 0.15); border-color: rgba(255, 255, 255, 0.3); }
  .btn:active { transform: scale(0.98); }
  .btn-cyan { background: rgba(6, 182, 212, 0.2); border-color: rgba(6, 182, 212, 0.4); color: #67e8f9; }
  .btn-cyan:hover { background: rgba(6, 182, 212, 0.3); }

  /* TERMINAL */
  .terminal-header {
    display: flex;
    justify-content: space-between;
    align-items: center;
    margin-bottom: 8px;
  }
  .terminal-actions { display: flex; gap: 6px; }
  .term-btn {
    background: transparent;
    border: 1px solid var(--card-border);
    color: var(--text-muted);
    font-size: 11px;
    padding: 3px 8px;
    border-radius: 5px;
    cursor: pointer;
  }
  .term-btn:hover { color: var(--text-primary); background: rgba(255, 255, 255, 0.05); }
  .terminal-box {
    background: #050810;
    border: 1px solid rgba(255, 255, 255, 0.06);
    border-radius: 8px;
    height: 190px;
    padding: 10px;
    font-family: Consolas, "SF Mono", "Fira Code", monospace;
    font-size: 12px;
    color: #38bdf8;
    overflow-y: auto;
    display: flex;
    flex-direction: column;
    gap: 3px;
    white-space: pre-wrap;
    word-break: break-all;
  }
  .terminal-box::-webkit-scrollbar { width: 6px; }
  .terminal-box::-webkit-scrollbar-thumb { background: rgba(255, 255, 255, 0.15); border-radius: 3px; }
  .term-line { line-height: 1.4; color: #cbd5e1; }
  .term-line.highlight { color: #f59e0b; font-weight: bold; }
  .term-line.sys { color: #34d399; }
</style>
</head>
<body>

<header>
  <div class="title-group">
    <div class="logo-icon">M</div>
    <div>
      <h1>MICROMOUSE TELEMETRY</h1>
      <div class="subtitle" id="netSubtitle">Connecting...</div>
    </div>
  </div>
  <div class="stats-bar">
    <span id="ratePill" class="pill">0 Hz</span>
    <span id="uptimePill" class="pill">Up: 0s</span>
    <span id="statusBadge" class="badge badge-live">ONLINE</span>
  </div>
</header>

<div class="section-title">5-ToF Proximity Radar (mm)</div>
<div class="sensors-grid">
  <!-- Left -->
  <div class="sensor-card" id="cardL">
    <div class="sensor-name">LEFT (0x30)</div>
    <div class="sensor-val"><span id="valL">--</span><span class="val-unit">mm</span></div>
    <div class="sensor-bar-bg"><div class="sensor-bar-fill" id="barL"></div></div>
    <div class="sensor-status" id="statL">WAITING</div>
  </div>
  <!-- Left Front -->
  <div class="sensor-card" id="cardLF">
    <div class="sensor-name">L-FRONT (0x31)</div>
    <div class="sensor-val"><span id="valLF">--</span><span class="val-unit">mm</span></div>
    <div class="sensor-bar-bg"><div class="sensor-bar-fill" id="barLF"></div></div>
    <div class="sensor-status" id="statLF">WAITING</div>
  </div>
  <!-- Front -->
  <div class="sensor-card" id="cardF">
    <div class="sensor-name">FRONT (0x32)</div>
    <div class="sensor-val"><span id="valF">--</span><span class="val-unit">mm</span></div>
    <div class="sensor-bar-bg"><div class="sensor-bar-fill" id="barF"></div></div>
    <div class="sensor-status" id="statF">WAITING</div>
  </div>
  <!-- Right Front -->
  <div class="sensor-card" id="cardRF">
    <div class="sensor-name">R-FRONT (0x33)</div>
    <div class="sensor-val"><span id="valRF">--</span><span class="val-unit">mm</span></div>
    <div class="sensor-bar-bg"><div class="sensor-bar-fill" id="barRF"></div></div>
    <div class="sensor-status" id="statRF">WAITING</div>
  </div>
  <!-- Right -->
  <div class="sensor-card" id="cardR">
    <div class="sensor-name">RIGHT (0x34)</div>
    <div class="sensor-val"><span id="valR">--</span><span class="val-unit">mm</span></div>
    <div class="sensor-bar-bg"><div class="sensor-bar-fill" id="barR"></div></div>
    <div class="sensor-status" id="statR">WAITING</div>
  </div>
</div>

<div class="telemetry-row">
  <!-- Encoders & Robot Controls -->
  <div class="panel-card">
    <div class="section-title" style="margin-top:0;">Encoders & Controls</div>
    <div class="encoder-boxes">
      <div class="enc-box">
        <div class="enc-label">Left Wheel</div>
        <div class="enc-num" id="encLeft">0</div>
      </div>
      <div class="enc-box">
        <div class="enc-label">Right Wheel</div>
        <div class="enc-num" id="encRight">0</div>
      </div>
    </div>
    <div class="btn-row">
      <button class="btn btn-cyan" onclick="resetEncoders()">Reset Encoders</button>
      <button class="btn" onclick="blinkLed()">Blink LED</button>
    </div>
  </div>

  <!-- Virtual Serial Monitor -->
  <div class="panel-card">
    <div class="terminal-header">
      <div class="section-title" style="margin:0;">Wireless Serial Monitor</div>
      <div class="terminal-actions">
        <button class="term-btn" onclick="togglePause()" id="pauseBtn">Pause</button>
        <button class="term-btn" onclick="clearTerm()">Clear</button>
      </div>
    </div>
    <div class="terminal-box" id="termBox">
      <div class="term-line sys">> Wireless serial console initialized. Polling /api/telemetry...</div>
    </div>
  </div>
</div>

<script>
  let lastSeq = 0;
  let isPaused = false;
  let pollCount = 0;
  let lastFpsTime = Date.now();
  let termBox = document.getElementById("termBox");

  function updateSensorCard(cardId, valId, barId, statId, dist, ok) {
    const card = document.getElementById(cardId);
    const valSpan = document.getElementById(valId);
    const bar = document.getElementById(barId);
    const stat = document.getElementById(statId);

    card.className = "sensor-card";

    if (!ok) {
      valSpan.innerText = "--";
      bar.style.width = "0%";
      stat.innerText = "OFFLINE";
      stat.style.color = "var(--offline)";
      card.classList.add("offline");
      return;
    }

    if (dist >= 8000 || dist <= 0) {
      valSpan.innerText = ">1200";
      bar.style.width = "100%";
      bar.style.backgroundColor = "var(--safe)";
      stat.innerText = "CLEAR";
      stat.style.color = "var(--safe)";
      card.classList.add("safe");
      return;
    }

    valSpan.innerText = dist;
    let pct = Math.min(100, Math.max(8, (dist / 280) * 100));
    bar.style.width = pct + "%";

    if (dist < 60) {
      bar.style.backgroundColor = "var(--danger)";
      stat.innerText = "WALL CLOSE";
      stat.style.color = "var(--danger)";
      card.classList.add("danger");
    } else if (dist < 150) {
      bar.style.backgroundColor = "var(--warning)";
      stat.innerText = "WALL DETECTED";
      stat.style.color = "var(--warning)";
      card.classList.add("warning");
    } else {
      bar.style.backgroundColor = "var(--safe)";
      stat.innerText = "OPEN PATH";
      stat.style.color = "var(--safe)";
      card.classList.add("safe");
    }
  }

  function appendLog(line) {
    if (isPaused) return;
    const div = document.createElement("div");
    div.className = "term-line";
    if (line.includes("RESET") || line.includes("CALIBRATION")) {
      div.className += " highlight";
    } else if (line.includes("ToF") || line.includes("OK") || line.includes("WiFi")) {
      div.className += " sys";
    }
    div.textContent = line;
    termBox.appendChild(div);
    if (termBox.childNodes.length > 200) {
      termBox.removeChild(termBox.firstChild);
    }
    termBox.scrollTop = termBox.scrollHeight;
  }

  function fetchTelemetry() {
    fetch('/api/telemetry?since=' + lastSeq)
      .then(res => {
        if (!res.ok) throw new Error("HTTP error " + res.status);
        return res.json();
      })
      .then(data => {
        document.getElementById("statusBadge").className = "badge badge-live";
        document.getElementById("statusBadge").innerText = "ONLINE";
        if (data.net_info) {
          document.getElementById("netSubtitle").innerText = data.net_info;
        }

        // Uptime
        document.getElementById("uptimePill").innerText = "Up: " + data.uptime_s + "s";

        // ToF Sensors
        updateSensorCard("cardL",  "valL",  "barL",  "statL",  data.tof.l,  data.tof_ok.l);
        updateSensorCard("cardLF", "valLF", "barLF", "statLF", data.tof.lf, data.tof_ok.lf);
        updateSensorCard("cardF",  "valF",  "barF",  "statF",  data.tof.f,  data.tof_ok.f);
        updateSensorCard("cardRF", "valRF", "barRF", "statRF", data.tof.rf, data.tof_ok.rf);
        updateSensorCard("cardR",  "valR",  "barR",  "statR",  data.tof.r,  data.tof_ok.r);

        // Encoders
        document.getElementById("encLeft").innerText = data.enc.left;
        document.getElementById("encRight").innerText = data.enc.right;

        // Logs
        if (data.logs && data.logs.length > 0) {
          data.logs.forEach(msg => appendLog(msg));
        }
        if (data.seq !== undefined) {
          lastSeq = data.seq;
        }

        // Measure poll frequency (Hz)
        pollCount++;
        const now = Date.now();
        if (now - lastFpsTime >= 1000) {
          document.getElementById("ratePill").innerText = pollCount + " Hz";
          pollCount = 0;
          lastFpsTime = now;
        }
      })
      .catch(err => {
        document.getElementById("statusBadge").className = "badge badge-lost";
        document.getElementById("statusBadge").innerText = "RECONNECTING...";
      })
      .finally(() => {
        setTimeout(fetchTelemetry, 100);
      });
  }

  function resetEncoders() {
    fetch('/api/reset_encoders', { method: 'POST' })
      .then(() => appendLog(">>> [WEB] Encoder reset command sent <<<"));
  }

  function blinkLed() {
    fetch('/api/blink', { method: 'POST' })
      .then(() => appendLog(">>> [WEB] Onboard LED blinked <<<"));
  }

  function clearTerm() {
    termBox.innerHTML = "";
  }

  function togglePause() {
    isPaused = !isPaused;
    document.getElementById("pauseBtn").innerText = isPaused ? "Resume" : "Pause";
  }

  fetchTelemetry();
</script>
</body>
</html>
)rawliteral";

// =========================================================================
// WI-FI STATION CONFIGURATION (HOTSPOT ONLY)
// =========================================================================
const char* WIFI_SSID = "alienware";
const char* WIFI_PASS = "12345678";

WebServer server(80);
String currentNetInfo = "Connecting to alienware...";

// =========================================================================
// PIN DEFINITIONS
// =========================================================================
#define I2C_SDA   21
#define I2C_SCL   22

// XSHUT Shutdown Pins for 5x VL53L0X ToF
#define XSHUT_L   23
#define XSHUT_LF  19
#define XSHUT_F   18
#define XSHUT_RF  17
#define XSHUT_R   16

// I2C Unique Sensor Addresses
#define ADDR_L    0x30
#define ADDR_LF   0x31
#define ADDR_F    0x32
#define ADDR_RF   0x33
#define ADDR_R    0x34

// Quadrature Encoders
#define L_ENC_A   36
#define L_ENC_B   39
#define R_ENC_A   34
#define R_ENC_B   35

// Buttons & Status LED
#define BTN1      4    // Press to zero tick counters
#define BTN2      13
#define LED_PIN   2

// =========================================================================
// GLOBALS & SENSORS
// =========================================================================
VL53L0X sensorL;
VL53L0X sensorLF;
VL53L0X sensorF;
VL53L0X sensorRF;
VL53L0X sensorR;

bool okL  = false;
bool okLF = false;
bool okF  = false;
bool okRF = false;
bool okR  = false;

uint16_t distL  = 9999;
uint16_t distLF = 9999;
uint16_t distF  = 9999;
uint16_t distRF = 9999;
uint16_t distR  = 9999;

// 64-bit atomic tick counters
volatile int64_t leftTicks  = 0;
volatile int64_t rightTicks = 0;

void IRAM_ATTR isrLeftEncoder() {
    int a = digitalRead(L_ENC_A);
    int b = digitalRead(L_ENC_B);
    if (a == b) leftTicks++;
    else        leftTicks--;
}

void IRAM_ATTR isrRightEncoder() {
    int a = digitalRead(R_ENC_A);
    int b = digitalRead(R_ENC_B);
    if (a == b) rightTicks++;
    else        rightTicks--;
}

// =========================================================================
// WIRELESS SERIAL LOG RING BUFFER
// =========================================================================
#define LOG_BUFFER_SIZE 30
struct LogEntry {
    uint32_t seq;
    String message;
};

LogEntry logBuffer[LOG_BUFFER_SIZE];
int logHead = 0;
int logCount = 0;
uint32_t currentLogSeq = 0;

void logMessage(const String& msg) {
    Serial.println(msg);
    currentLogSeq++;
    logBuffer[logHead].seq = currentLogSeq;
    logBuffer[logHead].message = msg;
    logHead = (logHead + 1) % LOG_BUFFER_SIZE;
    if (logCount < LOG_BUFFER_SIZE) {
        logCount++;
    }
}

// =========================================================================
// TOF SENSOR HARDWARE INITIALIZATION
// =========================================================================
void initToFSensors() {
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(100000); // 100kHz standard stable speed

    pinMode(XSHUT_L, OUTPUT_OPEN_DRAIN);
    pinMode(XSHUT_LF, OUTPUT_OPEN_DRAIN);
    pinMode(XSHUT_F, OUTPUT_OPEN_DRAIN);
    pinMode(XSHUT_RF, OUTPUT_OPEN_DRAIN);
    pinMode(XSHUT_R, OUTPUT_OPEN_DRAIN);

    // Keep all sensors in hardware standby
    digitalWrite(XSHUT_L, LOW);
    digitalWrite(XSHUT_LF, LOW);
    digitalWrite(XSHUT_F, LOW);
    digitalWrite(XSHUT_RF, LOW);
    digitalWrite(XSHUT_R, LOW);
    delay(40);

    // 1. Left Sensor
    digitalWrite(XSHUT_L, HIGH);
    delay(25);
    sensorL.setTimeout(200);
    if (sensorL.init()) {
        sensorL.setAddress(ADDR_L);
        sensorL.startContinuous(50);
        okL = true;
        logMessage("[ToF] Left Sensor (0x30): OK");
    } else {
        logMessage("[ToF] Left Sensor: NOT DETECTED");
    }
    delay(20);

    // 2. Left-Front Sensor
    digitalWrite(XSHUT_LF, HIGH);
    delay(25);
    sensorLF.setTimeout(200);
    if (sensorLF.init()) {
        sensorLF.setAddress(ADDR_LF);
        sensorLF.startContinuous(50);
        okLF = true;
        logMessage("[ToF] Left-Front (0x31): OK");
    } else {
        logMessage("[ToF] Left-Front: NOT DETECTED");
    }
    delay(20);

    // 3. Front Sensor
    digitalWrite(XSHUT_F, HIGH);
    delay(25);
    sensorF.setTimeout(200);
    if (sensorF.init()) {
        sensorF.setAddress(ADDR_F);
        sensorF.startContinuous(50);
        okF = true;
        logMessage("[ToF] Front Sensor (0x32): OK");
    } else {
        logMessage("[ToF] Front Sensor: NOT DETECTED");
    }
    delay(20);

    // 4. Right-Front Sensor
    digitalWrite(XSHUT_RF, HIGH);
    delay(25);
    sensorRF.setTimeout(200);
    if (sensorRF.init()) {
        sensorRF.setAddress(ADDR_RF);
        sensorRF.startContinuous(50);
        okRF = true;
        logMessage("[ToF] Right-Front (0x33): OK");
    } else {
        logMessage("[ToF] Right-Front: NOT DETECTED");
    }
    delay(20);

    // 5. Right Sensor
    digitalWrite(XSHUT_R, HIGH);
    delay(25);
    sensorR.setTimeout(200);
    if (sensorR.init()) {
        sensorR.setAddress(ADDR_R);
        sensorR.startContinuous(50);
        okR = true;
        logMessage("[ToF] Right Sensor (0x34): OK");
    } else {
        logMessage("[ToF] Right Sensor: NOT DETECTED");
    }
    delay(20);
}

// =========================================================================
// HTTP SERVER HANDLERS
// =========================================================================
void handleRoot() {
    server.send_P(200, "text/html", DASHBOARD_HTML);
}

void handleTelemetry() {
    uint32_t since = 0;
    if (server.hasArg("since")) {
        since = (uint32_t)server.arg("since").toInt();
    }

    noInterrupts();
    int64_t curL = leftTicks;
    int64_t curR = rightTicks;
    interrupts();

    String json = "{";
    json += "\"uptime_s\":" + String(millis() / 1000) + ",";
    json += "\"net_info\":\"" + currentNetInfo + "\",";

    // 5-ToF Distances (mm)
    json += "\"tof\":{\"l\":" + String(distL) + ",\"lf\":" + String(distLF) + ",\"f\":" + String(distF) + ",\"rf\":" + String(distRF) + ",\"r\":" + String(distR) + "},";
    json += "\"tof_ok\":{\"l\":" + String(okL ? "true" : "false") + ",\"lf\":" + String(okLF ? "true" : "false") + ",\"f\":" + String(okF ? "true" : "false") + ",\"rf\":" + String(okRF ? "true" : "false") + ",\"r\":" + String(okR ? "true" : "false") + "},";

    // Encoders
    json += "\"enc\":{\"left\":" + String((long)curL) + ",\"right\":" + String((long)curR) + "},";
    json += "\"seq\":" + String(currentLogSeq) + ",";

    // Buffered Serial Logs
    json += "\"logs\":[";
    bool first = true;
    for (int i = 0; i < logCount; i++) {
        int idx = (logHead - logCount + i + LOG_BUFFER_SIZE) % LOG_BUFFER_SIZE;
        if (logBuffer[idx].seq > since) {
            if (!first) json += ",";
            first = false;
            String escaped = logBuffer[idx].message;
            escaped.replace("\"", "\\\"");
            json += "\"" + escaped + "\"";
        }
    }
    json += "]}";

    server.send(200, "application/json", json);
}

void handleResetEncoders() {
    noInterrupts();
    leftTicks  = 0;
    rightTicks = 0;
    interrupts();
    logMessage(">>> [WEB] Encoder counters zeroed <<<");
    server.send(200, "text/plain", "OK");
}

void handleBlink() {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    server.send(200, "text/plain", "OK");
}

// =========================================================================
// SETUP & MAIN LOOP
// =========================================================================
bool lastBtnState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

void setup() {
    // Disable brownout detector for clean Wi-Fi boot
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

    // Keep all ToF sensors off immediately during boot
    pinMode(XSHUT_L, OUTPUT_OPEN_DRAIN);
    pinMode(XSHUT_LF, OUTPUT_OPEN_DRAIN);
    pinMode(XSHUT_F, OUTPUT_OPEN_DRAIN);
    pinMode(XSHUT_RF, OUTPUT_OPEN_DRAIN);
    pinMode(XSHUT_R, OUTPUT_OPEN_DRAIN);
    digitalWrite(XSHUT_L, LOW);
    digitalWrite(XSHUT_LF, LOW);
    digitalWrite(XSHUT_F, LOW);
    digitalWrite(XSHUT_RF, LOW);
    digitalWrite(XSHUT_R, LOW);

    Serial.begin(115200);
    delay(300);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    pinMode(L_ENC_A, INPUT);
    pinMode(L_ENC_B, INPUT);
    pinMode(R_ENC_A, INPUT);
    pinMode(R_ENC_B, INPUT);

    pinMode(BTN1, INPUT_PULLUP);
    pinMode(BTN2, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(L_ENC_A), isrLeftEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(R_ENC_A), isrRightEncoder, CHANGE);

    logMessage("==============================================");
    logMessage("  ESP32 MICROMOUSE WIRELESS TELEMETRY SYSTEM  ");
    logMessage("==============================================");

    // Station mode ONLY (Connect to Alienware Hotspot)
    WiFi.mode(WIFI_STA);
    WiFi.setTxPower(WIFI_POWER_11dBm);

    logMessage("[WiFi] Connecting to hotspot: " + String(WIFI_SSID) + "...");
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    // Wait for connection
    while (WiFi.status() != WL_CONNECTED) {
        delay(300);
        Serial.print(".");
    }
    Serial.println();

    String ip = WiFi.localIP().toString();
    currentNetInfo = "Hotspot: " + String(WIFI_SSID) + " • IP: " + ip;
    logMessage("[WiFi] >>> CONNECTED TO " + String(WIFI_SSID) + "! <<<");
    logMessage("[WiFi] Web Dashboard IP: http://" + ip);

    if (MDNS.begin("micromouse")) {
        logMessage("[mDNS] Local Domain: http://micromouse.local");
    }

    // Initialize 5x ToF sensors
    initToFSensors();

    // Start Web Server
    server.on("/", HTTP_GET, handleRoot);
    server.on("/api/telemetry", HTTP_GET, handleTelemetry);
    server.on("/api/reset_encoders", HTTP_POST, handleResetEncoders);
    server.on("/api/blink", HTTP_POST, handleBlink);
    server.begin();

    logMessage("[HTTP] Telemetry server listening on port 80");
    digitalWrite(LED_PIN, LOW);
}

void loop() {
    server.handleClient();

    // Reset button (BTN1) handling
    int btnVal = digitalRead(BTN1);
    if (btnVal != lastBtnState) {
        lastDebounceTime = millis();
    }
    if ((millis() - lastDebounceTime) > debounceDelay) {
        if (btnVal == LOW && lastBtnState == HIGH) {
            noInterrupts();
            leftTicks  = 0;
            rightTicks = 0;
            interrupts();
            logMessage(">>> [BTN1] Hardware Reset: Encoders zeroed <<<");
            digitalWrite(LED_PIN, HIGH);
            delay(80);
            digitalWrite(LED_PIN, LOW);
        }
    }
    lastBtnState = btnVal;

    // Periodic sensor read and telemetry broadcast (~10Hz)
    static unsigned long lastUpdate = 0;
    if (millis() - lastUpdate >= 100) {
        lastUpdate = millis();

        if (okL)  distL  = sensorL.readRangeContinuousMillimeters();
        if (okLF) distLF = sensorLF.readRangeContinuousMillimeters();
        if (okF)  distF  = sensorF.readRangeContinuousMillimeters();
        if (okRF) distRF = sensorRF.readRangeContinuousMillimeters();
        if (okR)  distR  = sensorR.readRangeContinuousMillimeters();

        noInterrupts();
        int64_t curL = leftTicks;
        int64_t curR = rightTicks;
        interrupts();

        char buf[128];
        snprintf(buf, sizeof(buf), "L:%4d LF:%4d F:%4d RF:%4d R:%4d | Enc L:%5ld R:%5ld",
                 distL, distLF, distF, distRF, distR, (long)curL, (long)curR);
        logMessage(String(buf));
    }

    // Automatic Wi-Fi reconnection if hotspot signal drops
    static unsigned long lastWifiCheck = 0;
    if (millis() - lastWifiCheck >= 3000) {
        lastWifiCheck = millis();
        if (WiFi.status() != WL_CONNECTED) {
            WiFi.reconnect();
        }
    }
}
