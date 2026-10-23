#pragma once

// Minified, self-contained modern dark-mode responsive dashboard HTML/CSS/JS
// Zero external dependencies (no internet needed when connected to ESP32 AP)
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
    // Map 0 - 300mm to 0 - 100% width
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
        // Poll continuously at ~100ms interval (10 Hz)
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

  // Start polling immediately on page load
  fetchTelemetry();
</script>
</body>
</html>
)rawliteral";
