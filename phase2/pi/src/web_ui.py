# ============================================================
#  web_ui.py
#  Simple Flask web dashboard for ESP32 tank mower
#
#  Run: python3 phase2/pi/src/web_ui.py
#  Open browser: http://tankmower.local:5000
# ============================================================

import sys
import os
sys.path.insert(0, os.path.dirname(__file__))

from flask import Flask, jsonify, render_template_string
from serial_bridge import ESP32Bridge
import logging
import threading

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(message)s"
)

app = Flask(__name__)
bridge = ESP32Bridge()

# ── HTML template ────────────────────────────────────────────
HTML = """
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Tank Mower</title>
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; }

    body {
      font-family: monospace;
      background: #0f0f0f;
      color: #e0e0e0;
      padding: 20px;
    }

    h1 {
      font-size: 1.4rem;
      color: #00aaff;
      margin-bottom: 20px;
      letter-spacing: 2px;
      text-transform: uppercase;
    }

    .grid {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 16px;
      max-width: 700px;
    }

    @media (max-width: 500px) { .grid { grid-template-columns: 1fr; } }

    .card {
      background: #1a1a1a;
      border: 1px solid #2a2a2a;
      border-radius: 8px;
      padding: 16px;
    }

    .card h2 {
      font-size: 0.75rem;
      color: #888;
      letter-spacing: 1px;
      text-transform: uppercase;
      margin-bottom: 12px;
    }

    /* Telemetry rows */
    .telem-row {
      display: flex;
      justify-content: space-between;
      padding: 5px 0;
      border-bottom: 1px solid #222;
      font-size: 0.9rem;
    }
    .telem-row:last-child { border-bottom: none; }
    .telem-label { color: #888; }
    .telem-value { color: #fff; font-weight: bold; }
    .telem-value.on  { color: #00ff88; }
    .telem-value.off { color: #ff4444; }
    .telem-value.warn { color: #ffaa00; }

    /* Status dot */
    .dot {
      display: inline-block;
      width: 10px; height: 10px;
      border-radius: 50%;
      margin-right: 6px;
      background: #ff4444;
    }
    .dot.online { background: #00ff88; animation: pulse 1.5s infinite; }
    @keyframes pulse {
      0%,100% { opacity: 1; } 50% { opacity: 0.4; }
    }

    /* Relay buttons */
    .relay-btn {
      width: 100%;
      padding: 12px;
      margin-bottom: 10px;
      border: none;
      border-radius: 6px;
      font-family: monospace;
      font-size: 0.9rem;
      font-weight: bold;
      letter-spacing: 1px;
      cursor: pointer;
      transition: opacity 0.15s;
    }
    .relay-btn:last-child { margin-bottom: 0; }
    .relay-btn:active { opacity: 0.7; }

    .btn-off  { background: #2a2a2a; color: #aaa; border: 1px solid #444; }
    .btn-on   { background: #00ff88; color: #000; border: 1px solid #00ff88; }
    .btn-motor-on { background: #ff4444; color: #fff; border: 1px solid #ff4444; }
    .btn-turbo-on { background: #00aaff; color: #000; border: 1px solid #00aaff; }

    /* Log */
    .log {
      background: #111;
      border: 1px solid #222;
      border-radius: 6px;
      padding: 10px;
      font-size: 0.75rem;
      color: #888;
      height: 120px;
      overflow-y: auto;
      margin-top: 4px;
    }
    .log-line { padding: 2px 0; border-bottom: 1px solid #1a1a1a; }
    .log-line.ok  { color: #00ff88; }
    .log-line.err { color: #ff4444; }
    .log-line.info { color: #00aaff; }

    .last-update {
      font-size: 0.7rem;
      color: #444;
      margin-top: 16px;
    }
  </style>
</head>
<body>

<h1>&#9651; Tank Mower</h1>

<div class="grid">

  <!-- Telemetry card -->
  <div class="card">
    <h2><span class="dot" id="conn-dot"></span>ESP32 Status</h2>
    <div class="telem-row">
      <span class="telem-label">Bridge</span>
      <span class="telem-value" id="t-bridge">--</span>
    </div>
    <div class="telem-row">
      <span class="telem-label">PS4 Controller</span>
      <span class="telem-value" id="t-ps4">--</span>
    </div>
    <div class="telem-row">
      <span class="telem-label">Drive Mode</span>
      <span class="telem-value" id="t-mode">--</span>
    </div>
    <div class="telem-row">
      <span class="telem-label">Battery</span>
      <span class="telem-value" id="t-batt">--</span>
    </div>
    <div class="telem-row">
      <span class="telem-label">Relay ARM</span>
      <span class="telem-value" id="t-arm">--</span>
    </div>
    <div class="telem-row">
      <span class="telem-label">Relay MOTOR</span>
      <span class="telem-value" id="t-motor">--</span>
    </div>
    <div class="telem-row">
      <span class="telem-label">Relay TURBO</span>
      <span class="telem-value" id="t-turbo">--</span>
    </div>
    <div class="telem-row">
      <span class="telem-label">Encoder L</span>
      <span class="telem-value" id="t-encl">--</span>
    </div>
    <div class="telem-row">
      <span class="telem-label">Encoder R</span>
      <span class="telem-value" id="t-encr">--</span>
    </div>
  </div>

  <!-- Relay controls card -->
  <div class="card">
    <h2>Relay Controls</h2>
    <button class="relay-btn btn-off" id="btn-motor" onclick="toggleRelay('motor')">
      MOTOR — OFF
    </button>
    <button class="relay-btn btn-off" id="btn-turbo" onclick="toggleRelay('turbo')">
      TURBO — OFF
    </button>

    <h2 style="margin-top:16px;">Action Log</h2>
    <div class="log" id="log"></div>
  </div>

</div>

<p class="last-update">Last update: <span id="last-update">--</span></p>

<script>
  let state = {};

  function addLog(msg, type='info') {
    const log = document.getElementById('log');
    const line = document.createElement('div');
    line.className = 'log-line ' + type;
    const t = new Date().toLocaleTimeString();
    line.textContent = `[${t}] ${msg}`;
    log.prepend(line);
    // keep last 50 lines
    while (log.children.length > 50) log.removeChild(log.lastChild);
  }

  function setVal(id, text, cls) {
    const el = document.getElementById(id);
    if (!el) return;
    el.textContent = text;
    el.className = 'telem-value ' + (cls || '');
  }

  function updateUI(s) {
    const online = s.bridge_connected;
    document.getElementById('conn-dot').className = 'dot ' + (online ? 'online' : '');
    setVal('t-bridge', online ? 'ONLINE' : 'OFFLINE', online ? 'on' : 'err');
    setVal('t-ps4',   s.connected ? 'CONNECTED' : 'WAITING', s.connected ? 'on' : 'warn');
    setVal('t-mode',  s.mode || '--');
    setVal('t-batt',  s.batt !== undefined ? s.batt + '%' : '--',
           s.batt > 30 ? 'on' : s.batt > 10 ? 'warn' : 'err');
    setVal('t-arm',   s.relay_arm   ? 'ON' : 'OFF', s.relay_arm   ? 'on' : 'off');
    setVal('t-motor', s.relay_motor ? 'ON' : 'OFF', s.relay_motor ? 'on' : 'off');
    setVal('t-turbo', s.relay_turbo ? 'ON' : 'OFF', s.relay_turbo ? 'on' : 'off');
    setVal('t-encl',  s.enc_l !== undefined ? s.enc_l : '--');
    setVal('t-encr',  s.enc_r !== undefined ? s.enc_r : '--');

    // Motor button
    const mb = document.getElementById('btn-motor');
    if (s.relay_motor) {
      mb.textContent = 'MOTOR — ON  (press to STOP)';
      mb.className = 'relay-btn btn-motor-on';
    } else {
      mb.textContent = 'MOTOR — OFF  (press to ARM + START)';
      mb.className = 'relay-btn btn-off';
    }

    // Turbo button
    const tb = document.getElementById('btn-turbo');
    if (s.relay_turbo) {
      tb.textContent = 'TURBO — ON  (press to OFF)';
      tb.className = 'relay-btn btn-turbo-on';
    } else {
      tb.textContent = 'TURBO — OFF  (press to ON)';
      tb.className = 'relay-btn btn-off';
    }

    document.getElementById('last-update').textContent = new Date().toLocaleTimeString();
    state = s;
  }

  async function fetchState() {
    try {
      const r = await fetch('/api/state');
      const s = await r.json();
      updateUI(s);
    } catch(e) {
      setVal('t-bridge', 'ERROR', 'err');
    }
  }

  async function toggleRelay(id) {
    const current = id === 'motor' ? state.relay_motor : state.relay_turbo;
    const newState = current ? 0 : 1;
    addLog(`${id.toUpperCase()} relay -> ${newState ? 'ON' : 'OFF'}`, newState ? 'ok' : 'info');
    try {
      await fetch(`/api/relay/${id}/${newState}`, { method: 'POST' });
    } catch(e) {
      addLog(`Error sending relay command: ${e}`, 'err');
    }
  }

  // Poll every 500ms
  fetchState();
  setInterval(fetchState, 500);
</script>
</body>
</html>
"""

# ── Flask routes ─────────────────────────────────────────────

@app.route("/")
def index():
    return render_template_string(HTML)

@app.route("/api/state")
def api_state():
    s = bridge.get_state()
    s["bridge_connected"] = bridge.is_connected()
    return jsonify(s)

@app.route("/api/relay/<relay_id>/<int:state>", methods=["POST"])
def api_relay(relay_id, state):
    if relay_id not in ("motor", "turbo"):
        return jsonify({"error": "unknown relay"}), 400
    bridge.set_relay(relay_id, bool(state))
    return jsonify({"ok": True, "relay": relay_id, "state": state})

# ── Main ─────────────────────────────────────────────────────

if __name__ == "__main__":
    try:
        bridge.start()
    except Exception as e:
        logging.error(f"Could not open serial port: {e}")
        logging.error("Is the ESP32 wired up? Check /dev/ttyAMA0")

    logging.info("Starting web UI at http://tankmower.local:5000")
    app.run(host="0.0.0.0", port=5000, debug=False, use_reloader=False)
