# ============================================================
#  web_ui.py
#  Flask web dashboard — telemetry + live LIDAR scan
#
#  Run: python3 phase2/pi/src/web_ui.py
#  Open: http://tankmower.local:5000
#
# ============================================================

import sys
import os
sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sensors'))

from flask import Flask, jsonify, render_template_string
from serial_bridge import ESP32Bridge
from lidar import LidarX4
from compass import Compass
import logging
import math

logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s %(levelname)s %(message)s")

app = Flask(__name__)
bridge = ESP32Bridge()
lidar   = LidarX4()
compass = Compass()

HTML = """
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Tank Mower</title>
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body { font-family: monospace; background: #0f0f0f; color: #e0e0e0; padding: 16px; }
    h1 { font-size: 1.3rem; color: #00aaff; margin-bottom: 16px; letter-spacing: 2px; text-transform: uppercase; }

    .layout { display: grid; grid-template-columns: 280px 1fr; gap: 16px; }
    @media(max-width:700px) { .layout { grid-template-columns: 1fr; } }

    .card { background: #1a1a1a; border: 1px solid #2a2a2a; border-radius: 8px; padding: 14px; margin-bottom: 16px; }
    .card h2 { font-size: 0.7rem; color: #888; letter-spacing: 1px; text-transform: uppercase; margin-bottom: 10px; }

    .telem-row { display: flex; justify-content: space-between; padding: 4px 0; border-bottom: 1px solid #222; font-size: 0.85rem; }
    .telem-row:last-child { border-bottom: none; }
    .telem-label { color: #888; }
    .telem-value { color: #fff; font-weight: bold; }
    .on  { color: #00ff88; }
    .off { color: #ff4444; }
    .warn { color: #ffaa00; }

    .dot { display: inline-block; width: 9px; height: 9px; border-radius: 50%; margin-right: 6px; background: #ff4444; }
    .dot.online { background: #00ff88; animation: pulse 1.5s infinite; }
    @keyframes pulse { 0%,100%{opacity:1} 50%{opacity:0.3} }

    .relay-btn { width: 100%; padding: 10px; margin-bottom: 8px; border: none; border-radius: 6px;
                 font-family: monospace; font-size: 0.85rem; font-weight: bold; cursor: pointer; transition: opacity 0.15s; }
    .relay-btn:last-child { margin-bottom: 0; }
    .relay-btn:active { opacity: 0.7; }
    .btn-off      { background: #2a2a2a; color: #aaa; border: 1px solid #444; }
    .btn-motor-on { background: #ff4444; color: #fff; }
    .btn-turbo-on { background: #00aaff; color: #000; }

    .log { background: #111; border: 1px solid #222; border-radius: 6px; padding: 8px;
           font-size: 0.72rem; color: #888; height: 100px; overflow-y: auto; margin-top: 8px; }
    .log-line { padding: 1px 0; }
    .log-line.ok   { color: #00ff88; }
    .log-line.err  { color: #ff4444; }
    .log-line.info { color: #00aaff; }

    #lidar-canvas { background: #111; border-radius: 8px; display: block; width: 100%; aspect-ratio: 1; }
    .lidar-stats { font-size: 0.72rem; color: #555; margin-top: 6px; }
    .lidar-stats span { color: #888; margin-right: 12px; }
    .footer { font-size: 0.65rem; color: #333; margin-top: 12px; }
  </style>
</head>
<body>

<h1>&#9651; Tank Mower</h1>

<div class="layout">
  <div>
    <div class="card">
      <h2><span class="dot" id="conn-dot"></span>ESP32</h2>
      <div class="telem-row"><span class="telem-label">Bridge</span><span class="telem-value" id="t-bridge">--</span></div>
      <div class="telem-row"><span class="telem-label">PS4</span><span class="telem-value" id="t-ps4">--</span></div>
      <div class="telem-row"><span class="telem-label">Mode</span><span class="telem-value" id="t-mode">--</span></div>
      <div class="telem-row"><span class="telem-label">Battery</span><span class="telem-value" id="t-batt">--</span></div>
      <div class="telem-row"><span class="telem-label">ARM relay</span><span class="telem-value" id="t-arm">--</span></div>
      <div class="telem-row"><span class="telem-label">MOTOR relay</span><span class="telem-value" id="t-motor">--</span></div>
      <div class="telem-row"><span class="telem-label">TURBO relay</span><span class="telem-value" id="t-turbo">--</span></div>
      <div class="telem-row"><span class="telem-label">Enc L</span><span class="telem-value" id="t-encl">--</span></div>
      <div class="telem-row"><span class="telem-label">Enc R</span><span class="telem-value" id="t-encr">--</span></div>
    </div>

    <div class="card">
      <h2>Relay Controls</h2>
      <button class="relay-btn btn-off" id="btn-motor" onclick="toggleRelay('motor')">MOTOR — OFF</button>
      <button class="relay-btn btn-off" id="btn-turbo" onclick="toggleRelay('turbo')">TURBO — OFF</button>
      <h2 style="margin-top:12px;">Log</h2>
      <div class="log" id="log"></div>
    </div>

    <div class="card">
      <h2><span class="dot" id="lidar-dot"></span>LIDAR</h2>
      <div class="telem-row"><span class="telem-label">Status</span><span class="telem-value" id="t-lidar">--</span></div>
      <div class="telem-row"><span class="telem-label">Points</span><span class="telem-value" id="t-points">--</span></div>
      <div class="telem-row"><span class="telem-label">Closest</span><span class="telem-value" id="t-closest">--</span></div>
      <div class="telem-row"><span class="telem-label">Front (0+-20deg)</span><span class="telem-value" id="t-front">--</span></div>
      <div class="telem-row"><span class="telem-label">Scans</span><span class="telem-value" id="t-scans">--</span></div>
    </div>

    <div class="card">
      <h2><span class="dot" id="compass-dot"></span>Compass / IMU</h2>
      <div class="telem-row"><span class="telem-label">Heading</span><span class="telem-value" id="t-heading">--</span></div>
      <div class="telem-row"><span class="telem-label">Roll</span><span class="telem-value" id="t-roll">--</span></div>
      <div class="telem-row"><span class="telem-label">Pitch</span><span class="telem-value" id="t-pitch">--</span></div>
      <div class="telem-row"><span class="telem-label">Temperature</span><span class="telem-value" id="t-temp">--</span></div>
      <div class="telem-row"><span class="telem-label">Calibration</span><span class="telem-value" id="t-cal">--</span></div>
    </div>
  </div>

  <div>
    <div class="card" style="height:calc(100% - 16px);">
      <h2>LIDAR Scan - Live 360</h2>
      <canvas id="lidar-canvas"></canvas>
      <div class="lidar-stats">
        <span id="ls-points">0 pts</span>
        <span id="ls-hz">-- Hz</span>
        <span style="color:#00ff88;">&#9679;</span> valid &nbsp;
        <span style="color:#ff4444;">&#9679;</span> no return &nbsp;
        <span style="color:#ffaa00;">&#9660;</span> robot
      </div>
    </div>
  </div>
</div>

<p class="footer">Last update: <span id="last-update">--</span></p>

<script>
  let state = {};
  let lastScanCount = 0;
  let lastScanTime = Date.now();
  let scanHz = 0;

  function addLog(msg, type='info') {
    const log = document.getElementById('log');
    const line = document.createElement('div');
    line.className = 'log-line ' + type;
    line.textContent = '[' + new Date().toLocaleTimeString() + '] ' + msg;
    log.prepend(line);
    while (log.children.length > 60) log.removeChild(log.lastChild);
  }

  function setVal(id, text, cls) {
    const el = document.getElementById(id);
    if (!el) return;
    el.textContent = text;
    el.className = 'telem-value ' + (cls || '');
  }

  function updateESP32(s) {
    const online = s.bridge_connected;
    document.getElementById('conn-dot').className = 'dot ' + (online ? 'online' : '');
    setVal('t-bridge', online ? 'ONLINE' : 'OFFLINE', online ? 'on' : 'err');
    setVal('t-ps4',    s.connected ? 'CONNECTED' : 'WAITING', s.connected ? 'on' : 'warn');
    setVal('t-mode',   s.mode || '--');
    setVal('t-batt',   s.batt !== undefined ? s.batt + '%' : '--',
           s.batt > 30 ? 'on' : s.batt > 10 ? 'warn' : 'err');
    setVal('t-arm',    s.relay_arm   ? 'ON' : 'OFF', s.relay_arm   ? 'on' : 'off');
    setVal('t-motor',  s.relay_motor ? 'ON' : 'OFF', s.relay_motor ? 'on' : 'off');
    setVal('t-turbo',  s.relay_turbo ? 'ON' : 'OFF', s.relay_turbo ? 'on' : 'off');
    setVal('t-encl',   s.enc_l !== undefined ? s.enc_l : '--');
    setVal('t-encr',   s.enc_r !== undefined ? s.enc_r : '--');

    const mb = document.getElementById('btn-motor');
    mb.textContent = s.relay_motor ? 'MOTOR - ON  (press to STOP)' : 'MOTOR - OFF  (press to ARM + START)';
    mb.className = 'relay-btn ' + (s.relay_motor ? 'btn-motor-on' : 'btn-off');

    const tb = document.getElementById('btn-turbo');
    tb.textContent = s.relay_turbo ? 'TURBO - ON  (press to OFF)' : 'TURBO - OFF  (press to ON)';
    tb.className = 'relay-btn ' + (s.relay_turbo ? 'btn-turbo-on' : 'btn-off');

    state = s;
  }

  function updateLidar(ls) {
    const online = ls.connected;
    document.getElementById('lidar-dot').className = 'dot ' + (online ? 'online' : '');
    setVal('t-lidar',   online ? 'ONLINE' : 'OFFLINE', online ? 'on' : 'err');
    setVal('t-points',  ls.points || 0);
    setVal('t-closest', ls.closest_dist < 99 ?
           ls.closest_dist.toFixed(2) + 'm @ ' + ls.closest_angle.toFixed(0) + 'deg' : '--');
    setVal('t-front',   ls.front_dist < 99 ? ls.front_dist.toFixed(2) + 'm' : '--',
           ls.front_dist < 0.5 ? 'err' : ls.front_dist < 1.0 ? 'warn' : 'on');
    setVal('t-scans',   ls.scan_count || 0);

    if (ls.scan_count !== lastScanCount) {
      const now = Date.now();
      const elapsed = (now - lastScanTime) / 1000;
      const newScans = ls.scan_count - lastScanCount;
      if (elapsed > 0) scanHz = (newScans / elapsed).toFixed(1);
      lastScanCount = ls.scan_count;
      lastScanTime = now;
    }

    document.getElementById('ls-points').textContent = (ls.points || 0) + ' pts';
    document.getElementById('ls-hz').textContent = scanHz + ' Hz';

    if (ls.scan && ls.scan.length > 0) drawScan(ls.scan);
  }

  function drawScan(points) {
    const canvas = document.getElementById('lidar-canvas');
    const W = canvas.offsetWidth;
    canvas.width  = W;
    canvas.height = W;
    const ctx = canvas.getContext('2d');
    const cx = W / 2, cy = W / 2;
    const maxDist = 8.0;
    const scale = (W / 2 - 20) / maxDist;

    ctx.clearRect(0, 0, W, W);

    // Grid rings
    ctx.lineWidth = 1;
    [2, 4, 6, 8].forEach(r => {
      ctx.strokeStyle = '#1e1e1e';
      ctx.beginPath();
      ctx.arc(cx, cy, r * scale, 0, Math.PI * 2);
      ctx.stroke();
      ctx.fillStyle = '#333';
      ctx.font = '10px monospace';
      ctx.fillText(r + 'm', cx + 2, cy - r * scale + 12);
    });

    // Grid lines
    ctx.strokeStyle = '#1e1e1e';
    [0, 45, 90, 135].forEach(deg => {
      const rad = deg * Math.PI / 180;
      ctx.beginPath();
      ctx.moveTo(cx + Math.cos(rad) * (W/2-10), cy + Math.sin(rad) * (W/2-10));
      ctx.lineTo(cx - Math.cos(rad) * (W/2-10), cy - Math.sin(rad) * (W/2-10));
      ctx.stroke();
    });

    // Labels
    ctx.fillStyle = '#444'; ctx.font = '11px monospace';
    ctx.fillText('0',   cx-4,  14);
    ctx.fillText('90',  W-22,  cy+4);
    ctx.fillText('180', cx-10, W-6);
    ctx.fillText('270', 4,     cy+4);

    // Points
    points.forEach(([angle, dist]) => {
      const rad = (angle - 90) * Math.PI / 180;
      const px = cx + Math.cos(rad) * Math.min(dist, maxDist) * scale;
      const py = cy + Math.sin(rad) * Math.min(dist, maxDist) * scale;
      const invalid = Math.abs(dist - 8.224) < 0.05 || dist > maxDist || dist < 0.1;
      ctx.beginPath();
      ctx.arc(px, py, 1.5, 0, Math.PI * 2);
      ctx.fillStyle = invalid ? '#ff444466' : '#00ff88';
      ctx.fill();
    });

    // Robot
    ctx.fillStyle = '#ffaa00';
    ctx.beginPath();
    ctx.moveTo(cx, cy-10);
    ctx.lineTo(cx-7, cy+6);
    ctx.lineTo(cx+7, cy+6);
    ctx.closePath();
    ctx.fill();

    ctx.strokeStyle = '#ffaa0066';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(cx, cy-30);
    ctx.stroke();
  }

  function updateCompass(c) {
    const online = c.connected;
    document.getElementById('compass-dot').className = 'dot ' + (online ? 'online' : '');
    setVal('t-heading', c.heading !== undefined ? c.heading.toFixed(1) + 'deg' : '--');
    setVal('t-roll',    c.roll    !== undefined ? c.roll.toFixed(1)    + 'deg' : '--');
    setVal('t-pitch',   c.pitch   !== undefined ? c.pitch.toFixed(1)   + 'deg' : '--');
    setVal('t-temp',    c.temp    !== undefined ? c.temp + 'C'         : '--');
    const calStr = c.cal_sys !== undefined ?
      'sys=' + c.cal_sys + ' gyro=' + c.cal_gyro + ' mag=' + c.cal_mag : '--';
    setVal('t-cal', calStr, c.calibrated ? 'on' : 'warn');
  }

  async function fetchState() {
    try {
      const r = await fetch('/api/state');
      updateESP32(await r.json());
    } catch(e) { setVal('t-bridge', 'ERROR', 'err'); }
  }

  async function fetchLidar() {
    try {
      const r = await fetch('/api/lidar');
      updateLidar(await r.json());
    } catch(e) {}
  }

  async function fetchCompass() {
    try {
      const r = await fetch('/api/compass');
      updateCompass(await r.json());
    } catch(e) {}
  }

  async function toggleRelay(id) {
    const current = id === 'motor' ? state.relay_motor : state.relay_turbo;
    const newState = current ? 0 : 1;
    addLog(id.toUpperCase() + ' -> ' + (newState ? 'ON' : 'OFF'), newState ? 'ok' : 'info');
    try {
      await fetch('/api/relay/' + id + '/' + newState, { method: 'POST' });
    } catch(e) { addLog('Error: ' + e, 'err'); }
  }

  fetchState(); fetchLidar(); fetchCompass();
  setInterval(fetchState,   500);
  setInterval(fetchLidar,   200);
  setInterval(fetchCompass, 200);
  setInterval(() => { document.getElementById('last-update').textContent = new Date().toLocaleTimeString(); }, 1000);
</script>
</body>
</html>
"""

@app.route("/")
def index():
    return render_template_string(HTML)

@app.route("/api/state")
def api_state():
    s = bridge.get_state()
    s["bridge_connected"] = bridge.is_connected()
    return jsonify(s)

@app.route("/api/lidar")
def api_lidar():
    scan    = lidar.get_scan()
    closest = lidar.get_closest()
    front   = lidar.get_sector_min(0, 20)
    stats   = lidar.get_stats()
    display = scan[::2] if len(scan) > 200 else scan
    return jsonify({
        "connected":      lidar.is_connected(),
        "points":         len(scan),
        "scan":           display,
        "closest_angle":  closest[0],
        "closest_dist":   closest[1],
        "front_dist":     front,
        "scan_count":     stats["scan_count"],
        "packets_parsed": stats.get("packets_parsed", stats.get("scan_count", 0)),
    })

@app.route("/api/relay/<relay_id>/<int:state>", methods=["POST"])
def api_relay(relay_id, state):
    if relay_id not in ("motor", "turbo"):
        return jsonify({"error": "unknown relay"}), 400
    bridge.set_relay(relay_id, bool(state))
    return jsonify({"ok": True, "relay": relay_id, "state": state})

@app.route("/api/compass")
def api_compass():
    return jsonify(compass.get_all())

if __name__ == "__main__":
    try:
        bridge.start()
    except Exception as e:
        logging.error(f"ESP32 bridge failed: {e} -- continuing without it")

    try:
        lidar.start()
        logging.info("LIDAR started on /dev/ttyUSB0")
    except Exception as e:
        logging.error(f"LIDAR failed: {e} -- continuing without it")

    try:
        compass.start()
        logging.info("Compass started")
    except Exception as e:
        logging.error(f"Compass failed: {e} -- continuing without it")

    logging.info("Web UI at http://tankmower.local:5000")
    app.run(host="0.0.0.0", port=5000, debug=False, use_reloader=False)
