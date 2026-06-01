#!/usr/bin/env python3
# ============================================================
# main.py
# Tank Mower — Phase 2 entry point
#
# Starts all sensors and services in the correct order,
# runs the web UI, and shuts everything down cleanly on exit.
#
# Run:
#   cd phase2/pi
#   python3 src/main.py
#
# Open dashboard:
#   http://tankmower.local:5000
#   (or http://<pi-ip>:5000)
#
# Controls (web UI):
#   Relay buttons    — arm/disarm mower blade, toggle turbo
#   START MOWING     — begin autonomous coverage (boustrophedon)
#   START SPIRAL     — begin autonomous spiral coverage
#   STOP             — halt all autonomous movement immediately
#   PS4 controller   — always overrides autonomous mode (RC priority)
#
# Architecture:
#
#   ESP32Bridge  ←→  serial_bridge.py   (motor commands + encoder ticks)
#   Compass      ←   compass.py         (BNO055 heading via I2C)
#   LidarX4      ←   lidar.py           (YDLIDAR X4 scan)
#   Odometry     ←   odometry.py        (dead-reckoning pose)
#   Navigator    ←   navigator.py       (reactive obstacle avoidance)
#   CoveragePlanner← coverage.py        (boustrophedon / spiral)
#   Flask/web_ui ←   web_ui_v2.py       (dashboard + WebSocket)
#
# ============================================================

import sys
import os
import logging
import signal
import time
import threading

# Add src/ to path so relative imports work when run from project root
sys.path.insert(0, os.path.dirname(__file__))

from flask import Flask, jsonify, render_template_string, request
from serial_bridge import ESP32Bridge
from lidar         import LidarX4
from compass       import Compass
from odometry      import Odometry
from navigator     import Navigator
from coverage      import CoveragePlanner, CoverageMode

# ── Logging ──────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(name)s %(message)s",
    handlers=[
        logging.StreamHandler(sys.stdout),
    ]
)
logger = logging.getLogger("main")

# ── Configuration ────────────────────────────────────────────
SERIAL_PORT  = "/dev/ttyAMA0"   # Pi 5 hardware UART — try /dev/serial0 if issues
LIDAR_PORT   = "/dev/ttyUSB0"
WEB_HOST     = "0.0.0.0"
WEB_PORT     = 5000

# ── Component instances (module-level so web routes can access) ──
bridge   = ESP32Bridge(port=SERIAL_PORT)
lidar    = LidarX4(port=LIDAR_PORT)
compass  = Compass()
odometry = None    # created after bridge/compass start
navigator = None   # created after odometry
coverage  = None   # created after navigator

app = Flask(__name__)

# ── Startup ──────────────────────────────────────────────────

def start_components():
    global odometry, navigator, coverage

    # 1. Serial bridge (must be first — everything reads encoder ticks from it)
    try:
        bridge.start()
        logger.info("✓ ESP32 serial bridge started")
    except Exception as e:
        logger.error(f"✗ ESP32 bridge failed: {e} — continuing without motors")

    # 2. Compass (I2C — independent of bridge)
    try:
        compass.start()
        logger.info("✓ BNO055 compass started")
        # Give it 2 seconds to settle before anyone reads heading
        time.sleep(2.0)
    except Exception as e:
        logger.error(f"✗ Compass failed: {e} — heading fusion disabled")

    # 3. LiDAR
    try:
        lidar.start()
        logger.info("✓ YDLIDAR X4 started")
    except Exception as e:
        logger.error(f"✗ LiDAR failed: {e} — obstacle avoidance disabled")

    # 4. Odometry (needs bridge + compass)
    odometry = Odometry(bridge, compass)
    try:
        odometry.start()
        logger.info("✓ Odometry started")
    except Exception as e:
        logger.error(f"✗ Odometry failed: {e}")

    # 5. Navigator (needs lidar + bridge + compass)
    navigator = Navigator(lidar, bridge, compass)
    try:
        navigator.start()
        logger.info("✓ Navigator (obstacle avoidance) started")
    except Exception as e:
        logger.error(f"✗ Navigator failed: {e}")

    # 6. Coverage planner (needs navigator + odometry + compass + lidar)
    coverage = CoveragePlanner(navigator, odometry, compass, lidar)
    logger.info("✓ Coverage planner ready")

    logger.info("=" * 50)
    logger.info(f"  Web UI: http://tankmower.local:{WEB_PORT}")
    logger.info("=" * 50)


def stop_components():
    logger.info("Shutting down...")
    if coverage:
        coverage.stop()
    if navigator:
        navigator.stop()
    if odometry:
        odometry.stop()
    try:
        lidar.stop()
    except Exception:
        pass
    try:
        compass.stop()
    except Exception:
        pass
    try:
        bridge.close()
    except Exception:
        pass
    logger.info("All components stopped.")


# ── Signal handler for clean Ctrl+C ─────────────────────────
def _signal_handler(sig, frame):
    logger.info("Interrupt received — shutting down")
    stop_components()
    sys.exit(0)

signal.signal(signal.SIGINT,  _signal_handler)
signal.signal(signal.SIGTERM, _signal_handler)

# ── Flask routes ─────────────────────────────────────────────

@app.route("/")
def index():
    return render_template_string(DASHBOARD_HTML)


@app.route("/api/state")
def api_state():
    s = bridge.get_state()
    s["bridge_connected"] = bridge.is_connected()
    if odometry:
        s["pose"] = odometry.get_pose()
    if navigator:
        s["nav"] = navigator.get_status()
    if coverage:
        s["coverage"] = coverage.get_status()
    return jsonify(s)


@app.route("/api/lidar")
def api_lidar():
    scan    = lidar.get_scan()
    closest = lidar.get_closest()
    front   = lidar.get_sector_min(0, 20)
    stats   = lidar.get_stats()
    display = scan[::2] if len(scan) > 200 else scan
    c_angle, c_dist = closest
    return jsonify({
        "connected":      lidar.is_connected(),
        "points":         len(scan),
        "scan":           display,
        "closest_angle":  c_angle if c_angle is not None else 0,
        "closest_dist":   c_dist  if c_dist  is not None else 999,
        "front_dist":     front,
        "scan_count":     stats["scan_count"],
    })


@app.route("/api/compass")
def api_compass():
    return jsonify(compass.get_all())


@app.route("/api/relay/<relay_id>/<int:state>", methods=["POST"])
def api_relay(relay_id, state):
    if relay_id not in ("motor", "turbo"):
        return jsonify({"error": "unknown relay"}), 400
    bridge.set_relay(relay_id, bool(state))
    return jsonify({"ok": True, "relay": relay_id, "state": state})


@app.route("/api/nav/enable", methods=["POST"])
def api_nav_enable():
    if not navigator:
        return jsonify({"error": "navigator not initialised"}), 500
    navigator.enable_movement()
    return jsonify({"ok": True, "movement_enabled": True})


@app.route("/api/nav/disable", methods=["POST"])
def api_nav_disable():
    if not navigator:
        return jsonify({"error": "navigator not initialised"}), 500
    navigator.disable_movement()
    return jsonify({"ok": True, "movement_enabled": False})


@app.route("/api/nav/avoidance/<int:state>", methods=["POST"])
def api_nav_avoidance(state):
    if not navigator:
        return jsonify({"error": "navigator not initialised"}), 500
    if state:
        navigator.enable_avoidance()
    else:
        navigator.disable_avoidance()
    return jsonify({"ok": True, "avoidance_enabled": bool(state)})


@app.route("/api/coverage/start", methods=["POST"])
def api_coverage_start():
    """Start autonomous coverage."""
    if not coverage:
        return jsonify({"error": "coverage not initialised"}), 500

    data = request.get_json(silent=True) or {}
    mode_str = data.get("mode", "BOUSTROPHEDON").upper()
    mode = (CoverageMode.SPIRAL if mode_str == "SPIRAL"
            else CoverageMode.BOUSTROPHEDON)

    initial_heading = data.get("heading", None)

    # Arm blade first if not already running
    if not bridge.motor_running():
        bridge.set_relay("motor", True)
        time.sleep(0.8)   # allow arm sequence to complete

    coverage.start(mode=mode, initial_heading=initial_heading)
    logger.info(f"[API] Coverage START — mode={mode}")
    return jsonify({"ok": True, "mode": mode})


@app.route("/api/coverage/stop", methods=["POST"])
def api_coverage_stop():
    """Stop autonomous coverage (keeps blade armed)."""
    if coverage:
        coverage.stop()
    navigator.request_drive(0, 0) if navigator else None
    logger.info("[API] Coverage STOP")
    return jsonify({"ok": True})


@app.route("/api/reset_pose", methods=["POST"])
def api_reset_pose():
    """Zero the odometry pose at current position."""
    if odometry:
        odometry.reset()
    return jsonify({"ok": True})


# ── Dashboard HTML ───────────────────────────────────────────
# Extends the original web_ui.py HTML with:
#   - Pose display (x, y, heading)
#   - Navigator state badge
#   - Coverage controls (Start Boustrophedon / Start Spiral / Stop)
#   - Coverage status strip

DASHBOARD_HTML = """
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
.layout { display: grid; grid-template-columns: 300px 1fr; gap: 16px; }
@media(max-width:700px){ .layout { grid-template-columns: 1fr; } }
.card { background: #1a1a1a; border: 1px solid #2a2a2a; border-radius: 8px; padding: 14px; margin-bottom: 12px; }
.card h2 { font-size: 0.7rem; color: #888; letter-spacing: 1px; text-transform: uppercase; margin-bottom: 10px; }
.row { display: flex; justify-content: space-between; padding: 4px 0; border-bottom: 1px solid #222; font-size: 0.85rem; }
.row:last-child { border-bottom: none; }
.lbl { color: #888; } .val { color: #fff; font-weight: bold; }
.on { color: #00ff88; } .off { color: #ff4444; } .warn { color: #ffaa00; }
.dot { display: inline-block; width: 9px; height: 9px; border-radius: 50%; background: #ff4444; margin-right: 6px; }
.dot.online { background: #00ff88; animation: pulse 1.5s infinite; }
@keyframes pulse { 0%,100%{opacity:1} 50%{opacity:0.3} }
.badge { display: inline-block; padding: 2px 8px; border-radius: 4px; font-size: 0.75rem; font-weight: bold; margin-left: 8px; }
.badge-clear    { background:#003820; color:#00ff88; }
.badge-warn     { background:#3a2800; color:#ffaa00; }
.badge-blocked  { background:#3a0000; color:#ff4444; }
.badge-avoiding { background:#002040; color:#00aaff; }
.badge-stuck    { background:#440000; color:#ff0000; animation: pulse 0.5s infinite; }
.btn { width: 100%; padding: 10px; margin-bottom: 8px; border: none; border-radius: 6px;
       font-family: monospace; font-size: 0.85rem; font-weight: bold; cursor: pointer; transition: opacity 0.15s; }
.btn:active { opacity: 0.7; } .btn:last-child { margin-bottom: 0; }
.btn-off   { background:#2a2a2a; color:#aaa; border:1px solid #444; }
.btn-motor { background:#ff4444; color:#fff; }
.btn-turbo { background:#00aaff; color:#000; }
.btn-mow   { background:#004d20; color:#00ff88; border:1px solid #00ff88; }
.btn-spiral{ background:#002040; color:#00aaff; border:1px solid #00aaff; }
.btn-stop  { background:#4d0000; color:#ff4444; border:1px solid #ff4444; }
.log { background:#111; border:1px solid #222; border-radius:6px; padding:8px;
       font-size:0.72rem; color:#888; height:90px; overflow-y:auto; margin-top:8px; }
.log-line.ok { color:#00ff88; } .log-line.err { color:#ff4444; } .log-line.info { color:#00aaff; }
#lidar-canvas { background:#111; border-radius:8px; display:block; width:100%; aspect-ratio:1; }
.footer { font-size:0.65rem; color:#333; margin-top:10px; }
</style>
</head>
<body>
<h1>&#9651; Tank Mower — Phase 2</h1>
<div class="layout">
<div>

  <!-- ESP32 telemetry -->
  <div class="card">
    <h2><span class="dot" id="conn-dot"></span>ESP32</h2>
    <div class="row"><span class="lbl">Bridge</span><span class="val" id="t-bridge">--</span></div>
    <div class="row"><span class="lbl">PS4</span><span class="val" id="t-ps4">--</span></div>
    <div class="row"><span class="lbl">Mode</span><span class="val" id="t-mode">--</span></div>
    <div class="row"><span class="lbl">Battery</span><span class="val" id="t-batt">--</span></div>
    <div class="row"><span class="lbl">MOTOR relay</span><span class="val" id="t-motor">--</span></div>
    <div class="row"><span class="lbl">TURBO relay</span><span class="val" id="t-turbo">--</span></div>
    <div class="row"><span class="lbl">Enc L / R</span><span class="val" id="t-enc">--</span></div>
    <div class="row"><span class="lbl">Dist L / R</span><span class="val" id="t-dist">--</span></div>
  </div>

  <!-- Pose -->
  <div class="card">
    <h2>Pose (odometry)</h2>
    <div class="row"><span class="lbl">X</span><span class="val" id="t-x">--</span></div>
    <div class="row"><span class="lbl">Y</span><span class="val" id="t-y">--</span></div>
    <div class="row"><span class="lbl">Heading</span><span class="val" id="t-hdg">--</span></div>
    <button class="btn btn-off" onclick="resetPose()">Reset pose to 0,0</button>
  </div>

  <!-- Navigator -->
  <div class="card">
    <h2>Navigator</h2>
    <div class="row"><span class="lbl">State</span><span class="val" id="t-nav">--</span></div>
    <div class="row"><span class="lbl">Front</span><span class="val" id="t-nav-front">--</span></div>
    <div class="row"><span class="lbl">Left / Right</span><span class="val" id="t-nav-sides">--</span></div>
    <div class="row" style="margin-bottom:8px;"><span class="lbl">Avoidances</span><span class="val" id="t-nav-count">--</span></div>
    <button class="btn btn-off" id="btn-movement" onclick="toggleMovement()">MOVEMENT — OFF</button>
    <button class="btn btn-off" id="btn-avoidance" onclick="toggleAvoidance()">AVOIDANCE — ON</button>
  </div>

  <!-- Relay controls -->
  <div class="card">
    <h2>Relay Controls</h2>
    <button class="btn btn-off" id="btn-motor" onclick="toggleRelay('motor')">MOTOR — OFF</button>
    <button class="btn btn-off" id="btn-turbo" onclick="toggleRelay('turbo')">TURBO — OFF</button>
  </div>

  <!-- Coverage controls -->
  <div class="card">
    <h2>Coverage</h2>
    <div class="row" style="margin-bottom:8px;"><span class="lbl">Status</span><span class="val" id="t-cov">--</span></div>
    <div class="row" style="margin-bottom:8px;"><span class="lbl">Strips / Dist</span><span class="val" id="t-cov-strips">--</span></div>
    <button class="btn btn-mow"    onclick="startCoverage('BOUSTROPHEDON')">&#9655; Start Boustrophedon</button>
    <button class="btn btn-spiral" onclick="startCoverage('SPIRAL')">&#8635; Start Spiral</button>
    <button class="btn btn-stop"   onclick="stopCoverage()">&#9646;&#9646; Stop Coverage</button>
    <div class="log" id="log"></div>
  </div>

  <!-- Compass -->
  <div class="card">
    <h2><span class="dot" id="compass-dot"></span>Compass / IMU</h2>
    <div class="row"><span class="lbl">Heading</span><span class="val" id="t-heading">--</span></div>
    <div class="row"><span class="lbl">Roll / Pitch</span><span class="val" id="t-rp">--</span></div>
    <div class="row"><span class="lbl">Temp</span><span class="val" id="t-temp">--</span></div>
    <div class="row"><span class="lbl">Cal</span><span class="val" id="t-cal">--</span></div>
  </div>

</div>
<div>
  <!-- LIDAR scan -->
  <div class="card">
    <h2><span class="dot" id="lidar-dot"></span>LIDAR Scan — Live 360°</h2>
    <canvas id="lidar-canvas"></canvas>
    <div style="font-size:0.72rem;color:#555;margin-top:6px;">
      <span id="ls-points">0 pts</span> &nbsp;
      <span id="ls-hz">-- Hz</span> &nbsp;
      <span style="color:#00ff88;">&#9679;</span> valid &nbsp;
      <span style="color:#ffaa00;">&#9660;</span> robot
    </div>
  </div>
</div>
</div>
<p class="footer">Last update: <span id="last-update">--</span></p>

<script>
let state = {};
let lastScanCount = 0, lastScanTime = Date.now(), scanHz = 0;

function log(msg, type='info') {
  const el = document.getElementById('log');
  const line = document.createElement('div');
  line.className = 'log-line ' + type;
  line.textContent = '[' + new Date().toLocaleTimeString() + '] ' + msg;
  el.prepend(line);
  while (el.children.length > 50) el.removeChild(el.lastChild);
}

function sv(id, text, cls) {
  const el = document.getElementById(id);
  if (!el) return;
  el.textContent = text;
  el.className = 'val ' + (cls || '');
}

function navBadge(s) {
  const map = { CLEAR:'badge-clear', WARN:'badge-warn',
                BLOCKED:'badge-blocked', AVOIDING:'badge-avoiding', STUCK:'badge-stuck' };
  return `<span class="badge ${map[s]||''}">${s}</span>`;
}

function updateAll(s) {
  // Bridge
  const online = s.bridge_connected;
  document.getElementById('conn-dot').className = 'dot ' + (online ? 'online' : '');
  sv('t-bridge', online ? 'ONLINE' : 'OFFLINE', online ? 'on' : 'err');
  sv('t-ps4',   s.connected ? 'CONNECTED' : 'waiting', s.connected ? 'on' : 'warn');
  sv('t-mode',  s.mode || '--');
  sv('t-batt',  s.batt !== undefined ? s.batt + '%' : '--',
     s.batt > 30 ? 'on' : s.batt > 10 ? 'warn' : 'err');
  sv('t-motor', s.relay_motor ? 'ON' : 'OFF', s.relay_motor ? 'on' : 'off');
  sv('t-turbo', s.relay_turbo ? 'ON' : 'OFF', s.relay_turbo ? 'on' : 'off');
  sv('t-enc',   `${s.enc_l ?? '--'} / ${s.enc_r ?? '--'}`);
  sv('t-dist',  s.dist_l !== undefined
    ? `${s.dist_l.toFixed(2)}m / ${s.dist_r.toFixed(2)}m` : '--');

  // Pose
  if (s.pose) {
    sv('t-x',   s.pose.x.toFixed(3) + ' m');
    sv('t-y',   s.pose.y.toFixed(3) + ' m');
    sv('t-hdg', s.pose.heading_deg.toFixed(1) + '°');
  }

  // Navigator
  if (s.nav) {
    document.getElementById('t-nav').innerHTML = navBadge(s.nav.state);
    sv('t-nav-front', s.nav.front_m + ' m',
       s.nav.front_m < 0.55 ? 'err' : s.nav.front_m < 1.2 ? 'warn' : 'on');
    sv('t-nav-sides', `L ${s.nav.left_m}m / R ${s.nav.right_m}m`);
    sv('t-nav-count', s.nav.avoidance_count);

    const mb2 = document.getElementById('btn-movement');
    const movOn = s.nav.movement_enabled;
    mb2.textContent = movOn ? 'MOVEMENT — ON (click to disable)' : 'MOVEMENT — OFF (click to enable)';
    mb2.className = 'btn ' + (movOn ? 'btn-motor' : 'btn-off');

    const ab = document.getElementById('btn-avoidance');
    const avOn = s.nav.avoidance_enabled;
    ab.textContent = avOn ? 'AVOIDANCE — ON (click to disable)' : 'AVOIDANCE — OFF (click to enable)';
    ab.className = 'btn ' + (avOn ? 'btn-turbo' : 'btn-off');
  }

  // Coverage
  if (s.coverage) {
    sv('t-cov', s.coverage.state + ' [' + s.coverage.mode + ']',
       s.coverage.state === 'MOWING' ? 'on' : '');
    sv('t-cov-strips',
       `${s.coverage.strip_count} strips / ${s.coverage.total_dist_m}m`);
  }

  // Relay buttons
  const mb = document.getElementById('btn-motor');
  mb.textContent = s.relay_motor ? 'MOTOR ON — press to STOP' : 'MOTOR OFF — press to ARM';
  mb.className = 'btn ' + (s.relay_motor ? 'btn-motor' : 'btn-off');

  const tb = document.getElementById('btn-turbo');
  tb.textContent = s.relay_turbo ? 'TURBO ON — press to OFF' : 'TURBO OFF — press to ON';
  tb.className = 'btn ' + (s.relay_turbo ? 'btn-turbo' : 'btn-off');

  state = s;
}

function updateLidar(ls) {
  const online = ls.connected;
  document.getElementById('lidar-dot').className = 'dot ' + (online ? 'online' : '');
  if (ls.scan_count !== lastScanCount) {
    const now = Date.now();
    const elapsed = (now - lastScanTime) / 1000;
    const newScans = ls.scan_count - lastScanCount;
    if (elapsed > 0) scanHz = (newScans / elapsed).toFixed(1);
    lastScanCount = ls.scan_count; lastScanTime = now;
  }
  document.getElementById('ls-points').textContent = (ls.points || 0) + ' pts';
  document.getElementById('ls-hz').textContent = scanHz + ' Hz';
  if (ls.scan && ls.scan.length > 0) drawScan(ls.scan);
}

function updateCompass(c) {
  const online = c.connected;
  document.getElementById('compass-dot').className = 'dot ' + (online ? 'online' : '');
  sv('t-heading', c.heading !== undefined ? c.heading.toFixed(1) + '°' : '--');
  sv('t-rp', c.roll !== undefined
    ? c.roll.toFixed(1) + '° / ' + c.pitch.toFixed(1) + '°' : '--');
  sv('t-temp', c.temp !== undefined ? c.temp + '°C' : '--');
  sv('t-cal',
    c.cal_sys !== undefined
      ? `sys=${c.cal_sys} gyro=${c.cal_gyro} mag=${c.cal_mag}`
      : '--',
    c.calibrated ? 'on' : 'warn');
}

function drawScan(points) {
  const canvas = document.getElementById('lidar-canvas');
  const W = canvas.offsetWidth;
  canvas.width = W; canvas.height = W;
  const ctx = canvas.getContext('2d');
  const cx = W/2, cy = W/2, maxDist = 8.0;
  const scale = (W/2 - 20) / maxDist;
  ctx.clearRect(0, 0, W, W);
  [2,4,6,8].forEach(r => {
    ctx.strokeStyle='#1e1e1e'; ctx.beginPath();
    ctx.arc(cx, cy, r*scale, 0, Math.PI*2); ctx.stroke();
    ctx.fillStyle='#333'; ctx.font='10px monospace';
    ctx.fillText(r+'m', cx+2, cy - r*scale + 12);
  });
  [0,45,90,135].forEach(deg => {
    const rad = deg*Math.PI/180;
    ctx.strokeStyle='#1e1e1e'; ctx.beginPath();
    ctx.moveTo(cx + Math.cos(rad)*(W/2-10), cy + Math.sin(rad)*(W/2-10));
    ctx.lineTo(cx - Math.cos(rad)*(W/2-10), cy - Math.sin(rad)*(W/2-10));
    ctx.stroke();
  });
  ctx.fillStyle='#444'; ctx.font='11px monospace';
  ctx.fillText('0',cx-4,14); ctx.fillText('90',W-22,cy+4);
  ctx.fillText('180',cx-10,W-6); ctx.fillText('270',4,cy+4);
  points.forEach(([angle,dist]) => {
    const rad = (angle-90)*Math.PI/180;
    const px = cx + Math.cos(rad)*Math.min(dist,maxDist)*scale;
    const py = cy + Math.sin(rad)*Math.min(dist,maxDist)*scale;
    ctx.beginPath(); ctx.arc(px,py,1.5,0,Math.PI*2);
    ctx.fillStyle = (dist<0.12||dist>maxDist) ? '#ff444466' : '#00ff88';
    ctx.fill();
  });
  ctx.fillStyle='#ffaa00'; ctx.beginPath();
  ctx.moveTo(cx,cy-10); ctx.lineTo(cx-7,cy+6); ctx.lineTo(cx+7,cy+6);
  ctx.closePath(); ctx.fill();
}

async function fetchState() {
  try { updateAll(await (await fetch('/api/state')).json()); }
  catch(e) { sv('t-bridge','ERROR','err'); }
}
async function fetchLidar() {
  try { updateLidar(await (await fetch('/api/lidar')).json()); }
  catch(e) {}
}
async function fetchCompass() {
  try { updateCompass(await (await fetch('/api/compass')).json()); }
  catch(e) {}
}

async function toggleRelay(id) {
  const cur = id === 'motor' ? state.relay_motor : state.relay_turbo;
  const newState = cur ? 0 : 1;
  log(id.toUpperCase() + ' → ' + (newState ? 'ON' : 'OFF'), newState ? 'ok' : 'info');
  try { await fetch('/api/relay/' + id + '/' + newState, {method:'POST'}); }
  catch(e) { log('Error: '+e,'err'); }
}

async function startCoverage(mode) {
  log('Starting ' + mode + '...', 'info');
  try {
    const r = await fetch('/api/coverage/start', {
      method:'POST', headers:{'Content-Type':'application/json'},
      body: JSON.stringify({mode})
    });
    const d = await r.json();
    log(d.ok ? mode + ' started' : 'Error: '+d.error, d.ok ? 'ok' : 'err');
  } catch(e) { log('Error: '+e,'err'); }
}

async function stopCoverage() {
  log('Coverage STOP', 'info');
  try { await fetch('/api/coverage/stop', {method:'POST'}); }
  catch(e) { log('Error: '+e,'err'); }
}

async function toggleMovement() {
  const movOn = state.nav && state.nav.movement_enabled;
  const url = movOn ? '/api/nav/disable' : '/api/nav/enable';
  log('Movement → ' + (movOn ? 'OFF' : 'ON'), movOn ? 'info' : 'ok');
  try { await fetch(url, {method:'POST'}); }
  catch(e) { log('Error: '+e,'err'); }
}

async function toggleAvoidance() {
  const avOn = state.nav && state.nav.avoidance_enabled;
  const url = '/api/nav/avoidance/' + (avOn ? '0' : '1');
  log('Avoidance → ' + (avOn ? 'OFF' : 'ON'), 'info');
  try { await fetch(url, {method:'POST'}); }
  catch(e) { log('Error: '+e,'err'); }
}

async function resetPose() {
  try { await fetch('/api/reset_pose', {method:'POST'}); log('Pose reset','info'); }
  catch(e) {}
}

fetchState(); fetchLidar(); fetchCompass();
setInterval(fetchState,   500);
setInterval(fetchLidar,   200);
setInterval(fetchCompass, 300);
setInterval(() => {
  document.getElementById('last-update').textContent = new Date().toLocaleTimeString();
}, 1000);
</script>
</body>
</html>
"""

# ── Entry point ──────────────────────────────────────────────

if __name__ == "__main__":
    logger.info("=" * 50)
    logger.info("  Tank Mower — Phase 2 startup")
    logger.info("=" * 50)

    start_components()

    # Run Flask in the main thread (use_reloader=False is essential
    # when background threads are already running)
    app.run(
        host=WEB_HOST,
        port=WEB_PORT,
        debug=False,
        use_reloader=False,
        threaded=True,
    )
