#!/usr/bin/env python3
"""
test_bearing.py — Bearing-hold + pivot test web UI.

Uses _YukonLink directly (same layer the nav module calls) so the RC TX
switch position cannot override the test.  A background heartbeat keeps the
Yukon in AUTO mode; iBUS stick inputs are ignored by the firmware in AUTO.

Two test modes:
  Forward  — drives both motors at power% forward while the Yukon bearing-hold
              PID steers to maintain the target heading (PASSING state).
  Pivot    — turns in place using Pi-side differential steering
              l = fwd - steer,  r = fwd + steer  (ALIGNING state).
              Stops automatically within 3° of target.

Usage:
    python3 tools/test_bearing.py
    python3 tools/test_bearing.py --yukon-port /dev/ttyACM0
    python3 tools/test_bearing.py --port 5010
"""

import sys
import os
import time
import json
import threading
import argparse

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from flask import Flask, Response, request, jsonify
from robot_daemon import _YukonLink
from robot_utils import _local_ip

# ── CLI ───────────────────────────────────────────────────────────────────────

parser = argparse.ArgumentParser(description="Bearing-hold + pivot test web UI")
parser.add_argument('--yukon-port', default=None,  help='Yukon serial port (default: auto)')
parser.add_argument('--port',       default=5010, type=int, help='Web server port (default: 5010)')
args = parser.parse_args()

# ── Yukon connection ──────────────────────────────────────────────────────────

def _find_port(override):
    if override:
        return override
    try:
        import serial.tools.list_ports
        for p in serial.tools.list_ports.comports():
            if p.vid == 0x2E8A:
                return p.device
    except Exception:
        pass
    return '/dev/yukon'

_port  = _find_port(args.yukon_port)
print(f"Connecting to Yukon on {_port} …")
yukon  = _YukonLink(_port)
print("Connected.")

# ── Shared state ──────────────────────────────────────────────────────────────

_lock         = threading.Lock()
_heading      = None   # last IMU heading from CMD_SENSOR
_telemetry    = None   # last full Telemetry object from CMD_SENSOR
_drive_l      = 0.0    # last left  speed sent to yukon.drive()
_drive_r      = 0.0    # last right speed sent to yukon.drive()
_fwd_bearing  = None   # target bearing set in Forward mode (None = not active)
_run_active   = False
_run_timer    = None
_pivot_active = False
_pivot_err    = None   # last heading error (degrees)

# Matches BEARING_KP in yukon firmware/main.py — used to compute expected correction
_BEARING_KP = 0.4
_tool_running = True   # cleared on shutdown to stop background threads

# ── Background: CMD_MODE=AUTO heartbeat ──────────────────────────────────────
# Keeps the Yukon in AUTO so iBUS sticks are ignored.
# Interval must be < PI_FAILSAFE_MS (500 ms); 400 ms gives comfortable margin.

def _heartbeat():
    while _tool_running:
        try:
            yukon.set_mode(1)   # 1 = AUTO
        except Exception:
            pass
        time.sleep(0.4)

threading.Thread(target=_heartbeat, daemon=True, name='heartbeat').start()

# ── Background: heading poll ──────────────────────────────────────────────────

def _poll_heading():
    global _heading, _telemetry
    while _tool_running:
        try:
            t = yukon.query_sensor()
            if t is not None:
                with _lock:
                    _telemetry = t
                    if t.heading is not None:
                        _heading = round(t.heading, 1)
        except Exception:
            pass
        time.sleep(0.1)

threading.Thread(target=_poll_heading, daemon=True, name='heading_poll').start()

# ── Helpers ───────────────────────────────────────────────────────────────────

def _clamp(v, lo=-1.0, hi=1.0):
    return max(lo, min(hi, v))

def _angle_diff(target, current):
    return (target - current + 180.0) % 360.0 - 180.0

def _kill():
    global _drive_l, _drive_r
    try:
        yukon.kill()
        yukon.clear_bearing()
    except Exception:
        pass
    with _lock:
        _drive_l = 0.0
        _drive_r = 0.0

def _drive(l, r):
    global _drive_l, _drive_r
    try:
        yukon.drive(l, r)
    except Exception:
        pass
    with _lock:
        _drive_l = l
        _drive_r = r

# ── Forward mode ──────────────────────────────────────────────────────────────

def _stop_run():
    global _run_active, _run_timer, _fwd_bearing
    with _lock:
        if _run_timer is not None:
            _run_timer.cancel()
        _run_active  = False
        _run_timer   = None
        _fwd_bearing = None
    _kill()

def _start_run(bearing, power, run_time):
    global _run_active, _run_timer, _fwd_bearing
    _stop_pivot()
    _stop_run()
    speed = power / 100.0
    try:
        yukon.set_bearing(bearing)
    except Exception:
        pass
    _drive(speed, speed)
    with _lock:
        _run_active  = True
        _fwd_bearing = bearing
        if run_time > 0:
            t = threading.Timer(run_time, _stop_run)
            t.daemon = True
            t.start()
            _run_timer = t

# ── Pivot mode ────────────────────────────────────────────────────────────────

_STEER_KP       = 0.6
_ALIGN_DEADBAND = 3.0

def _stop_pivot():
    global _pivot_active, _pivot_err
    with _lock:
        _pivot_active = False
        _pivot_err    = None
    _kill()

def _pivot_loop(target, max_speed, fwd_speed):
    global _pivot_active, _pivot_err
    while True:
        with _lock:
            if not _pivot_active:
                break
            hdg = _heading

        if hdg is None:
            time.sleep(0.05)
            continue

        err   = _angle_diff(target, hdg)
        steer = _clamp(_STEER_KP * err / 45.0) * max_speed
        l     = _clamp(fwd_speed - steer)
        r     = _clamp(fwd_speed + steer)

        with _lock:
            _pivot_err = round(err, 1)

        if abs(err) < _ALIGN_DEADBAND:
            break

        _drive(l, r)
        time.sleep(0.1)

    _kill()
    with _lock:
        _pivot_active = False

def _start_pivot(target, max_speed, fwd_speed):
    global _pivot_active
    _stop_run()
    _stop_pivot()
    with _lock:
        _pivot_active = True
    threading.Thread(
        target=_pivot_loop,
        args=(target, max_speed, fwd_speed),
        daemon=True,
    ).start()

# ── Flask ─────────────────────────────────────────────────────────────────────

app = Flask(__name__)

@app.route('/')
def index():
    return Response(PAGE_HTML, mimetype='text/html')

def _state_dict():
    with _lock:
        hdg  = _heading
        run  = _run_active
        pivot= _pivot_active
        perr = _pivot_err
        t    = _telemetry
        dl   = _drive_l
        dr   = _drive_r
        fbrg = _fwd_bearing

    # Motor speeds: use actual applied values from firmware v5+; estimate for v4
    fwd_err = fwd_corr = None
    use_actual = (t is not None and
                  t.firmware_version is not None and t.firmware_version >= 5 and
                  (t.applied_l != 0.0 or t.applied_r != 0.0 or not run))
    if use_actual:
        eff_l = round(t.applied_l * 100)
        eff_r = round(t.applied_r * 100)
        if run and fbrg is not None and hdg is not None:
            fwd_err = round(_angle_diff(fbrg, hdg), 1)
            fwd_corr = eff_l - round(dl * 100)   # actual vs commanded
    else:
        eff_l = round(dl * 100)
        eff_r = round(dr * 100)
        if run and fbrg is not None and hdg is not None:
            fwd_err  = round(_angle_diff(fbrg, hdg), 1)
            fwd_corr = round(_clamp(_BEARING_KP * fwd_err / 180.0) * 100)  # estimated %
            eff_l    = round(_clamp(dl + _clamp(_BEARING_KP * fwd_err / 180.0)) * 100)
            eff_r    = round(_clamp(dr - _clamp(_BEARING_KP * fwd_err / 180.0)) * 100)

    motors = None
    if t is not None:
        motors = dict(
            voltage  = round(t.voltage, 2),
            current  = round(t.current, 2),
            actual   = use_actual,
            # Left side: FL (SLOT3) and RL (SLOT4); right side: FR (SLOT2) and RR (SLOT1)
            fl = dict(speed=eff_l, current=round(t.fl_current, 2), fault=t.fl_fault),
            rl = dict(speed=eff_l, current=round(t.rl_current, 2), fault=t.rl_fault),
            fr = dict(speed=eff_r, current=round(t.fr_current, 2), fault=t.fr_fault),
            rr = dict(speed=eff_r, current=round(t.rr_current, 2), fault=t.rr_fault),
        )
    return dict(
        heading  = hdg,
        running  = run,
        pivot    = pivot,
        pivot_err= perr,
        fwd_err  = fwd_err,
        fwd_corr = fwd_corr,
        motors   = motors,
    )

@app.route('/stream')
def stream():
    def generate():
        while True:
            yield f"data: {json.dumps(_state_dict())}\n\n"
            time.sleep(0.05)
    return Response(generate(), mimetype='text/event-stream',
                    headers={'Cache-Control': 'no-cache', 'X-Accel-Buffering': 'no'})

@app.route('/start', methods=['POST'])
def start():
    d = request.get_json(force=True)
    _start_run(
        float(d.get('bearing',  0)) % 360,
        max(0, min(100, float(d.get('power',   50)))),
        max(0, float(d.get('run_time', 0))),
    )
    return jsonify(ok=True)

@app.route('/pivot', methods=['POST'])
def pivot():
    d = request.get_json(force=True)
    _start_pivot(
        float(d.get('target',    0)) % 360,
        max(0.0, min(1.0, float(d.get('max_speed', 0.5)))),
        max(0.0, min(0.5, float(d.get('fwd_speed', 0.0)))),
    )
    return jsonify(ok=True)

@app.route('/stop', methods=['POST'])
def stop():
    _stop_run()
    _stop_pivot()
    return jsonify(ok=True)

# ── HTML ──────────────────────────────────────────────────────────────────────

PAGE_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Bearing Test</title>
<style>
* { box-sizing: border-box; margin: 0; padding: 0; }
body {
  font-family: system-ui, sans-serif;
  background: #0f1117; color: #e2e8f0;
  display: flex; flex-direction: column; align-items: center;
  min-height: 100vh; padding: 24px 16px;
}
h1 { font-size: 1.3rem; margin-bottom: 18px; color: #94a3b8; letter-spacing:.05em; }

/* Top row: compass left, motor panel right */
.top-row {
  display:flex; gap:16px; align-items:flex-start;
  width:100%; max-width:740px; margin-bottom:16px;
}
.compass-col {
  display:flex; flex-direction:column; align-items:center; flex-shrink:0;
}

/* Compass */
.compass-wrap { position:relative; width:200px; height:200px; margin-bottom:12px; }
.compass-ring {
  width:200px; height:200px; border-radius:50%;
  border:3px solid #334155;
  background: radial-gradient(circle, #1e293b 60%, #0f172a);
  position:absolute;
}
.compass-needle {
  position:absolute; left:50%; top:50%;
  width:4px; height:82px; margin-left:-2px; margin-top:-82px;
  background: linear-gradient(to bottom, #ef4444 50%, #94a3b8 50%);
  border-radius:2px; transform-origin:bottom center;
  transition: transform 0.25s ease;
}
.compass-target {
  position:absolute; left:50%; top:50%;
  width:2px; height:72px; margin-left:-1px; margin-top:-72px;
  background:#22d3ee; border-radius:1px;
  transform-origin:bottom center; opacity:0.75;
}
.compass-center {
  position:absolute; left:50%; top:50%;
  width:11px; height:11px; margin-left:-5.5px; margin-top:-5.5px;
  background:#475569; border-radius:50%;
}
.compass-n {
  position:absolute; top:5px; left:50%;
  transform:translateX(-50%);
  font-size:.7rem; font-weight:bold; color:#ef4444;
}

/* Heading */
.heading-box { text-align:center; }
.heading-val {
  font-size:3.2rem; font-weight:700;
  font-variant-numeric:tabular-nums; color:#f1f5f9; line-height:1;
}
.heading-label { font-size:.75rem; color:#64748b; margin-top:3px; }

/* Two-card row */
.cards-row {
  display:flex; gap:14px;
  width:100%; max-width:740px; align-items:flex-start;
}
.card {
  flex:1;
  background:#1e293b; border:1px solid #334155;
  border-radius:12px; padding:16px;
}
.card-title {
  font-size:.7rem; font-weight:600; text-transform:uppercase;
  letter-spacing:.08em; color:#64748b; margin-bottom:12px;
}
.field { margin-bottom:12px; }
label { display:block; font-size:.78rem; color:#94a3b8; margin-bottom:5px; }
input[type=number] {
  width:100%; background:#0f1117; border:1px solid #334155;
  border-radius:7px; padding:9px 10px;
  font-size:1rem; color:#f1f5f9; outline:none;
  -moz-appearance:textfield;
}
input[type=number]::-webkit-inner-spin-button { opacity:1; }
.two-col { display:grid; grid-template-columns:1fr 1fr; gap:10px; margin-bottom:12px; }

/* Buttons */
.btn-row { display:flex; gap:10px; }
.btn {
  flex:1; padding:12px; border:none; border-radius:9px;
  font-size:1rem; font-weight:600; cursor:pointer;
  transition: opacity .15s, transform .1s;
}
.btn:active { transform:scale(0.97); }
.btn-go   { background:#22c55e; color:#052e16; }
.btn-stop { background:#ef4444; color:#fff; }

/* Status */
.status { font-size:.8rem; color:#64748b; margin-top:8px; text-align:center; min-height:1.1em; }
.status.running { color:#22c55e; }
.status.pivot   { color:#f59e0b; }

/* Error bar */
.err-bar-wrap {
  height:7px; background:#0f1117; border-radius:4px;
  margin-top:8px; overflow:hidden; position:relative;
}
.err-bar {
  position:absolute; top:0; height:100%;
  background:#f59e0b; border-radius:4px;
  transition: left .1s, width .1s;
}
.err-center {
  position:absolute; left:50%; top:0;
  width:2px; height:100%; background:#334155; margin-left:-1px;
}
.err-label { font-size:.72rem; color:#64748b; text-align:center; margin-top:3px; }

/* Motor panel — fills remaining width in top row */
.motor-panel { flex:1; min-width:0; }
.motor-header {
  font-size:.85rem; color:#94a3b8; margin-bottom:12px; text-align:center;
}
.motor-grid {
  display:grid; grid-template-columns:1fr 1fr; gap:8px;
}
.motor-cell {
  background:#0f1117; border:1px solid #334155; border-radius:8px; padding:10px;
}
.motor-cell.fault {
  border-color:#ef4444; background:#1c0a0a;
}
.motor-name {
  font-size:.7rem; font-weight:600; text-transform:uppercase;
  letter-spacing:.06em; color:#64748b; margin-bottom:6px;
}
.motor-speed-wrap {
  height:8px; background:#1e293b; border-radius:4px;
  position:relative; overflow:hidden; margin-bottom:6px;
}
.motor-speed-bar {
  position:absolute; top:0; height:100%; border-radius:4px;
  transition: left .1s, width .1s, background .1s;
}
.motor-speed-center {
  position:absolute; left:50%; top:0; width:2px; height:100%;
  background:#334155; margin-left:-1px;
}
.motor-meta {
  display:flex; justify-content:space-between; align-items:center;
  font-size:.78rem;
}
.motor-speed-val { color:#f1f5f9; font-variant-numeric:tabular-nums; }
.motor-current   { color:#94a3b8; font-variant-numeric:tabular-nums; }
.motor-fault-tag {
  font-size:.68rem; font-weight:700; color:#ef4444;
  background:#3f0000; border-radius:4px; padding:1px 5px;
}

/* Responsive: stack on narrow screens */
@media (max-width: 560px) {
  .top-row    { flex-direction: column; align-items: center; }
  .cards-row  { flex-direction: column; }
}
</style>
</head>
<body>
<h1>Bearing &amp; Pivot Test</h1>

<div class="top-row">

  <div class="compass-col">
    <div class="compass-wrap">
      <div class="compass-ring"></div>
      <div class="compass-n">N</div>
      <div class="compass-target" id="targetNeedle" style="display:none"></div>
      <div class="compass-needle" id="needle"></div>
      <div class="compass-center"></div>
    </div>
    <div class="heading-box">
      <div class="heading-val" id="hdgVal">---</div>
      <div class="heading-label">current heading</div>
    </div>
  </div>

  <!-- Motor panel -->
  <div class="card motor-panel" id="motorPanel">
    <div class="card-title">Motors &amp; Faults</div>
    <div class="motor-header" id="motorHeader">—</div>
    <div class="motor-grid">
      <div class="motor-cell" id="mFL"></div>
      <div class="motor-cell" id="mFR"></div>
      <div class="motor-cell" id="mRL"></div>
      <div class="motor-cell" id="mRR"></div>
    </div>
  </div>

</div><!-- top-row -->

<div class="cards-row">

  <!-- Forward / bearing hold -->
  <div class="card">
    <div class="card-title">Forward — Yukon bearing hold</div>
    <div class="field">
      <label>Target bearing (0–359°)</label>
      <input type="number" id="fBearing" min="0" max="359" value="0" step="1">
    </div>
    <div class="two-col">
      <div>
        <label>Power %</label>
        <input type="number" id="fPower" min="5" max="100" value="50" step="5">
      </div>
      <div>
        <label>Run time (s)</label>
        <input type="number" id="fTime" min="0" max="30" value="3" step="0.5">
      </div>
    </div>
    <div class="btn-row">
      <button class="btn btn-go"   onclick="doForward()">Go</button>
      <button class="btn btn-stop" onclick="doStop()">Stop</button>
    </div>
    <div class="err-bar-wrap">
      <div class="err-center"></div>
      <div class="err-bar" id="fErrBar" style="display:none"></div>
    </div>
    <div class="err-label" id="fErrLabel"></div>
    <div class="status" id="fStatus"></div>
  </div>

  <!-- Pivot / ALIGNING -->
  <div class="card">
    <div class="card-title">Pivot — Pi differential steer</div>
    <div class="field">
      <label>Target heading (0–359°)</label>
      <input type="number" id="pTarget" min="0" max="359" value="0" step="1">
    </div>
    <div class="two-col">
      <div>
        <label>Max speed %</label>
        <input type="number" id="pSpeed" min="5" max="100" value="40" step="5">
      </div>
      <div>
        <label>Fwd speed %</label>
        <input type="number" id="pFwd" min="0" max="30" value="0" step="5">
      </div>
    </div>
    <div class="btn-row">
      <button class="btn btn-go"   onclick="doPivot()">Go</button>
      <button class="btn btn-stop" onclick="doStop()">Stop</button>
    </div>
    <div class="err-bar-wrap">
      <div class="err-center"></div>
      <div class="err-bar" id="errBar" style="display:none"></div>
    </div>
    <div class="err-label" id="errLabel"></div>
    <div class="status" id="pStatus"></div>
  </div>

</div><!-- cards-row -->

<script>
const needle       = document.getElementById('needle');
const targetNeedle = document.getElementById('targetNeedle');
const hdgVal       = document.getElementById('hdgVal');
const fStatus      = document.getElementById('fStatus');
const pStatus      = document.getElementById('pStatus');
const errBar       = document.getElementById('errBar');
const errLabel     = document.getElementById('errLabel');
const fErrBar      = document.getElementById('fErrBar');
const fErrLabel    = document.getElementById('fErrLabel');
let lastTarget     = null;

function applyErrBar(barEl, labelEl, err, extra) {
  if (err === null) { barEl.style.display='none'; labelEl.textContent=''; return; }
  const pct = Math.min(50, Math.abs(err) / 90 * 50);
  barEl.style.display = '';
  if (err >= 0) { barEl.style.left='50%'; barEl.style.width=pct+'%'; }
  else          { barEl.style.left=(50-pct)+'%'; barEl.style.width=pct+'%'; }
  barEl.style.background = Math.abs(err) < 3 ? '#22c55e' : '#f59e0b';
  const dir = err > 0.5 ? '→ right' : err < -0.5 ? '← left' : 'on target';
  labelEl.textContent = `error ${err>=0?'+':''}${err.toFixed(1)}°  ${dir}${extra||''}`;
}

function setErrBar(err)          { applyErrBar(errBar,  errLabel,  err, null); }
function setFwdErrBar(err, corr) {
  const extra = corr !== null ? `  |  correction ${corr>=0?'+':''}${corr}%` : '';
  applyErrBar(fErrBar, fErrLabel, err, extra);
}

function renderMotorCell(el, label, m) {
  if (!m) { el.innerHTML = ''; return; }
  const spd    = m.speed;          // -100..+100
  const pct    = Math.min(100, Math.abs(spd) / 2);   // 0..50 % of bar half
  const center = 50;
  let barLeft, barWidth, barColor;
  if (spd >= 0) {
    barLeft  = center; barWidth = pct;
    barColor = spd > 5 ? '#22c55e' : '#334155';
  } else {
    barLeft  = center - pct; barWidth = pct;
    barColor = '#f59e0b';
  }
  const faultTag = m.fault ? '<span class="motor-fault-tag">FAULT</span>' : '';
  el.className = 'motor-cell' + (m.fault ? ' fault' : '');
  el.innerHTML = `
    <div class="motor-name">${label}${faultTag}</div>
    <div class="motor-speed-wrap">
      <div class="motor-speed-center"></div>
      <div class="motor-speed-bar" style="left:${barLeft}%;width:${barWidth}%;background:${barColor}"></div>
    </div>
    <div class="motor-meta">
      <span class="motor-speed-val">${spd > 0 ? '+' : ''}${spd}%</span>
      <span class="motor-current">${m.current.toFixed(2)} A</span>
    </div>`;
}

function applyMotors(m) {
  if (!m) return;
  const src = m.actual ? 'actual' : 'estimated';
  document.getElementById('motorHeader').textContent =
    `${m.voltage.toFixed(1)} V  ·  ${m.current.toFixed(2)} A  ·  ${src}`;
  renderMotorCell(document.getElementById('mFL'), 'FL front-left',  m.fl);
  renderMotorCell(document.getElementById('mFR'), 'FR front-right', m.fr);
  renderMotorCell(document.getElementById('mRL'), 'RL rear-left',   m.rl);
  renderMotorCell(document.getElementById('mRR'), 'RR rear-right',  m.rr);
}

function applyState(d) {
  if (d.heading !== null) {
    hdgVal.textContent = d.heading.toFixed(1)+'°';
    needle.style.transform = `rotate(${d.heading}deg)`;
  } else {
    hdgVal.textContent = '---';
  }
  applyMotors(d.motors);
  if (lastTarget !== null) {
    targetNeedle.style.display = '';
    targetNeedle.style.transform = `rotate(${lastTarget}deg)`;
  }
  if (d.running) {
    fStatus.textContent = 'Running'; fStatus.className = 'status running';
    setFwdErrBar(d.fwd_err, d.fwd_corr);
  } else {
    setFwdErrBar(null, null);
    if (!d.pivot && fStatus.className.includes('running')) {
      fStatus.textContent='Stopped'; fStatus.className='status';
    }
  }
  if (d.pivot) {
    setErrBar(d.pivot_err);
    const aligned = d.pivot_err !== null && Math.abs(d.pivot_err) < 3;
    pStatus.textContent = aligned ? 'Aligned ✓' : 'Pivoting…';
    pStatus.className   = aligned ? 'status running' : 'status pivot';
  } else {
    if (pStatus.className.includes('pivot') || pStatus.textContent==='Pivoting…') {
      pStatus.textContent='Stopped'; pStatus.className='status'; setErrBar(null);
    }
  }
}

function connectSSE() {
  const es = new EventSource('/stream');
  es.onmessage = e => { try { applyState(JSON.parse(e.data)); } catch(_){} };
  es.onerror   = () => {
    hdgVal.textContent='---';
    fStatus.textContent='Connection lost — retrying…'; fStatus.className='status';
    es.close(); setTimeout(connectSSE, 2000);
  };
}
connectSSE();

function doForward() {
  const bearing  = parseFloat(document.getElementById('fBearing').value)||0;
  const power    = parseFloat(document.getElementById('fPower').value)||50;
  const run_time = parseFloat(document.getElementById('fTime').value)||0;
  lastTarget = bearing;
  fStatus.textContent='Starting…'; fStatus.className='status running';
  targetNeedle.style.display='';
  targetNeedle.style.transform=`rotate(${bearing}deg)`;
  fetch('/start',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({bearing,power,run_time})});
}

function doPivot() {
  const target    = parseFloat(document.getElementById('pTarget').value)||0;
  const max_speed = (parseFloat(document.getElementById('pSpeed').value)||40)/100;
  const fwd_speed = (parseFloat(document.getElementById('pFwd').value)||0)/100;
  lastTarget = target;
  pStatus.textContent='Starting…'; pStatus.className='status pivot';
  targetNeedle.style.display='';
  targetNeedle.style.transform=`rotate(${target}deg)`;
  fetch('/pivot',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({target,max_speed,fwd_speed})});
}

function doStop() {
  fStatus.textContent='Stopping…'; fStatus.className='status';
  pStatus.textContent='Stopping…'; pStatus.className='status';
  setErrBar(null);
  fetch('/stop',{method:'POST'});
}
</script>
</body>
</html>
"""

# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == '__main__':
    ip = _local_ip()
    print(f"Bearing test UI → http://{ip}:{args.port}")
    try:
        app.run(host='0.0.0.0', port=args.port, threaded=True)
    finally:
        _tool_running = False
        _stop_run()
        _stop_pivot()
        yukon.close()
