#!/usr/bin/env python3
"""
yukon_sim_web.py — Browser-based GUI for the Yukon serial simulator.

Runs yukon_sim.py's PTY server in the background and serves a web dashboard
on port 5002 (by default).  Open http://<pi-ip>:5002/ on any device.

Features
--------
  Compass rose    : SVG compass with live heading needle + bearing-hold target
  Motor bars      : left / right speed bars
  Voltage slider  : simulate battery voltage (8–15 V)
  Current slider  : simulate motor current draw (0–5 A)
  IMU toggle      : enable / disable simulated IMU
  Fault buttons   : inject left / right motor faults
  State stream    : Server-Sent Events at 10 Hz

Usage
-----
  python3 yukon_sim_web.py          # 0.0.0.0:5002
  python3 yukon_sim_web.py --port 8090

Dependencies
------------
  pip install flask
"""

import json
import math
import os
import pty
import sys
import threading
import time
import tty
import argparse

from flask import Flask, Response, request, jsonify

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import yukon_sim as _sim

app       = Flask(__name__)
_yukon_path = ""


# ── State serialiser ──────────────────────────────────────────────────────────

def _get_state():
    with _sim._lock:
        s = dict(_sim._state)
    return {
        'left_speed'    : _sim._decode_speed(s.get('left_byte')),
        'right_speed'   : _sim._decode_speed(s.get('right_byte')),
        'led_a'         : s.get('led_a', False),
        'led_b'         : s.get('led_b', False),
        'cmds_rx'       : s.get('cmds_rx', 0),
        'imu_present'   : s.get('imu_present', True),
        'imu_heading'   : round(s.get('imu_heading', 0.0), 1),
        'bearing_target': s.get('bearing_target'),
        'fault_l'       : s.get('fault_l', False),
        'fault_r'       : s.get('fault_r', False),
        'voltage'       : round(_sim.SIM_VOLTAGE, 2),
        'current'       : round(_sim.SIM_CURRENT, 3),
        'yukon_port'    : _yukon_path,
    }


# ── Flask routes ──────────────────────────────────────────────────────────────

@app.route('/')
def index():
    return Response(_HTML, mimetype='text/html')


@app.route('/api/state')
def api_state():
    def _gen():
        while True:
            _sim._tick_imu()
            yield f"data: {json.dumps(_get_state())}\n\n"
            time.sleep(0.1)
    return Response(_gen(), mimetype='text/event-stream',
                    headers={'Cache-Control': 'no-cache',
                             'X-Accel-Buffering': 'no'})


@app.route('/api/cmd', methods=['POST'])
def api_cmd():
    body = request.json or {}
    cmd  = body.get('cmd', '')
    val  = body.get('value')

    if cmd == 'set_heading':
        with _sim._lock:
            if _sim._state['imu_present']:
                _sim._state['imu_heading'] = float(val) % 360.0

    elif cmd == 'nudge_heading':
        with _sim._lock:
            if _sim._state['imu_present']:
                _sim._state['imu_heading'] = (
                    _sim._state['imu_heading'] + float(val)
                ) % 360.0

    elif cmd == 'toggle_imu':
        with _sim._lock:
            _sim._state['imu_present'] = not _sim._state['imu_present']
            if not _sim._state['imu_present']:
                _sim._state['bearing_target'] = None

    elif cmd == 'set_voltage':
        _sim.SIM_VOLTAGE = max(8.0, min(15.0, float(val)))

    elif cmd == 'set_current':
        _sim.SIM_CURRENT = max(0.0, min(5.0, float(val)))

    elif cmd == 'fault_l':
        with _sim._lock:
            _sim._state['fault_l'] = not _sim._state.get('fault_l', False)

    elif cmd == 'fault_r':
        with _sim._lock:
            _sim._state['fault_r'] = not _sim._state.get('fault_r', False)

    elif cmd == 'clear_faults':
        with _sim._lock:
            _sim._state['fault_l'] = False
            _sim._state['fault_r'] = False

    else:
        return jsonify({'ok': False, 'error': f'Unknown cmd: {cmd}'}), 400

    return jsonify({'ok': True})


# ── Embedded HTML ─────────────────────────────────────────────────────────────

_HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Yukon Simulator</title>
<style>
*,*::before,*::after{box-sizing:border-box;margin:0;padding:0}
:root{
  --bg:#12121e;--panel:#1c1c2d;--border:#3c3c5a;
  --white:#e6e6f0;--gray:#82829a;
  --green:#3cdc50;--yellow:#f0c828;--orange:#f08c28;
  --red:#dc3c3c;--cyan:#3cc8dc;
}
body{background:var(--bg);color:var(--white);font-family:'Courier New',monospace;
  min-height:100vh;padding:10px}
h1{font-size:1.3rem;color:var(--cyan);margin-bottom:4px}
.subtitle{font-size:.75rem;color:var(--gray);margin-bottom:12px}
.grid{display:grid;grid-template-columns:1fr 1fr 1fr;gap:12px;max-width:960px;margin:0 auto}
.panel{background:var(--panel);border:1px solid var(--border);
  border-radius:10px;padding:14px}
.panel h2{font-size:.7rem;color:var(--gray);text-transform:uppercase;
  letter-spacing:1px;margin-bottom:10px}

/* Compass */
#compass-wrap{display:flex;justify-content:center;margin-bottom:8px;cursor:crosshair}
svg text{font-family:'Courier New',monospace}
#hdg-label{text-align:center;font-size:1.1rem;font-weight:bold;color:var(--white)}
#hold-label{text-align:center;font-size:.75rem;margin-top:4px}

/* Motor bars */
.motor-row{display:flex;align-items:center;gap:8px;margin-bottom:10px}
.motor-lbl{width:40px;font-size:.8rem;color:var(--gray)}
.motor-track{flex:1;height:32px;background:var(--bg);border-radius:5px;
  position:relative;overflow:hidden}
.motor-mid{position:absolute;left:50%;top:3px;bottom:3px;
  width:1px;background:var(--border)}
.motor-fill{position:absolute;top:3px;bottom:3px;
  border-radius:4px;transition:all .06s linear}
.motor-val{position:absolute;right:6px;top:50%;transform:translateY(-50%);
  font-size:.75rem}

/* Sliders */
.slider-row{margin-bottom:16px}
.slider-row label{font-size:.75rem;color:var(--gray);display:block;margin-bottom:4px}
input[type=range]{width:100%;accent-color:var(--cyan)}
.slider-val{font-size:.9rem;font-weight:bold;margin-top:2px}

/* Buttons */
.btn-row{display:flex;gap:8px;margin-bottom:10px;flex-wrap:wrap}
button{font-family:inherit;font-size:.8rem;padding:8px 14px;
  border-radius:6px;border:1px solid var(--border);
  background:var(--panel);color:var(--white);cursor:pointer}
button:active{opacity:.8}
button.active{background:#1a2a1a}
.btn-green{border-color:var(--green);color:var(--green)}
.btn-green.active{background:#0a2a0a}
.btn-red{border-color:var(--red);color:var(--red)}
.btn-red.active{background:#2a0a0a}
.btn-cyan{border-color:var(--cyan);color:var(--cyan)}

/* Status badges */
.badges{display:flex;flex-wrap:wrap;gap:6px;margin-top:8px}
.badge{font-size:.65rem;padding:3px 8px;border-radius:20px;
  border:1px solid var(--border);color:var(--gray)}
.badge.ok{border-color:var(--green);color:var(--green)}
.badge.warn{border-color:var(--yellow);color:var(--yellow)}
.badge.err{border-color:var(--red);color:var(--red)}
.badge.info{border-color:var(--cyan);color:var(--cyan)}

/* Connection */
#conn{position:fixed;top:10px;right:14px;font-size:.7rem;color:var(--gray)}

@media(max-width:680px){.grid{grid-template-columns:1fr}}
</style>
</head>
<body>
<div style="max-width:960px;margin:0 auto">
  <h1>🤖 Yukon Simulator</h1>
  <div class="subtitle" id="port-label">PTY: --</div>
</div>
<span id="conn">⚫ Connecting…</span>

<div class="grid">

  <!-- Compass -->
  <div class="panel">
    <h2>IMU Heading</h2>
    <div id="compass-wrap">
      <svg id="compass-svg" width="200" height="200" viewBox="-100 -100 200 200">
        <!-- Range rings -->
        <circle cx="0" cy="0" r="90" fill="none" stroke="#3c3c5a" stroke-width="1.5"/>
        <!-- Cardinal labels -->
        <text x="0"   y="-72" text-anchor="middle" fill="#82829a" font-size="11">N</text>
        <text x="0"   y="80"  text-anchor="middle" fill="#82829a" font-size="11">S</text>
        <text x="76"  y="4"   text-anchor="middle" fill="#82829a" font-size="11">E</text>
        <text x="-76" y="4"   text-anchor="middle" fill="#82829a" font-size="11">W</text>
        <!-- Cardinal ticks -->
        <line x1="0"  y1="-90" x2="0"  y2="-78" stroke="#82829a" stroke-width="2"/>
        <line x1="0"  y1="90"  x2="0"  y2="78"  stroke="#82829a" stroke-width="2"/>
        <line x1="90" y1="0"   x2="78" y2="0"   stroke="#82829a" stroke-width="2"/>
        <line x1="-90" y1="0"  x2="-78" y2="0"  stroke="#82829a" stroke-width="2"/>
        <!-- Target needle (cyan dashed) -->
        <line id="target-needle" x1="0" y1="0" x2="0" y2="-72"
              stroke="#3cc8dc" stroke-width="2" stroke-dasharray="6,4" opacity="0"/>
        <!-- Heading needle -->
        <line id="hdg-needle" x1="0" y1="12" x2="0" y2="-74"
              stroke="white" stroke-width="3" stroke-linecap="round"/>
        <line id="hdg-tail"   x1="0" y1="12" x2="0" y2="28"
              stroke="#dc3c3c" stroke-width="3" stroke-linecap="round"/>
        <circle cx="0" cy="0" r="5" fill="white"/>
        <!-- IMU absent overlay -->
        <text id="imu-absent-text" x="0" y="5" text-anchor="middle"
              fill="#dc3c3c" font-size="14" font-weight="bold" opacity="0">IMU ABSENT</text>
      </svg>
    </div>
    <div id="hdg-label">---</div>
    <div id="hold-label" style="color:var(--gray)">Hold: off</div>
    <div style="display:flex;gap:8px;margin-top:10px">
      <button class="btn-cyan" onclick="nudge(-5)">← -5°</button>
      <button class="btn-cyan" style="flex:1" id="imu-btn" onclick="toggleImu()">IMU: ON</button>
      <button class="btn-cyan" onclick="nudge(+5)">+5° →</button>
    </div>
  </div>

  <!-- Motors + Status -->
  <div class="panel">
    <h2>Drive</h2>
    <div class="motor-row">
      <span class="motor-lbl">Left</span>
      <div class="motor-track">
        <div class="motor-mid"></div>
        <div class="motor-fill" id="fill-l"></div>
        <span class="motor-val" id="val-l">---</span>
      </div>
    </div>
    <div class="motor-row">
      <span class="motor-lbl">Right</span>
      <div class="motor-track">
        <div class="motor-mid"></div>
        <div class="motor-fill" id="fill-r"></div>
        <span class="motor-val" id="val-r">---</span>
      </div>
    </div>

    <h2 style="margin-top:14px">Status</h2>
    <div class="badges">
      <span class="badge" id="bdg-leda">LED A: --</span>
      <span class="badge" id="bdg-ledb">LED B: --</span>
      <span class="badge" id="bdg-cmds">Cmds: 0</span>
      <span class="badge" id="bdg-fl">Fault L: --</span>
      <span class="badge" id="bdg-fr">Fault R: --</span>
      <span class="badge" id="bdg-hold">Hold: off</span>
    </div>

    <h2 style="margin-top:14px">Sim Values</h2>
    <div class="badges">
      <span class="badge info" id="bdg-volt">V: --</span>
      <span class="badge info" id="bdg-curr">A: --</span>
    </div>
  </div>

  <!-- Controls -->
  <div class="panel">
    <h2>Controls</h2>

    <div class="slider-row">
      <label>Voltage</label>
      <input type="range" id="volt-slider" min="8" max="15" step="0.1" value="12"
             oninput="sendVal('set_voltage', this.value)">
      <div class="slider-val" id="volt-val">12.0 V</div>
    </div>

    <div class="slider-row">
      <label>Current</label>
      <input type="range" id="curr-slider" min="0" max="5" step="0.05" value="0.5"
             oninput="sendVal('set_current', this.value)">
      <div class="slider-val" id="curr-val">0.50 A</div>
    </div>

    <h2 style="margin-top:4px">Fault Injection</h2>
    <div class="btn-row">
      <button class="btn-red" id="btn-fl" onclick="sendCmd('fault_l')">Fault L</button>
      <button class="btn-red" id="btn-fr" onclick="sendCmd('fault_r')">Fault R</button>
      <button onclick="sendCmd('clear_faults')">Clear</button>
    </div>

    <h2 style="margin-top:8px">Heading</h2>
    <p style="font-size:.7rem;color:var(--gray);margin-bottom:8px">
      Click compass rose to set heading directly
    </p>
    <div class="btn-row">
      <button class="btn-cyan" onclick="nudge(-45)">-45°</button>
      <button class="btn-cyan" onclick="nudge(-10)">-10°</button>
      <button class="btn-cyan" onclick="nudge(+10)">+10°</button>
      <button class="btn-cyan" onclick="nudge(+45)">+45°</button>
    </div>
  </div>

</div><!-- /grid -->

<script>
"use strict";
const el = id => document.getElementById(id);
const C  = {green:'#3cdc50',yellow:'#f0c828',orange:'#f08c28',
             red:'#dc3c3c',cyan:'#3cc8dc',gray:'#82829a'};

// ── Compass click ─────────────────────────────────────────────────────────────
el('compass-svg').addEventListener('click', e => {
  const svg  = el('compass-svg');
  const rect = svg.getBoundingClientRect();
  const cx   = rect.left + rect.width/2;
  const cy   = rect.top  + rect.height/2;
  const dx   = e.clientX - cx, dy = e.clientY - cy;
  const angle = ((Math.atan2(dy, dx) * 180/Math.PI) + 90 + 360) % 360;
  sendVal('set_heading', angle.toFixed(1));
});

// ── Needle update ─────────────────────────────────────────────────────────────
function setNeedle(id, degrees) {
  el(id).setAttribute('transform', `rotate(${degrees})`);
}

// ── Motor bar ─────────────────────────────────────────────────────────────────
function motorBar(fillId, valId, v) {
  const fill = el(fillId);
  if (v == null) {
    fill.style.width = '0'; el(valId).textContent = '---';
    el(valId).style.color = C.gray; return;
  }
  const col = v > 0 ? C.green : v < 0 ? C.orange : null;
  if (v >= 0) { fill.style.left='50%';  fill.style.width=(v*50)+'%'; }
  else        { fill.style.left=(50+v*50)+'%'; fill.style.width=(-v*50)+'%'; }
  fill.style.backgroundColor = col || 'transparent';
  el(valId).textContent = (v>=0?'+':'')+v.toFixed(2);
  el(valId).style.color = col || C.gray;
}

// ── Badge ─────────────────────────────────────────────────────────────────────
function setBadge(id, text, cls) {
  const b = el(id); b.textContent = text;
  b.className = 'badge' + (cls ? ' '+cls : '');
}

// ── Apply state ───────────────────────────────────────────────────────────────
function applyState(s) {
  el('port-label').textContent = 'PTY: ' + s.yukon_port;

  // Compass
  const absent = !s.imu_present;
  el('imu-absent-text').setAttribute('opacity', absent ? '1' : '0');
  el('hdg-needle').setAttribute('opacity', absent ? '0' : '1');
  el('hdg-tail').setAttribute('opacity',   absent ? '0' : '1');

  if (!absent) {
    setNeedle('hdg-needle', s.imu_heading);
    setNeedle('hdg-tail',   s.imu_heading);
    el('hdg-label').textContent = s.imu_heading.toFixed(1) + '°';
    el('hdg-label').style.color = C.white;
  } else {
    el('hdg-label').textContent = '---';
    el('hdg-label').style.color = C.red;
  }

  // Bearing hold target needle
  const tgt = s.bearing_target;
  if (tgt != null && !absent) {
    el('target-needle').setAttribute('opacity', '1');
    setNeedle('target-needle', tgt);
    const err = ((tgt - s.imu_heading + 180) % 360) - 180;
    el('hold-label').textContent = `Hold: ${tgt.toFixed(1)}°  err: ${err >= 0 ? '+' : ''}${err.toFixed(1)}°`;
    el('hold-label').style.color = Math.abs(err) < 5 ? C.green : C.yellow;
    setBadge('bdg-hold', `Hold: ${tgt.toFixed(0)}°`, 'info');
  } else {
    el('target-needle').setAttribute('opacity', '0');
    el('hold-label').textContent = 'Hold: off';
    el('hold-label').style.color = C.gray;
    setBadge('bdg-hold', 'Hold: off', '');
  }

  // IMU button
  const imuBtn = el('imu-btn');
  imuBtn.textContent = `IMU: ${s.imu_present ? 'PRESENT' : 'ABSENT'}`;
  imuBtn.style.borderColor = s.imu_present ? C.green : C.red;
  imuBtn.style.color       = s.imu_present ? C.green : C.red;

  // Motors
  motorBar('fill-l', 'val-l', s.left_speed);
  motorBar('fill-r', 'val-r', s.right_speed);

  // LED badges
  setBadge('bdg-leda', `LED A: ${s.led_a?'ON':'OFF'}`, s.led_a?'ok':'');
  setBadge('bdg-ledb', `LED B: ${s.led_b?'ON':'OFF'}`, s.led_b?'ok':'');
  setBadge('bdg-cmds', `Cmds: ${s.cmds_rx}`, 'info');

  // Fault badges
  setBadge('bdg-fl', `Fault L: ${s.fault_l?'YES':'no'}`, s.fault_l?'err':'');
  setBadge('bdg-fr', `Fault R: ${s.fault_r?'YES':'no'}`, s.fault_r?'err':'');
  el('btn-fl').className = 'btn-red' + (s.fault_l ? ' active' : '');
  el('btn-fr').className = 'btn-red' + (s.fault_r ? ' active' : '');

  // Sim values
  setBadge('bdg-volt', `${s.voltage.toFixed(1)} V`, 'info');
  setBadge('bdg-curr', `${s.current.toFixed(2)} A`, 'info');

  // Sync sliders (only if not being dragged)
  if (document.activeElement !== el('volt-slider')) {
    el('volt-slider').value = s.voltage;
    el('volt-val').textContent = s.voltage.toFixed(1) + ' V';
  }
  if (document.activeElement !== el('curr-slider')) {
    el('curr-slider').value = s.current;
    el('curr-val').textContent = s.current.toFixed(2) + ' A';
  }
}

// ── Commands ──────────────────────────────────────────────────────────────────
async function sendCmd(cmd, value) {
  const body = {cmd};
  if (value !== undefined) body.value = value;
  try {
    await fetch('/api/cmd', {method:'POST',
      headers:{'Content-Type':'application/json'},
      body: JSON.stringify(body)});
  } catch(e) { console.error('cmd failed:', e); }
}
function sendVal(cmd, value) { sendCmd(cmd, parseFloat(value)); }
function nudge(deg)          { sendCmd('nudge_heading', deg); }
function toggleImu()         { sendCmd('toggle_imu'); }

// Slider display
el('volt-slider').oninput = function() {
  el('volt-val').textContent = parseFloat(this.value).toFixed(1) + ' V';
  sendVal('set_voltage', this.value);
};
el('curr-slider').oninput = function() {
  el('curr-val').textContent = parseFloat(this.value).toFixed(2) + ' A';
  sendVal('set_current', this.value);
};

// ── SSE ───────────────────────────────────────────────────────────────────────
let evtSrc = null;
function connect() {
  if (evtSrc) evtSrc.close();
  const conn = el('conn');
  conn.textContent = '🟡 Connecting…';
  evtSrc = new EventSource('/api/state');
  evtSrc.onopen    = () => { conn.textContent = '🟢 Live'; };
  evtSrc.onmessage = e => {
    try { applyState(JSON.parse(e.data)); } catch(err) {}
  };
  evtSrc.onerror = () => {
    conn.textContent = '🔴 Reconnecting…';
    evtSrc.close(); setTimeout(connect, 3000);
  };
}
connect();
</script>
</body>
</html>"""


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    global _yukon_path

    parser = argparse.ArgumentParser(description="Yukon simulator web GUI")
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', default=5002, type=int)
    args = parser.parse_args()

    # Create PTY
    yukon_master, yukon_slave = pty.openpty()
    tty.setraw(yukon_slave)
    _yukon_path = os.ttyname(yukon_slave)

    print(f"Yukon PTY  : {_yukon_path}", file=sys.stderr)
    print(f"Connect with --port {_yukon_path}", file=sys.stderr)
    print(f"Web GUI    : http://{args.host}:{args.port}/", file=sys.stderr)

    # Initialise sim state
    with _sim._lock:
        _sim._state['last_imu_tick'] = time.monotonic()
        _sim._state['fault_l']       = False
        _sim._state['fault_r']       = False

    # Launch Yukon protocol server thread
    t_srv = threading.Thread(target=_sim.yukon_server,
                             args=(yukon_master,), daemon=True)
    t_srv.start()

    try:
        import logging
        logging.getLogger('werkzeug').setLevel(logging.WARNING)
        app.run(host=args.host, port=args.port,
                threaded=True, use_reloader=False)
    finally:
        with _sim._lock:
            _sim._state['running'] = False
        os.close(yukon_master)
        os.close(yukon_slave)


if __name__ == '__main__':
    main()
