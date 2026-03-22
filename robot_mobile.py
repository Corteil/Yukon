#!/usr/bin/env python3
"""
robot_mobile.py — Mobile-optimised web dashboard for the Yukon robot.

Same Robot backend as robot_web.py; touch-friendly tab UI designed for phones.

Tabs
----
  Drive   — camera stream, bearing overlay, motor bars, mode/speed, ArUco, nav state
  Telem   — voltage, current, temperatures, IMU heading compass, faults
  GPS     — fix, position, error, satellites, bookmark
  System  — CPU, temp, memory, disk, lidar plot

ESTOP button is always visible at the bottom.
No-motors banner shown when --no-motors is active.

Usage
-----
  python3 robot_mobile.py                  # 0.0.0.0:5001
  python3 robot_mobile.py --port 8080
  python3 robot_mobile.py --no-motors      # bench-test mode
  python3 robot_mobile.py --config robot.ini
"""

import argparse
import configparser
import io
import json
import logging
import os
import sys
import time

from flask import Flask, Response, request, jsonify

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from robot_daemon import Robot, RobotMode, AutoType, setup_logging

log = logging.getLogger(__name__)


# ── JPEG encoder ──────────────────────────────────────────────────────────────

def _make_jpeg_encoder():
    try:
        import cv2
        def _enc(frame, quality=80):
            _, buf = cv2.imencode('.jpg', frame[:, :, ::-1],
                                  [cv2.IMWRITE_JPEG_QUALITY, quality])
            return buf.tobytes()
        return _enc
    except ImportError:
        pass
    try:
        from PIL import Image as _PIL
        def _enc(frame, quality=80):
            buf = io.BytesIO()
            _PIL.fromarray(frame).save(buf, format='JPEG', quality=quality)
            return buf.getvalue()
        return _enc
    except ImportError:
        pass
    return None


_jpeg_encode = None


# ── State serialiser (mirrors robot_web.py) ────────────────────────────────────

def _serialise(state, cam_rotation=0, aruco_enabled=False,
               aruco_state=None, nav_bearing_err=None):
    g, t, d, li, s = state.gps, state.telemetry, state.drive, state.lidar, state.system
    aruco_info = None
    if aruco_enabled and aruco_state is not None:
        tags_detail = []
        for tag in aruco_state.tags.values():
            tags_detail.append({
                'id':       tag.id,
                'cx':       tag.center_x,
                'cy':       tag.center_y,
                'area':     tag.area,
                'bearing':  tag.bearing,
                'distance': tag.distance,
            })
        gates_detail = []
        for gate in aruco_state.gates.values():
            gates_detail.append({
                'gate_id':    gate.gate_id,
                'centre_x':  gate.centre_x,
                'centre_y':  gate.centre_y,
                'bearing':   gate.bearing,
                'distance':  gate.distance,
                'correct_dir': gate.correct_dir,
            })
        aruco_info = {
            'tag_count':  len(aruco_state.tags),
            'gate_count': len(aruco_state.gates),
            'fps':        aruco_state.fps,
            'tags':       tags_detail,
            'gates':      gates_detail,
        }
    return {
        'mode':          state.mode.name,
        'auto_type':     state.auto_type.label,
        'speed_scale':   state.speed_scale,
        'rc_active':     state.rc_active,
        'camera_ok':     state.camera_ok,
        'lidar_ok':      state.lidar_ok,
        'gps_ok':        state.gps_ok,
        'aruco_ok':      state.aruco_ok,
        'aruco_enabled': aruco_enabled,
        'cam_rotation':  cam_rotation,
        'gps_logging':   state.gps_logging,
        'no_motors':     state.no_motors,
        'nav_state':     state.nav_state,
        'nav_gate':      state.nav_gate,
        'nav_bearing_err': nav_bearing_err,
        'nav_wp':        state.nav_wp,
        'nav_wp_dist':   state.nav_wp_dist,
        'nav_wp_bear':   state.nav_wp_bear,
        'aruco':         aruco_info,
        'drive': {
            'left':  round(d.left,  3),
            'right': round(d.right, 3),
        },
        'telemetry': {
            'voltage':     round(t.voltage,    2),
            'current':     round(t.current,    3),
            'board_temp':  round(t.board_temp, 1),
            'left_temp':   round(t.left_temp,  1),
            'right_temp':  round(t.right_temp, 1),
            'left_fault':  t.left_fault,
            'right_fault': t.right_fault,
            'heading':     round(t.heading, 1) if t.heading is not None else None,
        },
        'gps': {
            'latitude':         g.latitude,
            'longitude':        g.longitude,
            'altitude':         g.altitude,
            'fix_quality':      g.fix_quality,
            'fix_quality_name': g.fix_quality_name,
            'h_error_m':        g.h_error_m,
            'satellites':       g.satellites,
            'satellites_view':  g.satellites_view,
            'hdop':             g.hdop,
        },
        'lidar': {
            'angles':    li.angles,
            'distances': li.distances,
        },
        'system': {
            'cpu_percent':   s.cpu_percent,
            'cpu_temp_c':    s.cpu_temp_c,
            'cpu_freq_mhz':  s.cpu_freq_mhz,
            'mem_used_mb':   s.mem_used_mb,
            'mem_total_mb':  s.mem_total_mb,
            'mem_percent':   s.mem_percent,
            'disk_used_gb':  s.disk_used_gb,
            'disk_total_gb': s.disk_total_gb,
            'disk_percent':  s.disk_percent,
        },
    }


# ── Flask ──────────────────────────────────────────────────────────────────────

app       = Flask(__name__)
_robot    = None
_log_path = None


@app.route('/')
def index():
    return Response(_HTML, mimetype='text/html')


@app.route('/stream')
def camera_stream():
    def _gen():
        while True:
            frame = _robot.get_frame()
            if frame is not None and _jpeg_encode is not None:
                try:
                    data = _jpeg_encode(frame)
                    yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
                           + data + b'\r\n')
                except Exception:
                    pass
            time.sleep(0.05)
    return Response(_gen(),
                    mimetype='multipart/x-mixed-replace; boundary=frame',
                    headers={'Cache-Control': 'no-store'})


@app.route('/api/state')
def api_state():
    def _gen():
        while True:
            try:
                nav_bearing_err = None
                if _robot._navigator is not None:
                    nav_bearing_err = _robot._navigator.bearing_err
                elif _robot._gps_navigator is not None:
                    nav_bearing_err = _robot._gps_navigator.heading_err
                data = _serialise(
                    _robot.get_state(),
                    cam_rotation    = _robot.get_cam_rotation(),
                    aruco_enabled   = _robot.get_aruco_enabled(),
                    aruco_state     = _robot.get_aruco_state(),
                    nav_bearing_err = nav_bearing_err,
                )
                yield f"data: {json.dumps(data)}\n\n"
            except Exception as exc:
                yield f"data: {json.dumps({'error': str(exc)})}\n\n"
            time.sleep(0.1)
    return Response(_gen(),
                    mimetype='text/event-stream',
                    headers={'Cache-Control': 'no-cache',
                             'X-Accel-Buffering': 'no'})


@app.route('/api/cmd', methods=['POST'])
def api_cmd():
    cmd = (request.json or {}).get('cmd', '')
    if   cmd == 'estop':         _robot.estop()
    elif cmd == 'reset':         _robot.reset_estop()
    elif cmd == 'aruco_toggle':  _robot.toggle_aruco()
    elif cmd == 'gps_bookmark':  _robot.bookmark_gps()
    elif cmd == 'rotate_cw':
        _robot.set_cam_rotation((_robot.get_cam_rotation() + 90) % 360)
    elif cmd == 'rotate_ccw':
        _robot.set_cam_rotation((_robot.get_cam_rotation() - 90 + 360) % 360)
    else:
        return jsonify({'ok': False, 'error': f'Unknown command: {cmd}'}), 400
    return jsonify({'ok': True})


@app.route('/api/logs')
def api_logs():
    n = min(int(request.args.get('n', 200)), 500)
    lines = []
    if _log_path and os.path.exists(_log_path):
        try:
            with open(_log_path, 'r', errors='replace') as f:
                lines = f.readlines()[-n:]
        except OSError:
            pass
    return jsonify({'lines': [l.rstrip('\n') for l in lines]})


# ── Mobile HTML ────────────────────────────────────────────────────────────────

_HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
<meta name="mobile-web-app-capable" content="yes">
<meta name="apple-mobile-web-app-capable" content="yes">
<title>Yukon</title>
<style>
*, *::before, *::after { box-sizing: border-box; margin: 0; padding: 0; }
:root {
  --bg:     #0e0e1a;
  --panel:  #181828;
  --card:   #1e1e32;
  --border: #3a3a58;
  --white:  #e8e8f2;
  --gray:   #7878a0;
  --green:  #3cdc50;
  --yellow: #f0c828;
  --orange: #f08028;
  --red:    #dc3c3c;
  --cyan:   #3cc8dc;
  --blue:   #3c78dc;
  --radius: 12px;
  --tab-h:  56px;
  --hdr-h:  52px;
  --estop-h:68px;
}

html, body {
  height: 100%;
  background: var(--bg);
  color: var(--white);
  font-family: 'Courier New', monospace;
  overflow: hidden;
}

/* ── Header ── */
#header {
  position: fixed; top: 0; left: 0; right: 0;
  height: var(--hdr-h);
  background: var(--panel);
  border-bottom: 1px solid var(--border);
  display: flex; align-items: center; gap: 10px;
  padding: 0 12px;
  z-index: 100;
}
#h-title { font-size: 1rem; font-weight: bold; color: var(--cyan); letter-spacing: 1px; }
#h-mode  { font-size: 0.85rem; font-weight: bold; margin-left: 4px; }
#h-speed { font-size: 0.75rem; color: var(--gray); }
#h-conn  { margin-left: auto; font-size: 0.7rem; color: var(--gray); }

/* ── Tab bar ── */
#tabs {
  position: fixed; top: var(--hdr-h); left: 0; right: 0;
  height: var(--tab-h);
  background: var(--panel);
  border-bottom: 1px solid var(--border);
  display: flex;
  z-index: 100;
}
.tab {
  flex: 1; display: flex; flex-direction: column;
  align-items: center; justify-content: center;
  font-size: 0.65rem; color: var(--gray);
  cursor: pointer; gap: 2px;
  -webkit-tap-highlight-color: transparent;
  border-bottom: 2px solid transparent;
  transition: color 0.15s, border-color 0.15s;
}
.tab .icon { font-size: 1.3rem; }
.tab.active { color: var(--cyan); border-bottom-color: var(--cyan); }

/* ── Pages ── */
#pages {
  position: fixed;
  top: calc(var(--hdr-h) + var(--tab-h));
  bottom: var(--estop-h);
  left: 0; right: 0;
  overflow-y: auto;
  -webkit-overflow-scrolling: touch;
}
.page { display: none; padding: 10px; flex-direction: column; gap: 10px; }
.page.active { display: flex; }

/* ── Cards ── */
.card {
  background: var(--card);
  border: 1px solid var(--border);
  border-radius: var(--radius);
  padding: 10px 12px;
}
.card-title {
  font-size: 0.65rem; color: var(--gray);
  text-transform: uppercase; letter-spacing: 1px;
  margin-bottom: 8px;
}

/* ── Camera ── */
#cam-wrap {
  background: #000; border-radius: 8px; overflow: hidden;
  aspect-ratio: 4/3; position: relative;
}
#cam-img { width: 100%; height: 100%; object-fit: contain; display: block; }
#cam-no  { position: absolute; inset: 0; display: flex; align-items: center;
           justify-content: center; color: var(--gray); font-size: 0.85rem; }
#cam-btns {
  display: flex; gap: 8px; margin-top: 8px;
}

/* ── Buttons ── */
.btn {
  flex: 1; padding: 10px 6px; border-radius: 8px;
  border: 1px solid var(--border);
  background: var(--panel); color: var(--white);
  font-family: inherit; font-size: 0.8rem;
  cursor: pointer; text-align: center;
  -webkit-tap-highlight-color: transparent;
  transition: background 0.1s;
}
.btn:active  { background: #2a2a44; }
.btn.on      { border-color: var(--cyan); color: var(--cyan); }
.btn.danger  { border-color: var(--red); color: var(--red); }

/* ── Motor bars ── */
.motor-row { display: flex; align-items: center; gap: 8px; margin-bottom: 8px; }
.motor-row:last-child { margin-bottom: 0; }
.motor-lbl { font-size: 0.75rem; color: var(--gray); width: 14px; }
.motor-track {
  flex: 1; height: 36px; background: var(--bg);
  border-radius: 6px; position: relative; overflow: hidden;
}
.motor-mid {
  position: absolute; left: 50%; top: 4px; bottom: 4px;
  width: 1px; background: var(--border);
}
.motor-fill {
  position: absolute; top: 4px; bottom: 4px;
  border-radius: 4px; transition: all 0.06s linear;
}
.motor-val {
  position: absolute; right: 8px; top: 50%;
  transform: translateY(-50%); font-size: 0.75rem;
}

/* ── Stat grid ── */
.stat-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; }
.stat-grid.three { grid-template-columns: 1fr 1fr 1fr; }
.stat-item .lbl { font-size: 0.6rem; color: var(--gray); margin-bottom: 2px; }
.stat-item .val { font-size: 1rem; font-weight: bold; }

/* ── Progress bars ── */
.prog-row { display: flex; align-items: center; gap: 8px; margin-bottom: 8px; }
.prog-row:last-child { margin-bottom: 0; }
.prog-lbl  { font-size: 0.7rem; color: var(--gray); width: 34px; flex-shrink: 0; }
.prog-track {
  flex: 1; height: 18px; background: var(--bg);
  border-radius: 4px; overflow: hidden; position: relative;
}
.prog-fill {
  position: absolute; left: 0; top: 0; bottom: 0;
  border-radius: 4px; transition: width 0.3s;
}
.prog-val {
  position: absolute; right: 4px; top: 50%;
  transform: translateY(-50%); font-size: 0.65rem;
}

/* ── Lidar ── */
#lidar-canvas { width: 100%; border-radius: 8px; display: block; }

/* ── GPS coords ── */
.coord { font-size: 0.9rem; font-variant-numeric: tabular-nums; }

/* ── ESTOP bar ── */
#estop-bar {
  position: fixed; bottom: 0; left: 0; right: 0;
  height: var(--estop-h);
  background: var(--panel);
  border-top: 1px solid var(--border);
  display: flex; align-items: center; gap: 10px;
  padding: 8px 12px;
  z-index: 100;
}
#btn-estop {
  flex: 2; height: 100%;
  background: #3a0a0a; border: 2px solid var(--red); color: var(--red);
  border-radius: 10px; font-family: inherit; font-size: 1.1rem;
  font-weight: bold; cursor: pointer; letter-spacing: 2px;
  -webkit-tap-highlight-color: transparent;
}
#btn-estop:active { background: #5a1010; }
#btn-reset {
  flex: 1; height: 100%;
  background: var(--panel); border: 1px solid var(--green); color: var(--green);
  border-radius: 10px; font-family: inherit; font-size: 0.85rem;
  cursor: pointer;
  -webkit-tap-highlight-color: transparent;
}
#btn-reset:active { background: #0a2a0a; }

/* ── Status badges (footer of drive tab) ── */
#badges { display: flex; flex-wrap: wrap; gap: 6px; }
.badge {
  font-size: 0.65rem; padding: 3px 8px;
  border-radius: 20px; border: 1px solid var(--border);
  color: var(--gray);
}
.badge.ok   { border-color: var(--green);  color: var(--green);  }
.badge.warn { border-color: var(--yellow); color: var(--yellow); }
.badge.err  { border-color: var(--red);    color: var(--red);    }
.badge.info { border-color: var(--cyan);   color: var(--cyan);   }

/* ── No-motors banner ── */
#no-motors-banner {
  display: none;
  position: fixed; top: var(--hdr-h); left: 0; right: 0;
  background: #3a0000; border-bottom: 1px solid var(--red);
  z-index: 99; text-align: center;
  color: var(--red); font-size: 0.75rem; font-weight: bold;
  padding: 3px 0; letter-spacing: 1px;
}

/* ── Bearing overlay canvas ── */
#bearing-canvas {
  position: absolute; top: 0; left: 0;
  width: 100%; height: 100%;
  pointer-events: none; display: none;
}

/* ── Nav badge on camera ── */
#nav-badge-cam {
  position: absolute; top: 6px; right: 6px;
  font-size: 0.62rem; background: rgba(14,14,26,.85);
  padding: 2px 7px; border-radius: 4px; display: none;
}

/* ── Compass (Telem tab) ── */
.compass-row { display: flex; align-items: center; gap: 10px; margin-top: 4px; }
#compass-canvas { width: 44px; height: 44px; flex-shrink: 0; }
.hdg-detail { display: flex; flex-direction: column; gap: 2px; }
#hdg-val { font-size: 1.1rem; font-weight: bold; color: var(--cyan); }
#hdg-sub { font-size: 0.65rem; color: var(--gray); }

/* ── Navigator state card ── */
#nav-card { display: none; }
.nav-row { display: flex; align-items: center; gap: 10px; flex-wrap: wrap; }
#nav-state-val { font-size: 1rem; font-weight: bold; }
#nav-gate-val  { font-size: 0.85rem; color: var(--gray); }
#nav-err-val   { font-size: 0.8rem; }

/* ── Log viewer ── */
#log-toolbar {
  display: flex; gap: 8px; margin-bottom: 8px; align-items: center;
}
#log-filter {
  flex: 1; background: var(--bg); border: 1px solid var(--border);
  color: var(--white); border-radius: 6px; padding: 6px 10px;
  font-family: inherit; font-size: 0.75rem;
}
#log-autoscroll-btn { white-space: nowrap; }
#log-box {
  background: var(--bg); border: 1px solid var(--border);
  border-radius: 8px; padding: 8px;
  height: calc(100vh - var(--hdr-h) - var(--tab-h) - var(--estop-h) - 80px);
  overflow-y: auto; font-size: 0.68rem; line-height: 1.5;
  -webkit-overflow-scrolling: touch;
}
.log-line { white-space: pre-wrap; word-break: break-all; }
.log-line.info     { color: #b0b0c8; }
.log-line.warning  { color: var(--yellow); }
.log-line.error    { color: var(--red); }
.log-line.critical { color: var(--red); font-weight: bold; }
.log-line.debug    { color: #585870; }
</style>
</head>
<body>

<!-- Header -->
<div id="header">
  <span id="h-title">&#x1F916; YUKON</span>
  <span id="h-mode">--</span>
  <span id="h-speed"></span>
  <span id="h-conn">&#x26AB; …</span>
</div>

<!-- No-motors banner -->
<div id="no-motors-banner">&#x26A0; NO-MOTORS MODE</div>

<!-- Tab bar -->
<div id="tabs">
  <div class="tab active" onclick="showTab('drive')" id="tab-drive">
    <span class="icon">&#x1F4F7;</span>Drive
  </div>
  <div class="tab" onclick="showTab('telem')" id="tab-telem">
    <span class="icon">&#x26A1;</span>Telem
  </div>
  <div class="tab" onclick="showTab('gps')" id="tab-gps">
    <span class="icon">&#x1F4CD;</span>GPS
  </div>
  <div class="tab" onclick="showTab('sys')" id="tab-sys">
    <span class="icon">&#x1F4BB;</span>System
  </div>
  <div class="tab" onclick="showTab('logs')" id="tab-logs">
    <span class="icon">&#x1F4C4;</span>Logs
  </div>
</div>

<!-- Pages -->
<div id="pages">

  <!-- Drive -->
  <div class="page active" id="page-drive">

    <div class="card">
      <div class="card-title">Camera <span id="cam-rot-lbl"></span></div>
      <div id="cam-wrap">
        <img id="cam-img" src="/stream" alt="">
        <div id="cam-no">No camera signal</div>
        <canvas id="bearing-canvas"></canvas>
        <div id="nav-badge-cam"></div>
      </div>
      <div id="cam-btns">
        <button class="btn" id="btn-bearing" onclick="toggleBearing()">Bearing: OFF</button>
        <button class="btn" id="btn-aruco" onclick="cmd('aruco_toggle')">ArUco: OFF</button>
        <button class="btn" onclick="cmd('rotate_ccw')">&#x21BA; CCW</button>
        <button class="btn" onclick="cmd('rotate_cw')">CW &#x21BB;</button>
      </div>
    </div>

    <!-- Navigator state (shown only in AUTO·Camera) -->
    <div class="card" id="nav-card">
      <div class="card-title">Navigator</div>
      <div class="nav-row">
        <span id="nav-state-val">--</span>
        <span id="nav-gate-val">Gate --</span>
        <span id="nav-err-val"></span>
      </div>
    </div>

    <div class="card">
      <div class="card-title">Drive</div>
      <div class="motor-row">
        <span class="motor-lbl">L</span>
        <div class="motor-track">
          <div class="motor-mid"></div>
          <div class="motor-fill" id="fill-l"></div>
          <span class="motor-val" id="val-l">+0.00</span>
        </div>
      </div>
      <div class="motor-row">
        <span class="motor-lbl">R</span>
        <div class="motor-track">
          <div class="motor-mid"></div>
          <div class="motor-fill" id="fill-r"></div>
          <span class="motor-val" id="val-r">+0.00</span>
        </div>
      </div>
    </div>

    <div class="card">
      <div class="card-title">Status</div>
      <div id="badges">
        <span class="badge" id="bdg-rc">RC: --</span>
        <span class="badge" id="bdg-cam">CAM: --</span>
        <span class="badge" id="bdg-aco">ACO: --</span>
        <span class="badge" id="bdg-ldr">LDR: --</span>
        <span class="badge" id="bdg-gps">GPS: --</span>
        <span class="badge" id="bdg-log">LOG: --</span>
        <span class="badge" id="bdg-hdg" style="display:none">HDG: --</span>
        <span class="badge" id="bdg-nav" style="display:none">NAV: --</span>
      </div>
    </div>

  </div><!-- /page-drive -->

  <!-- Telem -->
  <div class="page" id="page-telem">

    <div class="card">
      <div class="card-title">Power</div>
      <div class="stat-grid">
        <div class="stat-item"><div class="lbl">Voltage</div><div class="val" id="t-volt">--</div></div>
        <div class="stat-item"><div class="lbl">Current</div><div class="val" id="t-curr">--</div></div>
      </div>
    </div>

    <div class="card">
      <div class="card-title">Temperature</div>
      <div class="stat-grid three">
        <div class="stat-item"><div class="lbl">Board</div><div class="val" id="t-board">--</div></div>
        <div class="stat-item"><div class="lbl">Left</div> <div class="val" id="t-ltmp">--</div></div>
        <div class="stat-item"><div class="lbl">Right</div><div class="val" id="t-rtmp">--</div></div>
      </div>
    </div>

    <div class="card">
      <div class="card-title">Faults</div>
      <div class="val" id="t-fault" style="font-size:1.1rem">OK</div>
    </div>

    <div class="card">
      <div class="card-title">IMU Heading</div>
      <div class="compass-row">
        <canvas id="compass-canvas" width="44" height="44"></canvas>
        <div class="hdg-detail">
          <span id="hdg-val">---</span>
          <span id="hdg-sub">No IMU</span>
        </div>
      </div>
    </div>

  </div><!-- /page-telem -->

  <!-- GPS -->
  <div class="page" id="page-gps">

    <div class="card">
      <div class="card-title">Fix</div>
      <div class="stat-grid three">
        <div class="stat-item"><div class="lbl">Quality</div><div class="val" id="g-fix">--</div></div>
        <div class="stat-item"><div class="lbl">Sats</div>  <div class="val" id="g-sats">--</div></div>
        <div class="stat-item"><div class="lbl">HDOP</div>  <div class="val" id="g-hdop">--</div></div>
      </div>
    </div>

    <div class="card">
      <div class="card-title">Position</div>
      <div class="stat-grid">
        <div class="stat-item"><div class="lbl">Latitude</div> <div class="val coord" id="g-lat">--</div></div>
        <div class="stat-item"><div class="lbl">Longitude</div><div class="val coord" id="g-lon">--</div></div>
        <div class="stat-item"><div class="lbl">Altitude</div> <div class="val" id="g-alt">--</div></div>
        <div class="stat-item"><div class="lbl">H-Error</div>  <div class="val" id="g-herr">--</div></div>
      </div>
    </div>

    <div class="card">
      <div class="card-title">Logging</div>
      <div style="display:flex;gap:8px">
        <div class="stat-item" style="flex:1">
          <div class="lbl">Status</div>
          <div class="val" id="g-log-status">--</div>
        </div>
        <button class="btn" style="flex:1" onclick="cmd('gps_bookmark')">
          &#x1F4CC; Bookmark
        </button>
      </div>
    </div>

  </div><!-- /page-gps -->

  <!-- System -->
  <div class="page" id="page-sys">

    <div class="card">
      <div class="card-title">CPU</div>
      <div style="display:flex;gap:12px;align-items:baseline;margin-bottom:8px">
        <span class="val" id="sys-temp" style="font-size:1.3rem">--</span>
        <span style="font-size:0.75rem;color:var(--gray)" id="sys-freq">--</span>
      </div>
      <div class="prog-row">
        <span class="prog-lbl">CPU</span>
        <div class="prog-track">
          <div class="prog-fill" id="sys-cpu-fill"></div>
          <span class="prog-val" id="sys-cpu-val">--</span>
        </div>
      </div>
      <div class="prog-row">
        <span class="prog-lbl">Mem</span>
        <div class="prog-track">
          <div class="prog-fill" id="sys-mem-fill"></div>
          <span class="prog-val" id="sys-mem-val">--</span>
        </div>
      </div>
      <div class="prog-row">
        <span class="prog-lbl">Disk</span>
        <div class="prog-track">
          <div class="prog-fill" id="sys-disk-fill"></div>
          <span class="prog-val" id="sys-disk-val">--</span>
        </div>
      </div>
    </div>

    <div class="card">
      <div class="card-title">Lidar</div>
      <canvas id="lidar-canvas"></canvas>
    </div>

  </div><!-- /page-sys -->

  <!-- Logs -->
  <div class="page" id="page-logs">
    <div id="log-toolbar">
      <input id="log-filter" type="search" placeholder="Filter…" oninput="filterLogs()">
      <button class="btn" id="log-autoscroll-btn" onclick="toggleAutoScroll()" style="flex:0">
        &#x2193; Auto
      </button>
      <button class="btn" onclick="fetchLogs()" style="flex:0">&#x21BB;</button>
    </div>
    <div id="log-box"></div>
  </div><!-- /page-logs -->

</div><!-- /pages -->

<!-- ESTOP bar -->
<div id="estop-bar">
  <button id="btn-estop" onclick="cmd('estop')">&#x26D4; ESTOP</button>
  <button id="btn-reset" onclick="cmd('reset')">Reset</button>
</div>

<script>
"use strict";

// ── Colours ───────────────────────────────────────────────────────────────────
const C = {
  green:  '#3cdc50', yellow: '#f0c828', orange: '#f08028',
  red:    '#dc3c3c', cyan:   '#3cc8dc', gray:   '#7878a0',
  border: '#3a3a58', bg:     '#0e0e1a',
};
const MODE_COLORS = { MANUAL: C.yellow, AUTO: C.green, ESTOP: C.red };
const FIX_COLORS  = { 0: C.red, 1: C.yellow, 2: C.orange, 3: C.orange, 4: C.green, 5: C.cyan };
const NAV_COLORS  = {
  SEARCHING: C.yellow, ALIGNING: C.orange, APPROACHING: C.green,
  PASSING: C.cyan, COMPLETE: C.green, ERROR: C.red, IDLE: C.gray,
  WAITING_FIX: C.yellow, NAVIGATING: C.green, ARRIVED: C.cyan,
};

const el  = id => document.getElementById(id);
const fmt = (v, d, u) => v == null ? '--' : v.toFixed(d) + (u ?? '');

let showBearing = false;
let lastState   = null;
let camNatW = 640, camNatH = 480;

// ── Tabs ──────────────────────────────────────────────────────────────────────
function updateLayout() {
  const bannerH = el('no-motors-banner').style.display === 'block' ? 22 : 0;
  const hdrH = 52, tabH = 56;
  el('tabs').style.top  = (hdrH + bannerH) + 'px';
  el('pages').style.top = (hdrH + tabH + bannerH) + 'px';
}

function showTab(name) {
  document.querySelectorAll('.tab').forEach(t => t.classList.remove('active'));
  document.querySelectorAll('.page').forEach(p => p.classList.remove('active'));
  el('tab-' + name).classList.add('active');
  el('page-' + name).classList.add('active');
  if (name === 'sys')  resizeLidar();
  if (name === 'logs') _startLogPolling();
  else                 _stopLogPolling();
}

// ── Compass ───────────────────────────────────────────────────────────────────
const compassCtx = el('compass-canvas').getContext('2d');
function drawCompass(heading) {
  const ctx = compassCtx, w = 44, h = 44, cx = w/2, cy = h/2, r = 18;
  ctx.clearRect(0, 0, w, h);
  ctx.beginPath(); ctx.arc(cx, cy, r, 0, Math.PI*2);
  ctx.strokeStyle = C.border; ctx.lineWidth = 1.5; ctx.stroke();
  ctx.strokeStyle = C.red; ctx.lineWidth = 2;
  ctx.beginPath(); ctx.moveTo(cx, cy-r+2); ctx.lineTo(cx, cy-r+8); ctx.stroke();
  const rad = (heading - 90) * Math.PI / 180;
  ctx.strokeStyle = C.cyan; ctx.lineWidth = 2.5;
  ctx.beginPath(); ctx.moveTo(cx, cy);
  ctx.lineTo(cx + r*Math.cos(rad), cy + r*Math.sin(rad)); ctx.stroke();
  ctx.beginPath(); ctx.arc(cx, cy, 3, 0, Math.PI*2);
  ctx.fillStyle = C.cyan; ctx.fill();
}

// ── Bearing overlay ───────────────────────────────────────────────────────────
const bearingCanvas = el('bearing-canvas');
const bearingCtx    = bearingCanvas.getContext('2d');

function resizeBearingCanvas() {
  const img = el('cam-img');
  bearingCanvas.width  = img.clientWidth  || 320;
  bearingCanvas.height = img.clientHeight || 240;
}

function drawBearingOverlay(aruco, navGate, heading) {
  resizeBearingCanvas();
  const ctx = bearingCtx, W = bearingCanvas.width, H = bearingCanvas.height;
  ctx.clearRect(0, 0, W, H);
  if (!aruco) return;
  const sx = W / camNatW, sy = H / camNatH;
  const fcx = W/2, fcy = H/2;
  const targetOdd = navGate*2+1, targetEven = navGate*2+2;
  ctx.font = 'bold 10px monospace';
  for (const tag of aruco.tags) {
    const tx = tag.cx*sx, ty = tag.cy*sy;
    const isTarget = tag.id===targetOdd || tag.id===targetEven;
    const color = isTarget ? C.cyan : C.yellow;
    ctx.strokeStyle = color; ctx.globalAlpha = 0.65; ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.moveTo(fcx, fcy); ctx.lineTo(tx, ty); ctx.stroke();
    ctx.globalAlpha = 1;
    ctx.beginPath(); ctx.arc(tx, ty, 4, 0, Math.PI*2);
    ctx.strokeStyle = color; ctx.lineWidth = 2; ctx.stroke();
    let lbl = `#${tag.id}`;
    if (tag.bearing  != null) lbl += ` ${tag.bearing >=0?'+':''}${tag.bearing.toFixed(1)}°`;
    if (tag.distance != null) lbl += ` ${tag.distance.toFixed(2)}m`;
    const lx = tx < fcx+W/3 ? tx+6 : tx-ctx.measureText(lbl).width-6, ly = ty-7;
    ctx.fillStyle = 'rgba(14,14,26,.8)';
    ctx.fillRect(lx-2, ly-11, ctx.measureText(lbl).width+4, 13);
    ctx.fillStyle = color; ctx.fillText(lbl, lx, ly);
  }
  for (const gate of aruco.gates) {
    if (gate.gate_id !== navGate) continue;
    const gx = gate.centre_x*sx, gy = gate.centre_y*sy;
    ctx.strokeStyle = C.green; ctx.lineWidth = 1.5; ctx.globalAlpha = 0.75;
    ctx.beginPath(); ctx.moveTo(fcx, fcy); ctx.lineTo(gx, gy); ctx.stroke();
    ctx.globalAlpha = 1;
    const sz = 10;
    ctx.strokeStyle = C.green; ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(gx-sz, gy); ctx.lineTo(gx+sz, gy);
    ctx.moveTo(gx, gy-sz); ctx.lineTo(gx, gy+sz);
    ctx.stroke();
    ctx.beginPath(); ctx.arc(gx, gy, sz, 0, Math.PI*2); ctx.stroke();
    let glbl = 'AIM';
    if (gate.bearing  != null) glbl += ` ${gate.bearing >=0?'+':''}${gate.bearing.toFixed(1)}°`;
    if (gate.distance != null) glbl += ` ${gate.distance.toFixed(2)}m`;
    ctx.fillStyle = 'rgba(14,14,26,.8)';
    ctx.fillRect(gx+2, gy-20, ctx.measureText(glbl).width+4, 13);
    ctx.fillStyle = C.green; ctx.fillText(glbl, gx+4, gy-9);
  }
  // Boresight
  ctx.strokeStyle = C.gray; ctx.globalAlpha = 0.45; ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(fcx-10, fcy); ctx.lineTo(fcx+10, fcy);
  ctx.moveTo(fcx, fcy-10); ctx.lineTo(fcx, fcy+10);
  ctx.stroke(); ctx.globalAlpha = 1;
  // Mini compass arc
  if (heading != null) {
    const ccx = W/2, ccy = H-20, cr = 13;
    ctx.fillStyle = 'rgba(14,14,26,.8)';
    ctx.beginPath(); ctx.arc(ccx, ccy, cr+2, 0, Math.PI*2); ctx.fill();
    ctx.beginPath(); ctx.arc(ccx, ccy, cr, 0, Math.PI*2);
    ctx.strokeStyle = C.border; ctx.lineWidth = 1; ctx.stroke();
    ctx.strokeStyle = C.red; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(ccx, ccy-cr+2); ctx.lineTo(ccx, ccy-cr+6); ctx.stroke();
    const nrad = (heading-90)*Math.PI/180;
    ctx.strokeStyle = C.cyan; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(ccx, ccy);
    ctx.lineTo(ccx+cr*Math.cos(nrad), ccy+cr*Math.sin(nrad)); ctx.stroke();
    ctx.font = '9px monospace'; ctx.fillStyle = C.cyan; ctx.textAlign = 'center';
    ctx.fillText(heading.toFixed(0)+'°', ccx, ccy-cr-3);
    ctx.textAlign = 'left';
  }
}

function toggleBearing() {
  showBearing = !showBearing;
  const btn = el('btn-bearing');
  btn.textContent = showBearing ? 'Bearing: ON' : 'Bearing: OFF';
  btn.className   = showBearing ? 'btn on' : 'btn';
  el('bearing-canvas').style.display = showBearing ? 'block' : 'none';
  if (!showBearing) bearingCtx.clearRect(0, 0, bearingCanvas.width, bearingCanvas.height);
  if (lastState) applyState(lastState);
}

// ── Motor bars ────────────────────────────────────────────────────────────────
function motorBar(fillId, valId, v) {
  const fill = el(fillId);
  const col  = v > 0 ? C.green : v < 0 ? C.orange : null;
  if (v >= 0) { fill.style.left = '50%'; fill.style.width = (v * 50) + '%'; }
  else        { fill.style.left = (50 + v * 50) + '%'; fill.style.width = (-v * 50) + '%'; }
  fill.style.backgroundColor = col || 'transparent';
  const vEl = el(valId);
  vEl.textContent = (v >= 0 ? '+' : '') + v.toFixed(2);
  vEl.style.color = col || C.gray;
}

// ── Progress bars ─────────────────────────────────────────────────────────────
function pctColor(pct, warn, crit) {
  return pct >= crit ? C.red : pct >= warn ? C.yellow : C.green;
}
function progBar(fillId, valId, pct, label, color) {
  const fill = el(fillId);
  fill.style.width           = Math.min(pct, 100) + '%';
  fill.style.backgroundColor = color;
  const vEl = el(valId);
  vEl.textContent = label;
  vEl.style.color = pct > 55 ? C.bg : color;
}

// ── Badges ────────────────────────────────────────────────────────────────────
function badge(id, text, cls) {
  const b = el(id);
  b.textContent = text;
  b.className   = 'badge' + (cls ? ' ' + cls : '');
}

// ── Lidar ─────────────────────────────────────────────────────────────────────
const lidarCanvas = el('lidar-canvas');
const lidarCtx    = lidarCanvas.getContext('2d');

function resizeLidar() {
  const card = lidarCanvas.parentElement;
  lidarCanvas.width  = card.clientWidth - 24;
  lidarCanvas.height = lidarCanvas.width;  // square
}

function drawLidar(angles, distances) {
  const W = lidarCanvas.width, H = lidarCanvas.height;
  const ctx = lidarCtx;
  ctx.clearRect(0, 0, W, H);
  ctx.fillStyle = C.bg; ctx.fillRect(0, 0, W, H);

  const cx = W / 2, cy = H / 2;
  const r  = Math.min(W, H) / 2 - 24;

  if (!angles || !angles.length) {
    ctx.fillStyle = C.gray; ctx.font = '13px monospace';
    ctx.textAlign = 'center'; ctx.fillText('No lidar data', cx, cy);
    return;
  }

  const maxDist = Math.max(...distances) || 1;

  for (const frac of [0.25, 0.5, 0.75, 1.0]) {
    ctx.beginPath(); ctx.arc(cx, cy, r * frac, 0, Math.PI * 2);
    ctx.strokeStyle = C.border; ctx.lineWidth = 1; ctx.stroke();
    ctx.fillStyle = C.gray; ctx.font = '9px monospace'; ctx.textAlign = 'left';
    ctx.fillText((maxDist * frac / 1000).toFixed(1) + 'm', cx + r * frac + 2, cy - 3);
  }

  ctx.strokeStyle = C.border; ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(cx, cy - r); ctx.lineTo(cx, cy + r);
  ctx.moveTo(cx - r, cy); ctx.lineTo(cx + r, cy);
  ctx.stroke();

  ctx.fillStyle = C.gray; ctx.font = '10px monospace'; ctx.textAlign = 'center';
  ctx.fillText('N', cx, cy - r - 4);
  ctx.fillText('S', cx, cy + r + 12);
  ctx.textAlign = 'left';
  ctx.fillText('E', cx + r + 3, cy + 4);
  ctx.fillText('W', cx - r - 14, cy + 4);

  for (let i = 0; i < angles.length; i++) {
    const d = distances[i]; if (d <= 0) continue;
    const f  = Math.min(d / maxDist, 1);
    const pr = f * r;
    const rad = angles[i] * Math.PI / 180;
    const px = cx + pr * Math.sin(rad), py = cy - pr * Math.cos(rad);
    const rr = Math.round(220 * (1 - f) +  60 * f);
    const gg = Math.round( 60 * (1 - f) + 200 * f);
    const bb = Math.round( 60 * (1 - f) + 220 * f);
    ctx.beginPath(); ctx.arc(px, py, 2, 0, Math.PI * 2);
    ctx.fillStyle = `rgb(${rr},${gg},${bb})`; ctx.fill();
  }

  ctx.beginPath(); ctx.arc(cx, cy, 5, 0, Math.PI * 2);
  ctx.fillStyle = C.green; ctx.fill();
}

// ── Apply state ───────────────────────────────────────────────────────────────
function applyState(s) {
  lastState = s;

  // No-motors banner
  const banner = el('no-motors-banner');
  banner.style.display = s.no_motors ? 'block' : 'none';
  updateLayout();

  // Header
  const mc = MODE_COLORS[s.mode] || C.gray;
  const mEl = el('h-mode');
  mEl.textContent = s.mode === 'AUTO' ? `${s.mode}·${s.auto_type}` : s.mode;
  mEl.style.color = mc;
  el('header').style.borderBottomColor = mc;

  const sc = s.speed_scale, sEl = el('h-speed');
  if      (sc < 0.45) { sEl.textContent = '🐢 Slow';   sEl.style.color = C.cyan;   }
  else if (sc < 0.80) { sEl.textContent = '🚗 Medium'; sEl.style.color = C.yellow; }
  else                { sEl.textContent = '🏎 Fast';   sEl.style.color = C.green;  }

  // Drive
  motorBar('fill-l', 'val-l', s.drive.left);
  motorBar('fill-r', 'val-r', s.drive.right);

  // Camera
  el('cam-img').style.display  = s.camera_ok ? 'block' : 'none';
  el('cam-no').style.display   = s.camera_ok ? 'none'  : 'flex';
  el('cam-rot-lbl').textContent = s.cam_rotation ? `[${s.cam_rotation}°]` : '';

  // ArUco button
  const aBtn = el('btn-aruco');
  if (s.aruco_enabled) {
    aBtn.textContent = s.aruco
      ? `ArUco: ${s.aruco.tag_count}T ${s.aruco.gate_count}G`
      : 'ArUco: ON';
    aBtn.classList.add('on');
  } else {
    aBtn.textContent = 'ArUco: OFF';
    aBtn.classList.remove('on');
  }

  // Bearing overlay
  el('bearing-canvas').style.display = (showBearing && s.aruco_enabled) ? 'block' : 'none';
  if (showBearing && s.aruco_enabled)
    drawBearingOverlay(s.aruco, s.nav_gate, s.telemetry.heading);

  // Nav badge on camera + nav card
  const autoCamera = s.mode === 'AUTO' && (s.auto_type === 'Camera' || s.auto_type === 'Cam+GPS');
  const autoGps    = s.mode === 'AUTO' && (s.auto_type === 'GPS'     || s.auto_type === 'Cam+GPS');
  const navBadge   = el('nav-badge-cam');
  const navCard    = el('nav-card');
  if ((autoCamera || autoGps) && s.nav_state && s.nav_state !== 'IDLE') {
    const nc = NAV_COLORS[s.nav_state] || C.gray;
    let ntxt, gateOrWp, errTxt;
    if (autoGps) {
      ntxt = `GPS:${s.nav_state}  WP:${s.nav_wp}`;
      if (s.nav_wp_dist != null) ntxt += `  ${s.nav_wp_dist.toFixed(1)}m`;
      if (s.nav_wp_bear != null) ntxt += `  ${s.nav_wp_bear.toFixed(0)}°`;
      gateOrWp = `WP ${s.nav_wp}`;
      errTxt = s.nav_wp_dist != null ? `${s.nav_wp_dist.toFixed(1)}m` : '';
    } else {
      ntxt = `${s.nav_state}  G${s.nav_gate}`;
      if (s.nav_bearing_err != null)
        ntxt += `  ${s.nav_bearing_err >= 0 ? '+' : ''}${s.nav_bearing_err.toFixed(1)}°`;
      gateOrWp = `Gate ${s.nav_gate}`;
      errTxt = s.nav_bearing_err != null
        ? `${s.nav_bearing_err >= 0 ? '+' : ''}${s.nav_bearing_err.toFixed(1)}°` : '';
    }
    navBadge.textContent = ntxt; navBadge.style.color = nc; navBadge.style.display = 'block';
    navCard.style.display = 'block';
    el('nav-state-val').textContent = s.nav_state; el('nav-state-val').style.color = nc;
    el('nav-gate-val').textContent  = gateOrWp;
    el('nav-err-val').textContent   = errTxt;
    el('nav-err-val').style.color   = C.cyan;
  } else {
    navBadge.style.display = 'none';
    navCard.style.display  = 'none';
  }

  // Telemetry
  const t = s.telemetry;
  el('t-volt').textContent  = fmt(t.voltage,    1, ' V');
  el('t-curr').textContent  = fmt(t.current,    2, ' A');
  el('t-board').textContent = fmt(t.board_temp, 0, '°C');
  el('t-ltmp').textContent  = fmt(t.left_temp,  0, '°C');
  el('t-rtmp').textContent  = fmt(t.right_temp, 0, '°C');
  const fEl = el('t-fault');
  const faulted = t.left_fault || t.right_fault;
  fEl.textContent = (t.left_fault ? 'FAULT-L ' : '') + (t.right_fault ? 'FAULT-R' : '') || 'OK';
  fEl.style.color = faulted ? C.red : C.green;

  // IMU heading
  const hdgVal = el('hdg-val'), hdgSub = el('hdg-sub');
  if (t.heading != null) {
    hdgVal.textContent = t.heading.toFixed(1) + '°'; hdgVal.style.color = C.cyan;
    hdgSub.textContent = 'IMU OK'; hdgSub.style.color = C.gray;
    drawCompass(t.heading);
    el('compass-canvas').style.opacity = '1';
  } else {
    hdgVal.textContent = '---'; hdgVal.style.color = C.gray;
    hdgSub.textContent = 'No IMU'; hdgSub.style.color = C.gray;
    el('compass-canvas').style.opacity = '0.25';
  }
  fEl.textContent = (t.left_fault ? 'FAULT-L ' : '') + (t.right_fault ? 'FAULT-R' : '') || 'OK';
  fEl.style.color = faulted ? C.red : C.green;

  // GPS
  const g = s.gps, gOk = s.gps_ok;
  const fc = FIX_COLORS[g.fix_quality] || C.gray;
  const fixEl = el('g-fix');
  fixEl.textContent = gOk ? g.fix_quality_name : 'No GPS';
  fixEl.style.color = gOk ? fc : C.gray;
  el('g-sats').textContent = g.satellites != null
    ? g.satellites + (g.satellites_view ? '/' + g.satellites_view : '') : '--';
  el('g-hdop').textContent = fmt(g.hdop, 2);
  el('g-lat').textContent  = g.latitude  != null ? g.latitude.toFixed(7)  : '--';
  el('g-lon').textContent  = g.longitude != null ? g.longitude.toFixed(7) : '--';
  el('g-alt').textContent  = g.altitude  != null ? g.altitude.toFixed(1) + ' m' : '--';
  el('g-herr').textContent = g.h_error_m != null ? g.h_error_m.toFixed(3) + ' m' : '--';
  const logEl = el('g-log-status');
  logEl.textContent = s.gps_logging ? 'RECORDING' : 'Idle';
  logEl.style.color = s.gps_logging ? C.cyan : C.gray;

  // System
  const sys = s.system;
  const cpuC  = pctColor(sys.cpu_percent, 60, 85);
  const memC  = pctColor(sys.mem_percent, 70, 90);
  const diskC = pctColor(sys.disk_percent, 70, 90);
  const tmpC  = sys.cpu_temp_c < 60 ? C.green : sys.cpu_temp_c < 75 ? C.yellow : C.red;

  const tEl = el('sys-temp');
  tEl.textContent = sys.cpu_temp_c.toFixed(0) + '°C';
  tEl.style.color = tmpC;
  el('sys-freq').textContent = sys.cpu_freq_mhz.toFixed(0) + ' MHz';

  progBar('sys-cpu-fill',  'sys-cpu-val',  sys.cpu_percent,
          sys.cpu_percent.toFixed(0) + '%', cpuC);
  progBar('sys-mem-fill',  'sys-mem-val',  sys.mem_percent,
          sys.mem_percent.toFixed(0) + '%  '
          + (sys.mem_used_mb/1024).toFixed(1) + '/'
          + (sys.mem_total_mb/1024).toFixed(1) + 'GB', memC);
  progBar('sys-disk-fill', 'sys-disk-val', sys.disk_percent,
          sys.disk_percent.toFixed(0) + '%  '
          + sys.disk_used_gb.toFixed(1) + '/'
          + sys.disk_total_gb.toFixed(1) + 'GB', diskC);

  // Lidar (only paint if visible)
  if (el('page-sys').classList.contains('active'))
    drawLidar(s.lidar.angles, s.lidar.distances);

  // Badges
  badge('bdg-rc',  'RC: '  + (s.rc_active  ? 'OK' : '--'), s.rc_active  ? 'ok' : '');
  badge('bdg-cam', 'CAM: ' + (s.camera_ok  ? 'OK' : '--'), s.camera_ok  ? 'ok' : '');
  badge('bdg-aco', 'ACO: ' + (s.aruco_ok   ? 'OK' : '--'), s.aruco_ok   ? 'ok' : '');
  badge('bdg-ldr', 'LDR: ' + (s.lidar_ok   ? 'OK' : '--'), s.lidar_ok   ? 'ok' : '');
  badge('bdg-gps', 'GPS: ' + (gOk ? g.fix_quality_name : '--'),
        gOk ? (g.fix_quality >= 4 ? 'ok' : 'warn') : '');
  badge('bdg-log', 'LOG: ' + (s.gps_logging ? 'ON' : 'OFF'), s.gps_logging ? 'ok' : '');

  const hdgBadge = el('bdg-hdg');
  if (t.heading != null) {
    hdgBadge.textContent = `HDG: ${t.heading.toFixed(1)}°`;
    hdgBadge.className   = 'badge info';
    hdgBadge.style.display = 'inline';
  } else { hdgBadge.style.display = 'none'; }

  const navBdg = el('bdg-nav');
  if (s.nav_state && s.nav_state !== 'IDLE') {
    const nc = NAV_COLORS[s.nav_state] || C.gray;
    const navTxt = autoGps
      ? `GPS: ${s.nav_state} WP${s.nav_wp}${s.nav_wp_dist != null ? '  '+s.nav_wp_dist.toFixed(1)+'m' : ''}`
      : `NAV: ${s.nav_state} G${s.nav_gate}`;
    navBdg.textContent       = navTxt;
    navBdg.style.borderColor = nc;
    navBdg.style.color       = nc;
    navBdg.style.display     = 'inline';
  } else { navBdg.style.display = 'none'; }
}

// ── Commands ──────────────────────────────────────────────────────────────────
async function cmd(c) {
  try {
    await fetch('/api/cmd', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({cmd: c}),
    });
  } catch(e) { console.error('cmd failed:', e); }
}

// ── SSE ───────────────────────────────────────────────────────────────────────
let evtSrc = null;
function connect() {
  if (evtSrc) evtSrc.close();
  const conn = el('h-conn');
  conn.textContent = '🟡 Connecting…'; conn.style.color = C.yellow;

  evtSrc = new EventSource('/api/state');
  evtSrc.onopen = () => {
    conn.textContent = '🟢 Live'; conn.style.color = C.green;
  };
  evtSrc.onmessage = e => {
    try { applyState(JSON.parse(e.data)); }
    catch(err) { console.error('SSE parse:', err); }
  };
  evtSrc.onerror = () => {
    conn.textContent = '🔴 Reconnecting…'; conn.style.color = C.red;
    evtSrc.close(); setTimeout(connect, 3000);
  };
}

// ── Log viewer ────────────────────────────────────────────────────────────────
let _logAutoScroll = true;
let _logLines      = [];
let _logInterval   = null;

function toggleAutoScroll() {
  _logAutoScroll = !_logAutoScroll;
  const btn = el('log-autoscroll-btn');
  btn.classList.toggle('on', _logAutoScroll);
  btn.textContent = _logAutoScroll ? '↓ Auto' : '↓ Manual';
}

function _levelClass(line) {
  if (/\bCRITICAL\b/.test(line)) return 'critical';
  if (/\bERROR\b/.test(line))    return 'error';
  if (/\bWARNING\b/.test(line))  return 'warning';
  if (/\bDEBUG\b/.test(line))    return 'debug';
  return 'info';
}

function filterLogs() {
  const q = el('log-filter').value.toLowerCase();
  const box = el('log-box');
  for (const div of box.children) {
    div.style.display = (!q || div.textContent.toLowerCase().includes(q)) ? '' : 'none';
  }
}

async function fetchLogs() {
  try {
    const resp = await fetch('/api/logs?n=200');
    const data = await resp.json();
    _logLines = data.lines || [];
    const box  = el('log-box');
    const q    = el('log-filter').value.toLowerCase();
    box.innerHTML = '';
    for (const line of _logLines) {
      const div = document.createElement('div');
      div.className = 'log-line ' + _levelClass(line);
      div.textContent = line;
      if (q && !line.toLowerCase().includes(q)) div.style.display = 'none';
      box.appendChild(div);
    }
    if (_logAutoScroll) box.scrollTop = box.scrollHeight;
  } catch(e) { /* server not ready */ }
}

function _startLogPolling() {
  if (_logInterval) return;
  fetchLogs();
  _logInterval = setInterval(fetchLogs, 2000);
}

function _stopLogPolling() {
  if (_logInterval) { clearInterval(_logInterval); _logInterval = null; }
}

// ── Init ──────────────────────────────────────────────────────────────────────
el('cam-img').addEventListener('load', function() {
  if (this.naturalWidth)  camNatW = this.naturalWidth;
  if (this.naturalHeight) camNatH = this.naturalHeight;
});
window.addEventListener('resize', () => { resizeLidar(); resizeBearingCanvas(); });
updateLayout();
resizeLidar();
connect();
</script>
</body>
</html>"""


# ── Config helpers ─────────────────────────────────────────────────────────────

def _load_config(path):
    cfg = configparser.ConfigParser(inline_comment_prefixes=('#',))
    cfg.read(path)
    return cfg


def _cfg(cfg, section, key, fallback, cast=str):
    try:
        raw = cfg.get(section, key).strip()
        return cast(raw) if raw else fallback
    except Exception:
        return fallback


def _local_ip():
    import socket
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(('8.8.8.8', 80))
            return s.getsockname()[0]
    except Exception:
        return 'localhost'


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    global _robot, _jpeg_encode

    DEFAULT_CFG = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'robot.ini')

    parser = argparse.ArgumentParser(description='Yukon mobile web dashboard')
    parser.add_argument('--config',          default=DEFAULT_CFG)
    parser.add_argument('--host',            default=None)
    parser.add_argument('--port',            default=None, type=int)
    parser.add_argument('--yukon-port',      default=None)
    parser.add_argument('--ibus-port',       default=None)
    parser.add_argument('--gps-port',        default=None)
    parser.add_argument('--lidar-port',      default=None)
    parser.add_argument('--ntrip-host',      default=None)
    parser.add_argument('--ntrip-port',      default=None, type=int, dest='ntrip_port_arg')
    parser.add_argument('--ntrip-mount',     default=None)
    parser.add_argument('--ntrip-user',      default=None)
    parser.add_argument('--ntrip-password',  default=None)
    parser.add_argument('--rtcm-port',       default=None)
    parser.add_argument('--rtcm-baud',       default=None, type=int)
    parser.add_argument('--no-camera',       action='store_true', default=False)
    parser.add_argument('--no-lidar',        action='store_true', default=False)
    parser.add_argument('--no-gps',          action='store_true', default=False)
    parser.add_argument('--no-motors',       action='store_true', default=False,
                        help='Suppress all motor/LED/bearing commands (bench test mode)')
    args = parser.parse_args()

    global _log_path
    _log_path = setup_logging()
    log.info(f"Log file: {_log_path}")
    cfg = _load_config(args.config)

    web_host = args.host or _cfg(cfg, 'mobile', 'host', '0.0.0.0')
    web_port = args.port or _cfg(cfg, 'mobile', 'port', 5001, int)

    _jpeg_encode = _make_jpeg_encoder()

    bool_val = lambda x: x.lower() == 'true'
    port_val = lambda x: None if x.lower() in ('auto', '') else x

    def arg(cli_val, section, key, fallback, cast=str):
        return cli_val if cli_val is not None else _cfg(cfg, section, key, fallback, cast)

    _robot = Robot(
        yukon_port      = arg(args.yukon_port,     'robot', 'yukon_port',  None,  port_val),
        ibus_port       = arg(args.ibus_port,       'robot', 'ibus_port',   '/dev/ttyAMA3'),
        lidar_port      = arg(args.lidar_port,      'lidar', 'port',        '/dev/ttyUSB0'),
        gps_port        = arg(args.gps_port,        'gps',   'port',        '/dev/ttyUSB0'),
        ntrip_host      = '' if _cfg(cfg, 'ntrip', 'disabled', False, bool_val)
                          else arg(args.ntrip_host, 'ntrip', 'host', ''),
        ntrip_port      = arg(args.ntrip_port_arg,  'ntrip', 'port',        2101,  int),
        ntrip_mount     = arg(args.ntrip_mount,     'ntrip', 'mount',       ''),
        ntrip_user      = arg(args.ntrip_user,      'ntrip', 'user',        ''),
        ntrip_password  = arg(args.ntrip_password,  'ntrip', 'password',    ''),
        rtcm_port       = '' if _cfg(cfg, 'rtcm', 'disabled', False, bool_val)
                          else arg(args.rtcm_port,  'rtcm',  'port',        ''),
        rtcm_baud       = arg(args.rtcm_baud,       'rtcm',  'baud',        115200, int),
        enable_camera   = not (args.no_camera or _cfg(cfg, 'camera', 'disabled', False, bool_val)),
        enable_lidar    = not (args.no_lidar  or _cfg(cfg, 'lidar',  'disabled', False, bool_val)),
        enable_gps      = not (args.no_gps    or _cfg(cfg, 'gps',    'disabled', False, bool_val)),
        cam_width       = _cfg(cfg, 'camera', 'width',    640,  int),
        cam_height      = _cfg(cfg, 'camera', 'height',  480,  int),
        cam_fps         = _cfg(cfg, 'camera', 'fps',     30,   int),
        cam_rotation    = _cfg(cfg, 'camera', 'rotation', 0,   int),
        enable_aruco    = _cfg(cfg, 'aruco',  'enabled', False, bool_val),
        aruco_dict      = _cfg(cfg, 'aruco',  'dict',    'DICT_4X4_1000'),
        aruco_calib     = _cfg(cfg, 'aruco',  'calib_file', ''),
        aruco_tag_size  = _cfg(cfg, 'aruco',  'tag_size',   0.15, float),
        throttle_ch     = _cfg(cfg, 'rc', 'throttle_ch',  3,    int),
        steer_ch        = _cfg(cfg, 'rc', 'steer_ch',     1,    int),
        mode_ch         = _cfg(cfg, 'rc', 'mode_ch',      5,    int),
        speed_ch        = _cfg(cfg, 'rc', 'speed_ch',     6,    int),
        auto_type_ch    = _cfg(cfg, 'rc', 'auto_type_ch', 7,    int),
        gps_log_ch      = _cfg(cfg, 'rc', 'gps_log_ch',      8,   int),
        gps_bookmark_ch = _cfg(cfg, 'rc', 'gps_bookmark_ch', 2,   int),
        gps_log_dir     = _cfg(cfg, 'gps', 'log_dir',        ''),
        gps_log_hz      = _cfg(cfg, 'gps', 'log_hz',         5.0, float),
        deadzone        = _cfg(cfg, 'rc', 'deadzone',    30,   int),
        failsafe_s      = _cfg(cfg, 'rc', 'failsafe_s',  0.5,  float),
        speed_min       = _cfg(cfg, 'rc', 'speed_min',   0.25, float),
        control_hz      = _cfg(cfg, 'rc', 'control_hz',  50,   int),
        no_motors       = args.no_motors,
    )

    _robot.start()
    log.info(f'Mobile dashboard → http://{_local_ip()}:{web_port}/')
    try:
        app.run(host=web_host, port=web_port, threaded=True, use_reloader=False)
    finally:
        _robot.stop()


if __name__ == '__main__':
    main()
