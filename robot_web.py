#!/usr/bin/env python3
"""
robot_web.py — Flask web dashboard for the Yukon robot.

Serves a browser-based status display that mirrors robot_gui.py:
  · Real-time telemetry via Server-Sent Events (10 Hz)
  · Live camera as MJPEG stream
  · Lidar polar plot on Canvas
  · ESTOP / Reset controls
  · Camera rotation controls (0 / 90 / 180 / 270°)

Usage:
    python3 robot_web.py                     # listens on 0.0.0.0:5000
    python3 robot_web.py --port 8080
    python3 robot_web.py --host 127.0.0.1   # local only

Dependencies:
    pip install flask
    pip install pillow          # or: pip install opencv-python  (JPEG encoding)
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
from robot import Robot, RobotMode, AutoType

log = logging.getLogger(__name__)

# ── JPEG encoder (OpenCV preferred; Pillow fallback) ──────────────────────────

def _make_jpeg_encoder():
    try:
        import cv2
        def _enc(frame, quality=82):
            _, buf = cv2.imencode('.jpg', frame[:, :, ::-1],
                                  [cv2.IMWRITE_JPEG_QUALITY, quality])
            return buf.tobytes()
        log.info("JPEG encoder: OpenCV")
        return _enc
    except ImportError:
        pass
    try:
        from PIL import Image as _PILImage
        def _enc(frame, quality=82):
            buf = io.BytesIO()
            _PILImage.fromarray(frame).save(buf, format='JPEG', quality=quality)
            return buf.getvalue()
        log.info("JPEG encoder: Pillow")
        return _enc
    except ImportError:
        pass
    log.warning("No JPEG encoder — camera stream disabled (pip install pillow)")
    return None

_jpeg_encode = None   # set in main()


# ── State serialiser ──────────────────────────────────────────────────────────

def _serialise(state, cam_rotation=0, aruco_enabled=False, aruco_state=None):
    g, t, d, li, s = state.gps, state.telemetry, state.drive, state.lidar, state.system
    aruco_info = None
    if aruco_enabled and aruco_state is not None:
        aruco_info = {
            'tag_count':  len(aruco_state.tags),
            'gate_count': len(aruco_state.gates),
            'fps':        aruco_state.fps,
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


# ── Embedded HTML / CSS / JS ──────────────────────────────────────────────────

_HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Yukon Robot Monitor</title>
<style>
*, *::before, *::after { box-sizing: border-box; margin: 0; padding: 0; }
:root {
  --bg:     #12121e;  --panel:  #1c1c2d;  --border: #3c3c5a;
  --white:  #e6e6f0;  --gray:   #82829a;
  --green:  #3cdc50;  --yellow: #f0c828;  --orange: #f08c28;
  --red:    #dc3c3c;  --cyan:   #3cc8dc;
}
html, body {
  height: 100%; background: var(--bg); color: var(--white);
  font-family: 'Courier New', Courier, monospace; overflow: hidden;
}
body {
  display: flex; flex-direction: column;
  gap: 6px; padding: 6px; height: 100vh;
}
.panel {
  background: var(--panel); border: 1px solid var(--border);
  border-radius: 8px; padding: 6px 10px;
}
.ptitle { font-size: 12px; color: var(--gray); margin-bottom: 4px; }
.field label { font-size: 11px; color: var(--gray); display: block; }
.field span  { font-size: 15px; }

/* ── Title bar ── */
#titlebar {
  display: flex; align-items: center; gap: 16px;
  padding: 8px 14px; flex-shrink: 0;
}
#main-title { font-size: 20px; font-weight: bold; flex: 1; }
.info { display: flex; gap: 24px; font-size: 16px; }
.info span b { font-size: 18px; }
.controls { display: flex; gap: 8px; }
button {
  font-family: inherit; font-size: 13px; padding: 4px 12px;
  border-radius: 5px; border: 1px solid var(--border);
  background: var(--panel); color: var(--white); cursor: pointer;
}
button:hover { background: #2a2a42; }
#btn-estop { border-color: var(--red);   color: var(--red);   }
#btn-reset { border-color: var(--green); color: var(--green); }

/* ── Status panels row ── */
#panels-row {
  display: flex; gap: 6px; flex-shrink: 0; height: 188px;
}
#drive-panel { width: 230px; flex-shrink: 0; }
#telem-panel { width: 270px; flex-shrink: 0; }
#gps-panel   { flex: 1; }

/* Motor bars */
.mrow { display: flex; align-items: center; gap: 8px; margin-top: 12px; }
.mlabel { width: 14px; font-size: 13px; color: var(--gray); }
.mtrack {
  flex: 1; height: 40px; background: var(--bg);
  border-radius: 4px; position: relative;
}
.mmid {
  position: absolute; left: 50%; top: 4px; bottom: 4px;
  width: 1px; background: var(--border);
}
.mfill {
  position: absolute; top: 4px; bottom: 4px;
  border-radius: 3px; transition: all 0.06s linear;
}
.mval {
  position: absolute; right: 6px; top: 50%;
  transform: translateY(-50%); font-size: 12px; pointer-events: none;
}

/* Telemetry */
.tgrid       { display: grid; grid-template-columns: 1fr 1fr;     gap: 6px 14px; margin-top: 6px; }
.tgrid.three { display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 6px 12px; margin-top: 6px; }

/* GPS */
.ggrid       { display: grid; grid-template-columns: 1fr 1fr;     gap: 5px 14px; margin-top: 6px; }
.ggrid.three { display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 5px 12px; margin-top: 6px; }

/* System panel */
.sys-row { display: flex; align-items: center; gap: 4px; margin-top: 8px; }
.sys-row label { font-size: 11px; color: var(--gray); width: 30px; flex-shrink: 0; }
.sys-track {
  flex: 1; height: 16px; background: var(--bg);
  border-radius: 3px; position: relative; overflow: hidden;
}
.sys-fill {
  position: absolute; left: 0; top: 0; bottom: 0;
  border-radius: 3px; transition: width 0.3s ease;
}
.sys-val {
  position: absolute; right: 3px; top: 50%;
  transform: translateY(-50%); font-size: 10px; pointer-events: none;
}

/* ── Body row ── */
#body-row {
  display: flex; gap: 6px; flex: 1; min-height: 0;
}
#camera-panel, #lidar-panel {
  flex: 1; display: flex; flex-direction: column;
}
.cam-header {
  display: flex; align-items: center; gap: 8px; margin-bottom: 4px;
}
.cam-header .ptitle { margin: 0; flex: 1; }
.cam-header button  { padding: 2px 8px; font-size: 12px; }
#cam-wrap {
  flex: 1; overflow: hidden;
  display: flex; align-items: center; justify-content: center;
  background: var(--bg); border-radius: 4px;
}
#cam-img {
  max-width: 100%; max-height: 100%; object-fit: contain;
  transition: transform 0.2s ease;
}
#cam-nosignal { color: var(--gray); font-size: 14px; display: none; }
#lidar-canvas { width: 100%; flex: 1; display: block; }

/* ── Footer ── */
#footer {
  display: flex; align-items: center; gap: 20px;
  padding: 3px 8px; border-top: 1px solid var(--border);
  flex-shrink: 0; font-size: 13px;
}
#conn-badge { margin-left: auto; font-size: 12px; color: var(--gray); }
</style>
</head>
<body>

<!-- Title bar -->
<div class="panel" id="titlebar">
  <span id="main-title">Yukon Robot Monitor</span>
  <div class="info">
    <span>Speed:&nbsp;<b id="speed-lbl" style="color:var(--cyan)">--</b></span>
    <span>Mode:&nbsp;<b  id="mode-lbl">--</b></span>
  </div>
  <div class="controls">
    <button id="btn-estop">ESTOP</button>
    <button id="btn-reset">Reset</button>
  </div>
</div>

<!-- Status panels -->
<div id="panels-row">

  <!-- Drive -->
  <div class="panel" id="drive-panel">
    <div class="ptitle">Drive</div>
    <div class="mrow">
      <span class="mlabel">L</span>
      <div class="mtrack">
        <div class="mmid"></div>
        <div class="mfill" id="fill-l"></div>
        <span class="mval" id="val-l" style="color:var(--gray)">+0.00</span>
      </div>
    </div>
    <div class="mrow">
      <span class="mlabel">R</span>
      <div class="mtrack">
        <div class="mmid"></div>
        <div class="mfill" id="fill-r"></div>
        <span class="mval" id="val-r" style="color:var(--gray)">+0.00</span>
      </div>
    </div>
  </div>

  <!-- Telemetry -->
  <div class="panel" id="telem-panel">
    <div class="ptitle" id="telem-title">Telemetry</div>
    <div class="tgrid">
      <div class="field"><label>Voltage</label><span id="t-volt">--</span></div>
      <div class="field"><label>Current</label><span id="t-curr">--</span></div>
    </div>
    <div class="tgrid three">
      <div class="field"><label>Board</label><span id="t-board">--</span></div>
      <div class="field"><label>Left</label> <span id="t-ltmp">--</span></div>
      <div class="field"><label>Right</label><span id="t-rtmp">--</span></div>
    </div>
    <div class="field" style="margin-top:8px">
      <label>Faults</label>
      <span id="t-fault" style="color:var(--green)">OK</span>
    </div>
  </div>

  <!-- GPS -->
  <div class="panel" id="gps-panel">
    <div class="ptitle" id="gps-title">GPS</div>
    <div class="ggrid">
      <div class="field"><label>Latitude</label> <span id="g-lat">--</span></div>
      <div class="field"><label>Altitude</label> <span id="g-alt">--</span></div>
      <div class="field"><label>Longitude</label><span id="g-lon">--</span></div>
      <div class="field"><label>H-Error</label>  <span id="g-herr">--</span></div>
    </div>
    <div class="ggrid three" style="margin-top:6px">
      <div class="field"><label>Fix</label> <span id="g-fix">--</span></div>
      <div class="field"><label>Sats</label><span id="g-sats">--</span></div>
      <div class="field"><label>HDOP</label><span id="g-hdop">--</span></div>
    </div>
  </div>

  <!-- System -->
  <div class="panel" id="sys-panel" style="width:210px;flex-shrink:0">
    <div class="ptitle" id="sys-title">System</div>
    <div class="sys-row">
      <label>CPU</label>
      <div class="sys-track"><div class="sys-fill" id="sys-cpu-fill"></div>
        <span class="sys-val" id="sys-cpu-val">--</span></div>
    </div>
    <div class="sys-row" style="margin-top:4px">
      <label>Temp</label>
      <span id="sys-temp" style="font-size:15px;margin-left:6px">--</span>
      <span id="sys-freq" style="font-size:11px;color:var(--gray);margin-left:10px">--</span>
    </div>
    <div class="sys-row" style="margin-top:10px">
      <label>Mem</label>
      <div class="sys-track"><div class="sys-fill" id="sys-mem-fill"></div>
        <span class="sys-val" id="sys-mem-val">--</span></div>
    </div>
    <div class="sys-row" style="margin-top:10px">
      <label>Disk</label>
      <div class="sys-track"><div class="sys-fill" id="sys-disk-fill"></div>
        <span class="sys-val" id="sys-disk-val">--</span></div>
    </div>
  </div>

</div><!-- /panels-row -->

<!-- Body -->
<div id="body-row">

  <!-- Camera -->
  <div class="panel" id="camera-panel">
    <div class="cam-header">
      <span class="ptitle">Camera <span id="cam-rot-lbl"></span></span>
      <span id="cam-aruco-info" style="font-size:11px;color:var(--gray);margin-right:auto"></span>
      <button id="btn-aruco" title="Toggle ArUco tag detection (T key)">ArUco: OFF</button>
      <button id="btn-ccw" title="Rotate counter-clockwise ([ key)">↺ CCW</button>
      <button id="btn-cw"  title="Rotate clockwise (] key)">CW ↻</button>
    </div>
    <div id="cam-wrap">
      <img id="cam-img" src="/stream" alt="Camera feed">
      <span id="cam-nosignal">No camera</span>
    </div>
  </div>

  <!-- Lidar -->
  <div class="panel" id="lidar-panel">
    <div class="ptitle">Lidar</div>
    <canvas id="lidar-canvas"></canvas>
  </div>

</div><!-- /body-row -->

<!-- Footer -->
<div id="footer">
  <span id="badge-rc">RC: --</span>
  <span id="badge-cam">CAM: --</span>
  <span id="badge-aco">ACO: --</span>
  <span id="badge-ldr">LDR: --</span>
  <span id="badge-gps">GPS: --</span>
  <span id="footer-herr"></span>
  <span id="badge-log">LOG: --</span>
  <span id="conn-badge">⚫ Connecting…</span>
</div>

<script>
"use strict";

// ── Colour palette ────────────────────────────────────────────────────────────
const C = {
  green: '#3cdc50', yellow: '#f0c828', orange: '#f08c28',
  red:   '#dc3c3c', cyan:   '#3cc8dc', gray:   '#82829a',
  border:'#3c3c5a', bg:     '#12121e',
};
const MODE_COLORS = { MANUAL: C.yellow, AUTO: C.green, ESTOP: C.red };
const FIX_COLORS  = { 0: C.red, 1: C.yellow, 2: C.orange, 3: C.orange, 4: C.green, 5: C.cyan };

// ── Utilities ─────────────────────────────────────────────────────────────────
const el  = id => document.getElementById(id);
const fmt = (v, d, u) => v == null ? '--' : v.toFixed(d) + (u || '');

// ── Motor bars ────────────────────────────────────────────────────────────────
function updateBar(fillId, valId, v) {
  const fill = el(fillId), valEl = el(valId);
  const color = v > 0 ? C.green : v < 0 ? C.orange : null;
  if (v >= 0) {
    fill.style.left  = '50%';
    fill.style.width = (v * 50) + '%';
  } else {
    fill.style.left  = (50 + v * 50) + '%';
    fill.style.width = (-v * 50) + '%';
  }
  fill.style.backgroundColor = color || 'transparent';
  valEl.textContent = (v >= 0 ? '+' : '') + v.toFixed(2);
  valEl.style.color = color || C.gray;
}

// ── Lidar canvas ──────────────────────────────────────────────────────────────
const lidarCanvas = el('lidar-canvas');
const lidarCtx    = lidarCanvas.getContext('2d');

function resizeLidar() {
  const panel = el('lidar-panel');
  lidarCanvas.width  = panel.clientWidth  - 20;
  lidarCanvas.height = panel.clientHeight - 30;
}

function drawLidar(angles, distances) {
  const W = lidarCanvas.width, H = lidarCanvas.height;
  const ctx = lidarCtx;
  ctx.clearRect(0, 0, W, H);
  ctx.fillStyle = C.bg;
  ctx.fillRect(0, 0, W, H);

  const cx = W / 2, cy = H / 2 + 10;
  const r  = Math.min(W, H) / 2 - 28;

  if (!angles || angles.length === 0) {
    ctx.fillStyle = C.gray;
    ctx.font = '14px monospace';
    ctx.textAlign = 'center';
    ctx.fillText('No lidar data', cx, cy);
    return;
  }

  const maxDist = Math.max(...distances) || 1;

  // Range rings
  for (const frac of [0.25, 0.5, 0.75, 1.0]) {
    ctx.beginPath();
    ctx.arc(cx, cy, r * frac, 0, Math.PI * 2);
    ctx.strokeStyle = C.border; ctx.lineWidth = 1; ctx.stroke();
    ctx.fillStyle = C.border;
    ctx.font = '10px monospace'; ctx.textAlign = 'left';
    ctx.fillText((maxDist * frac / 1000).toFixed(1) + 'm',
                 cx + r * frac + 3, cy - 5);
  }

  // Cardinal crosshairs
  ctx.strokeStyle = C.border; ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(cx, cy - r); ctx.lineTo(cx, cy + r);
  ctx.moveTo(cx - r, cy); ctx.lineTo(cx + r, cy);
  ctx.stroke();
  ctx.fillStyle = C.gray; ctx.font = '11px monospace';
  ctx.textAlign = 'center';
  ctx.fillText('N', cx, cy - r - 5);
  ctx.fillText('S', cx, cy + r + 13);
  ctx.textAlign = 'left';
  ctx.fillText('E', cx + r + 3, cy + 4);
  ctx.fillText('W', cx - r - 14, cy + 4);

  // Scan points: red (close) → cyan (far)
  for (let i = 0; i < angles.length; i++) {
    const dist = distances[i];
    if (dist <= 0) continue;
    const frac = Math.min(dist / maxDist, 1.0);
    const pr   = frac * r;
    const rad  = angles[i] * Math.PI / 180;
    const px   = cx + pr * Math.sin(rad);
    const py   = cy - pr * Math.cos(rad);
    const rr   = Math.round(220 * (1 - frac) + 60 * frac);
    const gg   = Math.round( 60 * (1 - frac) + 200 * frac);
    const bb   = Math.round( 60 * (1 - frac) + 220 * frac);
    ctx.beginPath();
    ctx.arc(px, py, 2, 0, Math.PI * 2);
    ctx.fillStyle = `rgb(${rr},${gg},${bb})`;
    ctx.fill();
  }

  // Robot dot
  ctx.beginPath();
  ctx.arc(cx, cy, 5, 0, Math.PI * 2);
  ctx.fillStyle = C.green; ctx.fill();
}

// ── Camera rotation (server-side) and ArUco toggle ────────────────────────────
// Rotation is applied in the camera thread; buttons send server commands.
// The rotation label and ArUco status are updated from incoming SSE state.

el('btn-ccw').onclick   = () => sendCmd('rotate_ccw');
el('btn-cw').onclick    = () => sendCmd('rotate_cw');
el('btn-aruco').onclick = () => sendCmd('aruco_toggle');

// Keyboard shortcuts mirroring robot_gui.py
document.addEventListener('keydown', e => {
  if (e.key === '[')                   sendCmd('rotate_ccw');
  if (e.key === ']')                   sendCmd('rotate_cw');
  if (e.key === 't' || e.key === 'T') sendCmd('aruco_toggle');
  if (e.key === 'e' || e.key === 'E') sendCmd('estop');
  if (e.key === 'r' || e.key === 'R') sendCmd('reset');
});

// ── Apply incoming state ──────────────────────────────────────────────────────
function applyState(s) {
  // Title bar
  const modeColor = MODE_COLORS[s.mode] || C.gray;
  el('titlebar').style.borderColor = modeColor;
  const modeEl = el('mode-lbl');
  modeEl.textContent = s.mode === 'AUTO' ? `AUTO·${s.auto_type}` : s.mode;
  modeEl.style.color = modeColor;

  const sc = s.speed_scale, sEl = el('speed-lbl');
  if      (sc < 0.45) { sEl.textContent = 'Slow';   sEl.style.color = C.cyan;   }
  else if (sc < 0.80) { sEl.textContent = 'Medium'; sEl.style.color = C.yellow; }
  else                { sEl.textContent = 'Fast';   sEl.style.color = C.green;  }

  // Drive
  updateBar('fill-l', 'val-l', s.drive.left);
  updateBar('fill-r', 'val-r', s.drive.right);

  // Telemetry
  const t = s.telemetry;
  el('t-volt').textContent  = fmt(t.voltage,    1, ' V');
  el('t-curr').textContent  = fmt(t.current,    2, ' A');
  el('t-board').textContent = fmt(t.board_temp, 0, '°C');
  el('t-ltmp').textContent  = fmt(t.left_temp,  0, '°C');
  el('t-rtmp').textContent  = fmt(t.right_temp, 0, '°C');
  const fault = t.left_fault || t.right_fault;
  const fEl = el('t-fault');
  fEl.textContent = (t.left_fault ? 'FAULT-L ' : '') + (t.right_fault ? 'FAULT-R' : '') || 'OK';
  fEl.style.color = fault ? C.red : C.green;
  el('telem-title').style.color = fault ? C.red : C.gray;

  // GPS
  const g = s.gps, gOk = s.gps_ok;
  const fixColor = FIX_COLORS[g.fix_quality] || C.gray;
  el('g-lat').textContent  = g.latitude  != null ? g.latitude.toFixed(7)   : '--';
  el('g-lon').textContent  = g.longitude != null ? g.longitude.toFixed(7)  : '--';
  el('g-alt').textContent  = g.altitude  != null ? g.altitude.toFixed(1) + ' m' : '--';
  el('g-herr').textContent = g.h_error_m != null ? g.h_error_m.toFixed(3) + ' m' : '--';
  el('g-sats').textContent = g.satellites != null
    ? g.satellites + (g.satellites_view ? '/' + g.satellites_view : '') : '--';
  el('g-hdop').textContent = fmt(g.hdop, 2);
  const fixEl = el('g-fix');
  fixEl.textContent = gOk ? g.fix_quality_name : 'No GPS';
  fixEl.style.color = gOk ? fixColor : C.gray;
  el('gps-panel').style.borderColor = s.gps_logging ? C.cyan : (gOk ? fixColor : C.border);
  el('gps-title').style.color       = gOk ? fixColor : C.gray;

  // System
  applySystem(s.system);

  // Lidar
  drawLidar(s.lidar.angles, s.lidar.distances);

  // Camera rotation label (updated from server state)
  el('cam-rot-lbl').textContent = s.cam_rotation ? `[${s.cam_rotation}°]` : '';

  // ArUco status
  const acoBtn    = el('btn-aruco');
  const acoInfoEl = el('cam-aruco-info');
  if (s.aruco_enabled) {
    acoBtn.textContent   = 'ArUco: ON';
    acoBtn.style.color   = C.green;
    acoBtn.style.borderColor = C.green;
    if (s.aruco) {
      acoInfoEl.textContent = `tags:${s.aruco.tag_count}  gates:${s.aruco.gate_count}  ${s.aruco.fps}fps`;
      acoInfoEl.style.color = C.green;
    } else {
      acoInfoEl.textContent = 'ACO: init…';
      acoInfoEl.style.color = C.yellow;
    }
  } else {
    acoBtn.textContent       = 'ArUco: OFF';
    acoBtn.style.color       = C.gray;
    acoBtn.style.borderColor = C.border;
    acoInfoEl.textContent    = '';
  }

  // Camera visibility
  el('cam-img').style.display      = s.camera_ok ? 'block' : 'none';
  el('cam-nosignal').style.display = s.camera_ok ? 'none'  : 'block';

  // Footer badges
  function badge(id, text, active, color) {
    const b = el(id); b.textContent = text; b.style.color = active ? color : C.gray;
  }
  badge('badge-rc',  `RC: ${s.rc_active  ? 'OK' : '--'}`,       s.rc_active,  C.green);
  badge('badge-cam', `CAM: ${s.camera_ok ? 'OK' : '--'}`,       s.camera_ok,  C.green);
  badge('badge-aco', `ACO: ${s.aruco_ok  ? 'OK' : '--'}`,       s.aruco_ok,   C.green);
  badge('badge-ldr', `LDR: ${s.lidar_ok  ? 'OK' : '--'}`,       s.lidar_ok,   C.green);
  badge('badge-gps', `GPS: ${gOk ? g.fix_quality_name : '--'}`, gOk,          fixColor);

  const herrEl = el('footer-herr');
  if (gOk && g.h_error_m != null) {
    herrEl.textContent = `H-err: ${g.h_error_m.toFixed(3)}m`;
    herrEl.style.color = C.cyan;
  } else {
    herrEl.textContent = '';
  }

  s_gps_logging = s.gps_logging;
  badge('badge-log', `LOG: ${s.gps_logging ? 'ON' : 'OFF'}`, s.gps_logging, C.cyan);
}

// ── System panel ─────────────────────────────────────────────────────────────
function pctColor(pct, warn, crit) {
  return pct >= crit ? C.red : pct >= warn ? C.yellow : C.green;
}
function sysBar(fillId, valId, pct, label, color) {
  const fill = el(fillId);
  fill.style.width           = Math.min(pct, 100) + '%';
  fill.style.backgroundColor = color;
  el(valId).textContent      = label;
  el(valId).style.color      = pct > 55 ? C.bg : color;  // invert text inside fill
}
function applySystem(s) {
  const cpuColor  = pctColor(s.cpu_percent,  60, 85);
  const tempColor = s.cpu_temp_c < 60 ? C.green : s.cpu_temp_c < 75 ? C.yellow : C.red;
  const memColor  = pctColor(s.mem_percent,  70, 90);
  const diskColor = pctColor(s.disk_percent, 70, 90);

  sysBar('sys-cpu-fill',  'sys-cpu-val',  s.cpu_percent,
         `${s.cpu_percent.toFixed(0)}%`, cpuColor);
  sysBar('sys-mem-fill',  'sys-mem-val',  s.mem_percent,
         `${s.mem_percent.toFixed(0)}%  ${(s.mem_used_mb/1024).toFixed(1)}/${(s.mem_total_mb/1024).toFixed(1)}GB`,
         memColor);
  sysBar('sys-disk-fill', 'sys-disk-val', s.disk_percent,
         `${s.disk_percent.toFixed(0)}%  ${s.disk_used_gb.toFixed(1)}/${s.disk_total_gb.toFixed(1)}GB`,
         diskColor);

  const tempEl = el('sys-temp');
  tempEl.textContent = `${s.cpu_temp_c.toFixed(0)}°C`;
  tempEl.style.color = tempColor;
  el('sys-freq').textContent = `${s.cpu_freq_mhz.toFixed(0)} MHz`;
  el('sys-panel').style.borderColor = s.cpu_temp_c >= 75 ? C.red
                                    : s.cpu_temp_c >= 60 ? C.yellow : C.border;
  el('sys-title').style.color       = s.cpu_temp_c >= 60 ? tempColor : C.gray;
}

// ── GPS logging state (for key guard) ────────────────────────────────────────
let s_gps_logging = false;

// ── SSE connection ────────────────────────────────────────────────────────────
let evtSrc = null;

function connect() {
  if (evtSrc) evtSrc.close();
  const badge = el('conn-badge');
  badge.textContent = '🟡 Connecting…'; badge.style.color = C.yellow;

  evtSrc = new EventSource('/api/state');
  evtSrc.onopen = () => {
    badge.textContent = '🟢 Live'; badge.style.color = C.green;
  };
  evtSrc.onmessage = e => {
    try { applyState(JSON.parse(e.data)); }
    catch (err) { console.error('SSE parse:', err); }
  };
  evtSrc.onerror = () => {
    badge.textContent = '🔴 Reconnecting…'; badge.style.color = C.red;
    evtSrc.close();
    setTimeout(connect, 3000);
  };
}

// ── Commands ──────────────────────────────────────────────────────────────────
async function sendCmd(cmd) {
  try {
    await fetch('/api/cmd', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ cmd }),
    });
  } catch (e) { console.error('cmd failed:', e); }
}
el('btn-estop').onclick = () => sendCmd('estop');
el('btn-reset').onclick = () => sendCmd('reset');

// ── Init ──────────────────────────────────────────────────────────────────────
function onResize() { resizeLidar(); }
window.addEventListener('resize', onResize);
onResize();
connect();
</script>
</body>
</html>"""


# ── Flask app ─────────────────────────────────────────────────────────────────

app    = Flask(__name__)
_robot = None   # set in main()


@app.route('/')
def index():
    return Response(_HTML, mimetype='text/html')


@app.route('/stream')
def camera_stream():
    """MJPEG camera stream — connect with <img src="/stream">."""
    def _gen():
        while True:
            frame = _robot.get_frame()
            if frame is not None and _jpeg_encode is not None:
                try:
                    data = _jpeg_encode(frame)
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + data + b'\r\n')
                except Exception:
                    pass
            time.sleep(0.05)   # cap at ~20 fps

    return Response(
        _gen(),
        mimetype='multipart/x-mixed-replace; boundary=frame',
        headers={'Cache-Control': 'no-store'},
    )


@app.route('/api/state')
def api_state():
    """SSE stream of robot state JSON at 10 Hz."""
    def _gen():
        while True:
            try:
                data = _serialise(
                    _robot.get_state(),
                    cam_rotation  = _robot.get_cam_rotation(),
                    aruco_enabled = _robot.get_aruco_enabled(),
                    aruco_state   = _robot.get_aruco_state(),
                )
                yield f"data: {json.dumps(data)}\n\n"
            except Exception as exc:
                yield f"data: {json.dumps({'error': str(exc)})}\n\n"
            time.sleep(0.1)

    return Response(
        _gen(),
        mimetype='text/event-stream',
        headers={'Cache-Control': 'no-cache', 'X-Accel-Buffering': 'no'},
    )


@app.route('/api/cmd', methods=['POST'])
def api_cmd():
    cmd = (request.json or {}).get('cmd', '')
    if cmd == 'estop':
        _robot.estop()
    elif cmd == 'reset':
        _robot.reset_estop()
    elif cmd == 'aruco_toggle':
        _robot.toggle_aruco()
    elif cmd == 'gps_bookmark':
        _robot.bookmark_gps()
    elif cmd == 'rotate_cw':
        _robot.set_cam_rotation((_robot.get_cam_rotation() + 90) % 360)
    elif cmd == 'rotate_ccw':
        _robot.set_cam_rotation((_robot.get_cam_rotation() - 90 + 360) % 360)
    else:
        return jsonify({'ok': False, 'error': f'Unknown command: {cmd}'}), 400
    return jsonify({'ok': True})


# ── Config helpers (mirrors robot_gui.py) ─────────────────────────────────────

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


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    global _robot, _jpeg_encode

    DEFAULT_CFG = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'robot.ini')

    parser = argparse.ArgumentParser(description='Yukon robot web dashboard')
    parser.add_argument('--config',        default=DEFAULT_CFG, help='Config file (default: robot.ini)')
    parser.add_argument('--host',          default=None,        help='Bind address')
    parser.add_argument('--port',          default=None, type=int, help='HTTP port')
    parser.add_argument('--debug',         action='store_true', default=None, help='Flask debug mode')
    parser.add_argument('--yukon-port',    default=None, help='Yukon serial port')
    parser.add_argument('--ibus-port',     default=None)
    parser.add_argument('--gps-port',      default=None)
    parser.add_argument('--lidar-port',    default=None)
    parser.add_argument('--ntrip-host',    default=None)
    parser.add_argument('--ntrip-port',    default=None, type=int, dest='ntrip_port_arg')
    parser.add_argument('--ntrip-mount',   default=None)
    parser.add_argument('--ntrip-user',    default=None)
    parser.add_argument('--ntrip-password', default=None)
    parser.add_argument('--rtcm-port',     default=None)
    parser.add_argument('--rtcm-baud',     default=None, type=int)
    parser.add_argument('--no-camera',     action='store_true', default=False)
    parser.add_argument('--no-lidar',      action='store_true', default=False)
    parser.add_argument('--no-gps',        action='store_true', default=False)
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format='%(levelname)s %(name)s: %(message)s')
    cfg = _load_config(args.config)

    web_host  = args.host  if args.host  is not None else _cfg(cfg, 'web', 'host',  '0.0.0.0')
    web_port  = args.port  if args.port  is not None else _cfg(cfg, 'web', 'port',  5000, int)
    web_debug = args.debug if args.debug is not None else _cfg(cfg, 'web', 'debug', False, lambda x: x.lower() == 'true')
    _jpeg_encode = _make_jpeg_encoder()

    bool_val = lambda x: x.lower() == 'true'
    port_val = lambda x: None if x.lower() in ('auto', '') else x

    def arg(cli_val, section, key, fallback, cast=str):
        if cli_val is not None:
            return cli_val
        return _cfg(cfg, section, key, fallback, cast)

    _robot = Robot(
        yukon_port     = arg(args.yukon_port,      'robot', 'yukon_port',  None,  port_val),
        ibus_port      = arg(args.ibus_port,        'robot', 'ibus_port',   '/dev/ttyAMA3'),
        lidar_port     = arg(args.lidar_port,       'lidar', 'port',        '/dev/ttyUSB0'),
        gps_port       = arg(args.gps_port,         'gps',   'port',        '/dev/ttyUSB0'),
        ntrip_host     = '' if _cfg(cfg, 'ntrip', 'disabled', False, bool_val) else arg(args.ntrip_host,     'ntrip', 'host',  ''),
        ntrip_port     = arg(args.ntrip_port_arg,   'ntrip', 'port',        2101,  int),
        ntrip_mount    = arg(args.ntrip_mount,      'ntrip', 'mount',       ''),
        ntrip_user     = arg(args.ntrip_user,       'ntrip', 'user',        ''),
        ntrip_password = arg(args.ntrip_password,   'ntrip', 'password',    ''),
        rtcm_port      = '' if _cfg(cfg, 'rtcm', 'disabled', False, bool_val) else arg(args.rtcm_port, 'rtcm', 'port', ''),
        rtcm_baud      = arg(args.rtcm_baud,        'rtcm',  'baud',        115200, int),
        enable_camera  = not (args.no_camera or _cfg(cfg, 'camera', 'disabled', False, bool_val)),
        enable_lidar   = not (args.no_lidar  or _cfg(cfg, 'lidar',  'disabled', False, bool_val)),
        enable_gps     = not (args.no_gps    or _cfg(cfg, 'gps',    'disabled', False, bool_val)),
        cam_width      = _cfg(cfg, 'camera', 'width',    640, int),
        cam_height     = _cfg(cfg, 'camera', 'height',  480, int),
        cam_fps        = _cfg(cfg, 'camera', 'fps',     30,  int),
        cam_rotation   = _cfg(cfg, 'camera', 'rotation', 0,  int),
        enable_aruco   = _cfg(cfg, 'aruco',  'enabled', False, bool_val),
        aruco_dict     = _cfg(cfg, 'aruco',  'dict',    'DICT_4X4_1000'),
        throttle_ch    = _cfg(cfg, 'rc', 'throttle_ch',  3,    int),
        steer_ch       = _cfg(cfg, 'rc', 'steer_ch',     1,    int),
        mode_ch        = _cfg(cfg, 'rc', 'mode_ch',      5,    int),
        speed_ch       = _cfg(cfg, 'rc', 'speed_ch',     6,    int),
        auto_type_ch   = _cfg(cfg, 'rc', 'auto_type_ch', 7,    int),
        gps_log_ch      = _cfg(cfg, 'rc', 'gps_log_ch',      8,   int),
        gps_bookmark_ch = _cfg(cfg, 'rc', 'gps_bookmark_ch', 2,   int),
        gps_log_dir     = _cfg(cfg, 'gps', 'log_dir',        ''),
        gps_log_hz      = _cfg(cfg, 'gps', 'log_hz',         5.0, float),
        deadzone       = _cfg(cfg, 'rc', 'deadzone',    30,   int),
        failsafe_s     = _cfg(cfg, 'rc', 'failsafe_s',  0.5,  float),
        speed_min      = _cfg(cfg, 'rc', 'speed_min',   0.25, float),
        control_hz     = _cfg(cfg, 'rc', 'control_hz',  50,   int),
    )

    _robot.start()
    log.info(f'Web dashboard → http://{web_host}:{web_port}/')
    try:
        app.run(
            host=web_host, port=web_port,
            debug=web_debug, threaded=True, use_reloader=False,
        )
    finally:
        _robot.stop()


if __name__ == '__main__':
    main()
