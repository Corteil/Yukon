#!/usr/bin/env python3
"""
robot_web.py — Flask web dashboard for the Yukon robot.

Mirrors robot_gui.py features:
  · Real-time telemetry via Server-Sent Events (10 Hz)
  · Live camera as MJPEG stream with toggleable bearing overlay
  · Lidar polar plot on Canvas
  · IMU heading display (compass rose + numeric)
  · Navigator state + target gate badge
  · ESTOP / Reset controls
  · Camera rotation, ArUco toggle, bearing overlay toggle
  · No-motors warning banner
  · GPS logging badge
  · Mobile-responsive layout

Usage:
  python3 robot_web.py                # listens on 0.0.0.0:5000
  python3 robot_web.py --port 8080
  python3 robot_web.py --no-motors    # bench-test mode

Dependencies:
  pip install flask
  pip install pillow   # or: pip install opencv-python
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
        def _enc(frame, quality=82):
            _, buf = cv2.imencode('.jpg', frame[:, :, ::-1],
                                  [cv2.IMWRITE_JPEG_QUALITY, quality])
            return buf.tobytes()
        log.info("JPEG encoder: OpenCV")
        return _enc
    except ImportError:
        pass
    try:
        from PIL import Image as _PIL
        def _enc(frame, quality=82):
            buf = io.BytesIO()
            _PIL.fromarray(frame).save(buf, format='JPEG', quality=quality)
            return buf.getvalue()
        log.info("JPEG encoder: Pillow")
        return _enc
    except ImportError:
        pass
    log.warning("No JPEG encoder — camera stream disabled (pip install pillow)")
    return None

_jpeg_encode = None  # set in main()

# ── State serialiser ──────────────────────────────────────────────────────────

def _serialise(state, cam_rotation=0, aruco_enabled=False,
               aruco_state=None, nav_bearing_err=None):
    g, t, d, li, s = (state.gps, state.telemetry, state.drive,
                       state.lidar, state.system)

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
                'gate_id':     gate.gate_id,
                'centre_x':   gate.centre_x,
                'centre_y':   gate.centre_y,
                'bearing':    gate.bearing,
                'distance':   gate.distance,
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
        'mode':        state.mode.name,
        'auto_type':   state.auto_type.label,
        'speed_scale': state.speed_scale,
        'rc_active':   state.rc_active,
        'camera_ok':   state.camera_ok,
        'lidar_ok':    state.lidar_ok,
        'gps_ok':      state.gps_ok,
        'aruco_ok':    state.aruco_ok,
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
            'latitude':       g.latitude,
            'longitude':      g.longitude,
            'altitude':       g.altitude,
            'fix_quality':    g.fix_quality,
            'fix_quality_name': g.fix_quality_name,
            'h_error_m':      g.h_error_m,
            'satellites':     g.satellites,
            'satellites_view': g.satellites_view,
            'hdop':           g.hdop,
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

# ── Embedded HTML ─────────────────────────────────────────────────────────────

_HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
<title>Yukon Robot Monitor</title>
<style>
*,*::before,*::after{box-sizing:border-box;margin:0;padding:0}
:root{
  --bg:#12121e;--panel:#1c1c2d;--border:#3c3c5a;
  --white:#e6e6f0;--gray:#82829a;
  --green:#3cdc50;--yellow:#f0c828;--orange:#f08c28;
  --red:#dc3c3c;--cyan:#3cc8dc;--purple:#b450dc;
}
html,body{height:100%;background:var(--bg);color:var(--white);
  font-family:'Courier New',Courier,monospace;overflow:hidden}
body{display:flex;flex-direction:column;gap:5px;padding:5px;height:100vh}
.panel{background:var(--panel);border:1px solid var(--border);
  border-radius:8px;padding:5px 9px}
.ptitle{font-size:11px;color:var(--gray);margin-bottom:3px}
.field label{font-size:10px;color:var(--gray);display:block}
.field span{font-size:14px}

/* ── Title bar ── */
#titlebar{display:flex;align-items:center;gap:12px;padding:6px 12px;flex-shrink:0}
#main-title{font-size:18px;font-weight:bold;flex:1}
.info{display:flex;gap:18px;font-size:14px}
.info span b{font-size:16px}
.controls{display:flex;gap:6px}
button{font-family:inherit;font-size:12px;padding:4px 10px;
  border-radius:5px;border:1px solid var(--border);
  background:var(--panel);color:var(--white);cursor:pointer;touch-action:manipulation}
button:active{background:#2a2a42}
#btn-estop{border-color:var(--red);color:var(--red)}
#btn-reset{border-color:var(--green);color:var(--green)}

/* ── No-motors banner ── */
#no-motors-banner{
  display:none;background:#3a0000;border:1px solid var(--red);
  border-radius:6px;padding:4px 12px;text-align:center;
  color:var(--red);font-size:13px;font-weight:bold;flex-shrink:0
}

/* ── Panels row ── */
#panels-row{display:flex;gap:5px;flex-shrink:0;height:185px}
#drive-panel{width:210px;flex-shrink:0}
#telem-panel{width:255px;flex-shrink:0}
#gps-panel{flex:1}
#sys-panel{width:200px;flex-shrink:0}

/* Motor bars */
.mrow{display:flex;align-items:center;gap:7px;margin-top:10px}
.mlabel{width:12px;font-size:12px;color:var(--gray)}
.mtrack{flex:1;height:38px;background:var(--bg);border-radius:4px;position:relative}
.mmid{position:absolute;left:50%;top:4px;bottom:4px;width:1px;background:var(--border)}
.mfill{position:absolute;top:4px;bottom:4px;border-radius:3px;transition:all .06s linear}
.mval{position:absolute;right:5px;top:50%;transform:translateY(-50%);
  font-size:11px;pointer-events:none}

/* Telemetry */
.tgrid{display:grid;grid-template-columns:1fr 1fr;gap:5px 12px;margin-top:5px}
.tgrid.three{grid-template-columns:1fr 1fr 1fr}

/* GPS */
.ggrid{display:grid;grid-template-columns:1fr 1fr;gap:4px 12px;margin-top:5px}
.ggrid.three{grid-template-columns:1fr 1fr 1fr}

/* System */
.sys-row{display:flex;align-items:center;gap:4px;margin-top:7px}
.sys-row label{font-size:10px;color:var(--gray);width:28px;flex-shrink:0}
.sys-track{flex:1;height:15px;background:var(--bg);border-radius:3px;
  position:relative;overflow:hidden}
.sys-fill{position:absolute;left:0;top:0;bottom:0;border-radius:3px;
  transition:width .3s ease}
.sys-val{position:absolute;right:3px;top:50%;transform:translateY(-50%);
  font-size:10px;pointer-events:none}

/* ── Body row ── */
#body-row{display:flex;gap:5px;flex:1;min-height:0}
#camera-panel,#lidar-panel{flex:1;display:flex;flex-direction:column}
.cam-header{display:flex;align-items:center;gap:6px;margin-bottom:3px;flex-wrap:wrap}
.cam-header .ptitle{margin:0;flex:1}
.cam-header button{padding:2px 7px;font-size:11px}
#cam-wrap{flex:1;overflow:hidden;display:flex;align-items:center;
  justify-content:center;background:var(--bg);border-radius:4px;position:relative}
#cam-img{max-width:100%;max-height:100%;object-fit:contain}
#cam-nosignal{color:var(--gray);font-size:13px;display:none}
#bearing-canvas{position:absolute;top:0;left:0;width:100%;height:100%;
  pointer-events:none;display:none}
#nav-badge{position:absolute;top:6px;right:6px;font-size:10px;
  background:rgba(18,18,30,.8);padding:2px 6px;border-radius:3px;display:none}
#lidar-canvas{width:100%;flex:1;display:block}

/* ── Footer ── */
#footer{display:flex;align-items:center;gap:14px;padding:2px 7px;
  border-top:1px solid var(--border);flex-shrink:0;font-size:12px;flex-wrap:wrap}
#conn-badge{margin-left:auto;font-size:11px;color:var(--gray)}

/* ── Compass ── */
#compass-wrap{display:inline-flex;align-items:center;gap:6px}
#compass-canvas{width:36px;height:36px}
#hdg-text{font-size:13px;color:var(--cyan)}

/* Mobile breakpoint */
@media(max-width:700px){
  #panels-row{height:auto;flex-direction:column}
  #drive-panel,#telem-panel,#sys-panel{width:100%}
  #body-row{flex-direction:column}
  body{overflow-y:auto;height:auto}
}
</style>
</head>
<body>

<!-- Title bar -->
<div class="panel" id="titlebar">
  <span id="main-title">Yukon Robot Monitor</span>
  <div class="info">
    <span>Speed:&nbsp;<b id="speed-lbl" style="color:var(--cyan)">--</b></span>
    <span>Mode:&nbsp;<b id="mode-lbl">--</b></span>
  </div>
  <div class="controls">
    <button id="btn-estop">ESTOP</button>
    <button id="btn-reset">Reset</button>
  </div>
</div>

<!-- No-motors banner -->
<div id="no-motors-banner">⚠ NO-MOTORS MODE — drive commands suppressed</div>

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
      <div class="field"><label>Board</label> <span id="t-board">--</span></div>
      <div class="field"><label>Left</label>  <span id="t-ltmp">--</span></div>
      <div class="field"><label>Right</label> <span id="t-rtmp">--</span></div>
    </div>
    <div class="tgrid" style="margin-top:6px">
      <div class="field">
        <label>IMU Heading</label>
        <div id="compass-wrap">
          <canvas id="compass-canvas" width="36" height="36"></canvas>
          <span id="hdg-text">---</span>
        </div>
      </div>
      <div class="field">
        <label>Faults</label>
        <span id="t-fault" style="color:var(--green)">OK</span>
      </div>
    </div>
  </div>

  <!-- GPS -->
  <div class="panel" id="gps-panel">
    <div class="ptitle" id="gps-title">GPS</div>
    <div class="ggrid">
      <div class="field"><label>Latitude</label>  <span id="g-lat">--</span></div>
      <div class="field"><label>Altitude</label>  <span id="g-alt">--</span></div>
      <div class="field"><label>Longitude</label> <span id="g-lon">--</span></div>
      <div class="field"><label>H-Error</label>   <span id="g-herr">--</span></div>
    </div>
    <div class="ggrid three" style="margin-top:5px">
      <div class="field"><label>Fix</label>  <span id="g-fix">--</span></div>
      <div class="field"><label>Sats</label> <span id="g-sats">--</span></div>
      <div class="field"><label>HDOP</label> <span id="g-hdop">--</span></div>
    </div>
  </div>

  <!-- System -->
  <div class="panel" id="sys-panel">
    <div class="ptitle" id="sys-title">System</div>
    <div class="sys-row">
      <label>CPU</label>
      <div class="sys-track"><div class="sys-fill" id="sys-cpu-fill"></div>
        <span class="sys-val" id="sys-cpu-val">--</span></div>
    </div>
    <div class="sys-row" style="margin-top:3px">
      <label>Temp</label>
      <span id="sys-temp" style="font-size:14px;margin-left:5px">--</span>
      <span id="sys-freq" style="font-size:10px;color:var(--gray);margin-left:8px">--</span>
    </div>
    <div class="sys-row" style="margin-top:8px">
      <label>Mem</label>
      <div class="sys-track"><div class="sys-fill" id="sys-mem-fill"></div>
        <span class="sys-val" id="sys-mem-val">--</span></div>
    </div>
    <div class="sys-row" style="margin-top:8px">
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
      <span id="cam-aruco-info" style="font-size:10px;color:var(--gray);margin-right:auto"></span>
      <button id="btn-bearing" title="Toggle bearing overlay (B key)">Bearing: OFF</button>
      <button id="btn-aruco"   title="Toggle ArUco (T key)">ArUco: OFF</button>
      <button id="btn-ccw"     title="Rotate CCW ([ key)">↺</button>
      <button id="btn-cw"      title="Rotate CW (] key)">↻</button>
    </div>
    <div id="cam-wrap">
      <img id="cam-img" src="/stream" alt="Camera feed">
      <span id="cam-nosignal">No camera</span>
      <canvas id="bearing-canvas"></canvas>
      <div id="nav-badge"></div>
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
  <span id="badge-hdg" style="display:none"></span>
  <span id="badge-nav" style="display:none"></span>
  <span id="conn-badge">⚫ Connecting…</span>
</div>

<script>
"use strict";

// ── Palette ───────────────────────────────────────────────────────────────────
const C = {
  green:'#3cdc50', yellow:'#f0c828', orange:'#f08c28',
  red:'#dc3c3c',   cyan:'#3cc8dc',   gray:'#82829a',
  border:'#3c3c5a', bg:'#12121e', white:'#e6e6f0',
};
const MODE_COLORS = { MANUAL:C.yellow, AUTO:C.green, ESTOP:C.red };
const FIX_COLORS  = {0:C.red,1:C.yellow,2:C.orange,3:C.orange,4:C.green,5:C.cyan};
const NAV_COLORS  = {
  SEARCHING:C.yellow, ALIGNING:C.orange, APPROACHING:C.green,
  PASSING:C.cyan, COMPLETE:C.green, ERROR:C.red, IDLE:C.gray,
  WAITING_FIX:C.yellow, NAVIGATING:C.green, ARRIVED:C.cyan,
};

// ── Helpers ───────────────────────────────────────────────────────────────────
const el  = id => document.getElementById(id);
const fmt = (v, d, u) => v == null ? '--' : v.toFixed(d) + (u || '');

// ── State ─────────────────────────────────────────────────────────────────────
let showBearing   = false;
let lastState     = null;
let camNatW = 640, camNatH = 480;  // natural frame size (updated from stream)

// ── Motor bars ────────────────────────────────────────────────────────────────
function updateBar(fillId, valId, v) {
  const fill = el(fillId), valEl = el(valId);
  const color = v > 0 ? C.green : v < 0 ? C.orange : null;
  if (v >= 0) { fill.style.left='50%';  fill.style.width=(v*50)+'%'; }
  else        { fill.style.left=(50+v*50)+'%'; fill.style.width=(-v*50)+'%'; }
  fill.style.backgroundColor = color || 'transparent';
  valEl.textContent = (v>=0?'+':'')+v.toFixed(2);
  valEl.style.color = color || C.gray;
}

// ── Compass rose ──────────────────────────────────────────────────────────────
const compassCanvas = el('compass-canvas');
const compassCtx    = compassCanvas.getContext('2d');

function drawCompass(heading) {
  const ctx = compassCtx, w = 36, h = 36, cx = w/2, cy = h/2, r = 15;
  ctx.clearRect(0,0,w,h);
  ctx.beginPath(); ctx.arc(cx,cy,r,0,Math.PI*2);
  ctx.strokeStyle = C.border; ctx.lineWidth = 1; ctx.stroke();
  // N tick
  ctx.strokeStyle = C.red; ctx.lineWidth = 2;
  ctx.beginPath(); ctx.moveTo(cx,cy-r+2); ctx.lineTo(cx,cy-r+7); ctx.stroke();
  // Heading needle
  const rad = (heading - 90) * Math.PI / 180;
  ctx.strokeStyle = C.cyan; ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(cx, cy);
  ctx.lineTo(cx + r*Math.cos(rad), cy + r*Math.sin(rad));
  ctx.stroke();
  ctx.beginPath(); ctx.arc(cx,cy,2,0,Math.PI*2);
  ctx.fillStyle = C.cyan; ctx.fill();
}

// ── Bearing overlay on camera ─────────────────────────────────────────────────
const bearingCanvas = el('bearing-canvas');
const bearingCtx    = bearingCanvas.getContext('2d');

function resizeBearingCanvas() {
  const wrap = el('cam-wrap');
  const img  = el('cam-img');
  bearingCanvas.width  = img.clientWidth  || wrap.clientWidth;
  bearingCanvas.height = img.clientHeight || wrap.clientHeight;
}

function drawBearingOverlay(aruco, navGate, heading) {
  resizeBearingCanvas();
  const ctx = bearingCtx;
  const W   = bearingCanvas.width, H = bearingCanvas.height;
  ctx.clearRect(0, 0, W, H);

  if (!aruco) return;

  // Scale from frame coords to canvas coords
  const sx = W / camNatW, sy = H / camNatH;
  const fcx = W/2, fcy = H/2;

  const targetOdd  = navGate*2 + 1;
  const targetEven = navGate*2 + 2;

  ctx.font = 'bold 11px monospace';
  ctx.lineWidth = 1.5;

  // Draw each visible tag
  for (const tag of aruco.tags) {
    const tx = tag.cx * sx, ty = tag.cy * sy;
    const isTarget = tag.id === targetOdd || tag.id === targetEven;
    const color = isTarget ? C.cyan : C.yellow;

    // Line from centre
    ctx.strokeStyle = color; ctx.globalAlpha = 0.7;
    ctx.beginPath(); ctx.moveTo(fcx, fcy); ctx.lineTo(tx, ty); ctx.stroke();
    ctx.globalAlpha = 1.0;

    // Tag dot
    ctx.beginPath(); ctx.arc(tx, ty, 4, 0, Math.PI*2);
    ctx.strokeStyle = color; ctx.lineWidth = 2; ctx.stroke();

    // Label
    let lbl = `#${tag.id}`;
    if (tag.bearing  != null) lbl += ` ${tag.bearing >= 0 ? '+' : ''}${tag.bearing.toFixed(1)}°`;
    if (tag.distance != null) lbl += ` ${tag.distance.toFixed(2)}m`;

    const lx = tx < fcx + W/3 ? tx + 7 : tx - ctx.measureText(lbl).width - 7;
    const ly = ty - 6;
    ctx.fillStyle = 'rgba(18,18,30,0.75)';
    ctx.fillRect(lx-2, ly-12, ctx.measureText(lbl).width+4, 14);
    ctx.fillStyle = color;
    ctx.fillText(lbl, lx, ly);
  }

  // Gate aim crosshair
  for (const gate of aruco.gates) {
    if (gate.gate_id !== navGate) continue;
    const gx = gate.centre_x * sx, gy = gate.centre_y * sy;
    // Aim line
    ctx.strokeStyle = C.green; ctx.lineWidth = 1.5; ctx.globalAlpha = 0.8;
    ctx.beginPath(); ctx.moveTo(fcx, fcy); ctx.lineTo(gx, gy); ctx.stroke();
    ctx.globalAlpha = 1.0;
    // Crosshair
    const sz = 10;
    ctx.strokeStyle = C.green; ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(gx-sz, gy); ctx.lineTo(gx+sz, gy);
    ctx.moveTo(gx, gy-sz); ctx.lineTo(gx, gy+sz);
    ctx.stroke();
    ctx.beginPath(); ctx.arc(gx, gy, sz, 0, Math.PI*2);
    ctx.stroke();
    // Label
    let glbl = 'AIM';
    if (gate.bearing  != null) glbl += ` ${gate.bearing >= 0 ? '+' : ''}${gate.bearing.toFixed(1)}°`;
    if (gate.distance != null) glbl += ` ${gate.distance.toFixed(2)}m`;
    ctx.fillStyle = 'rgba(18,18,30,0.75)';
    ctx.fillRect(gx+2, gy-22, ctx.measureText(glbl).width+4, 14);
    ctx.fillStyle = C.green;
    ctx.fillText(glbl, gx+4, gy-10);
  }

  // Boresight crosshair
  ctx.strokeStyle = C.gray; ctx.lineWidth = 1; ctx.globalAlpha = 0.5;
  ctx.beginPath();
  ctx.moveTo(fcx-12, fcy); ctx.lineTo(fcx+12, fcy);
  ctx.moveTo(fcx, fcy-12); ctx.lineTo(fcx, fcy+12);
  ctx.stroke();
  ctx.beginPath(); ctx.arc(fcx, fcy, 4, 0, Math.PI*2); ctx.stroke();
  ctx.globalAlpha = 1.0;

  // IMU compass arc (bottom centre)
  if (heading != null) {
    const ccx = W/2, ccy = H - 26, cr = 16;
    ctx.beginPath(); ctx.arc(ccx, ccy, cr+2, 0, Math.PI*2);
    ctx.fillStyle='rgba(18,18,30,0.8)'; ctx.fill();
    ctx.beginPath(); ctx.arc(ccx, ccy, cr, 0, Math.PI*2);
    ctx.strokeStyle = C.border; ctx.lineWidth=1; ctx.stroke();
    // N tick
    ctx.strokeStyle=C.red; ctx.lineWidth=2;
    ctx.beginPath(); ctx.moveTo(ccx, ccy-cr+2); ctx.lineTo(ccx, ccy-cr+7); ctx.stroke();
    // Needle
    const nrad = (heading-90)*Math.PI/180;
    ctx.strokeStyle=C.cyan; ctx.lineWidth=2;
    ctx.beginPath();
    ctx.moveTo(ccx, ccy);
    ctx.lineTo(ccx+cr*Math.cos(nrad), ccy+cr*Math.sin(nrad));
    ctx.stroke();
    ctx.font='10px monospace'; ctx.fillStyle=C.cyan; ctx.textAlign='center';
    ctx.fillText(heading.toFixed(0)+'°', ccx, ccy-cr-4);
    ctx.textAlign='left';
  }
}

// ── Lidar canvas ──────────────────────────────────────────────────────────────
const lidarCanvas = el('lidar-canvas');
const lidarCtx    = lidarCanvas.getContext('2d');

function resizeLidar() {
  const panel = el('lidar-panel');
  lidarCanvas.width  = panel.clientWidth  - 18;
  lidarCanvas.height = panel.clientHeight - 28;
}

function drawLidar(angles, distances) {
  const W=lidarCanvas.width, H=lidarCanvas.height, ctx=lidarCtx;
  ctx.clearRect(0,0,W,H);
  ctx.fillStyle=C.bg; ctx.fillRect(0,0,W,H);
  const cx=W/2, cy=H/2+10, r=Math.min(W,H)/2-26;
  if (!angles || angles.length===0) {
    ctx.fillStyle=C.gray; ctx.font='13px monospace';
    ctx.textAlign='center'; ctx.fillText('No lidar data',cx,cy); return;
  }
  const maxDist = Math.max(...distances)||1;
  for (const frac of [0.25,0.5,0.75,1.0]) {
    ctx.beginPath(); ctx.arc(cx,cy,r*frac,0,Math.PI*2);
    ctx.strokeStyle=C.border; ctx.lineWidth=1; ctx.stroke();
    ctx.fillStyle=C.border; ctx.font='10px monospace'; ctx.textAlign='left';
    ctx.fillText((maxDist*frac/1000).toFixed(1)+'m', cx+r*frac+3, cy-5);
  }
  ctx.strokeStyle=C.border; ctx.lineWidth=1;
  ctx.beginPath(); ctx.moveTo(cx,cy-r); ctx.lineTo(cx,cy+r);
  ctx.moveTo(cx-r,cy); ctx.lineTo(cx+r,cy); ctx.stroke();
  ctx.fillStyle=C.gray; ctx.font='11px monospace'; ctx.textAlign='center';
  ctx.fillText('N',cx,cy-r-5); ctx.fillText('S',cx,cy+r+13);
  ctx.textAlign='left'; ctx.fillText('E',cx+r+3,cy+4);
  ctx.fillText('W',cx-r-14,cy+4);
  for (let i=0;i<angles.length;i++) {
    const dist=distances[i]; if(dist<=0) continue;
    const frac=Math.min(dist/maxDist,1.0), pr=frac*r;
    const rad=angles[i]*Math.PI/180;
    const px=cx+pr*Math.sin(rad), py=cy-pr*Math.cos(rad);
    const rr=Math.round(220*(1-frac)+60*frac);
    const gg=Math.round( 60*(1-frac)+200*frac);
    const bb=Math.round( 60*(1-frac)+220*frac);
    ctx.beginPath(); ctx.arc(px,py,2,0,Math.PI*2);
    ctx.fillStyle=`rgb(${rr},${gg},${bb})`; ctx.fill();
  }
  ctx.beginPath(); ctx.arc(cx,cy,5,0,Math.PI*2);
  ctx.fillStyle=C.green; ctx.fill();
}

// ── Apply incoming state ──────────────────────────────────────────────────────
function applyState(s) {
  lastState = s;

  // No-motors banner
  const banner = el('no-motors-banner');
  banner.style.display = s.no_motors ? 'block' : 'none';

  // Title bar
  const modeColor = MODE_COLORS[s.mode] || C.gray;
  el('titlebar').style.borderColor = modeColor;
  const modeEl = el('mode-lbl');
  modeEl.textContent = s.mode==='AUTO' ? `AUTO·${s.auto_type}` : s.mode;
  modeEl.style.color  = modeColor;
  const sc=s.speed_scale, sEl=el('speed-lbl');
  if      (sc<0.45){ sEl.textContent='Slow';   sEl.style.color=C.cyan;   }
  else if (sc<0.80){ sEl.textContent='Medium'; sEl.style.color=C.yellow; }
  else             { sEl.textContent='Fast';   sEl.style.color=C.green;  }

  // Drive
  updateBar('fill-l','val-l',s.drive.left);
  updateBar('fill-r','val-r',s.drive.right);

  // Telemetry
  const t=s.telemetry;
  el('t-volt').textContent  = fmt(t.voltage,1,' V');
  el('t-curr').textContent  = fmt(t.current,2,' A');
  el('t-board').textContent = fmt(t.board_temp,0,'°C');
  el('t-ltmp').textContent  = fmt(t.left_temp,0,'°C');
  el('t-rtmp').textContent  = fmt(t.right_temp,0,'°C');
  const fault=t.left_fault||t.right_fault;
  const fEl=el('t-fault');
  fEl.textContent=(t.left_fault?'FAULT-L ':'')+(t.right_fault?'FAULT-R':'')||'OK';
  fEl.style.color=fault?C.red:C.green;
  el('telem-title').style.color=fault?C.red:C.gray;

  // IMU heading
  const hdgEl=el('hdg-text');
  if (t.heading != null) {
    hdgEl.textContent = t.heading.toFixed(1)+'°';
    hdgEl.style.color = C.cyan;
    drawCompass(t.heading);
    el('compass-canvas').style.opacity='1';
  } else {
    hdgEl.textContent = '---';
    hdgEl.style.color = C.gray;
    el('compass-canvas').style.opacity='0.3';
  }

  // GPS
  const g=s.gps, gOk=s.gps_ok;
  const fixColor=FIX_COLORS[g.fix_quality]||C.gray;
  el('g-lat').textContent  = g.latitude  != null ? g.latitude.toFixed(7)  : '--';
  el('g-lon').textContent  = g.longitude != null ? g.longitude.toFixed(7) : '--';
  el('g-alt').textContent  = g.altitude  != null ? g.altitude.toFixed(1)+' m' : '--';
  el('g-herr').textContent = g.h_error_m != null ? g.h_error_m.toFixed(3)+' m' : '--';
  el('g-sats').textContent = g.satellites != null
    ? g.satellites+(g.satellites_view ? '/'+g.satellites_view : '') : '--';
  el('g-hdop').textContent = fmt(g.hdop,2);
  const fixEl=el('g-fix');
  fixEl.textContent=gOk?g.fix_quality_name:'No GPS';
  fixEl.style.color=gOk?fixColor:C.gray;
  el('gps-panel').style.borderColor=s.gps_logging?C.cyan:(gOk?fixColor:C.border);
  el('gps-title').style.color=gOk?fixColor:C.gray;

  // System
  applySystem(s.system);

  // Lidar
  drawLidar(s.lidar.angles, s.lidar.distances);

  // Camera
  el('cam-rot-lbl').textContent = s.cam_rotation ? `[${s.cam_rotation}°]` : '';
  const acoBtn=el('btn-aruco'), acoInfoEl=el('cam-aruco-info');
  if (s.aruco_enabled) {
    acoBtn.textContent='ArUco: ON';
    acoBtn.style.color=C.green; acoBtn.style.borderColor=C.green;
    if (s.aruco) {
      acoInfoEl.textContent=`tags:${s.aruco.tag_count} gates:${s.aruco.gate_count} ${s.aruco.fps}fps`;
      acoInfoEl.style.color=C.green;
    } else {
      acoInfoEl.textContent='ACO: init…'; acoInfoEl.style.color=C.yellow;
    }
  } else {
    acoBtn.textContent='ArUco: OFF';
    acoBtn.style.color=C.gray; acoBtn.style.borderColor=C.border;
    acoInfoEl.textContent='';
  }
  el('cam-img').style.display     = s.camera_ok ? 'block' : 'none';
  el('cam-nosignal').style.display= s.camera_ok ? 'none'  : 'block';

  // Bearing overlay
  el('bearing-canvas').style.display = (showBearing && s.aruco_enabled) ? 'block' : 'none';
  if (showBearing && s.aruco_enabled) {
    drawBearingOverlay(s.aruco, s.nav_gate, t.heading);
  }

  // Nav badge (top-right of camera)
  const navBadge   = el('nav-badge');
  const autoCamera = s.mode==='AUTO' && (s.auto_type==='Camera'||s.auto_type==='Cam+GPS');
  const autoGps    = s.mode==='AUTO' && (s.auto_type==='GPS'    ||s.auto_type==='Cam+GPS');
  if (autoCamera || autoGps) {
    const nc = NAV_COLORS[s.nav_state] || C.gray;
    let ntxt;
    if (autoGps && s.nav_state && s.nav_state !== 'IDLE') {
      ntxt = `GPS:${s.nav_state}  WP:${s.nav_wp}`;
      if (s.nav_wp_dist != null) ntxt += `  ${s.nav_wp_dist.toFixed(1)}m`;
      if (s.nav_wp_bear != null) ntxt += `  ${s.nav_wp_bear.toFixed(0)}°`;
    } else {
      ntxt = `${s.nav_state}  G${s.nav_gate}`;
      if (s.nav_bearing_err != null)
        ntxt += `  ${s.nav_bearing_err>=0?'+':''}${s.nav_bearing_err.toFixed(1)}°`;
    }
    navBadge.textContent   = ntxt;
    navBadge.style.color   = nc;
    navBadge.style.display = 'block';
  } else {
    navBadge.style.display = 'none';
  }

  // Footer badges
  function badge(id, text, active, color) {
    const b=el(id); b.textContent=text; b.style.color=active?color:C.gray;
  }
  badge('badge-rc',  `RC: ${s.rc_active?'OK':'--'}`,   s.rc_active,  C.green);
  badge('badge-cam', `CAM: ${s.camera_ok?'OK':'--'}`,  s.camera_ok,  C.green);
  badge('badge-aco', `ACO: ${s.aruco_ok?'OK':'--'}`,   s.aruco_ok,   C.green);
  badge('badge-ldr', `LDR: ${s.lidar_ok?'OK':'--'}`,   s.lidar_ok,   C.green);
  badge('badge-gps', `GPS: ${gOk?g.fix_quality_name:'--'}`, gOk, fixColor);
  const herrEl=el('footer-herr');
  if (gOk && g.h_error_m != null) {
    herrEl.textContent=`H-err: ${g.h_error_m.toFixed(3)}m`; herrEl.style.color=C.cyan;
  } else { herrEl.textContent=''; }
  badge('badge-log', `LOG: ${s.gps_logging?'ON':'OFF'}`, s.gps_logging, C.cyan);

  // IMU heading footer badge
  const hdgBadge=el('badge-hdg');
  if (t.heading != null) {
    hdgBadge.textContent=`HDG: ${t.heading.toFixed(1)}°`;
    hdgBadge.style.color=C.cyan; hdgBadge.style.display='inline';
  } else { hdgBadge.style.display='none'; }

  // Nav state footer badge
  const navFooter=el('badge-nav');
  if (s.nav_state && s.nav_state!=='IDLE') {
    const nc=NAV_COLORS[s.nav_state]||C.gray;
    let navTxt;
    if (autoGps) {
      navTxt = `GPS: ${s.nav_state}  WP${s.nav_wp}`;
      if (s.nav_wp_dist != null) navTxt += `  ${s.nav_wp_dist.toFixed(1)}m`;
    } else {
      navTxt = `NAV: ${s.nav_state}  G${s.nav_gate}`;
    }
    navFooter.textContent=navTxt;
    navFooter.style.color=nc; navFooter.style.display='inline';
  } else { navFooter.style.display='none'; }
}

// ── System bars ───────────────────────────────────────────────────────────────
function pctColor(p,w,c){ return p>=c?C.red:p>=w?C.yellow:C.green; }
function sysBar(fid,vid,pct,lbl,color){
  const f=el(fid);
  f.style.width=Math.min(pct,100)+'%'; f.style.backgroundColor=color;
  const v=el(vid); v.textContent=lbl; v.style.color=pct>55?C.bg:color;
}
function applySystem(s){
  const cpu=pctColor(s.cpu_percent,60,85);
  const tmp=s.cpu_temp_c<60?C.green:s.cpu_temp_c<75?C.yellow:C.red;
  const mem=pctColor(s.mem_percent,70,90);
  const dsk=pctColor(s.disk_percent,70,90);
  sysBar('sys-cpu-fill','sys-cpu-val',s.cpu_percent,`${s.cpu_percent.toFixed(0)}%`,cpu);
  sysBar('sys-mem-fill','sys-mem-val',s.mem_percent,
    `${s.mem_percent.toFixed(0)}% ${(s.mem_used_mb/1024).toFixed(1)}/${(s.mem_total_mb/1024).toFixed(1)}GB`,mem);
  sysBar('sys-disk-fill','sys-disk-val',s.disk_percent,
    `${s.disk_percent.toFixed(0)}% ${s.disk_used_gb.toFixed(1)}/${s.disk_total_gb.toFixed(1)}GB`,dsk);
  el('sys-temp').textContent=`${s.cpu_temp_c.toFixed(0)}°C`;
  el('sys-temp').style.color=tmp;
  el('sys-freq').textContent=`${s.cpu_freq_mhz.toFixed(0)} MHz`;
  el('sys-panel').style.borderColor=s.cpu_temp_c>=75?C.red:s.cpu_temp_c>=60?C.yellow:C.border;
  el('sys-title').style.color=s.cpu_temp_c>=60?tmp:C.gray;
}

// ── Commands ──────────────────────────────────────────────────────────────────
async function sendCmd(cmd) {
  try {
    await fetch('/api/cmd',{method:'POST',
      headers:{'Content-Type':'application/json'},body:JSON.stringify({cmd})});
  } catch(e){ console.error('cmd failed:',e); }
}

// Bearing toggle (client-side only)
function toggleBearing() {
  showBearing = !showBearing;
  const btn=el('btn-bearing');
  btn.textContent = showBearing ? 'Bearing: ON' : 'Bearing: OFF';
  btn.style.color  = showBearing ? C.cyan : C.gray;
  btn.style.borderColor = showBearing ? C.cyan : C.border;
  if (!showBearing) {
    el('bearing-canvas').style.display='none';
    bearingCtx.clearRect(0,0,bearingCanvas.width,bearingCanvas.height);
  }
  if (lastState) applyState(lastState);
}

el('btn-bearing').onclick = toggleBearing;
el('btn-estop').onclick   = ()=>sendCmd('estop');
el('btn-reset').onclick   = ()=>sendCmd('reset');
el('btn-ccw').onclick     = ()=>sendCmd('rotate_ccw');
el('btn-cw').onclick      = ()=>sendCmd('rotate_cw');
el('btn-aruco').onclick   = ()=>sendCmd('aruco_toggle');

document.addEventListener('keydown', e => {
  if (e.key==='[')                    sendCmd('rotate_ccw');
  if (e.key===']')                    sendCmd('rotate_cw');
  if (e.key==='t'||e.key==='T')       sendCmd('aruco_toggle');
  if (e.key==='b'||e.key==='B')       toggleBearing();
  if (e.key==='e'||e.key==='E')       sendCmd('estop');
  if (e.key==='r'||e.key==='R')       sendCmd('reset');
});

// ── SSE connection ────────────────────────────────────────────────────────────
let evtSrc=null;
function connect(){
  if(evtSrc) evtSrc.close();
  const badge=el('conn-badge');
  badge.textContent='🟡 Connecting…'; badge.style.color=C.yellow;
  evtSrc=new EventSource('/api/state');
  evtSrc.onopen=()=>{ badge.textContent='🟢 Live'; badge.style.color=C.green; };
  evtSrc.onmessage=e=>{
    try { applyState(JSON.parse(e.data)); } catch(err){console.error('SSE:',err);}
  };
  evtSrc.onerror=()=>{
    badge.textContent='🔴 Reconnecting…'; badge.style.color=C.red;
    evtSrc.close(); setTimeout(connect,3000);
  };
}

// ── Resize handling ───────────────────────────────────────────────────────────
function onResize(){
  resizeLidar();
  resizeBearingCanvas();
}
window.addEventListener('resize', onResize);

// Get natural camera frame size from stream image when it loads
el('cam-img').addEventListener('load', function(){
  if(this.naturalWidth)  camNatW=this.naturalWidth;
  if(this.naturalHeight) camNatH=this.naturalHeight;
});

onResize();
connect();
</script>
</body>
</html>"""

# ── Flask app ─────────────────────────────────────────────────────────────────

app = Flask(__name__)
_robot = None  # set in main()


@app.route('/')
def index():
    return Response(_HTML, mimetype='text/html')


@app.route('/stream')
def camera_stream():
    """MJPEG camera stream."""
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
            time.sleep(0.05)
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
    return Response(
        _gen(),
        mimetype='text/event-stream',
        headers={'Cache-Control': 'no-cache', 'X-Accel-Buffering': 'no'},
    )


@app.route('/api/cmd', methods=['POST'])
def api_cmd():
    cmd = (request.json or {}).get('cmd', '')
    if   cmd == 'estop':        _robot.estop()
    elif cmd == 'reset':        _robot.reset_estop()
    elif cmd == 'aruco_toggle': _robot.toggle_aruco()
    elif cmd == 'gps_bookmark': _robot.bookmark_gps()
    elif cmd == 'rotate_cw':    _robot.set_cam_rotation((_robot.get_cam_rotation() + 90) % 360)
    elif cmd == 'rotate_ccw':   _robot.set_cam_rotation((_robot.get_cam_rotation() - 90 + 360) % 360)
    else:
        return jsonify({'ok': False, 'error': f'Unknown command: {cmd}'}), 400
    return jsonify({'ok': True})


# ── Config helpers ────────────────────────────────────────────────────────────

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
    parser.add_argument('--config',          default=DEFAULT_CFG)
    parser.add_argument('--host',            default=None)
    parser.add_argument('--port',            default=None, type=int)
    parser.add_argument('--debug',           action='store_true', default=None)
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

    log_path = setup_logging()
    log.info(f"Log file: {log_path}")
    cfg = _load_config(args.config)

    web_host  = args.host  or _cfg(cfg, 'web', 'host',  '0.0.0.0')
    web_port  = args.port  or _cfg(cfg, 'web', 'port',  5000, int)
    web_debug = args.debug or _cfg(cfg, 'web', 'debug', False,
                                   lambda x: x.lower() == 'true')

    _jpeg_encode = _make_jpeg_encoder()

    bool_val = lambda x: x.lower() == 'true'
    port_val = lambda x: None if x.lower() in ('auto', '') else x

    def arg(cli_val, section, key, fallback, cast=str):
        if cli_val is not None:
            return cli_val
        return _cfg(cfg, section, key, fallback, cast)

    _robot = Robot(
        yukon_port     = arg(args.yukon_port,      'robot', 'yukon_port',     None, port_val),
        ibus_port      = arg(args.ibus_port,        'robot', 'ibus_port',      '/dev/ttyAMA3'),
        lidar_port     = arg(args.lidar_port,       'lidar', 'port',           '/dev/ttyUSB0'),
        gps_port       = arg(args.gps_port,         'gps',   'port',           '/dev/ttyUSB0'),
        ntrip_host     = '' if _cfg(cfg,'ntrip','disabled',False,bool_val)
                         else arg(args.ntrip_host,  'ntrip', 'host',           ''),
        ntrip_port     = arg(args.ntrip_port_arg,   'ntrip', 'port',           2101, int),
        ntrip_mount    = arg(args.ntrip_mount,       'ntrip', 'mount',          ''),
        ntrip_user     = arg(args.ntrip_user,        'ntrip', 'user',           ''),
        ntrip_password = arg(args.ntrip_password,    'ntrip', 'password',       ''),
        rtcm_port      = '' if _cfg(cfg,'rtcm','disabled',False,bool_val)
                         else arg(args.rtcm_port,   'rtcm',  'port',           ''),
        rtcm_baud      = arg(args.rtcm_baud,         'rtcm',  'baud',           115200, int),
        enable_camera  = not (args.no_camera or _cfg(cfg,'camera','disabled',False,bool_val)),
        enable_lidar   = not (args.no_lidar  or _cfg(cfg,'lidar', 'disabled',False,bool_val)),
        enable_gps     = not (args.no_gps    or _cfg(cfg,'gps',   'disabled',False,bool_val)),
        cam_width      = _cfg(cfg, 'camera',  'width',         640,  int),
        cam_height     = _cfg(cfg, 'camera',  'height',        480,  int),
        cam_fps        = _cfg(cfg, 'camera',  'fps',           30,   int),
        cam_rotation   = _cfg(cfg, 'camera',  'rotation',      0,    int),
        enable_aruco   = _cfg(cfg, 'aruco',   'enabled',       False, bool_val),
        aruco_dict     = _cfg(cfg, 'aruco',   'dict',          'DICT_4X4_1000'),
        aruco_calib    = _cfg(cfg, 'aruco',   'calib_file',    ''),
        aruco_tag_size = _cfg(cfg, 'aruco',   'tag_size',      0.15, float),
        throttle_ch    = _cfg(cfg, 'rc',      'throttle_ch',   3,    int),
        steer_ch       = _cfg(cfg, 'rc',      'steer_ch',      1,    int),
        mode_ch        = _cfg(cfg, 'rc',      'mode_ch',       5,    int),
        speed_ch       = _cfg(cfg, 'rc',      'speed_ch',      6,    int),
        auto_type_ch   = _cfg(cfg, 'rc',      'auto_type_ch',  7,    int),
        gps_log_ch     = _cfg(cfg, 'rc',      'gps_log_ch',    8,    int),
        gps_bookmark_ch= _cfg(cfg, 'rc',      'gps_bookmark_ch',2,   int),
        gps_log_dir    = _cfg(cfg, 'gps',     'log_dir',       ''),
        gps_log_hz     = _cfg(cfg, 'gps',     'log_hz',        5.0,  float),
        deadzone       = _cfg(cfg, 'rc',      'deadzone',      30,   int),
        failsafe_s     = _cfg(cfg, 'rc',      'failsafe_s',    0.5,  float),
        speed_min      = _cfg(cfg, 'rc',      'speed_min',     0.25, float),
        control_hz     = _cfg(cfg, 'rc',      'control_hz',    50,   int),
        no_motors      = args.no_motors,
    )
    _robot.start()
    log.info(f'Web dashboard → http://{web_host}:{web_port}/')

    try:
        app.run(host=web_host, port=web_port,
                debug=web_debug, threaded=True, use_reloader=False)
    finally:
        _robot.stop()


if __name__ == '__main__':
    main()
