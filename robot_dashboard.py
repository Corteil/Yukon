#!/usr/bin/env python3
"""
robot_dashboard.py — Unified web dashboard for HackyRacingRobot.

Replaces robot_web.py, robot_mobile.py, robot_gui.py, robot_quad_gui.py.

Layout
------
  Desktop / touchscreen (≥701px):
    Status bar (always visible) + 2×2 configurable panel grid.
    Single-tap quad → panel-type chooser.
    Double-tap / double-click → expand panel full-screen.
    Preset buttons: Race / Setup / Debug (from robot.ini [layout_presets]).

  Mobile (≤700px):
    Status bar + tab pages (Drive / Telem / GPS / System / Logs).
    Tab bar fixed at bottom.

Usage
-----
  python3 robot_dashboard.py            # 0.0.0.0:5000
  python3 robot_dashboard.py --port 8080
  python3 robot_dashboard.py --no-motors
"""

import argparse
import configparser
import io
import json
import logging
import os
import sys
import time
from typing import Optional

from flask import Flask, Response, request, jsonify

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from robot_daemon import Robot, RobotMode, AutoType, setup_logging
from robot_utils import _cfg, _local_ip

log = logging.getLogger(__name__)


# ── JPEG encoder ──────────────────────────────────────────────────────────────

def _make_jpeg_encoder():
    """Fallback JPEG encoder for when robot.get_jpeg() returns None (no cached frame yet).

    Priority: TurboJPEG (libjpeg-turbo NEON SIMD) → OpenCV → Pillow.
    Under normal operation the camera thread pre-encodes frames, so this path
    is rarely used — it only fires on the first frame before the cache warms up.
    """
    try:
        from turbojpeg import TurboJPEG, TJPF_RGB
        _tj = TurboJPEG()
        def _enc(frame, quality=75):
            return bytes(_tj.encode(frame, quality=quality, pixel_format=TJPF_RGB))
        log.info('Fallback JPEG encoder: TurboJPEG (libjpeg-turbo NEON)')
        return _enc
    except Exception:
        pass
    try:
        import cv2
        def _enc(frame, quality=75):
            _, buf = cv2.imencode('.jpg', frame[:, :, ::-1],
                                  [cv2.IMWRITE_JPEG_QUALITY, quality])
            return buf.tobytes()
        log.info('Fallback JPEG encoder: OpenCV')
        return _enc
    except ImportError:
        pass
    try:
        from PIL import Image as _PIL
        def _enc(frame, quality=75):
            buf = io.BytesIO()
            _PIL.fromarray(frame).save(buf, format='JPEG', quality=quality)
            return buf.getvalue()
        log.info('Fallback JPEG encoder: Pillow')
        return _enc
    except ImportError:
        pass
    log.warning('No JPEG encoder — camera stream disabled (pip install pillow)')
    return None

_jpeg_encode = None


# ── State serialiser ──────────────────────────────────────────────────────────

def _aruco_info(aruco_state, cap_w=0, cap_h=0):
    """Serialise a single ArUcoState to a JSON-safe dict.

    cap_w/cap_h: capture resolution where ArUco coordinates live.
    Included so the JS can scale corners from capture space to display space.
    """
    return {
        'tag_count':  len(aruco_state.tags),
        'gate_count': len(aruco_state.gates),
        'fps':        aruco_state.fps,
        'cap_w':      cap_w,
        'cap_h':      cap_h,
        'tags': [
            {'id': t2.id, 'cx': t2.center_x, 'cy': t2.center_y,
             'area': t2.area, 'bearing': t2.bearing, 'distance': t2.distance,
             'corners': [list(t2.top_left), list(t2.top_right),
                         list(t2.bottom_right), list(t2.bottom_left)]}
            for t2 in aruco_state.tags.values()
        ],
        'gates': [
            {'gate_id': gx.gate_id, 'centre_x': gx.centre_x,
             'centre_y': gx.centre_y, 'bearing': gx.bearing,
             'distance': gx.distance, 'correct_dir': gx.correct_dir}
            for gx in aruco_state.gates.values()
        ],
    }


def _serialise(state, cam_rotation=0, aruco_enabled=None,
               aruco_states=None, nav_bearing_err=None):
    g, t, d, li, s = (state.gps, state.telemetry, state.drive,
                      state.lidar, state.system)

    # aruco_enabled is a dict {cam_name: bool} from get_aruco_enabled('all')
    if aruco_enabled is None:
        aruco_enabled = {}

    # Per-camera ArUco info — only include when that camera's detection is enabled
    aruco_info = {}
    if aruco_states:
        for cam_name, (aruco_state, cap_w, cap_h) in aruco_states.items():
            if aruco_enabled.get(cam_name) and aruco_state is not None:
                aruco_info[cam_name] = _aruco_info(aruco_state, cap_w, cap_h)

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
        'cam_recording': state.cam_recording,
        'data_logging':  state.data_logging,
        'no_motors':     state.no_motors,
        'bench_enabled': state.bench_enabled,
        'nav_state':     state.nav_state,
        'nav_gate':      state.nav_gate,
        'nav_bearing_err': nav_bearing_err,
        'nav_wp':        state.nav_wp,
        'nav_wp_dist':   state.nav_wp_dist,
        'nav_wp_bear':   state.nav_wp_bear,
        'aruco':         aruco_info,   # dict keyed by camera name
        # Per-camera status
        'cam_fl_ok':  state.cam_front_left_ok,
        'cam_fr_ok':  state.cam_front_right_ok,
        'cam_re_ok':  state.cam_rear_ok,
        'cam_fl_rec': state.cam_front_left_recording,
        'cam_fr_rec': state.cam_front_right_recording,
        'cam_re_rec': state.cam_rear_recording,
        'gate_confirmed':  state.gate_confirmed,
        'current_gate_id': state.current_gate_id,
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
            'heading': round(t.heading, 1) if t.heading is not None else None,
            'pitch':   round(t.pitch,   1) if t.pitch   is not None else None,
            'roll':    round(t.roll,    1) if t.roll    is not None else None,
        },
        'gps': {
            'latitude':         g.latitude,
            'longitude':        g.longitude,
            'altitude':         g.altitude,
            'speed':            g.speed,
            'heading':          g.heading,
            'fix_quality':      g.fix_quality,
            'fix_quality_name': g.fix_quality_name,
            'h_error_m':        g.h_error_m,
            'satellites':       g.satellites,
            'satellites_view':  g.satellites_view,
            'satellites_data':  g.satellites_data,
            'hdop':             g.hdop,
            'ntrip_status':     g.ntrip_status,
            'ntrip_bytes_recv': g.ntrip_bytes_recv,
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


# ── Embedded HTML/CSS/JS ──────────────────────────────────────────────────────

_HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>HackyRacingRobot</title>
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
body{display:flex;flex-direction:column;padding:4px;gap:4px;height:100vh}

/* ── Status bar ── */
#sb{display:flex;align-items:center;flex-wrap:wrap;gap:6px;padding:5px 10px;
  background:var(--panel);border:1px solid var(--border);border-radius:7px;
  flex-shrink:0;min-height:40px;transition:border-color .3s}
#sb-mode{font-size:15px;font-weight:bold;padding:2px 8px;border:1px solid;border-radius:4px}
#sb-info{display:flex;gap:12px;font-size:13px;flex:1;flex-wrap:wrap}
.sb-badge{font-size:12px;color:var(--gray)}
#sb-presets{display:flex;gap:4px;margin-left:auto}
#sb-btns{display:flex;gap:5px}
button{font-family:inherit;font-size:12px;padding:3px 9px;border-radius:5px;
  border:1px solid var(--border);background:var(--panel);color:var(--white);
  cursor:pointer;touch-action:manipulation;-webkit-tap-highlight-color:transparent}
button:active{background:#2a2a42}
.btn-estop{border-color:var(--red)!important;color:var(--red)!important}
.btn-reset{border-color:var(--green)!important;color:var(--green)!important}
.btn-mode{border-color:var(--yellow);color:var(--yellow)}
.btn-preset{font-size:11px;padding:2px 7px;color:var(--gray)}
.btn-preset.active{color:var(--cyan);border-color:var(--cyan)}
@keyframes blink{50%{opacity:.25}}
@keyframes rec-blink{50%{opacity:0}}

/* ── No-motors banner ── */
#no-motors-banner{display:none;background:#3a0000;border:1px solid var(--red);
  border-radius:5px;padding:3px 10px;text-align:center;color:var(--red);
  font-size:12px;font-weight:bold;flex-shrink:0}

/* ── Main area ── */
#main{flex:1;min-height:0;display:flex;flex-direction:column}

/* ── Desktop grid ── */
#grid{display:grid;grid-template-columns:1fr 1fr;grid-template-rows:1fr 1fr;
  gap:4px;flex:1;min-height:0}
.quad{background:var(--panel);border:1px solid var(--border);border-radius:7px;
  overflow:hidden;display:flex;flex-direction:column;position:relative;
  cursor:pointer;min-height:0}
.quad.expanded{position:fixed;top:0;left:0;right:0;bottom:0;z-index:200;
  border-radius:0;border:none}
.quad-hdr{display:flex;align-items:center;gap:5px;padding:3px 7px;
  background:rgba(0,0,0,.3);flex-shrink:0;font-size:11px;color:var(--gray)}
.quad-title{font-weight:bold;color:var(--white);flex:1;cursor:pointer}
.quad-title:hover{color:var(--green)}
.quad-hdr button{padding:4px 10px;font-size:13px}
.quad-body{flex:1;min-height:0;overflow:hidden;display:flex;flex-direction:column}

/* Camera panel */
.cam-wrap{flex:1;min-height:0;overflow:hidden;display:flex;align-items:center;
  justify-content:center;background:var(--bg);position:relative}
.cam-img{width:100%;height:100%;object-fit:contain;display:block}
.cam-nosig{position:absolute;color:var(--gray);font-size:12px}
.cam-paused{position:absolute;color:var(--white);font-size:18px;font-weight:bold;
  background:rgba(0,0,0,.55);padding:8px 18px;border-radius:8px;pointer-events:none}
.cam-rec-dot{position:absolute;top:8px;left:8px;width:10px;height:10px;
  border-radius:50%;background:var(--red);display:none;
  animation:rec-blink .8s step-end infinite}
.cam-nav-badge{position:absolute;top:5px;right:5px;font-size:10px;
  background:rgba(18,18,30,.85);padding:2px 5px;border-radius:3px;display:none}
.bearing-canvas{position:absolute;top:0;left:0;width:100%;height:100%;
  pointer-events:none;display:none}

/* Motor bars */
.mrow{display:flex;align-items:center;gap:6px;padding:4px 8px}
.mlabel{width:10px;font-size:12px;color:var(--gray);flex-shrink:0}
.mtrack{flex:1;height:34px;background:var(--bg);border-radius:4px;position:relative}
.mmid{position:absolute;left:50%;top:3px;bottom:3px;width:1px;background:var(--border)}
.mfill{position:absolute;top:3px;bottom:3px;border-radius:3px;transition:width .06s,left .06s}
.mval{position:absolute;right:5px;top:50%;transform:translateY(-50%);
  font-size:11px;pointer-events:none}

/* Telemetry */
.tgrid{display:grid;grid-template-columns:1fr 1fr;gap:4px 10px;padding:5px 8px}
.tgrid.three{grid-template-columns:1fr 1fr 1fr}
/* Telemetry instruments */
.telem-instruments{flex:1;min-height:0;display:flex;gap:4px;padding:4px}
.telem-inst-wrap{flex:1;display:flex;flex-direction:column;align-items:center;min-width:0}
.telem-inst-canvas{width:100%;flex:1;min-height:0;display:block}
.telem-inst-lbl{font-size:10px;color:var(--gray);text-align:center;padding:1px 0}
.field label{font-size:10px;color:var(--gray);display:block}
.field span{font-size:13px}
.compass-wrap{display:inline-flex;align-items:center;gap:5px}
.compass-canvas{width:32px;height:32px}

/* GPS */
.ggrid{display:grid;grid-template-columns:1fr 1fr;gap:4px 10px;padding:5px 8px}
.ggrid.three{grid-template-columns:1fr 1fr 1fr}
.gps-row{display:flex;align-items:center;gap:6px;padding:2px 8px;font-size:11px}
.gps-row label{color:var(--gray);min-width:36px;flex-shrink:0}
.rtk-dot{display:inline-block;width:8px;height:8px;border-radius:50%;
  margin-right:4px;background:var(--gray);flex-shrink:0}
/* Signal bars */
.sig-wrap{padding:4px 8px;flex:1;overflow:hidden}
.sig-bars{display:flex;align-items:flex-end;gap:2px;height:42px}
.sig-bar{flex:1;min-width:4px;border-radius:2px 2px 0 0;position:relative;transition:height .3s}
.sig-lbl{font-size:8px;text-align:center;color:var(--gray);overflow:hidden;white-space:nowrap}
/* Canvas panels */
.gps-canvas-wrap{flex:1;display:flex;flex-direction:column;overflow:hidden;padding:4px}
.sky-canvas,.track-canvas,.scatter-canvas,.sig-canvas{width:100%;flex:1;display:block}
/* Terminal */
.term-box{flex:1;overflow-y:auto;background:#0a0a0a;font-size:11px;font-family:monospace;
  line-height:1.35;padding:4px 6px;color:#00cc44}
.term-line{white-space:pre-wrap;word-break:break-all}
.term-line.warning{color:#ccaa00}.term-line.error,.term-line.critical{color:#ff4444}
.term-line.debug{color:#448844}

/* System bars */
.sys-row{display:flex;align-items:center;gap:4px;padding:3px 8px}
.sys-row label{font-size:10px;color:var(--gray);width:28px;flex-shrink:0}
.sys-track{flex:1;height:14px;background:var(--bg);border-radius:3px;
  position:relative;overflow:hidden}
.sys-fill{position:absolute;left:0;top:0;bottom:0;border-radius:3px;transition:width .3s}
.sys-val{position:absolute;right:3px;top:50%;transform:translateY(-50%);
  font-size:10px;pointer-events:none}

/* Log box */
.log-toolbar{display:flex;gap:5px;padding:4px 7px;flex-shrink:0}
.log-filter{flex:1;background:var(--bg);border:1px solid var(--border);
  color:var(--white);border-radius:4px;padding:2px 6px;
  font-family:inherit;font-size:11px}
.log-box{flex:1;overflow-y:auto;background:var(--bg);font-size:12px;
  line-height:1.4;padding:4px 6px}
.tlog{white-space:pre-wrap;word-break:break-all;color:#b0b0c8}
.tlog.warning{color:var(--yellow)}.tlog.error,.tlog.critical{color:var(--red)}
.tlog.debug{color:var(--gray)}

/* Lidar */
.lidar-canvas{display:block;width:100%;flex:1}

/* ── Panel chooser overlay ── */
#chooser{display:none;position:fixed;inset:0;background:rgba(0,0,0,.7);
  z-index:300;align-items:center;justify-content:center}
#chooser.open{display:flex}
#chooser-box{background:var(--panel);border:1px solid var(--border);
  border-radius:10px;padding:16px;max-width:360px;width:90%}
#chooser-title{font-size:13px;color:var(--gray);margin-bottom:10px;text-align:center}
#chooser-grid{display:grid;grid-template-columns:1fr 1fr 1fr;gap:7px;margin-bottom:12px}
.chooser-btn{padding:8px 4px;font-size:11px;text-align:center;border-radius:5px}
.chooser-btn.active{border-color:var(--cyan);color:var(--cyan)}
#chooser-cancel{width:100%;padding:5px}

/* ── Mobile layout ── */
#tab-bar{display:none;flex-shrink:0;gap:2px;padding:4px;background:var(--panel);
  border-top:1px solid var(--border)}
.tab-btn{flex:1;padding:6px 2px;font-size:11px;text-align:center;border-radius:5px;
  border-color:transparent;color:var(--gray)}
.tab-btn.active{color:var(--cyan);border-color:var(--cyan)}
#mob-tabs{display:none;flex:1;min-height:0;overflow:hidden}
.mob-page{display:none;flex-direction:column;height:100%;overflow-y:auto}
.mob-page.active{display:flex}
.mob-section{padding:5px 8px;border-bottom:1px solid var(--border)}
.mob-title{font-size:10px;color:var(--gray);margin-bottom:4px}

@media(max-width:700px){
  body{overflow-y:hidden}
  #grid{display:none}
  #mob-tabs{display:flex}
  #tab-bar{display:flex}
  #sb-presets{display:none}
}
@media(max-width:700px) and (orientation:landscape){
  #mob-tabs .cam-wrap{max-height:55vw}
}
</style>
</head>
<body>

<!-- Status bar -->
<div id="sb">
  <span id="sb-mode">--</span>
  <div id="sb-info">
    <span class="sb-badge" id="sb-volt">--V</span>
    <span class="sb-badge" id="sb-rc">RC --</span>
    <span class="sb-badge" id="sb-rec" style="display:none;color:var(--red);animation:blink 1s step-end infinite">⏺ REC</span>
    <span class="sb-badge" id="sb-dlog" style="display:none;color:var(--purple)">⬤ DLOG</span>
    <span class="sb-badge" id="sb-conn" style="margin-left:auto">⚫ Connecting…</span>
  </div>
  <div id="sb-presets"></div>
  <div id="sb-btns">
    <button class="btn-estop" onclick="sendCmd('estop')">ESTOP</button>
    <button class="btn-reset" id="btn-reset" onclick="sendCmd('reset')" style="display:none">Reset ESTOP</button>
    <button class="btn-mode" id="btn-mode" onclick="toggleMode()">AUTO</button>
    <button onclick="sendCmd('record_start')" id="btn-rec" title="Start recording all cameras">⏺ REC</button>
    <button onclick="sendCmd('record_stop')"  id="btn-rec-stop" title="Stop recording all cameras">⏹ REC</button>
    <button onclick="sendCmd('data_log_toggle')" id="btn-dlog" title="Toggle data log">⬤ DLOG</button>
    <button onclick="sendCmd('bench_toggle')" id="btn-bench" title="Toggle bench power output">⚡ BENCH</button>
    <button onclick="stopAllStreams()" title="Pause all camera streams">📷 Off</button>
  </div>
</div>

<!-- No-motors banner -->
<div id="no-motors-banner">⚠ NO-MOTORS MODE — drive commands suppressed</div>

<!-- Main: grid (desktop) or tab pages (mobile) -->
<div id="main">

  <!-- Desktop 2×2 grid -->
  <div id="grid">
    <div class="quad" id="q0">
      <div class="quad-hdr" id="q0-hdr"><span class="quad-title" id="q0-title">--</span></div>
      <div class="quad-body" id="q0-body"></div>
    </div>
    <div class="quad" id="q1">
      <div class="quad-hdr" id="q1-hdr"><span class="quad-title" id="q1-title">--</span></div>
      <div class="quad-body" id="q1-body"></div>
    </div>
    <div class="quad" id="q2">
      <div class="quad-hdr" id="q2-hdr"><span class="quad-title" id="q2-title">--</span></div>
      <div class="quad-body" id="q2-body"></div>
    </div>
    <div class="quad" id="q3">
      <div class="quad-hdr" id="q3-hdr"><span class="quad-title" id="q3-title">--</span></div>
      <div class="quad-body" id="q3-body"></div>
    </div>
  </div>

  <!-- Mobile tab pages -->
  <div id="mob-tabs">

    <div class="mob-page active" id="mob-drive">
      <div class="cam-wrap" id="mob-cam-wrap" style="max-height:45vh">
        <img class="cam-img" id="mob-cam-img" alt="">
        <span class="cam-nosig" id="mob-cam-nosig">No camera</span>
        <canvas class="bearing-canvas" id="mob-bearing-canvas"></canvas>
        <div class="cam-rec-dot" id="mob-cam-rec-dot"></div>
        <div class="cam-nav-badge" id="mob-cam-nav-badge"></div>
      </div>
      <div class="mob-section">
        <div class="mob-title">Drive</div>
        <div class="mrow"><span class="mlabel">L</span>
          <div class="mtrack"><div class="mmid"></div>
            <div class="mfill" id="mob-fill-l"></div>
            <span class="mval" id="mob-val-l">+0.00</span></div></div>
        <div class="mrow"><span class="mlabel">R</span>
          <div class="mtrack"><div class="mmid"></div>
            <div class="mfill" id="mob-fill-r"></div>
            <span class="mval" id="mob-val-r">+0.00</span></div></div>
      </div>
      <div class="mob-section" style="display:flex;gap:6px;flex-wrap:wrap;padding:6px 8px">
        <button onclick="sendCmd('aruco_toggle',{cam:'front_left'})" id="mob-btn-aruco">ArUco: OFF</button>
        <button onclick="sendCmd('rotate_ccw',{cam:'front_left'})">↺ CCW</button>
        <button onclick="sendCmd('rotate_cw',{cam:'front_left'})">↻ CW</button>
        <button onclick="toggleMobBearing()" id="mob-btn-bearing">Bearing: OFF</button>
        <button onclick="sendCmd('gps_bookmark')">📍 Bookmark</button>
      </div>
      <div class="mob-section" style="font-size:12px;padding:5px 8px">
        <span id="mob-nav-info" style="color:var(--gray)"></span>
      </div>
    </div>

    <div class="mob-page" id="mob-telem">
      <div class="mob-section">
        <div class="mob-title">Telemetry</div>
        <div class="tgrid">
          <div class="field"><label>Voltage</label><span id="mob-volt">--</span></div>
          <div class="field"><label>Current</label><span id="mob-curr">--</span></div>
        </div>
        <div class="tgrid three">
          <div class="field"><label>Board</label><span id="mob-board">--</span></div>
          <div class="field"><label>Left</label><span id="mob-ltmp">--</span></div>
          <div class="field"><label>Right</label><span id="mob-rtmp">--</span></div>
        </div>
        <div class="tgrid">
          <div class="field">
            <label>IMU Heading</label>
            <div class="compass-wrap">
              <canvas class="compass-canvas" id="mob-compass" width="32" height="32"></canvas>
              <span id="mob-hdg-text" style="font-size:13px;color:var(--cyan)">---</span>
            </div>
          </div>
          <div class="field"><label>Faults</label><span id="mob-fault" style="color:var(--green)">OK</span></div>
        </div>
        <div class="tgrid">
          <div class="field"><label>Pitch</label><span id="mob-pitch" style="color:var(--gray)">---</span></div>
          <div class="field"><label>Roll</label><span id="mob-roll" style="color:var(--gray)">---</span></div>
        </div>
      </div>
    </div>

    <div class="mob-page" id="mob-gps">
      <div class="mob-section">
        <div class="mob-title">GPS</div>
        <div class="ggrid">
          <div class="field"><label>Latitude</label><span id="mob-lat">--</span></div>
          <div class="field"><label>Altitude</label><span id="mob-alt">--</span></div>
          <div class="field"><label>Longitude</label><span id="mob-lon">--</span></div>
          <div class="field"><label>H-Error</label><span id="mob-herr">--</span></div>
        </div>
        <div class="ggrid three">
          <div class="field"><label>Fix</label><span id="mob-fix">--</span></div>
          <div class="field"><label>Sats</label><span id="mob-sats">--</span></div>
          <div class="field"><label>HDOP</label><span id="mob-hdop">--</span></div>
        </div>
        <div style="padding:6px 8px">
          <button onclick="sendCmd('gps_bookmark')">📍 Bookmark GPS position</button>
        </div>
      </div>
    </div>

    <div class="mob-page" id="mob-sys">
      <div class="mob-section">
        <div class="mob-title">System</div>
        <div class="sys-row"><label>CPU</label>
          <div class="sys-track"><div class="sys-fill" id="mob-cpu-fill"></div>
            <span class="sys-val" id="mob-cpu-val">--</span></div></div>
        <div class="sys-row" style="padding:3px 8px">
          <label>Temp</label>
          <span id="mob-temp" style="font-size:13px;margin-left:5px">--</span>
          <span id="mob-freq" style="font-size:10px;color:var(--gray);margin-left:8px">--</span>
        </div>
        <div class="sys-row"><label>Mem</label>
          <div class="sys-track"><div class="sys-fill" id="mob-mem-fill"></div>
            <span class="sys-val" id="mob-mem-val">--</span></div></div>
        <div class="sys-row"><label>Disk</label>
          <div class="sys-track"><div class="sys-fill" id="mob-disk-fill"></div>
            <span class="sys-val" id="mob-disk-val">--</span></div></div>
      </div>
      <div class="mob-section" style="flex:1;display:flex;flex-direction:column;min-height:180px">
        <div class="mob-title">LiDAR</div>
        <canvas id="mob-lidar-canvas" class="lidar-canvas" style="flex:1;min-height:150px"></canvas>
      </div>
    </div>

    <div class="mob-page" id="mob-logs">
      <div class="log-toolbar">
        <input class="log-filter" id="mob-log-filter" type="search" placeholder="Filter…"
               oninput="filterLog('mob-log-box','mob-log-filter')">
        <button onclick="logAutoScroll=!logAutoScroll;this.style.color=logAutoScroll?'var(--cyan)':'var(--gray)'" style="color:var(--cyan)">↓</button>
      </div>
      <div class="log-box" id="mob-log-box"></div>
    </div>

  </div><!-- /mob-tabs -->
</div><!-- /main -->

<!-- Mobile tab bar -->
<div id="tab-bar">
  <button class="tab-btn active" id="tab-drive" onclick="showTab('drive')">Drive</button>
  <button class="tab-btn" id="tab-telem" onclick="showTab('telem')">Telem</button>
  <button class="tab-btn" id="tab-gps"   onclick="showTab('gps')">GPS</button>
  <button class="tab-btn" id="tab-sys"   onclick="showTab('sys')">System</button>
  <button class="tab-btn" id="tab-logs"  onclick="showTab('logs')">Logs</button>
</div>

<!-- Panel chooser overlay -->
<div id="chooser">
  <div id="chooser-box">
    <div id="chooser-title">Select panel</div>
    <div id="chooser-grid"></div>
    <button id="chooser-cancel" onclick="closeChooser()">Cancel</button>
  </div>
</div>

<script>
'use strict';

// ── Palette ───────────────────────────────────────────────────────────────────
const C = {
  green:'#3cdc50', yellow:'#f0c828', orange:'#f08c28',
  red:'#dc3c3c',   cyan:'#3cc8dc',   gray:'#82829a',
  border:'#3c3c5a', bg:'#12121e', white:'#e6e6f0', purple:'#b450dc',
};
const MODE_COLORS = { MANUAL:C.yellow, AUTO:C.green, ESTOP:C.red };
const FIX_COLORS  = {0:C.red,1:C.yellow,2:C.orange,3:C.orange,4:C.green,5:C.cyan};
const NAV_COLORS  = {
  SEARCHING:C.yellow, ALIGNING:C.orange, APPROACHING:C.green,
  PASSING:C.cyan, COMPLETE:C.green, ERROR:C.red, IDLE:C.gray,
  WAITING_FIX:C.yellow, NAVIGATING:C.green, ARRIVED:C.cyan,
};

const el  = id => document.getElementById(id);
const fmt = (v, d, u) => v == null ? '--' : v.toFixed(d) + (u || '');

// ── State ─────────────────────────────────────────────────────────────────────
let lastState       = null;
let showBearing     = false;
let mobShowBearing  = false;
let logAutoScroll   = true;
let logPolling      = null;

// ── Quad panel system ─────────────────────────────────────────────────────────
// Panel type strings: 'camera:front_left', 'camera:front_right', 'camera:rear',
//                     'lidar', 'motor', 'telemetry', 'gps', 'system', 'logs'
const PANEL_DEFS = [
  { type:'camera:front_left',  label:'Camera FL'    },
  { type:'camera:front_right', label:'Camera FR'    },
  { type:'camera:rear',        label:'Camera Rear'  },
  { type:'lidar',              label:'LiDAR'        },
  { type:'motor',              label:'Motors'       },
  { type:'telemetry',          label:'Telemetry'    },
  { type:'gps',                label:'GPS'          },
  { type:'gps:skyview',        label:'GPS Sky View' },
  { type:'gps:track',          label:'GPS Track'    },
  { type:'gps:scatter',        label:'GPS Scatter'  },
  { type:'gps:signals',        label:'GPS Signals'  },
  { type:'system',             label:'System'       },
  { type:'logs',               label:'Logs'         },
  { type:'terminal',           label:'Terminal'     },
];

// Default layout — overridden by preset buttons
let quadTypes = ['camera:front_left', 'camera:front_right', 'camera:rear', 'lidar'];
let expandedQuad = -1;
let chooserQuad  = -1;

// Tap tracking for double-tap detection
const tapTimes = [0, 0, 0, 0];

// ── Panel HTML builders ───────────────────────────────────────────────────────
function htmlCamera(i, camKey) {
  const camSlug = camKey.replace(':','_');
  return `
    <div class="cam-wrap">
      <img class="cam-img" id="q${i}-cam-img" src="/stream/${camKey}" alt="">
      <span class="cam-nosig" id="q${i}-cam-nosig">No camera</span>
      <div class="cam-paused" id="q${i}-cam-paused" style="display:none">⏸ Stream paused</div>
      <canvas class="bearing-canvas" id="q${i}-bearing-canvas"></canvas>
      <div class="cam-rec-dot" id="q${i}-cam-rec-dot"></div>
      <div class="cam-nav-badge" id="q${i}-cam-nav-badge"></div>
    </div>`;
}
function htmlMotor(i) {
  return `
    <div class="mrow"><span class="mlabel">L</span>
      <div class="mtrack"><div class="mmid"></div>
        <div class="mfill" id="q${i}-fill-l"></div>
        <span class="mval" id="q${i}-val-l">+0.00</span></div></div>
    <div class="mrow"><span class="mlabel">R</span>
      <div class="mtrack"><div class="mmid"></div>
        <div class="mfill" id="q${i}-fill-r"></div>
        <span class="mval" id="q${i}-val-r">+0.00</span></div></div>`;
}
function htmlTelemetry(i) {
  return `
    <div class="tgrid">
      <div class="field"><label>Voltage</label><span id="q${i}-volt">--</span></div>
      <div class="field"><label>Current</label><span id="q${i}-curr">--</span></div>
    </div>
    <div class="tgrid three" style="padding-top:0">
      <div class="field"><label>Board</label><span id="q${i}-board">--</span></div>
      <div class="field"><label>Left</label><span id="q${i}-ltmp">--</span></div>
      <div class="field"><label>Right</label><span id="q${i}-rtmp">--</span></div>
    </div>
    <div class="telem-instruments">
      <div class="telem-inst-wrap">
        <canvas class="telem-inst-canvas" id="q${i}-compass-large"></canvas>
        <div class="telem-inst-lbl">
          Heading <span id="q${i}-hdg" style="color:var(--cyan)">---</span>
          &nbsp;<span id="q${i}-fault" style="color:var(--green)">OK</span>
        </div>
      </div>
      <div class="telem-inst-wrap">
        <canvas class="telem-inst-canvas" id="q${i}-horizon"></canvas>
        <div class="telem-inst-lbl">
          P <span id="q${i}-pitch" style="color:var(--cyan)">---</span>
          &nbsp; R <span id="q${i}-roll" style="color:var(--cyan)">---</span>
        </div>
      </div>
    </div>`;
}
function htmlGps(i) {
  return `
    <div class="ggrid">
      <div class="field"><label>Latitude</label><span id="q${i}-lat">--</span></div>
      <div class="field"><label>Altitude</label><span id="q${i}-alt">--</span></div>
      <div class="field"><label>Longitude</label><span id="q${i}-lon">--</span></div>
      <div class="field"><label>H-Error</label><span id="q${i}-herr">--</span></div>
    </div>
    <div class="ggrid three">
      <div class="field"><label>Fix</label><span id="q${i}-fix">--</span></div>
      <div class="field"><label>Sats</label><span id="q${i}-sats">--</span></div>
      <div class="field"><label>HDOP</label><span id="q${i}-hdop">--</span></div>
    </div>
    <div class="ggrid">
      <div class="field"><label>Speed</label><span id="q${i}-spd">--</span></div>
      <div class="field"><label>Course</label><span id="q${i}-crs">--</span></div>
    </div>
    <div class="gps-row">
      <label>RTK</label>
      <span class="rtk-dot" id="q${i}-rtk-dot"></span>
      <span id="q${i}-rtk-st" style="font-size:11px">--</span>
      <span id="q${i}-rtk-bytes" style="font-size:10px;color:var(--gray);margin-left:4px"></span>
    </div>
    <div class="sig-wrap">
      <div style="font-size:9px;color:var(--gray);margin-bottom:2px">Signal (SNR)</div>
      <div class="sig-bars" id="q${i}-sig-bars"></div>
      <div class="sig-lbl" id="q${i}-sig-lbls" style="display:flex;gap:2px;margin-top:1px"></div>
    </div>
    <div style="padding:3px 8px">
      <span id="q${i}-nav-info" style="font-size:11px;color:var(--gray)"></span>
    </div>`;
}
function htmlGpsSkyview(i) {
  return `
    <div class="gps-canvas-wrap">
      <div style="font-size:9px;color:var(--gray);padding:0 2px 2px">
        Sky View · N↑ · colour = SNR · <span id="q${i}-sky-sats" style="color:var(--cyan)">0 sats</span>
      </div>
      <canvas class="sky-canvas" id="q${i}-sky-canvas"></canvas>
    </div>`;
}
function htmlGpsTrack(i) {
  return `
    <div class="gps-canvas-wrap">
      <div style="font-size:9px;color:var(--gray);padding:0 2px 2px">
        GPS Track · <span id="q${i}-trk-pts" style="color:var(--cyan)">0 pts</span>
        <button onclick="gpsTrackClear()" style="font-size:9px;padding:0 4px;margin-left:6px">Clear</button>
      </div>
      <canvas class="track-canvas" id="q${i}-trk-canvas"></canvas>
    </div>`;
}
function htmlGpsScatter(i) {
  return `
    <div class="gps-canvas-wrap">
      <div style="font-size:9px;color:var(--gray);padding:0 2px 2px">
        Position Scatter · <span id="q${i}-scat-pts" style="color:var(--cyan)">0 pts</span>
        <button onclick="gpsTrackClear()" style="font-size:9px;padding:0 4px;margin-left:6px">Clear</button>
      </div>
      <canvas class="scatter-canvas" id="q${i}-scat-canvas"></canvas>
    </div>`;
}
function htmlGpsSignals(i) {
  return `
    <div class="gps-canvas-wrap">
      <div style="font-size:9px;color:var(--gray);padding:0 2px 2px">Satellite SNR (dB-Hz)</div>
      <canvas class="sig-canvas" id="q${i}-gsig-canvas"></canvas>
    </div>`;
}
function htmlTerminal(i) {
  return `
    <div class="log-toolbar">
      <input class="log-filter" id="q${i}-log-filter" type="search" placeholder="Filter…"
             oninput="filterLog('q${i}-term-box','q${i}-log-filter')">
      <button onclick="logAutoScroll=!logAutoScroll;this.style.color=logAutoScroll?'#00cc44':'var(--gray)'"
              style="color:#00cc44">↓</button>
    </div>
    <div class="term-box" id="q${i}-term-box"></div>`;
}
function htmlSystem(i) {
  return `
    <div class="sys-row"><label>CPU</label>
      <div class="sys-track"><div class="sys-fill" id="q${i}-cpu-fill"></div>
        <span class="sys-val" id="q${i}-cpu-val">--</span></div></div>
    <div class="sys-row" style="padding:3px 8px">
      <label>Temp</label><span id="q${i}-temp" style="font-size:13px;margin-left:5px">--</span>
      <span id="q${i}-freq" style="font-size:10px;color:var(--gray);margin-left:8px">--</span>
    </div>
    <div class="sys-row"><label>Mem</label>
      <div class="sys-track"><div class="sys-fill" id="q${i}-mem-fill"></div>
        <span class="sys-val" id="q${i}-mem-val">--</span></div></div>
    <div class="sys-row"><label>Disk</label>
      <div class="sys-track"><div class="sys-fill" id="q${i}-disk-fill"></div>
        <span class="sys-val" id="q${i}-disk-val">--</span></div></div>`;
}
function htmlLidar(i) {
  return `<canvas id="q${i}-lidar-canvas" class="lidar-canvas"></canvas>`;
}
function htmlLogs(i) {
  return `
    <div class="log-toolbar">
      <input class="log-filter" id="q${i}-log-filter" type="search" placeholder="Filter…"
             oninput="filterLog('q${i}-log-box','q${i}-log-filter')">
      <button onclick="logAutoScroll=!logAutoScroll;this.style.color=logAutoScroll?'var(--cyan)':'var(--gray)'" style="color:var(--cyan)">↓</button>
    </div>
    <div class="log-box" id="q${i}-log-box"></div>`;
}

// ── Render quad ───────────────────────────────────────────────────────────────
function renderQuad(i) {
  const type   = quadTypes[i];
  const [kind, camKey] = type.includes(':') ? type.split(':') : [type, null];
  const def    = PANEL_DEFS.find(p => p.type === type) || { label: type };
  const titleEl = el(`q${i}-title`);
  if (titleEl) titleEl.textContent = def.label;

  // Build header buttons per panel type
  const hdr = el(`q${i}-hdr`);
  if (hdr) {
    // Preserve title span, rebuild buttons
    const titleSpan = hdr.querySelector('.quad-title');
    hdr.innerHTML = '';
    hdr.appendChild(titleSpan);
    if (kind === 'camera') {
      hdr.innerHTML += `
        <button onclick="sendCmd('aruco_toggle',{cam:'${camKey}'})" style="font-size:10px" id="q${i}-btn-aruco">ArUco</button>
        <button onclick="sendCmd('rotate_ccw',{cam:'${camKey}'})" style="font-size:10px">↺</button>
        <button onclick="sendCmd('rotate_cw',{cam:'${camKey}'})"  style="font-size:10px">↻</button>
        <button onclick="toggleBearing(${i})" id="q${i}-btn-bearing" style="font-size:10px">Bearing</button>
        <button onclick="sendCmd('record_toggle',{cam:'${camKey}'})" id="q${i}-btn-rec" style="font-size:10px">⏺</button>
        <button onclick="sendCmd('record_stop',{cam:'${camKey}'})" style="font-size:10px" title="Stop recording">⏹</button>
        <button onclick="toggleStream(${i})" id="q${i}-btn-stream" style="font-size:10px" title="Pause/resume stream">📷</button>`;
    }
  }

  // Build body
  const body = el(`q${i}-body`);
  if (!body) return;
  let html = '';
  if      (kind === 'camera')   html = htmlCamera(i, camKey);
  else if (kind === 'motor')    html = htmlMotor(i);
  else if (kind === 'telemetry')html = htmlTelemetry(i);
  else if (kind === 'gps' && camKey === 'skyview')  html = htmlGpsSkyview(i);
  else if (kind === 'gps' && camKey === 'track')    html = htmlGpsTrack(i);
  else if (kind === 'gps' && camKey === 'scatter')  html = htmlGpsScatter(i);
  else if (kind === 'gps' && camKey === 'signals')  html = htmlGpsSignals(i);
  else if (kind === 'gps')      html = htmlGps(i);
  else if (kind === 'system')   html = htmlSystem(i);
  else if (kind === 'lidar')    html = htmlLidar(i);
  else if (kind === 'logs')     html = htmlLogs(i);
  else if (kind === 'terminal') html = htmlTerminal(i);
  body.innerHTML = html;

  if (kind === 'lidar') resizeLidar(`q${i}-lidar-canvas`);
  if (kind === 'camera') {
    const img = el(`q${i}-cam-img`);
    if (img) img.addEventListener('load', function(){
      if (this.naturalWidth) camNatW = this.naturalWidth;
      if (this.naturalHeight) camNatH = this.naturalHeight;
    });
  }
  if (kind === 'logs' || kind === 'terminal') startLogPolling();
}

function renderAllQuads() {
  for (let i = 0; i < 4; i++) renderQuad(i);
}

// ── Update functions ──────────────────────────────────────────────────────────
let camNatW = 640, camNatH = 480;

function updateBar(fillId, valId, v) {
  const fill = el(fillId), valE = el(valId);
  if (!fill || !valE) return;
  const color = v > 0 ? C.green : v < 0 ? C.orange : null;
  if (v >= 0) { fill.style.left='50%';         fill.style.width=(v*50)+'%'; }
  else        { fill.style.left=(50+v*50)+'%'; fill.style.width=(-v*50)+'%'; }
  fill.style.backgroundColor = color || 'transparent';
  valE.textContent = (v>=0?'+':'')+v.toFixed(2);
  valE.style.color = color || C.gray;
}

function pctColor(p,w,c){ return p>=c?C.red:p>=w?C.yellow:C.green; }
function sysBar(fid, vid, pct, lbl, color) {
  const f=el(fid); if(!f) return;
  f.style.width=Math.min(pct,100)+'%'; f.style.backgroundColor=color;
  const v=el(vid); if(!v) return;
  v.textContent=lbl; v.style.color=pct>55?C.bg:color;
}

// Track which quad streams are paused (src cleared)
const streamPaused = {};
function toggleStream(i) {
  const img     = el(`q${i}-cam-img`);
  const btn     = el(`q${i}-btn-stream`);
  const overlay = el(`q${i}-cam-paused`);
  if (!img) return;
  if (streamPaused[i]) {
    const type = quadTypes[i];
    const camKey = type.includes(':') ? type.split(':')[1] : null;
    if (camKey) img.src = '/stream/' + camKey;
    streamPaused[i] = false;
    if (btn)     { btn.textContent = '📷'; btn.style.color = ''; }
    if (overlay)   overlay.style.display = 'none';
  } else {
    img.src = '';
    streamPaused[i] = true;
    if (btn)     { btn.textContent = '▶'; btn.style.color = 'var(--green)'; }
    if (overlay)   overlay.style.display = 'block';
  }
}
function stopAllStreams() {
  for (let i = 0; i < 4; i++) {
    const type = quadTypes[i];
    if (type && type.startsWith('camera:') && !streamPaused[i]) toggleStream(i);
  }
}

function updateCameraPanel(i, camKey, s) {
  const okKey  = camKey==='front_left'?'cam_fl_ok' : camKey==='front_right'?'cam_fr_ok':'cam_re_ok';
  const recKey = camKey==='front_left'?'cam_fl_rec': camKey==='front_right'?'cam_fr_rec':'cam_re_rec';
  const ok  = !!s[okKey];
  const rec = !!s[recKey];

  const imgEl = el(`q${i}-cam-img`);
  if (!imgEl) return;
  imgEl.style.display       = ok ? 'block' : 'none';
  const nosig = el(`q${i}-cam-nosig`); if(nosig) nosig.style.display = ok?'none':'block';
  const dot   = el(`q${i}-cam-rec-dot`);
  if (dot) dot.style.display = rec ? 'block' : 'none';

  // Nav badge
  const badge = el(`q${i}-cam-nav-badge`);
  if (badge) updateNavBadgeEl(badge, s);

  // ArUco button highlight — per camera
  const camEnabled = s.aruco_enabled && s.aruco_enabled[camKey];
  const aBtn = el(`q${i}-btn-aruco`);
  if (aBtn) {
    aBtn.style.color       = camEnabled ? C.green : C.gray;
    aBtn.style.borderColor = camEnabled ? C.green : C.border;
  }

  // ArUco overlay — tag boxes when ArUco active; bearing info when Bearing toggled
  const bCanvas = el(`q${i}-bearing-canvas`);
  if (bCanvas) {
    const camAruco = s.aruco && s.aruco[camKey] ? s.aruco[camKey] : null;
    const show = camEnabled && camAruco !== null;
    bCanvas.style.display = show ? 'block' : 'none';
    if (show) drawBearingOverlay(bCanvas, camAruco, s.nav_gate, s.telemetry.heading, showBearing);
  }
}

function updateMotorPanel(i, s) {
  updateBar(`q${i}-fill-l`, `q${i}-val-l`, s.drive.left);
  updateBar(`q${i}-fill-r`, `q${i}-val-r`, s.drive.right);
}

function updateTelemPanel(i, s) {
  const t = s.telemetry;
  const se = (id, v, c) => { const e=el(id); if(!e)return; e.textContent=v; if(c)e.style.color=c; };
  se(`q${i}-volt`,  fmt(t.voltage,1,' V'));
  se(`q${i}-curr`,  fmt(t.current,2,' A'));
  se(`q${i}-board`, fmt(t.board_temp,0,'°C'));
  se(`q${i}-ltmp`,  fmt(t.left_temp,0,'°C'));
  se(`q${i}-rtmp`,  fmt(t.right_temp,0,'°C'));
  const fault = t.left_fault||t.right_fault;
  const fEl = el(`q${i}-fault`);
  if (fEl){ fEl.textContent=(t.left_fault?'FAULT-L ':'')+(t.right_fault?'FAULT-R':'')||'OK';
            fEl.style.color=fault?C.red:C.green; }
  se(`q${i}-hdg`,   t.heading!=null ? t.heading.toFixed(1)+'°' : '---',
                    t.heading!=null ? C.cyan : C.gray);
  se(`q${i}-pitch`, t.pitch!=null ? (t.pitch>=0?'+':'')+t.pitch.toFixed(1)+'°' : '---',
                    t.pitch!=null ? C.cyan : C.gray);
  se(`q${i}-roll`,  t.roll!=null  ? (t.roll>=0?'+':'')+t.roll.toFixed(1)+'°'   : '---',
                    t.roll!=null  ? C.cyan : C.gray);
  drawCompassLarge(el(`q${i}-compass-large`), t.heading);
  drawArtificialHorizon(el(`q${i}-horizon`), t.pitch, t.roll);
}

// ── SNR colour helper ─────────────────────────────────────────────────────────
function snrColor(snr) {
  if (snr == null || snr < 10) return C.red;
  if (snr < 25) return C.orange;
  if (snr < 35) return C.yellow;
  if (snr < 42) return C.green;
  return C.cyan;
}
// ── GPS position history (client-side, shared across panels) ──────────────────
const GPS_TRACK_MAX = 600;
let gpsTrackPts = [];   // [{lat,lon}]
function gpsTrackClear() { gpsTrackPts = []; }
function gpsTrackPush(g) {
  if (g.latitude == null || g.longitude == null) return;
  const last = gpsTrackPts.length ? gpsTrackPts[gpsTrackPts.length-1] : null;
  if (last && last.lat === g.latitude && last.lon === g.longitude) return;
  gpsTrackPts.push({lat: g.latitude, lon: g.longitude});
  if (gpsTrackPts.length > GPS_TRACK_MAX) gpsTrackPts.shift();
}

function updateGpsPanel(i, s) {
  const g=s.gps, ok=s.gps_ok;
  const fixColor=FIX_COLORS[g.fix_quality]||C.gray;
  const se=(id,v,c)=>{ const e=el(id); if(!e)return; e.textContent=v; if(c)e.style.color=c; };
  se(`q${i}-lat`,  g.latitude  !=null ? g.latitude.toFixed(7)  : '--');
  se(`q${i}-lon`,  g.longitude !=null ? g.longitude.toFixed(7) : '--');
  se(`q${i}-alt`,  g.altitude  !=null ? g.altitude.toFixed(1)+' m' : '--');
  se(`q${i}-herr`, g.h_error_m !=null ? g.h_error_m.toFixed(3)+' m' : '--');
  se(`q${i}-sats`, g.satellites!=null ? g.satellites+(g.satellites_view?'/'+g.satellites_view:'') : '--');
  se(`q${i}-hdop`, fmt(g.hdop,2));
  se(`q${i}-fix`,  ok ? g.fix_quality_name : 'No GPS', ok?fixColor:C.gray);
  se(`q${i}-spd`,  g.speed   !=null ? (g.speed*3.6).toFixed(1)+' km/h' : '--');
  se(`q${i}-crs`,  g.heading !=null ? g.heading.toFixed(1)+'°' : '--');
  // RTK link
  const rtkDot = el(`q${i}-rtk-dot`);
  const rtkSt  = el(`q${i}-rtk-st`);
  const rtkBy  = el(`q${i}-rtk-bytes`);
  if (rtkDot && rtkSt) {
    const st = g.ntrip_status || '';
    const dotColor = st==='connected'?C.green:st==='connecting'?C.yellow:st==='error'?C.red:C.gray;
    const label    = st||'No RTK';
    rtkDot.style.background = dotColor;
    rtkSt.textContent = label;
    rtkSt.style.color = dotColor;
  }
  if (rtkBy) {
    rtkBy.textContent = g.ntrip_bytes_recv > 0
      ? (g.ntrip_bytes_recv < 1024 ? g.ntrip_bytes_recv+'B'
         : g.ntrip_bytes_recv < 1048576 ? (g.ntrip_bytes_recv/1024).toFixed(1)+'KB'
         : (g.ntrip_bytes_recv/1048576).toFixed(2)+'MB')
      : '';
  }
  // Compact signal bars
  const barsEl = el(`q${i}-sig-bars`);
  const lblsEl = el(`q${i}-sig-lbls`);
  if (barsEl && g.satellites_data) {
    const sats = [...g.satellites_data].sort((a,b)=>(b.snr||0)-(a.snr||0)).slice(0,16);
    barsEl.innerHTML = sats.map(sv => {
      const h = Math.max(2, Math.round(((sv.snr||0)/55)*100));
      const c = snrColor(sv.snr);
      return `<div class="sig-bar" style="height:${h}%;background:${c}" title="${sv.svid||''} ${sv.snr||0}dB"></div>`;
    }).join('');
    if (lblsEl) lblsEl.innerHTML = sats.map(sv =>
      `<div style="flex:1;font-size:8px;text-align:center;color:${snrColor(sv.snr)};overflow:hidden">${sv.svid||'?'}</div>`
    ).join('');
  }
  const ni=el(`q${i}-nav-info`); if(ni) updateNavInfoEl(ni, s);
}

// ── Sky View panel ────────────────────────────────────────────────────────────
function updateSkyviewPanel(i, s) {
  const g = s.gps;
  const c = el(`q${i}-sky-canvas`); if (!c) return;
  if (!_syncCanvasSize(c)) return;
  const W = c.width, H = c.height;
  const ctx = c.getContext('2d');
  const cx = W/2, cy = H/2, r = Math.min(W,H)/2 - 10;
  ctx.clearRect(0,0,W,H);
  // Rings
  ctx.strokeStyle = '#333'; ctx.lineWidth = 1;
  [1, 2/3, 1/3].forEach(f => {
    ctx.beginPath(); ctx.arc(cx,cy,r*f,0,2*Math.PI); ctx.stroke();
  });
  // Cardinal labels
  ctx.fillStyle='#666'; ctx.font='10px sans-serif'; ctx.textAlign='center';
  ctx.fillText('N',cx,cy-r+10); ctx.fillText('S',cx,cy+r-2);
  ctx.textAlign='right';  ctx.fillText('W',cx-r+2,cy+4);
  ctx.textAlign='left';   ctx.fillText('E',cx+r-2,cy+4);
  // Satellites
  const sats = g.satellites_data || [];
  sats.forEach(sv => {
    const elev = sv.elev || 0, azim = sv.azim || 0;
    const dist = r * (1 - elev/90);
    const rad  = (azim - 90) * Math.PI / 180;
    const sx = cx + dist * Math.cos(rad);
    const sy = cy + dist * Math.sin(rad);
    const col = snrColor(sv.snr);
    ctx.beginPath(); ctx.arc(sx, sy, 5, 0, 2*Math.PI);
    ctx.fillStyle = col; ctx.fill();
    ctx.strokeStyle = '#000'; ctx.lineWidth=0.5; ctx.stroke();
    ctx.fillStyle = '#fff'; ctx.font='8px sans-serif'; ctx.textAlign='center';
    ctx.fillText(sv.svid||'', sx, sy+3);
  });
  const cntEl = el(`q${i}-sky-sats`);
  if (cntEl) cntEl.textContent = sats.length + ' sat' + (sats.length===1?'':'s');
}

// ── GPS Track panel ───────────────────────────────────────────────────────────
function updateTrackPanel(i, s) {
  gpsTrackPush(s.gps);
  const c = el(`q${i}-trk-canvas`); if (!c) return;
  if (!_syncCanvasSize(c)) return;
  const W = c.width, H = c.height;
  const ctx = c.getContext('2d');
  ctx.clearRect(0,0,W,H);
  const pts = el(`q${i}-trk-pts`);
  if (pts) pts.textContent = gpsTrackPts.length + ' pts';
  if (gpsTrackPts.length < 2) return;
  const lats = gpsTrackPts.map(p=>p.lat), lons = gpsTrackPts.map(p=>p.lon);
  const minLat=Math.min(...lats), maxLat=Math.max(...lats);
  const minLon=Math.min(...lons), maxLon=Math.max(...lons);
  const spanLat=maxLat-minLat||1e-6, spanLon=maxLon-minLon||1e-6;
  const pad=16;
  const toX = lon => pad + (lon-minLon)/spanLon * (W-pad*2);
  const toY = lat => H - pad - (lat-minLat)/spanLat * (H-pad*2);
  // Trail — fade older segments
  ctx.lineWidth = 1.5;
  for (let k=1; k<gpsTrackPts.length; k++) {
    const alpha = 0.3 + 0.7*(k/gpsTrackPts.length);
    ctx.strokeStyle = `rgba(0,200,100,${alpha})`;
    ctx.beginPath();
    ctx.moveTo(toX(gpsTrackPts[k-1].lon), toY(gpsTrackPts[k-1].lat));
    ctx.lineTo(toX(gpsTrackPts[k].lon),   toY(gpsTrackPts[k].lat));
    ctx.stroke();
  }
  // Current position dot
  const cur = gpsTrackPts[gpsTrackPts.length-1];
  ctx.beginPath(); ctx.arc(toX(cur.lon), toY(cur.lat), 5, 0, 2*Math.PI);
  ctx.fillStyle = C.cyan; ctx.fill();
}

// ── GPS Scatter panel ─────────────────────────────────────────────────────────
function updateScatterPanel(i, s) {
  gpsTrackPush(s.gps);
  const c = el(`q${i}-scat-canvas`); if (!c) return;
  if (!_syncCanvasSize(c)) return;
  const W = c.width, H = c.height;
  const ctx = c.getContext('2d');
  ctx.clearRect(0,0,W,H);
  const pts = el(`q${i}-scat-pts`);
  if (pts) pts.textContent = gpsTrackPts.length + ' pts';
  if (!gpsTrackPts.length) return;
  const lats = gpsTrackPts.map(p=>p.lat), lons = gpsTrackPts.map(p=>p.lon);
  const minLat=Math.min(...lats), maxLat=Math.max(...lats);
  const minLon=Math.min(...lons), maxLon=Math.max(...lons);
  const spanLat=maxLat-minLat||1e-6, spanLon=maxLon-minLon||1e-6;
  const span = Math.max(spanLat, spanLon);
  const midLat=(minLat+maxLat)/2, midLon=(minLon+maxLon)/2;
  const pad=16;
  const scale = (Math.min(W,H)-pad*2)/span;
  const toX = lon => W/2 + (lon-midLon)*scale;
  const toY = lat => H/2 - (lat-midLat)*scale;
  // Cross-hair
  ctx.strokeStyle='#333'; ctx.lineWidth=1;
  ctx.beginPath(); ctx.moveTo(W/2,pad); ctx.lineTo(W/2,H-pad); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(pad,H/2); ctx.lineTo(W-pad,H/2); ctx.stroke();
  // Dots — colour oldest→newest
  const n = gpsTrackPts.length;
  gpsTrackPts.forEach((p, k) => {
    const t = k/Math.max(n-1,1);
    ctx.beginPath(); ctx.arc(toX(p.lon), toY(p.lat), 2, 0, 2*Math.PI);
    ctx.fillStyle = `rgba(${Math.round(255*(1-t))},${Math.round(200*t)},100,0.8)`;
    ctx.fill();
  });
  // Current position
  const cur = gpsTrackPts[n-1];
  ctx.beginPath(); ctx.arc(toX(cur.lon), toY(cur.lat), 5, 0, 2*Math.PI);
  ctx.fillStyle = C.cyan; ctx.fill();
  // Scale bar — approx metres
  const mPerDeg = 111320;
  const spanM = span * mPerDeg;
  ctx.fillStyle='#666'; ctx.font='9px sans-serif'; ctx.textAlign='center';
  ctx.fillText(spanM<1 ? (spanM*100).toFixed(0)+'cm'
               : spanM<1000 ? spanM.toFixed(1)+'m'
               : (spanM/1000).toFixed(2)+'km', W/2, H-2);
}

// ── GPS Signals panel ─────────────────────────────────────────────────────────
function updateSignalsPanel(i, s) {
  const c = el(`q${i}-gsig-canvas`); if (!c) return;
  if (!_syncCanvasSize(c)) return;
  const W = c.width, H = c.height;
  const ctx = c.getContext('2d');
  ctx.clearRect(0,0,W,H);
  const sats = [...(s.gps.satellites_data||[])].sort((a,b)=>a.svid<b.svid?-1:1);
  if (!sats.length) {
    ctx.fillStyle='#444'; ctx.font='12px sans-serif'; ctx.textAlign='center';
    ctx.fillText('No satellite data', W/2, H/2); return;
  }
  const pad=20, barW=Math.max(4, Math.floor((W-pad*2)/sats.length)-2);
  const maxSnr=55, barArea=H-pad-10;
  // Scale labels
  ctx.fillStyle='#555'; ctx.font='8px sans-serif'; ctx.textAlign='right';
  [0,20,40].forEach(v => {
    const y = pad + barArea - (v/maxSnr)*barArea;
    ctx.fillText(v, pad-2, y+3);
    ctx.strokeStyle='#2a2a2a'; ctx.lineWidth=1;
    ctx.beginPath(); ctx.moveTo(pad,y); ctx.lineTo(W,y); ctx.stroke();
  });
  sats.forEach((sv, idx) => {
    const snr = sv.snr || 0;
    const bh  = Math.max(2, (snr/maxSnr)*barArea);
    const x   = pad + idx*(barW+2);
    const y   = pad + barArea - bh;
    ctx.fillStyle = snrColor(snr);
    ctx.fillRect(x, y, barW, bh);
    // Label
    ctx.fillStyle = '#666'; ctx.font = '8px sans-serif'; ctx.textAlign = 'center';
    ctx.fillText(sv.svid||'?', x+barW/2, H-2);
    ctx.fillStyle = snrColor(snr); ctx.font = '8px sans-serif';
    ctx.fillText(snr||'', x+barW/2, y-1);
  });
}

// ── Terminal panel ────────────────────────────────────────────────────────────
function updateTerminalPanel(i, s) {
  // terminal boxes pick up log lines via the shared fetchLogs() / logPolling mechanism
  // The term-box is treated the same as log-box by fetchLogs — it has class term-box
  // but we hook into the same querySelectorAll by adding it to the selector
}

function updateSystemPanel(i, s) {
  const sys=s.system;
  const cpu=pctColor(sys.cpu_percent,60,85);
  const mem=pctColor(sys.mem_percent,70,90);
  const dsk=pctColor(sys.disk_percent,70,90);
  sysBar(`q${i}-cpu-fill`,`q${i}-cpu-val`,sys.cpu_percent,`${sys.cpu_percent.toFixed(0)}%`,cpu);
  sysBar(`q${i}-mem-fill`,`q${i}-mem-val`,sys.mem_percent,
    `${sys.mem_percent.toFixed(0)}% ${(sys.mem_used_mb/1024).toFixed(1)}/${(sys.mem_total_mb/1024).toFixed(1)}GB`,mem);
  sysBar(`q${i}-disk-fill`,`q${i}-disk-val`,sys.disk_percent,
    `${sys.disk_percent.toFixed(0)}% ${sys.disk_used_gb.toFixed(1)}/${sys.disk_total_gb.toFixed(1)}GB`,dsk);
  const te=el(`q${i}-temp`); if(te){te.textContent=`${sys.cpu_temp_c.toFixed(0)}°C`;
    te.style.color=sys.cpu_temp_c<60?C.green:sys.cpu_temp_c<75?C.yellow:C.red;}
  const fe=el(`q${i}-freq`); if(fe) fe.textContent=`${sys.cpu_freq_mhz.toFixed(0)} MHz`;
}

function updateLidarPanel(i, s) {
  const c = el(`q${i}-lidar-canvas`); if(!c) return;
  drawLidar(c, s.lidar.angles, s.lidar.distances);
}

// ── Shared nav helpers ────────────────────────────────────────────────────────
function updateNavBadgeEl(badge, s) {
  const autoCamera = s.mode==='AUTO' && (s.auto_type==='Camera'||s.auto_type==='Cam+GPS');
  const autoGps    = s.mode==='AUTO' && (s.auto_type==='GPS'   ||s.auto_type==='Cam+GPS');
  if (!autoCamera && !autoGps) { badge.style.display='none'; return; }
  const nc = NAV_COLORS[s.nav_state]||C.gray;
  let txt;
  if (autoGps && s.nav_state && s.nav_state!=='IDLE') {
    txt = `GPS:${s.nav_state} WP:${s.nav_wp}`;
    if (s.nav_wp_dist!=null) txt += ` ${s.nav_wp_dist.toFixed(1)}m`;
    if (s.nav_wp_bear!=null) txt += ` ${s.nav_wp_bear.toFixed(0)}°`;
  } else {
    txt = `${s.nav_state} G${s.nav_gate}`;
    if (s.nav_bearing_err!=null) txt += ` ${s.nav_bearing_err>=0?'+':''}${s.nav_bearing_err.toFixed(1)}°`;
  }
  badge.textContent=txt; badge.style.color=nc; badge.style.display='block';
}
function updateNavInfoEl(el2, s) {
  if (!el2) return;
  const autoGps = s.mode==='AUTO' && (s.auto_type==='GPS'||s.auto_type==='Cam+GPS');
  if (!s.nav_state || s.nav_state==='IDLE') { el2.textContent=''; return; }
  const nc=NAV_COLORS[s.nav_state]||C.gray;
  let txt = autoGps ? `GPS:${s.nav_state} WP${s.nav_wp}` : `NAV:${s.nav_state} G${s.nav_gate}`;
  if (autoGps && s.nav_wp_dist!=null) txt += ` ${s.nav_wp_dist.toFixed(1)}m`;
  el2.textContent=txt; el2.style.color=nc;
}

// ── Apply state to all quads ──────────────────────────────────────────────────
function updateAllQuads(s) {
  for (let i = 0; i < 4; i++) {
    const type = quadTypes[i];
    const [kind, camKey] = type.includes(':') ? type.split(':') : [type, null];
    if      (kind==='camera')    updateCameraPanel(i, camKey, s);
    else if (kind==='motor')     updateMotorPanel(i, s);
    else if (kind==='telemetry') updateTelemPanel(i, s);
    else if (kind==='gps' && camKey==='skyview')  updateSkyviewPanel(i, s);
    else if (kind==='gps' && camKey==='track')    updateTrackPanel(i, s);
    else if (kind==='gps' && camKey==='scatter')  updateScatterPanel(i, s);
    else if (kind==='gps' && camKey==='signals')  updateSignalsPanel(i, s);
    else if (kind==='gps')       updateGpsPanel(i, s);
    else if (kind==='system')    updateSystemPanel(i, s);
    else if (kind==='lidar')     updateLidarPanel(i, s);
    else if (kind==='terminal')  updateTerminalPanel(i, s);
    // logs: updated by polling, not state
  }
}

// ── Mobile update ─────────────────────────────────────────────────────────────
function updateMobile(s) {
  const t=s.telemetry, g=s.gps, sys=s.system;
  const ok = s.cam_fl_ok;
  const se=(id,v,c)=>{ const e=el(id); if(!e)return; e.textContent=v; if(c)e.style.color=c; };

  // Drive tab
  const mi=el('mob-cam-img');
  if(mi){ mi.style.display=ok?'block':'none'; }
  const mn=el('mob-cam-nosig'); if(mn) mn.style.display=ok?'none':'block';
  const md=el('mob-cam-rec-dot'); if(md) md.style.display=s.cam_fl_rec?'block':'none';
  const mbadge=el('mob-cam-nav-badge'); if(mbadge) updateNavBadgeEl(mbadge,s);
  updateBar('mob-fill-l','mob-val-l',s.drive.left);
  updateBar('mob-fill-r','mob-val-r',s.drive.right);
  const aBtn=el('mob-btn-aruco');
  if(aBtn){ aBtn.textContent=s.aruco_enabled?'ArUco: ON':'ArUco: OFF';
            aBtn.style.color=s.aruco_enabled?C.green:C.gray; }
  const ni=el('mob-nav-info'); if(ni) updateNavInfoEl(ni,s);

  // Bearing overlay on mobile — tag boxes when ArUco active; bearing info when toggled
  const mc=el('mob-bearing-canvas');
  if(mc){
    const camAruco = s.aruco && s.aruco['front_left'] ? s.aruco['front_left'] : null;
    const camEnabled = s.aruco_enabled && s.aruco_enabled['front_left'];
    const show = camEnabled && camAruco !== null;
    mc.style.display=show?'block':'none';
    if(show) drawBearingOverlay(mc, camAruco, s.nav_gate, t.heading, mobShowBearing);
  }

  // Telem tab
  se('mob-volt',  fmt(t.voltage,1,' V'));
  se('mob-curr',  fmt(t.current,2,' A'));
  se('mob-board', fmt(t.board_temp,0,'°C'));
  se('mob-ltmp',  fmt(t.left_temp,0,'°C'));
  se('mob-rtmp',  fmt(t.right_temp,0,'°C'));
  const fault=t.left_fault||t.right_fault;
  se('mob-fault',(t.left_fault?'FAULT-L ':'')+(t.right_fault?'FAULT-R':'')||'OK',fault?C.red:C.green);
  se('mob-hdg-text',t.heading!=null?t.heading.toFixed(1)+'°':'---',t.heading!=null?C.cyan:C.gray);
  drawCompass(el('mob-compass'),t.heading);
  se('mob-pitch',t.pitch!=null?(t.pitch>=0?'+':'')+t.pitch.toFixed(1)+'°':'---',t.pitch!=null?C.cyan:C.gray);
  se('mob-roll', t.roll !=null?(t.roll>=0?'+':'')+t.roll.toFixed(1)+'°':'---',t.roll !=null?C.cyan:C.gray);

  // GPS tab
  const fixColor=FIX_COLORS[g.fix_quality]||C.gray;
  se('mob-lat',  g.latitude !=null ? g.latitude.toFixed(7) : '--');
  se('mob-lon',  g.longitude!=null ? g.longitude.toFixed(7): '--');
  se('mob-alt',  g.altitude !=null ? g.altitude.toFixed(1)+' m':'--');
  se('mob-herr', g.h_error_m!=null ? g.h_error_m.toFixed(3)+' m':'--');
  se('mob-sats', g.satellites!=null ? g.satellites+(g.satellites_view?'/'+g.satellites_view:''):'--');
  se('mob-hdop', fmt(g.hdop,2));
  se('mob-fix',  s.gps_ok ? g.fix_quality_name : 'No GPS', s.gps_ok?fixColor:C.gray);

  // System tab
  const cpu=pctColor(sys.cpu_percent,60,85);
  const mem=pctColor(sys.mem_percent,70,90);
  const dsk=pctColor(sys.disk_percent,70,90);
  sysBar('mob-cpu-fill','mob-cpu-val',sys.cpu_percent,`${sys.cpu_percent.toFixed(0)}%`,cpu);
  sysBar('mob-mem-fill','mob-mem-val',sys.mem_percent,
    `${sys.mem_percent.toFixed(0)}% ${(sys.mem_used_mb/1024).toFixed(1)}/${(sys.mem_total_mb/1024).toFixed(1)}GB`,mem);
  sysBar('mob-disk-fill','mob-disk-val',sys.disk_percent,
    `${sys.disk_percent.toFixed(0)}% ${sys.disk_used_gb.toFixed(1)}/${sys.disk_total_gb.toFixed(1)}GB`,dsk);
  const te=el('mob-temp');
  if(te){te.textContent=`${sys.cpu_temp_c.toFixed(0)}°C`;
    te.style.color=sys.cpu_temp_c<60?C.green:sys.cpu_temp_c<75?C.yellow:C.red;}
  const fe=el('mob-freq'); if(fe) fe.textContent=`${sys.cpu_freq_mhz.toFixed(0)} MHz`;
  drawLidar(el('mob-lidar-canvas'),s.lidar.angles,s.lidar.distances);
}

// ── applyState ────────────────────────────────────────────────────────────────
function applyState(s) {
  lastState = s;
  updateStatusBar(s);
  el('no-motors-banner').style.display = s.no_motors ? 'block' : 'none';
  if (s.gps_ok && s.gps) gpsTrackPush(s.gps);
  updateAllQuads(s);
  updateMobile(s);
}

// ── Status bar ────────────────────────────────────────────────────────────────
function updateStatusBar(s) {
  const mc = MODE_COLORS[s.mode]||C.gray;
  const mEl=el('sb-mode');
  mEl.textContent = s.mode==='AUTO' ? `AUTO·${s.auto_type}` : s.mode;
  mEl.style.color = mEl.style.borderColor = mc;
  el('sb').style.borderColor = mc;

  const volt=s.telemetry.voltage;
  const vc=volt!=null ? (volt<10.5?C.red:volt<11.5?C.yellow:C.green) : C.gray;
  const ve=el('sb-volt'); ve.textContent=fmt(volt,1,' V'); ve.style.color=vc;

  const re=el('sb-rc'); re.textContent=s.rc_active?'RC OK':'RC --';
  re.style.color=s.rc_active?C.green:C.gray;

  el('sb-rec').style.display  = s.cam_recording  ? 'inline' : 'none';
  el('sb-dlog').style.display = s.data_logging   ? 'inline' : 'none';

  const bm=el('btn-mode');
  bm.textContent = s.mode==='AUTO' ? 'SET MANUAL' : 'SET AUTO';
  bm.style.color = bm.style.borderColor = s.mode==='AUTO' ? C.green : C.yellow;

  const br=el('btn-rec');
  if(br){ br.style.color=s.cam_recording?C.red:C.gray;
          br.style.borderColor=s.cam_recording?C.red:C.border; }
  const brs=el('btn-rec-stop');
  if(brs){ brs.style.color=s.cam_recording?C.red:C.gray;
           brs.style.borderColor=s.cam_recording?C.red:C.border; }
  const bd=el('btn-dlog');
  if(bd){ bd.style.color=s.data_logging?C.purple:C.gray;
          bd.style.borderColor=s.data_logging?C.purple:C.border; }

  const bBench=el('btn-bench');
  if(bBench){ bBench.style.color=s.bench_enabled?C.green:C.gray;
              bBench.style.borderColor=s.bench_enabled?C.green:C.border; }

  const bReset=el('btn-reset');
  if(bReset){
    if(s.mode==='ESTOP'){
      bReset.style.display='';
      bReset.style.animation='blink .5s step-end infinite';
    } else {
      bReset.style.display='none';
      bReset.style.animation='';
    }
  }
}

// ── Mode toggle ───────────────────────────────────────────────────────────────
function toggleMode() {
  if (!lastState) return;
  sendCmd(lastState.mode==='AUTO' ? 'set_mode' : 'set_mode',
          lastState.mode==='AUTO' ? {mode:'MANUAL'} : {mode:'AUTO'});
}

// ── Bearing overlay toggle ────────────────────────────────────────────────────
function toggleBearing(quadIdx) {
  showBearing = !showBearing;
  const btn=el(`q${quadIdx}-btn-bearing`);
  if(btn){ btn.style.color=showBearing?C.cyan:C.gray;
           btn.style.borderColor=showBearing?C.cyan:C.border; }
  if(!showBearing) {
    const c=el(`q${quadIdx}-bearing-canvas`);
    if(c){ const ctx=c.getContext('2d'); ctx.clearRect(0,0,c.width,c.height);
           c.style.display='none'; }
  }
  if(lastState) applyState(lastState);
}
function toggleMobBearing() {
  mobShowBearing = !mobShowBearing;
  const b=el('mob-btn-bearing');
  if(b){ b.textContent=mobShowBearing?'Bearing: ON':'Bearing: OFF';
         b.style.color=mobShowBearing?C.cyan:C.gray; }
  if(lastState) applyState(lastState);
}

// ── Canvas drawing ────────────────────────────────────────────────────────────
function drawCompass(canvas, heading) {
  if (!canvas) return;
  const ctx=canvas.getContext('2d'), w=canvas.width, h=canvas.height, cx=w/2, cy=h/2, r=Math.min(w,h)/2-2;
  ctx.clearRect(0,0,w,h);
  ctx.beginPath(); ctx.arc(cx,cy,r,0,Math.PI*2);
  ctx.strokeStyle=C.border; ctx.lineWidth=1; ctx.stroke();
  ctx.strokeStyle=C.red; ctx.lineWidth=2;
  ctx.beginPath(); ctx.moveTo(cx,cy-r+1); ctx.lineTo(cx,cy-r+5); ctx.stroke();
  canvas.style.opacity = heading!=null ? '1' : '0.3';
  if (heading==null) return;
  const rad=(heading-90)*Math.PI/180;
  ctx.strokeStyle=C.cyan; ctx.lineWidth=2;
  ctx.beginPath(); ctx.moveTo(cx,cy);
  ctx.lineTo(cx+r*Math.cos(rad), cy+r*Math.sin(rad)); ctx.stroke();
  ctx.beginPath(); ctx.arc(cx,cy,2,0,Math.PI*2);
  ctx.fillStyle=C.cyan; ctx.fill();
}

function _syncCanvasSize(canvas) {
  const W = canvas.clientWidth, H = canvas.clientHeight;
  if (!W || !H) return false;
  if (canvas.width !== W || canvas.height !== H) { canvas.width = W; canvas.height = H; }
  return true;
}
function drawCompassLarge(canvas, heading) {
  if (!canvas) return;
  if (!_syncCanvasSize(canvas)) return;
  const W = canvas.width, H = canvas.height;
  const ctx = canvas.getContext('2d');
  const cx = W/2, cy = H/2, r = Math.min(W,H)/2 - 4;
  ctx.clearRect(0,0,W,H);

  const avail = heading != null;
  ctx.globalAlpha = avail ? 1 : 0.35;

  // Rotating card — the whole rose turns so the robot's forward direction stays at top
  const cardAngle = avail ? -heading * Math.PI / 180 : 0;
  ctx.save();
  ctx.translate(cx, cy);
  ctx.rotate(cardAngle);

  // Outer ring
  ctx.beginPath(); ctx.arc(0,0,r,0,2*Math.PI);
  ctx.strokeStyle='#444'; ctx.lineWidth=1.5; ctx.stroke();

  // Tick marks
  for (let deg=0; deg<360; deg+=5) {
    const rad = deg*Math.PI/180;
    const inner = deg%30===0 ? r-10 : deg%10===0 ? r-6 : r-4;
    ctx.strokeStyle = deg%90===0 ? '#888' : '#444';
    ctx.lineWidth   = deg%90===0 ? 1.5 : 1;
    ctx.beginPath();
    ctx.moveTo(Math.sin(rad)*r,     -Math.cos(rad)*r);
    ctx.lineTo(Math.sin(rad)*inner, -Math.cos(rad)*inner);
    ctx.stroke();
  }

  // Degree labels every 30°
  ctx.font = `${Math.max(8, Math.round(r*0.14))}px sans-serif`;
  ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
  const cardinals = {0:'N',90:'E',180:'S',270:'W'};
  for (let deg=0; deg<360; deg+=30) {
    const rad = deg*Math.PI/180;
    const lr = r - 18;
    const tx = Math.sin(rad)*lr, ty = -Math.cos(rad)*lr;
    ctx.save(); ctx.translate(tx,ty); ctx.rotate(deg*Math.PI/180);
    if (cardinals[deg]) {
      ctx.fillStyle = deg===0 ? C.red : '#ccc';
      ctx.font = `bold ${Math.max(10, Math.round(r*0.18))}px sans-serif`;
      ctx.fillText(cardinals[deg], 0, 0);
      ctx.font = `${Math.max(8, Math.round(r*0.14))}px sans-serif`;
    } else {
      ctx.fillStyle = '#666';
      ctx.fillText(deg, 0, 0);
    }
    ctx.restore();
  }
  ctx.restore(); // end rotating card

  // Fixed lubber line (forward marker) — top, always stationary
  ctx.strokeStyle = C.cyan; ctx.lineWidth = 2;
  ctx.beginPath(); ctx.moveTo(cx, cy-r+2); ctx.lineTo(cx, cy-r+12); ctx.stroke();

  // Fixed aircraft/robot reference symbol
  if (avail) {
    ctx.strokeStyle = C.cyan; ctx.lineWidth = 2;
    const ws = r*0.45;
    // Wings
    ctx.beginPath(); ctx.moveTo(cx-ws,cy); ctx.lineTo(cx+ws,cy); ctx.stroke();
    // Nose tick
    ctx.beginPath(); ctx.moveTo(cx,cy-ws*0.3); ctx.lineTo(cx,cy+ws*0.3); ctx.stroke();
    // Centre dot
    ctx.beginPath(); ctx.arc(cx,cy,3,0,2*Math.PI);
    ctx.fillStyle=C.cyan; ctx.fill();
  }
  ctx.globalAlpha = 1;
}

function drawArtificialHorizon(canvas, pitch, roll) {
  if (!canvas) return;
  if (!_syncCanvasSize(canvas)) return;
  const W = canvas.width, H = canvas.height;
  const ctx = canvas.getContext('2d');
  const cx = W/2, cy = H/2;
  const r  = Math.min(W,H)/2 - 2;

  ctx.clearRect(0,0,W,H);

  // Clip to a circle
  ctx.save();
  ctx.beginPath(); ctx.arc(cx,cy,r,0,2*Math.PI); ctx.clip();

  if (pitch == null || roll == null) {
    ctx.fillStyle='#222'; ctx.fillRect(0,0,W,H);
    ctx.fillStyle='#555'; ctx.font='11px sans-serif';
    ctx.textAlign='center'; ctx.fillText('No IMU', cx, cy+4);
    ctx.restore(); return;
  }

  const rollRad  = -roll  * Math.PI / 180;  // negate: sensor +roll = right-down → CCW on screen
  // Pitch shift: negate so nose-up moves horizon line down (more sky above reference)
  const pitchPx  = -pitch * (r / 45);

  // Draw tilted sky/ground
  ctx.save();
  ctx.translate(cx, cy + pitchPx);
  ctx.rotate(rollRad);

  // Sky
  ctx.fillStyle = '#1a3a6e';
  ctx.fillRect(-W*1.5, -H*1.5, W*3, H*1.5);
  // Ground
  ctx.fillStyle = '#5c3317';
  ctx.fillRect(-W*1.5, 0, W*3, H*1.5);
  // Horizon line
  ctx.strokeStyle = '#ddd'; ctx.lineWidth = 1.5;
  ctx.beginPath(); ctx.moveTo(-W*1.5, 0); ctx.lineTo(W*1.5, 0); ctx.stroke();

  // Pitch ladder (every 10°, ±40°)
  ctx.strokeStyle = 'rgba(255,255,255,0.5)'; ctx.lineWidth = 1;
  ctx.fillStyle   = 'rgba(255,255,255,0.6)';
  ctx.font = '8px sans-serif'; ctx.textAlign = 'center';
  for (let p = -40; p <= 40; p += 10) {
    if (p === 0) continue;
    const py = -(p * r / 45);
    const hw = p%20===0 ? r*0.35 : r*0.2;
    ctx.beginPath(); ctx.moveTo(-hw, py); ctx.lineTo(hw, py); ctx.stroke();
    ctx.fillText(Math.abs(p), hw+8, py+3);
  }
  ctx.restore(); // end pitch/roll transform

  ctx.restore(); // end clip

  // Outer bezel ring
  ctx.beginPath(); ctx.arc(cx,cy,r,0,2*Math.PI);
  ctx.strokeStyle='#555'; ctx.lineWidth=2; ctx.stroke();

  // Roll arc at top of bezel
  const arcR = r - 3;
  ctx.strokeStyle='#666'; ctx.lineWidth=1;
  ctx.beginPath(); ctx.arc(cx,cy,arcR,-Math.PI*5/6,-Math.PI/6); ctx.stroke();
  // Roll tick marks on bezel: ±10, ±20, ±30, ±45, ±60
  [10,20,30,45,60].forEach(deg => {
    [-deg, deg].forEach(d => {
      const a = (-90 + d) * Math.PI/180;
      const inner = d%30===0 ? arcR-6 : arcR-4;
      ctx.strokeStyle = '#888'; ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(cx+arcR*Math.cos(a), cy+arcR*Math.sin(a));
      ctx.lineTo(cx+inner*Math.cos(a), cy+inner*Math.sin(a));
      ctx.stroke();
    });
  });

  // Roll triangle pointer (fixed at top, points inward along roll angle)
  ctx.save();
  ctx.translate(cx, cy);
  ctx.rotate(rollRad);
  ctx.fillStyle = C.cyan;
  ctx.beginPath();
  ctx.moveTo(0, -(r-2));
  ctx.lineTo(-5, -(r-11));
  ctx.lineTo(5,  -(r-11));
  ctx.closePath(); ctx.fill();
  ctx.restore();

  // Fixed aircraft reference marks
  ctx.strokeStyle = C.cyan; ctx.lineWidth = 2;
  const aw = r * 0.45;
  // Left wing
  ctx.beginPath(); ctx.moveTo(cx-aw, cy); ctx.lineTo(cx-aw*0.4, cy);
  ctx.lineTo(cx-aw*0.4, cy+6); ctx.stroke();
  // Right wing
  ctx.beginPath(); ctx.moveTo(cx+aw, cy); ctx.lineTo(cx+aw*0.4, cy);
  ctx.lineTo(cx+aw*0.4, cy+6); ctx.stroke();
  // Centre dot
  ctx.beginPath(); ctx.arc(cx, cy, 3, 0, 2*Math.PI);
  ctx.fillStyle=C.cyan; ctx.fill();
}

function resizeLidar(canvasId) {
  const c=el(canvasId); if(!c) return;
  const p=c.parentElement; if(!p) return;
  const side=Math.min(p.clientWidth-10, p.clientHeight-10);
  if(side>20){ c.width=p.clientWidth-10; c.height=Math.max(side,80); }
}

function drawLidar(canvas, angles, distances) {
  if (!canvas) return;
  const ctx=canvas.getContext('2d'), W=canvas.width, H=canvas.height;
  ctx.clearRect(0,0,W,H);
  ctx.fillStyle=C.bg; ctx.fillRect(0,0,W,H);
  const cx=W/2, cy=H/2+8, r=Math.min(W,H)/2-24;
  if (!angles||angles.length===0) {
    ctx.fillStyle=C.gray; ctx.font='12px monospace';
    ctx.textAlign='center'; ctx.fillText('No lidar data',cx,cy); return;
  }
  const maxDist=Math.max(...distances)||1;
  for (const frac of [0.25,0.5,0.75,1.0]) {
    ctx.beginPath(); ctx.arc(cx,cy,r*frac,0,Math.PI*2);
    ctx.strokeStyle=C.border; ctx.lineWidth=1; ctx.stroke();
    ctx.fillStyle=C.border; ctx.font='9px monospace'; ctx.textAlign='left';
    ctx.fillText((maxDist*frac/1000).toFixed(1)+'m',cx+r*frac+2,cy-3);
  }
  ctx.strokeStyle=C.border; ctx.lineWidth=1;
  ctx.beginPath(); ctx.moveTo(cx,cy-r); ctx.lineTo(cx,cy+r);
  ctx.moveTo(cx-r,cy); ctx.lineTo(cx+r,cy); ctx.stroke();
  ctx.fillStyle=C.gray; ctx.font='10px monospace'; ctx.textAlign='center';
  ctx.fillText('N',cx,cy-r-3); ctx.fillText('S',cx,cy+r+11);
  ctx.textAlign='left'; ctx.fillText('E',cx+r+2,cy+4);
  ctx.fillText('W',cx-r-12,cy+4);
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

function drawBearingOverlay(canvas, aruco, navGate, heading, showBearingInfo=false) {
  if (!canvas) return;
  // Resize canvas to match its CSS display size
  const W=canvas.offsetWidth||canvas.parentElement.clientWidth;
  const H=canvas.offsetHeight||canvas.parentElement.clientHeight;
  if(canvas.width!==W) canvas.width=W;
  if(canvas.height!==H) canvas.height=H;
  const ctx=canvas.getContext('2d');
  ctx.clearRect(0,0,W,H);
  if (!aruco) return;

  // The <img> uses object-fit:contain — it scales uniformly to fit within the
  // canvas element while preserving aspect ratio, leaving letterbox/pillarbox
  // bars.  Compute where the rendered image actually sits so we can align the
  // canvas overlay exactly on top of it.
  const natW = aruco.cap_w > 0 ? aruco.cap_w : camNatW;
  const natH = aruco.cap_h > 0 ? aruco.cap_h : camNatH;
  const fitScale = Math.min(W / natW, H / natH);
  const rW = natW * fitScale;   // rendered image width in canvas pixels
  const rH = natH * fitScale;   // rendered image height in canvas pixels
  const ox = (W - rW) / 2;     // left offset (pillarbox) in canvas pixels
  const oy = (H - rH) / 2;     // top  offset (letterbox) in canvas pixels

  // ArUco coords are in capture space (natW × natH after rotation).
  // Scale them to the rendered image area inside the canvas.
  const csx = rW / natW;
  const csy = rH / natH;
  // Camera boresight = centre of the rendered image
  const fcx = ox + rW / 2;
  const fcy = oy + rH / 2;

  const targetOdd=navGate*2+1, targetEven=navGate*2+2;
  ctx.font='bold 10px monospace'; ctx.lineWidth=1.5;
  for (const tag of aruco.tags) {
    const tx=ox+tag.cx*csx, ty=oy+tag.cy*csy;
    const isTarget=tag.id===targetOdd||tag.id===targetEven;
    const color=isTarget?C.cyan:C.yellow;
    // Bounding box from corners (always shown when ArUco active)
    if (tag.corners && tag.corners.length===4) {
      ctx.strokeStyle=color; ctx.lineWidth=2; ctx.globalAlpha=0.9;
      ctx.beginPath();
      ctx.moveTo(ox+tag.corners[0][0]*csx, oy+tag.corners[0][1]*csy);
      for (let c=1;c<4;c++) ctx.lineTo(ox+tag.corners[c][0]*csx, oy+tag.corners[c][1]*csy);
      ctx.closePath(); ctx.stroke();
      ctx.globalAlpha=1.0;
      // Filled highlight on top-left corner to show orientation
      ctx.fillStyle=color; ctx.globalAlpha=0.8;
      ctx.fillRect(ox+tag.corners[0][0]*csx-3, oy+tag.corners[0][1]*csy-3, 6, 6);
      ctx.globalAlpha=1.0;
    }
    if (showBearingInfo) {
      // Line from boresight to tag centre
      ctx.strokeStyle=color; ctx.lineWidth=1; ctx.globalAlpha=0.5;
      ctx.beginPath(); ctx.moveTo(fcx,fcy); ctx.lineTo(tx,ty); ctx.stroke();
      ctx.globalAlpha=1.0;
      // Centre dot
      ctx.beginPath(); ctx.arc(tx,ty,3,0,Math.PI*2);
      ctx.fillStyle=color; ctx.fill();
      // Label with bearing/distance
      let lbl=`#${tag.id}`;
      if(tag.bearing!=null) lbl+=` ${tag.bearing>=0?'+':''}${tag.bearing.toFixed(1)}°`;
      if(tag.distance!=null) lbl+=` ${tag.distance.toFixed(2)}m`;
      const lx=tx<fcx+rW/3 ? tx+6 : tx-ctx.measureText(lbl).width-6;
      const ly=ty-8;
      ctx.fillStyle='rgba(18,18,30,.75)';
      ctx.fillRect(lx-2,ly-11,ctx.measureText(lbl).width+4,13);
      ctx.fillStyle=color; ctx.fillText(lbl,lx,ly);
    }
  }
  if (showBearingInfo) {
    for (const gate of aruco.gates) {
      if(gate.gate_id!==navGate) continue;
      const gx=ox+gate.centre_x*csx, gy=oy+gate.centre_y*csy;
      ctx.strokeStyle=C.green; ctx.lineWidth=1.5; ctx.globalAlpha=0.8;
      ctx.beginPath(); ctx.moveTo(fcx,fcy); ctx.lineTo(gx,gy); ctx.stroke();
      ctx.globalAlpha=1.0;
      const sz=10; ctx.strokeStyle=C.green; ctx.lineWidth=2;
      ctx.beginPath();
      ctx.moveTo(gx-sz,gy); ctx.lineTo(gx+sz,gy);
      ctx.moveTo(gx,gy-sz); ctx.lineTo(gx,gy+sz); ctx.stroke();
      ctx.beginPath(); ctx.arc(gx,gy,sz,0,Math.PI*2); ctx.stroke();
      let glbl='AIM';
      if(gate.bearing!=null) glbl+=` ${gate.bearing>=0?'+':''}${gate.bearing.toFixed(1)}°`;
      if(gate.distance!=null) glbl+=` ${gate.distance.toFixed(2)}m`;
      ctx.fillStyle='rgba(18,18,30,.75)';
      ctx.fillRect(gx+2,gy-20,ctx.measureText(glbl).width+4,13);
      ctx.fillStyle=C.green; ctx.fillText(glbl,gx+4,gy-9);
    }
    // Boresight crosshair
    ctx.strokeStyle=C.gray; ctx.lineWidth=1; ctx.globalAlpha=0.5;
    ctx.beginPath();
    ctx.moveTo(fcx-12,fcy); ctx.lineTo(fcx+12,fcy);
    ctx.moveTo(fcx,fcy-12); ctx.lineTo(fcx,fcy+12); ctx.stroke();
    ctx.beginPath(); ctx.arc(fcx,fcy,4,0,Math.PI*2); ctx.stroke();
    ctx.globalAlpha=1.0;
  }
  // IMU arc — pinned to bottom-centre of the rendered image area
  if (showBearingInfo && heading!=null) {
    const ccx=ox+rW/2, ccy=oy+rH-24, cr=15;
    ctx.beginPath(); ctx.arc(ccx,ccy,cr+2,0,Math.PI*2);
    ctx.fillStyle='rgba(18,18,30,.8)'; ctx.fill();
    ctx.beginPath(); ctx.arc(ccx,ccy,cr,0,Math.PI*2);
    ctx.strokeStyle=C.border; ctx.lineWidth=1; ctx.stroke();
    ctx.strokeStyle=C.red; ctx.lineWidth=2;
    ctx.beginPath(); ctx.moveTo(ccx,ccy-cr+2); ctx.lineTo(ccx,ccy-cr+6); ctx.stroke();
    const nrad=(heading-90)*Math.PI/180;
    ctx.strokeStyle=C.cyan; ctx.lineWidth=2;
    ctx.beginPath(); ctx.moveTo(ccx,ccy);
    ctx.lineTo(ccx+cr*Math.cos(nrad),ccy+cr*Math.sin(nrad)); ctx.stroke();
    ctx.font='9px monospace'; ctx.fillStyle=C.cyan; ctx.textAlign='center';
    ctx.fillText(heading.toFixed(0)+'°',ccx,ccy-cr-3); ctx.textAlign='left';
  }
}

// ── Quad interaction ──────────────────────────────────────────────────────────
function onQuadClick(i) {
  const now = Date.now();
  if (now - tapTimes[i] < 400) {
    // Double-tap → expand/collapse
    tapTimes[i] = 0;
    toggleExpand(i);
  } else {
    tapTimes[i] = now;
    // Single tap on header → open chooser (handled by header click)
  }
}

function onQuadHeaderClick(i, e) {
  e.stopPropagation();
  openChooser(i);
}

function toggleExpand(i) {
  if (expandedQuad === i) {
    expandedQuad = -1;
    el(`q${i}`).classList.remove('expanded');
  } else {
    if (expandedQuad >= 0) el(`q${expandedQuad}`).classList.remove('expanded');
    expandedQuad = i;
    el(`q${i}`).classList.add('expanded');
  }
  setTimeout(() => {
    for (let j=0; j<4; j++) {
      const type=quadTypes[j], [kind]=type.includes(':')?type.split(':'):[type];
      if (kind==='lidar') resizeLidar(`q${j}-lidar-canvas`);
    }
  }, 60);
}

// Wire quad events
for (let i=0; i<4; i++) {
  const q=el(`q${i}`);
  const h=el(`q${i}-hdr`);
  q.addEventListener('dblclick', ()=>toggleExpand(i));
  h.addEventListener('click', e=>{ if(e.target.tagName==='BUTTON') return; e.stopPropagation(); openChooser(i); });
  // Escape key to collapse
}
document.addEventListener('keydown', e=>{
  if(e.key==='Escape'){ if(expandedQuad>=0) toggleExpand(expandedQuad); closeChooser(); }
  if(e.key==='[') { const k=_activeCamKey(); sendCmd('rotate_ccw',{cam:k||'front_left'}); }
  if(e.key===']') { const k=_activeCamKey(); sendCmd('rotate_cw', {cam:k||'front_left'}); }
  if(e.key==='t'||e.key==='T') sendCmd('aruco_toggle');
  if(e.key==='e'||e.key==='E') sendCmd('estop');
  if(e.key==='r'||e.key==='R') sendCmd('reset');
  if(e.key==='1') openChooser(0);
  if(e.key==='2') openChooser(1);
  if(e.key==='3') openChooser(2);
  if(e.key==='4') openChooser(3);
  if(e.key==='Enter'&&chooserQuad>=0) closeChooser();
});

// ── Panel chooser ─────────────────────────────────────────────────────────────
function openChooser(i) {
  chooserQuad = i;
  const grid = el('chooser-grid');
  grid.innerHTML = PANEL_DEFS.map(p =>
    `<button class="chooser-btn${quadTypes[i]===p.type?' active':''}"
             onclick="choosePanel(${i},'${p.type}')">${p.label}</button>`
  ).join('');
  el('chooser-title').textContent = `Quad ${i+1} — select panel`;
  el('chooser').classList.add('open');
}
function closeChooser() {
  el('chooser').classList.remove('open');
  chooserQuad = -1;
}
function choosePanel(i, type) {
  quadTypes[i] = type;
  closeChooser();
  renderQuad(i);
  if (lastState) applyState(lastState);
}

// ── Preset buttons ────────────────────────────────────────────────────────────
const PRESET_MAP = {
  front_left:'camera:front_left', front_right:'camera:front_right',
  rear:'camera:rear', lidar:'lidar', telemetry:'telemetry',
  gps_sky:'gps', gps_track:'gps', system:'system', imu:'telemetry',
  depth_map:'system', motor:'motor', logs:'logs',
};
const PRESETS_DEFAULT = {
  Race:  ['camera:front_left','camera:front_right','camera:rear','lidar'],
  Setup: ['camera:front_left','camera:front_right','gps','telemetry'],
  Debug: ['lidar','camera:front_left','telemetry','logs'],
};
let activePreset = '';

function buildPresetButtons(presets) {
  const bar = el('sb-presets');
  bar.innerHTML = '';
  for (const name of Object.keys(presets)) {
    const btn = document.createElement('button');
    btn.className = 'btn-preset';
    btn.textContent = name;
    btn.onclick = () => applyPreset(name, presets);
    bar.appendChild(btn);
  }
}
function applyPreset(name, presets) {
  const layout = presets[name];
  if (!layout) return;
  layout.forEach((t,i) => { quadTypes[i] = PRESET_MAP[t]||t; });
  activePreset = name;
  document.querySelectorAll('.btn-preset').forEach(b=>{
    b.classList.toggle('active', b.textContent===name);
  });
  renderAllQuads();
  if (lastState) applyState(lastState);
}

// ── Commands ──────────────────────────────────────────────────────────────────
function _activeCamKey() {
  // Return the camera key of the first focused (or any) camera quad panel.
  if (expandedQuad >= 0) {
    const t = quadTypes[expandedQuad];
    if (t && t.startsWith('camera:')) return t.split(':')[1];
  }
  for (let i = 0; i < 4; i++) {
    const t = quadTypes[i];
    if (t && t.startsWith('camera:')) return t.split(':')[1];
  }
  return null;
}

async function sendCmd(cmd, extra) {
  try {
    await fetch('/api/cmd', {
      method:'POST',
      headers:{'Content-Type':'application/json'},
      body: JSON.stringify({cmd, ...(extra||{})}),
    });
  } catch(e){ console.error('cmd failed:',e); }
}

// ── Mobile tabs ───────────────────────────────────────────────────────────────
let activeTab = 'drive';
function showTab(name) {
  activeTab = name;
  document.querySelectorAll('.mob-page').forEach(p=>p.classList.remove('active'));
  document.querySelectorAll('.tab-btn').forEach(b=>b.classList.remove('active'));
  el('mob-'+name).classList.add('active');
  el('tab-'+name).classList.add('active');
  if (name==='logs') startLogPolling();
  else if (logPolling) { clearInterval(logPolling); logPolling=null; }
  if (name==='sys')   { setTimeout(()=>resizeLidar('mob-lidar-canvas'),50); }
  if (name==='drive') {
    // Lazily open the mobile camera stream the first time the Drive tab is shown
    const mi = el('mob-cam-img');
    if (mi && !mi.src) mi.src = '/stream/front_left';
  }
}

// ── Log viewer ────────────────────────────────────────────────────────────────
function _logLevel(line) {
  if(/\bCRITICAL\b/.test(line)) return 'critical';
  if(/\bERROR\b/.test(line))    return 'error';
  if(/\bWARNING\b/.test(line))  return 'warning';
  if(/\bDEBUG\b/.test(line))    return 'debug';
  return '';
}
function filterLog(boxId, filterId) {
  const q=(el(filterId)||{}).value||'';
  const box=el(boxId); if(!box) return;
  for (const d of box.children)
    d.style.display=(!q||d.textContent.toLowerCase().includes(q.toLowerCase()))?'':'none';
}
async function fetchLogs() {
  try {
    const data=await (await fetch('/api/logs?n=200')).json();
    const lines=data.lines||[];
    // Update all log boxes (quad panels + mobile) and terminal boxes
    const boxes=[...document.querySelectorAll('.log-box'),
                 ...document.querySelectorAll('.term-box')];
    const isTerminal = b => b.classList.contains('term-box');
    for (const box of boxes) {
      const filterId=box.id.replace(/-(?:log|term)-box$/,'-log-filter');
      const q=(el(filterId)||{}).value||'';
      box.innerHTML='';
      for (const line of lines) {
        const d=document.createElement('div');
        const lvl=_logLevel(line);
        d.className=(isTerminal(box)?'term-line ':'tlog ')+lvl;
        d.textContent=line;
        if(q&&!line.toLowerCase().includes(q.toLowerCase())) d.style.display='none';
        box.appendChild(d);
      }
      if(logAutoScroll) box.scrollTop=box.scrollHeight;
    }
  } catch(e){}
}
function startLogPolling() {
  if (logPolling) return;
  fetchLogs();
  logPolling = setInterval(fetchLogs, 2000);
}

// ── SSE connection ────────────────────────────────────────────────────────────
let evtSrc=null;
function connect() {
  if(evtSrc) evtSrc.close();
  const cb=el('sb-conn');
  cb.textContent='⚫ Connecting…'; cb.style.color=C.yellow;
  evtSrc=new EventSource('/api/state');
  evtSrc.onopen=()=>{ cb.textContent='🟢 Live'; cb.style.color=C.green; };
  evtSrc.onmessage=e=>{
    try { applyState(JSON.parse(e.data)); } catch(err){ console.error('SSE:',err); }
  };
  evtSrc.onerror=()=>{
    cb.textContent='🔴 Reconnecting…'; cb.style.color=C.red;
    evtSrc.close(); setTimeout(connect,3000);
  };
}

// ── Resize ────────────────────────────────────────────────────────────────────
function onResize() {
  for (let i=0; i<4; i++) {
    if (quadTypes[i]==='lidar') resizeLidar(`q${i}-lidar-canvas`);
  }
  resizeLidar('mob-lidar-canvas');
}
window.addEventListener('resize', onResize);

// ── Init ──────────────────────────────────────────────────────────────────────
buildPresetButtons(PRESETS_DEFAULT);
applyPreset('Race', PRESETS_DEFAULT);  // sets quadTypes + renders quads

// Only start log polling if a log panel is actually visible at startup
if (quadTypes.includes('logs')) startLogPolling();

// Lazily set mobile camera src — only when actually in mobile layout
if (window.innerWidth <= 700) {
  const mi = el('mob-cam-img');
  if (mi && !mi.src) mi.src = '/stream/front_left';
}

connect();

// Natural camera frame size
document.addEventListener('load', function(e){
  if(e.target.classList&&e.target.classList.contains('cam-img')){
    if(e.target.naturalWidth)  camNatW=e.target.naturalWidth;
    if(e.target.naturalHeight) camNatH=e.target.naturalHeight;
  }
}, true);

</script>
</body>
</html>"""


# ── Flask app ─────────────────────────────────────────────────────────────────

app       = Flask(__name__)
_robot    = None
_log_path = None


@app.route('/')
def index():
    return Response(_HTML, mimetype='text/html')


@app.route('/stream')
def stream_default():
    return _stream_gen('front_left')


@app.route('/stream/<cam>')
def stream_cam(cam):
    if cam == 'depth_map':
        return stream_depth_map()
    if cam not in ('front_left', 'front_right', 'rear'):
        return 'Not found', 404
    return _stream_gen(cam)


def _stream_gen(cam_name):
    def _gen():
        _robot.add_stream_client(cam_name)
        try:
            while True:
                # Fast path: use pre-encoded JPEG from camera thread (encoded once, shared by all clients)
                data = _robot.get_jpeg(cam_name)
                if data is None:
                    # Cache not yet warm — fall back to get_frame() + encode
                    frame = _robot.get_frame(cam_name)
                    if frame is not None and _jpeg_encode is not None:
                        try:
                            data = _jpeg_encode(frame)
                        except (OSError, ValueError):
                            data = None
                if data:
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + data + b'\r\n')
                    time.sleep(0.1)   # 10 fps cap
                else:
                    time.sleep(0.5)   # camera absent — back off
        finally:
            _robot.remove_stream_client(cam_name)
    return Response(
        _gen(),
        mimetype='multipart/x-mixed-replace; boundary=frame',
        headers={'Cache-Control': 'no-store'},
    )


@app.route('/stream/depth_map')
def stream_depth_map():
    """MJPEG stream of the depth map rendered as a Jet colourmap JPEG (~10 fps)."""
    def _gen():
        while True:
            dm = _robot.get_depth_map()
            if dm.data is not None:
                data = _depth_map_to_jpeg(dm)
                if data:
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + data + b'\r\n')
                    time.sleep(0.1)
                    continue
            time.sleep(0.5)
    return Response(
        _gen(),
        mimetype='multipart/x-mixed-replace; boundary=frame',
        headers={'Cache-Control': 'no-store'},
    )


def _depth_map_to_jpeg(dm) -> Optional[bytes]:
    """Convert a DepthMap to a Jet-colourmap JPEG for streaming."""
    try:
        import cv2 as _cv2
        import numpy as _np
        data = dm.data
        if data is None:
            return None
        # Normalise to 0–255 using a fixed range for metric depth, percentile for relative
        if dm.metric:
            norm = _np.clip(data / 8.0, 0.0, 1.0)   # 0–8 m → 0–1
        else:
            mn, mx = float(data.min()), float(data.max())
            norm = (data - mn) / (mx - mn + 1e-8) if mx > mn else data
        grey  = (norm * 255).astype(_np.uint8)
        coloured = _cv2.applyColorMap(grey, _cv2.COLORMAP_JET)
        # Label source and metric status
        label = f"{dm.source.upper()} {'[m]' if dm.metric else '[rel]'}"
        _cv2.putText(coloured, label, (8, 22),
                     _cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        ok, buf = _cv2.imencode('.jpg', coloured, [_cv2.IMWRITE_JPEG_QUALITY, 75])
        return bytes(buf) if ok else None
    except Exception:
        return None


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
                    aruco_enabled   = _robot.get_aruco_enabled('all'),
                    aruco_states    = {
                        cam: (_robot.get_aruco_state(cam),
                              *_robot.get_cam_capture_size(cam))
                        for cam in ('front_left', 'front_right', 'rear')
                    },
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
    body = request.json or {}
    cmd  = body.get('cmd', '')
    cam  = body.get('cam', 'all')

    if   cmd == 'estop':          _robot.estop()
    elif cmd == 'reset':          _robot.reset_estop()
    elif cmd == 'aruco_toggle':   _robot.toggle_aruco(cam=cam)
    elif cmd == 'gps_bookmark':   _robot.bookmark_gps()
    elif cmd == 'rotate_cw':
        cur = _robot.get_cam_rotation(cam)
        _robot.set_cam_rotation((cur + 90) % 360, cam=cam)
    elif cmd == 'rotate_ccw':
        cur = _robot.get_cam_rotation(cam)
        _robot.set_cam_rotation((cur - 90 + 360) % 360, cam=cam)
    elif cmd == 'record_toggle':
        if _robot.is_cam_recording(cam):
            _robot.stop_cam_recording(cam)
        else:
            _robot.start_cam_recording(cam)
    elif cmd == 'record_start':
        _robot.start_cam_recording(cam)
    elif cmd == 'record_stop':
        _robot.stop_cam_recording(cam)
    elif cmd == 'data_log_toggle':
        if _robot.is_data_logging():
            _robot.stop_data_log()
        else:
            _robot.start_data_log()
    elif cmd == 'bench_toggle':
        _robot.set_bench(not _robot.get_state().bench_enabled)
    elif cmd == 'set_mode':
        mode = body.get('mode', '')
        if   mode == 'MANUAL': _robot.set_mode(RobotMode.MANUAL)
        elif mode == 'AUTO':   _robot.set_mode(RobotMode.AUTO)
        else: return jsonify({'ok': False, 'error': 'Unknown mode'}), 400
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


# ── Config + entry point ──────────────────────────────────────────────────────

def _load_config(path):
    cfg = configparser.ConfigParser(inline_comment_prefixes=('#',))
    cfg.read(path)
    return cfg


def main():
    global _robot, _jpeg_encode, _log_path

    DEFAULT_CFG = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'robot.ini')

    parser = argparse.ArgumentParser(description='HackyRacingRobot unified dashboard')
    parser.add_argument('--config',         default=DEFAULT_CFG)
    parser.add_argument('--host',           default=None)
    parser.add_argument('--port',           default=None, type=int)
    parser.add_argument('--debug',          action='store_true', default=None)
    parser.add_argument('--yukon-port',     default=None)
    parser.add_argument('--ibus-port',      default=None)
    parser.add_argument('--gps-port',       default=None)
    parser.add_argument('--lidar-port',     default=None)
    parser.add_argument('--ntrip-host',     default=None)
    parser.add_argument('--ntrip-port',     default=None, type=int, dest='ntrip_port_arg')
    parser.add_argument('--ntrip-mount',    default=None)
    parser.add_argument('--ntrip-user',     default=None)
    parser.add_argument('--ntrip-password', default=None)
    parser.add_argument('--rtcm-port',      default=None)
    parser.add_argument('--rtcm-baud',      default=None, type=int)
    parser.add_argument('--no-camera',      action='store_true', default=False)
    parser.add_argument('--no-lidar',       action='store_true', default=False)
    parser.add_argument('--no-gps',         action='store_true', default=False)
    parser.add_argument('--no-motors',      action='store_true', default=False,
                        help='Suppress all motor/LED commands (bench test mode)')
    args = parser.parse_args()

    _log_path = setup_logging()
    log.info(f'Log: {_log_path}')
    cfg = _load_config(args.config)

    # Config: [dashboard] section, falls back to [web] for compatibility
    def cfgd(key, fallback, cast=str):
        v = _cfg(cfg, 'dashboard', key, None)
        if v is None:
            v = _cfg(cfg, 'web', key, fallback, cast)
        else:
            v = cast(v) if cast != str else v
        return v

    web_host  = args.host  or cfgd('host',  '0.0.0.0')
    web_port  = args.port  or cfgd('port',  5000, int)
    web_debug = args.debug or cfgd('debug', False, lambda x: x.lower()=='true')

    _jpeg_encode = _make_jpeg_encoder()

    bool_val = lambda x: x.lower() == 'true'
    port_val = lambda x: None if x.lower() in ('auto', '') else x

    def arg(cli_val, section, key, fallback, cast=str):
        if cli_val is not None:
            return cli_val
        return _cfg(cfg, section, key, fallback, cast)

    _robot = Robot(
        yukon_port     = arg(args.yukon_port,     'robot', 'yukon_port',     None, port_val),
        ibus_port      = arg(args.ibus_port,       'robot', 'ibus_port',      '/dev/ttyAMA3'),
        lidar_port     = arg(args.lidar_port,      'lidar', 'port',           '/dev/ttyUSB0'),
        gps_port       = arg(args.gps_port,        'gps',   'port',           '/dev/ttyUSB0'),
        ntrip_host     = '' if _cfg(cfg,'ntrip','disabled',False,bool_val)
                         else arg(args.ntrip_host, 'ntrip', 'host',           ''),
        ntrip_port     = arg(args.ntrip_port_arg,  'ntrip', 'port',           2101, int),
        ntrip_mount    = arg(args.ntrip_mount,      'ntrip', 'mount',          ''),
        ntrip_user     = arg(args.ntrip_user,       'ntrip', 'user',           ''),
        ntrip_password = arg(args.ntrip_password,   'ntrip', 'password',       ''),
        rtcm_port      = '' if _cfg(cfg,'rtcm','disabled',False,bool_val)
                         else arg(args.rtcm_port,  'rtcm',  'port',           ''),
        rtcm_baud      = arg(args.rtcm_baud,        'rtcm',  'baud',           115200, int),
        enable_camera  = not (args.no_camera or _cfg(cfg,'camera','disabled',False,bool_val)),
        enable_lidar   = not (args.no_lidar  or _cfg(cfg,'lidar', 'disabled',False,bool_val)),
        enable_gps     = not (args.no_gps    or _cfg(cfg,'gps',   'disabled',False,bool_val)),
        cam_width      = _cfg(cfg, 'camera', 'width',       640,   int),
        cam_height     = _cfg(cfg, 'camera', 'height',      480,   int),
        cam_fps        = _cfg(cfg, 'camera', 'fps',         30,    int),
        cam_rotation   = _cfg(cfg, 'camera', 'rotation',    0,     int),
        enable_aruco   = _cfg(cfg, 'aruco',  'enabled',     False, bool_val),
        aruco_dict     = _cfg(cfg, 'aruco',  'dict',        'DICT_4X4_1000'),
        aruco_calib    = _cfg(cfg, 'aruco',  'calib_file',  ''),
        aruco_tag_size = _cfg(cfg, 'aruco',  'tag_size',    0.15,  float),
        throttle_ch    = _cfg(cfg, 'rc',     'throttle_ch', 3,     int),
        steer_ch       = _cfg(cfg, 'rc',     'steer_ch',    1,     int),
        mode_ch        = _cfg(cfg, 'rc',     'mode_ch',     5,     int),
        speed_ch       = _cfg(cfg, 'rc',     'speed_ch',    6,     int),
        auto_type_ch   = _cfg(cfg, 'rc',     'auto_type_ch',7,     int),
        gps_log_ch     = _cfg(cfg, 'rc',     'gps_log_ch',  8,     int),
        gps_bookmark_ch= _cfg(cfg, 'rc',     'gps_bookmark_ch',2,  int),
        gps_log_dir    = _cfg(cfg, 'gps',    'log_dir',     ''),
        gps_log_hz     = _cfg(cfg, 'gps',    'log_hz',      5.0,   float),
        deadzone       = _cfg(cfg, 'rc',     'deadzone',    30,    int),
        failsafe_s     = _cfg(cfg, 'rc',     'failsafe_s',  0.5,   float),
        speed_min      = _cfg(cfg, 'rc',     'speed_min',   0.25,  float),
        control_hz     = _cfg(cfg, 'rc',     'control_hz',  50,    int),
        no_motors      = args.no_motors,
        rec_dir               = _cfg(cfg, 'output', 'videos_dir',           ''),
        max_recording_minutes = _cfg(cfg, 'output', 'max_recording_minutes', 0.0, float),
        data_log_dir          = _cfg(cfg, 'output', 'data_log_dir',          ''),
    )
    _robot.start()
    log.info(f'Dashboard → http://{_local_ip()}:{web_port}/')

    try:
        app.run(host=web_host, port=web_port,
                debug=web_debug, threaded=True, use_reloader=False)
    finally:
        _robot.stop()


if __name__ == '__main__':
    main()
