#!/usr/bin/env python3
"""
gps_route_builder_web.py — Web-based GPS route builder for HackyRacingRobot.

Serves a browser-based map editor on port 5003.  Works on desktop and mobile.
All map interaction uses Leaflet.js with OSM tiles — no extra Python deps.

Features
--------
  Map          : OpenStreetMap via Leaflet (handles tiles, zoom, pan natively)
  Add          : click empty map → add waypoint
  Move         : drag marker to reposition
  Delete       : click waypoint → delete button in popup
  Reorder      : drag rows in sidebar list
  Labels       : click label in sidebar to edit inline
  Import JSON  : upload waypoints.json
  Import CSV   : upload GPS log CSV from robot.py
  Export JSON  : download waypoints.json
  Live mode    : SSE stream from robot.py → live robot position + accuracy ring,
                 GPS track overlay, nav state badge
  Send route   : push current waypoints to GPS navigator

Usage
-----
  python3 gps_route_builder_web.py              # 0.0.0.0:5003
  python3 gps_route_builder_web.py --port 8080
  python3 gps_route_builder_web.py --live       # auto-connect to robot.py

Dependencies
------------
  pip install flask
  (Leaflet.js loaded from CDN — works offline if tiles already cached by browser)
"""

import argparse
import csv
import io
import json
import logging
import math
import os
import sys
import threading
import time
from pathlib import Path

from flask import Flask, Response, jsonify, request

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

log = logging.getLogger(__name__)

# ── Shared state ──────────────────────────────────────────────────────────────

_lock      = threading.Lock()
_waypoints = []     # list of {"lat", "lon", "label"}
_robot     = None   # robot.py Robot instance when connected
_track     = []     # [(lat, lon), ...] recorded GPS track
_live_state= None   # latest robot state snapshot

# ── Haversine ─────────────────────────────────────────────────────────────────

def _haversine(lat1, lon1, lat2, lon2):
    R  = 6_371_000
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a  = math.sin(dp/2)**2 + math.cos(p1)*math.cos(p2)*math.sin(dl/2)**2
    return 2 * R * math.asin(math.sqrt(a))


def _total_distance(wps):
    if len(wps) < 2:
        return 0
    return sum(_haversine(wps[i]['lat'], wps[i]['lon'],
                           wps[i+1]['lat'], wps[i+1]['lon'])
               for i in range(len(wps) - 1))


def _fmt_dist(m):
    return f"{m/1000:.2f} km" if m >= 1000 else f"{m:.0f} m"

# ── CSV importer ──────────────────────────────────────────────────────────────

def _import_csv(text):
    wps  = []
    seen = set()
    reader = csv.DictReader(io.StringIO(text))
    for row in reader:
        try:
            lat = float(row.get('latitude') or row.get('lat') or 0)
            lon = float(row.get('longitude') or row.get('lon') or 0)
        except (ValueError, TypeError):
            continue
        if lat == 0 and lon == 0:
            continue
        key = (round(lat, 5), round(lon, 5))
        if key in seen:
            continue
        seen.add(key)
        bm  = str(row.get('bookmark', '')).lower() == 'true'
        wps.append({"lat": lat, "lon": lon, "label": "Bookmark" if bm else ""})
    return wps

# ── Robot live polling ────────────────────────────────────────────────────────

def _start_robot_poll():
    def _poll():
        global _live_state, _track
        while True:
            if _robot is None:
                time.sleep(0.5)
                continue
            try:
                state = _robot.get_state()
                g     = state.gps
                snap  = {
                    'mode':         state.mode.name,
                    'auto_type':    state.auto_type.label,
                    'nav_state':    state.nav_state,
                    'nav_wp':       state.nav_wp,
                    'nav_wp_dist':  state.nav_wp_dist,
                    'nav_wp_bear':  state.nav_wp_bear,
                    'gps': {
                        'latitude':         g.latitude,
                        'longitude':        g.longitude,
                        'fix_quality':      g.fix_quality,
                        'fix_quality_name': g.fix_quality_name,
                        'h_error_m':        g.h_error_m,
                        'heading':          g.heading,
                        'speed':            g.speed,
                    } if g else None,
                }
                with _lock:
                    _live_state = snap
                    if g and g.latitude is not None:
                        _track.append([g.latitude, g.longitude])
                        if len(_track) > 5000:
                            _track = _track[-5000:]
            except Exception as e:
                log.debug("Robot poll error: %s", e)
            time.sleep(0.1)
    t = threading.Thread(target=_poll, daemon=True)
    t.start()

# ── Flask app ─────────────────────────────────────────────────────────────────

app = Flask(__name__)


@app.route('/')
def index():
    return Response(_HTML, mimetype='text/html')


@app.route('/api/waypoints', methods=['GET'])
def get_waypoints():
    with _lock:
        wps = list(_waypoints)
    dist = _total_distance(wps)
    return jsonify({'waypoints': wps, 'total_distance': dist,
                    'total_distance_fmt': _fmt_dist(dist) if dist else ''})


@app.route('/api/waypoints', methods=['POST'])
def set_waypoints():
    data = request.json or {}
    wps  = data.get('waypoints', [])
    with _lock:
        _waypoints.clear()
        _waypoints.extend(wps)
    return jsonify({'ok': True, 'count': len(wps)})


@app.route('/api/waypoints/add', methods=['POST'])
def add_waypoint():
    d = request.json or {}
    wp = {'lat': float(d['lat']), 'lon': float(d['lon']),
          'label': d.get('label', '')}
    with _lock:
        idx = d.get('index')
        if idx is None:
            _waypoints.append(wp)
            idx = len(_waypoints) - 1
        else:
            _waypoints.insert(int(idx), wp)
    return jsonify({'ok': True, 'index': idx, 'waypoint': wp})


@app.route('/api/waypoints/<int:idx>', methods=['PUT'])
def update_waypoint(idx):
    d = request.json or {}
    with _lock:
        if 0 <= idx < len(_waypoints):
            if 'lat' in d: _waypoints[idx]['lat']   = float(d['lat'])
            if 'lon' in d: _waypoints[idx]['lon']   = float(d['lon'])
            if 'label' in d: _waypoints[idx]['label'] = str(d['label'])
            return jsonify({'ok': True, 'waypoint': _waypoints[idx]})
    return jsonify({'ok': False, 'error': 'Index out of range'}), 404


@app.route('/api/waypoints/<int:idx>', methods=['DELETE'])
def delete_waypoint(idx):
    with _lock:
        if 0 <= idx < len(_waypoints):
            _waypoints.pop(idx)
            return jsonify({'ok': True})
    return jsonify({'ok': False}), 404


@app.route('/api/waypoints/reorder', methods=['POST'])
def reorder_waypoints():
    d    = request.json or {}
    from_idx = int(d.get('from', 0))
    to_idx   = int(d.get('to',   0))
    with _lock:
        if 0 <= from_idx < len(_waypoints) and 0 <= to_idx < len(_waypoints):
            wp = _waypoints.pop(from_idx)
            _waypoints.insert(to_idx, wp)
    return jsonify({'ok': True})


@app.route('/api/waypoints/export')
def export_waypoints():
    with _lock:
        data = json.dumps(_waypoints, indent=2)
    return Response(data, mimetype='application/json',
                    headers={'Content-Disposition':
                             'attachment; filename="waypoints.json"'})


@app.route('/api/waypoints/import', methods=['POST'])
def import_waypoints():
    if 'file' not in request.files:
        return jsonify({'ok': False, 'error': 'No file'}), 400
    f    = request.files['file']
    text = f.read().decode('utf-8', errors='replace')
    try:
        if f.filename.endswith('.csv'):
            wps = _import_csv(text)
        else:
            wps = json.loads(text)
            if isinstance(wps, list):
                wps = [{'lat': float(w['lat']), 'lon': float(w['lon']),
                        'label': str(w.get('label', ''))} for w in wps]
            else:
                raise ValueError("Expected JSON array")
        with _lock:
            _waypoints.clear()
            _waypoints.extend(wps)
        return jsonify({'ok': True, 'count': len(wps), 'waypoints': wps})
    except Exception as e:
        return jsonify({'ok': False, 'error': str(e)}), 400


@app.route('/api/live')
def live_state():
    """SSE stream of robot state at 5 Hz."""
    def _gen():
        while True:
            with _lock:
                state = _live_state
                track = list(_track[-200:])   # last 200 points to keep payload small
            payload = {
                'connected': _robot is not None,
                'state':     state,
                'track':     track,
            }
            yield f"data: {json.dumps(payload)}\n\n"
            time.sleep(0.2)
    return Response(_gen(), mimetype='text/event-stream',
                    headers={'Cache-Control': 'no-cache',
                             'X-Accel-Buffering': 'no'})


@app.route('/api/robot/connect', methods=['POST'])
def robot_connect():
    global _robot
    try:
        from robot_daemon import Robot
        # Try to find an existing instance — for now just import
        # In practice user would pass robot.ini path
        _robot = Robot()
        _robot.start()
        _start_robot_poll()
        return jsonify({'ok': True, 'message': 'Connected to robot.py'})
    except Exception as e:
        return jsonify({'ok': False, 'error': str(e)}), 500


@app.route('/api/robot/disconnect', methods=['POST'])
def robot_disconnect():
    global _robot
    if _robot:
        try: _robot.stop()
        except Exception: pass
        _robot = None
    return jsonify({'ok': True})


@app.route('/api/robot/send_route', methods=['POST'])
def send_route():
    if _robot is None:
        return jsonify({'ok': False, 'error': 'Not connected'}), 400
    with _lock:
        wps = list(_waypoints)
    if not wps:
        return jsonify({'ok': False, 'error': 'No waypoints'}), 400
    try:
        nav = _robot.get_gps_navigator()
        if nav is None:
            return jsonify({'ok': False, 'error': 'GPS navigator not available'}), 500
        nav.load_waypoints(wps)
        return jsonify({'ok': True, 'count': len(wps)})
    except Exception as e:
        return jsonify({'ok': False, 'error': str(e)}), 500


@app.route('/api/robot/clear_track', methods=['POST'])
def clear_track():
    global _track
    with _lock:
        _track = []
    return jsonify({'ok': True})


# ── Embedded HTML/CSS/JS ──────────────────────────────────────────────────────

_HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>HackyRacingRobot GPS Route Builder</title>
<link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.9.4/leaflet.min.css"/>
<script src="https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.9.4/leaflet.min.js"></script>
<style>
@import url('https://fonts.googleapis.com/css2?family=JetBrains+Mono:wght@400;600;700&display=swap');

*,*::before,*::after{box-sizing:border-box;margin:0;padding:0}
:root{
  --bg:#0d0d18;--panel:#161624;--card:#1e1e30;--border:#2e2e48;
  --white:#e8eaf6;--gray:#6b7080;--lgray:#9ea3b8;
  --green:#32e05a;--yellow:#f5c842;--orange:#f08030;
  --red:#e04040;--cyan:#28c8e0;--blue:#4880e8;--purple:#a040e0;
  --route:#28c8e0;--track:#40d870;
  --sidebar:280px;
}
html,body{height:100%;font-family:'JetBrains Mono',monospace;
  background:var(--bg);color:var(--white);overflow:hidden}
body{display:flex;flex-direction:column}

/* ── Header ── */
#header{
  display:flex;align-items:center;gap:12px;
  padding:8px 14px;background:var(--panel);
  border-bottom:1px solid var(--border);flex-shrink:0;
  height:48px;
}
#header h1{font-size:.95rem;font-weight:700;color:var(--cyan);
  letter-spacing:2px;white-space:nowrap}
#header-info{font-size:.7rem;color:var(--gray);flex:1}
#conn-badge{font-size:.7rem;margin-left:auto}

/* ── Main layout ── */
#main{display:flex;flex:1;min-height:0}

/* ── Map ── */
#map{flex:1;min-width:0;cursor:crosshair}
#map.dragging{cursor:grabbing}

/* ── Sidebar ── */
#sidebar{
  width:var(--sidebar);flex-shrink:0;
  background:var(--panel);border-left:1px solid var(--border);
  display:flex;flex-direction:column;overflow:hidden;
}
#sidebar-header{padding:10px 12px 6px;border-bottom:1px solid var(--border);flex-shrink:0}
#sidebar-title{font-size:.75rem;color:var(--cyan);font-weight:700;
  letter-spacing:1px;margin-bottom:4px}
#route-stats{font-size:.65rem;color:var(--lgray)}

/* ── Waypoint list ── */
#wp-list{flex:1;overflow-y:auto;padding:6px 8px}
#wp-list::-webkit-scrollbar{width:4px}
#wp-list::-webkit-scrollbar-track{background:transparent}
#wp-list::-webkit-scrollbar-thumb{background:var(--border);border-radius:2px}

.wp-item{
  display:flex;align-items:center;gap:8px;
  background:var(--card);border:1px solid var(--border);
  border-radius:7px;padding:7px 8px;margin-bottom:5px;
  cursor:pointer;transition:border-color .15s,background .15s;
  user-select:none;
}
.wp-item:hover{border-color:var(--cyan);background:#1e2035}
.wp-item.selected{border-color:var(--yellow);background:#22201a}
.wp-item.dragging-over{border-color:var(--green);border-style:dashed}
.wp-item.being-dragged{opacity:.4}

.wp-num{
  width:22px;height:22px;border-radius:50%;
  background:var(--card);border:2px solid var(--cyan);
  font-size:.65rem;font-weight:700;color:var(--cyan);
  display:flex;align-items:center;justify-content:center;
  flex-shrink:0;
}
.wp-item.selected .wp-num{border-color:var(--yellow);color:var(--yellow)}

.wp-body{flex:1;min-width:0}
.wp-label{font-size:.75rem;color:var(--white);white-space:nowrap;
  overflow:hidden;text-overflow:ellipsis}
.wp-label input{
  background:transparent;border:none;border-bottom:1px solid var(--cyan);
  color:var(--yellow);font-family:inherit;font-size:.75rem;
  width:100%;outline:none;padding:0;
}
.wp-coords{font-size:.6rem;color:var(--gray);margin-top:1px}
.wp-dist{font-size:.6rem;color:var(--route);margin-top:1px}

.wp-handle{color:var(--border);cursor:grab;font-size:.9rem;flex-shrink:0;
  padding:0 2px;line-height:1}
.wp-handle:active{cursor:grabbing}
.wp-del{
  background:none;border:none;color:var(--gray);cursor:pointer;
  font-size:.8rem;padding:2px 4px;border-radius:3px;flex-shrink:0;
  transition:color .1s;
}
.wp-del:hover{color:var(--red)}

/* ── Active waypoint badge on map ── */
.wp-active-ring .wp-num{border-color:var(--green)!important;color:var(--green)!important}

/* ── Sidebar buttons ── */
#sidebar-buttons{padding:8px;border-top:1px solid var(--border);flex-shrink:0}
.btn-grid{display:grid;grid-template-columns:1fr 1fr;gap:5px;margin-bottom:5px}
.btn{
  font-family:inherit;font-size:.68rem;padding:7px 4px;
  border-radius:6px;border:1px solid var(--border);
  background:var(--card);color:var(--lgray);cursor:pointer;
  text-align:center;transition:border-color .1s,color .1s;
  white-space:nowrap;
}
.btn:hover{border-color:var(--cyan);color:var(--white)}
.btn-cyan{border-color:var(--cyan);color:var(--cyan)}
.btn-cyan:hover{background:#0e2530}
.btn-green{border-color:var(--green);color:var(--green)}
.btn-green:hover{background:#0a2010}
.btn-yellow{border-color:var(--yellow);color:var(--yellow)}
.btn-yellow:hover{background:#251f08}
.btn-red{border-color:var(--red);color:var(--red)}
.btn-red:hover{background:#200808}
.btn-orange{border-color:var(--orange);color:var(--orange)}
.btn-orange:hover{background:#1e1008}
.btn-full{grid-column:1/-1}
#file-input{display:none}

/* ── Live status bar ── */
#live-bar{
  padding:5px 12px;border-top:1px solid var(--border);
  flex-shrink:0;font-size:.65rem;color:var(--gray);
  display:flex;gap:12px;flex-wrap:wrap;align-items:center;
}
.live-badge{
  padding:2px 7px;border-radius:10px;border:1px solid var(--border);
  white-space:nowrap;
}
.live-badge.ok{border-color:var(--green);color:var(--green)}
.live-badge.warn{border-color:var(--yellow);color:var(--yellow)}
.live-badge.info{border-color:var(--cyan);color:var(--cyan)}
.live-badge.err{border-color:var(--red);color:var(--red)}

/* ── Toast ── */
#toast{
  position:fixed;bottom:20px;left:50%;transform:translateX(-50%);
  background:var(--card);border:1px solid var(--cyan);border-radius:8px;
  padding:8px 20px;font-size:.8rem;color:var(--white);
  pointer-events:none;opacity:0;transition:opacity .25s;z-index:9999;
  white-space:nowrap;
}
#toast.show{opacity:1}

/* ── Map overlays ── */
.leaflet-container{background:#0d0d18}
.wp-icon{
  width:24px;height:24px;border-radius:50%;
  background:#1e1e30;border:2px solid #28c8e0;
  display:flex;align-items:center;justify-content:center;
  font-family:'JetBrains Mono',monospace;font-size:10px;
  font-weight:700;color:#28c8e0;
  box-shadow:0 2px 6px rgba(0,0,0,.5);
  cursor:pointer;
}
.wp-icon.selected{border-color:#f5c842;color:#f5c842}
.wp-icon.active{border-color:#32e05a;color:#32e05a}

/* ── Mobile ── */
@media(max-width:600px){
  :root{--sidebar:100vw}
  #main{flex-direction:column}
  #map{height:55vh}
  #sidebar{width:100%;border-left:none;border-top:1px solid var(--border)}
}
</style>
</head>
<body>

<div id="header">
  <h1>⬡ HACKYRACINGROBOT ROUTE BUILDER</h1>
  <span id="header-info"></span>
  <span id="conn-badge" class="live-badge">⚫ offline</span>
</div>

<div id="main">
  <div id="map"></div>

  <div id="sidebar">
    <div id="sidebar-header">
      <div id="sidebar-title">WAYPOINTS</div>
      <div id="route-stats">0 waypoints</div>
    </div>

    <div id="wp-list"></div>

    <div id="sidebar-buttons">
      <div class="btn-grid">
        <button class="btn btn-cyan"   onclick="fitAll()">Fit Map</button>
        <button class="btn btn-cyan"   onclick="clearAll()">Clear All</button>
        <button class="btn btn-green"  onclick="exportJSON()">Export JSON</button>
        <button class="btn btn-cyan"   onclick="document.getElementById('file-input').click()">Import</button>
        <button class="btn btn-yellow btn-full" onclick="sendRoute()">▶ Send to Robot</button>
        <button class="btn btn-orange btn-full" id="connect-btn" onclick="toggleConnect()">Connect Robot</button>
        <button class="btn btn-red"    onclick="clearTrack()">Clear Track</button>
        <button class="btn"            id="centre-robot-btn" onclick="centreRobot()" disabled>→ Robot</button>
      </div>
      <input type="file" id="file-input" accept=".json,.csv" onchange="importFile(this)">
    </div>

    <div id="live-bar">
      <span id="lb-gps"  class="live-badge">GPS: --</span>
      <span id="lb-mode" class="live-badge">Mode: --</span>
      <span id="lb-nav"  class="live-badge" style="display:none">NAV: --</span>
      <span id="lb-pos"  style="color:var(--gray);font-size:.6rem"></span>
    </div>
  </div>
</div>

<div id="toast"></div>

<script>
"use strict";

// ── Leaflet map ───────────────────────────────────────────────────────────────
const map = L.map('map', {
  center: [52.2053, 0.1218],
  zoom:   17,
  zoomControl: true,
});

L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png', {
  attribution: '© <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a>',
  maxZoom: 19,
}).addTo(map);

// Dark filter on tiles
const style = document.createElement('style');
style.textContent = '.leaflet-tile{filter:brightness(.85) saturate(.9) hue-rotate(180deg) invert(1) sepia(.3)}';
document.head.appendChild(style);

// ── State ─────────────────────────────────────────────────────────────────────
let waypoints    = [];   // [{lat,lon,label}, ...]
let markers      = [];   // L.Marker for each waypoint
let routeLine    = null;
let selectedIdx  = -1;
let robotMarker  = null;
let accuracyCircle = null;
let trackLine    = null;
let trackPoints  = [];
let liveConnected= false;
let robotPos     = null;

// drag-reorder state
let dragSrc = -1;

// ── Haversine ─────────────────────────────────────────────────────────────────
function haversine(lat1, lon1, lat2, lon2) {
  const R = 6371000;
  const p1 = lat1*Math.PI/180, p2 = lat2*Math.PI/180;
  const dp = (lat2-lat1)*Math.PI/180, dl = (lon2-lon1)*Math.PI/180;
  const a  = Math.sin(dp/2)**2 + Math.cos(p1)*Math.cos(p2)*Math.sin(dl/2)**2;
  return 2*R*Math.asin(Math.sqrt(a));
}
function fmtDist(m) { return m>=1000 ? (m/1000).toFixed(2)+' km' : m.toFixed(0)+' m'; }
function totalDist(wps) {
  if (wps.length<2) return 0;
  let t=0; for(let i=0;i<wps.length-1;i++) t+=haversine(wps[i].lat,wps[i].lon,wps[i+1].lat,wps[i+1].lon);
  return t;
}

// ── Waypoint icons ────────────────────────────────────────────────────────────
function makeIcon(num, sel=false, active=false) {
  const cls = 'wp-icon'+(sel?' selected':'')+(active?' active':'');
  return L.divIcon({
    className:'',
    html:`<div class="${cls}">${num}</div>`,
    iconSize:[24,24], iconAnchor:[12,12], popupAnchor:[0,-14],
  });
}

// ── Add waypoint ──────────────────────────────────────────────────────────────
function addWaypoint(lat, lon, label='', refreshList=true) {
  const wp = {lat, lon, label: label || `WP ${waypoints.length+1}`};
  waypoints.push(wp);
  const idx = waypoints.length - 1;
  _addMarker(idx);
  if (refreshList) {
    renderList();
    syncToServer();
  }
  return idx;
}

function _addMarker(idx) {
  const wp = waypoints[idx];
  const m  = L.marker([wp.lat, wp.lon], {
    icon:      makeIcon(idx+1, idx===selectedIdx),
    draggable: true,
  }).addTo(map);

  m.on('click', e => {
    L.DomEvent.stopPropagation(e);
    selectWaypoint(idx);
  });
  m.on('dragend', e => {
    const pos = e.target.getLatLng();
    waypoints[idx].lat = pos.lat;
    waypoints[idx].lon = pos.lng;
    updateRoute();
    renderList();
    syncToServer();
  });
  m.on('contextmenu', e => {
    L.DomEvent.stopPropagation(e);
    deleteWaypoint(idx);
  });
  markers[idx] = m;
}

function selectWaypoint(idx) {
  selectedIdx = idx;
  refreshMarkerIcons();
  renderList();
  // Scroll sidebar to selected
  const items = document.querySelectorAll('.wp-item');
  if (items[idx]) items[idx].scrollIntoView({block:'nearest'});
}

function deleteWaypoint(idx) {
  map.removeLayer(markers[idx]);
  markers.splice(idx, 1);
  waypoints.splice(idx, 1);
  if (selectedIdx >= waypoints.length) selectedIdx = waypoints.length-1;
  // Re-add all markers with correct indices
  markers.forEach(m => map.removeLayer(m));
  markers = [];
  waypoints.forEach((_, i) => _addMarker(i));
  updateRoute();
  renderList();
  syncToServer();
}

function clearAll() {
  markers.forEach(m => map.removeLayer(m));
  markers = []; waypoints = []; selectedIdx = -1;
  updateRoute(); renderList(); syncToServer();
  toast('Cleared');
}

// ── Route line ────────────────────────────────────────────────────────────────
function updateRoute() {
  if (routeLine) { map.removeLayer(routeLine); routeLine=null; }
  if (waypoints.length < 2) return;
  const pts = waypoints.map(w=>[w.lat,w.lon]);
  routeLine = L.polyline(pts, {
    color:'#28c8e0', weight:2, opacity:.8, dashArray:null,
  }).addTo(map);
}

function refreshMarkerIcons(activeWp=-1) {
  markers.forEach((m,i) => {
    if (m) m.setIcon(makeIcon(i+1, i===selectedIdx, i===activeWp));
  });
}

// ── Sidebar list ──────────────────────────────────────────────────────────────
function renderList() {
  const list = document.getElementById('wp-list');
  const dist = totalDist(waypoints);
  document.getElementById('route-stats').textContent =
    `${waypoints.length} waypoint${waypoints.length!==1?'s':''}` +
    (dist>0 ? `  ·  ${fmtDist(dist)}` : '');

  list.innerHTML = '';
  waypoints.forEach((wp, i) => {
    const item = document.createElement('div');
    item.className = 'wp-item' + (i===selectedIdx ? ' selected' : '');
    item.draggable = true;
    item.dataset.idx = i;

    // Distance from previous
    let distStr = '';
    if (i>0) {
      const d = haversine(waypoints[i-1].lat,waypoints[i-1].lon,wp.lat,wp.lon);
      distStr = `+${fmtDist(d)}`;
    }

    item.innerHTML = `
      <div class="wp-num">${i+1}</div>
      <div class="wp-body">
        <div class="wp-label" data-idx="${i}" ondblclick="startEdit(${i})">${escHtml(wp.label||'WP '+(i+1))}</div>
        <div class="wp-coords">${wp.lat.toFixed(6)}, ${wp.lon.toFixed(6)}</div>
        ${distStr ? `<div class="wp-dist">${distStr}</div>` : ''}
      </div>
      <span class="wp-handle" title="Drag to reorder">⠿</span>
      <button class="wp-del" onclick="deleteWaypoint(${i})" title="Delete">✕</button>`;

    // Select on click
    item.addEventListener('click', e => {
      if (e.target.classList.contains('wp-del')) return;
      if (e.target.tagName === 'INPUT') return;
      selectWaypoint(i);
      if (markers[i]) map.panTo(markers[i].getLatLng());
    });

    // Drag reorder
    item.addEventListener('dragstart', e => {
      dragSrc = i;
      item.classList.add('being-dragged');
      e.dataTransfer.effectAllowed = 'move';
    });
    item.addEventListener('dragend', () => {
      item.classList.remove('being-dragged');
      document.querySelectorAll('.wp-item').forEach(el=>el.classList.remove('dragging-over'));
    });
    item.addEventListener('dragover', e => {
      e.preventDefault(); e.dataTransfer.dropEffect='move';
      item.classList.add('dragging-over');
    });
    item.addEventListener('dragleave', () => item.classList.remove('dragging-over'));
    item.addEventListener('drop', e => {
      e.preventDefault();
      item.classList.remove('dragging-over');
      if (dragSrc < 0 || dragSrc === i) return;
      reorderWaypoints(dragSrc, i);
    });

    list.appendChild(item);
  });
}

function reorderWaypoints(from, to) {
  const wp = waypoints.splice(from, 1)[0];
  waypoints.splice(to, 0, wp);
  selectedIdx = to;
  // Rebuild markers
  markers.forEach(m => map.removeLayer(m));
  markers = [];
  waypoints.forEach((_,i)=>_addMarker(i));
  updateRoute(); renderList(); syncToServer();
}

function escHtml(s) {
  return s.replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;')
           .replace(/"/g,'&quot;');
}

// ── Label editing ─────────────────────────────────────────────────────────────
function startEdit(idx) {
  const labelEl = document.querySelector(`.wp-label[data-idx="${idx}"]`);
  if (!labelEl) return;
  const cur = waypoints[idx].label || '';
  labelEl.innerHTML = `<input type="text" value="${escHtml(cur)}" autofocus
    onblur="commitEdit(${idx},this.value)"
    onkeydown="if(event.key==='Enter')this.blur();if(event.key==='Escape'){this.value='${escHtml(cur)}';this.blur();}">`;
  labelEl.querySelector('input').select();
}
function commitEdit(idx, value) {
  waypoints[idx].label = value.trim() || `WP ${idx+1}`;
  renderList(); syncToServer();
}

// ── Map click → add waypoint ──────────────────────────────────────────────────
map.on('click', e => {
  addWaypoint(e.latlng.lat, e.latlng.lng);
  toast(`WP ${waypoints.length} added`);
});

// ── Fit / centre ──────────────────────────────────────────────────────────────
function fitAll() {
  if (!waypoints.length) return;
  const bounds = L.latLngBounds(waypoints.map(w=>[w.lat,w.lon]));
  map.fitBounds(bounds, {padding:[40,40]});
}
function centreRobot() {
  if (robotPos) map.panTo(robotPos);
}

// ── Import / Export ───────────────────────────────────────────────────────────
function exportJSON() {
  const blob = new Blob([JSON.stringify(waypoints, null, 2)],
                        {type:'application/json'});
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = 'waypoints.json';
  a.click();
  toast('Exported waypoints.json');
}

async function importFile(input) {
  if (!input.files.length) return;
  const fd = new FormData();
  fd.append('file', input.files[0]);
  const fname = input.files[0].name;
  input.value = '';
  try {
    const r = await fetch('/api/waypoints/import', {method:'POST', body:fd});
    const d = await r.json();
    if (!d.ok) { toast('Import failed: '+d.error, true); return; }
    waypoints = d.waypoints;
    markers.forEach(m => map.removeLayer(m));
    markers = [];
    waypoints.forEach((_,i)=>_addMarker(i));
    updateRoute(); renderList();
    fitAll();
    toast(`Imported ${d.count} waypoints from ${fname}`);
  } catch(e) { toast('Import error: '+e, true); }
}

// ── Sync to server ────────────────────────────────────────────────────────────
let _syncTimer = null;
function syncToServer() {
  clearTimeout(_syncTimer);
  _syncTimer = setTimeout(async () => {
    try {
      await fetch('/api/waypoints', {
        method:'POST',
        headers:{'Content-Type':'application/json'},
        body: JSON.stringify({waypoints}),
      });
    } catch(e){}
  }, 300);
}

// ── Robot connection ──────────────────────────────────────────────────────────
async function toggleConnect() {
  if (liveConnected) {
    await fetch('/api/robot/disconnect', {method:'POST'});
    liveConnected = false;
    document.getElementById('connect-btn').textContent = 'Connect Robot';
    document.getElementById('connect-btn').className = 'btn btn-orange btn-full';
    document.getElementById('centre-robot-btn').disabled = true;
    toast('Disconnected');
  } else {
    const r = await fetch('/api/robot/connect', {method:'POST'});
    const d = await r.json();
    if (d.ok) {
      liveConnected = true;
      document.getElementById('connect-btn').textContent = 'Disconnect Robot';
      document.getElementById('connect-btn').className = 'btn btn-red btn-full';
      document.getElementById('centre-robot-btn').disabled = false;
      toast('Connected to robot.py');
    } else {
      toast('Connect failed: '+d.error, true);
    }
  }
}

async function sendRoute() {
  if (!waypoints.length) { toast('No waypoints to send', true); return; }
  const r = await fetch('/api/robot/send_route', {method:'POST'});
  const d = await r.json();
  if (d.ok) toast(`Sent ${d.count} waypoints to navigator`);
  else toast('Send failed: '+d.error, true);
}

async function clearTrack() {
  await fetch('/api/robot/clear_track', {method:'POST'});
  trackPoints = [];
  if (trackLine) { map.removeLayer(trackLine); trackLine=null; }
  toast('Track cleared');
}

// ── SSE live state ────────────────────────────────────────────────────────────
const evtSrc = new EventSource('/api/live');
evtSrc.onmessage = e => {
  const d = JSON.parse(e.data);
  const conn = el('conn-badge');
  conn.textContent = d.connected ? '🟢 live' : '⚫ offline';
  conn.className   = 'live-badge' + (d.connected ? ' ok' : '');

  const s = d.state;
  if (!s) return;

  // GPS badge
  const g = s.gps || {};
  const gOk = g.latitude != null;
  const lbGps = el('lb-gps');
  lbGps.textContent = `GPS: ${gOk ? g.fix_quality_name : '--'}`;
  lbGps.className = 'live-badge' + (gOk ? (g.fix_quality>=4?' ok':' warn') : '');

  // Mode badge
  const lbMode = el('lb-mode');
  lbMode.textContent = `${s.mode}·${s.auto_type}`;
  lbMode.className = 'live-badge' + (s.mode==='AUTO' ? ' ok' : ' warn');

  // Nav badge
  const lbNav = el('lb-nav');
  if (s.nav_state && s.nav_state !== 'IDLE') {
    lbNav.style.display = '';
    let ntxt = `${s.nav_state}  WP${s.nav_wp}`;
    if (s.nav_wp_dist != null) ntxt += `  ${s.nav_wp_dist.toFixed(1)}m`;
    lbNav.textContent = ntxt;
    lbNav.className = 'live-badge info';
    refreshMarkerIcons(s.nav_wp);
  } else {
    lbNav.style.display = 'none';
    refreshMarkerIcons(-1);
  }

  // Position text
  el('lb-pos').textContent = gOk
    ? `${g.latitude.toFixed(6)}, ${g.longitude.toFixed(6)}`
    + (g.h_error_m != null ? `  ±${g.h_error_m.toFixed(2)}m` : '')
    : '';

  // Robot marker
  if (gOk) {
    robotPos = [g.latitude, g.longitude];
    if (!robotMarker) {
      const rIcon = L.divIcon({
        className:'',
        html:`<svg width="24" height="24" viewBox="-12 -12 24 24">
          <circle r="8" fill="#f5c842" opacity=".9"/>
          <circle r="4" fill="#0d0d18"/>
          <line id="hdg-line" x1="0" y1="0" x2="0" y2="-10" stroke="#f5c842" stroke-width="2"/>
        </svg>`,
        iconSize:[24,24], iconAnchor:[12,12],
      });
      robotMarker = L.marker(robotPos, {icon:rIcon, zIndexOffset:1000}).addTo(map);
    } else {
      robotMarker.setLatLng(robotPos);
    }

    // Accuracy ring
    if (g.h_error_m) {
      if (!accuracyCircle) {
        accuracyCircle = L.circle(robotPos,
          {radius:g.h_error_m, color:'#28c8e0', weight:1,
           fillColor:'#28c8e0', fillOpacity:.08}).addTo(map);
      } else {
        accuracyCircle.setLatLng(robotPos).setRadius(g.h_error_m);
      }
    }
  }

  // Track
  if (d.track && d.track.length > 1) {
    trackPoints = d.track;
    if (!trackLine) {
      trackLine = L.polyline(trackPoints,
        {color:'#40d870', weight:2, opacity:.7}).addTo(map);
    } else {
      trackLine.setLatLngs(trackPoints);
    }
  }
};

// ── Header info ───────────────────────────────────────────────────────────────
map.on('zoomend moveend', () => {
  const z    = map.getZoom();
  const cnt  = map.getCenter();
  el('header-info').textContent =
    `zoom ${z}  ·  ${cnt.lat.toFixed(5)}, ${cnt.lng.toFixed(5)}`;
});

// ── Toast ─────────────────────────────────────────────────────────────────────
let _toastTimer = null;
function toast(msg, err=false) {
  const t = el('toast');
  t.textContent = msg;
  t.style.borderColor = err ? '#e04040' : '#28c8e0';
  t.style.color = err ? '#e04040' : '#e8eaf6';
  t.classList.add('show');
  clearTimeout(_toastTimer);
  _toastTimer = setTimeout(() => t.classList.remove('show'), 2800);
}

const el = id => document.getElementById(id);

// ── Keyboard shortcuts ────────────────────────────────────────────────────────
document.addEventListener('keydown', e => {
  if (e.target.tagName === 'INPUT') return;
  if ((e.ctrlKey||e.metaKey) && e.key==='s') { e.preventDefault(); exportJSON(); }
  if (e.key==='c' || e.key==='C') fitAll();
  if (e.key==='r' || e.key==='R') centreRobot();
  if ((e.key==='Delete'||e.key==='Backspace') && selectedIdx>=0) {
    deleteWaypoint(selectedIdx);
  }
});

// ── Init ──────────────────────────────────────────────────────────────────────
// Load any server-side waypoints (e.g. from --waypoints arg)
(async () => {
  try {
    const r = await fetch('/api/waypoints');
    const d = await r.json();
    if (d.waypoints && d.waypoints.length) {
      waypoints = d.waypoints;
      waypoints.forEach((_,i)=>_addMarker(i));
      updateRoute(); renderList(); fitAll();
    }
  } catch(e){}
})();
</script>
</body>
</html>"""


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
    global _robot

    parser = argparse.ArgumentParser(description="HackyRacingRobot GPS route builder (web)")
    parser.add_argument('--host',       default='0.0.0.0')
    parser.add_argument('--port',       default=5003, type=int)
    parser.add_argument('--waypoints',  default=None, help='Load waypoints JSON on start')
    parser.add_argument('--live',       action='store_true', help='Auto-connect to robot.py')
    args = parser.parse_args()

    # Pre-load waypoints
    if args.waypoints:
        try:
            with open(args.waypoints) as f:
                wps = json.load(f)
            with _lock:
                _waypoints.extend(wps)
            print(f"Loaded {len(wps)} waypoints from {args.waypoints}")
        except Exception as e:
            print(f"Warning: could not load {args.waypoints}: {e}", file=sys.stderr)

    # Auto-connect
    if args.live:
        try:
            from robot_daemon import Robot
            _robot = Robot()
            _robot.start()
            _start_robot_poll()
            print("Connected to robot.py")
        except Exception as e:
            print(f"Warning: could not connect to robot.py: {e}", file=sys.stderr)

    logging.basicConfig(level=logging.INFO,
                        format='%(levelname)s %(name)s: %(message)s')
    logging.getLogger('werkzeug').setLevel(logging.WARNING)

    print(f"Route builder → http://{_local_ip()}:{args.port}/")
    try:
        app.run(host=args.host, port=args.port,
                threaded=True, use_reloader=False)
    finally:
        if _robot:
            try: _robot.stop()
            except Exception: pass


if __name__ == '__main__':
    main()
