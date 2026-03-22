#!/usr/bin/env python3
"""
camera_web.py — Mobile-friendly web GUI for the camera.

Streams live MJPEG video, shows sharpness and stats, and exposes
controls for capture, ArUco detection, exposure, gain, resolution,
rotation, and lens calibration.

Usage
-----
  python3 camera_web.py
  python3 camera_web.py --port 8080 --width 1456 --height 1088

Open in any browser on the same network:
  http://<pi-ip>:8080/
"""

import argparse
import io
import threading
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
from flask import Flask, Response, jsonify, request, send_from_directory
from picamera2 import Picamera2

from robot.aruco_detector import ArucoDetector, ArUcoState, ARUCO_DICT

# ── Paths ─────────────────────────────────────────────────────────────────────

_HERE       = Path(__file__).parent
IMAGE_DIR   = _HERE / "saved_images"
CALIB_FILE  = _HERE / "camera_cal.npz"

# ── Capture sizes ─────────────────────────────────────────────────────────────

CAPTURE_SIZES = [
    (640,  480),
    (1280, 720),
    (1456, 1088),
]

GAINS     = [1.0, 2.0, 4.0, 8.0, 16.0]
EXP_STEPS = [500, 1000, 2000, 4000, 8000, 16000, 33000, 66000]

ARUCO_DICTS = [
    "DICT_4X4_50",
    "DICT_4X4_100",
    "DICT_4X4_1000",
    "DICT_5X5_100",
    "DICT_6X6_100",
]

# ── Frame processing ──────────────────────────────────────────────────────────

def _sharpness(gray: np.ndarray) -> float:
    small = cv2.resize(gray, (640, 480), interpolation=cv2.INTER_AREA)
    return float(cv2.Laplacian(small, cv2.CV_64F).var())


def _rotate(frame: np.ndarray, degrees: int) -> np.ndarray:
    if degrees == 0:
        return frame
    return np.rot90(frame, k=-(degrees // 90))


# ── Camera state (shared between threads) ─────────────────────────────────────

class CameraState:
    def __init__(self):
        self.lock        = threading.Lock()

        # camera
        self.size_idx    = 2          # 1456×1088 native
        self.rotation    = 0
        self.gain_idx    = 0
        self.exp_idx     = 4          # 8000 µs
        self.auto_exp    = True

        # features
        self.show_aruco  = False
        self.dict_idx    = 0
        self.use_calib   = False

        # live stats
        self.sharpness   = 0.0
        self.fps         = 0.0
        self.aruco_state = ArUcoState()

        # calibration maps
        self._cam_mtx    = None
        self._dist       = None
        self._cal_size   = None
        self._map1       = None
        self._map2       = None
        self._map_wh     = None

        # latest JPEG for streaming + saving
        self._jpeg       = None
        self._raw_frame  = None   # RGB, pre-annotation

        self._load_calib()

    def _load_calib(self):
        if CALIB_FILE.exists():
            try:
                cal = np.load(CALIB_FILE)
                self._cam_mtx  = cal['camera_matrix']
                self._dist     = cal['dist_coeffs']
                self._cal_size = tuple(int(v) for v in cal['frame_size'])
            except Exception as e:
                print(f"Warning: could not load calibration: {e}")

    @property
    def calib_available(self):
        return self._cam_mtx is not None

    def get_cal_maps(self, w, h):
        if self._cam_mtx is None:
            return None, None
        if self._map_wh != (w, h):
            cal_w, cal_h = self._cal_size
            sx, sy = w / cal_w, h / cal_h
            mtx = self._cam_mtx.copy()
            mtx[0, 0] *= sx; mtx[1, 1] *= sy
            mtx[0, 2] *= sx; mtx[1, 2] *= sy
            new_mtx, _ = cv2.getOptimalNewCameraMatrix(
                mtx, self._dist, (w, h), 1, (w, h))
            self._map1, self._map2 = cv2.initUndistortRectifyMap(
                mtx, self._dist, None, new_mtx, (w, h), cv2.CV_16SC2)
            self._map_wh = (w, h)
        return self._map1, self._map2

    def get_jpeg(self):
        with self.lock:
            return self._jpeg

    def set_frame(self, jpeg, raw):
        with self.lock:
            self._jpeg      = jpeg
            self._raw_frame = raw

    def get_raw(self):
        with self.lock:
            return self._raw_frame


# ── Camera worker thread ──────────────────────────────────────────────────────

def _make_cam(w, h):
    cam = Picamera2()
    cam.configure(cam.create_video_configuration(
        main={"size": (w, h), "format": "RGB888"},
        controls={"FrameRate": 30},
    ))
    cam.start()
    return cam


def camera_worker(state: CameraState):
    cap_w, cap_h = CAPTURE_SIZES[state.size_idx]
    cam      = _make_cam(cap_w, cap_h)
    detector = ArucoDetector(ARUCO_DICTS[state.dict_idx], draw=False)

    prev_size_idx = state.size_idx
    prev_dict_idx = state.dict_idx

    fps_count = 0
    fps_t0    = time.time()

    def apply_controls():
        if state.auto_exp:
            cam.set_controls({"AeEnable": True,
                              "AnalogueGain": GAINS[state.gain_idx]})
        else:
            cam.set_controls({"AeEnable":     False,
                              "ExposureTime": EXP_STEPS[state.exp_idx],
                              "AnalogueGain": GAINS[state.gain_idx]})

    while True:
        # Restart camera if resolution changed
        if state.size_idx != prev_size_idx:
            cam.stop(); cam.close()
            cap_w, cap_h = CAPTURE_SIZES[state.size_idx]
            cam = _make_cam(cap_w, cap_h)
            prev_size_idx = state.size_idx
            apply_controls()

        # Swap detector if dictionary changed
        if state.dict_idx != prev_dict_idx:
            detector      = ArucoDetector(ARUCO_DICTS[state.dict_idx], draw=False)
            prev_dict_idx = state.dict_idx

        try:
            raw = cam.capture_array()[:, :, ::-1]   # BGR → RGB
        except Exception:
            time.sleep(0.05)
            continue

        frame = _rotate(raw, state.rotation)

        # Undistort
        if state.use_calib:
            h, w = frame.shape[:2]
            m1, m2 = state.get_cal_maps(w, h)
            if m1 is not None:
                frame = cv2.remap(frame, m1, m2, cv2.INTER_LINEAR)

        gray           = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        state.sharpness = _sharpness(gray)

        annotated = frame.copy()

        if state.show_aruco:
            aruco_st = detector.detect(annotated)
            state.aruco_state = aruco_st
            # Draw overlays
            font = cv2.FONT_HERSHEY_SIMPLEX
            for tag in aruco_st.tags.values():
                tl, tr = tag.top_left,     tag.top_right
                br, bl = tag.bottom_right, tag.bottom_left
                cv2.line(annotated, tl, tr, (0, 220, 80), 2)
                cv2.line(annotated, tr, br, (0, 220, 80), 2)
                cv2.line(annotated, br, bl, (0, 220, 80), 2)
                cv2.line(annotated, bl, tl, (0, 220, 80), 2)
                cv2.circle(annotated, (tag.center_x, tag.center_y), 4, (220, 40, 40), -1)
                cv2.putText(annotated, str(tag.id),
                            (tl[0], tl[1] - 10), font, 0.6, (230, 230, 240), 2)
            for gate in aruco_st.gates.values():
                col = (40, 120, 220) if gate.correct_dir else (220, 40, 40)
                cv2.line(annotated,
                         (gate.centre_x, gate.centre_y - 50),
                         (gate.centre_x, gate.centre_y), col, 4)
                cv2.putText(annotated, f"G{gate.gate_id}",
                            (gate.centre_x + 6, gate.centre_y - 52),
                            font, 0.6, col, 2)
        else:
            state.aruco_state = ArUcoState()

        # Encode JPEG
        bgr  = cv2.cvtColor(annotated, cv2.COLOR_RGB2BGR)
        ok, buf = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            state.set_frame(buf.tobytes(), frame.copy())

        fps_count += 1
        now = time.time()
        if now - fps_t0 >= 1.0:
            state.fps  = fps_count / (now - fps_t0)
            fps_count  = 0
            fps_t0     = now


# ── Flask app ─────────────────────────────────────────────────────────────────

app   = Flask(__name__)
state = CameraState()


def _mjpeg_stream():
    while True:
        jpeg = state.get_jpeg()
        if jpeg:
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
                   + jpeg + b'\r\n')
        time.sleep(0.033)


@app.route('/stream')
def stream():
    return Response(_mjpeg_stream(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/stats')
def stats():
    s  = state.sharpness
    at = state.aruco_state
    cap_w, cap_h = CAPTURE_SIZES[state.size_idx]
    return jsonify(
        sharpness      = round(s, 1),
        sharpness_pct  = min(int(s / 600 * 100), 100),
        sharpness_band = ("good" if s > 300 else "ok" if s > 80 else "poor"),
        fps            = round(state.fps, 1),
        size           = f"{cap_w}×{cap_h}",
        rotation       = state.rotation,
        gain           = f"{GAINS[state.gain_idx]:.0f}×",
        exposure       = ("auto" if state.auto_exp
                          else f"{EXP_STEPS[state.exp_idx]/1000:.1f} ms"),
        aruco          = state.show_aruco,
        aruco_dict     = ARUCO_DICTS[state.dict_idx].replace("DICT_", ""),
        aruco_tags     = len(at.tags),
        aruco_gates    = len(at.gates),
        calib          = state.use_calib,
        calib_avail    = state.calib_available,
    )


@app.route('/control', methods=['POST'])
def control():
    cmd = request.json or {}
    action = cmd.get('action', '')

    if action == 'size':
        state.size_idx = (state.size_idx + 1) % len(CAPTURE_SIZES)

    elif action == 'rotate':
        state.rotation = (state.rotation + 90) % 360

    elif action == 'exp_up':
        state.auto_exp = False
        state.exp_idx  = min(state.exp_idx + 1, len(EXP_STEPS) - 1)

    elif action == 'exp_down':
        state.auto_exp = False
        state.exp_idx  = max(state.exp_idx - 1, 0)

    elif action == 'exp_auto':
        state.auto_exp = True

    elif action == 'gain_up':
        state.gain_idx = min(state.gain_idx + 1, len(GAINS) - 1)

    elif action == 'gain_down':
        state.gain_idx = max(state.gain_idx - 1, 0)

    elif action == 'aruco':
        state.show_aruco = not state.show_aruco

    elif action == 'dict':
        state.dict_idx = (state.dict_idx + 1) % len(ARUCO_DICTS)

    elif action == 'calib':
        if state.calib_available:
            state.use_calib = not state.use_calib

    elif action == 'capture':
        raw = state.get_raw()
        if raw is not None:
            IMAGE_DIR.mkdir(exist_ok=True)
            ts   = datetime.now().strftime("%Y%m%d_%H%M%S")
            path = IMAGE_DIR / f"capture_{ts}.png"
            jpeg = state.get_jpeg()
            if jpeg:
                buf = np.frombuffer(jpeg, np.uint8)
                img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
                cv2.imwrite(str(path), img)
            return jsonify(ok=True, file=path.name)

    return jsonify(ok=True)


@app.route('/')
def index():
    return _HTML


# ── HTML ──────────────────────────────────────────────────────────────────────

_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1">
<title>Camera Monitor</title>
<style>
  *, *::before, *::after { box-sizing: border-box; margin: 0; padding: 0; }

  :root {
    --bg:     #12121e;
    --panel:  #1c1c2e;
    --border: #3c3c5a;
    --white:  #e6e6f0;
    --gray:   #8080a0;
    --green:  #3cdc50;
    --yellow: #f0c828;
    --red:    #dc3c3c;
    --cyan:   #3cc8dc;
    --blue:   #3c78dc;
    --btn:    #2a2a42;
    --btn-h:  #3a3a5a;
    --radius: 10px;
  }

  body {
    background: var(--bg);
    color: var(--white);
    font-family: 'Courier New', monospace;
    min-height: 100vh;
    display: flex;
    flex-direction: column;
    gap: 8px;
    padding: 8px;
  }

  h1 {
    font-size: 1rem;
    color: var(--cyan);
    text-align: center;
    padding: 4px 0;
    letter-spacing: 2px;
  }

  /* ── Stream ── */
  #stream-wrap {
    position: relative;
    background: #000;
    border-radius: var(--radius);
    overflow: hidden;
    border: 1px solid var(--border);
  }
  #stream-wrap img {
    width: 100%;
    display: block;
  }
  #crosshair {
    position: absolute;
    inset: 0;
    pointer-events: none;
  }
  #crosshair::before, #crosshair::after {
    content: '';
    position: absolute;
    background: rgba(60,220,80,0.7);
  }
  #crosshair::before { top: 50%; left: calc(50% - 20px); width: 40px; height: 1px; }
  #crosshair::after  { left: 50%; top: calc(50% - 20px); height: 40px; width: 1px; }
  #save-flash {
    position: absolute;
    top: 8px; left: 50%;
    transform: translateX(-50%);
    background: rgba(60,220,80,0.9);
    color: #000;
    font-weight: bold;
    padding: 4px 14px;
    border-radius: 6px;
    font-size: 0.8rem;
    display: none;
  }

  /* ── Sharpness bar ── */
  #sharp-row {
    display: flex;
    align-items: center;
    gap: 8px;
    padding: 0 2px;
  }
  #sharp-label { font-size: 0.7rem; color: var(--gray); white-space: nowrap; }
  #sharp-track {
    flex: 1;
    height: 10px;
    background: var(--panel);
    border-radius: 5px;
    border: 1px solid var(--border);
    overflow: hidden;
  }
  #sharp-fill {
    height: 100%;
    width: 0%;
    border-radius: 5px;
    transition: width 0.3s, background 0.3s;
  }
  #sharp-val { font-size: 0.75rem; min-width: 36px; text-align: right; }

  /* ── Stats grid ── */
  #stats {
    display: grid;
    grid-template-columns: repeat(4, 1fr);
    gap: 4px;
  }
  .stat {
    background: var(--panel);
    border: 1px solid var(--border);
    border-radius: 8px;
    padding: 5px 4px;
    text-align: center;
  }
  .stat .lbl { font-size: 0.6rem; color: var(--gray); }
  .stat .val { font-size: 0.8rem; font-weight: bold; }

  /* ── Buttons ── */
  #controls {
    display: grid;
    grid-template-columns: repeat(3, 1fr);
    gap: 6px;
  }
  .btn {
    background: var(--btn);
    border: 1px solid var(--border);
    border-radius: var(--radius);
    color: var(--white);
    font-family: inherit;
    font-size: 0.8rem;
    padding: 12px 6px;
    cursor: pointer;
    text-align: center;
    -webkit-tap-highlight-color: transparent;
    user-select: none;
    transition: background 0.1s;
  }
  .btn:active  { background: var(--btn-h); }
  .btn.active  { border-color: var(--cyan); color: var(--cyan); }
  .btn.capture { border-color: var(--green); color: var(--green); font-size: 1rem; }
  .btn.danger  { border-color: var(--red);   color: var(--red); }

  .btn .icon { font-size: 1.1rem; display: block; }
  .btn .sub  { font-size: 0.65rem; color: var(--gray); margin-top: 2px; }

  /* ── ArUco tag list ── */
  #tag-list {
    background: var(--panel);
    border: 1px solid var(--border);
    border-radius: var(--radius);
    padding: 8px;
    font-size: 0.75rem;
    display: none;
    min-height: 40px;
  }
  #tag-list.visible { display: block; }
  .tag-row { display: flex; gap: 12px; padding: 2px 0; border-bottom: 1px solid var(--border); }
  .tag-row:last-child { border-bottom: none; }
  .tag-id { color: var(--cyan); min-width: 40px; }
</style>
</head>
<body>

<h1>&#128247; Camera Monitor</h1>

<div id="stream-wrap">
  <img id="stream" src="/stream" alt="stream">
  <div id="crosshair"></div>
  <div id="save-flash">&#10003; Saved</div>
</div>

<div id="sharp-row">
  <span class="lbl" id="sharp-label">Sharpness</span>
  <div id="sharp-track"><div id="sharp-fill"></div></div>
  <span id="sharp-val">—</span>
</div>

<div id="stats">
  <div class="stat"><div class="lbl">FPS</div><div class="val" id="s-fps">—</div></div>
  <div class="stat"><div class="lbl">Size</div><div class="val" id="s-size">—</div></div>
  <div class="stat"><div class="lbl">Exposure</div><div class="val" id="s-exp">—</div></div>
  <div class="stat"><div class="lbl">Gain</div><div class="val" id="s-gain">—</div></div>
  <div class="stat"><div class="lbl">Rotation</div><div class="val" id="s-rot">—</div></div>
  <div class="stat"><div class="lbl">Calib</div><div class="val" id="s-calib">—</div></div>
  <div class="stat"><div class="lbl">Tags</div><div class="val" id="s-tags">—</div></div>
  <div class="stat"><div class="lbl">Dict</div><div class="val" id="s-dict">—</div></div>
</div>

<div id="controls">
  <button class="btn capture" onclick="cmd('capture')">
    <span class="icon">&#128247;</span>Save
  </button>
  <button class="btn" id="btn-aruco" onclick="cmd('aruco')">
    <span class="icon">&#9634;</span>ArUco
  </button>
  <button class="btn" id="btn-calib" onclick="cmd('calib')">
    <span class="icon">&#128259;</span>Calib
  </button>

  <button class="btn" onclick="cmd('exp_up')">
    <span class="icon">&#9728;</span>Exp +
  </button>
  <button class="btn" onclick="cmd('exp_down')">
    <span class="icon">&#9727;</span>Exp −
  </button>
  <button class="btn" onclick="cmd('exp_auto')">
    <span class="icon">&#9654;</span>Auto Exp
  </button>

  <button class="btn" onclick="cmd('gain_up')">
    <span class="icon">&#43;</span>Gain +
  </button>
  <button class="btn" onclick="cmd('gain_down')">
    <span class="icon">&#8722;</span>Gain −
  </button>
  <button class="btn" onclick="cmd('rotate')">
    <span class="icon">&#8635;</span>Rotate
  </button>

  <button class="btn" onclick="cmd('size')" style="grid-column:span 2">
    <span class="icon">&#8597;</span>Cycle Resolution
  </button>
  <button class="btn" onclick="cmd('dict')">
    <span class="icon">&#8644;</span>Dict
  </button>
</div>

<div id="tag-list"></div>

<script>
async function cmd(action) {
  const res = await fetch('/control', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({action})
  });
  const data = await res.json();
  if (action === 'capture' && data.ok) {
    const fl = document.getElementById('save-flash');
    fl.textContent = '✓ ' + (data.file || 'Saved');
    fl.style.display = 'block';
    setTimeout(() => fl.style.display = 'none', 2000);
  }
}

const SHARP_COLORS = { good: '#3cdc50', ok: '#f0c828', poor: '#dc3c3c' };

async function poll() {
  try {
    const r  = await fetch('/stats');
    const d  = await r.json();

    // Sharpness bar
    const fill = document.getElementById('sharp-fill');
    fill.style.width      = d.sharpness_pct + '%';
    fill.style.background = SHARP_COLORS[d.sharpness_band] || '#888';
    document.getElementById('sharp-val').textContent  = d.sharpness;
    document.getElementById('sharp-val').style.color  = SHARP_COLORS[d.sharpness_band];

    // Stats
    document.getElementById('s-fps').textContent   = d.fps;
    document.getElementById('s-size').textContent  = d.size;
    document.getElementById('s-exp').textContent   = d.exposure;
    document.getElementById('s-gain').textContent  = d.gain;
    document.getElementById('s-rot').textContent   = d.rotation + '°';

    const calibEl = document.getElementById('s-calib');
    calibEl.textContent  = d.calib_avail ? (d.calib ? 'ON' : 'OFF') : 'none';
    calibEl.style.color  = d.calib ? '#3cdc50' : (d.calib_avail ? '#e6e6f0' : '#8080a0');

    document.getElementById('s-tags').textContent  =
      d.aruco ? d.aruco_tags + (d.aruco_gates ? '/' + d.aruco_gates + 'g' : '') : 'OFF';
    document.getElementById('s-dict').textContent  = d.aruco ? d.aruco_dict : '—';

    // Button states
    document.getElementById('btn-aruco').classList.toggle('active', d.aruco);
    document.getElementById('btn-calib').classList.toggle('active', d.calib);

    // Tag list
    const tl = document.getElementById('tag-list');
    if (d.aruco && d.aruco_tags > 0) {
      tl.classList.add('visible');
      // Tag detail comes from stats — we just show count here;
      // full detail would need a separate endpoint
      tl.innerHTML = '<div style="color:var(--gray);font-size:0.7rem">'
        + d.aruco_tags + ' tag(s) detected'
        + (d.aruco_gates ? ' · ' + d.aruco_gates + ' gate(s)' : '') + '</div>';
    } else {
      tl.classList.remove('visible');
    }
  } catch(e) {}
}

setInterval(poll, 500);
poll();
</script>
</body>
</html>
"""

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
    parser = argparse.ArgumentParser(description="Camera web monitor")
    parser.add_argument('--host',   default='0.0.0.0')
    parser.add_argument('--port',   default=8080, type=int)
    args = parser.parse_args()

    t = threading.Thread(target=camera_worker, args=(state,), daemon=True)
    t.start()

    print(f"Camera web monitor → http://{_local_ip()}:{args.port}/")
    app.run(host=args.host, port=args.port, threaded=True, use_reloader=False)


if __name__ == '__main__':
    main()
