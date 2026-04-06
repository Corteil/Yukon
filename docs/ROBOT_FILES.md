# Robot Files Reference

Documentation for all Python files in the project root.
Run every script from the repo root unless noted otherwise.

---

## Full robot stack

### Common flags

All robot scripts — `robot_daemon.py`, `robot_dashboard.py` —
accept the same core flags.  All are optional — config file values are used when a
flag is omitted.

| Flag | Default | Description |
|------|---------|-------------|
| `--config PATH` | `robot.ini` | Config file |
| `--yukon-port PORT` | from config | Yukon USB serial port |
| `--ibus-port PORT` | from config | iBUS UART device (**deprecated** — RC input is now handled by Yukon GP26; only used by `rc_drive.py`) |
| `--gps-port PORT` | from config | GPS serial port |
| `--lidar-port PORT` | from config | LiDAR serial port |
| `--ntrip-host HOST` | from config | NTRIP caster hostname |
| `--ntrip-port N` | from config | NTRIP caster port |
| `--ntrip-mount MOUNT` | from config | NTRIP mountpoint |
| `--ntrip-user USER` | from config | NTRIP username |
| `--ntrip-password PASS` | from config | NTRIP password |
| `--rtcm-port PORT` | from config | RTCM serial correction input (overrides NTRIP) |
| `--rtcm-baud N` | from config | RTCM baud rate |
| `--no-camera` | off | Disable camera subsystem |
| `--no-lidar` | off | Disable LiDAR subsystem |
| `--no-gps` | off | Disable GPS subsystem |
| `--no-motors` | off | Suppress drive commands at startup (bench-test mode). RC speed data is still read. The Yukon receives `CMD_MODE=ESTOP` each heartbeat tick so motors stop in all modes. Can also be toggled at runtime via the **No Motors** button in the dashboard status bar. |

---

### robot_daemon.py

The central Pi-side daemon.  All hardware subsystems run as daemon threads inside
this single process.  GUI and web frontends import `Robot` from here — they do not
talk to hardware directly.

Subsystem threads:

| Thread | Rate | Responsibility |
|--------|------|----------------|
| `yukon_rx` | event-driven | Read ACK / NAK / sensor packets from Yukon over USB serial |
| `camera` | configured fps (default 30 Hz) | Capture, rotate, ArUco detect, store latest frame |
| `ld06` | packet-driven | Parse LD06 LiDAR packets, update latest scan |
| `gps` | NMEA rate | Parse NMEA sentences, inject RTCM corrections |
| `gps_log` | 5 Hz (configurable) | Write GPS CSV log when enabled |
| `telemetry` | 1 Hz | Request sensor data from Yukon, update `Telemetry` |
| `control` | 50 Hz (configurable) | Query RC channels via CMD_RC_QUERY (10 Hz), send CMD_MODE heartbeat, compute motor speeds from navigator in AUTO, send to Yukon |
| `system` | 1 Hz | Poll CPU / mem / disk via psutil |

Key public API:

```python
robot = Robot(...)
robot.start()                       # connect hardware, launch all threads
state = robot.get_state()           # RobotState snapshot (thread-safe)
frame = robot.get_frame(cam='front_left')      # latest camera frame (numpy RGB) or None
                                               # cam: 'front_left' | 'front_right' | 'rear'
aruco = robot.get_aruco_state(cam='front_left') # latest ArUcoState or None
hdg   = robot.get_heading()         # IMU heading in degrees or None

# Per-camera ArUco control
robot.set_aruco_enabled(enabled, cam='all')  # cam: 'all' | 'front_left' | 'front_right' | 'rear'
robot.toggle_aruco(cam='all') -> bool        # returns new enabled state
robot.get_aruco_enabled(cam='all')           # cam='all' returns dict; specific cam returns bool

# Post-rotation capture size (ArUco runs on the rotated frame)
robot.get_cam_capture_size(cam) -> (width, height)  # swapped for 90°/270° rotations

robot.start_cam_recording(cam='all') # cam: 'all' | 'front_left' | 'front_right' | 'rear'
robot.stop_cam_recording(cam='all') # returns list of saved file paths
robot.is_cam_recording(cam='any')   # cam='any' returns True if any camera is recording

# FPV camera power (DualOutputModule output 0 in SLOT4)
robot.set_bench(on: bool)           # enable / disable FPV camera power output

# No-motors mode (runtime toggle — also set by --no-motors at startup)
robot.set_no_motors(on: bool)       # suppress drive commands; Yukon receives CMD_MODE=ESTOP
                                    # so motors stop in MANUAL (RC-driven) and AUTO modes

# Depth map (requires [depth] enabled = true in robot.ini)
robot.get_depth_map() -> DepthMap                    # latest depth map snapshot
robot.get_depth_at(px: int, py: int) -> float | None # metric depth at pixel coords

robot.start_data_log()              # save JSONL to ~/Documents/HackyRacingRobot/
robot.stop_data_log()
robot.is_data_logging()             # bool

robot.stop()                        # shutdown all subsystems cleanly

# LED strip (NeoPixel, 8 LEDs on Yukon SLOT3)
robot.yukon.set_strip(preset)           # fill all 8 LEDs with a colour preset
                                        # preset: STRIP_OFF(0) … STRIP_WHITE(8)
robot.yukon.set_pixel(index, colour)    # stage one pixel (0-indexed); call show_pixels() to commit
robot.yukon.show_pixels()              # push staged pixel values to the strip
robot.yukon.set_pixels(colours)         # set all 8 pixels at once; colours is a list of 8 preset ints
robot.yukon.set_pattern(pattern, colour=0)  # run an autonomous animation on the Yukon
                                            # pattern: PATTERN_OFF(0), PATTERN_LARSON(1),
                                            #          PATTERN_RANDOM(2), PATTERN_RAINBOW(3),
                                            #          PATTERN_RETRO_COMPUTER(4), PATTERN_CONVERGE(5),
                                            #          PATTERN_ESTOP_FLASH(6)
                                            # colour: palette index used by pattern (0 = keep current)
```

Colour preset constants (defined on `_YukonLink`):

| Constant | Index | RGB |
|----------|-------|-----|
| `STRIP_OFF` | 0 | (0, 0, 0) |
| `STRIP_RED` | 1 | (255, 0, 0) |
| `STRIP_GREEN` | 2 | (0, 255, 0) |
| `STRIP_BLUE` | 3 | (0, 0, 255) |
| `STRIP_ORANGE` | 4 | (255, 165, 0) |
| `STRIP_YELLOW` | 5 | (255, 255, 0) |
| `STRIP_CYAN` | 6 | (0, 255, 255) |
| `STRIP_MAGENTA` | 7 | (255, 0, 255) |
| `STRIP_WHITE` | 8 | (255, 255, 255) |

The control thread applies an LED preset automatically on mode changes — configured in `[leds]` in `robot.ini`.
Defaults: `larson` → MANUAL, `retro_computer` → AUTO, `estop_flash` → ESTOP.

**ML data log** (`start_data_log()`) writes one JSONL record per tick (default 10 Hz).
Each record contains a complete snapshot of all sensor inputs and motor outputs:

| Field | Contents |
|-------|----------|
| `ts` / `ts_iso` | Unix timestamp + ISO datetime string |
| `mode`, `auto_type`, `speed_scale` | Robot operating state |
| `rc_channels` | All 14 raw RC µs values |
| `drive` | `left` / `right` motor outputs — the training labels |
| `telemetry` | Voltage, current, temperatures, IMU heading / pitch / roll, fault flags |
| `gps` | Lat, lon, alt, speed, fix quality, satellites, HDOP, h_error |
| `lidar` | Full angle and distance arrays |
| `aruco` | All detected tags and gates with bearings and distances |
| `nav` | Navigator state, target gate/waypoint, bearing error |
| `system` | CPU %, CPU temp, memory %, disk % |

`robot.start()` retries the Yukon serial connection every 3 seconds if the port is
not yet available — it will not raise an error or exit.

`robot_daemon.py` can also be run directly as a headless daemon:

```bash
python3 robot_daemon.py
python3 robot_daemon.py --no-motors
python3 robot_daemon.py --yukon-port /dev/ttyACM0
```

Accepts all [common flags](#common-flags) — no additional flags.

All configuration is read from `robot.ini`.  Every parameter can be overridden by
passing keyword arguments to `Robot(...)`.

`setup_logging()` is exported from `robot_daemon.py` and used by all frontends.  It
configures a console handler and a rotating file handler (`logs/robot.log`,
5 MB × 3 files) in one call:

```python
from robot_daemon import setup_logging
log_path = setup_logging()          # writes to logs/robot.log
log_path = setup_logging('/tmp')    # custom log directory
```

---

### robot_dashboard.py

Unified Flask web dashboard (port 5000).  Same `Robot` backend as `robot_daemon.py`.

On desktop and touchscreen the UI shows a configurable 2×2 panel grid; on mobile browsers (≤700 px) it switches to a tab view.

**Panel types**

| Panel | Contents |
|-------|----------|
| `front_left` | Front-left camera stream with optional ArUco / bearing overlay |
| `front_right` | Front-right camera stream with optional ArUco / bearing overlay |
| `rear` | Rear camera stream |
| `lidar` | LiDAR polar plot |
| `gps_sky` | GPS sky view (satellites) |
| `gps_track` | GPS track map |
| `depth_map` | Depth map visualisation |
| `telemetry` | Voltage, current, temperatures, IMU heading / pitch / roll |
| `system` | CPU, memory, disk, system stats |
| `imu` | IMU compass and orientation display |

The 2×2 panel layout is configurable via `[layout_presets]` in `robot.ini`.  Double-tap any panel to expand it.

**ArUco overlay** — two levels, controlled per camera:
- ArUco button (level 1): enables ArUco detection and shows tag bounding boxes
- Bearing button (level 2): additionally shows bearing lines, distance labels, boresight crosshair, and IMU arc

**Features**
- Real-time telemetry via Server-Sent Events (10 Hz)
- MJPEG camera streams for all three cameras
- LiDAR polar plot
- Navigation quad panel — overhead radar with gates, tags, target bearing line, and aim-point distance/bearing readout
- ESTOP / Reset controls
- ArUco toggle and bearing overlay toggle (per camera)
- **No Motors** toggle button — suppresses drive commands and forces Yukon ESTOP so motors stop in both MANUAL (RC-driven) and AUTO modes; button glows orange when active
- No-motors warning banner
- GPS logging badge
- Camera recording and ML data logging buttons
- FPV camera power toggle
- Terminal log panel with filter bar and auto-scroll
- Configurable layout presets (Race / Setup / Nav etc.) via `[layout_presets]` in `robot.ini`

```bash
python3 robot_dashboard.py              # 0.0.0.0:5000
python3 robot_dashboard.py --port 5000
python3 robot_dashboard.py --no-motors
```

Accepts all [common flags](#common-flags), plus:

| Flag | Default | Description |
|------|---------|-------------|
| `--host HOST` | `0.0.0.0` | Bind address |
| `--port N` | `5000` | HTTP port |
| `--debug` | off | Enable Flask debug mode |

Open `http://<pi-ip>:5000/` in any browser on the same network.

---

## Output directories

All output paths are configured in the `[output]` section of `robot.ini`.
Leave a value blank to use the default.

| Setting | Default | Contents |
|---------|---------|----------|
| `images_dir` | `~/Pictures/HackyRacingRobot` | Snapshots from `camera_web.py` and `camera_monitor.py` |
| `videos_dir` | `~/Videos/HackyRacingRobot` | MP4 recordings from camera recording feature |
| `data_log_dir` | `~/Documents/HackyRacingRobot` | JSONL ML training data logs |
| `max_recording_minutes` | `0` (unlimited) | Roll video to a new file every N minutes; `0` = no limit |

Directories are created automatically on first use.

Video filenames: `recording_YYYYMMDD_HHMMSS.mp4` — each frame has a `DD-MM-YY HH:MM:SS` timestamp burned in the top-left corner.  When `max_recording_minutes > 0` each segment gets its own timestamped filename.

Data log filenames: `data_YYYYMMDD_HHMMSS.jsonl`

---

## Standalone tools

### rc_drive.py

Minimal RC-to-motor bridge.  No GUI, no camera, no navigator — just reads iBUS
and drives the Yukon motors.  Useful for driving tests without the full robot
stack, or as a fallback if `robot_daemon.py` will not start.

Tank mix: `left = clamp(throttle − aileron)`, `right = clamp(throttle + aileron)`

Sends a `CMD_MODE=AUTO` heartbeat to the Yukon every 100 ms — required because the
Yukon firmware only accepts `CMD_LEFT`/`CMD_RIGHT` in AUTO mode, and will ESTOP
within 500 ms if the heartbeat stops (Pi crash / USB disconnect watchdog).

Failsafe: sends `CMD_KILL` if no valid iBUS packet is received for 0.5 s.

```bash
python3 rc_drive.py
python3 rc_drive.py --yukon-port /dev/ttyACM0 --ibus-port /dev/ttyAMA3
python3 rc_drive.py --reverse-left      # flip left motor direction
python3 rc_drive.py --reverse-right     # flip right motor direction
```

| Flag | Default | Description |
|------|---------|-------------|
| `--yukon-port PORT` | auto-detect | Yukon USB serial port |
| `--ibus-port PORT` | `/dev/ttyAMA3` | iBUS UART device |
| `--throttle-ch N` | `3` | Throttle channel (1-based) |
| `--steer-ch N` | `1` | Steering channel (1-based) |
| `--reverse-left` | off | Flip left motor direction |
| `--reverse-right` | off | Flip right motor direction |
| `--ramp-rate R` | `1.0` | Max speed change per second; `0` = instant |

---

### lidar_gui.py

Standalone pygame polar visualiser for the LD06 LiDAR.  Runs independently of
the robot stack — useful for checking LiDAR alignment and testing the sensor
before a run.

Points are distance-coloured: red (near) → yellow → cyan (far).  A ghost trail
of the previous scan is shown dimmed behind the current scan.

```bash
python3 lidar_gui.py                    # default /dev/ttyAMA0
python3 lidar_gui.py /dev/ttyAMA0
```

**Keys**

| Key | Action |
|-----|--------|
| Scroll / `+` / `-` | Zoom in / out (500 mm – 12 000 mm range) |
| `F` | Freeze / unfreeze scan |
| `G` | Toggle grid (range rings + compass spokes) |
| `Q` / `Esc` | Quit |

Status bar shows: RPM, scan rate (Hz), point count, display range, nearest object.

---

### camera_monitor.py

Standalone pygame camera tool for lens setup, focus, and ArUco overlay testing.
Requires picamera2 (Pi only).

```bash
python3 camera_monitor.py
```

**Keys**

| Key | Action |
|-----|--------|
| `M` | Cycle colour mode (Colour → Greyscale → False Colour → Edges → Focus Peak) |
| `S` | Cycle capture resolution (640×480 → 1280×720 → 1456×1088) |
| `R` | Rotate 90° clockwise |
| `H` | Toggle histogram overlay |
| `↑` / `↓` | Exposure +/− step (switches to manual) |
| `[` / `]` | Analogue gain down / up |
| `A` | Restore auto exposure / gain |
| `T` | Toggle ArUco tag detection |
| `D` | Cycle ArUco dictionary |
| `Space` | Save frame to `~/Pictures/HackyRacingRobot/` |
| `Q` / `Esc` | Quit |

**Colour modes**

| Mode | Description |
|------|-------------|
| Colour | Normal RGB |
| Greyscale | Greyscale conversion |
| False Colour | Jet colour map — highlights exposure gradients |
| Edges | Canny edge detection |
| Focus Peak | Sharpest edges overlaid red — move the lens until the highlight is brightest on your target |

The **Sharpness** score (Laplacian variance) in the status bar rises as focus improves.
`camera_cal.npz` is loaded automatically for lens undistortion if present.

---

### camera_web.py

Mobile-friendly Flask camera interface (port 8080).  Streams live MJPEG video with
controls for all the same features as `camera_monitor.py`, accessible from any
browser on the same network.  Requires picamera2.

```bash
python3 camera_web.py                   # 0.0.0.0:8080
python3 camera_web.py --port 8080
python3 camera_web.py --width 1456 --height 1088
```

**Controls (via browser UI)**
- Capture resolution selector (640×480 / 1280×720 / 1456×1088)
- Rotation (0° / 90° / 180° / 270°)
- Colour mode selector
- Exposure and gain sliders (auto / manual)
- ArUco detection toggle + dictionary selector
- Lens calibration toggle (applies `camera_cal.npz` if present)
- Snapshot button — saves to `~/Pictures/HackyRacingRobot/`

Open `http://<pi-ip>:8080/` in a browser.

---

## Library modules

### `drivers/` package — hardware drivers

Generic hardware drivers with no robot-specific logic.

#### drivers/ibus.py

FlySky iBUS receiver library.  Reads 32-byte iBUS packets from a UART and exposes
14 RC channel values (1000–2000 µs range).

```python
from drivers.ibus import IBusReader
with IBusReader('/dev/ttyAMA3') as ibus:
    channels = ibus.read()   # blocking; returns list[int] or None on timeout
```

> **Note:** `drivers/ibus.py` is used by `rc_drive.py` only. The main robot stack (`robot_daemon.py` and all frontends) no longer reads iBUS directly — RC input is handled by the Yukon RP2040 via GP26 PIO UART and queried over USB serial with `CMD_RC_QUERY`. The `uart3-pi5` device tree overlay is not required for the main stack.

---

#### drivers/ld06.py

LD06 LiDAR driver.  Parses 47-byte packets from the LD06 over UART and maintains
a `LidarScan` dataclass with the latest full scan.

```python
from drivers.ld06 import LD06
lidar = LD06('/dev/ttyAMA0')
lidar.start()
scan = lidar.get_scan()   # LidarScan(angles, distances, rpm)
lidar.stop()
```

Hardware: GPIO 15 (RX) ← LD06 Tx @ 230400 8N1 via `uart0-pi5` overlay.
GPIO 12 (PWM) → LD06 PWM control @ 30 kHz, 40% duty ≈ 10 Hz scan rate.

---

### `robot/` package — higher-level logic

Robot-specific modules: vision, navigation, and autonomous control.

#### robot/camera_controls.py

Shared constants, helpers, and utilities used by both `camera_monitor.py` and `camera_web.py`.

| Item | Description |
|------|-------------|
| `CAPTURE_SIZES` | List of supported resolutions: 640×480, 1280×720, 1456×1088 |
| `GAINS` | Analogue gain steps: 1×, 2×, 4×, 8×, 16× |
| `EXP_STEPS` | Exposure time steps in µs (500 – 66000) |
| `EXP_DEFAULT` | Default exposure index (index 4 = 8000 µs) |
| `ARUCO_DICTS` | Supported ArUco dictionary names |
| `IMAGE_DIR` | Snapshot output path (from `robot.ini [output]` or `~/Pictures/HackyRacingRobot`) |
| `CALIB_FILE` | Default calibration file path (`camera_cal.npz` in repo root) |
| `sharpness(gray)` | Laplacian variance sharpness score (resolution-independent) |
| `rotate(frame, degrees)` | `np.rot90` wrapper for 90° increments |
| `make_cam(w, h)` | Create and start a `Picamera2` instance at the given resolution |
| `draw_aruco_on_frame(frame, state)` | Draw tag boxes and gate lines onto an RGB numpy frame in-place |
| `CalibrationMaps(calib_file)` | Loads `camera_cal.npz`; `.available` property; `.get_maps(w, h)` returns lazily-built undistortion maps |

---

#### robot/aruco_detector.py

OpenCV ArUco marker detector.  Detects markers in RGB camera frames, identifies
gate pairs, and optionally estimates metric distance and bearing via `cv2.solvePnP`.

**Gate and tag numbering convention:**
- Gate N has two posts: **outside** post (tag ID `2N`, even) and **inside** post (tag ID `2N+1`, odd)
- **Front face** tags: IDs 0–99 (robot approaching)
- **Rear face** tags: IDs 100–199 (rear = front + 100; seeing a rear tag means the gate has already been passed)
- Single-tag navigation: aim **right** of an outside (even) tag, **left** of an inside (odd) tag

Pose estimation requires a calibration file (`camera_cal.npz`) produced by
`tools/calibrate_camera.py` at the same resolution the camera runs at.

```python
from robot.aruco_detector import ArucoDetector
det   = ArucoDetector(calib_file='camera_cal_640x480.npz', tag_size=0.15)
state = det.detect(frame)   # annotates frame in-place; returns ArUcoState
for gate in state.gates.values():
    print(gate.gate_id, gate.distance, gate.bearing)
```

---

#### robot/aruco_navigator.py

Autonomous gate navigator.  State machine:
`IDLE → SEARCHING → ALIGNING → APPROACHING → PASSING → COMPLETE`
with `RECOVERING` when a gate is lost mid-approach.

- `SEARCHING` — rotates in IMU-controlled steps to find the next gate
- `ALIGNING` — steers toward the gate centre bearing
- `APPROACHING` — drives forward while holding alignment; LiDAR obstacle-stop active
- `PASSING` — straight-line burst through the gate at locked IMU heading (`pass_distance` trigger); LiDAR obstacle-stop active
- `RECOVERING` — gate lost while approaching; reverses briefly then returns to SEARCHING

When only one post of a gate is visible, the navigator aims offset left/right of the visible tag (see gate numbering convention above). The aim offset scales with apparent tag pixel area so it stays consistent as the robot approaches.

Diagnostic attributes (readable on the navigator instance):

| Attribute | Type | Description |
|-----------|------|-------------|
| `target_bearing` | `float \| None` | Camera-relative bearing to the aim point (degrees) |
| `tag_dist` | `float \| None` | Estimated metric distance to the target tag |
| `tags_visible` | `int` | Number of ArUco tags visible in the current frame |
| `bearing_err` | `float \| None` | Signed bearing error (positive = target right of centre) |

These are exposed in `RobotState` as `nav_target_bearing`, `nav_target_dist`, and `nav_tags_visible`.

Requires ArUco detection + optionally the BNO085 IMU (via Yukon telemetry) for
heading hold between camera frames.

All parameters are read from `[navigator]` in `robot.ini`.

---

#### robot/gps_navigator.py

GPS waypoint navigator.  Loads waypoints from a versioned JSON file
(`{"version": 1, "waypoints": [...]}`) and navigates through them in
sequence using GPS bearing and IMU heading hold between fixes.

- GPS fix (≤10 Hz) sets the IMU target heading via Haversine bearing
- 50 Hz control loop steers by IMU heading error between GPS updates
- Waypoint arrival detected by distance threshold (`arrival_radius`)
- Look-ahead smoothing: starts blending toward the next waypoint's bearing when within `lookahead_m` metres, reducing overshoot at transitions
- LiDAR obstacle-stop: halts when any point in the forward cone is closer than `obstacle_stop_dist` metres

All parameters are read from `[gps_navigator]` in `robot.ini`.

---

## Configuration

All runtime settings live in `robot.ini` in the repo root.  Every value has a
comment in the file.  Key sections:

| Section | Controls |
|---------|----------|
| `[robot]` | Yukon serial port, iBUS port (iBUS port only used by `rc_drive.py`) |
| `[rc]` | Channel mapping, deadzone, failsafe, control rate |
| `[camera]` | Legacy single-camera settings (backward compat fallback) |
| `[camera_front_left]` | IMX296 CSI CAM0 — resolution, fps, rotation (180°), ArUco, calibration file |
| `[camera_front_right]` | IMX296 CSI CAM1 — resolution, fps, rotation (180°), ArUco, calibration file |
| `[camera_rear]` | IMX477 USB/UVC — resolution, fps, rotation (0°), mirror, ArUco, calibration file |
| `[stereo]` | Hardware XVS sync GPIO for front stereo pair |
| `[aruco]` | Detection dict, calibration file template, tag size |
| `[camera_calibrations]` | Target resolutions for `derive_calibrations.py` |
| `[navigator]` | ArUco gate navigator tuning (obstacle stop, recovery, search) |
| `[gps_navigator]` | GPS waypoint navigator tuning (lookahead, obstacle stop, arrival) |
| `[lidar]` | LiDAR port |
| `[gps]` | GPS port, log directory, log rate |
| `[ntrip]` | RTK correction caster credentials |
| `[output]` | Output directories, `max_recording_minutes` |
| `[gui]` | Pygame GUI fps, display mode, layout presets |
| `[layout_presets]` | Named 2×2 panel grid layouts for `robot_dashboard.py` |
| `[dashboard]` | Unified web dashboard host / port |
| `[battery]` | Battery chemistry and cell count for voltage thresholds |
| `[depth]` | Depth mapping: enable, mode (stereo/mono/fusion), Hailo model path, BM params |
| `[imu]` | BNO085 rotation vector mode (game / absolute) |
| `[gpio]` | Physical buttons (GPIO 17/27) and status LEDs (GPIO 22/23/25) |
