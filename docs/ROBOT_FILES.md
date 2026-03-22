# Robot Files Reference

Documentation for all Python files in the project root.
Run every script from the repo root unless noted otherwise.

---

## Full robot stack

### Common flags

All four robot scripts — `robot_daemon.py`, `robot_gui.py`, `robot_web.py`, `robot_mobile.py` —
accept the same core flags.  All are optional — config file values are used when a
flag is omitted.

| Flag | Default | Description |
|------|---------|-------------|
| `--config PATH` | `robot.ini` | Config file |
| `--yukon-port PORT` | from config | Yukon USB serial port |
| `--ibus-port PORT` | from config | iBUS UART device |
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
| `--no-motors` | off | Suppress all motor commands (bench-test mode) |

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
| `rc_reader` | ~143 Hz | Read iBUS packets, update RC channel state |
| `telemetry` | 1 Hz | Request sensor data from Yukon, update `Telemetry` |
| `control` | 50 Hz (configurable) | Compute motor speeds from RC / navigator, send to Yukon |
| `system` | 1 Hz | Poll CPU / mem / disk via psutil |

Key public API:

```python
robot = Robot(...)
robot.start()                 # connect hardware, launch all threads
state = robot.get_state()     # RobotState snapshot (thread-safe)
frame = robot.get_frame()     # latest camera frame (numpy RGB) or None
aruco = robot.get_aruco_state()  # latest ArUcoState or None
hdg   = robot.get_heading()   # IMU heading in degrees or None
robot.stop()                  # shutdown all subsystems cleanly
```

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

### robot_gui.py

4-panel pygame status display.  Reads `robot.get_state()` and `robot.get_frame()`
at the configured GUI fps (default 10 Hz).

**Panels**

| Panel | Contents |
|-------|----------|
| Drive | Left / right motor speed bars, throttle, steer, speed scale, RC status |
| Telemetry | Voltage, current, board + motor temperatures, fault indicators, IMU heading |
| GPS | Fix quality, position, horizontal error, HDOP, satellites |
| Camera | Live frame with optional ArUco / bearing overlay and IMU compass arc |
| LiDAR | Polar distance scan (distance-coloured points, range rings) |
| Footer | Subsystem health badges, mode badge, no-motors warning |

**Keyboard**

| Key | Action |
|-----|--------|
| `E` | ESTOP — kill motors immediately |
| `R` | Reset ESTOP → MANUAL mode |
| `[` / `]` | Rotate camera −90° / +90° |
| `T` | Toggle ArUco detection |
| `B` | Toggle bearing overlay on camera panel |
| `C` | Open / close config overlay (live edit of `robot.ini` values) |
| `Q` / `Esc` | Quit |

**Bearing overlay** (press `B`)
- Line from frame centre to each visible ArUco tag
- Distance label in metres (when calibrated) or pixel-area proxy
- Bearing angle in degrees from camera centre
- Cyan crosshair and aim line for the active gate target
- IMU compass arc at the bottom of the camera panel
- Navigator state and target gate label in AUTO·Camera mode

```bash
python3 robot_gui.py
python3 robot_gui.py --fps 15
python3 robot_gui.py --no-motors
```

Accepts all [common flags](#common-flags), plus:

| Flag | Default | Description |
|------|---------|-------------|
| `--fps N` | from config | GUI update rate |

---

### robot_web.py

Flask web dashboard (port 5000).  Same `Robot` backend as `robot_gui.py`.

**Features**
- Real-time telemetry via Server-Sent Events at 10 Hz
- Live camera as MJPEG stream with toggleable bearing overlay
- LiDAR polar plot on Canvas
- IMU heading display (compass rose + numeric)
- Navigator state and target gate badge
- ESTOP / Reset controls
- Camera rotation, ArUco toggle, bearing overlay toggle
- No-motors warning banner
- GPS logging badge
- Mobile-responsive layout

```bash
python3 robot_web.py                    # 0.0.0.0:5000
python3 robot_web.py --port 8080
python3 robot_web.py --no-motors
```

Accepts all [common flags](#common-flags), plus:

| Flag | Default | Description |
|------|---------|-------------|
| `--host HOST` | `0.0.0.0` | Bind address |
| `--port N` | `5000` | HTTP port |
| `--debug` | off | Enable Flask debug mode |

Open `http://<pi-ip>:5000/` in any browser on the same network.

---

### robot_mobile.py

Mobile-optimised Flask dashboard (port 5001).  Same `Robot` backend as
`robot_web.py`; touch-friendly tab UI designed for phones.

**Tabs**

| Tab | Contents |
|-----|----------|
| Drive | Camera stream, bearing overlay, motor bars, mode / speed, ArUco, nav state |
| Telem | Voltage, current, temperatures, IMU heading compass, faults |
| GPS | Fix quality, position, horizontal error, satellites, bookmark button |
| System | CPU, temperature, memory, disk, LiDAR polar plot |
| Logs | Live log viewer — colour-coded by level, filter bar, auto-scroll |

ESTOP button is always visible at the bottom of every tab.

```bash
python3 robot_mobile.py                 # 0.0.0.0:5001
python3 robot_mobile.py --port 8080
python3 robot_mobile.py --no-motors
```

Accepts all [common flags](#common-flags), plus:

| Flag | Default | Description |
|------|---------|-------------|
| `--host HOST` | `0.0.0.0` | Bind address |
| `--port N` | `5001` | HTTP port |

---

## Standalone tools

### rc_drive.py

Minimal RC-to-motor bridge.  No GUI, no camera, no navigator — just reads iBUS
and drives the Yukon motors.  Useful for driving tests without the full robot
stack, or as a fallback if `robot_daemon.py` will not start.

Tank mix: `left = clamp(throttle − aileron)`, `right = clamp(throttle + aileron)`

Failsafe: sends `CMD_KILL` if no valid iBUS packet is received for 0.5 s.

```bash
python3 rc_drive.py
python3 rc_drive.py --port /dev/ttyACM0 --ibus-port /dev/ttyAMA3
python3 rc_drive.py --reverse-left      # flip left motor direction
python3 rc_drive.py --reverse-right     # flip right motor direction
```

| Flag | Default | Description |
|------|---------|-------------|
| `--port PORT` | auto-detect | Yukon USB serial port |
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
| `Space` | Save frame to `saved_images/` |
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
- Snapshot button — saves to `saved_images/`

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

Hardware: GPIO 9 (RX only) → `/dev/ttyAMA3` via `uart3-pi5` device tree overlay.
See `docs/SETUP.md` for overlay configuration.

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
GPIO 18 (PWM) → LD06 PWM control @ 30 kHz, 40% duty ≈ 10 Hz scan rate.

---

### `robot/` package — higher-level logic

Robot-specific modules: vision, navigation, and autonomous control.

#### robot/aruco_detector.py

OpenCV ArUco marker detector.  Detects markers in RGB camera frames, identifies
gate pairs (consecutive odd/even tag IDs: tags 1+2 = gate 0, tags 3+4 = gate 1,
…), and optionally estimates metric distance and bearing via `cv2.solvePnP`.

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

- `SEARCHING` — rotates to find the next gate; uses IMU heading for stepped search
- `ALIGNING` — steers toward the gate centre bearing
- `APPROACHING` — drives forward while holding alignment
- `PASSING` — straight-line burst through the gate at locked IMU heading (requires pose estimation distance < `pass_distance`)

Requires ArUco detection + optionally the BNO085 IMU (via Yukon telemetry) for
heading hold between camera frames.

All parameters are read from `[navigator]` in `robot.ini`.

---

#### robot/gps_navigator.py

GPS waypoint navigator.  Loads waypoints from a JSON file and navigates through
them in sequence using GPS bearing and IMU heading hold between fixes.

- GPS fix (≤10 Hz) sets the IMU target heading via Haversine bearing
- 50 Hz control loop steers by IMU heading error between GPS updates
- Waypoint arrival detected by distance threshold (`arrival_radius`)

All parameters are read from `[gps_navigator]` in `robot.ini`.

---

## Configuration

All runtime settings live in `robot.ini` in the repo root.  Every value has a
comment in the file.  Key sections:

| Section | Controls |
|---------|----------|
| `[robot]` | Yukon serial port, iBUS port |
| `[rc]` | Channel mapping, deadzone, failsafe, control rate |
| `[camera]` | Resolution, fps, rotation |
| `[aruco]` | Detection dict, calibration file template, tag size |
| `[camera_calibrations]` | Target resolutions for `derive_calibrations.py` |
| `[navigator]` | ArUco gate navigator tuning |
| `[gps_navigator]` | GPS waypoint navigator tuning |
| `[lidar]` | LiDAR port |
| `[gps]` | GPS port, log directory, log rate |
| `[ntrip]` | RTK correction caster credentials |
| `[gui]` | Pygame GUI fps |
| `[web]` | Web dashboard host / port |
| `[mobile]` | Mobile dashboard host / port |
