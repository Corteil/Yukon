# System Architecture

## Component diagram

```
FlySky TX ──iBUS──► RC Receiver ──► Yukon GP26 (PIO UART)
                                         │
                                    main.py (iBUS poll)
                                    Core 0: decodes packets
                                    Core 1: applies motor speeds
                                    CMD_RC_QUERY / CMD_MODE
                                         │ (USB serial)
                 ┌───────────────────────▼────┐
                 │ robot_daemon.py             │  ← main Pi-side daemon
                 │   Robot class               │
                 └──┬──┬──┬──┬───────────────┘
                    │  │  │  │
          ┌─────────┘  │  │  └──────────────┐
          │            │  │                 │
    _YukonLink      _Camera ×3          _Lidar           _Gps
   (USB serial)   (picamera2/OpenCV)  (LD06 UART)    (GNSS UART)
   /dev/ttyACM0   aruco_detector       /dev/ttyAMA0   /dev/ttyUSB0
          │            │                  │                │
    Yukon RP2040   front_left/right    LD06 LiDAR    TAU1308 RTK
    main.py        rear (OpenCV)       GPIO12 PWM    NTRIP client
   SLOT4: DualOutputModule (out0=FPV camera)   ArUco detection
   SLOT2/SLOT5: DualMotorModule
   SLOT3: LEDStripModule
```

Consumers of robot state:
- `robot_dashboard.py` — Unified Flask web dashboard, port 5000 (2×2 configurable panel grid on desktop/touchscreen; tab view on mobile ≤700 px)
- `rc_drive.py` — lightweight RC-only driver (no GUI, direct Yukon link)

---

## Key files and their roles

| File | Role |
|------|------|
| `yukon_firmware_and_software/main.py` | MicroPython firmware on the Yukon RP2040. Core 1 drives motors; Core 0 parses serial commands and monitors health |
| `robot_daemon.py` | Pi-side daemon. Owns all subsystem threads; exposes `get_state()`, `get_frame(cam)`, `get_aruco_state(cam)` |
| `robot_dashboard.py` | Unified Flask web dashboard (port 5000) — 2×2 configurable panel grid on desktop/touchscreen; tab view on mobile |
| `camera_monitor.py` | Standalone pygame camera tool — resolution cycling, ArUco overlay, sharpness, lens calibration toggle, spacebar capture |
| `camera_web.py` | Standalone mobile Flask camera tool with MJPEG stream, ArUco toggle, calibration toggle, capture (port 8080) |
| `rc_drive.py` | Minimal RC-to-motor bridge without full robot stack |
| `drivers/ibus.py` | FlySky iBUS reader library (32-byte packets, 14 channels) |
| `drivers/ld06.py` | LD06 LiDAR driver (47-byte packets, CRC8, `LidarScan` dataclass) |
| `robot/aruco_detector.py` | OpenCV ArUco wrapper (corners, ID, distance, bearing) |
| `robot/camera_controls.py` | Shared camera constants, helpers, and `CalibrationMaps` class used by `camera_monitor.py` and `camera_web.py` |
| `robot/aruco_navigator.py` | Autonomous gate navigator — state machine (SEARCHING → ALIGNING → APPROACHING → PASSING → COMPLETE, RECOVERING on gate loss) with IMU heading hold |
| `robot/gps_navigator.py` | GPS waypoint navigator |
| `gnss/` | GNSS driver package (TAU1308, UBlox7, UBloxM8P, NTRIP client) |
| `robot.ini` | Runtime configuration for all subsystems |
| `tools/upload.py` | MicroPython uploader (handles Yukon double-USB-reset) |
| `tools/yukon_sim.py` | PTY-based Yukon serial simulator — `--mode gui\|web\|headless` (default: gui); web UI at port 5002 |
| `tools/read_data_log.py` | Flask JSONL data-log viewer at port 5004 — Drive, Telemetry, GPS map, LiDAR, Inspector tabs |
| `tools/calibrate_camera.py` | Interactive calibration tool — zone guidance, auto-capture, mirror mode; saves `camera_cal.npz` at 1456×1088 |
| `tools/derive_calibrations.py` | Scales a master calibration to any set of target resolutions; writes per-resolution `.npz` files |
| `tools/generate_aruco_tags.py` | CLI tool to generate ArUco tag PDFs (custom IDs, paper size, dictionary) |
| `tools/make_checkerboard_pdf.py` | Generates printable checkerboard calibration target PDF |
| `tools/nav_visualiser.py` | Real-time overhead navigation view — `--live`, `--sim`, or `--udp` modes |
| `yukon_firmware_and_software/i2c_scan.py` | I2C bus scanner for the Yukon Qw/ST port |
| `tools/test_gnss.py` | 57 unit tests for the `gnss/` package — no hardware required |
| `tools/test_*.py` | Unit tests and live-display tools for iBUS, LiDAR, protocol, GPS, BNO085, ArUco, robot integration |

---

## Data flow between subsystems

```
RC receiver ──► Yukon GP26 (PIO UART) ──► main.py iBUS poll (Core 0)
                                               │  MANUAL: apply motors directly (Core 1)
                                               │  CMD_RC_QUERY response (10 Hz)
                                               ▼
                              _control_thread (50 Hz) — queries RC channels,
                              sends CMD_MODE heartbeat, runs navigator in AUTO
                                     │
                              _YukonLink.drive(left, right)   [AUTO only]
                                     │
                              5-byte serial packet ──► Yukon RP2040
                                                        Core1: applies speeds
                                                        Core0: ACK/NAK + sensors

_Camera._run() × 3  (front_left / front_right / rear)
    front_left / front_right: picamera2 CSI (IMX296, 180° rotation)
    rear: OpenCV UVC (IMX477, rotation=0, mirror=true)
    ──► capture_array() ──► BGR→RGB ──► rotate ──► ArUco detect
    ──► RobotState.cam_front_left_ok / cam_front_right_ok / cam_rear_ok,
        latest frame and aruco_state per camera

_Lidar._run() ──► LD06 packets ──► LidarScan(angles, distances)
               ──► RobotState.lidar

_Gps._run() ──► NMEA parse ──► GpsState(lat, lon, fix_quality, …)
             ◄── NTRIP RTCM ──► serial inject
             ──► optional CSV log via _gps_log_thread

_System._run() ──► psutil CPU/mem/disk ──► SystemState
```

---

## State dataclasses

`RobotState` is the top-level snapshot passed to GUIs:

```python
@dataclass
class RobotState:
    mode:           RobotMode       # MANUAL | AUTO | ESTOP
    auto_type:      AutoType        # CAMERA | GPS | CAMERA_GPS
    drive:          DriveState      # left/right motor speeds, throttle, steer
    telemetry:      Telemetry       # voltage, current, temps, faults
    gps:            GpsState        # lat, lon, fix_quality, hdop, satellites, …
    lidar:          LidarScan       # angles[], distances[], timestamp
    system:         SystemState     # CPU%, temp, mem, disk
    rc_active:      bool
    # Legacy single-camera flags (backward compat)
    camera_ok:      bool
    aruco_ok:       bool
    # Per-camera status (triple-camera setup)
    cam_front_left_ok:         bool
    cam_front_right_ok:        bool
    cam_rear_ok:               bool
    cam_front_left_recording:  bool
    cam_front_right_recording: bool
    cam_rear_recording:        bool
    gate_confirmed:            bool  # rear camera confirmed last gate passage
    current_gate_id:           int
    lidar_ok:       bool
    gps_ok:         bool
    gps_logging:    bool
    cam_recording:  bool            # True if any camera is recording
    data_logging:   bool
    no_motors:      bool            # drive commands suppressed (bench testing)
    speed_scale:    float           # current speed limit (0.0–1.0)
    nav_state:          str             # navigator state name (e.g. "SEARCHING", "PASSING")
    nav_gate:           int             # ArUco target gate index
    nav_wp:             int             # GPS target waypoint index
    nav_wp_dist:        Optional[float] # metres to current GPS waypoint
    nav_wp_bear:        Optional[float] # bearing to current GPS waypoint
    nav_bearing_err:    Optional[float] # ArUco navigator bearing error (degrees)
    nav_target_bearing: Optional[float] # camera-relative bearing to aim point (degrees)
    nav_target_dist:    Optional[float] # metric distance to target tag
    nav_tags_visible:   int             # number of ArUco tags visible in current frame
```

`SystemState` (Pi health, polled by `_System` thread):

```python
@dataclass
class SystemState:
    cpu_percent:   float   # 0–100 %
    cpu_temp_c:    float   # °C
    cpu_freq_mhz:  float
    mem_used_mb:   float
    mem_total_mb:  float
    mem_percent:   float
    disk_used_gb:  float
    disk_total_gb: float
    disk_percent:  float
    timestamp:     float
```

---

## Threading model

All subsystem threads are daemon threads (die when the main process exits).

| Thread name     | Owner              | Frequency   | Responsibility |
|-----------------|--------------------|-------------|----------------|
| `yukon_rx`      | `_YukonLink`       | event-driven| Read ACK/NAK/sensor packets from Yukon |
| `camera`        | `_Camera`          | 30 Hz (cfg) | Capture, rotate, ArUco detect, store latest frame |
| `ld06`          | `LD06`             | packet-driven | Parse LD06 LiDAR packets, update latest scan |
| `gps`           | `_Gps`             | NMEA rate   | Parse NMEA sentences, inject RTCM |
| `gps_log`       | `Robot`            | 5 Hz (cfg)  | Write GPS CSV log when enabled |
| `telemetry`     | `Robot`            | 1 Hz        | Request sensor data from Yukon, update Telemetry |
| `control`       | `Robot`            | 50 Hz (cfg) | Query RC channels via CMD_RC_QUERY (10 Hz), send CMD_MODE heartbeat, run navigator in AUTO, send drive commands |
| `system`        | `_System`          | 1 Hz        | Poll CPU/mem/disk via psutil |
| `depth`         | `Robot`            | camera fps  | Compute stereo/mono/fusion depth map from latest frames (only when `[depth] enabled = true`) |

Shared state is protected by `threading.Lock()` per subsystem object. GUIs call `robot.get_state()` which returns a shallow copy of the latest `RobotState`.
