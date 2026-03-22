# System Architecture

## Component diagram

```
FlySky TX ──iBUS──► RC Receiver (GPIO9 / /dev/ttyAMA3)
                         │
                    drivers/ibus.py (IBusReader)
                         │
                    Robot._rc_thread
                         │
                 ┌───────▼────────┐
                 │ robot_daemon.py│  ← main Pi-side daemon
                 │   Robot class  │
                 └──┬──┬──┬──┬───┘
                    │  │  │  │
          ┌─────────┘  │  │  └──────────────┐
          │            │  │                 │
    _YukonLink       _Camera            _Lidar           _Gps
   (USB serial)   (picamera2)         (LD06 UART)    (GNSS UART)
   /dev/ttyACM0   aruco_detector       /dev/ttyAMA0   /dev/ttyUSB0
          │            │                  │                │
    Yukon RP2040   IMX296 camera       LD06 LiDAR    TAU1308 RTK
    main.py        ArUco detection     GPIO18 PWM    NTRIP client
   SLOT2/SLOT5
   DualMotorModule
```

Consumers of robot state:
- `robot_gui.py` — 4-panel pygame dashboard (calls `robot.get_state()`, `robot.get_frame()`)
- `robot_web.py` — Flask web dashboard, port 5000 (same interface)
- `robot_mobile.py` — Mobile-optimised Flask dashboard, port 5001 (tab navigation: Drive, Telem, GPS, System, Logs)
- `rc_drive.py` — lightweight RC-only driver (no GUI, direct Yukon link)

---

## Key files and their roles

| File | Role |
|------|------|
| `yukon_firmware_and_software/main.py` | MicroPython firmware on the Yukon RP2040. Core 1 drives motors; Core 0 parses serial commands and monitors health |
| `robot_daemon.py` | Pi-side daemon. Owns all subsystem threads; exposes `get_state()`, `get_frame()`, `get_aruco_state()` |
| `robot_gui.py` | Pygame 4-panel monitor (Drive, Telemetry, GPS, Camera+LiDAR) |
| `robot_web.py` | Flask web dashboard with MJPEG camera stream (port 5000) |
| `robot_mobile.py` | Mobile Flask dashboard with tab navigation (port 5001); same Robot backend as robot_web.py |
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
| `tools/yukon_sim.py` | PTY-based Yukon serial simulator for offline testing (headless) |
| `tools/yukon_sim_gui.py` | Yukon simulator with Pygame UI — live motor/LED/fault display |
| `tools/yukon_sim_web.py` | Yukon simulator with web UI at port 5002 — fault injection, LED toggle |
| `tools/read_data_log.py` | Flask JSONL data-log viewer at port 5004 — Drive, Telemetry, GPS map, LiDAR, Inspector tabs |
| `tools/calibrate_camera.py` | Interactive calibration tool — zone guidance, auto-capture, mirror mode; saves `camera_cal.npz` at 1456×1088 |
| `tools/derive_calibrations.py` | Scales a master calibration to any set of target resolutions; writes per-resolution `.npz` files |
| `tools/generate_aruco_tags.py` | CLI tool to generate ArUco tag PDFs (custom IDs, paper size, dictionary) |
| `tools/make_checkerboard_pdf.py` | Generates printable checkerboard calibration target PDF |
| `yukon_firmware_and_software/i2c_scan.py` | I2C bus scanner for the Yukon Qw/ST port |
| `tools/test_gnss.py` | 57 unit tests for the `gnss/` package — no hardware required |
| `tools/test_*.py` | Unit tests and live-display tools for iBUS, LiDAR, protocol, GPS, BNO085, ArUco, robot integration |

---

## Data flow between subsystems

```
RC receiver ──► _rc_thread ──► RobotState.drive (throttle/steer)
                                     │
                              _control_thread (50 Hz)
                                     │
                              _YukonLink.drive(left, right)
                                     │
                              5-byte serial packet ──► Yukon RP2040
                                                        Core1: applies speeds
                                                        Core0: ACK/NAK + sensors

_Camera._run() ──► capture_array() ──► BGR→RGB ──► rotate ──► ArUco detect
                ──► RobotState.camera_ok, latest frame, aruco_state

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
    camera_ok:      bool
    lidar_ok:       bool
    gps_ok:         bool
    aruco_ok:       bool
    gps_logging:    bool
    cam_recording:  bool
    data_logging:   bool
    no_motors:      bool            # drive commands suppressed (bench testing)
    speed_scale:    float           # current speed limit (0.0–1.0)
    nav_state:      str             # navigator state name (e.g. "SEARCHING", "PASSING")
    nav_gate:       int             # ArUco target gate index
    nav_wp:         int             # GPS target waypoint index
    nav_wp_dist:    Optional[float] # metres to current GPS waypoint
    nav_wp_bear:    Optional[float] # bearing to current GPS waypoint
    nav_bearing_err: Optional[float] # ArUco navigator bearing error (degrees)
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
| `rc_reader`     | `Robot`            | ~143 Hz     | Read iBUS packets, update RC channel state |
| `telemetry`     | `Robot`            | 1 Hz        | Request sensor data from Yukon, update Telemetry |
| `control`       | `Robot`            | 50 Hz (cfg) | Compute motor speeds from RC, send to Yukon |
| `system`        | `_System`          | 1 Hz        | Poll CPU/mem/disk via psutil |

Shared state is protected by `threading.Lock()` per subsystem object. GUIs call `robot.get_state()` which returns a shallow copy of the latest `RobotState`.
