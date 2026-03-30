# HackyRacingRobot — AI Assistant Guide

Autonomous racing robot on Raspberry Pi 5 with a Pimoroni Yukon RP2040 motor controller, FlySky RC receiver, IMX296 camera, LD06 LiDAR, and Allystar TAU1308 RTK-GNSS.

---

## Repository Layout

```
HackyRacingRobot/
├── robot_daemon.py          # Central Robot class; all subsystems live here
├── robot_web.py             # Flask dashboard at :5000
├── robot_mobile.py          # Mobile Flask dashboard at :5001
├── robot_gui.py             # Pygame desktop GUI
├── rc_drive.py              # Minimal iBUS→motor bridge (no GUI/camera)
├── camera_monitor.py        # Standalone Pygame camera tool
├── camera_web.py            # Standalone web camera tool at :8080
├── lidar_gui.py             # Standalone Pygame LiDAR visualizer
├── robot_utils.py           # _cfg() config helper; _local_ip()
├── robot.ini                # ALL runtime configuration (every param has a comment)
├── requirements.txt         # pip dependencies
│
├── drivers/                 # Generic hardware drivers
│   ├── ibus.py              # FlySky iBUS RC protocol
│   └── ld06.py              # LDROBOT LD06 LiDAR
│
├── robot/                   # Robot-specific logic
│   ├── camera_controls.py   # Shared camera constants, helpers, CalibrationMaps
│   ├── aruco_detector.py    # OpenCV ArUco marker detection
│   ├── aruco_navigator.py   # State-machine gate navigator
│   └── gps_navigator.py     # GPS waypoint navigator
│
├── gnss/                    # Multi-driver GNSS library
│   ├── base.py              # GNSSBase — shared NMEA parsing
│   ├── ntrip.py             # NTRIP v1/v2 RTK corrections client
│   ├── rtcm_serial.py       # Serial RTCM corrections
│   ├── tau1308.py           # Allystar TAU1308 driver
│   ├── ublox7.py            # u-blox 7 driver
│   └── ubloxm8p.py          # u-blox NEO-M8P RTK rover driver
│
├── yukon_firmware_and_software/
│   └── main.py              # MicroPython firmware for Yukon RP2040
│
├── tools/                   # Tests, simulators, calibration
│   ├── test_*.py            # Per-subsystem test scripts
│   ├── yukon_sim*.py        # PTY-based Yukon hardware simulator
│   ├── calibrate_camera.py  # Interactive checkerboard calibration
│   ├── gps_route_builder*.py# Waypoint route editors
│   └── README.md            # Tools reference
│
└── docs/
    ├── ARCHITECTURE.md      # Component diagram and threading model
    ├── PROTOCOL.md          # 5-byte Yukon serial protocol spec
    ├── ROBOT_FILES.md       # Pi-side Python API reference
    └── SETUP.md             # Wiring, device tree, dependencies
```

---

## Core Architecture

### `Robot` class (`robot_daemon.py`)

The `Robot` class is the **sole backend**. All three frontends (GUI, web, mobile) import and instantiate it identically.

**Thread-per-subsystem model:**

| Thread | Rate | Purpose |
|--------|------|---------|
| `yukon_rx` | event-driven | ACK/NAK and sensor packet reader |
| `camera` | 30 Hz (cfg) | Capture + optional ArUco detection |
| `ld06` | event-driven | LD06 packet → LidarScan |
| `gps` | NMEA events | NMEA parsing + RTCM/NTRIP injection |
| `telemetry` | 1 Hz | Poll Yukon voltage/temp/IMU |
| `control` | 50 Hz | Query RC via CMD_RC_QUERY (10 Hz), send CMD_MODE heartbeat, run navigator→motor in AUTO |
| `system` | 1 Hz | CPU/mem/disk metrics |

**Thread safety rules:**
- `_lock` protects camera frame, GPS state, and LiDAR scan.
- `_cmd_lock` serializes Yukon send+drain pairs — never interleave commands.
- `robot.get_state()` returns an immutable `RobotState` dataclass snapshot — safe to read from any thread.
- Queues (not locks) carry ACK/NAK and sensor data to avoid priority inversion.

**Public API:**
```python
robot = Robot(config_dict)
robot.start()                  # Connect hardware, launch threads
state  = robot.get_state()     # RobotState snapshot
frame  = robot.get_frame()     # Latest RGB numpy array or None
aruco  = robot.get_aruco_state()
hdg    = robot.get_heading()   # IMU heading degrees or None
robot.drive(left, right)       # float -1.0…+1.0 (AUTO mode only)
robot.start_cam_recording()
robot.start_data_log()
robot.stop()
```

### State dataclasses (`robot_daemon.py`)

```
RobotState          # Full snapshot returned by get_state()
  ├── mode          # RobotMode: MANUAL | AUTO | ESTOP
  ├── drive         # DriveState: left/right -1.0…+1.0
  ├── telemetry     # Voltage, current, temps, faults, IMU heading
  ├── gps           # GpsState: lat/lon, fix, HDOP, h_error
  ├── lidar         # LidarScan: angle/distance arrays
  ├── system        # SystemState: CPU%, temp, mem%, disk%
  └── nav           # Navigator target/status string
```

---

## Yukon Serial Protocol

5-byte command frames over USB serial `/dev/ttyACM0` @ 115200 baud:

```
[0x7E, CMD, V_HIGH, V_LOW, CHK]   CHK = SYNC ^ CMD ^ V_HIGH ^ V_LOW
```

| CMD byte | Name | Purpose |
|----------|------|---------|
| 0x21 | CMD_LED | Set LED colour |
| 0x22 | CMD_LEFT | Left motor speed (encoded 0–200) |
| 0x23 | CMD_RIGHT | Right motor speed |
| 0x24 | CMD_KILL | Zero both motors |
| 0x25 | CMD_SENSOR | Request telemetry packet |
| 0x26 | CMD_BEARING | IMU heading hold (0–254 → 0–359°) |
| 0x27 | CMD_STRIP | Fill all LEDs with colour preset |
| 0x28 | CMD_PIXEL_SET | Stage one pixel colour (no hardware update) |
| 0x29 | CMD_PIXEL_SHOW | Push staged pixel data to strip |
| 0x2A | CMD_PATTERN | Start autonomous LED animation |

Responses: `ACK (0x06)` / `NAK (0x15)`. Sensor data comes as a sequence of 5-byte `(RESP_TYPE, V_HIGH, V_LOW, CHK)` packets followed by ACK. See `docs/PROTOCOL.md` for full encoding.

---

## Configuration (`robot.ini`)

Every runtime parameter lives in `robot.ini`. The file is extensively commented — **read it before changing hardware assignments or tuning values**.

Key sections:

| Section | What it controls |
|---------|-----------------|
| `[robot]` | Yukon port, iBUS port (iBUS port only used by `rc_drive.py`) |
| `[rc]` | Channel map, deadzone, failsafe, 50 Hz control rate |
| `[camera]` | Resolution, fps, rotation |
| `[aruco]` | Dictionary, calibration file, tag size |
| `[camera_calibrations]` | Target resolutions for `derive_calibrations.py` |
| `[leds]` | LED preset per mode (manual/auto/estop) |
| `[lidar]` | LD06 port |
| `[gps]` | TAU1308 port, log dir, log rate |
| `[ntrip]` | NTRIP caster host/port/mount/credentials |
| `[rtcm]` | Serial RTCM correction input (alternative to NTRIP) |
| `[navigator]` | ArUco gate navigator PID tuning |
| `[gps_navigator]` | GPS waypoint navigator tuning |
| `[output]` | Snapshot/video/data-log directories, recording limits |
| `[gui]` | Pygame GUI fps |
| `[web]` | Web dashboard host/port |
| `[mobile]` | Mobile dashboard host/port |

Every value can be overridden via CLI flag — run `python3 robot_daemon.py --help`.

**Config helper** (`robot_utils.py`):
```python
from robot_utils import _cfg
value = _cfg(cfg, 'section', 'key', fallback, cast=int)
```

---

## Hardware Devices

| Device | Interface | Path | Notes |
|--------|-----------|------|-------|
| Yukon RP2040 | USB serial | `/dev/ttyACM0` | 115200 baud; 5-byte protocol |
| FlySky iBUS RX | Yukon GP26 (PIO UART) | — | 115200 baud; decoded by Yukon firmware; queried by Pi via CMD_RC_QUERY |
| LD06 LiDAR | UART0 RX (GPIO 15) | `/dev/ttyAMA0` | 230400 baud; GPIO 18 PWM @ 30 kHz |
| TAU1308 GNSS | USB serial | `/dev/ttyUSB0` | 115200 baud; NMEA output |
| IMX296 camera | CSI / picamera2 | — | Fallback to `/dev/video0` via OpenCV |
| BNO085 IMU | I2C on Yukon | — | Optional; heading via CMD_SENSOR response |

---

## Running the Robot

```bash
# Full daemon with Pygame GUI
python3 robot_gui.py

# Full daemon with desktop web dashboard
python3 robot_web.py

# Full daemon with mobile-optimised web dashboard
python3 robot_mobile.py

# Headless daemon only
python3 robot_daemon.py

# Minimal RC→motor bridge (no camera, no GPS)
python3 rc_drive.py

# Standalone camera tool (no Robot instance)
python3 camera_monitor.py
python3 camera_web.py

# Standalone LiDAR visualizer
python3 lidar_gui.py
```

**Logging:** `setup_logging()` writes to `logs/robot.log` (rotating, 5 MB × 3 files). Use Python `logging` — never `print()` in daemon code.

---

## Testing and Simulation

### Run tests without hardware

```bash
# Yukon protocol (no hardware needed)
python3 tools/test_main.py --dry-run

# LiDAR driver (unit mode)
python3 tools/test_ld06.py -u

# Robot integration test using PTY simulator
python3 tools/test_robot.py

# ArUco detection (synthetic frames)
python3 tools/test_aruco.py
```

### Yukon simulator

Provides a virtual `/dev/ttyACM0` via PTY — useful when the Yukon board is unavailable:
```bash
python3 tools/yukon_sim.py           # headless
python3 tools/yukon_sim_gui.py       # with Pygame controls
python3 tools/yukon_sim_web.py       # web UI at :5002
```

### Camera calibration

```bash
python3 tools/calibrate_camera.py    # produces camera_cal.npz
python3 tools/derive_calibrations.py # generate per-resolution files
```

---

## Code Conventions

### Error handling
- Subsystems fail independently — a missing camera or GPS must never crash the daemon.
- Use `try/except` with specific exception types; log with `log.warning()` or `log.error()`.
- Retry loops use `time.sleep(3)` intervals; always check a `_stop` event flag to allow clean shutdown.

### Logging
```python
import logging
log = logging.getLogger(__name__)
log.info("Started")
log.warning("GPS not available")
log.error("Yukon connection failed: %s", e)
```
Never use `print()` in daemon or driver code. `print()` is acceptable only in standalone tools.

### Thread safety
- Acquire `_lock` for the shortest time possible — copy the value out, release, then use it.
- Never call `robot.drive()` from a web handler directly; queue it or use the control thread.
- Use `threading.Event` for clean shutdown signalling (`_stop.set()`).

### Imports and packages
- `drivers/` — generic protocol drivers, no robot logic.
- `robot/` — robot-specific algorithms (navigation, detection).
- `gnss/` — GNSS drivers; extend `GNSSBase` for new receivers.
- New top-level scripts should instantiate `Robot` and delegate; don't duplicate subsystem logic.

### Configuration changes
- Add new parameters to `robot.ini` with a comment explaining units and defaults.
- Add the corresponding `_cfg()` call with a sensible fallback so old config files keep working.
- Add a `--new-param` CLI flag in the `argparse` block.

### MicroPython firmware (`yukon_firmware_and_software/main.py`)
- Runs on Yukon RP2040; edit carefully — serial protocol changes need matching updates in `robot_daemon.py` and `docs/PROTOCOL.md`.
- Upload via `python3 tools/upload.py`.

---

## Key Files Quick Reference

| Task | File |
|------|------|
| Change motor/RC behaviour | `robot_daemon.py` (`_YukonLink`, `_RcReader`) |
| Tune ArUco navigation | `robot/aruco_navigator.py` + `[navigator]` in robot.ini |
| Tune GPS navigation | `robot/gps_navigator.py` + `[gps_navigator]` in robot.ini |
| Add a GNSS driver | Subclass `gnss/base.py:GNSSBase` |
| Change web dashboard layout | `robot_web.py` (Jinja2 templates inline) |
| Change mobile dashboard | `robot_mobile.py` |
| Shared camera constants / helpers | `robot/camera_controls.py` |
| Protocol between Pi and Yukon | `docs/PROTOCOL.md` + `robot_daemon.py:_YukonLink` + `yukon_firmware_and_software/main.py` |
| Device wiring / GPIO assignments | `docs/SETUP.md` |

---

## Output Directories

| Content | Default path |
|---------|-------------|
| Camera snapshots | `~/Pictures/HackyRacingRobot/` |
| Video recordings | `~/Videos/HackyRacingRobot/` |
| ML data logs (JSONL) | `~/Documents/HackyRacingRobot/` |
| GPS logs (CSV) | `logs/` (configured in `[gps]`) |
| Robot log | `logs/robot.log` |

Video files: `recording_YYYYMMDD_HHMMSS.mp4` (timestamp burned into each frame).
Data logs: `data_YYYYMMDD_HHMMSS.jsonl` (one JSON object per line, 10 Hz).
