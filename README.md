# Hacky Racing Robot

Raspberry Pi–hosted controller for a Pimoroni Yukon robot. The Pi reads an RC transmitter via iBUS and sends motor commands to the Yukon RP2040 over USB serial using a compact 5-byte protocol. Supports autonomous gate navigation with ArUco markers, RTK GPS waypoint following, LiDAR, camera recording, ML sensor data logging, and a live pygame or web dashboard.

---

## Quick start

> **Run only ONE frontend at a time.** `robot_gui.py`, `robot_web.py`, and `robot_mobile.py` each open the same hardware ports — running two simultaneously causes port conflicts.

```bash
# Full robot stack with pygame GUI
python3 robot_gui.py

# Web dashboard (desktop, port 5000)
python3 robot_web.py

# Mobile web dashboard (port 5001)
python3 robot_mobile.py

# RC drive only (no GUI)
python3 rc_drive.py

# Standalone LiDAR visualiser
python3 lidar_gui.py

# Camera monitor with ArUco overlay and calibration (pygame)
python3 camera_monitor.py

# Camera web interface (mobile, port 8080)
python3 camera_web.py
```

---

## Hardware

| Component | Detail |
|-----------|--------|
| Pimoroni Yukon | RP2040-based motor controller |
| LED strip | `LEDStripModule` (NeoPixel, 8 LEDs) in SLOT3 |
| Left motors | `DualMotorModule` in SLOT2 |
| Right motors | `DualMotorModule` in SLOT5 |
| RC receiver | FlySky iBUS on GPIO 9 / `/dev/ttyAMA3` |
| Host ↔ Yukon | USB serial `/dev/ttyACM0` at 115200 baud |
| Camera | IMX296 global shutter via picamera2 |
| LiDAR | LD06 on `/dev/ttyAMA0` |
| GNSS | Allystar TAU1308 RTK receiver |
| IMU | BNO085 on Yukon I2C (Qw/ST port) |

---

## Project layout

```
robot_daemon.py             Pi-side daemon (all subsystem threads)
robot_utils.py              Shared utilities (config helper, local IP)
robot_gui.py                Pygame monitor with drive/telem/GPS/camera/lidar/log panels
robot_web.py                Flask web dashboard with terminal log panel (port 5000)
robot_mobile.py             Mobile Flask dashboard (port 5001)
rc_drive.py                 Minimal RC-to-motor bridge (no GUI)
lidar_gui.py                Standalone LD06 LiDAR visualiser
camera_monitor.py           Pygame camera monitor (ArUco, sharpness, calibration)
camera_web.py               Mobile camera web interface (port 8080)
robot.ini                   Runtime configuration
drivers/
  ibus.py                   FlySky iBUS receiver library
  ld06.py                   LD06 LiDAR driver
robot/
  camera_controls.py        Shared camera constants, helpers, CalibrationMaps
  aruco_detector.py         OpenCV ArUco marker detector
  aruco_navigator.py        Autonomous gate navigator (ArUco + IMU)
  gps_navigator.py          GPS waypoint navigator
gnss/                       GNSS driver package (TAU1308, UBlox, NTRIP)
yukon_firmware_and_software/
  main.py                   MicroPython firmware for the Yukon RP2040
  i2c_scan.py               I2C bus scanner (run on Yukon via Thonny)
tools/
  upload.py                 MicroPython uploader (handles Yukon USB reset)
  calibrate_camera.py       Lens calibration tool (outputs camera_cal.npz)
  generate_aruco_tags.py    Generate ArUco tag PDFs
  yukon_sim.py              PTY-based Yukon serial simulator
  read_data_log.py          Web viewer for JSONL data logs (port 5004)
  test_*.py                 Unit tests and live-display tools
docs/
  ARCHITECTURE.md           Component diagram, data flow, threading model
  PROTOCOL.md               5-byte serial protocol specification
  SETUP.md                  Wiring, overlays, dependencies, upload guide
```

---

## Output directories

Configured via `[output]` in `robot.ini` — all default to the Pi user's home folders.

| Output | Default path |
|--------|-------------|
| Camera snapshots | `~/Pictures/HackyRacingRobot/` |
| Video recordings | `~/Videos/HackyRacingRobot/` |
| ML data logs (JSONL) | `~/Documents/HackyRacingRobot/` |

Set `max_recording_minutes` in `[output]` to roll video to a new file automatically (0 = unlimited).

---

## Documentation

- [ARCHITECTURE.md](docs/ARCHITECTURE.md) — Component diagram, key files, data flow, threading model
- [ROBOT_FILES.md](docs/ROBOT_FILES.md) — All Pi-side Python files: usage, keys, flags, API
- [PROTOCOL.md](docs/PROTOCOL.md) — 5-byte serial protocol specification
- [SETUP.md](docs/SETUP.md) — Wiring, device tree overlays, dependencies, firmware upload
- [tools/README.md](tools/README.md) — All tools and tests reference

---

## Firmware upload

```bash
python3 tools/upload.py yukon_firmware_and_software/main.py
```

Use `tools/upload.py` instead of `mpremote` or `ampy` — `yukon.reset()` drops the USB connection on every Ctrl+C, which breaks standard uploaders.
