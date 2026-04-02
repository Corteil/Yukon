# Hacky Racing Robot

Raspberry Pi–hosted controller for a Pimoroni Yukon robot. The Yukon RP2040 reads the RC transmitter directly via iBUS on GP26 (PIO UART) and drives the motors in MANUAL mode without Pi involvement. The Pi communicates with the Yukon over USB serial using a compact 5-byte protocol, handles autonomous navigation (ArUco gate navigation, RTK GPS waypoints, LiDAR), and serves a unified web dashboard.

---

## Quick start

```bash
# Unified web dashboard (port 5000) — desktop, mobile, and touchscreen
python3 robot_dashboard.py

# RC drive only (no GUI)
python3 rc_drive.py

# Standalone LiDAR visualiser
python3 lidar_gui.py

# Standalone camera tools
python3 camera_monitor.py    # pygame, ArUco overlay + calibration
python3 camera_web.py        # web interface at :8080
```

Open `http://<pi-ip>:5000/` in any browser. On desktop/touchscreen the UI shows a configurable 2×2 panel grid; on mobile (≤700px) it switches to a tab view.

---

## Hardware

| Component | Detail |
|-----------|--------|
| Pimoroni Yukon | RP2040-based motor controller |
| Left motors | `DualMotorModule` in SLOT2 |
| LED strip | `LEDStripModule` (NeoPixel, 8 LEDs) in SLOT3 |
| Bench power | `BenchPowerModule` (5 V regulated output) in SLOT4 |
| Right motors | `DualMotorModule` in SLOT5 |
| RC receiver | FlySky iBUS → Yukon GP26 (PIO UART, decoded by firmware) |
| Host ↔ Yukon | USB serial `/dev/ttyACM0` at 115200 baud |
| Front cameras | IMX296 global shutter (×2) via picamera2 CSI (180° rotation — mounted inverted) |
| Rear camera | IMX477 HQ camera via USB/UVC (OpenCV, mirror=true) |
| LiDAR | LD06 on `/dev/ttyAMA0`; PWM motor drive on GPIO 12 |
| GNSS | Allystar TAU1308 RTK receiver on `/dev/ttyUSB0` |
| IMU | BNO085 on Yukon I2C (Qw/ST port) |
| GPIO buttons | ESTOP (GPIO 17) / START (GPIO 27) |
| GPIO LEDs | Status LEDs on GPIO 22, 23, 25 |

---

## Project layout

```
robot_daemon.py             Pi-side daemon (all subsystem threads)
robot_utils.py              Shared utilities (config helper, local IP)
robot_dashboard.py          Unified web dashboard — 2×2 configurable grid + mobile tab view (port 5000)
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
  stereo_depth.py           OpenCV stereo depth (BM algorithm)
  depth_estimator.py        Hailo-8L monocular depth (scdepthv3, optional)
gnss/                       GNSS driver package (TAU1308, UBlox, NTRIP)
yukon_firmware_and_software/
  main.py                   MicroPython firmware for the Yukon RP2040
  i2c_scan.py               I2C bus scanner (run on Yukon via Thonny)
tools/
  upload.py                 MicroPython uploader (handles Yukon USB reset)
  calibrate_camera.py       Lens calibration tool (outputs camera_cal.npz)
  derive_calibrations.py    Scale master calibration to all target resolutions
  generate_aruco_tags.py    Generate ArUco tag PDFs
  ibus_sim.py               Interactive iBUS RC receiver simulator (PTY)
  pygame_gamepad_ibus_rx.py Gamepad → iBUS PTY simulator (physical joystick)
  depth_viewer.py           Standalone depth map visualiser (connects to :5000)
  setup_depth_model.py      Download scdepthv3 HEF model for Hailo-8L
  nav_visualiser.py         Real-time overhead navigation view
  yukon_sim.py              PTY-based Yukon serial simulator (headless/GUI/web)
  gps_route_builder.py      Pygame GPS waypoint editor
  gps_route_builder_web.py  Web GPS waypoint editor (port 5003)
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
- [CALIBRATION.md](docs/CALIBRATION.md) — Camera lens calibration workflow
- [GPIO.md](docs/GPIO.md) — GPIO pin allocation reference
- [tools/README.md](tools/README.md) — All tools and tests reference

---

## Firmware upload

Use **Thonny** to upload `yukon_firmware_and_software/main.py` to the Yukon RP2040:

1. Open `yukon_firmware_and_software/main.py` in Thonny
2. **File → Save as… → MicroPython device** → save as `main.py`
3. Press **Stop / Restart** to reboot the Yukon with the new firmware
