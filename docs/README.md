# HackyRacingRobot

Raspberry Pi–hosted controller for a Pimoroni Yukon robot. A Pi reads an RC transmitter via iBUS and sends motor commands to the Yukon over USB serial using a compact 5-byte protocol.

---

## Documentation

- [ARCHITECTURE.md](ARCHITECTURE.md) — Component diagram, key files, data flow, `SystemState` dataclass, threading model
- [ROBOT_FILES.md](ROBOT_FILES.md) — All Pi-side Python files: usage, keys, flags, Robot API reference
- [PROTOCOL.md](PROTOCOL.md) — 5-byte serial protocol specification, command table, motor encoding, sensor response format
- [SETUP.md](SETUP.md) — Hardware wiring, device tree overlays, udev rules, dependencies, upload instructions
- [CALIBRATION.md](CALIBRATION.md) — Camera lens calibration workflow
- [GPIO.md](GPIO.md) — GPIO pin allocation reference
- [allystar.pdf](allystar.pdf) — Allystar TAU1308 GNSS module datasheet
- [../tools/README.md](../tools/README.md) — Helper scripts (calibration, ArUco PDF, firmware upload, simulator, I2C scan)

---

## Quick start

```bash
# Unified web dashboard (port 5000) — desktop, touchscreen, and mobile
python3 robot_dashboard.py

# RC drive only (no GUI)
python3 rc_drive.py

# Standalone LiDAR visualiser
python3 lidar_gui.py

# Camera focus/ArUco monitor (pygame)
python3 camera_monitor.py

# Camera web interface (standalone, port 8080)
python3 camera_web.py
```

---

## Hardware

| Component | Detail |
|-----------|--------|
| Pimoroni Yukon | RP2040-based motor controller board |
| Left motors | `DualMotorModule` in SLOT2 |
| LED strip | `LEDStripModule` (NeoPixel, 8 LEDs) in SLOT3 |
| Bench power | `BenchPowerModule` (5 V regulated output) in SLOT4 |
| Right motors | `DualMotorModule` in SLOT5 |
| RC receiver | FlySky iBUS → Yukon GP26 (PIO UART, decoded by Yukon firmware) |
| Host ↔ Yukon | USB serial `/dev/ttyACM0` at 115200 baud |
| Front cameras | IMX296 global shutter (×2) via picamera2 CSI (180° rotation — mounted inverted) |
| Rear camera | IMX477 HQ camera via USB/UVC (OpenCV, mirror=true) |
| LiDAR | LD06 on `/dev/ttyAMA0`; PWM motor drive on GPIO 12 |
| GNSS | Allystar TAU1308 RTK receiver on `/dev/ttyUSB0` |
| IMU | BNO085 on Yukon I2C (Qw/ST port) |
| GPIO buttons | ESTOP (GPIO 17) / START (GPIO 27) |
| GPIO LEDs | Status LEDs on GPIO 22, 23, 25 |

---

## Key files

| File | Purpose |
|------|---------|
| `yukon_firmware_and_software/main.py` | MicroPython firmware for the Yukon RP2040 |
| `robot_daemon.py` | Pi-side robot daemon (cameras, LiDAR, GPS, RC, Yukon serial) |
| `robot_dashboard.py` | Unified Flask web dashboard — 2×2 panel grid + mobile tab view (port 5000) |
| `camera_monitor.py` | Pygame camera monitor with ArUco overlay, sharpness, and calibration |
| `camera_web.py` | Standalone Flask camera interface with MJPEG stream (port 8080) |
| `rc_drive.py` | Minimal RC-to-motor bridge |
| `drivers/ibus.py` | FlySky iBUS receiver library |
| `drivers/ld06.py` | LD06 LiDAR driver |
| `robot/aruco_detector.py` | OpenCV ArUco tag detector |
| `robot/aruco_navigator.py` | Autonomous gate navigator (ArUco + IMU) |
| `robot/gps_navigator.py` | GPS waypoint navigator |
| `robot/stereo_depth.py` | OpenCV stereo depth (BM algorithm) |
| `robot/depth_estimator.py` | Hailo-8L monocular depth inference (optional) |
| `gnss/` | GNSS driver package (TAU1308, UBlox variants, NTRIP) |
| `robot.ini` | Runtime configuration |
| `tools/yukon_sim.py` | Yukon serial simulator (`--mode gui\|web\|headless`) |
| `tools/calibrate_camera.py` | Interactive camera lens calibration tool |
| `tools/depth_viewer.py` | Standalone depth map visualiser (connects to robot dashboard) |
| `tools/setup_depth_model.py` | Download scdepthv3 HEF model for Hailo-8L |
| `tools/nav_visualiser.py` | Real-time overhead navigation view |
| `tools/generate_aruco_tags.py` | Generate ArUco tag PDFs (custom IDs, paper size, dictionary) |
| `tools/make_checkerboard_pdf.py` | Generate printable checkerboard calibration target PDF |
| `yukon_firmware_and_software/i2c_scan.py` | I2C bus scanner for the Yukon Qw/ST port |
| `tools/test_*.py` | Unit tests and live-display tools |
