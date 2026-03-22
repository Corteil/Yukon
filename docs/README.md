# Yukon Robot Controller

Raspberry Pi–hosted controller for a Pimoroni Yukon robot. A Pi reads an RC transmitter via iBUS and sends motor commands to the Yukon over USB serial using a compact 5-byte protocol.

---

## Documentation

- [ARCHITECTURE.md](ARCHITECTURE.md) — Component diagram, key files, data flow, `SystemState` dataclass, threading model
- [PROTOCOL.md](PROTOCOL.md) — 5-byte serial protocol specification, command table, motor encoding, sensor response format
- [SETUP.md](SETUP.md) — Hardware wiring, device tree overlays, udev rules, dependencies, upload instructions
- [allystar.pdf](allystar.pdf) — Allystar TAU1308 GNSS module datasheet
- [../tools/README.md](../tools/README.md) — Helper scripts (calibration, ArUco PDF, firmware upload, simulator, I2C scan)

---

## Quick start

```bash
# Run the full robot stack with pygame GUI
python3 robot_gui.py

# Run the desktop web dashboard
python3 robot_web.py

# Run the mobile web dashboard (port 5001)
python3 robot_mobile.py

# RC drive only (no GUI)
python3 rc_drive.py

# Standalone LiDAR visualiser
python3 lidar_gui.py

# Camera focus/ArUco monitor (pygame)
python3 camera_monitor.py

# Camera web interface (mobile, port 8080)
python3 camera_web.py
```

---

## Hardware

| Component | Detail |
|-----------|--------|
| Pimoroni Yukon | RP2040-based motor controller board |
| Left motors | `DualMotorModule` in SLOT2 |
| Right motors | `DualMotorModule` in SLOT5 |
| RC receiver | FlySky iBUS on GPIO 9 / `/dev/ttyAMA3` (`uart3-pi5` overlay) |
| Host ↔ Yukon | USB serial `/dev/ttyACM0` at 115200 baud |
| Camera | IMX296 (global shutter, fixed focus) via picamera2 |
| LiDAR | LD06 on `/dev/ttyAMA0`; PWM motor drive on GPIO 18 |

---

## Key files

| File | Purpose |
|------|---------|
| `yukon_firmware_and_software/main.py` | MicroPython firmware for the Yukon RP2040 |
| `robot_daemon.py` | Pi-side robot daemon (camera, LiDAR, GPS, RC, Yukon serial) |
| `robot_gui.py` | 4-panel pygame monitor |
| `robot_web.py` | Flask web dashboard with MJPEG stream (port 5000) |
| `robot_mobile.py` | Mobile-optimised Flask dashboard with tab navigation (port 5001) |
| `camera_monitor.py` | Pygame camera monitor with ArUco overlay, sharpness, and calibration |
| `camera_web.py` | Mobile Flask camera interface with MJPEG stream (port 8080) |
| `rc_drive.py` | Minimal RC-to-motor bridge |
| `drivers/ibus.py` | FlySky iBUS receiver library |
| `drivers/ld06.py` | LD06 LiDAR driver |
| `robot/aruco_detector.py` | OpenCV ArUco tag detector |
| `robot/aruco_navigator.py` | Autonomous gate navigator (ArUco + IMU) |
| `robot/gps_navigator.py` | GPS waypoint navigator |
| `gnss/` | GNSS driver package (TAU1308, UBlox variants, NTRIP) |
| `robot.ini` | Runtime configuration |
| `tools/upload.py` | MicroPython file uploader |
| `tools/yukon_sim.py` | Yukon serial simulator (PTY) |
| `tools/calibrate_camera.py` | Interactive camera lens calibration tool (outputs `camera_cal.npz`) |
| `tools/generate_aruco_tags.py` | Generate ArUco tag PDFs (custom IDs, paper size, dictionary) |
| `tools/make_checkerboard_pdf.py` | Generate printable checkerboard calibration target PDF |
| `yukon_firmware_and_software/i2c_scan.py` | I2C bus scanner for the Yukon Qw/ST port |
| `tools/test_*.py` | Unit tests and live-display tools |
