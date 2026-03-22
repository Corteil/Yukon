# Hardware Setup

## Hardware overview

| Component     | Detail |
|---------------|--------|
| Pimoroni Yukon | RP2040-based motor controller board |
| Left motors   | `DualMotorModule` in SLOT2 |
| Right motors  | `DualMotorModule` in SLOT5 |
| RC receiver   | FlySky iBUS on GPIO 9 / `/dev/ttyAMA3` (`uart3-pi5` overlay) |
| Host ↔ Yukon  | USB serial `/dev/ttyACM0` at 115200 baud |
| Camera        | IMX296 (global shutter, fixed focus) via picamera2 |
| LiDAR         | LD06 on `/dev/ttyAMA0`; PWM motor drive on GPIO 18 |

---

## Device tree overlays

Add the following lines to `/boot/firmware/config.txt`:

```ini
# UART3 for iBUS RC receiver (GPIO 9 RX)
dtoverlay=uart3-pi5

# UART0 for LD06 LiDAR (GPIO 15 RX)
dtoverlay=uart0-pi5

# Hardware PWM on GPIO 18 for LD06 spin motor
dtoverlay=pwm,pin=18,func=2
```

Reboot after editing.

---

## LiDAR PWM permissions

The LD06 spin motor is driven by hardware PWM on GPIO 18. The kernel creates the sysfs PWM node as `root:root`. Install the following udev rule so the `gpio` group can write to it without `sudo`:

```
# /etc/udev/rules.d/99-pwm.rules
SUBSYSTEM=="pwm", KERNEL=="pwmchip[0-9]*", ACTION=="add", \
    RUN+="/bin/chgrp gpio /sys%p/export /sys%p/unexport", \
    RUN+="/bin/chmod g+w  /sys%p/export /sys%p/unexport"
SUBSYSTEM=="pwm", KERNEL=="pwm[0-9]*", ACTION=="add", \
    RUN+="/bin/sh -c 'chgrp -R gpio /sys%p && chmod -R g+rw /sys%p'"
```

Reload rules: `sudo udevadm control --reload-rules`

---

## Python dependencies

```bash
pip install pyserial picamera2 pygame opencv-python-headless flask pillow psutil
```

| Package | Used by |
|---------|---------|
| `pyserial` | Yukon serial link, iBUS, LiDAR, GNSS |
| `picamera2` | Camera capture (IMX296) |
| `pygame` | `robot_gui.py`, `camera_monitor.py` |
| `opencv-python-headless` | ArUco detection, image processing |
| `flask` | `robot_web.py`, `robot_mobile.py`, `camera_web.py` |
| `pillow` | `tools/generate_aruco_tags.py`, `tools/make_checkerboard_pdf.py` |
| `psutil` | System stats (CPU, memory, disk) |

---

## Uploading MicroPython firmware to the Yukon

Use `tools/upload.py` (not `mpremote` or `ampy` — these fail because `yukon.reset()` drops USB on every Ctrl+C):

```bash
python3 tools/upload.py yukon_firmware_and_software/main.py
```

This handles the double USB reconnect caused by `yukon.reset()` on each Ctrl+C interrupt before entering the raw REPL.

---

## Camera channel order

picamera2 `RGB888` format returns BGR bytes in memory. `robot_daemon.py` and `camera_monitor.py` both apply `[:, :, ::-1]` immediately after `capture_array()` to correct this before any processing or display.

---

## Camera lens calibration

The camera is an IMX296 global shutter module with a 2.1 mm lens (~100° FOV). Calibration corrects barrel/radial distortion.

1. Print a calibration target:
   ```bash
   python3 tools/make_checkerboard_pdf.py   # → docs/checkerboard_9x6.pdf
   ```
2. Run the calibration tool:
   ```bash
   python3 tools/calibrate_camera.py
   ```
   - Press `M` to mirror the display (useful when holding the board yourself).
   - Press `A` to toggle auto-capture (default: on — holds still for 1.2 s then captures).
   - Cover all 9 zones shown on screen (corners, edges, centre) from various angles.
   - Press `C` to compute calibration once you have ≥ 6 captures (more = better, aim for 20+).
   - Press `Q` or `Esc` to quit — calibration is saved automatically if computed.
   - Output: `camera_cal.npz` in the repo root.
3. Once `camera_cal.npz` exists, `camera_monitor.py` and `camera_web.py` show a **Calib** toggle (key `K`) to enable lens undistortion.

> **Important:** Calibrate at the same resolution the camera runs at (default **640×480** in `robot.ini`). The maps are resolution-specific — a calibration at 1456×1088 will not apply to a 640×480 stream and undistortion will be silently skipped.

---

## Web interfaces

| Script | Port | Description |
|--------|------|-------------|
| `robot_web.py` | 5000 | Desktop robot dashboard |
| `robot_mobile.py` | 5001 | Mobile robot dashboard (tabs: Drive, Telem, GPS, System) |
| `camera_web.py` | 8080 | Mobile camera interface (MJPEG stream, ArUco, capture) |

Access from any browser on the same network: `http://<pi-ip>:<port>/`

---

## Configuration

All runtime settings live in `robot.ini`. See the file for section-by-section documentation. The config overlay in `robot_gui.py` (press `C`) allows live editing without restarting.
