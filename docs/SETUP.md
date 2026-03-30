# Hardware Setup

## Hardware overview

| Component     | Detail |
|---------------|--------|
| Pimoroni Yukon | RP2040-based motor controller board |
| PowerBench    | `BenchPowerModule` in SLOT1 (5 V regulated output) |
| Left motors   | `DualMotorModule` in SLOT2 |
| Right motors  | `DualMotorModule` in SLOT5 |
| RC receiver   | FlySky iBUS → Yukon GP26 (PIO UART, decoded by Yukon firmware) |
| Host ↔ Yukon  | USB serial `/dev/ttyACM0` at 115200 baud |
| Camera        | IMX296 (global shutter, fixed focus) via picamera2 |
| LiDAR         | LD06 on `/dev/ttyAMA0`; PWM motor drive on GPIO 12 |

---

## Device tree overlays

Add the following lines to `/boot/firmware/config.txt`:

```ini
# UART0 for LD06 LiDAR (GPIO 15 RX)
dtoverlay=uart0-pi5

# Hardware PWM on GPIO 12 for LD06 spin motor
dtoverlay=pwm,pin=12,func=4

# I2S audio
# dtoverlay=hifiberry-dac        # uncomment and replace with your audio overlay
```

> **Note:** The `uart3-pi5` overlay (GPIO 9 / `/dev/ttyAMA3` for iBUS) is no longer needed for the main robot stack. The RC receiver is now wired directly to the Yukon RP2040 on GP26 and decoded by the Yukon firmware. `uart3-pi5` is only required if you use `rc_drive.py` or `drivers/ibus.py` standalone tools.

> **GPIO 18 is now reserved for I2S audio (BCLK).** The LiDAR PWM was moved from GPIO 18 to GPIO 12 to free up the I2S pin group (GPIO 18–21). See [`docs/GPIO.md`](GPIO.md) for the full pin allocation.

Reboot after editing.

---

## LiDAR PWM permissions

The LD06 spin motor is driven by hardware PWM on GPIO 12. The kernel creates the sysfs PWM node as `root:root`. Install the following udev rule so the `gpio` group can write to it without `sudo`:

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

See [`docs/CALIBRATION.md`](CALIBRATION.md) for the full walk-through. Quick summary:

1. Print a calibration target:
   ```bash
   python3 tools/make_checkerboard_pdf.py   # → docs/checkerboard_9x6.pdf
   ```
2. Run the calibration tool at the **maximum sensor resolution** (1456×1088):
   ```bash
   python3 tools/calibrate_camera.py
   ```
   - Cover all 9 zones, aim for 20+ captures. Press `C` to compute, `Q` to quit and save.
   - Output: `camera_cal_1456x1088.npz` in the repo root.
3. Derive per-resolution files for the resolution you run the camera at:
   ```bash
   python3 tools/derive_calibrations.py camera_cal_1456x1088.npz 640x480
   ```
4. Set `calib_file` in `robot.ini` `[aruco]` to the derived file, e.g. `camera_cal_640x480.npz`.
5. Once the file exists, `camera_monitor.py` and `camera_web.py` show a **Calib** toggle (key `K`) to enable lens undistortion in live view.

---

## Running frontends

> **Run only ONE frontend at a time.**
> `robot_gui.py`, `robot_web.py`, and `robot_mobile.py` each create their own `Robot` instance and claim the same hardware ports (Yukon USB serial, iBUS UART, LiDAR UART, GPS USB serial). Launching two simultaneously causes port-already-open errors and undefined behaviour.

If you need both a GUI and a web interface consider running `robot_daemon.py` headlessly and accessing it via the web dashboard only.

---

## Web interfaces

| Script | Port | Description |
|--------|------|-------------|
| `robot_web.py` | 5000 | Desktop robot dashboard |
| `robot_mobile.py` | 5001 | Mobile robot dashboard (tabs: Drive, Telem, GPS, System) |
| `tools/yukon_sim_web.py` | 5002 | Yukon hardware simulator web UI (offline testing, fault injection) |
| `tools/read_data_log.py` | 5004 | JSONL data-log viewer (Drive, Telemetry, GPS map, LiDAR, Inspector) |
| `camera_web.py` | 8080 | Mobile camera interface (MJPEG stream, ArUco, capture) |

Access from any browser on the same network: `http://<pi-ip>:<port>/`

---

## Configuration

All runtime settings live in `robot.ini`. See the file for section-by-section documentation. The config overlay in `robot_gui.py` (press `C`) allows live editing without restarting.
