# Hardware Setup

## Hardware overview

| Component     | Detail |
|---------------|--------|
| Pimoroni Yukon | RP2040-based motor controller board |
| Rear-right motor  | `BigMotorModule` in SLOT1 |
| Front-right motor | `BigMotorModule` in SLOT2 |
| Front-left motor  | `BigMotorModule` in SLOT3 |
| Rear-left motor   | `BigMotorModule` in SLOT4 |
| LED strip     | `LEDStripModule` in SLOT5 |
| Dual power switch | `DualOutputModule` in SLOT6 — output 0 = bench/accessory power switch (on by default) |
| RC receiver   | FlySky iBUS → Yukon GP26 (PIO UART, decoded by Yukon firmware) |
| Host ↔ Yukon  | USB serial `/dev/ttyACM0` at 115200 baud |
| Camera FL     | IMX296 global shutter, CSI CAM0 (`/base/.../i2c@80000`) via Picamera2 |
| Camera FR     | IMX296 global shutter, CSI CAM1 (`/base/.../i2c@88000`) via Picamera2 |
| Camera Rear   | IMX477 HQ camera via USB/UVC adapter → `/dev/camera_rear` (OpenCV) |
| LiDAR         | LD06 on `/dev/ttyAMA0`; PWM motor drive on GPIO 12 |
| INA237        | Adafruit INA237 breakout (product 6340) on I²C1 (GPIO 2 SDA / GPIO 3 SCL); 15 mΩ shunt, 10 A max; monitors 12 V input to Pi DC-DC converter |

---

## User group membership

The robot process (and your login session) need access to serial ports and GPIO
without `sudo`. Add your user to the required groups once:

```bash
sudo usermod -aG dialout,gpio pi
```

Log out and back in (or reboot) for the change to take effect. Verify with:

```bash
groups
# should include: dialout gpio
```

`dialout` covers all serial devices. `gpio` covers the PWM sysfs node (see LiDAR PWM section below).

---

## Disable the serial console on `/dev/ttyAMA0`

By default Raspberry Pi OS assigns a login shell to `/dev/ttyAMA0`. This
**must** be removed before the LD06 LiDAR can use that port.

```bash
sudo raspi-config
```

Navigate to: **Interface Options → Serial Port**

- "Would you like a login shell to be accessible over the serial port?" → **No**
- "Would you like the serial port hardware to be enabled?" → **Yes**

Finish and reboot. Confirm the console is gone:

```bash
ls -la /dev/ttyAMA0           # device should exist
sudo systemctl status serial-getty@ttyAMA0.service   # should be inactive/disabled
```

---

## System packages (apt)

Install these before running `pip install -r requirements.txt`:

```bash
sudo apt update
sudo apt install -y \
    python3-picamera2 \
    python3-lgpio \
    python3-rpi.gpio \
    python3-numpy
```

| Package | Used by |
|---------|---------|
| `python3-picamera2` | Camera capture (IMX296); must be installed via apt, not pip |
| `python3-lgpio` | GPIO buttons (ESTOP/start) and status LEDs on Pi 5 |
| `python3-rpi.gpio` | Stereo camera hardware sync pulse (GPIO 24); only needed when `[stereo] enabled = true` |
| `python3-numpy` | Required by OpenCV and picamera2; apt version avoids build issues |

### Hailo AI HAT+ 2 (optional)

The robot detector (`robot/robot_detector.py`) requires the Hailo-10H AI HAT+ 2 and its software stack. If the HAT is fitted, install the Hailo packages via the Raspberry Pi package repo before running the robot stack:

```bash
sudo apt install -y hailo-all
```

This installs `h10-hailort`, `python3-h10-hailort`, `hailo-tappas-core`, `hailo-models`, and `hailo-gen-ai-model-zoo`. Reboot after installation.

To download the trained robot detector model (after completing the training pipeline in `docs/robot_detector_training.html`):

```bash
mkdir -p models
scp windows-machine:Downloads/robot_detector.hef models/
```

Enable in `robot.ini`:
```ini
[robot_detector]
enabled = true

[camera_front_left]
robot_detector = true
```

The robot stack runs without the HAT — `robot_detector.py` degrades gracefully when `hailort` is absent.

---

## Device tree overlays

Add the following lines to `/boot/firmware/config.txt`:

```ini
# UART0 for LD06 LiDAR (GPIO 15 RX)
dtoverlay=uart0-pi5

# Hardware PWM on GPIO 12 for LD06 spin motor
dtoverlay=pwm,pin=12,func=4

# I²C bus 1 (GPIO 2 SDA / GPIO 3 SCL) for INA237 power monitor and BNO085 IMU
dtparam=i2c_arm=on

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

## Serial port udev symlinks

Three USB serial devices are used by the robot. Their `/dev/ttyUSB*` and `/dev/ttyACM*` node numbers can change between reboots depending on plug order. Stable symlinks are installed via a udev rule so `robot.ini` always resolves correctly.

Create `/etc/udev/rules.d/99-hacky-robot.rules`:

```
# Yukon RP2040 — match by USB vendor:product + serial number
SUBSYSTEM=="tty", ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="105b", \
    ATTRS{serial}=="e4612d169b585036", SYMLINK+="yukon", MODE="0666"

# SiK V3 radio dongle (FTDI FT230X)
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6015", \
    ATTRS{serial}=="DU0E5ZBY", SYMLINK+="sik", MODE="0666"

# Allystar TAU1308 GNSS (CH340 USB-serial)
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", \
    SYMLINK+="gnss", MODE="0666"

# JC3248W535C display (ESP32-S3 native USB CDC)
SUBSYSTEM=="tty", ATTRS{idVendor}=="303a", ATTRS{idProduct}=="4001", \
    SYMLINK+="ttyACM_display"
```

Replace the serial numbers with your actual device serials. To find them:

```bash
udevadm info -a -n /dev/ttyACM0 | grep serial
udevadm info -a -n /dev/ttyUSB0 | grep serial
```

Reload and trigger:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
ls -la /dev/yukon /dev/sik /dev/gnss /dev/ttyACM_display   # verify symlinks
```

The `robot.ini` defaults reference these symlinks (`/dev/yukon`, `/dev/gnss`, `/dev/sik`).

---

## Rear camera udev symlink

The USB/UVC rear camera gets assigned a `/dev/videoN` node that can change on reboot. Install a stable symlink so `robot.ini`'s `device = /dev/camera_rear` always resolves correctly.

1. Find the vendor/product ID and device node of your USB camera adapter:
   ```bash
   lsusb
   # e.g.  Bus 002 Device 002: ID 0c45:636d ...

   v4l2-ctl --list-devices
   # shows each camera and its /dev/videoN node(s)
   ```

2. Create the rule (replace `0c45` and `636d` with your adapter's IDs):
   ```
   # /etc/udev/rules.d/99-camera-rear.rules
   SUBSYSTEM=="video4linux", ATTRS{idVendor}=="0c45", ATTRS{idProduct}=="636d", \
       SYMLINK+="camera_rear"
   ```

3. Reload and trigger:
   ```bash
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   ls -la /dev/camera_rear   # should point to /dev/videoN
   ```

> Without this rule the daemon falls back automatically to the first USB video device it finds via sysfs, but the stable symlink is more reliable.

---

## Python dependencies

```bash
pip install -r requirements.txt
```

| Package | Used by |
|---------|---------|
| `pyserial` | Yukon serial link, iBUS, LiDAR, GNSS |
| `picamera2` | Camera capture (IMX296) |
| `pygame` | `camera_monitor.py`, `lidar_gui.py` |
| `opencv-python-headless` | ArUco detection, image processing |
| `flask` | `robot_dashboard.py`, `camera_web.py` |
| `pillow` | `tools/generate_aruco_tags.py`, `tools/make_checkerboard_pdf.py` |
| `psutil` | System stats (CPU, memory, disk) |
| `adafruit-circuitpython-ina23x` | INA237 power monitor driver (`drivers/ina237.py`) |
| `adafruit-blinka` | CircuitPython hardware abstraction layer (required by adafruit sensor drivers) |
| `adafruit-circuitpython-busdevice` | I²C/SPI device helper (required by adafruit sensor drivers) |
| `smbus2` | Direct I²C access (`tools/i2c_scan.py`) |

---

## INA237 power monitor wiring

The Adafruit INA237 breakout (product 6340) monitors the 12 V supply that feeds the Pi's DC-DC converter.

| INA237 pin | Connects to |
|-----------|-------------|
| VCC | 3.3 V (Pi pin 1) |
| GND | GND (Pi pin 6) |
| SDA | GPIO 2 / Pi pin 3 (I²C1 SDA) |
| SCL | GPIO 3 / Pi pin 5 (I²C1 SCL) |
| VIN+ | 12 V supply positive (before DC-DC converter) |
| VIN− | DC-DC converter positive input |

The 15 mΩ shunt resistor is on the breakout between VIN+ and VIN−.  Leave the A0/A1 address pins unconnected for the default I²C address of `0x40`.

Verify the sensor is visible before enabling it:

```bash
python3 tools/i2c_scan.py
# Expected output: Found devices at: 0x40
```

Enable in `robot.ini`:

```ini
[ina237]
enabled     = true
address     = 0x40
r_shunt     = 0.015
max_current = 10.0
```

Voltage warn/critical thresholds are derived automatically from the `[battery]` chemistry and cell count — no manual threshold configuration is needed.

---

## Create required directories

The daemon writes logs and recordings to fixed paths that must exist before
first run:

```bash
mkdir -p ~/Code/HackyRacingRobot/logs
mkdir -p ~/Pictures/HackyRacingRobot
mkdir -p ~/Videos/HackyRacingRobot
mkdir -p ~/Documents/HackyRacingRobot
```

Adjust the repo path if your clone is elsewhere.

---

## Uploading MicroPython firmware to the Yukon

Use **Thonny** to upload `yukon_firmware_and_software/main.py`:

1. Open `yukon_firmware_and_software/main.py` in Thonny.
2. Select **File → Save as… → MicroPython device** and save as `main.py`.
3. Press **Stop / Restart** to reboot the Yukon with the new firmware.

> **Note:** `tools/upload.py`, `mpremote`, and `ampy` all fail reliably because `yukon.reset()` drops the USB connection on every Ctrl+C before the raw REPL handshake completes. Use Thonny instead.

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
> `robot_dashboard.py` and `robot_daemon.py` each create their own `Robot` instance and claim the same hardware ports (Yukon USB serial, LiDAR UART, GPS USB serial). Launching two simultaneously causes port-already-open errors and undefined behaviour.

---

## Web interfaces

| Script | Port | Description |
|--------|------|-------------|
| `robot_dashboard.py` | 5000 | Unified robot dashboard — 2×2 panel grid (desktop/touchscreen) or tab view (mobile) |
| `tools/yukon_sim.py --mode web` | 5002 | Yukon hardware simulator web UI (offline testing, fault injection) |
| `tools/gps_route_builder_web.py` | 5003 | GPS route builder web UI |
| `tools/read_data_log.py` | 5004 | JSONL data-log viewer (Drive, Telemetry, GPS map, LiDAR, Inspector) |
| `camera_web.py` | 8080 | Standalone camera interface (MJPEG stream, ArUco, capture) |

Access from any browser on the same network: `http://<pi-ip>:<port>/`

---

## Configuration

All runtime settings live in `robot.ini`. See the file for section-by-section documentation.
