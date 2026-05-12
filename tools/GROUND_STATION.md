# Ground Station

Web-based operator dashboard for HackyRacingRobot. Runs on the operator's
laptop and communicates with the robot over a Holybro SiK V3 433 MHz radio
link using a compact binary telemetry protocol. Open `http://localhost:5000`
in any browser to see live telemetry, an FPV camera feed, GPS/LiDAR/ArUco
data, and to forward RTK corrections back to the robot.

---

## Files

| File | Purpose |
|---|---|
| `tools/serial_telemetry_v2.py` | Runs on the **robot Pi** — binary SiK radio bridge |
| `tools/ground_station_v2.py` | Runs on the **operator laptop** — Flask web server |
| `tools/ground_station.html` | Browser frontend (shared; served by both robot dashboard and ground station) |
| `robot/telemetry_proto.py` | Shared binary framing library (encoder + decoder) |

---

## Protocol overview

All data travels as binary frames rather than JSON, giving ~6× better
bandwidth efficiency over the SiK radio link.

```
Frame: [SYNC:2][TYPE:1][FLAGS:1][LEN:2 LE][PAYLOAD:N][CRC16:2 LE]
```

| Type | Direction | Rate | Content |
|------|-----------|------|---------|
| STATE  (0x01) | ↓ robot | 5 Hz | Mode, drive, flags, speed scale |
| TELEM  (0x02) | ↓ robot | 5 Hz | Voltage, current, temps, IMU heading, applied motor speeds (firmware v5+) |
| GPS    (0x03) | ↓ robot | 2 Hz | Position, fix, satellites + per-satellite sky data |
| SYS    (0x04) | ↓ robot | 1 Hz | CPU, memory, disk, Pi temperature |
| NAV    (0x05) | ↓ robot | 2 Hz | Navigator state, gate/WP, bearing, distance, tag IDs, gate labels |
| LIDAR  (0x06) | ↓ robot | 1 Hz | Subsampled distance array (zlib compressed) |
| ALARM  (0x07) | ↓ robot | event | Fault transition alarms |
| TAGS   (0x08) | ↓ robot | 5 Hz | Visible ArUco tags: ID, camera, position, distance, bearing |
| CMD    (0x81) | ↑ ground | on demand | ESTOP, mode change, logging toggle, etc. |
| RTCM   (0x82) | ↑ ground | continuous | RTK correction bytes from NTRIP caster |

Bandwidth at default rates: ~500 bytes/sec downlink (~9% of 57600-baud budget),
leaving ample headroom for RTCM uplink.

---

## Architecture

```
Robot Pi                                  Operator Laptop
──────────────────────────────────        ──────────────────────────────────────
serial_telemetry_v2.py                    ground_station_v2.py
  │                                         │
  ├─ robot.get_state()                      ├─ FrameDecoder → _GsState
  ├─ encode binary frames ──────────────────┤──→ /api/state SSE → browser
  │    5 Hz downlink                        │
  │                                         ├─ /stream/fpv → browser (local)
  │◄── encode_rtcm() frames ────────────────┤◄── NTRIP caster (internet)
  │    RTCM uplink                          │
  │◄── encode_cmd() frames ─────────────────┤◄── browser → /api/cmd
  │    commands                             │
```

---

## Setup

### 1. Install dependencies

**On the robot Pi** (if not already installed):
```bash
pip install pyserial
```

**On the operator laptop:**
```bash
pip install flask pyserial
pip install opencv-python   # optional — needed for FPV feed
```

### 2. Configure robot.ini

In the `[telemetry_radio]` section on the Pi:

```ini
[telemetry_radio]
disabled     = false
port         = /dev/sik   ; udev symlink — see docs/SETUP.md
baud         = 57600
telemetry_hz = 5
lidar        = true
lidar_step   = 5    # degrees between LiDAR samples (72 points at 5°)
```

Set `[rtcm] disabled = true` when the SiK radio is active (the bridge owns
that port and handles RTCM forwarding itself).

### 3. Configure SiK baud rate (if needed)

The SiK radios default to 57600 baud. To change both radios to 115200:

```
+++          (wait 1 second — enters command mode)
ATS1=115200
AT&W
ATZ
```

Repeat for both radios. The air data rate is independent and unchanged.

---

## Running

### Robot side (Pi)

```bash
python3 tools/serial_telemetry_v2.py

# Override port/baud
python3 tools/serial_telemetry_v2.py --port /dev/ttyUSB1 --baud 115200

# TCP only — no physical radio (LAN/loopback testing)
python3 tools/serial_telemetry_v2.py --no-serial --tcp-port 5010

# Minimal — no hardware conflicts, TCP only
python3 tools/serial_telemetry_v2.py --no-serial --no-yukon --no-gps --tcp-port 5010
```

| Flag | Description |
|------|-------------|
| `--port DEV` | SiK serial port (default: from robot.ini) |
| `--baud N` | Baud rate (default: 57600) |
| `--hz N` | Downlink rate in Hz (default: 5) |
| `--no-lidar` | Disable LiDAR downlink |
| `--no-serial` | Skip serial port — TCP-only mode |
| `--tcp-port N` | Also expose on TCP port N |
| `--no-camera` | Disable cameras |
| `--no-gps` | Disable GPS |
| `--no-motors` | Suppress drive commands |
| `--no-yukon` | Skip Yukon (use when robot_dashboard.py already owns /dev/ttyACM0) |

### Ground station (laptop)

```bash
# Real SiK radio
python3 tools/ground_station_v2.py --serial-port /dev/ttyUSB0      # Linux
python3 tools/ground_station_v2.py --serial-port COM5               # Windows

# LAN/WiFi (Pi running --tcp-port 5010)
python3 tools/ground_station_v2.py --backend network --network-host 192.168.1.10

# Fully synthetic — no hardware needed
python3 tools/ground_station_v2.py --backend fake
python3 tools/ground_station_v2.py --backend fake --fpv-device 0
```

Then open **http://localhost:5000** in any browser.

---

## Backends

### `real` (default)
Physical SiK radio on a USB serial port.

```bash
python3 tools/ground_station_v2.py --serial-port /dev/ttyUSB0 --baud 57600
```

### `network`
TCP socket. The robot must run `serial_telemetry_v2.py --tcp-port N`.

```bash
# Robot Pi
python3 tools/serial_telemetry_v2.py --tcp-port 5010

# Laptop
python3 tools/ground_station_v2.py --backend network \
    --network-host 192.168.1.10 --network-port 5010
```

### `fake`
Generates synthetic binary telemetry locally — no hardware, no radio needed.
GPS, IMU, motors, battery sag, LiDAR, and ArUco tags are all simulated.

```bash
python3 tools/ground_station_v2.py --backend fake
python3 tools/ground_station_v2.py --backend fake --fpv-device 0   # with webcam
```

---

## Battery voltage thresholds

The status bar colours the pack voltage reading red/yellow/green based on configurable thresholds derived from the `[battery]` section of `robot.ini` (chemistry × cells).  Pass overrides on the command line if the robot is running a different pack:

```bash
python3 tools/ground_station_v2.py --batt-warn-v 11.1 --batt-crit-v 10.5   # 3S LiPo tighter margins
python3 tools/ground_station_v2.py --batt-warn-v 10.5 --batt-crit-v 9.9    # default (lipo 3S)
```

| Flag | Default | Description |
|------|---------|-------------|
| `--batt-warn-v V` | `10.5` | Yellow below this voltage (lipo 3S default) |
| `--batt-crit-v V` | `9.9`  | Red below this voltage (lipo 3S default) |

---

## FPV camera

The FPV feed comes from a local USB capture card or webcam on the operator
laptop — not over the radio. `opencv-python` must be installed.

```bash
python3 tools/ground_station_v2.py --fpv-device 0    # first capture device
python3 tools/ground_station_v2.py --fpv-device 1    # second device
python3 tools/ground_station_v2.py --no-fpv           # disable
```

---

## NTRIP (RTK corrections)

The ground station connects to an NTRIP caster and forwards RTK bytes back
to the robot over the SiK uplink as RTCM frames. The robot bridge injects
them into the TAU1308 GNSS receiver.

```bash
python3 tools/ground_station_v2.py \
    --ntrip-host www.rtk2go.com \
    --ntrip-mount CAMBRIDGE \
    --ntrip-user your@email.com \
    --ntrip-password none
```

Via environment variables:
```bash
export NTRIP_USER=your@email.com
export NTRIP_PASSWORD=none
python3 tools/ground_station_v2.py --ntrip-mount CAMBRIDGE
```

---

## Commands

Buttons in the browser send CMD frames over the SiK uplink.

| Button | CMD | Effect |
|--------|-----|--------|
| ESTOP | 0x01 | Kill motors immediately |
| Reset ESTOP | 0x02 | Clear ESTOP, return to MANUAL |
| AUTO / MANUAL | 0x03 | Switch robot mode |
| ⬤ DLOG | 0x04 | Toggle ML data logging |
| 📍 Bookmark | 0x05 | Save current GPS position |
| ⏺ REC | 0x06 | Toggle camera recording |
| FPV Camera | 0x07 | Toggle bench power (FPV camera supply) |
| No Motors | 0x08 | Toggle no-motors bench mode |
| ArUco | 0x09 | Toggle ArUco detection |
| Reset Nav | 0x0A | Restart navigator from gate/waypoint 0 |
| Pause / Resume Nav | 0x0B | Pause or resume autonomous navigation (motors stop while paused; robot stays in AUTO) |

---

## ArUco distance calibration (area_k fallback)

When no camera calibration file is available, distance to tags is estimated
from bounding-box area using:

```
distance = area_k / sqrt(area_pixels²)
```

To calibrate:
1. Enable ArUco, hold a tag at a known distance D (e.g. 2.0 m)
2. Read the `area` value from the ground station tag panel
3. Calculate: `area_k = D × sqrt(area)`
4. Set in `robot.ini`:

```ini
[aruco]
area_k = 276.5   ; D * sqrt(area) at known distance
hfov   = 62.0    ; IMX296 horizontal FOV in degrees
```

---

## Bandwidth budget

SiK V3 air rate: 64 kbps; usable ~5760 bytes/sec after overhead.

| Data | Rate | Typical size | Bandwidth |
|------|------|-------------|-----------|
| STATE + TELEM | 5 Hz | 22+24 B | ~230 B/s ↓ |
| GPS + satellites | 2 Hz | ~70 B | ~140 B/s ↓ |
| NAV + TAGS | 2+5 Hz | ~20–50+~30 B | ~190 B/s ↓ |
| SYS + LIDAR | 1 Hz | 12+~21 B | ~33 B/s ↓ |
| **Total downlink** | | | **~620 B/s (11%)** |
| RTCM corrections | continuous | 500–2000 B/s | 0.5–2 KB/s ↑ |

Use `--no-lidar` to reduce downlink if needed.

---

## Port reference

| Service | Default | Machine |
|---------|---------|---------|
| Ground station web UI | `5000` | Laptop |
| TCP bridge (network backend) | `5010` | Pi |
| Robot dashboard | `5000` | Pi |
| SiK radio — robot side | `/dev/sik` | Pi |
| SiK radio — laptop side | `/dev/ttyUSB0` | Laptop |
| TAU1308 GNSS | `/dev/gnss` | Pi |

> The Pi uses udev symlinks (`/dev/sik`, `/dev/gnss`, `/dev/yukon`) so device
> ordering on reboot doesn't matter. See `docs/SETUP.md` for the rule file.

---

## Troubleshooting

**"Cannot open serial port"**
Run `ls /dev/ttyUSB*` (Linux) or check Device Manager (Windows). On Linux:
`sudo usermod -aG dialout $USER` then log out and back in.

**Badge shows "Robot offline"**
Check `serial_telemetry_v2.py` is running on the Pi and both SiK radios
have matching `NET_ID` and `AIR_SPEED` (use `ATI5` to inspect). The ground
station reconnects automatically — badge turns green within 3 seconds of
the link recovering.

**GPS shows no satellite sky view**
Per-satellite data (elevation, azimuth, SNR) is included in the GPS packet.
Check that `serial_telemetry_v2.py` is running on the Pi and that
`[telemetry_radio] disabled = false` in `robot.ini`.

**FPV feed not showing**
Install `opencv-python`. Try `--fpv-device 1` if device 0 is the built-in
webcam. On Linux, `ls /dev/video*` lists devices.

**NTRIP not connecting**
Check `http://www.rtk2go.com:2101/` for available mountpoints. RTK2go
password is literally `none`.
