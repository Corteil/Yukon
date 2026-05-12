# Hacky Racing Robot Tools & Tests

Helper scripts, calibration tools, simulators, and host-side tests.
All scripts run on the Raspberry Pi host unless noted otherwise.
Run from the repo root.

---

## Tests

### test_main.py

Host-side tests for the Yukon `main.py` serial protocol.
Connects over USB serial and exercises motor control, LED, kill, and sensor commands.

```
python3 tools/test_main.py [flags]
```

| Flag | Default | Description |
|------|---------|-------------|
| `--port PORT` | auto-detect | Serial port (e.g. `/dev/ttyACM0`) |
| `--baud N` | `115200` | Baud rate |
| `--dry-run` | off | Run protocol encoder tests only — no hardware required |
| `--ramp` | off | Run 30-second motor ramp test instead of the standard suite |

**Examples**
```
python3 tools/test_main.py --dry-run
python3 tools/test_main.py --port /dev/ttyACM0
python3 tools/test_main.py --port /dev/ttyACM0 --ramp
```

---

### test_ibus.py

Unit tests and live channel display for the iBUS RC receiver via Pi UART.
Shows a live bar graph of all 14 channels, updating in-place.

> **Note:** The main robot stack no longer reads iBUS on the Pi — RC input is handled by Yukon GP26. `test_ibus.py` is useful when testing `rc_drive.py` or `drivers/ibus.py` standalone (requires the `uart3-pi5` overlay and iBUS wired to GPIO 9).

```
python3 tools/test_ibus.py [PORT]
```

| Argument | Default | Description |
|----------|---------|-------------|
| `PORT` (positional) | `/dev/ttyAMA3` | Serial port for the iBUS receiver |

Press `Ctrl+C` to exit the live display.

**Examples**
```
python3 tools/test_ibus.py
python3 tools/test_ibus.py /dev/ttyAMA3
```

---

### test_ld06.py

Unit tests and live display for the LD06 LiDAR driver.
Shows 16 compass sectors (22.5° each) as a bar graph of minimum distance per sector.

```
python3 tools/test_ld06.py [PORT]
python3 tools/test_ld06.py -u
```

| Argument | Default | Description |
|----------|---------|-------------|
| `PORT` (positional) | `/dev/ttyAMA0` | Serial port for the LD06 |
| `-u` | off | Run unit tests only — no hardware required |

Press `Ctrl+C` to exit the live display.

**Examples**
```
python3 tools/test_ld06.py -u
python3 tools/test_ld06.py /dev/ttyAMA0
```

---

### test_gps.py

TAU1308 RTK GNSS rover with a pygame GUI.
Settings are read from `tools/config.ini` if it exists; CLI flags override them.
Create `tools/config.ini` from the `[gnss]` and `[ntrip]` sections of `robot.ini` as a starting point, or use `--no-config` to skip module configuration.

```
python3 tools/test_gps.py [flags]
```

| Flag | Default | Description |
|------|---------|-------------|
| `--config PATH` | `tools/config.ini` (optional) | Path to config file |
| `--port PORT` | from config | Serial port for the TAU1308 |
| `--baud N` | from config | Baud rate |
| `--host HOST` | from config | NTRIP caster hostname |
| `--ntrip-port N` | from config | NTRIP caster port |
| `--mount MOUNT` | from config | NTRIP mountpoint |
| `--user USER` | from config | NTRIP username |
| `--password PASS` | from config | NTRIP password |
| `--lat N` | from config | Approximate latitude (decimal degrees) |
| `--lon N` | from config | Approximate longitude (decimal degrees) |
| `--height N` | from config | Approximate height above ellipsoid (m) |
| `--hz N` | from config | GNSS update rate in Hz |
| `--no-config` | off | Skip module configuration on startup |
| `--debug` | off | Print raw NMEA / RTCM output |
| `--dry-run` | off | NMEA parsing unit tests only — no hardware or display required |

**Examples**
```
python3 tools/test_gps.py --dry-run
python3 tools/test_gps.py
python3 tools/test_gps.py --port /dev/ttyUSB0 --debug
python3 tools/test_gps.py --no-config --hz 5
```

---

### test_gnss.py

Unit tests for the `gnss/` package — no hardware required.
Tests NMEA parsing, coordinate conversions, checksum functions, binary helpers, and GNSSBase sentence handling.

```
python3 tools/test_gnss.py
```

Tests covered:

| # | Section | What is checked |
|---|---------|-----------------|
| 1 | Checksum | `_nmea_checksum` XOR correctness; `_verify_nmea_checksum` good/bad/missing markers |
| 2 | Coordinates | `_parse_lat` / `_parse_lon` N/S/E/W sign, empty and non-numeric inputs |
| 3 | Numeric helpers | `_safe_float` / `_safe_int` valid, empty, and non-numeric inputs |
| 4 | Binary helpers | `_pack_u8/u16/u32` little-endian; `_fletcher8` checksum |
| 5 | Initial state | `GNSSBase(None)` default fields all None/zero/False |
| 6 | GGA parsing | Lat, lon, fix quality, satellites, HDOP, altitude, `has_fix`, `position`, `stats` |
| 7 | RMC parsing | `rmc_valid`, `speed_knots`, `speed_ms` conversion, `course` |
| 8 | GST parsing | `std_lat`, `std_lon` field positions; `h_error_m` calculation |
| 9 | RTK fix | Quality=4 → `has_rtk_fixed`, `fix_quality_name` = "RTK Fixed" |
| 10 | Bad sentences | Unknown type, missing `$`, empty string all return False |
| 11 | NTRIPClient | `status == NTRIP_DISCONNECTED` on construction |

---

### test_bno085.py

Unit tests and hardware verification for the BNO085 IMU.
Tests quaternion/heading math and bearing protocol without hardware; hardware flags require the Yukon connected with the IMU fitted.

```
python3 tools/test_bno085.py [flags]
```

| Flag | Default | Description |
|------|---------|-------------|
| `--port PORT` | auto-detect | Yukon serial port |
| `--hardware` | off | Run hardware protocol round-trip tests (requires Yukon + IMU) |
| `--live` | off | Live heading display (requires Yukon + IMU) |

**Examples**
```
python3 tools/test_bno085.py
python3 tools/test_bno085.py --hardware
python3 tools/test_bno085.py --port /dev/ttyACM0 --live
```

---

### test_robot.py

Integration tests for `robot_daemon.py` using the Yukon PTY simulator.
No hardware required — a pseudo-terminal connects `Robot` to `yukon_sim` in-process.

```
python3 tools/test_robot.py
```

Tests covered:

| # | Test | What is checked |
|---|------|-----------------|
| 1 | Lifecycle | `Robot` object created; initial mode is `MANUAL` |
| 2 | `get_state()` | Returns `RobotState`; correct types and zero defaults |
| 3 | Drive commands | `drive(0.5, −0.5)` → correct wire bytes in simulator |
| 4 | Kill | `kill()` zeroes both motor bytes |
| 5 | LED | `set_led_a` / `set_led_b` update simulator LED state |
| 6 | Telemetry | 1 Hz thread populates voltage, current, temperature |
| 7 | IMU heading | `get_heading()` decodes `RESP_HEADING` within 3° tolerance |
| 8 | Estop | `estop()` does not raise; mode becomes `ESTOP` |
| 9 | Reset ESTOP | `reset_estop()` returns mode to `MANUAL` |
| 10 | AUTO mode | `set_mode(AUTO/MANUAL)` transitions; `set_mode(ESTOP)` raises; `drive()` clamps and stores values |
| 11 | Data logging | `start_data_log()` / `stop_data_log()` create valid JSONL with `ts`, `mode`, `drive` fields |
| 12 | Bearing hold | `set_bearing()` sets target in sim; heading drifts to target at 90°/s within 2.5 s; `clear_bearing()` removes target |
| 13 | RC SC dlog | CH9 high starts data logging; low stops it; first poll at startup syncs baseline silently |
| 14 | RC SD pause | CH10 mid/high (>1333 µs) activates no-motors; edge-triggered so dashboard toggle is not overridden; startup does not auto-activate |
| 15 | RC SH ESTOP reset | Rising edge on CH12 resets ESTOP; same rising edge in MANUAL has no effect |

The iBUS port is set to a non-existent path — this is expected and harmless; the main robot stack no longer uses Pi-side iBUS (RC input moved to Yukon GP26).

---

### test_ground_station.py

Unit tests for `ground_station_v2.py`. No hardware, radio, or Flask server required — tests run fully in-process using Flask's test client.

```
python3 tools/test_ground_station.py
```

Tests covered:

| # | Test | What is checked |
|---|------|-----------------|
| 1 | Default state | Fresh `_GsState` has correct zero/False defaults |
| 2 | `apply_cmd` mode | estop→ESTOP; reset in ESTOP→MANUAL; reset in MANUAL no-op; set_mode blocked in ESTOP |
| 3 | `apply_cmd` flags | `no_motors_toggle`, `data_log_toggle`, `nav_pause_toggle` toggle pairs |
| 4 | `apply_cmd` recording | `record_start` sets flag (idempotent); `record_stop` clears flag (idempotent); `record_toggle` flips both ways |
| 5 | `handle_state` | Flags decoded into `mode`, `rc_active`, `cam_recording`, `speed_scale` |
| 6 | `handle_telem` | Voltage, current, board_temp, heading round-trip |
| 7 | `handle_gps` | Lat/lon, fix quality name, satellite count |
| 8 | `handle_alarm` | Entry fields; ring buffer capped at `MAX_ALARMS` |
| 9 | `_body_to_cmd_frame` | All 10 commands produce the correct CMD byte; unknown→None; set_mode encodes mode parameter |
| 10 | `api_cmd` idempotency | `record_start` only queues `CMD_RECORD_TOGGLE` when not recording; `record_stop` only when recording |
| 11 | Telemetry pipeline | `encode_state` / `encode_telem` / `encode_gps` → `FrameDecoder` → `handle_*` full roundtrip |

---

### test_leds.py

Host-side tests for the NeoPixel LED strip module (SLOT5).
Tests all colour presets, individual pixel control, `set_pixels()`, and all built-in patterns.

```
python3 tools/test_leds.py [flags]
```

| Flag | Default | Description |
|------|---------|-------------|
| `--yukon-port PORT` | `/dev/ttyACM0` | Yukon serial port |
| `--baud N` | `115200` | Baud rate |
| `--dry-run` | off | Protocol encoding tests only — no hardware required |

**Examples**
```
python3 tools/test_leds.py --dry-run
python3 tools/test_leds.py --yukon-port /dev/ttyACM0
```

---

### test_aruco.py

Unit tests and live camera test for ArUco marker detection and gate navigation.
Tests detector initialisation, synthetic frame detection, navigator helper functions,
and state-machine transitions without hardware; `--live` requires a camera.

```
python3 tools/test_aruco.py [flags]
```

| Flag | Default | Description |
|------|---------|-------------|
| `--live` | off | Run live camera detection view |
| `--camera N` | `0` | Camera index for live mode |

**Examples**
```
python3 tools/test_aruco.py
python3 tools/test_aruco.py --live
python3 tools/test_aruco.py --live --camera 0
```

---

### test_aruco_detector.py

Unit tests for `robot/aruco_detector.py` using synthetic OpenCV-generated frames — no camera required.

```
python3 tools/test_aruco_detector.py
```

Tests: blank frame, single tag, gate pair formation (even+odd base IDs), gate centre calculation, multiple gates, non-consecutive tags, FPS/timestamp fields, calibrated distance/bearing, `draw=False`.

---

### test_aruco_navigator.py

Unit tests for `robot/aruco_navigator.py` state machine using synthetic `ArUcoState` inputs — no camera, motors, or IMU required.

```
python3 tools/test_aruco_navigator.py
```

Tests all state transitions (IDLE → SEARCHING → ALIGNING → APPROACHING → PASSING → RECOVERING → COMPLETE), obstacle stop, single-tag fallback, IMU spin-search stepping, serpentine search mode, and `from_ini()` config loading.

---

### test_footage.py

Play back recorded MP4 footage through the ArUco detector and/or the Hailo-10H robot detector, with a live annotated window showing bounding boxes and detection stats.  Useful for tuning detector settings, verifying detection modules against real race footage, and building intuition about what the robot sees before a run.

```
python3 tools/test_footage.py FOOTAGE.mp4 [flags]
```

| Flag | Default | Description |
|------|---------|-------------|
| `--aruco` | off | Enable ArUco marker detection |
| `--aruco-dict NAME` | `DICT_4X4_250` | ArUco dictionary to use |
| `--aruco-calib PATH` | none | Camera calibration `.npz` for pose estimation |
| `--robot-detector HEF` | off | Enable robot detector; path to compiled `.hef` model |
| `--conf FLOAT` | `0.45` | Robot detector confidence threshold |
| `--iou FLOAT` | `0.45` | Robot detector NMS IoU threshold |
| `--save OUTPUT.mp4` | off | Save annotated video to file |
| `--start SECONDS` | `0` | Start playback at this timestamp |

**Controls**

| Key | Action |
|-----|--------|
| `Space` | Pause / resume |
| `→` / `←` | Step one frame (when paused) |
| `+` / `-` | Cycle speed: 0.1× → 0.25× → 0.5× → 1× → 2× → 4× → 8× |
| `S` | Save snapshot JPEG of current frame |
| `Q` / `Esc` | Quit |

**Examples**
```bash
# ArUco only (no Hailo required)
python3 tools/test_footage.py race.mp4 --aruco

# Robot detector only (requires trained HEF on Pi)
python3 tools/test_footage.py race.mp4 --robot-detector models/robot_detector.hef

# Both detectors together
python3 tools/test_footage.py race.mp4 --aruco --robot-detector models/robot_detector.hef

# Start 30 s in, slow to half speed for a tricky section
python3 tools/test_footage.py race.mp4 --aruco --start 30

# Save annotated video
python3 tools/test_footage.py race.mp4 --aruco --save annotated.mp4
```

---

### test_gps_navigator.py

Unit tests for `robot/gps_navigator.py` using synthetic GPS fixes and a mock Yukon — no hardware, radio, or GNSS receiver required.

```
python3 tools/test_gps_navigator.py
```

Tests covered:

| # | Section | What is checked |
|---|---------|-----------------|
| 1 | Idle state | `update()` before `start()` returns (0, 0) and state is IDLE |
| 2 | No fix → WAITING | `start()` with no GPS fix → state WAITING_FIX, motors stopped |
| 3 | Fix → NAVIGATING | Valid fix acquired → transitions to NAVIGATING |
| 4 | Heading toward WP | Bearing-to-waypoint ≈ compass heading to target |
| 5 | Arrival detection | Robot within `arrival_radius` → state ARRIVED |
| 6 | ARRIVED pause | Motors stopped during arrival pause; resumes after |
| 7 | Waypoint advance | After arriving at WP0, advances to WP1 |
| 8 | Final waypoint | After last WP → COMPLETE; returns (0, 0) |
| 9 | Loop mode | COMPLETE → restarts from WP0 when `loop=True` |
| 10 | No-loop mode | COMPLETE stays COMPLETE when `loop=False` |
| 11 | Reset | `reset()` returns to IDLE and clears waypoint index |
| 12 | Speed ramping | Motor output ramps up from zero at start of each leg |
| 13 | Max speed clamp | `fwd_speed` clamps output to ≤ 1.0 |
| 14 | Steer left | Negative heading error (target left of heading) → left > right |
| 15 | Steer right | Positive heading error (target right of heading) → right > left |
| 16 | Steer clamp | Steering clamped at `steer_max` regardless of heading error |
| 17 | Dead ahead | Zero heading error → symmetric motor outputs |
| 18 | Steer direction | heading_err = −20° → left > right (counter-clockwise correction) |
| 19 | LiDAR obstacle stop | Obstacle within `obstacle_stop_dist` in cone → motors halted |
| 20 | Look-ahead blending | Near final waypoint, `heading_err` reflects blend toward next WP |
| 21 | IMU priority | IMU heading used in preference to GPS course when available |
| 22 | `from_ini()` all keys | All config keys loaded correctly from a `ConfigParser` section |
| 23 | `from_ini()` defaults | Missing `[gps_navigator]` section → all values use defaults |

---

### test_bearing.py

Bearing-hold and pivot test web UI.  Uses `_YukonLink` directly so the RC TX switch cannot override the test.  A background heartbeat keeps the Yukon in AUTO mode.

Two test modes:
- **Forward** — drives both motors at *power%* forward while the Yukon bearing-hold PID steers to maintain the target heading (PASSING state).
- **Pivot** — turns in place using Pi-side differential steering (`l = fwd − steer`, `r = fwd + steer`); stops automatically within 3° of target (ALIGNING state).

The motor panel header shows **actual** post-correction speeds reported by firmware v5 (`applied_l`/`applied_r`) alongside the estimated speeds.

```
python3 tools/test_bearing.py
python3 tools/test_bearing.py --yukon-port /dev/ttyACM0
python3 tools/test_bearing.py --port 5010
```

---

### test_ina237.py

Live terminal monitor for the Adafruit INA237 power monitor (product 6340) on I²C1.
Displays voltage, current, power, and die temperature at a configurable sample rate.
Prints min/max summary on Ctrl-C.

```
python3 tools/test_ina237.py
python3 tools/test_ina237.py --addr 0x41 --max-current 3.0
python3 tools/test_ina237.py --shunt 0.05 --rate 5
```

| Flag | Default | Description |
|------|---------|-------------|
| `--addr HEX` | `0x40` | I²C address |
| `--max-current A` | `2.0` | Max expected current (sets shunt calibration) |
| `--shunt Ω` | `0.1` | Shunt resistor value |
| `--rate Hz` | `2.0` | Sample rate |

Requires: `pip3 install adafruit-circuitpython-ina23x adafruit-blinka adafruit-circuitpython-busdevice`

---

### test_hardware.py

Interactive real-world hardware tests requiring physical robot hardware.
Guides the operator through verifying the Yukon connection, telemetry, motors (forward/reverse per side), IMU heading, and ArUco tag detection.

```
python3 tools/test_hardware.py [--port PORT]
```

---

### test_tag_approach.py

Fluid ArUco tag approach tests.  Requires a running robot with camera and motors.
Three tests: direct approach, arc from left, arc from right.  The robot uses a continuous
steering law — no stop-and-pivot phases.

```
python3 tools/test_tag_approach.py [flags]
```

---

### nav_visualiser.py

Real-time overhead navigation view — shows robot position, visible ArUco tags, gate lines, camera bearing cone, planned aim point, navigator state, motor bars, IMU compass, and LiDAR returns.

```bash
python3 tools/nav_visualiser.py --live      # poll running robot_dashboard.py at :5000
python3 tools/nav_visualiser.py --sim       # closed-loop simulation (no hardware)
python3 tools/nav_visualiser.py --udp       # listen for UDP telemetry broadcast
python3 tools/nav_visualiser.py --live --url http://pi-ip:5000
```

---

### yukon_firmware_and_software/test_bno085_yukon.py

BNO085 IMU driver tests — runs directly on the Yukon RP2040 (MicroPython).

Upload this file **and** `lib/bno085.py` to the Yukon with Thonny, then run from the Thonny shell. Do **not** save as `main.py`.

Tests:
1. I2C scan — BNO085 visible at expected address
2. Driver init — `BNO085()` constructs without error
3. Data flow — `update()` delivers fresh quaternion data within 500 ms

---

## Calibration & Generation

### calibrate_camera.py

Interactive lens calibration tool for the IMX296 global shutter camera.
Captures checkerboard images across a 3×3 zone grid and outputs `camera_cal.npz`.

```bash
python3 tools/calibrate_camera.py
python3 tools/calibrate_camera.py --width 640 --height 480
python3 tools/calibrate_camera.py --config robot.ini
```

| Flag | Default | Description |
|------|---------|-------------|
| `--width N` | from `robot.ini` `[camera]` (640) | Capture width in pixels |
| `--height N` | from `robot.ini` `[camera]` (480) | Capture height in pixels |
| `--config PATH` | `../robot.ini` | Config file to read resolution from |

**Keys**

| Key | Action |
|-----|--------|
| `Space` | Manual capture |
| `A` | Toggle auto-capture (default: on — holds still 1.2 s then captures) |
| `M` | Toggle mirror display (useful when holding the board yourself) |
| `R` | Rotate image 90° |
| `C` | Compute calibration from current captures (≥ 6 needed) |
| `U` | Toggle undistort preview (after calibration) |
| `Q` / `Esc` | Save and quit |

**Workflow**

1. Print `docs/checkerboard_9x6.pdf` (25 mm squares, 9×6 inner corners).
2. Run the tool. Cover all 9 zones shown on screen — corners first, then edges, then centre.
3. Hold the board still in each zone; auto-capture triggers after 1.2 s.
4. Press `C` to preview calibration quality (RMS < 0.5 = excellent, < 1.0 = acceptable).
5. Press `Q` to save `camera_cal.npz`.

Output: `camera_cal_{width}x{height}.npz` in the repo root (e.g. `camera_cal_640x480.npz`).
Used by `robot/aruco_detector.py` for lens undistortion and pose estimation.

**Note:** calibrate at the highest resolution the camera supports for best accuracy,
then use `derive_calibrations.py` to generate files for all other resolutions.
The tool reads `[camera] width/height` from `robot.ini` automatically, so running
without flags will always match your configured resolution.

---

### derive_calibrations.py

Derives per-resolution calibration `.npz` files from a master calibration.
Calibrate once at full sensor resolution, then run this tool to generate
files for every other resolution automatically.

```bash
# Resolutions from [camera_calibrations] resolutions in robot.ini
python3 tools/derive_calibrations.py camera_cal_1456x1088.npz

# Explicit resolutions
python3 tools/derive_calibrations.py camera_cal_1456x1088.npz 640x480 1280x720
```

| Argument | Description |
|----------|-------------|
| `master` | Source `.npz` produced by `calibrate_camera.py` |
| `WxH ...` | Target resolutions (optional — defaults to `[camera_calibrations] resolutions` in `robot.ini`) |
| `--config PATH` | Path to `robot.ini` (default: `../robot.ini`) |

Output files are written alongside the master (e.g. `camera_cal_640x480.npz`).
`robot_daemon.py` selects the right file automatically using the `{width}x{height}` template in `[aruco] calib_file`.

---

### calibrate_aruco_distance.py

Interactive tool to calibrate the `area_k` distance fallback for ArUco detection.
Place two tags `--gate-width` metres apart and position the robot `--setup-dist`
metres from the gate centre.  The tool detects the tags and computes:

```
area_k = D × sqrt(A)
```

where D is the known distance to each post and A is the detected pixel area.
Multiple captures are averaged.  If a `camera_cal.npz` is present,
`solvePnP` distances are also shown for comparison.

```bash
python3 tools/calibrate_aruco_distance.py
python3 tools/calibrate_aruco_distance.py --setup-dist 2.0 --gate-width 1.0
python3 tools/calibrate_aruco_distance.py --camera 1   # right front camera
```

| Flag | Default | Description |
|------|---------|-------------|
| `--setup-dist M` | `2.0` | Distance from robot to gate centre in metres |
| `--gate-width M` | `1.0` | Distance between the two posts in metres |
| `--camera N` | `0` | Camera index (0 = front-left) |
| `--ini PATH` | `../robot.ini` | Config file |

**Keys**

| Key | Action |
|-----|--------|
| `Space` | Capture current frame and add to average |
| `C` | Clear all captures |
| `S` | Save recommended `area_k` to `robot.ini [aruco]` |
| `Q` / `Esc` | Quit |

---

### generate_aruco_tags.py

Generate ArUco marker PDFs — one tag per page at 300 DPI.

```bash
python3 tools/generate_aruco_tags.py                    # IDs 1–4, 4X4_50, A4
python3 tools/generate_aruco_tags.py 1 2 3 4 5 6       # specific IDs
python3 tools/generate_aruco_tags.py --dict 6X6_100 7 8
python3 tools/generate_aruco_tags.py --paper LETTER --size 150 1 2
```

**Options**

| Option | Default | Description |
|--------|---------|-------------|
| `IDs` (positional) | `1 2 3 4` | Tag IDs to generate |
| `--dict NAME` | `4X4_50` | ArUco dictionary (`4X4_50`, `4X4_100`, `5X5_100`, `6X6_100`, …) |
| `--paper SIZE` | `A4` | Paper size (`A3`, `A4`, `A5`, `LETTER`, `LEGAL`, `HALF`) |
| `--size MM` | 80% of page width | Printed tag size in mm |
| `--out PATH` | `docs/aruco_tags_<ids>.pdf` | Output PDF path |

---

### make_checkerboard_pdf.py

Generate a printable checkerboard calibration target.

```bash
python3 tools/make_checkerboard_pdf.py
```

Output: `docs/checkerboard_9x6.pdf`
- 9×6 inner corners (10×7 squares)
- 25 mm squares (matches `calibrate_camera.py`)
- Centred on A4 at 300 DPI

---

## Simulators

### pygame_gamepad_ibus_rx.py

Physical gamepad → iBUS PTY simulator. Uses a pygame joystick/gamepad to drive iBUS packets on a PTY, replacing the keyboard-based `ibus_sim.py` for use with a physical controller.

Channel layout matches `robot.ini` defaults: CH1 aileron, CH2 elevator, CH3 throttle, CH4 rudder, CH5 SF mode, CH6 SE speed select (3-pos: slow/mid/full), CH7 SA auto-type, CH8 SB GPS log, CH9 SC dlog, CH10 SD pause, CH11 SG recording, CH12 SH bookmark/ESTOP-reset.

```bash
python3 tools/pygame_gamepad_ibus_rx.py                          # default joystick 0
python3 tools/pygame_gamepad_ibus_rx.py --list-joysticks         # list available devices
python3 tools/pygame_gamepad_ibus_rx.py --joystick 1 --hz 143
python3 tools/pygame_gamepad_ibus_rx.py --axis-aileron 3 --axis-elevator 4  # Xbox layout
```

| Flag | Default | Description |
|------|---------|-------------|
| `--list-joysticks` | off | Print available joystick devices and exit |
| `--joystick N` | `0` | Joystick device index |
| `--hz N` | `143` | iBUS packet rate in Hz |
| `--deadzone N` | `0.05` | Axis deadzone (0.0–1.0) |
| `--axis-rudder N` | `0` | Axis index for CH4 rudder (left stick H) |
| `--axis-throttle N` | `1` | Axis index for CH3 throttle (left stick V, inverted) |
| `--axis-aileron N` | `2` | Axis index for CH1 aileron (right stick H) |
| `--axis-elevator N` | `3` | Axis index for CH2 elevator (right stick V) |
| `--btn-mode N` | `0` | Button for CH5 SF mode toggle (MANUAL/AUTO) |
| `--btn-speed N` | `1` | Button for CH6 SE speed cycle (slow/mid/max) |
| `--btn-type N` | `2` | Button for CH7 SA type cycle (Camera/GPS/Cam+GPS) |
| `--btn-gpslog N` | `3` | Button for CH8 SB GPS log toggle |
| `--btn-bookmark N` | `4` | Button for CH12 SH momentary bookmark |
| `--btn-pause N` | `5` | Button for CH10 SD AUTO pause toggle |
| `--btn-centre N` | `6` | Button to centre sticks and drop throttle |
| `--btn-signal N` | `7` | Button to toggle RC signal loss |

On startup the PTY path is printed to stderr. Pass it to `yukon_sim.py` or `rc_drive.py` via `--ibus-port`.

---

### ibus_sim.py

Interactive iBUS RC receiver simulator — creates a PTY and emits iBUS packets at ~143 Hz, mimicking a FlySky receiver connected to a RadioMaster TX-16S.

```bash
python3 tools/ibus_sim.py
python3 tools/ibus_sim.py --hz 143 --step 50
```

| Flag | Default | Description |
|------|---------|-------------|
| `--hz N` | `143` | Packet rate in Hz (~7 ms/packet, matching real hardware) |
| `--step N` | `50` | Stick µs change per keypress |

**Keys**

| Key | Action |
|-----|--------|
| `W` / `S` | Left Y up / down (CH3) |
| `A` / `D` | Left X left / right (CH4) |
| Arrow keys | Right X / Right Y (CH1 / CH2) |
| `1` | Toggle SF — MANUAL / AUTO (CH5) |
| `2` | Cycle SE — slow (`speed_min`) / mid (`speed_mid`) / full power (CH6) |
| `3` | Cycle SA — Camera / GPS / Cam+GPS (CH7) |
| `4` | Toggle SB — GPS log off / on (CH8) |
| `8` | Toggle SC — data log off / on (CH9) |
| `6` | Toggle SD — AUTO motors running / paused (CH10) — mid/high activates no-motors |
| `7` | Cycle SG — recording off / front cam / all cameras (CH11) |
| `5` | Momentary SH — GPS bookmark or ESTOP reset (CH12, 300 ms rising edge) |
| `Space` | Centre sticks and drop throttle to 1000 |
| `V` | Toggle RC signal loss (stops emitting packets) |
| `Q` | Quit |

On startup the PTY path is printed to stderr. Pass it to any consumer:

```bash
# Pipe into rc_drive.py
python3 rc_drive.py --ibus-port /dev/pts/3

# Pipe into yukon_sim so CMD_RC_QUERY returns live stick values
python3 tools/yukon_sim_gui.py --ibus-port /dev/pts/3
```

---

### yukon_sim.py

Consolidated PTY-based Yukon USB serial simulator for offline development and testing.
Supports three display modes selected via `--mode` (default: gui).

```bash
python3 tools/yukon_sim.py                        # GUI mode (default)
python3 tools/yukon_sim.py --mode gui
python3 tools/yukon_sim.py --mode web [--port 5002]
python3 tools/yukon_sim.py --mode headless
python3 tools/yukon_sim.py --ibus-port /dev/pts/3  # pipe ibus_sim into RC channels
```

Creates a virtual serial port (PTY) that emulates `/dev/ttyACM0`:
- Parses 5-byte command packets and responds with ACK/NAK.
- Returns simulated sensor data for `CMD_SENSOR` and RC channel data for `CMD_RC_QUERY`.
- Tracks `--no-motors` mode (sees `CMD_RC_QUERY` but no `CMD_MODE` heartbeat).
- Optionally reads live RC channels from `ibus_sim.py` via `--ibus-port`.

Simulated IMU controls (when IMU toggle is enabled):

| GUI keys | Web UI | Effect |
|----------|--------|--------|
| `]` / `[` | Pitch slider / ±15° nudge | Pitch nose-up / nose-down (−90°…+90°) |
| `'` / `;` | Roll slider / ±15° nudge | Roll right / left (−180°…+180°) |
| Heading slider | Heading slider / ±15° nudge | Compass heading (0°…360°) |

| Flag | Default | Description |
|------|---------|-------------|
| `--mode MODE` | `gui` | Display mode: `gui` \| `web` \| `headless` |
| `--port N` | `5002` | HTTP port (web mode only) |
| `--ibus-port DEV` | none | iBUS PTY/device to read RC channels from (e.g. `ibus_sim.py` PTY) |

Point any client (`robot_daemon.py`, `tools/test_main.py`) at the PTY path printed on startup.

---

## Ground Station

### ground_station_v2.py

Web-based operator dashboard for remote use. Runs on the **operator's laptop** and
connects to the robot over a Holybro SiK V3 433 MHz radio link (or TCP over LAN for testing).
Uses a compact binary telemetry protocol (~6× more bandwidth-efficient than JSON), giving
~620 B/s downlink — under 11% of the 57600-baud SiK budget with plenty of headroom for
RTCM uplink.

Three backends — select with `--backend`:

| Backend | Description |
|---------|-------------|
| `real` (default) | Physical SiK radio on a USB serial port |
| `network` | TCP socket — use with `serial_telemetry_v2.py --tcp-port` for LAN/WiFi testing |
| `fake` | Synthetic binary telemetry, no hardware needed — GPS, IMU, LiDAR, and ArUco tags all simulated |

```bash
# Physical SiK radio
python3 tools/ground_station_v2.py --serial-port /dev/ttyUSB0

# LAN/WiFi (robot running serial_telemetry_v2.py --tcp-port 5010)
python3 tools/ground_station_v2.py --backend network --network-host 192.168.1.10

# Fake data, no hardware at all
python3 tools/ground_station_v2.py --backend fake

# Fake data with local webcam as FPV
python3 tools/ground_station_v2.py --backend fake --fpv-device 0
```

Open **http://localhost:5000** (or the IP shown) in any browser.

Full setup, NTRIP configuration, protocol reference, and bandwidth budget: see [GROUND_STATION.md](GROUND_STATION.md).

---

### serial_telemetry_v2.py

Binary SiK radio telemetry bridge — runs on the **robot Pi**.
Owns the SiK radio serial port, encodes `RobotState` as binary frames at 5 Hz (STATE,
TELEM, GPS, NAV, TAGS), LiDAR at 1 Hz, SYS at 1 Hz, and sends ALARM frames on fault
transitions. Receives RTCM corrections and CMD frames over the uplink.

Also exposes a TCP port (`--tcp-port`) for LAN/WiFi testing without a physical radio.

```bash
# With SiK radio on /dev/ttyUSB1
python3 tools/serial_telemetry_v2.py

# Override port and baud
python3 tools/serial_telemetry_v2.py --port /dev/ttyUSB1 --baud 115200

# TCP-only — no radio needed (for LAN testing)
python3 tools/serial_telemetry_v2.py --tcp-port 5010 --no-serial

# Skip camera and GPS for faster startup during bench testing
python3 tools/serial_telemetry_v2.py --tcp-port 5010 --no-camera --no-gps --no-serial --no-yukon
```

| Flag | Description |
|------|-------------|
| `--port DEV` | SiK radio serial port (default: from `robot.ini`) |
| `--baud N` | Baud rate (default: 57600) |
| `--hz N` | Telemetry packet rate in Hz (default: 5) |
| `--no-lidar` | Disable 1 Hz LiDAR downlink |
| `--no-serial` | Skip serial port entirely — TCP-only mode |
| `--tcp-port N` | Also listen for TCP connections on this port |
| `--no-camera` | Disable cameras |
| `--no-gps` | Disable GPS (RTCM uplink will be silently dropped) |
| `--no-motors` | Suppress all drive commands |
| `--no-yukon` | Skip Yukon connection — use when `robot_dashboard.py` is already running and owns `/dev/ttyACM0` |

The `[telemetry_radio]` section is already in `robot.ini` (set `disabled = false` to enable it).
Set `[rtcm] disabled = true` when the SiK radio is active to avoid port conflicts.

**Local same-Pi testing** (no radio hardware needed):

```bash
# Terminal 1 — telemetry bridge with TCP, no hardware conflicts
python3 tools/serial_telemetry_v2.py --no-serial --no-yukon --no-camera --no-gps --tcp-port 5010

# Terminal 2 — ground station connects via loopback
python3 tools/ground_station_v2.py --backend network --network-host localhost --network-port 5010 --web-port 8080
```

Open **http://localhost:8080/** for the ground station UI. The `--no-yukon` flag is required when `robot_dashboard.py` is already running to avoid both processes fighting over `/dev/ttyACM0`.

---

### build_gs_html.py

Generates `tools/ground_station.html` from `robot_dashboard.py`.
Run once after cloning, and again whenever `robot_dashboard.py` changes.

```bash
python3 tools/build_gs_html.py
```

Applies seven patches to the dashboard HTML: adds the FPV Camera panel, updates the
Race preset, injects radio/RTK badges into the status bar, and hides robot-only controls.

---

## Utilities

### upload.py

Upload a MicroPython file to the Yukon RP2040 over USB serial.

```bash
python3 tools/upload.py yukon_firmware_and_software/main.py
python3 tools/upload.py yukon_firmware_and_software/main.py main.py
```

| Argument | Default | Description |
|----------|---------|-------------|
| `SRC` (positional, required) | — | Source file to upload |
| `DEST` (positional) | basename of `SRC` | Destination filename on device |

Port is hardcoded to `/dev/ttyACM0` at 115200 baud.

Use this instead of `mpremote` or `ampy` — those fail because `yukon.reset()` drops the USB connection on every Ctrl+C. This tool handles the double USB reconnect before entering the raw REPL.

---

### gps_route_builder.py

Pygame GUI for recording and editing GPS waypoint routes.
Saves and loads the versioned JSON format: `{"version": 1, "waypoints": [...]}`.

```bash
python3 tools/gps_route_builder.py
python3 tools/gps_route_builder.py --waypoints route.json --live
```

| Flag | Default | Description |
|------|---------|-------------|
| `--waypoints PATH` | none | Load waypoints JSON on start |
| `--live` | off | Connect to robot.py on start |

---

### gps_route_builder_web.py

Web-based GPS route builder — browser UI for recording and editing waypoints.
Exports the versioned JSON format: `{"version": 1, "waypoints": [...]}`.

```bash
python3 tools/gps_route_builder_web.py
python3 tools/gps_route_builder_web.py --waypoints route.json --live
```

| Flag | Default | Description |
|------|---------|-------------|
| `--host HOST` | `0.0.0.0` | Bind address |
| `--port N` | `5003` | HTTP port |
| `--waypoints PATH` | none | Load waypoints JSON on start |
| `--live` | off | Auto-connect to robot.py |

---

### read_data_log.py

Web viewer for JSONL data logs produced by `robot_daemon.py`.
Opens log files from `~/Documents/HackyRacingRobot/` and serves an interactive
browser dashboard on port 5004.

```bash
python3 tools/read_data_log.py
python3 tools/read_data_log.py --dir /path/to/logs --port 5004
```

| Flag | Default | Description |
|------|---------|-------------|
| `--dir PATH` | `~/Documents/HackyRacingRobot/` | Directory containing `.jsonl` files |
| `--host HOST` | `0.0.0.0` | Bind address |
| `--port N` | `5004` | HTTP port |

**Tabs**

| Tab | Content |
|-----|---------|
| Drive | Mode timeline (colour-coded), left/right motor commands, RC steer/throttle |
| Telemetry | Voltage, IMU heading, board/motor temps, CPU % and temp |
| GPS | Leaflet map with track coloured by mode, speed-over-time chart |
| LiDAR | Polar scatter plot for any frame via slider |
| Inspector | Raw JSON for any frame |

Charts downsample long runs to 800 points; full LiDAR data is fetched on demand per frame.

---

### depth_viewer.py

Standalone depth map visualiser. Connects to a running `robot_dashboard.py` and displays the left/right rectified camera frames, a Jet-colourmap depth map (stereo, mono, or fusion), a depth histogram, and source/metric status overlay.

Does not instantiate `Robot` directly — reads over the web stream, so it works against a remote Pi.

```bash
python3 tools/depth_viewer.py                       # local robot at :5000
python3 tools/depth_viewer.py --host 192.168.1.42
python3 tools/depth_viewer.py --max-depth 5.0 --fps 20
```

| Flag | Default | Description |
|------|---------|-------------|
| `--host HOST` | `localhost` | Dashboard host |
| `--port N` | `5000` | Dashboard port |
| `--max-depth M` | `8.0` | Depth range for colour scale in metres |
| `--fps N` | `15` | Display refresh rate in Hz |

**Keys:** `Q` / `Esc` — quit.

---

### setup_depth_model.py

Download and verify the `scdepthv3` HEF model for the Hailo-8L depth accelerator.
Downloads from the Hailo Model Zoo S3 bucket and places the file at `models/scdepthv3.hef` relative to the project root. Also verifies the Hailo device is reachable.

```bash
python3 tools/setup_depth_model.py
python3 tools/setup_depth_model.py --model-dir /path/to/models/
python3 tools/setup_depth_model.py --force         # re-download even if file exists
python3 tools/setup_depth_model.py --no-verify     # skip Hailo device check
```

| Flag | Default | Description |
|------|---------|-------------|
| `--model-dir DIR` | `<project_root>/models/` | Destination directory |
| `--force` | off | Re-download even if the file already exists |
| `--no-verify` | off | Skip Hailo device verification after download |

Enable depth in `robot.ini` (`[depth] enabled = true`) then configure `mode` and `hailo_model` as needed.

---

### yukon_monitor.py

Terminal voltage and telemetry monitor for the Yukon RP2040.
Sends `CMD_SENSOR` over USB serial and prints voltage, current, power, motor temperatures, and fault flags.
No `robot_daemon` dependency — works standalone even when the full robot stack is not running.

```bash
python3 tools/yukon_monitor.py                     # single reading
python3 tools/yukon_monitor.py --watch             # live display (overwrites in-place)
python3 tools/yukon_monitor.py --watch --interval 0.5
python3 tools/yukon_monitor.py --port /dev/ttyACM0
```

| Flag | Default | Description |
|------|---------|-------------|
| `--port DEV` | `/dev/yukon` | Serial port |
| `--baud N` | `115200` | Baud rate |
| `--watch` | off | Continuously overwrite display |
| `--interval N` | `1.0` | Refresh interval in seconds (`--watch` only) |

Voltage warnings: `< 10.5 V` → ⚠ LOW, `< 9.0 V` → ⚠ CRITICAL LOW, `< 6.0 V` → DEAD.
Motor temperature warning at `> 60 °C`.

---

### yukon_battery_monitor.py

Pygame battery gauge and telemetry display for the Yukon RP2040.
Shows a live battery bar (colour-coded green → yellow → red), voltage sparkline, current, power, motor temperatures, and pulsing fault indicators.
Reconnects automatically if the serial port drops.

```bash
python3 tools/yukon_battery_monitor.py             # 3S LiPo thresholds (default)
python3 tools/yukon_battery_monitor.py --cells 4   # 4S LiPo
python3 tools/yukon_battery_monitor.py --port /dev/ttyACM0 --interval 0.5
```

| Flag | Default | Description |
|------|---------|-------------|
| `--port DEV` | `/dev/yukon` | Serial port |
| `--baud N` | `115200` | Baud rate |
| `--cells N` | `3` | LiPo cell count (2/3/4/6) — sets voltage range |
| `--interval N` | `1.0` | Sensor poll interval in seconds |

Press `Esc` or close the window to quit.

---

### monitor_power.py

Reads Raspberry Pi 5 input voltage and current from the onboard PMIC via the
kernel `hwmon` interface.  On Pi 4 and earlier, falls back to `vcgencmd` for
core voltage only.

```bash
python3 tools/monitor_power.py              # single reading
python3 tools/monitor_power.py --watch      # live updating display
python3 tools/monitor_power.py --watch --interval 0.5
```

| Flag | Default | Description |
|------|---------|-------------|
| `--watch` | off | Continuously overwrite display |
| `--interval N` | `1.0` | Refresh interval in seconds (`--watch` only) |

---

### yukon_firmware_and_software/i2c_scan.py

I2C bus scanner — runs directly on the Yukon RP2040 (MicroPython).

Upload with Thonny and run from the Thonny shell. Do **not** save as `main.py`.

```
# In Thonny shell after uploading:
import i2c_scan
```

Scans the Yukon's built-in Qw/ST I2C bus (I2C0, 400 kHz) and prints found device addresses.
