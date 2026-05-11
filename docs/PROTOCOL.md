# Serial Protocols

Two binary protocols are used by the robot:
1. **Host ↔ Yukon** (5-byte frames, USB serial) — documented in this file.
2. **Robot ↔ Ground Station** (variable-length frames, SiK 433 MHz radio) — see [`tools/GROUND_STATION.md`](../tools/GROUND_STATION.md) and `robot/telemetry_proto.py`.

---

# Protocol 1 — Host ↔ Yukon

Communication between the Raspberry Pi (host) and the Pimoroni Yukon (RP2040) uses a compact 5-byte packet format. All bytes are printable ASCII so they do not interfere with the MicroPython REPL.

---

## Packet format (Host → Yukon)

```
[SYNC, CMD, V_HIGH, V_LOW, CHK]
```

| Field  | Encoding                     | Range             |
|--------|------------------------------|-------------------|
| SYNC   | `0x7E` (`~`) — fixed marker  | always `0x7E`     |
| CMD    | `cmd_code + 0x20`            | `0x21–0x2D`       |
| V_HIGH | `(value >> 4) + 0x40`        | `0x40–0x4F`       |
| V_LOW  | `(value & 0xF) + 0x50`       | `0x50–0x5F`       |
| CHK    | `CMD ^ V_HIGH ^ V_LOW`       | never equals SYNC, ACK, or NAK |

Response: `ACK` (`0x06`) on success, `NAK` (`0x15`) on any framing or checksum error.

---

## Commands

| Command          | Code | Value meaning |
|------------------|------|---------------|
| `CMD_LED`        | 1    | 0 = LED_A off, 1 = LED_A on, 2 = LED_B off, 3 = LED_B on |
| `CMD_LEFT`       | 2    | Motor speed byte (see below) — **only applied in AUTO mode** |
| `CMD_RIGHT`      | 3    | Motor speed byte — **only applied in AUTO mode** |
| `CMD_KILL`       | 4    | Ignored — zeros both motors and disables bearing hold |
| `CMD_SENSOR`     | 5    | Ignored — device replies with 25 sensor data packets then ACK (27 packets on firmware v5+) |
| `CMD_BEARING`    | 6    | 0–254 = target bearing in degrees (see below); 255 = disable bearing hold |
| `CMD_STRIP`      | 7    | Colour preset index (0=off 1=red 2=green 3=blue 4=orange 5=yellow 6=cyan 7=magenta 8=white) — stops any active pattern |
| `CMD_PIXEL_SET`  | 8    | High nibble = LED index (0–15), low nibble = colour index (0–15) — stages pixel, no hardware update |
| `CMD_PIXEL_SHOW` | 9    | Ignored — pushes all staged pixel data to strip hardware |
| `CMD_PATTERN`    | 10   | High nibble = colour index (0=keep current), low nibble = pattern (0=off 1=larson 2=random 3=rainbow 4=retro_computer 5=converge 6=estop_flash) |
| `CMD_MODE`       | 11   | 0 = MANUAL, 1 = AUTO, 2 = ESTOP. Must be sent at ≥2 Hz as a Pi heartbeat; if absent for 500 ms the Yukon triggers ESTOP. |
| `CMD_RC_QUERY`   | 12   | Ignored — Yukon replies with 15 RC data packets (14 channels + validity) then ACK |
| `CMD_BENCH`      | 13   | 0 = disable dual power switch output (DualOutputModule output 0, SLOT6), 1 = enable. Output is **on by default** at startup. |

---

## Motor speed encoding

Speed is encoded as a single byte in the range 0–200:

| Speed          | Byte value |
|----------------|------------|
| 0% (stop)      | 0          |
| +50% (forward) | 50         |
| +100% (forward)| 100        |
| −50% (reverse) | 150        |
| −100% (reverse)| 200        |

Decode logic on the Yukon:
- `byte <= 100` → `speed = byte / 100` (forward)
- `byte > 100`  → `speed = -(byte - 100) / 100` (reverse)

---

## Bearing hold (CMD_BEARING)

`CMD_BEARING` activates a PID heading-hold loop running on the Yukon's Core 1. The BNO085 IMU (connected to the Yukon Qw/ST I2C port) provides the current heading via the Game Rotation Vector report (relative to startup orientation, not compass north).

While bearing hold is active:
- `CMD_LEFT` / `CMD_RIGHT` still set the base drive speed (e.g. forward throttle).
- The PID computes a correction from the heading error and adjusts left/right motors differentially.
- `CMD_KILL` zeros the motors **and** disables bearing hold.

**Bearing encoding:**

| Value | Meaning |
|-------|---------|
| 0–254 | Target bearing in degrees: `degrees = value × 359 / 254` (~1.4° resolution) |
| 255   | Disable bearing hold — revert to direct motor control |

Encode on host: `value = round(degrees × 254 / 359)`, clamped to 0–254.

**Tuning constant** (in `yukon_firmware_and_software/main.py`):

| Constant     | Default | Effect |
|--------------|---------|--------|
| `BEARING_KP` | 0.4     | Proportional gain — tune up if robot drifts, down if it oscillates |

Correction formula: `correction = BEARING_KP × (error_degrees / 180)`, clamped to `±BEARING_KP`.

---

## Sensor response (Device → Host)

`CMD_SENSOR` triggers 25 data packets followed by ACK. Each data packet uses the same 5-byte wire format with `RESP_TYPE` replacing `CMD`:

| ID | Name              | Scale factor / encoding                              | Unit |
|----|-------------------|------------------------------------------------------|------|
| 0  | Voltage           | raw ÷ 10                                             | V    |
| 1  | Current           | raw ÷ 100                                            | A    |
| 2  | Board temp        | raw ÷ 3                                              | °C   |
| 3  | Left motor temp   | `max(front-left, rear-left)` ÷ 3                     | °C   |
| 4  | Right motor temp  | `max(front-right, rear-right)` ÷ 3                   | °C   |
| 5  | Left fault        | OR of front-left and rear-left fault signals         | —    |
| 6  | Right fault       | OR of front-right and rear-right fault signals       | —    |
| 7  | IMU heading       | same as `CMD_BEARING` (0–254 = 0–359°); 255 = absent | °    |
| 8  | IMU pitch         | `(pitch + 90) × 254 / 180`; 255 = absent. Decode: `raw × 180 / 254 − 90` (~0.7° res) | ° |
| 9  | IMU roll          | `(roll + 180) × 254 / 360`; 255 = absent. Decode: `raw × 360 / 254 − 180` (~1.4° res) | ° |
| 10 | Dual power switch temp  | raw ÷ 3                                        | °C   |
| 11 | Dual power switch fault | 0 or 1                                         | —    |
| 12 | Firmware version  | `FIRMWARE_VERSION` constant from `main.py`; 0 = old firmware (pre-versioning) | — |
| 13 | RR temp (SLOT1)   | rear-right  BigMotorModule temp ÷ 3                  | °C   |
| 14 | FR temp (SLOT2)   | front-right BigMotorModule temp ÷ 3                  | °C   |
| 15 | FL temp (SLOT3)   | front-left  BigMotorModule temp ÷ 3                  | °C   |
| 16 | RL temp (SLOT4)   | rear-left   BigMotorModule temp ÷ 3                  | °C   |
| 17 | RR current (SLOT1)| `abs(current) × 10`; decode ÷ 10                    | A    |
| 18 | FR current (SLOT2)| `abs(current) × 10`; decode ÷ 10                    | A    |
| 19 | FL current (SLOT3)| `abs(current) × 10`; decode ÷ 10                    | A    |
| 20 | RL current (SLOT4)| `abs(current) × 10`; decode ÷ 10                    | A    |
| 21 | RR fault (SLOT1)  | 0 or 1                                               | —    |
| 22 | FR fault (SLOT2)  | 0 or 1                                               | —    |
| 23 | FL fault (SLOT3)  | 0 or 1                                               | —    |
| 24 | RL fault (SLOT4)  | 0 or 1                                               | —    |
| 25 | Applied left speed  | **firmware v5+** — actual post-correction speed sent to left motors (FL+RL). Same encoding as `CMD_LEFT` (0–100 forward, 101–200 reverse). | — |
| 26 | Applied right speed | **firmware v5+** — actual post-correction speed sent to right motors (FR+RR). Same encoding as `CMD_RIGHT`. | — |

`RESP_TYPE` encoding: `resp_id + 0x30` (range `0x30–0x4A`).

**Note on ID overlap with CMD_RC_QUERY:** `CMD_RC_QUERY` response IDs start at 8 (`RESP_RC_BASE`). The host disambiguates by queue: `CMD_SENSOR` responses always start with ID 0 (Voltage), which routes to the sensor queue; `CMD_RC_QUERY` responses start with ID 8 (channel 0), which routes to the RC queue.

---

## RC channel response (Device → Host)

`CMD_RC_QUERY` triggers 15 data packets followed by ACK, using the same 5-byte wire format:

| ID    | Name              | Encoding / decode |
|-------|-------------------|-------------------|
| 8–21  | RC channels 0–13  | `raw = (µs − 1000) ÷ 5`; decode: `µs = raw × 5 + 1000`. Range 1000–2000 µs, ~5 µs resolution |
| 22    | RC validity flag  | 1 = live iBUS signal (packet within 500 ms); 0 = signal lost |

`RESP_TYPE` encoding: `resp_id + 0x30` (range `0x38–0x46`).

**Failsafes implemented on Yukon:**
- If no iBUS packet for 500 ms in MANUAL mode → motors zeroed.
- If no `CMD_MODE` for 500 ms (Pi crash / USB drop) → ESTOP (motors zeroed, mode locked until `CMD_MODE` resumes).

---

## LED strip (NeoPixel module, SLOT5)

### Colour palette

| Index | Name     | RGB           |
|-------|----------|---------------|
| 0     | off      | (0, 0, 0)     |
| 1     | red      | (255, 0, 0)   |
| 2     | green    | (0, 255, 0)   |
| 3     | blue     | (0, 0, 255)   |
| 4     | orange   | (255, 165, 0) |
| 5     | yellow   | (255, 255, 0) |
| 6     | cyan     | (0, 255, 255) |
| 7     | magenta  | (255, 0, 255) |
| 8     | white    | (255, 255, 255) |

### CMD_STRIP

Sets all LEDs to a single palette colour immediately. Stops any active pattern.

### CMD_PIXEL_SET / CMD_PIXEL_SHOW

Stage individual pixel colours without updating hardware, then push all staged data at once:

```
value = (led_index << 4) | colour_index
```

Example — set LED 3 to cyan (index 6):
```
CMD_PIXEL_SET  value = (3 << 4) | 6 = 0x36 = 54
CMD_PIXEL_SHOW value = 0 (ignored)
```

### CMD_PATTERN

Starts a built-in animation running autonomously on the Yukon. The high nibble optionally sets the colour used by colour-aware patterns (sparkle/retro_computer, converge); 0 keeps the current colour (default: white).

| Pattern | Index | Description |
|---------|-------|-------------|
| off     | 0     | Stop pattern and clear strip |
| larson  | 1     | Red comet bouncing end-to-end with decay trail (~60 ms/step) |
| random  | 2     | Each LED randomly picks a palette colour or off (~200 ms/step) |
| rainbow | 3     | Full spectrum across all LEDs, rotating hue (~40 ms/step) |
| retro_computer | 4 | Single-colour random on/off — default white (~200 ms/step) |
| converge      | 5     | LEDs fill in from both ends to centre and back (~80 ms/step) |
| estop_flash   | 6     | All LEDs flash red at ~4 Hz — used for ESTOP mode |

Example — start rainbow:
```
CMD_PATTERN  value = 0x03  (colour nibble=0 keep current, pattern=3)
```

Example — start cyan converge:
```
CMD_PATTERN  value = (6 << 4) | 5 = 0x65  (colour=cyan, pattern=converge)
```

---

# Protocol 2 — Robot ↔ Ground Station (SiK radio binary telemetry)

All frames share a common 8-byte envelope:

```
[SYNC:2][TYPE:1][FLAGS:1][LEN:2 LE][PAYLOAD:N][CRC16:2 LE]
```

SYNC = `0x48 0x52` (`HR`). CRC16 covers TYPE through end of PAYLOAD.

## Packet types (downlink — robot → ground station)

| Type | Hex  | Rate   | Payload format |
|------|------|--------|----------------|
| STATE  | 0x01 | 5 Hz | `<HBBBhhhBBB>` — flags(u16), mode, drive_left, drive_right, imu_heading, pitch, roll, speed_scale_pct, … |
| TELEM  | 0x02 | 5 Hz | `<HHbbbBHhhbb>` (16 bytes) — voltage(mV), current(mA), temps(°C), fault flags, heading(0.1°), pitch(0.1°), roll(0.1°), applied_l(×100), applied_r(×100) |
| GPS    | 0x03 | 2 Hz | `<iiBBHHHHHBBBB>` — lat/lon (1e-7°), alt(mm), fix, sats, hdop, … + variable per-satellite records |
| SYS    | 0x04 | 1 Hz | `<BBBBh>` — cpu%, mem%, disk%, temp(°C) |
| NAV    | 0x05 | 2 Hz | Variable length — see below |
| LIDAR  | 0x06 | 1 Hz | `u8` step + zlib-compressed `u16[]` distances (360/step values) |
| ALARM  | 0x07 | event | Alarm name, severity, message |
| TAGS   | 0x08 | 5 Hz | Array of 12-byte tag records: `<BBHHHhI>` — tag_id, cam_id, cx, cy, dist(mm), bearing(0.1°), area |

## NAV packet (TYPE 0x05) — variable length

```
Fixed header (15 bytes): <BBBHHhBBBBBB>
  nav_state:u8        NAV_IDLE=0, NAV_DRIVING=1 …
  gate:u8             Current target gate index
  wp:u8               Current GPS waypoint index
  dist:u16            Distance (0.1 m; 0xFFFF = null)
  bearing:u16         Bearing to target (0.1°; 0xFFFF = null)
  bearing_err:i16     Bearing error (0.1°; 0x8000 = null)
  tags:u8             ArUco tags visible in current frame
  outside_tag:u8      Front-face tag ID for outside post (0xFF = use gate*2 formula)
  inside_tag:u8       Front-face tag ID for inside post  (0xFF = use gate*2+1 formula)
  next_outside_tag:u8 Outside tag ID for next gate
  next_inside_tag:u8  Inside  tag ID for next gate
  next_gate:u8        Next gate index in sequence (0xFF = use gate+1 formula)

Variable tail (labels, both optional if payload ends early):
  gate_label_len:u8   Length of gate label string (0–31)
  gate_label:N bytes  UTF-8 gate label (e.g. "Start / Finish")
  next_label_len:u8   Length of next gate label string
  next_label:M bytes  UTF-8 next gate label
```

Tag IDs use the gate-numbering formula (`gate*2`, `gate*2+1`) when `0xFF` or when the payload is shorter than 14 bytes (backward compatibility with older firmware). `next_gate` uses `gate+1` when `0xFF` or when the payload is shorter than 15 bytes.

## Uplink (ground station → robot)

| Type | Hex  | Direction | Content |
|------|------|-----------|---------|
| CMD  | 0x81 | ↑ | Command: `cmd_id:u8` + optional body bytes |
| RTCM | 0x82 | ↑ | Raw RTCM3 correction bytes injected into TAU1308 |

CMD IDs: 0x01=ESTOP, 0x02=reset, 0x03=set_mode, 0x04=data_log_toggle, 0x05=gps_bookmark, 0x06=record_toggle, 0x07=bench_toggle, 0x08=no_motors_toggle, 0x09=aruco_toggle, 0x0A=nav_reset, 0x0B=nav_pause_toggle.
