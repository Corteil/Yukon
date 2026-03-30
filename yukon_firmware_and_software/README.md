# Yukon Firmware & Software

MicroPython files that run directly on the Yukon RP2040.
Upload with Thonny or `tools/upload.py`. Do **not** use `mpremote` or `ampy` — `yukon.reset()` drops the USB connection on every Ctrl+C.

---

## main.py

Core firmware. Upload as `main.py` so it runs automatically on boot.

```bash
python3 tools/upload.py yukon_firmware_and_software/main.py
```

### Hardware

| Slot / Pin | Module / Device | Role |
|------------|-----------------|------|
| SLOT3 | `LEDStripModule` (NeoPixel, 8 LEDs) | Status LED strip |
| SLOT2 | `DualMotorModule` | Left motors |
| SLOT5 | `DualMotorModule` | Right motors |
| Qw/ST I2C | BNO085 (optional) | Heading for bearing hold |
| GP26 (PIO UART RX) | FlySky iBUS RC receiver | RC input at 115200 baud via PIO state machine (SM1) |

- Current limit: **2 A** per motor module
- Motor loop rate: **50 Hz** (20 ms per tick)
- Sensor log period: **1 s** (printed to serial, human-readable)

### Threading

| Core | Responsibility |
|------|---------------|
| Core 1 | Motor drive loop — applies left/right speeds with optional bearing-hold correction at 50 Hz |
| Core 0 | iBUS PIO poll (non-blocking FIFO drain), IMU heading updates, serial command parser, Pi watchdog, Yukon health monitoring |

Shared state (`_left_speed`, `_right_speed`, `_bearing_target`, `_current_heading`) is protected by a single mutex lock.

### Serial protocol

5-byte packets: `[SYNC, CMD, V_HIGH, V_LOW, CHK]`

| Byte | Value | Notes |
|------|-------|-------|
| `SYNC` | `0x7E` | Framing byte — resets state machine if seen anywhere |
| `CMD` | `cmd_code + 0x20` | Range `0x21`–`0x2C` |
| `V_HIGH` | `(value >> 4) + 0x40` | Range `0x40`–`0x4F` |
| `V_LOW` | `(value & 0xF) + 0x50` | Range `0x50`–`0x5F` |
| `CHK` | `CMD ^ V_HIGH ^ V_LOW` | XOR checksum |

Device replies **ACK** (`0x06`) on success, **NAK** (`0x15`) on error.

### Command table

| Command | Code | Value | Notes |
|---------|------|-------|-------|
| `CMD_LED` | 1 | 0=LED_A off, 1=LED_A on, 2=LED_B off, 3=LED_B on | Yukon status LEDs |
| `CMD_LEFT` | 2 | speed byte | Left motor speed — **AUTO mode only** (ignored in MANUAL/ESTOP) |
| `CMD_RIGHT` | 3 | speed byte | Right motor speed — **AUTO mode only** |
| `CMD_KILL` | 4 | ignored | Zero both motors and disable bearing hold |
| `CMD_SENSOR` | 5 | ignored | Returns 10 sensor data packets then ACK |
| `CMD_BEARING`    | 6  | 0–254 = heading, 255 = disable | Set/clear bearing hold (NAK if no IMU) |
| `CMD_STRIP`      | 7  | Colour preset index (0–8) | Set all LEDs to palette colour; stops any pattern |
| `CMD_PIXEL_SET`  | 8  | `(led_index << 4) \| colour_index` | Stage one pixel colour (no hardware update) |
| `CMD_PIXEL_SHOW` | 9  | Ignored | Push all staged pixel data to strip hardware |
| `CMD_PATTERN`    | 10 | `(colour_index << 4) \| pattern_id` | Start built-in animation (0=off 1=larson 2=random 3=rainbow 4=retro_computer 5=converge) |
| `CMD_MODE`       | 11 | 0=MANUAL, 1=AUTO, 2=ESTOP | Pi heartbeat — must arrive at ≥2 Hz; absent for >500 ms triggers ESTOP |
| `CMD_RC_QUERY`   | 12 | ignored | Returns 15 RC data packets (14 channels + validity flag) then ACK |

### Motor speed encoding

| Byte range | Decoded speed |
|------------|--------------|
| 0–100 | 0.0 → +1.0 (forward) |
| 101–200 | -0.01 → -1.0 (reverse, `value - 100` / 100) |

Right motor is inverted in hardware (`motor.speed(-right)`).

### Bearing hold

When a target bearing is set via `CMD_BEARING`, Core 1 applies a proportional correction each tick:

```
error      = shortest_arc(target − current_heading)   # −180..+180°
correction = BEARING_KP × (error / 180)               # max ±BEARING_KP
left  += correction
right -= correction
```

`BEARING_KP = 0.4` — tune upward if the robot drifts, downward if it oscillates.
Bearing hold is automatically cleared by `CMD_KILL`.
NAK is returned if `CMD_BEARING` (value ≠ 255) is received with no IMU fitted.

### CMD_SENSOR response

The Yukon sends 10 data packets before the final ACK:

| ID | Resp type | Encoding | Example |
|----|-----------|----------|---------|
| 0 `RESP_VOLTAGE` | Input voltage | `value × 10` | 11.2 V → 112 |
| 1 `RESP_CURRENT` | Total current | `value × 100` | 1.21 A → 121 |
| 2 `RESP_TEMP` | Board temperature | `value × 3` | 22.5 °C → 67 |
| 3 `RESP_TEMP_L` | Left module temp | `value × 3` | |
| 4 `RESP_TEMP_R` | Right module temp | `value × 3` | |
| 5 `RESP_FAULT_L` | Left motor fault | 0 or 1 | |
| 6 `RESP_FAULT_R` | Right motor fault | 0 or 1 | |
| 7 `RESP_HEADING` | IMU heading | same encoding as `CMD_BEARING`; 255 = IMU absent | |
| 8 `RESP_PITCH` | IMU pitch | `(pitch + 90) × 254 / 180`; decode: `raw × 180/254 − 90`; 255 = absent | +45° → 127 |
| 9 `RESP_ROLL`  | IMU roll  | `(roll + 180) × 254 / 360`; decode: `raw × 360/254 − 180`; 255 = absent | 0° → 127 |

Each data packet: `[SYNC, RESP_TYPE, V_HIGH, V_LOW, CHK]` where `RESP_TYPE = sensor_id + 0x30`.

### CMD_RC_QUERY response

The Yukon sends 15 data packets before the final ACK:

| ID | Content | Encoding | Decode |
|----|---------|----------|--------|
| 8–21 | RC channels 0–13 | `(µs − 1000) // 5` → 0–200 | `value × 5 + 1000` µs |
| 22 | RC validity flag | 0 or 1 | 1 = live iBUS signal (packet within 500 ms) |

`RESP_TYPE = resp_id + 0x30` (range `0x38`–`0x46`).

### Failsafes

Two independent failsafe mechanisms protect the robot:

| Condition | Trigger | Action |
|-----------|---------|--------|
| RC signal lost (MANUAL mode) | No valid iBUS packet for 500 ms | Zero both motors |
| Pi crash / USB drop | No `CMD_MODE` received for 500 ms | Enter ESTOP — zero motors, lock mode until `CMD_MODE` resumes |

ESTOP mode clears automatically once `CMD_MODE` packets resume.

### LED strip

The NeoPixel LED strip module in SLOT3 provides visual status feedback.

**Colour palette (indices 0–8):** off, red, green, blue, orange, yellow, cyan, magenta, white

**Built-in patterns** (run autonomously on the Yukon, zero Pi serial traffic during animation):

| Pattern | ID | Rate | Notes |
|---------|----|------|-------|
| larson  | 1  | ~60 ms/step  | Red comet bouncing with decay trail |
| random  | 2  | ~200 ms/step | Random palette colours per LED |
| rainbow | 3  | ~40 ms/step  | Rotating full-spectrum hue |
| retro_computer | 4 | ~200 ms/step | Single-colour random on/off (default white) |
| converge | 5 | ~80 ms/step  | Fill from both ends toward centre and back |

**Default behaviour (set by Pi daemon):** blue = MANUAL, green = AUTO, red = ESTOP.
Faults override to solid red. Strip clears to off on shutdown.

See `docs/PROTOCOL.md` for full encoding details.

### Fault recovery

The Yukon SDK disables main output before raising any monitoring exception. The firmware handles faults in two categories:

**Non-recoverable** (`OverVoltageError`, `OverTemperatureError`):
1. Logs `CRITICAL:` with exception type and last sensor readings
2. Zeros both motor speeds and disables bearing hold
3. Exits the main loop — `yukon.reset()` is called in the `finally` block
4. Host will see the serial connection drop

**Recoverable** (`FaultError`, `OverCurrentError`, `UnderVoltageError`, other):
1. Zeros both motor speeds and disables bearing hold
2. Waits 500 ms (cooldown to avoid hammering a transient fault)
3. Re-enables main output and motor modules
4. Continues running

Recovery is limited to **5 consecutive faults** (`MAX_CONSECUTIVE_FAULTS`). If the fault counter is exceeded, or if re-enabling fails (e.g. short circuit detected), the firmware shuts down. The counter resets to 0 after every successful monitoring cycle.

### Periodic sensor log

Every second, a human-readable line is printed to serial for debugging:

```
SENS v=11.85 i=0.123 t=23.4 tL=21.0 tR=22.1 fL=0 fR=0 hdg=142.5 tgt=off
```

### Shutdown

Pressing the Yukon **BOOT** button exits the main loop cleanly: motors are stopped and `yukon.reset()` is called.

---

## test_bno085_yukon.py

BNO085 IMU driver tests. Upload with Thonny and run from the Thonny shell. Do **not** save as `main.py`.

Tests:
1. I2C scan — BNO085 visible at expected address
2. Driver init — `BNO085()` constructs without error
3. Data flow — `update()` delivers fresh quaternion data within 500 ms

---

## i2c_scan.py

I2C bus scanner. Upload with Thonny and run from the Thonny shell. Do **not** save as `main.py`.

```
# In Thonny shell after uploading:
import i2c_scan
```

Scans the Yukon's built-in Qw/ST I2C bus (I2C0, 400 kHz) and prints found device addresses. Useful for verifying wiring before writing drivers.
