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

| Slot | Module | Role |
|------|--------|------|
| SLOT2 | `DualMotorModule` | Left motors |
| SLOT5 | `DualMotorModule` | Right motors |
| Qw/ST I2C | BNO085 (optional) | Heading for bearing hold |

- Current limit: **2 A** per motor module
- Motor loop rate: **50 Hz** (20 ms per tick)
- Sensor log period: **1 s** (printed to serial, human-readable)

### Threading

| Core | Responsibility |
|------|---------------|
| Core 1 | Motor drive loop — applies left/right speeds with optional bearing-hold correction at 50 Hz |
| Core 0 | IMU heading updates, serial command parser, Yukon health monitoring |

Shared state (`_left_speed`, `_right_speed`, `_bearing_target`, `_current_heading`) is protected by a single mutex lock.

### Serial protocol

5-byte packets: `[SYNC, CMD, V_HIGH, V_LOW, CHK]`

| Byte | Value | Notes |
|------|-------|-------|
| `SYNC` | `0x7E` | Framing byte — resets state machine if seen anywhere |
| `CMD` | `cmd_code + 0x20` | Range `0x21`–`0x26` |
| `V_HIGH` | `(value >> 4) + 0x40` | Range `0x40`–`0x4F` |
| `V_LOW` | `(value & 0xF) + 0x50` | Range `0x50`–`0x5F` |
| `CHK` | `CMD ^ V_HIGH ^ V_LOW` | XOR checksum |

Device replies **ACK** (`0x06`) on success, **NAK** (`0x15`) on error.

### Command table

| Command | Code | Value | Notes |
|---------|------|-------|-------|
| `CMD_LED` | 1 | 0=LED_A off, 1=LED_A on, 2=LED_B off, 3=LED_B on | Yukon status LEDs |
| `CMD_LEFT` | 2 | speed byte | Left motor speed (see encoding below) |
| `CMD_RIGHT` | 3 | speed byte | Right motor speed |
| `CMD_KILL` | 4 | ignored | Zero both motors and disable bearing hold |
| `CMD_SENSOR` | 5 | ignored | Returns sensor data packets then ACK |
| `CMD_BEARING` | 6 | 0–254 = heading, 255 = disable | Set/clear bearing hold (NAK if no IMU) |

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

The Yukon sends 8 data packets before the final ACK:

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

Each data packet: `[SYNC, RESP_TYPE, V_HIGH, V_LOW, CHK]` where `RESP_TYPE = sensor_id + 0x30`.

### Fault recovery

If a `FaultError` is raised during Yukon monitoring, the firmware:
1. Zeros both motor speeds
2. Waits 50 ms
3. Re-enables main output and motor modules
4. Continues running (does not reboot)

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
