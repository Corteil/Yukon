# Serial Protocol: Host ↔ Yukon

Communication between the Raspberry Pi (host) and the Pimoroni Yukon (RP2040) uses a compact 5-byte packet format. All bytes are printable ASCII so they do not interfere with the MicroPython REPL.

---

## Packet format (Host → Yukon)

```
[SYNC, CMD, V_HIGH, V_LOW, CHK]
```

| Field  | Encoding                     | Range             |
|--------|------------------------------|-------------------|
| SYNC   | `0x7E` (`~`) — fixed marker  | always `0x7E`     |
| CMD    | `cmd_code + 0x20`            | `0x21–0x26`       |
| V_HIGH | `(value >> 4) + 0x40`        | `0x40–0x4F`       |
| V_LOW  | `(value & 0xF) + 0x50`       | `0x50–0x5F`       |
| CHK    | `CMD ^ V_HIGH ^ V_LOW`       | `49–62`, never equals SYNC |

Response: `ACK` (`0x06`) on success, `NAK` (`0x15`) on any framing or checksum error.

---

## Commands

| Command        | Code | Value meaning |
|----------------|------|---------------|
| `CMD_LED`      | 1    | 0 = LED_A off, 1 = LED_A on, 2 = LED_B off, 3 = LED_B on |
| `CMD_LEFT`     | 2    | Motor speed byte (see below) |
| `CMD_RIGHT`    | 3    | Motor speed byte |
| `CMD_KILL`     | 4    | Ignored — zeros both motors and disables bearing hold |
| `CMD_SENSOR`   | 5    | Ignored — device replies with 8 sensor data packets then ACK |
| `CMD_BEARING`  | 6    | 0–254 = target bearing in degrees (see below); 255 = disable bearing hold |

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

`CMD_SENSOR` triggers 8 data packets followed by ACK. Each data packet uses the same 5-byte wire format with `RESP_TYPE` replacing `CMD`:

| ID | Name              | Scale factor    | Unit |
|----|-------------------|-----------------|------|
| 0  | Voltage           | raw ÷ 10        | V    |
| 1  | Current           | raw ÷ 100       | A    |
| 2  | Board temp        | raw ÷ 3         | °C   |
| 3  | Left module temp  | raw ÷ 3         | °C   |
| 4  | Right module temp | raw ÷ 3         | °C   |
| 5  | Left fault        | 1.0 (raw = 0/1) | —    |
| 6  | Right fault       | 1.0 (raw = 0/1) | —    |
| 7  | IMU heading       | same as `CMD_BEARING`; 255 = IMU absent | ° |

`RESP_TYPE` encoding: `resp_id + 0x30` (range `0x30–0x37`).
