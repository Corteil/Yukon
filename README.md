# Yukon Robot Controller

Raspberry Pi–hosted controller for a Pimoroni Yukon robot. A Pi reads an RC transmitter via iBUS and sends motor commands to the Yukon over USB serial using a compact 5-byte protocol.

---

## Hardware

| Component | Detail |
|-----------|--------|
| Pimoroni Yukon | RP2040-based motor controller board |
| Left motors | `DualMotorModule` in SLOT2 |
| Right motors | `DualMotorModule` in SLOT5 |
| RC receiver | FlySky iBUS on GPIO 9 / `/dev/ttyAMA3` (uart3-pi5 overlay) |
| Host ↔ Yukon | USB serial `/dev/ttyACM0` at 115200 baud |

---

## Files

### `main.py` — Yukon firmware (MicroPython)

Runs on the Yukon (RP2040). Responsibilities:

- Registers and enables two `DualMotorModule`s (SLOT2 = left, SLOT5 = right)
- **Core 1:** continuously applies the latest left/right motor speeds
- **Core 0:** reads 5-byte serial commands from the host, runs Yukon health monitoring, and periodically logs sensor data
- On `FaultError`: zeros speeds, re-enables output and modules, then continues
- Exits cleanly on boot button press (`yukon.reset()`)

### `ibus.py` — FlySky iBUS receiver library

Reads the iBUS RC protocol from a serial port on the Pi.

- 32-byte packets at ~7 ms intervals (115200 8N1)
- Syncs to `0x20 0x40` header, verifies checksum (`0xFFFF − sum(bytes[0:30])`)
- Decodes 14 channels as 16-bit little-endian values (range 1000–2000, mid 1500)
- API: `IBusReader(port)` as context manager; `.read()` returns a list of 14 ints or `None`

### `test_ibus.py` — iBUS tests and live display

- **Unit tests** (no hardware): packet construction, header, checksum, channel decode
- **Live display**: bar graph of all 14 channels, updating in-place; defaults to `/dev/ttyAMA3`

```
python3 test_ibus.py              # unit tests + live display
python3 test_ibus.py --unit-only  # unit tests only
```

Channel mapping (transmitter must assign switches in its channel menu):

| Channel | Function |
|---------|----------|
| CH1 | Aileron |
| CH2 | Elevator |
| CH3 | Throttle |
| CH4 | Rudder |
| CH5 | SwA |
| CH6 | SwB |

### `test_main.py` — Host-side protocol tester

Tests the Yukon serial protocol from the Pi.

- **Encoder tests** (no hardware): round-trip encode/decode, printable-ASCII validation, checksum, speed encoding
- **Hardware tests**: LED, left/right motors, kill, sensor readback, error detection (bad checksum, out-of-range fields)
- **Ramp test**: 30-second motor speed profile with live sensor logging

```
python3 test_main.py              # encoder tests + hardware tests (auto-detect port)
python3 test_main.py --dry-run    # encoder tests only
python3 test_main.py --ramp       # 30-second ramp test
python3 test_main.py --port /dev/ttyACM0
```

### `upload.py` — MicroPython file uploader

Uploads a file to the Yukon via raw REPL, handling the double USB reconnect caused by `yukon.reset()` on each Ctrl+C interrupt.

```
python3 upload.py main.py
```

---

## Serial Protocol (Host → Yukon)

5-byte packets, all bytes printable ASCII (no REPL interference):

```
[SYNC, CMD, V_HIGH, V_LOW, CHK]
```

| Field  | Encoding | Range |
|--------|----------|-------|
| SYNC   | `0x7E` (`~`) | fixed |
| CMD    | `cmd_code + 0x20` | `0x21–0x25` |
| V_HIGH | `(value >> 4) + 0x40` | `0x40–0x4F` |
| V_LOW  | `(value & 0xF) + 0x50` | `0x50–0x5F` |
| CHK    | `CMD ^ V_HIGH ^ V_LOW` | never equals SYNC |

Response: `ACK` (0x06) on success, `NAK` (0x15) on any error.

### Commands

| Command | Code | Value |
|---------|------|-------|
| `CMD_LED` | 1 | 0=LED_A off, 1=LED_A on, 2=LED_B off, 3=LED_B on |
| `CMD_LEFT` | 2 | motor speed byte (see below) |
| `CMD_RIGHT` | 3 | motor speed byte |
| `CMD_KILL` | 4 | ignored — zeros both motors |
| `CMD_SENSOR` | 5 | ignored — replies with sensor data then ACK |

### Motor speed encoding

| Speed | Byte value |
|-------|-----------|
| 0% (stop) | 0 |
| +50% (fwd) | 50 |
| +100% (fwd) | 100 |
| −50% (rev) | 150 |
| −100% (rev) | 200 |

### Sensor response (Device → Host)

`CMD_SENSOR` triggers 7 data packets followed by ACK:

| ID | Name | Scale |
|----|------|-------|
| 0 | Voltage | raw ÷ 10 → V |
| 1 | Current | raw ÷ 100 → A |
| 2 | Board temp | raw ÷ 3 → °C |
| 3 | Left module temp | raw ÷ 3 → °C |
| 4 | Right module temp | raw ÷ 3 → °C |
| 5 | Left fault | 0 or 1 |
| 6 | Right fault | 0 or 1 |
