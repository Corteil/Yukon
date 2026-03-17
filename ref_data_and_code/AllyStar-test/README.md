# TAU1308 RTK GNSS Monitor

Pygame GUI for the Allystar TAU1308 single-band RTK GNSS module with live NTRIP correction streaming.

## Features

- Live position, altitude, accuracy (H/V error, DOP)
- RTK fix quality display (GPS → DGPS → RTK Float → RTK Fixed)
- NTRIP v1/v2 client — pulls RTCM3 corrections and forwards to module
- Satellite sky map (polar plot, colour-coded by constellation)
- Signal strength bar chart (SNR per satellite)
- Position track map
- 24-hour UTC time and date display
- Antenna status indicator
- Config file (`config.ini`) — no CLI args needed for normal use

## Hardware

| Item | Detail |
|------|--------|
| Module | Allystar TAU1308-1216A00 |
| Chip | HD9301 |
| GNSS | GPS L1C/A, BDS B1I, GLONASS L1OF, Galileo E1 |
| RTK accuracy | 1.0 cm + 1 ppm (H), 2.5 cm + 1 ppm (V) |
| Interface | UART 115200 baud 8N1 |
| RTCM input | 3.0 / 3.2 / 3.3 |

## Requirements

```bash
pip install pyserial pygame
```

## Quick Start

1. Edit `config.ini` with your serial port and NTRIP credentials
2. Run:
```bash
python test-gps.py
```

## Config File (`config.ini`)

```ini
[gnss]
port      = /dev/ttyUSB0   # serial port
baud      = 115200
hz        = 5              # update rate (1-10 Hz)
debug     = false          # print raw NMEA + binary frames
no_config = true           # skip rover configuration on startup

[ntrip]
host          = www.rtk2go.com
port          = 2101
mount         = CAMBRIDGE          # mountpoint name
user          = your@email.com
password      = none               # RTK2go uses "none"
lat           = 52.2016            # approximate rover position
lon           = 0.1168
height        = 24.5
ntrip_version = 1                  # 1 = compatible with RTK2go/SNIP
```

## CLI Overrides

All config values can be overridden on the command line:

```bash
python test-gps.py --port /dev/ttyACM0 --mount NEAREST --debug
python test-gps.py --config /path/to/other.ini
python test-gps.py --host www.rtk2go.com --list   # list mountpoints
```

## RTK Tips

- Antenna must be **outdoors** with clear sky view for RTK to work
- RTK Float typically appears within 1–2 minutes of RTCM flowing
- RTK Fixed within 5 minutes under good conditions
- Use `python tau1308.py --host your.caster.com --list` to find nearby mountpoints
- First run: set `no_config = false` to configure the module, then set back to `true`

## GUI Controls

| Key | Action |
|-----|--------|
| Q or Esc | Quit |

## Fix Quality Colours

| Colour | Mode |
|--------|------|
| Red | No fix |
| Yellow | GPS |
| Orange | DGPS / PPS |
| Cyan | RTK Float |
| Green | RTK Fixed |

## Files

| File | Purpose |
|------|---------|
| `tau1308.py` | TAU1308 driver + NTRIPClient library |
| `test-gps.py` | Pygame GUI application |
| `config.ini` | Settings |
| `allystar.pdf` | TAU1308 datasheet V1.4 |
