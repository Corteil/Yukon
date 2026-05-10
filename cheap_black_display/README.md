# cheap_black_display — JC3248W535C Display Firmware

MicroPython firmware for the JC3248W535C 3.5 inch 320×480 touchscreen display
(ESP32-S3, AXS15231B driver, QSPI). Connected to the Pi via USB-CDC serial and
driven by `display_bridge.py` on the Pi side.

## Hardware

| Property | Value |
|----------|-------|
| MCU | ESP32-S3 (native USB) |
| Display | AXS15231B, 320×480 QSPI |
| Touch | AXS15231 I2C (SDA=GPIO4, SCL=GPIO8) |
| Backlight | GPIO1 PWM |
| SPI SCK | GPIO47 |
| SPI quad pins | GPIO21, 48, 40, 39 |
| DC | GPIO8, CS GPIO45 |
| USB | Native CDC at 115200 baud |

## Files

| File | Purpose |
|------|---------|
| `main.py` | Main firmware — LVGL UI, serial packet parser, command sender |
| `boot.py` | MicroPython boot stub (minimal) |
| `lib/axs15231b.py` | Display driver (AXS15231B QSPI) |
| `lib/axs15231.py` | Touch driver (AXS15231 I2C) |
| `lib/_axs15231b_init.py` | Low-level display init sequence |
| `lib/lv_config.py` | LVGL display config (portrait 320×480) |
| `lib/lv_config_90.py` | LVGL display config (landscape 480×320) |
| `test.py` | Basic LVGL widget smoke test |
| `Redtest.py` | Minimal display init and red-fill test |

## UI

Six tab-switched screens, navigated by tapping the tab bar:

| Tab | Content |
|-----|---------|
| **Drive** | IMU heading arc, speed, mode/nav state, per-motor speed bars (FL/FR/RL/RR), GPS/battery/CPU footer strip, coordinates |
| **GPS** | Lat/lon/alt, fix quality, satellite count, HDOP, speed, track, NTRIP status, scrolling GPS track map |
| **TagNav** | ArUco gate navigator — current gate, tag IDs, dist/bearing/error, next gate, waypoint, camera canvas |
| **Info** | Network (IP, hostname), main pack voltage/current, per-motor current+temp, Pi CPU/memory/disk |
| **Cam** | Per-camera status (OK, recording, lens cap), ArUco/robot-detector state, robot count |
| **Faults** | Subsystem health bitmask, active fault list with severity |

A green `*` indicator in the top-right corner shows link status (lit = packets arriving within 3 s).

## Serial Protocol

The display communicates with the Pi over USB-CDC using the same binary
protocol defined in `display_bridge.py`:

```
[0xAA, 0x55, TYPE, LEN, ...payload..., CRC8]
```

**Pi → Display** (telemetry, 10 Hz):

| Type | Name | Content |
|------|------|---------|
| 0x10 | PKT_DRIVE | Drive %, motor bars, mode, nav state, ESTOP flag |
| 0x11 | PKT_GPS | Lat/lon/alt, fix, sats, HDOP, NTRIP |
| 0x12 | PKT_TAGNAV | ArUco gate navigator state |
| 0x13 | PKT_INFO | IP, hostname, voltage, per-motor current/temp, Pi stats |
| 0x14 | PKT_CAMERA | Per-camera flags, robot count |
| 0x15 | PKT_FAULTS | Health bitmask, fault string list |

**Display → Pi** (commands, touch-triggered):

| Type | Name |
|------|------|
| 0x30 | CMD_ESTOP |
| 0x31 | CMD_RESET_ESTOP |
| 0x32 | CMD_SET_AUTO |
| 0x33 | CMD_SET_MANUAL |
| 0x34 | CMD_REBOOT |
| 0x35 | CMD_SNAPSHOT |
| 0x36 | CMD_RECORD_START |
| 0x37 | CMD_RECORD_STOP |
| 0x38 | CMD_CLEAR_FAULTS |
| 0x39 | CMD_BENCH_ON |
| 0x3A | CMD_BENCH_OFF |

## Uploading

Use [Thonny](https://thonny.org/) to upload files to the ESP32-S3:

1. Connect the display via USB.
2. In Thonny → Tools → Options → Interpreter, select **MicroPython (ESP32)** and the correct port.
3. Upload `main.py` and `boot.py` to `/` on the device.
4. Upload the contents of `lib/` to `/lib/` on the device.

The display requires the LVGL MicroPython firmware build for ESP32-S3 with the
AXS15231B driver included. See the
[lv_micropython](https://github.com/lvgl/lv_micropython) project for build instructions.

## udev Rule (Pi side)

Add to `/etc/udev/rules.d/99-hacky-robot.rules` so the port is stable:

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="303a", ATTRS{idProduct}=="4001", \
    SYMLINK+="ttyACM_display"
```

Reload: `sudo udevadm control --reload && sudo udevadm trigger`

`display_bridge.py` will then find the display automatically at `/dev/ttyACM_display`.
