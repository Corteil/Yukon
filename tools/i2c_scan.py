#!/usr/bin/env python3
"""Scan the I2C bus on GPIO 2/3 (i2c-1) for connected devices."""
import smbus2

BUS = 1  # GPIO 2 (SDA) / GPIO 3 (SCL)

found = []
try:
    with smbus2.SMBus(BUS) as bus:
        for addr in range(0x08, 0x78):
            try:
                bus.read_byte(addr)
                found.append(addr)
            except OSError:
                pass
except OSError as e:
    print(f"Cannot open /dev/i2c-{BUS}: {e}")
    print("Enable with: dtparam=i2c_arm=on in /boot/firmware/config.txt, then reboot.")
    raise SystemExit(1)

if found:
    print(f"Found {len(found)} device(s): {', '.join(f'0x{a:02X}' for a in found)}")
else:
    print("No devices found.")
