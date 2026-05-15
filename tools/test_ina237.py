#!/usr/bin/env python3
"""Live power monitor for Adafruit INA237 on Pi I2C1 (GPIO 2/3).

Displays input voltage, current, power, and die temperature.
Press Ctrl-C to quit.

Usage:
    python3 tools/test_ina237.py
    python3 tools/test_ina237.py --addr 0x41 --max-current 3.0
"""

import sys
import time
import argparse
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--addr",        type=lambda x: int(x, 0), default=0x40,
                        help="I2C address (default 0x40)")
    parser.add_argument("--max-current", type=float, default=10.0,
                        help="Max expected current A (default 10.0)")
    parser.add_argument("--shunt",       type=float, default=0.015,
                        help="Shunt resistor Ω (default 0.015, Adafruit breakout)")
    parser.add_argument("--rate",        type=float, default=2.0,
                        help="Sample rate Hz (default 2)")
    args = parser.parse_args()

    try:
        from drivers.ina237 import INA237
    except ImportError as e:
        print(f"Import error: {e}")
        print("Install: pip3 install smbus2")
        sys.exit(1)

    print(f"Opening INA237 at 0x{args.addr:02X}  shunt={args.shunt} Ω  max={args.max_current} A")
    try:
        ina = INA237(address=args.addr, r_shunt=args.shunt, max_current=args.max_current)
    except Exception as e:
        print(f"Failed to open INA237: {e}")
        sys.exit(1)

    print(f"{'Time':>8}  {'Voltage':>10}  {'Current':>10}  {'Power':>10}  {'Temp':>8}")
    print("-" * 56)

    v_min = v_max = None
    i_min = i_max = None
    samples = 0
    interval = 1.0 / max(0.1, args.rate)

    try:
        while True:
            try:
                v, i, p = ina.read_all()
                t = ina.temperature
                samples += 1

                if v_min is None:
                    v_min = v_max = v
                    i_min = i_max = i
                v_min, v_max = min(v_min, v), max(v_max, v)
                i_min, i_max = min(i_min, i), max(i_max, i)

                ts = time.strftime("%H:%M:%S")
                print(
                    f"\r{ts}  {v:9.3f} V  {i:9.4f} A  {p:9.3f} W  {t:6.1f} °C",
                    end="", flush=True,
                )
            except Exception as e:
                print(f"\r[read error: {e}]" + " " * 30, end="", flush=True)

            time.sleep(interval)

    except KeyboardInterrupt:
        print()
        if samples > 0:
            print(f"\n{samples} samples")
            print(f"  Voltage: {v_min:.3f} – {v_max:.3f} V")
            print(f"  Current: {i_min:.4f} – {i_max:.4f} A")


if __name__ == "__main__":
    main()
