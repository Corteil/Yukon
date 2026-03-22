#!/usr/bin/env python3
"""
test_ld06.py — Unit tests and live display for the LD06 LiDAR driver.

Usage:
    python3 test_ld06.py [PORT]
    python3 test_ld06.py -u          # unit tests only

Default port: /dev/ttyAMA0

Live display shows 16 compass sectors (22.5° each) with minimum distance
per sector as a bar graph.  Full bar = far away (clear), empty = no reading.
Press Ctrl+C to exit.
"""

import struct
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import time

from drivers.ld06 import LD06, LidarScan, _crc8, _PACKET_LEN, _N_POINTS


# ── packet builder (unit-test helper) ────────────────────────────────────────

def _make_packet(speed_raw=6000, start_raw=0, end_raw=1100,
                 distances=None, intensities=None, ts=0):
    """Build a valid 47-byte LD06 packet."""
    if distances  is None: distances  = [1000] * _N_POINTS
    if intensities is None: intensities = [200]  * _N_POINTS

    body = bytearray([0x54, 0x2C])
    body += struct.pack('<H', speed_raw)
    body += struct.pack('<H', start_raw)
    for d, inten in zip(distances, intensities):
        body += struct.pack('<H', d)
        body.append(inten)
    body += struct.pack('<H', end_raw)
    body += struct.pack('<H', ts)
    body.append(_crc8(bytes(body)))
    return bytes(body)


# ── unit tests ────────────────────────────────────────────────────────────────

def run_unit_tests():
    passed = 0
    failed = 0

    def check(name, result, expected=True):
        nonlocal passed, failed
        ok = result == expected
        print(f"  {'PASS' if ok else 'FAIL'}  {name}")
        if ok:
            passed += 1
        else:
            failed += 1
            print(f"          got {result!r}, expected {expected!r}")

    print("=== LD06 unit tests ===\n")

    pkt = _make_packet()

    # 1. Packet length
    check("packet is 47 bytes", len(pkt), _PACKET_LEN)

    # 2. Header bytes
    check("header byte 0 = 0x54", pkt[0], 0x54)
    check("header byte 1 = 0x2C", pkt[1], 0x2C)

    # 3. CRC on valid packet
    check("CRC valid on nominal packet", _crc8(pkt[:-1]) == pkt[-1])

    # 4. CRC detects corruption
    bad = bytearray(pkt)
    bad[10] ^= 0xFF
    check("corrupted packet fails CRC", _crc8(bytes(bad[:-1])) != bad[-1])

    # 5. Angle interpolation — span 0.00° → 11.00°, 12 points (step = 1.0°)
    angles, _, _ = LD06._parse_packet(_make_packet(start_raw=0, end_raw=1100))
    check("12 angle points returned", len(angles), _N_POINTS)
    check("first angle = 0.00°",  angles[0],  0.0)
    check("last angle  = 11.00°", angles[-1], 11.0)

    # 6. RPM calculation  (speed_raw=6000 → 60 °/s → 10 RPM)
    _, _, rpm = LD06._parse_packet(pkt)
    check("RPM = 10.0", rpm, 10.0)

    # 7. Distance passthrough
    _, dists, _ = LD06._parse_packet(_make_packet(distances=[1234] * _N_POINTS))
    check("all distances = 1234 mm", dists, [1234] * _N_POINTS)

    # 8. Zero intensity clamps distance to 0
    pkt_zero = _make_packet(distances=[500] * _N_POINTS,
                             intensities=[0]  * _N_POINTS)
    _, dists_z, _ = LD06._parse_packet(pkt_zero)
    check("zero intensity → distance = 0", dists_z, [0] * _N_POINTS)

    # 9. Max distance value (65535) passes through
    pkt_max = _make_packet(distances=[65535] * _N_POINTS,
                            intensities=[255]  * _N_POINTS)
    _, dists_max, _ = LD06._parse_packet(pkt_max)
    check("max distance value preserved", dists_max[0], 65535)

    # 10. Wrap-around span: start=350.00°, end=5.00°  →  span=15°, step≈1.36°
    pkt_wrap = _make_packet(start_raw=35000, end_raw=500)
    angles_w, _, _ = LD06._parse_packet(pkt_wrap)
    check("wrap-around: 12 points returned", len(angles_w), _N_POINTS)
    check("wrap-around first angle = 350.00°", angles_w[0], 350.0)
    check("wrap-around last  angle =   5.00°", angles_w[-1], 5.0)

    # 11. Uniform angles: every point in range [0°, 360°)
    angles_full, _, _ = LD06._parse_packet(
        _make_packet(start_raw=0, end_raw=35900))  # 0° → 359°
    check("all angles in [0, 360)", all(0.0 <= a < 360.0 for a in angles_full))

    print(f"\n{'='*40}")
    print(f"Results: {passed}/{passed+failed} passed", end='')
    print("  (all passed)" if not failed else f"  ({failed} FAILED)")
    print('='*40)
    return failed == 0


# ── live display ──────────────────────────────────────────────────────────────

# 16 compass sectors × 22.5°.  N wraps through 0, so lo > hi there.
_SECTORS = [
    ('N  ', 348.75,  11.25),
    ('NNE',  11.25,  33.75),
    ('NE ',  33.75,  56.25),
    ('ENE',  56.25,  78.75),
    ('E  ',  78.75, 101.25),
    ('ESE', 101.25, 123.75),
    ('SE ', 123.75, 146.25),
    ('SSE', 146.25, 168.75),
    ('S  ', 168.75, 191.25),
    ('SSW', 191.25, 213.75),
    ('SW ', 213.75, 236.25),
    ('WSW', 236.25, 258.75),
    ('W  ', 258.75, 281.25),
    ('WNW', 281.25, 303.75),
    ('NW ', 303.75, 326.25),
    ('NNW', 326.25, 348.75),
]

_MAX_DIST_MM = 4000
_BAR_WIDTH   = 30


def _in_sector(angle: float, lo: float, hi: float) -> bool:
    if lo > hi:          # N sector wraps through 0°
        return angle >= lo or angle < hi
    return lo <= angle < hi


def _sector_min(scan: LidarScan, lo: float, hi: float) -> int:
    """Minimum valid distance (mm) in this sector; 0 if no reading."""
    best = _MAX_DIST_MM + 1
    for a, d in zip(scan.angles, scan.distances):
        if d > 0 and _in_sector(a, lo, hi):
            best = min(best, d)
    return best if best <= _MAX_DIST_MM else 0


def _bar(dist_mm: int) -> str:
    """Full bar = far/clear; empty = no reading or max range."""
    if dist_mm <= 0:
        return '░' * _BAR_WIDTH
    frac = min(dist_mm / _MAX_DIST_MM, 1.0)
    fill = int(frac * _BAR_WIDTH)
    return '█' * fill + '░' * (_BAR_WIDTH - fill)


def live_display(port: str):
    print(f"\nStarting LD06 on {port} …")
    lidar = LD06(port)
    lidar.start()

    n_sectors   = len(_SECTORS)
    header_lines = 3
    total_lines  = header_lines + n_sectors

    # Reserve lines for in-place redraw
    print('\n' * total_lines, end='')

    t_start    = time.time()
    scan_count = 0
    last_pts   = -1

    try:
        while True:
            time.sleep(0.1)
            scan = lidar.get_scan()

            if len(scan.angles) != last_pts:
                scan_count += 1
                last_pts = len(scan.angles)

            elapsed = time.time() - t_start
            if lidar.ok:
                status = (f"\033[32mOK\033[0m"
                          f"  rpm={scan.rpm:.1f}"
                          f"  scans={scan_count}"
                          f"  pts={len(scan.angles)}"
                          f"  t={elapsed:.1f}s")
            else:
                status = f"\033[31mNO SIGNAL\033[0m  t={elapsed:.1f}s"

            # Move cursor up and redraw
            print(f"\033[{total_lines}A", end='')
            print(f"\033[K  LD06 LiDAR live — {status}")
            print(f"\033[K  {'Sector':<5}  {'Angle range':^13}  {'Min':>7}  "
                  f"{'':^{_BAR_WIDTH}}")
            print(f"\033[K")

            for label, lo, hi in _SECTORS:
                mn  = _sector_min(scan, lo, hi)
                bar = _bar(mn)
                dist_str = f"{mn:4d} mm" if mn > 0 else "    —  "
                print(f"\033[K  {label}  {lo:5.2f}–{hi:6.2f}°  "
                      f"{dist_str}  {bar}")

    except KeyboardInterrupt:
        print(f"\n\nExiting.  scans={scan_count}")
    finally:
        lidar.stop()


# ── entry point ───────────────────────────────────────────────────────────────

if __name__ == '__main__':
    ok = run_unit_tests()

    if len(sys.argv) > 1 and sys.argv[1] in ('-u', '--unit-only'):
        sys.exit(0 if ok else 1)

    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyAMA0'
    print()
    live_display(port)
