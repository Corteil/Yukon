#!/usr/bin/env python3
"""
test_ibus.py — iBUS reader test and live display.

Usage:
    python3 test_ibus.py [PORT]

Default port: /dev/ttyUSB0  (override with e.g. /dev/ttyS0 or /dev/ttyAMA0)

Displays a live bar graph of all 14 channels, updating in-place.
Press Ctrl+C to exit.
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import time
import struct

# ── offline unit tests (no hardware needed) ──────────────────────────────────

def _make_packet(channels):
    """Build a valid 32-byte iBUS packet from a list of 14 channel values."""
    body = b'\x20\x40'
    for ch in channels:
        body += struct.pack('<H', ch)
    chk = (0xFFFF - sum(body)) & 0xFFFF
    return body + struct.pack('<H', chk)


def run_unit_tests():
    from drivers.ibus import IBusReader, NUM_CHANNELS, PACKET_LEN

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

    print("=== iBUS unit tests ===\n")

    # 1. Packet length
    pkt = _make_packet([1500] * NUM_CHANNELS)
    check("packet is 32 bytes", len(pkt), PACKET_LEN)

    # 2. Header bytes
    check("header byte 0 = 0x20", pkt[0], 0x20)
    check("header byte 1 = 0x40", pkt[1], 0x40)

    # 3. Checksum verify
    check("checksum valid on neutral packet", IBusReader._verify(pkt))

    # 4. Corrupt checksum detected
    bad = bytearray(pkt)
    bad[30] ^= 0xFF
    check("corrupt checksum detected", IBusReader._verify(bytes(bad)), False)

    # 5. Channel decode — all neutral
    channels = [
        struct.unpack_from('<H', pkt, 2 + i * 2)[0]
        for i in range(NUM_CHANNELS)
    ]
    check("all channels = 1500 (neutral)", channels, [1500] * NUM_CHANNELS)

    # 6. Channel decode — known values
    vals = [1000, 1100, 1200, 1300, 1400, 1500, 1600, 1700, 1800, 1900, 2000, 1500, 1500, 1500]
    pkt2 = _make_packet(vals)
    check("checksum valid on varied packet", IBusReader._verify(pkt2))
    channels2 = [
        struct.unpack_from('<H', pkt2, 2 + i * 2)[0]
        for i in range(NUM_CHANNELS)
    ]
    check("channel values decoded correctly", channels2, vals)

    # 7. Min/max values
    pkt3 = _make_packet([1000] * NUM_CHANNELS)
    check("checksum valid on all-min packet", IBusReader._verify(pkt3))
    pkt4 = _make_packet([2000] * NUM_CHANNELS)
    check("checksum valid on all-max packet", IBusReader._verify(pkt4))

    # 8. Checksum formula: 0xFFFF - sum(bytes[0:30])
    body = pkt[:30]
    expected_chk = (0xFFFF - sum(body)) & 0xFFFF
    stored_chk   = struct.unpack_from('<H', pkt, 30)[0]
    check("checksum formula correct", expected_chk, stored_chk)

    print(f"\n{'='*40}")
    print(f"Results: {passed}/{passed+failed} passed", end='')
    print("  (all passed)" if not failed else f"  ({failed} FAILED)")
    print('='*40)
    return failed == 0


# ── live display ──────────────────────────────────────────────────────────────

CHANNEL_NAMES = [
    'CH1  Aileron ',
    'CH2  Elevator',
    'CH3  Throttle',
    'CH4  Rudder  ',
    'CH5  SwA     ',
    'CH6  SwB     ',
    'CH7         ',
    'CH8         ',
    'CH9         ',
    'CH10        ',
    'CH11        ',
    'CH12        ',
    'CH13        ',
    'CH14        ',
]

BAR_WIDTH = 30


def _bar(value, lo=1000, hi=2000):
    mid  = (lo + hi) // 2
    frac = (value - lo) / (hi - lo)   # 0.0 … 1.0
    fill = int(frac * BAR_WIDTH)
    bar  = '█' * fill + '░' * (BAR_WIDTH - fill)
    return bar


def live_display(port):
    from drivers.ibus import IBusReader, IBusError

    print(f"\nOpening {port} at 115200 baud …")
    try:
        reader = IBusReader(port)
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)

    print("Waiting for iBUS signal (Ctrl+C to quit)…\n")

    # Reserve space for channel lines
    header_lines = 2
    print('\n' * (14 + header_lines), end='')

    t_start = time.time()
    last_ok  = None

    try:
        while True:
            try:
                ch = reader.read()
            except IBusError as e:
                print(f"\r\033[KSerial error: {e}")
                break

            now = time.time()

            if ch is None:
                age = f"  (no signal — {now - last_ok:.1f}s ago)" if last_ok else "  (no signal yet)"
                status = f"\033[33mWAITING\033[0m{age}"
            else:
                last_ok = now
                status  = f"\033[32mOK\033[0m  pkt={reader.packets_ok}  bad={reader.packets_bad}  t={now-t_start:.1f}s"

            # Move cursor up to redraw
            move_up = 14 + header_lines
            print(f"\033[{move_up}A", end='')

            print(f"\033[K  iBUS live — {status}")
            print(f"\033[K  {'Name':<14} {'Value':>5}  {'':^{BAR_WIDTH}}  {'Raw':>5}")

            vals = ch if ch else reader.channels
            for i, (name, val) in enumerate(zip(CHANNEL_NAMES, vals)):
                bar = _bar(val)
                print(f"\033[K  {name}  {val:>5}  {bar}  {val:>5}")

    except KeyboardInterrupt:
        print(f"\n\nExiting. OK={reader.packets_ok}  BAD={reader.packets_bad}")
    finally:
        reader.close()


# ── entry point ───────────────────────────────────────────────────────────────

if __name__ == '__main__':
    # Always run unit tests first
    ok = run_unit_tests()

    if len(sys.argv) > 1 and sys.argv[1] in ('-u', '--unit-only'):
        sys.exit(0 if ok else 1)

    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyAMA3'
    print()
    live_display(port)
