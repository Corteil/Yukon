#!/usr/bin/env python3
"""
ibus_sim.py — RadioMaster TX-16S iBUS receiver simulator

Creates a virtual serial port (PTY) that emits iBUS packets at ~7 ms intervals,
mimicking a FlySky iBUS receiver connected to a RadioMaster TX-16S transmitter.

Channel layout matches robot.ini defaults:
  CH1   Aileron      right stick ← →   Arrow left / right
  CH2   Elevator     right stick ↑ ↓   Arrow up / down
  CH3   Throttle     left  stick ↑ ↓   W / S
  CH4   Rudder       left  stick ← →   A / D
  CH5   SwA  mode    2-pos              1  (toggle MANUAL / AUTO)
  CH6   SwB  speed   3-pos              2  (cycle slow / mid / max)
  CH7   SwC  type    3-pos              3  (cycle Camera / GPS / Cam+GPS)
  CH8   SwD  GPS log 2-pos              4  (toggle off / on)
  CH10  Bookmark     momentary          5  (momentary high, auto-returns after 300 ms)
  CH9, CH11–14       unused             held at 1500

TX-16S behaviour notes:
  - Throttle starts at 1000 (stick bottom) for safety.
  - Space centres right gimbal + rudder and drops throttle to 1000.
  - V toggles signal: when lost, packet emission stops → robot failsafe triggers.
  - Stick increment is 50 µs per keypress (adjustable with --step).

Usage:
    python3 tools/ibus_sim.py
    python3 rc_drive.py --ibus-port <PTY path printed on stderr>
"""

import os
import sys
import pty
import tty
import termios
import struct
import threading
import time
import select
import argparse

# ── iBUS protocol constants ───────────────────────────────────────────────────

PACKET_LEN   = 32
NUM_CHANNELS = 14
CH_MIN       = 1000
CH_MAX       = 2000
CH_MID       = 1500
PACKET_HZ    = 143        # ~7 ms per packet, same as real FlySky receiver

# ── TX-16S channel indices (0-based) ─────────────────────────────────────────

CH_AILERON   = 0    # CH1   right stick horizontal
CH_ELEVATOR  = 1    # CH2   right stick vertical
CH_THROTTLE  = 2    # CH3   left  stick vertical
CH_RUDDER    = 3    # CH4   left  stick horizontal
CH_MODE      = 4    # CH5   SwA  2-pos
CH_SPEED     = 5    # CH6   SwB  3-pos
CH_AUTO_TYPE = 6    # CH7   SwC  3-pos
CH_GPS_LOG   = 7    # CH8   SwD  2-pos
                    # CH9   unused (index 8)
CH_BOOKMARK  = 9    # CH10  momentary button

# ── Named switch positions ────────────────────────────────────────────────────

SWA_NAMES = {1000: 'MANUAL', 2000: 'AUTO'}
SWB_NAMES = {1000: 'slow 25%', 1500: 'mid', 2000: 'max'}
SWC_NAMES = {1000: 'Camera', 1500: 'GPS', 2000: 'Cam+GPS'}
SWD_NAMES = {1000: 'off', 2000: 'on'}

# ── Shared state ─────────────────────────────────────────────────────────────

_lock  = threading.Lock()
_state = {
    'channels'       : [CH_MID] * NUM_CHANNELS,
    'rc_valid'       : True,     # False = stop emitting (simulate signal loss)
    'packets_sent'   : 0,
    'running'        : True,
    'bookmark_until' : 0.0,      # monotonic time after which CH10 returns to 1000
}

# Safe power-on defaults
_state['channels'][CH_THROTTLE]  = CH_MIN    # throttle at bottom (safety)
_state['channels'][CH_AILERON]   = CH_MID
_state['channels'][CH_ELEVATOR]  = CH_MID
_state['channels'][CH_RUDDER]    = CH_MID
_state['channels'][CH_MODE]      = 1000      # SwA: MANUAL
_state['channels'][CH_SPEED]     = 1500      # SwB: mid
_state['channels'][CH_AUTO_TYPE] = 1000      # SwC: Camera
_state['channels'][CH_GPS_LOG]   = 1000      # SwD: GPS log off
_state['channels'][CH_BOOKMARK]  = 1000      # CH10: low

# ── iBUS packet builder ───────────────────────────────────────────────────────

def _build_packet(channels):
    """Return a 32-byte iBUS packet for the given 14-channel list."""
    data = bytearray([0x20, 0x40])
    for ch in channels:
        data += struct.pack('<H', max(CH_MIN, min(CH_MAX, ch)))
    checksum = (0xFFFF - sum(data)) & 0xFFFF
    data += struct.pack('<H', checksum)
    assert len(data) == PACKET_LEN
    return bytes(data)


# ── Sender thread ─────────────────────────────────────────────────────────────

def _ibus_sender(master_fd, packet_hz):
    """Write iBUS packets to the PTY master at packet_hz."""
    interval  = 1.0 / packet_hz
    next_send = time.monotonic()

    while True:
        with _lock:
            if not _state['running']:
                break
            valid    = _state['rc_valid']
            channels = list(_state['channels'])
            # Auto-release bookmark after its hold time expires
            if _state['bookmark_until'] and time.monotonic() >= _state['bookmark_until']:
                _state['channels'][CH_BOOKMARK] = CH_MIN
                _state['bookmark_until'] = 0.0
                channels = list(_state['channels'])

        now = time.monotonic()
        if now >= next_send:
            if valid:
                try:
                    os.write(master_fd, _build_packet(channels))
                    with _lock:
                        _state['packets_sent'] += 1
                except OSError:
                    break
            next_send += interval
            # Prevent unbounded catch-up after a stall
            if time.monotonic() > next_send + interval * 5:
                next_send = time.monotonic()
        else:
            time.sleep(max(0.0, next_send - time.monotonic() - 0.0005))


# ── Display ───────────────────────────────────────────────────────────────────

def _bar(val, width=28):
    """Render a channel value (1000–2000) as a centred bar."""
    half = width // 2
    bar  = [' '] * width
    bar[half] = '|'
    frac = (val - CH_MID) / (CH_MAX - CH_MID)   # −1.0 … +1.0
    pos  = int(round(frac * half))
    if pos > 0:
        for i in range(half + 1, min(half + 1 + pos, width)):
            bar[i] = '#'
    elif pos < 0:
        for i in range(max(0, half + pos), half):
            bar[i] = '#'
    return '[' + ''.join(bar) + f']  {val:4d} µs'


def draw(pty_path):
    with _lock:
        ch    = list(_state['channels'])
        valid = _state['rc_valid']
        pkts  = _state['packets_sent']

    swa = ch[CH_MODE]
    swb = ch[CH_SPEED]
    swc = ch[CH_AUTO_TYPE]
    swd = ch[CH_GPS_LOG]
    bkm = ch[CH_BOOKMARK]

    sig_str = 'OK' if valid else '*** LOST (V to restore) ***'

    lines = [
        '\033[2J\033[H',
        '=== TX-16S iBUS Simulator ' + '=' * 35,
        f'  iBUS PTY   : {pty_path}',
        f'  Packets    : {pkts:>7d}   Signal: {sig_str}',
        '',
        '─── Right gimbal ─── Arrow keys ─────────────────────────────────',
        f'  CH1 Aileron   {_bar(ch[CH_AILERON])}',
        f'  CH2 Elevator  {_bar(ch[CH_ELEVATOR])}',
        '',
        '─── Left gimbal  ─── W/S=throttle  A/D=rudder ───────────────────',
        f'  CH3 Throttle  {_bar(ch[CH_THROTTLE])}',
        f'  CH4 Rudder    {_bar(ch[CH_RUDDER])}',
        '',
        '─── Switches ────────────────────────────────────────────────────',
        f'  [1] CH5  SwA mode    {SWA_NAMES.get(swa, str(swa)):>10s}  {"●" if swa == 2000 else "○"}',
        f'  [2] CH6  SwB speed   {SWB_NAMES.get(swb, str(swb)):>10s}',
        f'  [3] CH7  SwC type    {SWC_NAMES.get(swc, str(swc)):>10s}',
        f'  [4] CH8  SwD GPS log {SWD_NAMES.get(swd, str(swd)):>10s}  {"●" if swd == 2000 else "○"}',
        f'  [5] CH10 Bookmark    {"TRIGGERED" if bkm >= CH_MID else "ready":>10s}',
        '',
        '─── Keys ────────────────────────────────────────────────────────',
        '  Arrows  right gimbal (aileron / elevator)',
        '  W / S   throttle up / down',
        '  A / D   rudder left / right',
        '  1–5     switches / bookmark (see above)',
        '  Space   centre sticks + throttle to 1000',
        '  V       toggle RC signal (simulate loss)',
        '  Q       quit',
        '=' * 63,
    ]
    sys.stdout.write('\r\n'.join(lines) + '\r\n')
    sys.stdout.flush()


# ── Key helpers ───────────────────────────────────────────────────────────────

def _clamp(val, step):
    return max(CH_MIN, min(CH_MAX, val + step))


def _cycle3(val):
    """Cycle a 3-position switch: 1000 → 1500 → 2000 → 1000."""
    if val <= 1000:
        return 1500
    if val <= 1500:
        return 2000
    return 1000


def _handle_key(ch, step):
    """Apply a single decoded character to _state['channels']."""
    with _lock:
        chs = _state['channels']

        # Sticks
        if   ch in ('w', 'W'):
            chs[CH_THROTTLE]  = _clamp(chs[CH_THROTTLE], +step)
        elif ch in ('s', 'S'):
            chs[CH_THROTTLE]  = _clamp(chs[CH_THROTTLE], -step)
        elif ch in ('a', 'A'):
            chs[CH_RUDDER]    = _clamp(chs[CH_RUDDER], -step)
        elif ch in ('d', 'D'):
            chs[CH_RUDDER]    = _clamp(chs[CH_RUDDER], +step)

        # Switches
        elif ch == '1':
            chs[CH_MODE]      = 2000 if chs[CH_MODE] == 1000 else 1000
        elif ch == '2':
            chs[CH_SPEED]     = _cycle3(chs[CH_SPEED])
        elif ch == '3':
            chs[CH_AUTO_TYPE] = _cycle3(chs[CH_AUTO_TYPE])
        elif ch == '4':
            chs[CH_GPS_LOG]   = 2000 if chs[CH_GPS_LOG] == 1000 else 1000
        elif ch == '5':
            chs[CH_BOOKMARK]  = CH_MAX
            _state['bookmark_until'] = time.monotonic() + 0.30

        # Centre / kill-switch
        elif ch == ' ':
            chs[CH_AILERON]   = CH_MID
            chs[CH_ELEVATOR]  = CH_MID
            chs[CH_RUDDER]    = CH_MID
            chs[CH_THROTTLE]  = CH_MIN   # throttle to safe bottom

        # Signal toggle
        elif ch in ('v', 'V'):
            _state['rc_valid'] = not _state['rc_valid']


def _handle_arrow(code, step):
    """Apply an arrow key escape code to _state['channels']."""
    with _lock:
        chs = _state['channels']
        if   code == b'A':   # up
            chs[CH_ELEVATOR] = _clamp(chs[CH_ELEVATOR], +step)
        elif code == b'B':   # down
            chs[CH_ELEVATOR] = _clamp(chs[CH_ELEVATOR], -step)
        elif code == b'C':   # right
            chs[CH_AILERON]  = _clamp(chs[CH_AILERON],  +step)
        elif code == b'D':   # left
            chs[CH_AILERON]  = _clamp(chs[CH_AILERON],  -step)


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="TX-16S iBUS receiver simulator — creates a PTY and emits iBUS packets"
    )
    parser.add_argument('--hz',   type=float, default=143,
                        help='Packet rate in Hz (default: 143 ≈ 7 ms/packet)')
    parser.add_argument('--step', type=int,   default=50,
                        help='Stick µs change per keypress (default: 50)')
    args = parser.parse_args()

    # Create PTY: write to master_fd → data readable from slave_fd (the PTY device)
    master_fd, slave_fd = pty.openpty()
    tty.setraw(slave_fd)   # disable tty byte-mangling on the slave side
    pty_path = os.ttyname(slave_fd)

    print(f'iBUS PTY   : {pty_path}', file=sys.stderr)
    print(f'rc_drive   : python3 rc_drive.py --ibus-port {pty_path}', file=sys.stderr)
    print(f'yukon_sim  : python3 tools/yukon_sim.py --ibus-port {pty_path}', file=sys.stderr)
    print('(press Q to quit)', file=sys.stderr)

    t = threading.Thread(target=_ibus_sender, args=(master_fd, args.hz), daemon=True)
    t.start()

    stdin_fd  = sys.stdin.fileno()
    old_attrs = termios.tcgetattr(stdin_fd)

    try:
        tty.setraw(stdin_fd)
        last_draw = 0.0
        esc_buf   = b''

        while True:
            now = time.monotonic()
            if now - last_draw >= 0.1:
                try:
                    draw(pty_path)
                except Exception:
                    pass
                last_draw = now

            r, _, _ = select.select([stdin_fd], [], [], 0.05)
            if not r:
                continue

            esc_buf += os.read(stdin_fd, 16)

            # Process buffered input byte by byte, handling escape sequences
            while esc_buf:
                if esc_buf.startswith(b'\x1b['):
                    if len(esc_buf) < 3:
                        break                         # wait for rest of sequence
                    _handle_arrow(esc_buf[2:3], args.step)
                    esc_buf = esc_buf[3:]
                    continue

                if esc_buf[0:1] == b'\x1b':
                    if len(esc_buf) < 2:
                        break                         # wait — could be start of CSI
                    esc_buf = esc_buf[1:]             # lone ESC, skip
                    continue

                b  = esc_buf[0]
                esc_buf = esc_buf[1:]

                if b in (ord('q'), ord('Q'), 0x03, 0x04):   # Q / Ctrl+C / Ctrl+D
                    with _lock:
                        _state['running'] = False
                    return

                _handle_key(chr(b), args.step)

    finally:
        termios.tcsetattr(stdin_fd, termios.TCSADRAIN, old_attrs)
        with _lock:
            _state['running'] = False
        os.close(master_fd)
        os.close(slave_fd)
        sys.stdout.write('\r\n\r\niBUS simulator stopped.\r\n')
        sys.stdout.flush()


if __name__ == '__main__':
    main()
