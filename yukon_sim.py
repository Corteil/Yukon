#!/usr/bin/env python3
"""
yukon_sim.py — Yukon USB serial simulator

Creates a virtual serial port (PTY) that emulates the Yukon (/dev/ttyACM0):
  - Parses 5-byte CMD packets and responds with ACK/NAK
  - Returns simulated sensor data for CMD_SENSOR

Any client that speaks the Yukon serial protocol can connect to the PTY port
(rc_drive.py, robot.py, test_main.py, etc.).

Usage:
    python3 yukon_sim.py
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

# ---------------------------------------------------------------------------
# Protocol constants
# ---------------------------------------------------------------------------

SYNC       = 0x7E
ACK        = 0x06
NAK        = 0x15
CMD_LED    = 1
CMD_LEFT   = 2
CMD_RIGHT  = 3
CMD_KILL   = 4
CMD_SENSOR = 5

RESP_VOLTAGE = 0
RESP_CURRENT = 1
RESP_TEMP    = 2
RESP_TEMP_L  = 3
RESP_TEMP_R  = 4
RESP_FAULT_L = 5
RESP_FAULT_R = 6

# ---------------------------------------------------------------------------
# iBUS constants
# ---------------------------------------------------------------------------


# ---------------------------------------------------------------------------
# Simulated sensor values (returned for CMD_SENSOR)
# ---------------------------------------------------------------------------

SIM_VOLTAGE  = 12.0   # V
SIM_CURRENT  = 0.5    # A
SIM_TEMP     = 25.0   # °C board
SIM_TEMP_MOD = 23.0   # °C motor modules

# ---------------------------------------------------------------------------
# Shared state  (all access protected by _lock)
# ---------------------------------------------------------------------------

_lock  = threading.Lock()
_state = {
    'left_byte' : None,     # last CMD_LEFT  byte received from rc_drive.py
    'right_byte': None,     # last CMD_RIGHT byte received from rc_drive.py
    'led_a'     : False,
    'led_b'     : False,
    'cmds_rx'   : 0,        # total Yukon commands received
    'running'   : True,
}

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _decode_speed(b):
    """Wire byte → float speed (-1.0 .. +1.0). None if not received yet."""
    if b is None:
        return None
    if b > 100:
        return -((b - 100) / 100.0)
    return b / 100.0


def _bar(val, width=30):
    """
    Render a -1..+1 value as a bar, e.g.:
      [ negative ███████│              ]
      [          │███████ positive     ]
      [          │                     ]  (zero)
      [??????????│??????????           ]  (None)
    """
    half = width // 2
    bar  = [' '] * width
    bar[half] = '|'
    if val is None:
        bar[half] = '?'
    else:
        pos = int(round(val * half))
        if pos > 0:
            for i in range(half + 1, min(half + 1 + pos, width)):
                bar[i] = '#'
        elif pos < 0:
            for i in range(max(0, half + pos), half):
                bar[i] = '#'
    return '[' + ''.join(bar) + ']'




# ---------------------------------------------------------------------------
# Yukon PTY server  (background thread)
# ---------------------------------------------------------------------------

def _send_sensor_packet(fd, resp_id, value):
    value  = max(0, min(255, int(value)))
    rtype  = resp_id + 0x30
    v_high = (value >> 4)   + 0x40
    v_low  = (value & 0x0F) + 0x50
    chk    = rtype ^ v_high ^ v_low
    os.write(fd, bytes([SYNC, rtype, v_high, v_low, chk]))


def yukon_server(master_fd):
    """Parse the 5-byte protocol from rc_drive.py and respond ACK/NAK."""
    sm = 'SYNC'
    pkt_cmd = pkt_vhigh = pkt_vlow = 0

    while True:
        with _lock:
            if not _state['running']:
                break
        try:
            r, _, _ = select.select([master_fd], [], [], 0.05)
            if not r:
                continue
            data = os.read(master_fd, 1)
        except OSError:
            break
        if not data:
            continue
        b = data[0]

        if b == SYNC:
            sm = 'CMD'
            continue

        if sm == 'CMD':
            if 0x21 <= b <= 0x25:
                pkt_cmd = b
                sm = 'V_HIGH'
            else:
                os.write(master_fd, bytes([NAK, ord('\n')]))
                sm = 'SYNC'

        elif sm == 'V_HIGH':
            if 0x40 <= b <= 0x4F:
                pkt_vhigh = b
                sm = 'V_LOW'
            else:
                os.write(master_fd, bytes([NAK, ord('\n')]))
                sm = 'SYNC'

        elif sm == 'V_LOW':
            if 0x50 <= b <= 0x5F:
                pkt_vlow = b
                sm = 'CHK'
            else:
                os.write(master_fd, bytes([NAK, ord('\n')]))
                sm = 'SYNC'

        elif sm == 'CHK':
            expected = pkt_cmd ^ pkt_vhigh ^ pkt_vlow
            if b != expected:
                os.write(master_fd, bytes([NAK, ord('\n')]))
            else:
                cmd_code = pkt_cmd - 0x20
                value    = ((pkt_vhigh - 0x40) << 4) | (pkt_vlow - 0x50)
                with _lock:
                    _state['cmds_rx'] += 1
                    if cmd_code == CMD_LEFT:
                        _state['left_byte'] = value
                    elif cmd_code == CMD_RIGHT:
                        _state['right_byte'] = value
                    elif cmd_code == CMD_KILL:
                        _state['left_byte']  = 0
                        _state['right_byte'] = 0
                    elif cmd_code == CMD_LED:
                        if   value == 0: _state['led_a'] = False
                        elif value == 1: _state['led_a'] = True
                        elif value == 2: _state['led_b'] = False
                        elif value == 3: _state['led_b'] = True
                    elif cmd_code == CMD_SENSOR:
                        _send_sensor_packet(master_fd, RESP_VOLTAGE, SIM_VOLTAGE  * 10)
                        _send_sensor_packet(master_fd, RESP_CURRENT, SIM_CURRENT  * 100)
                        _send_sensor_packet(master_fd, RESP_TEMP,    SIM_TEMP     * 3)
                        _send_sensor_packet(master_fd, RESP_TEMP_L,  SIM_TEMP_MOD * 3)
                        _send_sensor_packet(master_fd, RESP_TEMP_R,  SIM_TEMP_MOD * 3)
                        _send_sensor_packet(master_fd, RESP_FAULT_L, 0)
                        _send_sensor_packet(master_fd, RESP_FAULT_R, 0)
                os.write(master_fd, bytes([ACK, ord('\n')]))
            sm = 'SYNC'



# ---------------------------------------------------------------------------
# Display
# ---------------------------------------------------------------------------


def draw(yukon_path):
    with _lock:
        lb    = _state['left_byte']
        rb    = _state['right_byte']
        led_a = _state['led_a']
        led_b = _state['led_b']
        cmds  = _state['cmds_rx']

    rx_l = _decode_speed(lb)
    rx_r = _decode_speed(rb)

    def speed_line(label, rx):
        if rx is None:
            return f'  {label}  {_bar(None)}  (waiting...)'
        return f'  {label}  {_bar(rx)}  {rx:+.2f}'

    lines = [
        '\033[2J\033[H',
        '=== Yukon Simulator ' + '=' * 42,
        f'  Yukon port : {yukon_path}',
        f'  Connect any client with --port {yukon_path}',
        '--- Motor commands received ' + '-' * 34,
        speed_line('Left ', rx_l),
        speed_line('Right', rx_r),
        '--- Status ' + '-' * 51,
        f'  LED A: {"ON " if led_a else "OFF"}  LED B: {"ON " if led_b else "OFF"}'
        f'  Cmds rx: {cmds}',
        '=' * 62,
    ]
    sys.stdout.write('\r\n'.join(lines) + '\r\n')
    sys.stdout.flush()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Yukon serial simulator")
    args = parser.parse_args()

    # Yukon PTY (emulates /dev/ttyACM0)
    yukon_master, yukon_slave = pty.openpty()
    tty.setraw(yukon_slave)
    yukon_path = os.ttyname(yukon_slave)

    # Print command before entering raw mode
    print(f'Yukon PTY  : {yukon_path}', file=sys.stderr)
    print(f'Connect with --port {yukon_path}', file=sys.stderr)
    print('(press Q to quit)', file=sys.stderr)

    # Launch Yukon protocol server thread
    t_yukon = threading.Thread(target=yukon_server, args=(yukon_master,), daemon=True)
    t_yukon.start()

    # Save terminal state; enter raw mode for single-keypress input
    stdin_fd  = sys.stdin.fileno()
    old_attrs = termios.tcgetattr(stdin_fd)
    try:
        tty.setraw(stdin_fd)
        last_draw = 0.0

        while True:
            now = time.monotonic()
            if now - last_draw >= 0.1:
                try:
                    draw(yukon_path)
                except Exception:
                    pass   # never let a display error kill the sim
                last_draw = now

            r, _, _ = select.select([stdin_fd], [], [], 0.05)
            if not r:
                continue

            data = os.read(stdin_fd, 16)
            for b in data:
                if chr(b) in ('q', 'Q', '\x03', '\x04'):   # Q, Ctrl+C, Ctrl+D
                    with _lock:
                        _state['running'] = False
                    return

    finally:
        termios.tcsetattr(stdin_fd, termios.TCSADRAIN, old_attrs)
        with _lock:
            _state['running'] = False
        os.close(yukon_master)
        os.close(yukon_slave)
        sys.stdout.write('\r\n\r\nSimulator stopped.\r\n')
        sys.stdout.flush()


if __name__ == '__main__':
    main()
