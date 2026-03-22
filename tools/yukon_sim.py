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

SYNC        = 0x7E
ACK         = 0x06
NAK         = 0x15
CMD_LED     = 1
CMD_LEFT    = 2
CMD_RIGHT   = 3
CMD_KILL    = 4
CMD_SENSOR  = 5
CMD_BEARING = 6   # value: 0–254 = target bearing (encoded 0–359°), 255 = disable

RESP_VOLTAGE = 0
RESP_CURRENT = 1
RESP_TEMP    = 2
RESP_TEMP_L  = 3
RESP_TEMP_R  = 4
RESP_FAULT_L = 5
RESP_FAULT_R = 6
RESP_HEADING = 7  # IMU heading, same encoding as CMD_BEARING; 255 = IMU absent

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

# IMU simulation
IMU_HEADING_STEP = 5.0    # degrees per keypress (< / >)
IMU_DRIFT_RATE   = 90.0   # degrees/second the sim heading moves toward bearing target

# ---------------------------------------------------------------------------
# Shared state  (all access protected by _lock)
# ---------------------------------------------------------------------------

_lock  = threading.Lock()
_state = {
    'left_byte'      : None,    # last CMD_LEFT  byte received
    'right_byte'     : None,    # last CMD_RIGHT byte received
    'led_a'          : False,
    'led_b'          : False,
    'cmds_rx'        : 0,       # total Yukon commands received
    'running'        : True,
    # IMU simulation
    'imu_present'    : True,    # toggle with I key
    'imu_heading'    : 0.0,     # current simulated heading (0–359°)
    'bearing_target' : None,    # None = hold disabled; float = active target
    'last_imu_tick'  : 0.0,     # monotonic time of last heading update
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


def _bearing_encode(degrees):
    """Encode degrees 0–359 → byte 0–254 (255 reserved for absent/disabled)."""
    return min(254, int(degrees * 254.0 / 359.0))


def _bearing_decode(value):
    """Decode protocol byte 0–254 → degrees 0–359."""
    return value * 359.0 / 254.0


def _angle_diff(target, current):
    """Signed shortest-arc difference (target − current), range −180..+180."""
    return (target - current + 180.0) % 360.0 - 180.0


def _tick_imu():
    """Advance simulated IMU heading toward bearing_target (call periodically)."""
    now = time.monotonic()
    with _lock:
        if not _state['imu_present']:
            _state['last_imu_tick'] = now
            return
        dt = now - _state['last_imu_tick']
        _state['last_imu_tick'] = now
        target = _state['bearing_target']
        if target is not None and dt > 0:
            err  = _angle_diff(target, _state['imu_heading'])
            step = IMU_DRIFT_RATE * dt
            if abs(err) <= step:
                _state['imu_heading'] = target % 360.0
            else:
                _state['imu_heading'] = (
                    _state['imu_heading'] + step * (1 if err > 0 else -1)
                ) % 360.0




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
            if 0x21 <= b <= 0x27:          # commands 1–7 (0x21–0x27)
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
                        _state['left_byte']     = 0
                        _state['right_byte']    = 0
                        _state['bearing_target'] = None
                    elif cmd_code == CMD_LED:
                        if   value == 0: _state['led_a'] = False
                        elif value == 1: _state['led_a'] = True
                        elif value == 2: _state['led_b'] = False
                        elif value == 3: _state['led_b'] = True
                    elif cmd_code == CMD_BEARING:
                        if value == 255:
                            _state['bearing_target'] = None
                        elif _state['imu_present']:
                            _state['bearing_target'] = _bearing_decode(value)
                        else:
                            # NAK if no IMU
                            os.write(master_fd, bytes([NAK, ord('\n')]))
                            sm = 'SYNC'
                            continue
                    elif cmd_code == CMD_SENSOR:
                        _send_sensor_packet(master_fd, RESP_VOLTAGE, SIM_VOLTAGE  * 10)
                        _send_sensor_packet(master_fd, RESP_CURRENT, SIM_CURRENT  * 100)
                        _send_sensor_packet(master_fd, RESP_TEMP,    SIM_TEMP     * 3)
                        _send_sensor_packet(master_fd, RESP_TEMP_L,  SIM_TEMP_MOD * 3)
                        _send_sensor_packet(master_fd, RESP_TEMP_R,  SIM_TEMP_MOD * 3)
                        _send_sensor_packet(master_fd, RESP_FAULT_L, int(_state.get('fault_l', False)))
                        _send_sensor_packet(master_fd, RESP_FAULT_R, int(_state.get('fault_r', False)))
                        # RESP_HEADING: send encoded heading, or 255 if IMU absent
                        if _state['imu_present']:
                            _send_sensor_packet(master_fd, RESP_HEADING,
                                                _bearing_encode(_state['imu_heading']))
                        else:
                            _send_sensor_packet(master_fd, RESP_HEADING, 255)
                os.write(master_fd, bytes([ACK, ord('\n')]))
            sm = 'SYNC'



# ---------------------------------------------------------------------------
# Display
# ---------------------------------------------------------------------------


def draw(yukon_path):
    with _lock:
        lb          = _state['left_byte']
        rb          = _state['right_byte']
        led_a       = _state['led_a']
        led_b       = _state['led_b']
        cmds        = _state['cmds_rx']
        imu_present = _state['imu_present']
        imu_heading = _state['imu_heading']
        bearing_tgt = _state['bearing_target']

    rx_l = _decode_speed(lb)
    rx_r = _decode_speed(rb)

    def speed_line(label, rx):
        if rx is None:
            return f'  {label}  {_bar(None)}  (waiting...)'
        return f'  {label}  {_bar(rx)}  {rx:+.2f}'

    # IMU compass bar: 0–359° mapped to a 36-char wide bar
    def compass_bar(heading, width=36):
        pos = int(round(heading / 360.0 * width)) % width
        bar = ['-'] * width
        bar[pos] = 'N' if (heading < 5 or heading > 355) else '+'
        return '[' + ''.join(bar) + f']  {heading:6.1f}°'

    imu_str    = 'PRESENT' if imu_present else 'ABSENT (toggle: I)'
    tgt_str    = f'{bearing_tgt:.1f}°' if bearing_tgt is not None else 'off'
    err_str    = ''
    if imu_present and bearing_tgt is not None:
        err = _angle_diff(bearing_tgt, imu_heading)
        err_str = f'  err: {err:+.1f}°'

    lines = [
        '\033[2J\033[H',
        '=== Yukon Simulator ' + '=' * 42,
        f'  Yukon port : {yukon_path}',
        f'  Connect any client with --port {yukon_path}',
        '--- Motor commands received ' + '-' * 34,
        speed_line('Left ', rx_l),
        speed_line('Right', rx_r),
        '--- IMU ' + '-' * 54,
        f'  IMU      : {imu_str}',
        f'  Heading  : {compass_bar(imu_heading) if imu_present else "---"}',
        f'  Brg hold : {tgt_str}{err_str}',
        f'  Keys     : < decrease heading   > increase heading   I toggle IMU',
        '--- Status ' + '-' * 51,
        f'  LED A: {"ON " if led_a else "OFF"}  LED B: {"ON " if led_b else "OFF"}'
        f'  Cmds rx: {cmds}',
        '--- Keys ' + '-' * 53,
        '  Q  quit',
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

    # Initialise IMU tick timestamp
    with _lock:
        _state['last_imu_tick'] = time.monotonic()

    # Save terminal state; enter raw mode for single-keypress input
    stdin_fd  = sys.stdin.fileno()
    old_attrs = termios.tcgetattr(stdin_fd)
    try:
        tty.setraw(stdin_fd)
        last_draw = 0.0

        while True:
            now = time.monotonic()

            # Tick IMU heading toward bearing target
            _tick_imu()

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
                ch = chr(b)
                if ch in ('q', 'Q', '\x03', '\x04'):   # Q, Ctrl+C, Ctrl+D
                    with _lock:
                        _state['running'] = False
                    return
                elif ch == '>':    # increase heading
                    with _lock:
                        if _state['imu_present']:
                            _state['imu_heading'] = (
                                _state['imu_heading'] + IMU_HEADING_STEP
                            ) % 360.0
                elif ch == '<':    # decrease heading
                    with _lock:
                        if _state['imu_present']:
                            _state['imu_heading'] = (
                                _state['imu_heading'] - IMU_HEADING_STEP
                            ) % 360.0
                elif ch in ('i', 'I'):   # toggle IMU present
                    with _lock:
                        _state['imu_present'] = not _state['imu_present']
                        if not _state['imu_present']:
                            _state['bearing_target'] = None

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
