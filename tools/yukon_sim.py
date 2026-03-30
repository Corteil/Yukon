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
import random

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
CMD_BEARING    = 6   # value: 0–254 = target bearing (encoded 0–359°), 255 = disable
CMD_STRIP      = 7
CMD_PIXEL_SET  = 8
CMD_PIXEL_SHOW = 9
CMD_PATTERN    = 10
CMD_MODE       = 11
CMD_RC_QUERY   = 12

RESP_RC_BASE   = 8   # resp IDs 8–21 = RC channels 0–13, 22 = validity flag

RC_MANUAL = 0
RC_AUTO   = 1
RC_ESTOP  = 2
RC_MODE_NAMES = {RC_MANUAL: 'MANUAL', RC_AUTO: 'AUTO', RC_ESTOP: 'ESTOP'}

STRIP_COLOURS = [
    (  0,   0,   0),  # 0  off
    (255,   0,   0),  # 1  red
    (  0, 255,   0),  # 2  green
    (  0,   0, 255),  # 3  blue
    (255, 165,   0),  # 4  orange
    (255, 255,   0),  # 5  yellow
    (  0, 255, 255),  # 6  cyan
    (255,   0, 255),  # 7  magenta
    (255, 255, 255),  # 8  white
]
NUM_LEDS = 8
PAT_RATE = {1: 0.06, 2: 0.20, 3: 0.04, 4: 0.20, 5: 0.08, 6: 0.30}  # seconds/step
PAT_NAMES = {0: 'off', 1: 'larson', 2: 'random', 3: 'rainbow',
             4: 'retro_computer', 5: 'converge', 6: 'estop_flash'}

RESP_VOLTAGE = 0
RESP_CURRENT = 1
RESP_TEMP    = 2
RESP_TEMP_L  = 3
RESP_TEMP_R  = 4
RESP_FAULT_L = 5
RESP_FAULT_R = 6
RESP_HEADING = 7  # IMU heading, same encoding as CMD_BEARING; 255 = IMU absent
RESP_PITCH   = 8  # IMU pitch  (pitch+90)*254/180;  255 = absent
RESP_ROLL    = 9  # IMU roll   (roll+180)*254/360;   255 = absent

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
    # Mode (set by CMD_MODE heartbeat from Pi)
    'rc_mode'        : RC_MANUAL,
    'rc_channels'    : [1500] * 14,  # simulated RC channels (µs), default centre
    'rc_valid'       : True,         # simulated RC signal validity
    # IMU simulation
    'imu_present'    : True,    # toggle with I key
    'imu_heading'    : 0.0,     # current simulated heading (0–359°)
    'imu_pitch'      : 0.0,     # simulated pitch  (−90 … +90°, +ve = nose up)
    'imu_roll'       : 0.0,     # simulated roll   (−180 … +180°, +ve = right down)
    'bearing_target' : None,    # None = hold disabled; float = active target
    'last_imu_tick'  : 0.0,     # monotonic time of last heading update
    # LED strip simulation
    'strip_pixels'    : [(0, 0, 0)] * NUM_LEDS,
    'strip_pattern'   : 0,
    'strip_pat_pos'   : 0,
    'strip_pat_dir'   : 1,
    'strip_colour_idx': 8,       # default white
    'strip_last_tick' : 0.0,
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




def _hsv_to_rgb(h):
    """Hue 0-359 -> (r, g, b) at full saturation, half brightness."""
    h = int(h) % 360
    s = h // 60
    f = (h % 60) * 255 // 60
    q = 255 - f
    if s == 0: return 127, f // 2, 0
    if s == 1: return q // 2, 127, 0
    if s == 2: return 0, 127, f // 2
    if s == 3: return 0, q // 2, 127
    if s == 4: return f // 2, 0, 127
    return 127, 0, q // 2


def set_fault_leds():
    """Mirror Yukon firmware: pattern 6 + red when any motor fault is active, clear when none."""
    with _lock:
        any_fault = _state.get('fault_l', False) or _state.get('fault_r', False)
        if any_fault:
            _state['strip_pattern']    = 6
            _state['strip_colour_idx'] = 1   # red = Yukon hardware fault colour
            _state['strip_pat_pos']    = 0
            _state['strip_pat_dir']    = 1
        else:
            if _state['strip_pattern'] == 6 and _state['strip_colour_idx'] == 1:
                _state['strip_pattern'] = 0
                _state['strip_pixels']  = [(0, 0, 0)] * NUM_LEDS


def _tick_strip():
    """Advance LED strip pattern animation (call periodically)."""
    now = time.monotonic()
    with _lock:
        pattern = _state['strip_pattern']
        if pattern == 0:
            _state['strip_last_tick'] = now
            return
        rate = PAT_RATE.get(pattern, 0.20)
        if now - _state['strip_last_tick'] < rate:
            return
        _state['strip_last_tick'] = now

        pixels    = list(_state['strip_pixels'])
        col_idx   = _state['strip_colour_idx']
        pos       = _state['strip_pat_pos']
        direction = _state['strip_pat_dir']
        r, g, b   = STRIP_COLOURS[col_idx]

        if pattern == 1:                                   # larson
            for i in range(NUM_LEDS):
                d = abs(i - pos)
                if d == 0:   pixels[i] = (255, 0, 0)
                elif d == 1: pixels[i] = (48,  0, 0)
                elif d == 2: pixels[i] = (8,   0, 0)
                else:        pixels[i] = (0,   0, 0)
            pos += direction
            if pos >= NUM_LEDS - 1: direction = -1
            elif pos <= 0:          direction =  1

        elif pattern == 2:                                 # random
            for i in range(NUM_LEDS):
                if random.getrandbits(1):
                    pixels[i] = STRIP_COLOURS[random.randint(1, len(STRIP_COLOURS) - 1)]
                else:
                    pixels[i] = (0, 0, 0)

        elif pattern == 3:                                 # rainbow
            for i in range(NUM_LEDS):
                pixels[i] = _hsv_to_rgb((pos + i * (360 // NUM_LEDS)) % 360)
            pos = (pos + 8) % 360

        elif pattern == 4:                                 # retro_computer
            for i in range(NUM_LEDS):
                pixels[i] = (r, g, b) if random.getrandbits(1) else (0, 0, 0)

        elif pattern == 5:                                 # converge
            fill = pos
            for i in range(NUM_LEDS):
                pixels[i] = (r, g, b) if (i < fill or i >= NUM_LEDS - fill) else (0, 0, 0)
            pos += direction
            if pos > NUM_LEDS // 2:
                direction = -1
                pos = NUM_LEDS // 2
            elif pos < 0:
                direction = 1
                pos = 0

        elif pattern == 6:                                 # estop_flash
            # LEDs 0-2 and 5-7 flash red; LEDs 3-4 show fault colour
            flash = (255, 0, 0) if pos == 0 else (0, 0, 0)
            fault_col = STRIP_COLOURS[col_idx]
            for i in range(3):
                pixels[i] = flash
            pixels[3] = fault_col
            pixels[4] = fault_col
            for i in range(5, NUM_LEDS):
                pixels[i] = flash
            pos = 1 - pos                                  # toggle 0/1

        _state['strip_pixels']   = pixels
        _state['strip_pat_pos']  = pos
        _state['strip_pat_dir']  = direction


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
            if 0x21 <= b <= 0x2C:          # commands 1–12 (0x21–0x2C)
                pkt_cmd = b
                sm = 'V_HIGH'
            else:
                os.write(master_fd, bytes([NAK]))
                sm = 'SYNC'

        elif sm == 'V_HIGH':
            if 0x40 <= b <= 0x4F:
                pkt_vhigh = b
                sm = 'V_LOW'
            else:
                os.write(master_fd, bytes([NAK]))
                sm = 'SYNC'

        elif sm == 'V_LOW':
            if 0x50 <= b <= 0x5F:
                pkt_vlow = b
                sm = 'CHK'
            else:
                os.write(master_fd, bytes([NAK]))
                sm = 'SYNC'

        elif sm == 'CHK':
            expected = pkt_cmd ^ pkt_vhigh ^ pkt_vlow
            if b != expected:
                os.write(master_fd, bytes([NAK]))
            else:
                cmd_code = pkt_cmd - 0x20
                value    = ((pkt_vhigh - 0x40) << 4) | (pkt_vlow - 0x50)
                with _lock:
                    _state['cmds_rx'] += 1
                    if cmd_code == CMD_LEFT:
                        if _state['rc_mode'] == RC_AUTO:   # AUTO-only; ignored in MANUAL/ESTOP
                            _state['left_byte'] = value
                    elif cmd_code == CMD_RIGHT:
                        if _state['rc_mode'] == RC_AUTO:
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
                            os.write(master_fd, bytes([NAK]))
                            sm = 'SYNC'
                            continue
                    elif cmd_code == CMD_STRIP:
                        _state['strip_pattern'] = 0
                        idx = min(value, len(STRIP_COLOURS) - 1)
                        _state['strip_pixels'] = [STRIP_COLOURS[idx]] * NUM_LEDS

                    elif cmd_code == CMD_PIXEL_SET:
                        led_idx    = (value >> 4) & 0x0F
                        colour_idx = value & 0x0F
                        if led_idx < NUM_LEDS and colour_idx < len(STRIP_COLOURS):
                            pixels = list(_state['strip_pixels'])
                            pixels[led_idx] = STRIP_COLOURS[colour_idx]
                            _state['strip_pixels'] = pixels

                    elif cmd_code == CMD_PIXEL_SHOW:
                        pass   # pixels already staged in state

                    elif cmd_code == CMD_PATTERN:
                        colour_nibble = (value >> 4) & 0x0F
                        pat           = value & 0x0F
                        if colour_nibble > 0 and colour_nibble < len(STRIP_COLOURS):
                            _state['strip_colour_idx'] = colour_nibble
                        _state['strip_pattern']  = pat if pat <= 6 else 0
                        _state['strip_pat_pos']  = 0
                        _state['strip_pat_dir']  = 1
                        if pat == 0:
                            _state['strip_pixels'] = [(0, 0, 0)] * NUM_LEDS

                    elif cmd_code == CMD_MODE:
                        _state['rc_mode'] = value
                        if value == RC_ESTOP:
                            _state['left_byte']      = 0
                            _state['right_byte']     = 0
                            _state['bearing_target'] = None

                    elif cmd_code == CMD_RC_QUERY:
                        # 14 channel packets + 1 validity packet then ACK
                        channels = list(_state['rc_channels'])
                        valid    = int(_state['rc_valid'])
                        for i, us in enumerate(channels):
                            _send_sensor_packet(master_fd, RESP_RC_BASE + i,
                                                (us - 1000) // 5)
                        _send_sensor_packet(master_fd, RESP_RC_BASE + 14, valid)

                    elif cmd_code == CMD_SENSOR:
                        _send_sensor_packet(master_fd, RESP_VOLTAGE, SIM_VOLTAGE  * 10)
                        _send_sensor_packet(master_fd, RESP_CURRENT, SIM_CURRENT  * 100)
                        _send_sensor_packet(master_fd, RESP_TEMP,    SIM_TEMP     * 3)
                        _send_sensor_packet(master_fd, RESP_TEMP_L,  SIM_TEMP_MOD * 3)
                        _send_sensor_packet(master_fd, RESP_TEMP_R,  SIM_TEMP_MOD * 3)
                        _send_sensor_packet(master_fd, RESP_FAULT_L, int(_state.get('fault_l', False)))
                        _send_sensor_packet(master_fd, RESP_FAULT_R, int(_state.get('fault_r', False)))
                        # RESP_HEADING/PITCH/ROLL: send encoded values, or 255 if IMU absent
                        if _state['imu_present']:
                            _send_sensor_packet(master_fd, RESP_HEADING,
                                                _bearing_encode(_state['imu_heading']))
                            _send_sensor_packet(master_fd, RESP_PITCH,
                                int((_state['imu_pitch'] + 90.0)  * 254.0 / 180.0 + 0.5))
                            _send_sensor_packet(master_fd, RESP_ROLL,
                                int((_state['imu_roll']  + 180.0) * 254.0 / 360.0 + 0.5))
                        else:
                            _send_sensor_packet(master_fd, RESP_HEADING, 255)
                            _send_sensor_packet(master_fd, RESP_PITCH,   255)
                            _send_sensor_packet(master_fd, RESP_ROLL,    255)
                os.write(master_fd, bytes([ACK]))
            sm = 'SYNC'



# ---------------------------------------------------------------------------
# Display
# ---------------------------------------------------------------------------


def draw(yukon_path):
    with _lock:
        lb           = _state['left_byte']
        rb           = _state['right_byte']
        led_a        = _state['led_a']
        led_b        = _state['led_b']
        cmds         = _state['cmds_rx']
        rc_mode      = _state['rc_mode']
        imu_present  = _state['imu_present']
        imu_heading  = _state['imu_heading']
        imu_pitch    = _state['imu_pitch']
        imu_roll     = _state['imu_roll']
        bearing_tgt  = _state['bearing_target']
        strip_pixels = list(_state['strip_pixels'])
        strip_pat    = _state['strip_pattern']

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

    mode_str = RC_MODE_NAMES.get(rc_mode, str(rc_mode))

    lines = [
        '\033[2J\033[H',
        '=== Yukon Simulator ' + '=' * 42,
        f'  Yukon port : {yukon_path}',
        f'  Connect any client with --port {yukon_path}',
        f'  Mode       : {mode_str}',
        '--- Motor commands received ' + '-' * 34,
        speed_line('Left ', rx_l),
        speed_line('Right', rx_r),
        '  (CMD_LEFT/RIGHT only applied in AUTO mode)',
        '--- IMU ' + '-' * 54,
        f'  IMU      : {imu_str}',
        f'  Heading  : {compass_bar(imu_heading) if imu_present else "---"}',
        f'  Pitch    : {imu_pitch:+.1f}°  Roll: {imu_roll:+.1f}°' if imu_present else '  Pitch    : ---  Roll: ---',
        f'  Brg hold : {tgt_str}{err_str}',
        f'  Keys     : < decrease heading   > increase heading   I toggle IMU',
        '--- Status ' + '-' * 51,
        f'  LED A: {"ON " if led_a else "OFF"}  LED B: {"ON " if led_b else "OFF"}'
        f'  Cmds rx: {cmds}',
        '--- LED Strip ' + '-' * 48,
        '  ' + ''.join(
            f'\033[48;2;{r};{g};{b}m  \033[0m' for r, g, b in strip_pixels
        ),
        f'  pattern: {PAT_NAMES.get(strip_pat, "?")}',
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
            _tick_strip()

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
