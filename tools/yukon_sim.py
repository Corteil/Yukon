#!/usr/bin/env python3
"""
yukon_sim.py — Yukon USB serial simulator

Creates a virtual serial port (PTY) that emulates the Yukon (/dev/ttyACM0):
  - Parses 5-byte CMD packets and responds with ACK/NAK
  - Returns simulated sensor data for CMD_SENSOR and RC channels for CMD_RC_QUERY
  - Tracks whether the Pi is in --no-motors mode (queries RC but no CMD_MODE heartbeat)
    and suppresses iBUS motor drive accordingly — mirrors main.py behaviour

Modes (--mode, default: gui)
  gui       Pygame GUI with compass, motor bars, sliders, fault injection
  web       Browser dashboard at :5002 (useful for headless Pi)
  headless  Terminal display with keyboard controls

Usage
-----
  python3 yukon_sim.py                        # GUI mode
  python3 yukon_sim.py --mode gui
  python3 yukon_sim.py --mode web [--port 5002]
  python3 yukon_sim.py --mode headless
"""

import json
import math
import os
import pty
import select
import sys
import termios
import threading
import time
import tty
import argparse
import random

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# ---------------------------------------------------------------------------
# Protocol constants
# ---------------------------------------------------------------------------

SYNC           = 0x7E
ACK            = 0x06
NAK            = 0x15
CMD_LED        = 1
CMD_LEFT       = 2
CMD_RIGHT      = 3
CMD_KILL       = 4
CMD_SENSOR     = 5
CMD_BEARING    = 6
CMD_STRIP      = 7
CMD_PIXEL_SET  = 8
CMD_PIXEL_SHOW = 9
CMD_PATTERN    = 10
CMD_MODE       = 11
CMD_RC_QUERY   = 12
CMD_BENCH      = 13

RESP_RC_BASE    = 8   # resp IDs 8–21 = RC channels 0–13, 22 = validity flag
RESP_BENCH_TEMP  = 10
RESP_BENCH_FAULT = 11

RC_MANUAL    = 0
RC_AUTO      = 1
RC_ESTOP     = 2
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
NUM_LEDS  = 8
PAT_RATE  = {1: 0.06, 2: 0.20, 3: 0.04, 4: 0.20, 5: 0.08, 6: 0.30}
PAT_NAMES = {0: 'off', 1: 'larson', 2: 'random', 3: 'rainbow',
             4: 'retro_computer', 5: 'converge', 6: 'estop_flash'}

RESP_VOLTAGE    = 0
RESP_CURRENT    = 1
RESP_TEMP       = 2
RESP_TEMP_L     = 3
RESP_TEMP_R     = 4
RESP_FAULT_L    = 5
RESP_FAULT_R    = 6
RESP_HEADING    = 7
RESP_PITCH      = 8
RESP_ROLL       = 9

# iBUS constants (match main.py)
IBUS_CH_THROTTLE = 2
IBUS_CH_STEER    = 0
IBUS_CH_SPEED    = 5
IBUS_DEADZONE    = 30
IBUS_SPEED_MIN   = 0.25

# ---------------------------------------------------------------------------
# Simulated sensor values (mutable globals — GUI sliders write these)
# ---------------------------------------------------------------------------

SIM_VOLTAGE  = 12.0
SIM_CURRENT  = 0.5
SIM_TEMP     = 25.0
SIM_TEMP_MOD = 23.0

IMU_HEADING_STEP = 5.0
IMU_DRIFT_RATE   = 90.0   # degrees/s toward bearing target

# ---------------------------------------------------------------------------
# Shared state  (all access protected by _lock)
# ---------------------------------------------------------------------------

_lock  = threading.Lock()
_state = {
    'left_byte'           : None,
    'right_byte'          : None,
    'led_a'               : False,
    'led_b'               : False,
    'cmds_rx'             : 0,
    'running'             : True,
    'rc_mode'             : RC_MANUAL,
    'rc_channels'         : [1500] * 14,
    'rc_valid'            : True,
    'pi_last_cmd_ms'      : 0.0,   # monotonic time of last CMD_MODE from Pi
    'pi_last_rc_query_ms' : 0.0,   # monotonic time of last CMD_RC_QUERY from Pi
    'imu_present'         : True,
    'imu_heading'         : 0.0,
    'imu_pitch'           : 0.0,
    'imu_roll'            : 0.0,
    'bearing_target'      : None,
    'last_imu_tick'       : 0.0,
    'fault_l'             : False,
    'fault_r'             : False,
    'bench_enabled'       : False,
    'bench_fault'         : False,
    'strip_pixels'        : [(0, 0, 0)] * NUM_LEDS,
    'strip_pattern'       : 0,
    'strip_pat_pos'       : 0,
    'strip_pat_dir'       : 1,
    'strip_colour_idx'    : 8,
    'strip_last_tick'     : 0.0,
    'ibus_port'           : None,    # set by start_ibus_reader()
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


def _encode_speed(v):
    """Float speed -1.0..+1.0 → wire byte 0–200."""
    if v >= 0:
        return min(100, int(round(v * 100)))
    return min(200, int(round(-v * 100)) + 100)


def _ibus_tank_mix(ch):
    """iBUS channel list → (left, right) motor speeds -1.0..+1.0 (mirrors main.py)."""
    def norm(v):
        raw = v - 1500
        if abs(raw) < IBUS_DEADZONE:
            return 0.0
        return max(-1.0, min(1.0, raw / 500.0))
    thr   = norm(ch[IBUS_CH_THROTTLE])
    ste   = norm(ch[IBUS_CH_STEER])
    t     = max(0.0, min(1.0, (ch[IBUS_CH_SPEED] - 1000) / 1000.0))
    scale = IBUS_SPEED_MIN + t * (1.0 - IBUS_SPEED_MIN)
    return (max(-1.0, min(1.0, thr - ste)) * scale,
            max(-1.0, min(1.0, thr + ste)) * scale)


def _bar(val, width=30):
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
    return min(254, int(degrees * 254.0 / 359.0))


def _bearing_decode(value):
    return value * 359.0 / 254.0


def _angle_diff(target, current):
    return (target - current + 180.0) % 360.0 - 180.0


# ---------------------------------------------------------------------------
# Periodic ticks  (called from display loop in each mode)
# ---------------------------------------------------------------------------

def _tick_imu():
    now = time.monotonic()
    with _lock:
        if not _state['imu_present']:
            _state['last_imu_tick'] = now
            return
        dt     = now - _state['last_imu_tick']
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


def _tick_rc_motors():
    """Update motor speeds from RC channels in MANUAL mode — mirrors main.py _decode_ibus.

    No-motors detection: Pi is sending CMD_RC_QUERY but not CMD_MODE heartbeats.
    In that case suppress iBUS motor drive and hold motors at zero.
    """
    with _lock:
        rc_mode = _state['rc_mode']
        if rc_mode == RC_ESTOP:
            _state['left_byte']  = 0
            _state['right_byte'] = 0
            return
        if rc_mode == RC_AUTO:
            return  # set by CMD_LEFT/CMD_RIGHT
        # MANUAL: check no-motors mode
        pi_no_motors = (
            _state['pi_last_rc_query_ms'] != 0.0 and
            _state['pi_last_cmd_ms'] == 0.0
        )
        if pi_no_motors:
            _state['left_byte']  = 0
            _state['right_byte'] = 0
            return
        ch = list(_state['rc_channels'])
    left, right = _ibus_tank_mix(ch)
    with _lock:
        _state['left_byte']  = _encode_speed(left)
        _state['right_byte'] = _encode_speed(right)


def _tick_strip():
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

        if pattern == 1:
            for i in range(NUM_LEDS):
                d = abs(i - pos)
                if d == 0:   pixels[i] = (255, 0, 0)
                elif d == 1: pixels[i] = (48,  0, 0)
                elif d == 2: pixels[i] = (8,   0, 0)
                else:        pixels[i] = (0,   0, 0)
            pos += direction
            if pos >= NUM_LEDS - 1: direction = -1
            elif pos <= 0:          direction =  1

        elif pattern == 2:
            for i in range(NUM_LEDS):
                if random.getrandbits(1):
                    pixels[i] = STRIP_COLOURS[random.randint(1, len(STRIP_COLOURS) - 1)]
                else:
                    pixels[i] = (0, 0, 0)

        elif pattern == 3:
            def _hsv(h):
                h = int(h) % 360; s = h // 60; f = (h % 60) * 255 // 60; q = 255 - f
                if s == 0: return 127, f // 2, 0
                if s == 1: return q // 2, 127, 0
                if s == 2: return 0, 127, f // 2
                if s == 3: return 0, q // 2, 127
                if s == 4: return f // 2, 0, 127
                return 127, 0, q // 2
            for i in range(NUM_LEDS):
                pixels[i] = _hsv((pos + i * (360 // NUM_LEDS)) % 360)
            pos = (pos + 8) % 360

        elif pattern == 4:
            for i in range(NUM_LEDS):
                pixels[i] = (r, g, b) if random.getrandbits(1) else (0, 0, 0)

        elif pattern == 5:
            fill = pos
            for i in range(NUM_LEDS):
                pixels[i] = (r, g, b) if (i < fill or i >= NUM_LEDS - fill) else (0, 0, 0)
            pos += direction
            if pos > NUM_LEDS // 2:
                direction = -1; pos = NUM_LEDS // 2
            elif pos < 0:
                direction = 1;  pos = 0

        elif pattern == 6:
            flash     = (255, 0, 0) if pos == 0 else (0, 0, 0)
            fault_col = STRIP_COLOURS[col_idx]
            for i in range(3):        pixels[i] = flash
            pixels[3] = fault_col;    pixels[4] = fault_col
            for i in range(5, NUM_LEDS): pixels[i] = flash
            pos = 1 - pos

        _state['strip_pixels']  = pixels
        _state['strip_pat_pos'] = pos
        _state['strip_pat_dir'] = direction


def set_fault_leds():
    with _lock:
        any_fault = _state.get('fault_l', False) or _state.get('fault_r', False)
        if any_fault:
            _state['strip_pattern']    = 6
            _state['strip_colour_idx'] = 1
            _state['strip_pat_pos']    = 0
            _state['strip_pat_dir']    = 1
        else:
            if _state['strip_pattern'] == 6 and _state['strip_colour_idx'] == 1:
                _state['strip_pattern'] = 0
                _state['strip_pixels']  = [(0, 0, 0)] * NUM_LEDS


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


# ---------------------------------------------------------------------------
# iBUS reader  (optional — pipe ibus_sim channels into rc_channels/rc_valid)
# ---------------------------------------------------------------------------

_IBUS_FAILSAFE = 0.5   # seconds without a valid packet → rc_valid = False


def _ibus_reader_thread(port):
    sys.path.insert(0, _ROOT)
    from drivers.ibus import IBusReader, IBusError
    last_valid = 0.0
    while True:
        with _lock:
            if not _state['running']:
                return
        try:
            with IBusReader(port, timeout=0.5) as ibus:
                while True:
                    with _lock:
                        if not _state['running']:
                            return
                    channels = ibus.read()
                    now = time.monotonic()
                    with _lock:
                        if channels:
                            _state['rc_channels'] = channels
                            _state['rc_valid']    = True
                            last_valid = now
                        elif now - last_valid > _IBUS_FAILSAFE:
                            _state['rc_valid'] = False
                    if channels:
                        _tick_manual_drive()
        except Exception:
            with _lock:
                _state['rc_valid'] = False
            time.sleep(1.0)


def start_ibus_reader(port):
    """Start a background thread reading iBUS packets from *port* into rc_channels."""
    with _lock:
        _state['ibus_port'] = port
    t = threading.Thread(target=_ibus_reader_thread, args=(port,), daemon=True)
    t.start()


def yukon_server(master_fd):
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
            if 0x21 <= b <= 0x2D:
                pkt_cmd = b; sm = 'V_HIGH'
            else:
                os.write(master_fd, bytes([NAK])); sm = 'SYNC'

        elif sm == 'V_HIGH':
            if 0x40 <= b <= 0x4F:
                pkt_vhigh = b; sm = 'V_LOW'
            else:
                os.write(master_fd, bytes([NAK])); sm = 'SYNC'

        elif sm == 'V_LOW':
            if 0x50 <= b <= 0x5F:
                pkt_vlow = b; sm = 'CHK'
            else:
                os.write(master_fd, bytes([NAK])); sm = 'SYNC'

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
                        if _state['rc_mode'] == RC_AUTO:
                            _state['left_byte'] = value
                    elif cmd_code == CMD_RIGHT:
                        if _state['rc_mode'] == RC_AUTO:
                            _state['right_byte'] = value
                    elif cmd_code == CMD_KILL:
                        _state['left_byte']      = 0
                        _state['right_byte']     = 0
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
                        pass
                    elif cmd_code == CMD_PATTERN:
                        colour_nibble = (value >> 4) & 0x0F
                        pat           = value & 0x0F
                        if colour_nibble > 0 and colour_nibble < len(STRIP_COLOURS):
                            _state['strip_colour_idx'] = colour_nibble
                        _state['strip_pattern'] = pat if pat <= 6 else 0
                        _state['strip_pat_pos'] = 0
                        _state['strip_pat_dir'] = 1
                        if pat == 0:
                            _state['strip_pixels'] = [(0, 0, 0)] * NUM_LEDS
                    elif cmd_code == CMD_MODE:
                        _state['pi_last_cmd_ms'] = time.monotonic()
                        _state['rc_mode'] = value
                        if value == RC_ESTOP:
                            _state['left_byte']      = 0
                            _state['right_byte']     = 0
                            _state['bearing_target'] = None
                    elif cmd_code == CMD_RC_QUERY:
                        _state['pi_last_rc_query_ms'] = time.monotonic()
                        channels = list(_state['rc_channels'])
                        valid    = int(_state['rc_valid'])
                        for i, us in enumerate(channels):
                            _send_sensor_packet(master_fd, RESP_RC_BASE + i,
                                                (us - 1000) // 5)
                        _send_sensor_packet(master_fd, RESP_RC_BASE + 14, valid)
                    elif cmd_code == CMD_BENCH:
                        _state['bench_enabled'] = bool(value)
                    elif cmd_code == CMD_SENSOR:
                        _send_sensor_packet(master_fd, RESP_VOLTAGE, SIM_VOLTAGE  * 10)
                        _send_sensor_packet(master_fd, RESP_CURRENT, SIM_CURRENT  * 100)
                        _send_sensor_packet(master_fd, RESP_TEMP,    SIM_TEMP     * 3)
                        _send_sensor_packet(master_fd, RESP_TEMP_L,  SIM_TEMP_MOD * 3)
                        _send_sensor_packet(master_fd, RESP_TEMP_R,  SIM_TEMP_MOD * 3)
                        _send_sensor_packet(master_fd, RESP_FAULT_L, int(_state.get('fault_l', False)))
                        _send_sensor_packet(master_fd, RESP_FAULT_R, int(_state.get('fault_r', False)))
                        _send_sensor_packet(master_fd, RESP_BENCH_TEMP,  SIM_TEMP_MOD * 3)
                        _send_sensor_packet(master_fd, RESP_BENCH_FAULT, int(_state.get('bench_fault', False)))
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
# Headless terminal mode
# ---------------------------------------------------------------------------

def _draw_headless(yukon_path):
    with _lock:
        lb           = _state['left_byte']
        rb           = _state['right_byte']
        led_a        = _state['led_a']
        led_b        = _state['led_b']
        cmds         = _state['cmds_rx']
        rc_mode      = _state['rc_mode']
        pi_no_motors = (_state['pi_last_rc_query_ms'] != 0.0 and
                        _state['pi_last_cmd_ms'] == 0.0)
        rc_valid     = _state['rc_valid']
        rc_ch        = list(_state['rc_channels'])
        ibus_port    = _state.get('ibus_port')
        imu_present  = _state['imu_present']
        imu_heading  = _state['imu_heading']
        imu_pitch    = _state['imu_pitch']
        imu_roll     = _state['imu_roll']
        bearing_tgt   = _state['bearing_target']
        bench_enabled = _state.get('bench_enabled', False)
        bench_fault   = _state.get('bench_fault', False)
        strip_pixels  = list(_state['strip_pixels'])
        strip_pat     = _state['strip_pattern']

    rx_l = _decode_speed(lb)
    rx_r = _decode_speed(rb)
    mode_str = RC_MODE_NAMES.get(rc_mode, str(rc_mode))
    no_motors_flag = '  [NO-MOTORS]' if pi_no_motors else ''
    ibus_str = (f'{ibus_port}   RC: {"OK" if rc_valid else "LOST"}'
                f'   thr={rc_ch[2]:4d} ste={rc_ch[0]:4d}') if ibus_port else 'none'

    def speed_line(label, rx):
        if rx is None:
            return f'  {label}  {_bar(None)}  (waiting...)'
        return f'  {label}  {_bar(rx)}  {rx:+.2f}'

    def compass_bar(heading, width=36):
        pos = int(round(heading / 360.0 * width)) % width
        bar = ['-'] * width
        bar[pos] = 'N' if (heading < 5 or heading > 355) else '+'
        return '[' + ''.join(bar) + f']  {heading:6.1f}°'

    imu_str = 'PRESENT' if imu_present else 'ABSENT (toggle: I)'
    tgt_str = f'{bearing_tgt:.1f}°' if bearing_tgt is not None else 'off'
    err_str = ''
    if imu_present and bearing_tgt is not None:
        err     = _angle_diff(bearing_tgt, imu_heading)
        err_str = f'  err: {err:+.1f}°'

    lines = [
        '\033[2J\033[H',
        '=== Yukon Simulator ' + '=' * 42,
        f'  Yukon port : {yukon_path}',
        f'  Connect any client with --yukon-port {yukon_path}',
        f'  Mode       : {mode_str}{no_motors_flag}',
        f'  iBUS       : {ibus_str}',
        '--- Motor commands ' + '-' * 43,
        speed_line('Left ', rx_l),
        speed_line('Right', rx_r),
        '  (MANUAL: from RC channels  AUTO: CMD_LEFT/RIGHT)',
        '--- IMU ' + '-' * 54,
        f'  IMU      : {imu_str}',
        f'  Heading  : {compass_bar(imu_heading) if imu_present else "---"}',
        (f'  Pitch    : {imu_pitch:+.1f}°  Roll: {imu_roll:+.1f}°'
         if imu_present else '  Pitch    : ---  Roll: ---'),
        f'  Brg hold : {tgt_str}{err_str}',
        f'  Keys     : < decrease heading   > increase heading   I toggle IMU',
        '--- Status ' + '-' * 51,
        f'  LED A: {"ON " if led_a else "OFF"}  LED B: {"ON " if led_b else "OFF"}'
        f'  Cmds rx: {cmds}',
        f'  Bench  : {"ON " if bench_enabled else "OFF"}'
        f'{"  FAULT" if bench_fault else ""}',
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


def run_headless(yukon_path):
    stdin_fd  = sys.stdin.fileno()
    old_attrs = termios.tcgetattr(stdin_fd)
    try:
        tty.setraw(stdin_fd)
        last_draw = 0.0

        while True:
            now = time.monotonic()
            _tick_imu()
            _tick_rc_motors()
            _tick_strip()

            if now - last_draw >= 0.1:
                try:
                    _draw_headless(yukon_path)
                except Exception:
                    pass
                last_draw = now

            r, _, _ = select.select([stdin_fd], [], [], 0.05)
            if not r:
                continue

            data = os.read(stdin_fd, 16)
            for b in data:
                ch = chr(b)
                if ch in ('q', 'Q', '\x03', '\x04'):
                    with _lock:
                        _state['running'] = False
                    return
                elif ch == '>':
                    with _lock:
                        if _state['imu_present']:
                            _state['imu_heading'] = (
                                _state['imu_heading'] + IMU_HEADING_STEP
                            ) % 360.0
                elif ch == '<':
                    with _lock:
                        if _state['imu_present']:
                            _state['imu_heading'] = (
                                _state['imu_heading'] - IMU_HEADING_STEP
                            ) % 360.0
                elif ch in ('i', 'I'):
                    with _lock:
                        _state['imu_present'] = not _state['imu_present']
                        if not _state['imu_present']:
                            _state['bearing_target'] = None
    finally:
        termios.tcsetattr(stdin_fd, termios.TCSADRAIN, old_attrs)
        with _lock:
            _state['running'] = False
        sys.stdout.write('\r\n\r\nSimulator stopped.\r\n')
        sys.stdout.flush()


# ---------------------------------------------------------------------------
# Pygame GUI mode
# ---------------------------------------------------------------------------

def run_gui(yukon_path):
    import pygame

    C_BG      = ( 18,  18,  30)
    C_PANEL   = ( 28,  28,  45)
    C_BORDER  = ( 60,  60,  90)
    C_WHITE   = (230, 230, 240)
    C_GRAY    = (130, 130, 150)
    C_GREEN   = ( 60, 220,  80)
    C_YELLOW  = (240, 200,  40)
    C_ORANGE  = (240, 140,  40)
    C_RED     = (220,  60,  60)
    C_CYAN    = ( 60, 200, 220)
    W, H      = 900, 650

    def _panel(surf, rect, title=None):
        pygame.draw.rect(surf, C_PANEL,  rect, border_radius=8)
        pygame.draw.rect(surf, C_BORDER, rect, width=1, border_radius=8)
        if title:
            t = FONT_SM.render(title, True, C_GRAY)
            surf.blit(t, (rect.x + 10, rect.y + 6))

    def _label(surf, text, x, y, color=None, font=None):
        f = font or FONT_SM
        surf.blit(f.render(text, True, color or C_GRAY), (x, y))

    def _button(surf, rect, text, color, active=False):
        bg = tuple(min(255, c + 30) for c in C_PANEL) if active else C_PANEL
        pygame.draw.rect(surf, bg,    rect, border_radius=6)
        pygame.draw.rect(surf, color, rect, width=2, border_radius=6)
        t = FONT_SM.render(text, True, color)
        surf.blit(t, (rect.centerx - t.get_width()//2,
                      rect.centery - t.get_height()//2))

    def _motor_bar(surf, rect, speed):
        pygame.draw.rect(surf, C_BG, rect, border_radius=4)
        cx = rect.x + rect.width // 2
        pygame.draw.line(surf, C_BORDER, (cx, rect.y+2), (cx, rect.bottom-2), 1)
        if speed is not None and speed != 0:
            fill_w = int(abs(speed) * rect.width // 2)
            color  = C_GREEN if speed > 0 else C_ORANGE
            fill   = (pygame.Rect(cx, rect.y+3, fill_w, rect.height-6) if speed > 0
                      else pygame.Rect(cx - fill_w, rect.y+3, fill_w, rect.height-6))
            pygame.draw.rect(surf, color, fill, border_radius=3)
        val = "---" if speed is None else f"{speed:+.2f}"
        col = C_GRAY if speed is None else (C_GREEN if speed >= 0 else C_ORANGE)
        t   = FONT_SM.render(val, True, col)
        surf.blit(t, (rect.right - t.get_width() - 6,
                      rect.centery - t.get_height()//2))

    def _compass(surf, cx, cy, radius, heading, bearing_target, imu_present):
        pygame.draw.circle(surf, C_BORDER, (cx, cy), radius, 2)
        pygame.draw.circle(surf, C_BG,     (cx, cy), radius - 3)
        for angle, label in ((0,"N"),(90,"E"),(180,"S"),(270,"W")):
            rad = math.radians(angle - 90)
            ox  = cx + int((radius - 2)  * math.cos(rad))
            oy  = cy + int((radius - 2)  * math.sin(rad))
            ix  = cx + int((radius - 14) * math.cos(rad))
            iy  = cy + int((radius - 14) * math.sin(rad))
            pygame.draw.line(surf, C_GRAY, (ix, iy), (ox, oy), 2)
            lx  = cx + int((radius - 26) * math.cos(rad))
            ly  = cy + int((radius - 26) * math.sin(rad))
            t   = FONT_TINY.render(label, True, C_GRAY)
            surf.blit(t, (lx - t.get_width()//2, ly - t.get_height()//2))
        for angle in range(0, 360, 45):
            if angle % 90 == 0:
                continue
            rad = math.radians(angle - 90)
            ox  = cx + int((radius - 2)  * math.cos(rad))
            oy  = cy + int((radius - 2)  * math.sin(rad))
            ix  = cx + int((radius - 10) * math.cos(rad))
            iy  = cy + int((radius - 10) * math.sin(rad))
            pygame.draw.line(surf, C_BORDER, (ix, iy), (ox, oy), 1)
        if not imu_present:
            t = FONT_MD.render("IMU ABSENT", True, C_RED)
            surf.blit(t, (cx - t.get_width()//2, cy - t.get_height()//2))
            return
        if bearing_target is not None:
            trad  = math.radians(bearing_target - 90)
            steps = 10
            for i in range(0, steps, 2):
                sx = cx + int((radius - 20) * i/steps * math.cos(trad))
                sy = cy + int((radius - 20) * i/steps * math.sin(trad))
                ex = cx + int((radius - 20) * (i+1)/steps * math.cos(trad))
                ey = cy + int((radius - 20) * (i+1)/steps * math.sin(trad))
                pygame.draw.line(surf, C_CYAN, (sx, sy), (ex, ey), 2)
            lx = cx + int((radius + 10) * math.cos(trad))
            ly = cy + int((radius + 10) * math.sin(trad))
            t  = FONT_TINY.render(f"{bearing_target:.0f}°", True, C_CYAN)
            surf.blit(t, (lx - t.get_width()//2, ly - t.get_height()//2))
        hrad   = math.radians(heading - 90)
        needle = int(radius - 18)
        tail   = int(radius * 0.3)
        nx     = cx + int(needle * math.cos(hrad))
        ny     = cy + int(needle * math.sin(hrad))
        tx2    = cx - int(tail   * math.cos(hrad))
        ty2    = cy - int(tail   * math.sin(hrad))
        pygame.draw.line(surf, C_RED,   (cx, cy), (tx2, ty2), 3)
        pygame.draw.line(surf, C_WHITE, (cx, cy), (nx, ny),   3)
        pygame.draw.circle(surf, C_WHITE, (cx, cy), 5)
        t = FONT_MD.render(f"{heading:.1f}°", True, C_WHITE)
        surf.blit(t, (cx - t.get_width()//2, cy + radius//2))

    class Slider:
        def __init__(self, rect, vmin, vmax, value, color, label, fmt="{:.1f}"):
            self.rect   = pygame.Rect(rect)
            self.vmin   = vmin
            self.vmax   = vmax
            self.value  = value
            self.color  = color
            self.label  = label
            self.fmt    = fmt
            self.active = False

        def draw(self, surf):
            _label(surf, self.label, self.rect.x, self.rect.y - 18)
            pygame.draw.rect(surf, C_BG,     self.rect, border_radius=4)
            pygame.draw.rect(surf, C_BORDER, self.rect, width=1, border_radius=4)
            t      = (self.value - self.vmin) / (self.vmax - self.vmin)
            fill_w = int(t * self.rect.width)
            if fill_w > 0:
                pygame.draw.rect(surf, self.color,
                                 pygame.Rect(self.rect.x, self.rect.y,
                                             fill_w, self.rect.height),
                                 border_radius=4)
            kx  = self.rect.x + fill_w
            pygame.draw.circle(surf, C_WHITE, (kx, self.rect.centery), 7)
            lbl = FONT_SM.render(self.fmt.format(self.value), True, C_WHITE)
            surf.blit(lbl, (self.rect.x + self.rect.width//2 - lbl.get_width()//2,
                            self.rect.centery - lbl.get_height()//2))

        def handle_event(self, event):
            if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                if self.rect.collidepoint(event.pos):
                    self.active = True; self._set(event.pos[0])
            elif event.type == pygame.MOUSEBUTTONUP:
                self.active = False
            elif event.type == pygame.MOUSEMOTION and self.active:
                self._set(event.pos[0])

        def _set(self, mx):
            t = (mx - self.rect.x) / self.rect.width
            self.value = max(self.vmin, min(self.vmax,
                             self.vmin + t * (self.vmax - self.vmin)))

    pygame.init()
    screen = pygame.display.set_mode((W, H))
    pygame.display.set_caption("Yukon Simulator")
    clock  = pygame.time.Clock()
    FONT_BIG  = pygame.font.SysFont("monospace", 26, bold=True)
    FONT_MD   = pygame.font.SysFont("monospace", 18)
    FONT_SM   = pygame.font.SysFont("monospace", 14)
    FONT_TINY = pygame.font.SysFont("monospace", 11)

    volt_slider = Slider((480, 100, 200, 22), 8.0, 15.0, 12.0,
                         C_YELLOW, "Voltage (V)", "{:.1f} V")
    curr_slider = Slider((480, 160, 200, 22), 0.0,  5.0,  0.5,
                         C_ORANGE, "Current (A)", "{:.2f} A")
    fault_left  = False
    fault_right = False
    comp_cx, comp_cy, comp_r = 180, 302, 140

    running = True
    while running:
        clock.tick(30)
        _tick_imu()
        _tick_rc_motors()
        _tick_strip()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            volt_slider.handle_event(event)
            curr_slider.handle_event(event)
            if event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_q, pygame.K_ESCAPE):
                    running = False
                elif event.key == pygame.K_i:
                    with _lock:
                        _state['imu_present'] = not _state['imu_present']
                        if not _state['imu_present']:
                            _state['bearing_target'] = None
                elif event.key in (pygame.K_GREATER, pygame.K_PERIOD):
                    with _lock:
                        if _state['imu_present']:
                            _state['imu_heading'] = (
                                _state['imu_heading'] + IMU_HEADING_STEP) % 360.0
                elif event.key in (pygame.K_LESS, pygame.K_COMMA):
                    with _lock:
                        if _state['imu_present']:
                            _state['imu_heading'] = (
                                _state['imu_heading'] - IMU_HEADING_STEP) % 360.0
            if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                mx, my = event.pos
                dx, dy = mx - comp_cx, my - comp_cy
                if dx*dx + dy*dy <= comp_r*comp_r:
                    angle = (math.degrees(math.atan2(dy, dx)) + 90) % 360
                    with _lock:
                        if _state['imu_present']:
                            _state['imu_heading'] = angle
                imu_btn_rect = pygame.Rect(480, 220, 200, 36)
                if imu_btn_rect.collidepoint(mx, my):
                    with _lock:
                        _state['imu_present'] = not _state['imu_present']
                        if not _state['imu_present']:
                            _state['bearing_target'] = None
                fault_l_rect = pygame.Rect(480, 280,  95, 36)
                fault_r_rect = pygame.Rect(585, 280,  95, 36)
                reset_f_rect = pygame.Rect(480, 330, 200, 36)
                if fault_l_rect.collidepoint(mx, my):
                    fault_left  = not fault_left
                if fault_r_rect.collidepoint(mx, my):
                    fault_right = not fault_right
                if reset_f_rect.collidepoint(mx, my):
                    fault_left = fault_right = False

        global SIM_VOLTAGE, SIM_CURRENT
        SIM_VOLTAGE = volt_slider.value
        SIM_CURRENT = curr_slider.value
        with _lock:
            prev_fl = _state.get('fault_l', False)
            prev_fr = _state.get('fault_r', False)
            _state['fault_l'] = fault_left
            _state['fault_r'] = fault_right
        if fault_left != prev_fl or fault_right != prev_fr:
            set_fault_leds()

        with _lock:
            lb           = _state['left_byte']
            rb           = _state['right_byte']
            led_a        = _state['led_a']
            led_b        = _state['led_b']
            cmds         = _state['cmds_rx']
            rc_mode      = _state['rc_mode']
            pi_no_motors = (_state['pi_last_rc_query_ms'] != 0.0 and
                            _state['pi_last_cmd_ms'] == 0.0)
            imu_present  = _state['imu_present']
            imu_heading  = _state['imu_heading']
            imu_pitch    = _state['imu_pitch']
            imu_roll     = _state['imu_roll']
            bearing_tgt   = _state['bearing_target']
            bench_enabled = _state.get('bench_enabled', False)
            bench_fault   = _state.get('bench_fault', False)
            strip_pixels  = list(_state['strip_pixels'])
            strip_pat    = _state['strip_pattern']

        rx_l = _decode_speed(lb)
        rx_r = _decode_speed(rb)
        screen.fill(C_BG)

        # Title bar
        title_rect = pygame.Rect(10, 8, W - 20, 65)
        pygame.draw.rect(screen, C_PANEL, title_rect, border_radius=6)
        pygame.draw.rect(screen, C_CYAN,  title_rect, width=1, border_radius=6)
        screen.blit(FONT_BIG.render("Yukon Simulator", True, C_WHITE), (22, 16))
        port_t = FONT_SM.render(f"PTY: {yukon_path}", True, C_GRAY)
        screen.blit(port_t, (W - port_t.get_width() - 16, 22))
        if pi_no_motors:
            nm_t = FONT_SM.render("[NO-MOTORS]", True, C_ORANGE)
            screen.blit(nm_t, (W - nm_t.get_width() - 16, 40))
        hint = FONT_SM.render(
            "< > = heading   I = IMU toggle   Q = quit   click compass = set heading",
            True, C_GRAY)
        screen.blit(hint, (W//2 - hint.get_width()//2, 50))

        # Compass
        comp_panel = pygame.Rect(10, 82, 350, 400)
        _panel(screen, comp_panel, "IMU Heading")
        _compass(screen, comp_cx, comp_cy, comp_r,
                 imu_heading, bearing_tgt, imu_present)
        if imu_present and bearing_tgt is not None:
            err  = _angle_diff(bearing_tgt, imu_heading)
            info = f"Hold: {bearing_tgt:.1f}°   Err: {err:+.1f}°"
            col  = C_GREEN if abs(err) < 5 else C_YELLOW
        elif imu_present:
            info, col = "Bearing hold: OFF", C_GRAY
        else:
            info, col = "IMU ABSENT", C_RED
        t = FONT_SM.render(info, True, col)
        screen.blit(t, (comp_panel.centerx - t.get_width()//2, comp_panel.bottom - 30))

        # Drive panel
        mid_panel = pygame.Rect(370, 82, 240, 260)
        _panel(screen, mid_panel, "Drive")
        mode_name = RC_MODE_NAMES.get(rc_mode, str(rc_mode))
        mode_col  = {RC_MANUAL: C_CYAN, RC_AUTO: C_GREEN, RC_ESTOP: C_RED}.get(rc_mode, C_GRAY)
        nm_suffix = ' [NO-MOTORS]' if pi_no_motors else ''
        _label(screen, f"Mode: {mode_name}{nm_suffix}", mid_panel.x + 12, mid_panel.y + 8,
               mode_col if not pi_no_motors else C_ORANGE)
        for i, (label, speed) in enumerate((("Left", rx_l), ("Right", rx_r))):
            by = mid_panel.y + 30 + i * 90
            _label(screen, label, mid_panel.x + 12, by + 4)
            bar = pygame.Rect(mid_panel.x + 12, by + 22, mid_panel.width - 24, 40)
            _motor_bar(screen, bar, speed)
        led_y = mid_panel.y + 195
        for i, (label, state) in enumerate((("A", led_a), ("B", led_b))):
            sq_x = mid_panel.x + 12 + i * 80
            sq   = pygame.Rect(sq_x, led_y, 20, 20)
            pygame.draw.rect(screen, C_WHITE if state else C_BORDER, sq, border_radius=3)
            _label(screen, f"LED {label}", sq_x + 24, led_y + 3)
        _label(screen, f"Cmds rx: {cmds}", mid_panel.x + 12, mid_panel.bottom - 22)

        # Sim values panel
        stat_panel = pygame.Rect(370, 352, 240, 155)
        _panel(screen, stat_panel, "Sim Values")
        _label(screen, f"Voltage : {volt_slider.value:.1f} V", stat_panel.x+12, stat_panel.y+26, C_YELLOW)
        _label(screen, f"Current : {curr_slider.value:.2f} A", stat_panel.x+12, stat_panel.y+50, C_ORANGE)
        fl_col = C_RED if fault_left  else C_GRAY
        fr_col = C_RED if fault_right else C_GRAY
        _label(screen, f"Fault L : {'YES' if fault_left  else 'no'}", stat_panel.x+12, stat_panel.y+74, fl_col)
        _label(screen, f"Fault R : {'YES' if fault_right else 'no'}", stat_panel.x+12, stat_panel.y+98, fr_col)
        bench_col = (C_RED if bench_fault else C_GREEN) if bench_enabled else C_GRAY
        bench_str = ('ON' + (' FAULT' if bench_fault else '')) if bench_enabled else 'OFF'
        _label(screen, f"Bench   : {bench_str}", stat_panel.x+12, stat_panel.y+122, bench_col)

        # IMU pitch/roll
        imu_panel = pygame.Rect(370, 492, 240, 65)
        _panel(screen, imu_panel, "IMU")
        imu_col = C_GRAY if not imu_present else C_WHITE
        _label(screen, (f"Pitch: {imu_pitch:+.1f}°" if imu_present else "Pitch: ---"),
               imu_panel.x+12, imu_panel.y+26, imu_col)
        _label(screen, (f"Roll:  {imu_roll:+.1f}°"  if imu_present else "Roll:  ---"),
               imu_panel.x+12+110, imu_panel.y+26, imu_col)

        # Controls
        ctrl_panel = pygame.Rect(620, 82, 270, 400)
        _panel(screen, ctrl_panel, "Controls")
        volt_slider.rect = pygame.Rect(ctrl_panel.x+12, ctrl_panel.y+40,  ctrl_panel.width-24, 22)
        curr_slider.rect = pygame.Rect(ctrl_panel.x+12, ctrl_panel.y+100, ctrl_panel.width-24, 22)
        volt_slider.draw(screen)
        curr_slider.draw(screen)
        imu_btn_r = pygame.Rect(ctrl_panel.x+12, ctrl_panel.y+155, ctrl_panel.width-24, 36)
        imu_col2  = C_GREEN if imu_present else C_RED
        _button(screen, imu_btn_r, f"IMU: {'PRESENT' if imu_present else 'ABSENT'}",
                imu_col2, active=imu_present)
        fault_l_r = pygame.Rect(ctrl_panel.x+12,          ctrl_panel.y+210, (ctrl_panel.width-28)//2, 36)
        fault_r_r = pygame.Rect(fault_l_r.right+4,        ctrl_panel.y+210, (ctrl_panel.width-28)//2, 36)
        reset_f_r = pygame.Rect(ctrl_panel.x+12,          ctrl_panel.y+258, ctrl_panel.width-24, 36)
        _button(screen, fault_l_r, "Fault L", C_RED,  active=fault_left)
        _button(screen, fault_r_r, "Fault R", C_RED,  active=fault_right)
        _button(screen, reset_f_r, "Clear Faults", C_GRAY)
        hl = pygame.Rect(ctrl_panel.x+12,              ctrl_panel.y+310, 80, 36)
        hr = pygame.Rect(ctrl_panel.right-12-80,       ctrl_panel.y+310, 80, 36)
        _button(screen, hl, "< -5°", C_CYAN)
        _button(screen, hr, "+5° >", C_CYAN)

        # LED strip
        strip_panel = pygame.Rect(10, 492, W - 20, 95)
        _panel(screen, strip_panel, "LED Strip")
        sq_sz = 80; sq_h = 46
        gap   = NUM_LEDS * sq_sz + (NUM_LEDS - 1) * 10
        off_x = strip_panel.x + (strip_panel.width - gap) // 2
        sq_y  = strip_panel.y + 34
        for i, (pr, pg, pb) in enumerate(strip_pixels):
            sx = off_x + i * (sq_sz + 10)
            sq = pygame.Rect(sx, sq_y, sq_sz, sq_h)
            pygame.draw.rect(screen, (pr//3, pg//3, pb//3), sq, border_radius=5)
            if pr > 0 or pg > 0 or pb > 0:
                pygame.draw.rect(screen, (pr, pg, pb), sq.inflate(-6,-6), border_radius=4)
            pygame.draw.rect(screen, C_BORDER, sq, width=1, border_radius=5)
        pat_label = PAT_NAMES.get(strip_pat, '?')
        _label(screen, f"pattern: {pat_label}", strip_panel.x+120, strip_panel.y+6)

        # Footer
        pygame.draw.line(screen, C_BORDER, (10, H-42), (W-10, H-42), 1)
        footer = FONT_SM.render(
            "Click compass to set heading  ·  < >  nudge  ·  I  toggle IMU  ·  Q  quit",
            True, C_GRAY)
        screen.blit(footer, (W//2 - footer.get_width()//2, H - 30))

        pygame.display.flip()

    pygame.quit()


# ---------------------------------------------------------------------------
# Web mode
# ---------------------------------------------------------------------------

def _build_web_app():
    from flask import Flask, Response, request, jsonify
    app = Flask(__name__)

    def _get_state():
        with _lock:
            s = dict(_state)
            strip_pixels = [list(p) for p in s.get('strip_pixels', [])]
        rc_mode      = s.get('rc_mode', RC_MANUAL)
        pi_no_motors = (s.get('pi_last_rc_query_ms', 0.0) != 0.0 and
                        s.get('pi_last_cmd_ms', 0.0) == 0.0)
        return {
            'left_speed'    : _decode_speed(s.get('left_byte')),
            'right_speed'   : _decode_speed(s.get('right_byte')),
            'led_a'         : s.get('led_a', False),
            'led_b'         : s.get('led_b', False),
            'cmds_rx'       : s.get('cmds_rx', 0),
            'rc_mode'       : rc_mode,
            'rc_mode_name'  : RC_MODE_NAMES.get(rc_mode, str(rc_mode)),
            'no_motors'     : pi_no_motors,
            'imu_present'   : s.get('imu_present', True),
            'imu_heading'   : round(s.get('imu_heading', 0.0), 1),
            'imu_pitch'     : round(s.get('imu_pitch',   0.0), 1),
            'imu_roll'      : round(s.get('imu_roll',    0.0), 1),
            'bearing_target': s.get('bearing_target'),
            'fault_l'       : s.get('fault_l', False),
            'fault_r'       : s.get('fault_r', False),
            'bench_enabled' : s.get('bench_enabled', False),
            'bench_fault'   : s.get('bench_fault', False),
            'voltage'       : round(SIM_VOLTAGE, 2),
            'current'       : round(SIM_CURRENT, 3),
            'yukon_port'    : _yukon_path[0],
            'strip_pixels'  : strip_pixels,
            'strip_pattern' : s.get('strip_pattern', 0),
        }

    @app.route('/')
    def index():
        return Response(_HTML, mimetype='text/html')

    @app.route('/api/state')
    def api_state():
        def _gen():
            while True:
                _tick_imu()
                _tick_rc_motors()
                _tick_strip()
                yield f"data: {json.dumps(_get_state())}\n\n"
                time.sleep(0.1)
        return Response(_gen(), mimetype='text/event-stream',
                        headers={'Cache-Control': 'no-cache',
                                 'X-Accel-Buffering': 'no'})

    @app.route('/api/cmd', methods=['POST'])
    def api_cmd():
        global SIM_VOLTAGE, SIM_CURRENT
        body = request.json or {}
        cmd  = body.get('cmd', '')
        val  = body.get('value')
        if cmd == 'set_heading':
            with _lock:
                if _state['imu_present']:
                    _state['imu_heading'] = float(val) % 360.0
        elif cmd == 'nudge_heading':
            with _lock:
                if _state['imu_present']:
                    _state['imu_heading'] = (_state['imu_heading'] + float(val)) % 360.0
        elif cmd == 'toggle_imu':
            with _lock:
                _state['imu_present'] = not _state['imu_present']
                if not _state['imu_present']:
                    _state['bearing_target'] = None
        elif cmd == 'set_voltage':
            SIM_VOLTAGE = max(8.0, min(15.0, float(val)))
        elif cmd == 'set_current':
            SIM_CURRENT = max(0.0, min(5.0, float(val)))
        elif cmd == 'fault_l':
            with _lock:
                _state['fault_l'] = not _state.get('fault_l', False)
            set_fault_leds()
        elif cmd == 'fault_r':
            with _lock:
                _state['fault_r'] = not _state.get('fault_r', False)
            set_fault_leds()
        elif cmd == 'clear_faults':
            with _lock:
                _state['fault_l'] = False
                _state['fault_r'] = False
            set_fault_leds()
        else:
            return jsonify({'ok': False, 'error': f'Unknown cmd: {cmd}'}), 400
        return jsonify({'ok': True})

    return app


_yukon_path = ['']   # mutable container so web routes can read it

_HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Yukon Simulator</title>
<style>
*,*::before,*::after{box-sizing:border-box;margin:0;padding:0}
:root{
  --bg:#12121e;--panel:#1c1c2d;--border:#3c3c5a;
  --white:#e6e6f0;--gray:#82829a;
  --green:#3cdc50;--yellow:#f0c828;--orange:#f08c28;
  --red:#dc3c3c;--cyan:#3cc8dc;
}
body{background:var(--bg);color:var(--white);font-family:'Courier New',monospace;
  min-height:100vh;padding:10px}
h1{font-size:1.3rem;color:var(--cyan);margin-bottom:4px}
.subtitle{font-size:.75rem;color:var(--gray);margin-bottom:12px}
.grid{display:grid;grid-template-columns:1fr 1fr 1fr;gap:12px;max-width:960px;margin:0 auto}
.panel{background:var(--panel);border:1px solid var(--border);
  border-radius:10px;padding:14px}
.panel h2{font-size:.7rem;color:var(--gray);text-transform:uppercase;
  letter-spacing:1px;margin-bottom:10px}
#compass-wrap{display:flex;justify-content:center;margin-bottom:8px;cursor:crosshair}
svg text{font-family:'Courier New',monospace}
#hdg-label{text-align:center;font-size:1.1rem;font-weight:bold;color:var(--white)}
#hold-label{text-align:center;font-size:.75rem;margin-top:4px}
.motor-row{display:flex;align-items:center;gap:8px;margin-bottom:10px}
.motor-lbl{width:40px;font-size:.8rem;color:var(--gray)}
.motor-track{flex:1;height:32px;background:var(--bg);border-radius:5px;
  position:relative;overflow:hidden}
.motor-mid{position:absolute;left:50%;top:3px;bottom:3px;
  width:1px;background:var(--border)}
.motor-fill{position:absolute;top:3px;bottom:3px;
  border-radius:4px;transition:all .06s linear}
.motor-val{position:absolute;right:6px;top:50%;transform:translateY(-50%);
  font-size:.75rem}
.slider-row{margin-bottom:16px}
.slider-row label{font-size:.75rem;color:var(--gray);display:block;margin-bottom:4px}
input[type=range]{width:100%;accent-color:var(--cyan)}
.slider-val{font-size:.9rem;font-weight:bold;margin-top:2px}
.btn-row{display:flex;gap:8px;margin-bottom:10px;flex-wrap:wrap}
button{font-family:inherit;font-size:.8rem;padding:8px 14px;
  border-radius:6px;border:1px solid var(--border);
  background:var(--panel);color:var(--white);cursor:pointer}
button:active{opacity:.8}
button.active{background:#1a2a1a}
.btn-green{border-color:var(--green);color:var(--green)}
.btn-green.active{background:#0a2a0a}
.btn-red{border-color:var(--red);color:var(--red)}
.btn-red.active{background:#2a0a0a}
.btn-cyan{border-color:var(--cyan);color:var(--cyan)}
.badges{display:flex;flex-wrap:wrap;gap:6px;margin-top:8px}
.badge{font-size:.65rem;padding:3px 8px;border-radius:20px;
  border:1px solid var(--border);color:var(--gray)}
.badge.ok{border-color:var(--green);color:var(--green)}
.badge.warn{border-color:var(--yellow);color:var(--yellow)}
.badge.err{border-color:var(--red);color:var(--red)}
.badge.info{border-color:var(--cyan);color:var(--cyan)}
.badge.warn2{border-color:var(--orange);color:var(--orange)}
.led-row{display:flex;gap:16px;margin-top:10px;align-items:center}
.led-indicator{display:flex;align-items:center;gap:6px;font-size:.75rem;color:var(--gray)}
.led-sq{width:18px;height:18px;border-radius:3px;border:1px solid var(--border);
  background:var(--border);flex-shrink:0}
.led-sq.on{background:#e6e6f0;border-color:#e6e6f0}
.strip-wrap{margin-top:12px;max-width:960px;margin-left:auto;margin-right:auto}
.strip-panel{background:var(--panel);border:1px solid var(--border);
  border-radius:10px;padding:14px}
.strip-panel h2{font-size:.7rem;color:var(--gray);text-transform:uppercase;
  letter-spacing:1px;margin-bottom:10px;display:inline-block}
.strip-pat-label{font-size:.7rem;color:var(--gray);float:right;margin-top:2px}
.strip-pixels{display:flex;gap:8px;justify-content:center;margin-top:8px}
.pixel{flex:1;max-width:90px;height:44px;border-radius:6px;
  border:1px solid var(--border);position:relative;transition:background .08s}
#conn{position:fixed;top:10px;right:14px;font-size:.7rem;color:var(--gray)}
@media(max-width:680px){.grid{grid-template-columns:1fr}}
</style>
</head>
<body>
<div style="max-width:960px;margin:0 auto">
  <h1>Yukon Simulator</h1>
  <div class="subtitle" id="port-label">PTY: --</div>
</div>
<span id="conn">&#9899; Connecting&#8230;</span>
<div class="grid">
  <div class="panel">
    <h2>IMU Heading</h2>
    <div id="compass-wrap">
      <svg id="compass-svg" width="200" height="200" viewBox="-100 -100 200 200">
        <circle cx="0" cy="0" r="90" fill="none" stroke="#3c3c5a" stroke-width="1.5"/>
        <text x="0"   y="-72" text-anchor="middle" fill="#82829a" font-size="11">N</text>
        <text x="0"   y="80"  text-anchor="middle" fill="#82829a" font-size="11">S</text>
        <text x="76"  y="4"   text-anchor="middle" fill="#82829a" font-size="11">E</text>
        <text x="-76" y="4"   text-anchor="middle" fill="#82829a" font-size="11">W</text>
        <line x1="0"  y1="-90" x2="0"  y2="-78" stroke="#82829a" stroke-width="2"/>
        <line x1="0"  y1="90"  x2="0"  y2="78"  stroke="#82829a" stroke-width="2"/>
        <line x1="90" y1="0"   x2="78" y2="0"   stroke="#82829a" stroke-width="2"/>
        <line x1="-90" y1="0"  x2="-78" y2="0"  stroke="#82829a" stroke-width="2"/>
        <line id="target-needle" x1="0" y1="0" x2="0" y2="-72"
              stroke="#3cc8dc" stroke-width="2" stroke-dasharray="6,4" opacity="0"/>
        <line id="hdg-needle" x1="0" y1="12" x2="0" y2="-74"
              stroke="white" stroke-width="3" stroke-linecap="round"/>
        <line id="hdg-tail"   x1="0" y1="12" x2="0" y2="28"
              stroke="#dc3c3c" stroke-width="3" stroke-linecap="round"/>
        <circle cx="0" cy="0" r="5" fill="white"/>
        <text id="imu-absent-text" x="0" y="5" text-anchor="middle"
              fill="#dc3c3c" font-size="14" font-weight="bold" opacity="0">IMU ABSENT</text>
      </svg>
    </div>
    <div id="hdg-label">---</div>
    <div id="hold-label" style="color:var(--gray)">Hold: off</div>
    <div style="display:flex;gap:8px;margin-top:10px">
      <button class="btn-cyan" onclick="nudge(-5)">&#8592; -5&#176;</button>
      <button class="btn-cyan" style="flex:1" id="imu-btn" onclick="toggleImu()">IMU: ON</button>
      <button class="btn-cyan" onclick="nudge(+5)">+5&#176; &#8594;</button>
    </div>
  </div>
  <div class="panel">
    <h2>Drive</h2>
    <div class="motor-row">
      <span class="motor-lbl">Left</span>
      <div class="motor-track">
        <div class="motor-mid"></div>
        <div class="motor-fill" id="fill-l"></div>
        <span class="motor-val" id="val-l">---</span>
      </div>
    </div>
    <div class="motor-row">
      <span class="motor-lbl">Right</span>
      <div class="motor-track">
        <div class="motor-mid"></div>
        <div class="motor-fill" id="fill-r"></div>
        <span class="motor-val" id="val-r">---</span>
      </div>
    </div>
    <h2 style="margin-top:14px">Status</h2>
    <div class="led-row">
      <div class="led-indicator"><div class="led-sq" id="sq-leda"></div>LED A</div>
      <div class="led-indicator"><div class="led-sq" id="sq-ledb"></div>LED B</div>
    </div>
    <div class="badges" style="margin-top:8px">
      <span class="badge" id="bdg-mode">Mode: --</span>
      <span class="badge" id="bdg-nm" style="display:none">NO-MOTORS</span>
      <span class="badge" id="bdg-cmds">Cmds: 0</span>
      <span class="badge" id="bdg-fl">Fault L: --</span>
      <span class="badge" id="bdg-fr">Fault R: --</span>
      <span class="badge" id="bdg-bench">Bench: OFF</span>
      <span class="badge" id="bdg-hold">Hold: off</span>
    </div>
    <h2 style="margin-top:14px">Sim Values</h2>
    <div class="badges">
      <span class="badge info" id="bdg-volt">V: --</span>
      <span class="badge info" id="bdg-curr">A: --</span>
      <span class="badge" id="bdg-pitch">Pitch: --</span>
      <span class="badge" id="bdg-roll">Roll: --</span>
    </div>
  </div>
  <div class="panel">
    <h2>Controls</h2>
    <div class="slider-row">
      <label>Voltage</label>
      <input type="range" id="volt-slider" min="8" max="15" step="0.1" value="12"
             oninput="sendVal('set_voltage', this.value)">
      <div class="slider-val" id="volt-val">12.0 V</div>
    </div>
    <div class="slider-row">
      <label>Current</label>
      <input type="range" id="curr-slider" min="0" max="5" step="0.05" value="0.5"
             oninput="sendVal('set_current', this.value)">
      <div class="slider-val" id="curr-val">0.50 A</div>
    </div>
    <h2 style="margin-top:4px">Fault Injection</h2>
    <div class="btn-row">
      <button class="btn-red" id="btn-fl" onclick="sendCmd('fault_l')">Fault L</button>
      <button class="btn-red" id="btn-fr" onclick="sendCmd('fault_r')">Fault R</button>
      <button onclick="sendCmd('clear_faults')">Clear</button>
    </div>
    <h2 style="margin-top:8px">Heading</h2>
    <p style="font-size:.7rem;color:var(--gray);margin-bottom:8px">
      Click compass rose to set heading directly
    </p>
    <div class="btn-row">
      <button class="btn-cyan" onclick="nudge(-45)">-45&#176;</button>
      <button class="btn-cyan" onclick="nudge(-10)">-10&#176;</button>
      <button class="btn-cyan" onclick="nudge(+10)">+10&#176;</button>
      <button class="btn-cyan" onclick="nudge(+45)">+45&#176;</button>
    </div>
  </div>
</div>
<div class="strip-wrap">
  <div class="strip-panel">
    <h2>LED Strip</h2>
    <span class="strip-pat-label" id="strip-pat-label">pattern: off</span>
    <div class="strip-pixels" id="strip-pixels"></div>
  </div>
</div>
<script>
"use strict";
const el = id => document.getElementById(id);
const C = {green:'#3cdc50',yellow:'#f0c828',orange:'#f08c28',
           red:'#dc3c3c',cyan:'#3cc8dc',gray:'#82829a'};
el('compass-svg').addEventListener('click', e => {
  const svg = el('compass-svg'), rect = svg.getBoundingClientRect();
  const dx = e.clientX - (rect.left + rect.width/2);
  const dy = e.clientY - (rect.top  + rect.height/2);
  sendVal('set_heading', (((Math.atan2(dy,dx)*180/Math.PI)+90+360)%360).toFixed(1));
});
function setNeedle(id, deg) { el(id).setAttribute('transform', `rotate(${deg})`); }
function motorBar(fillId, valId, v) {
  const fill = el(fillId);
  if (v == null) { fill.style.width='0'; el(valId).textContent='---';
    el(valId).style.color=C.gray; return; }
  const col = v>0?C.green:v<0?C.orange:null;
  if (v>=0){fill.style.left='50%';fill.style.width=(v*50)+'%';}
  else{fill.style.left=(50+v*50)+'%';fill.style.width=(-v*50)+'%';}
  fill.style.backgroundColor = col||'transparent';
  el(valId).textContent = (v>=0?'+':'')+v.toFixed(2);
  el(valId).style.color = col||C.gray;
}
function setBadge(id, text, cls) {
  const b=el(id); b.textContent=text;
  b.className='badge'+(cls?' '+cls:'');
}
function applyState(s) {
  el('port-label').textContent = 'PTY: ' + s.yukon_port;
  const absent = !s.imu_present;
  el('imu-absent-text').setAttribute('opacity', absent?'1':'0');
  el('hdg-needle').setAttribute('opacity', absent?'0':'1');
  el('hdg-tail').setAttribute('opacity',   absent?'0':'1');
  if (!absent) {
    setNeedle('hdg-needle', s.imu_heading); setNeedle('hdg-tail', s.imu_heading);
    el('hdg-label').textContent = s.imu_heading.toFixed(1)+'°';
    el('hdg-label').style.color = C.white;
  } else { el('hdg-label').textContent='---'; el('hdg-label').style.color=C.red; }
  const tgt = s.bearing_target;
  if (tgt!=null && !absent) {
    el('target-needle').setAttribute('opacity','1'); setNeedle('target-needle', tgt);
    const err = ((tgt-s.imu_heading+180)%360)-180;
    el('hold-label').textContent=`Hold: ${tgt.toFixed(1)}°  err: ${err>=0?'+':''}${err.toFixed(1)}°`;
    el('hold-label').style.color = Math.abs(err)<5?C.green:C.yellow;
    setBadge('bdg-hold',`Hold: ${tgt.toFixed(0)}°`,'info');
  } else {
    el('target-needle').setAttribute('opacity','0');
    el('hold-label').textContent='Hold: off'; el('hold-label').style.color=C.gray;
    setBadge('bdg-hold','Hold: off','');
  }
  const imuBtn=el('imu-btn');
  imuBtn.textContent=`IMU: ${s.imu_present?'PRESENT':'ABSENT'}`;
  imuBtn.style.borderColor=s.imu_present?C.green:C.red;
  imuBtn.style.color=s.imu_present?C.green:C.red;
  motorBar('fill-l','val-l',s.left_speed);
  motorBar('fill-r','val-r',s.right_speed);
  el('sq-leda').className='led-sq'+(s.led_a?' on':'');
  el('sq-ledb').className='led-sq'+(s.led_b?' on':'');
  const modeCls = s.rc_mode===1?'ok':s.rc_mode===2?'err':'info';
  setBadge('bdg-mode',`Mode: ${s.rc_mode_name}`,modeCls);
  el('bdg-nm').style.display = s.no_motors?'inline':'none';
  setBadge('bdg-cmds',`Cmds: ${s.cmds_rx}`,'info');
  const PAT=['off','larson','random','rainbow','retro_computer','converge','estop_flash'];
  el('strip-pat-label').textContent='pattern: '+(PAT[s.strip_pattern]||'?');
  const container=el('strip-pixels'), pixels=s.strip_pixels||[];
  if(container.children.length!==pixels.length){
    container.innerHTML='';
    pixels.forEach(()=>{const d=document.createElement('div');d.className='pixel';container.appendChild(d);});
  }
  pixels.forEach(([r,g,b],i)=>{
    const lit=r>0||g>0||b>0;
    const dim=`rgb(${Math.round(r/3)},${Math.round(g/3)},${Math.round(b/3)})`;
    const bright=`rgb(${r},${g},${b})`;
    container.children[i].style.background=lit?`radial-gradient(ellipse at center,${bright} 40%,${dim} 100%)`:dim;
    container.children[i].style.borderColor=lit?bright:'var(--border)';
  });
  setBadge('bdg-fl',`Fault L: ${s.fault_l?'YES':'no'}`,s.fault_l?'err':'');
  setBadge('bdg-fr',`Fault R: ${s.fault_r?'YES':'no'}`,s.fault_r?'err':'');
  setBadge('bdg-bench',`Bench: ${s.bench_enabled?(s.bench_fault?'ON FAULT':'ON'):'OFF'}`,
           s.bench_enabled?(s.bench_fault?'err':'ok'):'');
  el('btn-fl').className='btn-red'+(s.fault_l?' active':'');
  el('btn-fr').className='btn-red'+(s.fault_r?' active':'');
  setBadge('bdg-volt',`${s.voltage.toFixed(1)} V`,'info');
  setBadge('bdg-curr',`${s.current.toFixed(2)} A`,'info');
  if(s.imu_present){
    setBadge('bdg-pitch',`Pitch: ${s.imu_pitch>=0?'+':''}${s.imu_pitch.toFixed(1)}°`,'info');
    setBadge('bdg-roll', `Roll: ${s.imu_roll>=0?'+':''}${s.imu_roll.toFixed(1)}°`,'info');
  } else { setBadge('bdg-pitch','Pitch: ---',''); setBadge('bdg-roll','Roll: ---',''); }
  if(document.activeElement!==el('volt-slider')){
    el('volt-slider').value=s.voltage; el('volt-val').textContent=s.voltage.toFixed(1)+' V';}
  if(document.activeElement!==el('curr-slider')){
    el('curr-slider').value=s.current; el('curr-val').textContent=s.current.toFixed(2)+' A';}
}
async function sendCmd(cmd,value){
  const body={cmd};
  if(value!==undefined)body.value=value;
  try{await fetch('/api/cmd',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});}
  catch(e){console.error('cmd failed:',e);}
}
function sendVal(cmd,value){sendCmd(cmd,parseFloat(value));}
function nudge(deg){sendCmd('nudge_heading',deg);}
function toggleImu(){sendCmd('toggle_imu');}
el('volt-slider').oninput=function(){el('volt-val').textContent=parseFloat(this.value).toFixed(1)+' V';sendVal('set_voltage',this.value);};
el('curr-slider').oninput=function(){el('curr-val').textContent=parseFloat(this.value).toFixed(2)+' A';sendVal('set_current',this.value);};
let evtSrc=null;
function connect(){
  if(evtSrc)evtSrc.close();
  const conn=el('conn'); conn.textContent='&#128993; Connecting&#8230;';
  evtSrc=new EventSource('/api/state');
  evtSrc.onopen=()=>{conn.textContent='&#128994; Live';};
  evtSrc.onmessage=e=>{try{applyState(JSON.parse(e.data));}catch(err){}};
  evtSrc.onerror=()=>{conn.textContent='&#128308; Reconnecting&#8230;';evtSrc.close();setTimeout(connect,3000);};
}
connect();
</script>
</body>
</html>"""


def run_web(yukon_path, host, port):
    import socket
    def _local_ip():
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.connect(('8.8.8.8', 80))
                return s.getsockname()[0]
        except Exception:
            return 'localhost'

    app = _build_web_app()
    print(f"Web GUI    : http://{_local_ip()}:{port}/", file=sys.stderr)
    import logging
    logging.getLogger('werkzeug').setLevel(logging.WARNING)
    app.run(host=host, port=port, threaded=True, use_reloader=False)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Yukon serial simulator",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "Modes:\n"
            "  gui       Pygame GUI (default)\n"
            "  web       Browser dashboard\n"
            "  headless  Terminal display\n"
        ),
    )
    parser.add_argument('--mode', choices=['gui', 'web', 'headless'],
                        default='gui',
                        help='Display mode (default: gui)')
    parser.add_argument('--host', default='0.0.0.0',
                        help='Web mode bind address (default: 0.0.0.0)')
    parser.add_argument('--port', default=5002, type=int,
                        help='Web mode port (default: 5002)')
    parser.add_argument('--ibus-port', metavar='DEV',
                        help='iBUS PTY/device to read RC channels from '
                             '(e.g. the PTY printed by ibus_sim.py)')
    args = parser.parse_args()

    # Create PTY
    yukon_master, yukon_slave = pty.openpty()
    tty.setraw(yukon_slave)
    path = os.ttyname(yukon_slave)
    _yukon_path[0] = path

    print(f"Yukon PTY  : {path}", file=sys.stderr)
    print(f"Connect with --yukon-port {path}", file=sys.stderr)
    if args.mode == 'web':
        print(f"Web port   : {args.port}", file=sys.stderr)

    with _lock:
        _state['last_imu_tick'] = time.monotonic()

    t_srv = threading.Thread(target=yukon_server, args=(yukon_master,), daemon=True)
    t_srv.start()

    if args.ibus_port:
        print(f"iBUS port  : {args.ibus_port}", file=sys.stderr)
        start_ibus_reader(args.ibus_port)

    try:
        if args.mode == 'gui':
            run_gui(path)
        elif args.mode == 'web':
            run_web(path, args.host, args.port)
        else:
            run_headless(path)
    finally:
        with _lock:
            _state['running'] = False
        os.close(yukon_master)
        os.close(yukon_slave)
        print("\nSimulator stopped.", file=sys.stderr)


if __name__ == '__main__':
    main()
