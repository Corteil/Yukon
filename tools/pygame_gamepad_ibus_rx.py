#!/usr/bin/env python3
"""
pygame_gamepad_ibus_rx.py — Gamepad → iBUS receiver simulator

Uses a pygame joystick/gamepad to drive iBUS packets on a PTY, replacing the
keyboard-based ibus_sim.py for use with a physical gamepad or RC-style joystick.

Channel layout matches robot.ini defaults:
  CH1   Aileron      right stick ← →
  CH2   Elevator     right stick ↑ ↓
  CH3   Throttle     left  stick ↑ ↓
  CH4   Rudder       left  stick ← →
  CH5   SwA  mode    button toggle  (MANUAL / AUTO)
  CH6   SwB  speed   button cycle   (slow / mid / max)
  CH7   SwC  type    button cycle   (Camera / GPS / Cam+GPS)
  CH8   SwD  GPS log button toggle  (off / on)
  CH10  Bookmark     momentary button (auto-returns after 300 ms)

Default button layout (Xbox-style / generic USB gamepad):
  Axis 0   Left  stick H  → CH4 Rudder
  Axis 1   Left  stick V  → CH3 Throttle  (inverted: up = 2000)
  Axis 2   Right stick H  → CH1 Aileron   (generic; Xbox users: --axis-aileron=3)
  Axis 3   Right stick V  → CH2 Elevator  (generic; Xbox users: --axis-elevator=4)
  Btn  0   (A / Cross)    → CH5 mode toggle
  Btn  1   (B / Circle)   → CH6 speed cycle
  Btn  2   (X / Square)   → CH7 type cycle
  Btn  3   (Y / Triangle) → CH8 GPS log toggle
  Btn  4   (LB / L1)      → CH10 bookmark (momentary)
  Btn  6   (Back/Select)  → Centre sticks + throttle to 1000
  Btn  7   (Start)        → Toggle RC signal loss

Xbox One/360 on Linux — right stick is on axes 3/4, not 2/3:
  python3 tools/pygame_gamepad_ibus_rx.py --axis-aileron=3 --axis-elevator=4

Usage:
  python3 tools/pygame_gamepad_ibus_rx.py [options]
  python3 rc_drive.py --ibus-port <PTY path printed on stderr>

Options:
  --list-joysticks    Print detected gamepads and exit
  --joystick N        Joystick index to use (default: 0)
  --hz HZ             iBUS packet rate in Hz (default: 143 ≈ 7 ms/packet)
  --deadzone D        Axis deadzone 0.0–1.0 (default: 0.05)
  --axis-rudder N     Axis index for CH4 Rudder          (default: 0)
  --axis-throttle N   Axis index for CH3 Throttle        (default: 1)
  --axis-aileron N    Axis index for CH1 Aileron         (default: 2)
  --axis-elevator N   Axis index for CH2 Elevator        (default: 3)
  --btn-mode N        Button for CH5 mode toggle         (default: 0)
  --btn-speed N       Button for CH6 speed cycle         (default: 1)
  --btn-type N        Button for CH7 type cycle          (default: 2)
  --btn-gpslog N      Button for CH8 GPS log toggle      (default: 3)
  --btn-bookmark N    Button for CH10 bookmark           (default: 4)
  --btn-centre N      Button to centre sticks            (default: 6)
  --btn-signal N      Button to toggle signal loss       (default: 7)
"""

import os
import sys
import pty
import tty
import struct
import threading
import time
import argparse

import pygame

# ── iBUS protocol constants ───────────────────────────────────────────────────

PACKET_LEN   = 32
NUM_CHANNELS = 14
CH_MIN       = 1000
CH_MAX       = 2000
CH_MID       = 1500
PACKET_HZ    = 143        # ~7 ms per packet — same as a real FlySky receiver

# ── Channel indices (0-based) ─────────────────────────────────────────────────

CH_AILERON   = 0    # CH1   right stick horizontal
CH_ELEVATOR  = 1    # CH2   right stick vertical
CH_THROTTLE  = 2    # CH3   left  stick vertical
CH_RUDDER    = 3    # CH4   left  stick horizontal
CH_MODE      = 4    # CH5   SwA 2-pos
CH_SPEED     = 5    # CH6   SwB 3-pos
CH_AUTO_TYPE = 6    # CH7   SwC 3-pos
CH_GPS_LOG   = 7    # CH8   SwD 2-pos
                    # CH9   unused (index 8)
CH_BOOKMARK  = 9    # CH10  momentary

# ── Named switch positions ────────────────────────────────────────────────────

SWA_NAMES = {1000: 'MANUAL', 2000: 'AUTO'}
SWB_NAMES = {1000: 'slow 25%', 1500: 'mid', 2000: 'max'}
SWC_NAMES = {1000: 'Camera', 1500: 'GPS', 2000: 'Cam+GPS'}
SWD_NAMES = {1000: 'off', 2000: 'on'}

# ── Shared state ──────────────────────────────────────────────────────────────

_lock  = threading.Lock()
_state = {
    'channels'       : [CH_MID] * NUM_CHANNELS,
    'rc_valid'       : True,
    'packets_sent'   : 0,
    'running'        : True,
    'bookmark_until' : 0.0,
}

# Safe power-on defaults
_state['channels'][CH_THROTTLE]  = CH_MIN    # throttle at bottom for safety
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

# ── iBUS sender thread ────────────────────────────────────────────────────────

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
            # Auto-release bookmark
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
            if time.monotonic() > next_send + interval * 5:
                next_send = time.monotonic()
        else:
            time.sleep(max(0.0, next_send - time.monotonic() - 0.0005))

# ── Axis helpers ──────────────────────────────────────────────────────────────

def _axis_to_ch(val, invert=False, deadzone=0.05):
    """Map a pygame axis value (-1.0…+1.0) to an iBUS channel (1000…2000)."""
    if abs(val) < deadzone:
        val = 0.0
    if invert:
        val = -val
    return int(CH_MID + val * (CH_MAX - CH_MID))


def _cycle3(val):
    """Cycle a 3-position switch: 1000 → 1500 → 2000 → 1000."""
    if val <= 1000:
        return 1500
    if val <= 1500:
        return 2000
    return 1000

# ── Pygame colours ────────────────────────────────────────────────────────────

BLACK  = (  0,   0,   0)
WHITE  = (255, 255, 255)
GREEN  = (  0, 200,   0)
RED    = (200,   0,   0)
BLUE   = ( 60, 140, 220)
GREY   = ( 80,  80,  80)
LGREY  = (140, 140, 140)
YELLOW = (220, 200,   0)
ORANGE = (220, 120,   0)
TEAL   = (  0, 180, 180)

WIN_W = 860
WIN_H = 620

# ── Drawing helpers ───────────────────────────────────────────────────────────

def _draw_channel_bar(surface, font, x, y, label, val, color=GREEN):
    """Draw a labelled horizontal channel bar."""
    BAR_W = 280
    BAR_H = 16
    frac  = (val - CH_MIN) / (CH_MAX - CH_MIN)

    pygame.draw.rect(surface, GREY, (x + 130, y, BAR_W, BAR_H))
    fill_w = int(frac * BAR_W)
    pygame.draw.rect(surface, color, (x + 130, y, fill_w, BAR_H))
    # centre marker
    cx = x + 130 + BAR_W // 2
    pygame.draw.line(surface, WHITE, (cx, y), (cx, y + BAR_H), 1)

    lbl = font.render(label, True, WHITE)
    surface.blit(lbl, (x, y))
    val_lbl = font.render(f'{val:4d}µs', True, WHITE)
    surface.blit(val_lbl, (x + 130 + BAR_W + 6, y))


def _draw_stick(surface, cx, cy, radius, ch_h, ch_v, label, font):
    """Draw a 2-D stick-position indicator.

    ch_h: horizontal channel (1000=left, 2000=right)
    ch_v: vertical channel   (1000=down, 2000=up)
    """
    # Background circle + crosshair
    pygame.draw.circle(surface, GREY, (cx, cy), radius, 2)
    pygame.draw.line(surface, GREY, (cx - radius, cy), (cx + radius, cy), 1)
    pygame.draw.line(surface, GREY, (cx, cy - radius), (cx, cy + radius), 1)

    fh = (ch_h - CH_MID) / (CH_MAX - CH_MID)   # -1..+1  right=positive
    fv = (ch_v - CH_MID) / (CH_MAX - CH_MID)   # -1..+1  up=positive (RC)

    dx = int(fh * radius)
    dy = int(fv * radius)   # pygame y-axis is inverted, compensate below

    pygame.draw.circle(surface, GREEN, (cx + dx, cy - dy), 8)

    lbl = font.render(label, True, LGREY)
    surface.blit(lbl, (cx - lbl.get_width() // 2, cy + radius + 4))


def _section(surface, font, x, y, text):
    lbl = font.render(text, True, LGREY)
    surface.blit(lbl, (x, y))
    return y + lbl.get_height() + 4

# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='Gamepad → iBUS receiver simulator (pygame)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument('--list-joysticks', action='store_true',
                        help='Print detected gamepads and exit')
    parser.add_argument('--joystick',      type=int,   default=0,
                        help='Joystick index to use (default: 0)')
    parser.add_argument('--hz',            type=float, default=PACKET_HZ,
                        help=f'Packet rate in Hz (default: {PACKET_HZ})')
    parser.add_argument('--deadzone',      type=float, default=0.05,
                        help='Axis deadzone 0.0–1.0 (default: 0.05)')
    # Axis assignments
    parser.add_argument('--axis-rudder',   type=int, default=0,
                        help='Axis → CH4 Rudder          (default: 0, left H)')
    parser.add_argument('--axis-throttle', type=int, default=1,
                        help='Axis → CH3 Throttle        (default: 1, left V)')
    parser.add_argument('--axis-aileron',  type=int, default=2,
                        help='Axis → CH1 Aileron         (default: 2, right H; Xbox: 3)')
    parser.add_argument('--axis-elevator', type=int, default=3,
                        help='Axis → CH2 Elevator        (default: 3, right V; Xbox: 4)')
    # Button assignments
    parser.add_argument('--btn-mode',      type=int, default=0,
                        help='Button → CH5 mode toggle   (default: 0, A/Cross)')
    parser.add_argument('--btn-speed',     type=int, default=1,
                        help='Button → CH6 speed cycle   (default: 1, B/Circle)')
    parser.add_argument('--btn-type',      type=int, default=2,
                        help='Button → CH7 type cycle    (default: 2, X/Square)')
    parser.add_argument('--btn-gpslog',    type=int, default=3,
                        help='Button → CH8 GPS log       (default: 3, Y/Triangle)')
    parser.add_argument('--btn-bookmark',  type=int, default=4,
                        help='Button → CH10 bookmark     (default: 4, LB/L1)')
    parser.add_argument('--btn-centre',    type=int, default=6,
                        help='Button → centre sticks     (default: 6, Back/Select)')
    parser.add_argument('--btn-signal',    type=int, default=7,
                        help='Button → toggle signal     (default: 7, Start)')
    args = parser.parse_args()

    # ── List joysticks and exit ───────────────────────────────────────────────
    pygame.init()
    pygame.joystick.init()

    if args.list_joysticks:
        count = pygame.joystick.get_count()
        if count == 0:
            print('No gamepads detected.')
        else:
            for i in range(count):
                j = pygame.joystick.Joystick(i)
                j.init()
                print(f'[{i}] {j.get_name()}  '
                      f'axes={j.get_numaxes()}  '
                      f'buttons={j.get_numbuttons()}  '
                      f'hats={j.get_numhats()}')
        pygame.quit()
        return

    # ── Create PTY ────────────────────────────────────────────────────────────
    master_fd, slave_fd = pty.openpty()
    tty.setraw(slave_fd)
    pty_path = os.ttyname(slave_fd)

    print(f'iBUS PTY  : {pty_path}', file=sys.stderr)
    print(f'rc_drive  : python3 rc_drive.py --ibus-port {pty_path}', file=sys.stderr)
    print(f'yukon_sim : python3 tools/yukon_sim.py --ibus-port {pty_path}', file=sys.stderr)

    # ── Start iBUS sender thread ──────────────────────────────────────────────
    sender = threading.Thread(
        target=_ibus_sender, args=(master_fd, args.hz), daemon=True
    )
    sender.start()

    # ── Pygame setup ──────────────────────────────────────────────────────────
    screen = pygame.display.set_mode((WIN_W, WIN_H))
    pygame.display.set_caption('Gamepad iBUS Simulator')
    clock   = pygame.time.Clock()
    font    = pygame.font.SysFont('monospace', 16)
    font_sm = pygame.font.SysFont('monospace', 13)

    # Open the joystick (if any present)
    joy = None
    if pygame.joystick.get_count() > 0:
        idx = min(args.joystick, pygame.joystick.get_count() - 1)
        joy = pygame.joystick.Joystick(idx)
        joy.init()
        print(f'Gamepad   : [{idx}] {joy.get_name()}', file=sys.stderr)
    else:
        print('WARNING: No gamepad detected — channels will hold defaults.', file=sys.stderr)

    # Per-button rising-edge tracker (toggle on press, not hold)
    btn_prev: dict[int, int] = {}

    def _btn_pressed(btn_idx: int) -> bool:
        """Return True on the rising edge of a button press."""
        if joy is None or btn_idx >= joy.get_numbuttons():
            return False
        cur  = joy.get_button(btn_idx)
        prev = btn_prev.get(btn_idx, 0)
        btn_prev[btn_idx] = cur
        return bool(cur and not prev)

    def _safe_axis(idx: int) -> float:
        if joy is None or idx >= joy.get_numaxes():
            return 0.0
        return joy.get_axis(idx)

    try:
        while True:
            # ── Event pump ────────────────────────────────────────────────────
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    with _lock:
                        _state['running'] = False
                    return
                if event.type == pygame.JOYDEVICEADDED and joy is None:
                    joy = pygame.joystick.Joystick(args.joystick)
                    joy.init()
                    btn_prev.clear()
                    print(f'Gamepad reconnected: {joy.get_name()}', file=sys.stderr)
                if event.type == pygame.JOYDEVICEREMOVED:
                    joy = None
                    print('Gamepad disconnected.', file=sys.stderr)

            # ── Read axes → channels ──────────────────────────────────────────
            if joy is not None:
                with _lock:
                    chs = _state['channels']
                    # Sticks: vertical axes inverted (axis up = −1.0 → 2000)
                    chs[CH_RUDDER]   = _axis_to_ch(
                        _safe_axis(args.axis_rudder),   invert=False, deadzone=args.deadzone)
                    chs[CH_THROTTLE] = _axis_to_ch(
                        _safe_axis(args.axis_throttle), invert=True,  deadzone=args.deadzone)
                    chs[CH_AILERON]  = _axis_to_ch(
                        _safe_axis(args.axis_aileron),  invert=False, deadzone=args.deadzone)
                    chs[CH_ELEVATOR] = _axis_to_ch(
                        _safe_axis(args.axis_elevator), invert=True,  deadzone=args.deadzone)

                # ── Button edge detection → switches ──────────────────────────
                pressed = {
                    'mode'    : _btn_pressed(args.btn_mode),
                    'speed'   : _btn_pressed(args.btn_speed),
                    'type'    : _btn_pressed(args.btn_type),
                    'gpslog'  : _btn_pressed(args.btn_gpslog),
                    'bookmark': _btn_pressed(args.btn_bookmark),
                    'centre'  : _btn_pressed(args.btn_centre),
                    'signal'  : _btn_pressed(args.btn_signal),
                }
                with _lock:
                    chs = _state['channels']
                    if pressed['mode']:
                        chs[CH_MODE]      = 2000 if chs[CH_MODE]    == 1000 else 1000
                    if pressed['speed']:
                        chs[CH_SPEED]     = _cycle3(chs[CH_SPEED])
                    if pressed['type']:
                        chs[CH_AUTO_TYPE] = _cycle3(chs[CH_AUTO_TYPE])
                    if pressed['gpslog']:
                        chs[CH_GPS_LOG]   = 2000 if chs[CH_GPS_LOG] == 1000 else 1000
                    if pressed['bookmark']:
                        chs[CH_BOOKMARK]  = CH_MAX
                        _state['bookmark_until'] = time.monotonic() + 0.30
                    if pressed['centre']:
                        chs[CH_AILERON]   = CH_MID
                        chs[CH_ELEVATOR]  = CH_MID
                        chs[CH_RUDDER]    = CH_MID
                        chs[CH_THROTTLE]  = CH_MIN   # throttle to safe bottom
                    if pressed['signal']:
                        _state['rc_valid'] = not _state['rc_valid']

            # ── Snapshot state for rendering ──────────────────────────────────
            with _lock:
                ch    = list(_state['channels'])
                valid = _state['rc_valid']
                pkts  = _state['packets_sent']

            # ── Draw ──────────────────────────────────────────────────────────
            screen.fill(BLACK)
            y = 8

            # Header
            title = font.render('Gamepad iBUS Simulator', True, YELLOW)
            screen.blit(title, (10, y))
            y += title.get_height() + 4

            pty_lbl = font_sm.render(f'PTY: {pty_path}     Packets: {pkts}', True, WHITE)
            screen.blit(pty_lbl, (10, y))
            y += pty_lbl.get_height() + 4

            sig_color = GREEN if valid else RED
            sig_lbl   = font.render(
                f'Signal: {"OK" if valid else "LOST  (press Start to restore)"}',
                True, sig_color,
            )
            screen.blit(sig_lbl, (10, y))
            y += sig_lbl.get_height() + 4

            if joy is None:
                warn = font.render('No gamepad detected — plug one in to continue', True, RED)
                screen.blit(warn, (10, y))
            else:
                joy_lbl = font_sm.render(f'Gamepad: {joy.get_name()}', True, TEAL)
                screen.blit(joy_lbl, (10, y))
            y += 22

            # ── Channel bars ──────────────────────────────────────────────────
            y = _section(screen, font_sm, 10, y,
                         '─── Channels ─────────────────────────────────────────────')
            bars = [
                ('CH1  Aileron',  ch[CH_AILERON],   BLUE),
                ('CH2  Elevator', ch[CH_ELEVATOR],  BLUE),
                ('CH3  Throttle', ch[CH_THROTTLE],  GREEN),
                ('CH4  Rudder',   ch[CH_RUDDER],    GREEN),
            ]
            for lbl, val, color in bars:
                _draw_channel_bar(screen, font_sm, 10, y, lbl, val, color)
                y += 20
            y += 4

            # ── Switches ──────────────────────────────────────────────────────
            y = _section(screen, font_sm, 10, y,
                         '─── Switches ─────────────────────────────────────────────')
            swa = ch[CH_MODE]
            swb = ch[CH_SPEED]
            swc = ch[CH_AUTO_TYPE]
            swd = ch[CH_GPS_LOG]
            bkm = ch[CH_BOOKMARK]

            switch_rows = [
                (f'CH5  SwA Mode   : {SWA_NAMES.get(swa, str(swa)):<12s}',  swa == 2000),
                (f'CH6  SwB Speed  : {SWB_NAMES.get(swb, str(swb)):<12s}',  swb != 1000),
                (f'CH7  SwC Type   : {SWC_NAMES.get(swc, str(swc)):<12s}',  swc != 1000),
                (f'CH8  SwD GPSLog : {SWD_NAMES.get(swd, str(swd)):<12s}',  swd == 2000),
                (f'CH10 Bookmark   : {"TRIGGERED" if bkm >= CH_MID else "ready":<12s}',
                 bkm >= CH_MID),
            ]
            for text, active in switch_rows:
                color = ORANGE if active else WHITE
                lbl = font_sm.render(text, True, color)
                screen.blit(lbl, (10, y))
                y += lbl.get_height() + 2
            y += 6

            # ── Control reference ─────────────────────────────────────────────
            y = _section(screen, font_sm, 10, y,
                         '─── Controls ─────────────────────────────────────────────')
            ctrl_lines = [
                f'Left  stick H  axis {args.axis_rudder}   → Rudder',
                f'Left  stick V  axis {args.axis_throttle}   → Throttle (up=2000)',
                f'Right stick H  axis {args.axis_aileron}   → Aileron',
                f'Right stick V  axis {args.axis_elevator}   → Elevator (up=2000)',
                f'Btn {args.btn_mode} (A)       → Mode toggle      '
                f'Btn {args.btn_speed} (B) → Speed cycle',
                f'Btn {args.btn_type} (X)       → Type cycle       '
                f'Btn {args.btn_gpslog} (Y) → GPS log toggle',
                f'Btn {args.btn_bookmark} (LB)      → Bookmark         '
                f'Btn {args.btn_centre} (Back) → Centre sticks',
                f'Btn {args.btn_signal} (Start)   → Toggle signal loss',
                'Close window  → Quit',
            ]
            for line in ctrl_lines:
                lbl = font_sm.render(line, True, LGREY)
                screen.blit(lbl, (10, y))
                y += lbl.get_height() + 1

            # ── Stick visualisers (right side) ────────────────────────────────
            _draw_stick(screen, 650, 200, 75,
                        ch[CH_RUDDER], ch[CH_THROTTLE],
                        'Left Stick', font_sm)
            _draw_stick(screen, 790, 200, 75,
                        ch[CH_AILERON], ch[CH_ELEVATOR],
                        'Right Stick', font_sm)

            pygame.display.flip()
            clock.tick(30)

    finally:
        with _lock:
            _state['running'] = False
        pygame.quit()
        try:
            os.close(master_fd)
            os.close(slave_fd)
        except OSError:
            pass
        print('\niBUS gamepad simulator stopped.', file=sys.stderr)


if __name__ == '__main__':
    main()
