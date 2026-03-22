#!/usr/bin/env python3
"""
test_leds.py — LED strip tests for the Yukon NeoPixel module (SLOT3).

Tests all colour presets, individual pixel control, set_pixels(), and all
built-in patterns.  With --dry-run, only protocol encoding is tested — no
hardware required.

Usage:
    python3 tools/test_leds.py --dry-run
    python3 tools/test_leds.py --yukon-port /dev/ttyACM0
"""

import sys
import os
import time
import argparse

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# ── Protocol constants (mirrors robot_daemon._YukonLink) ──────────────────────

SYNC           = 0x7E
ACK            = 0x06
NAK            = 0x15

CMD_LED        = 1
CMD_STRIP      = 7
CMD_PIXEL_SET  = 8
CMD_PIXEL_SHOW = 9
CMD_PATTERN    = 10

LED_A_OFF = 0
LED_A_ON  = 1
LED_B_OFF = 2
LED_B_ON  = 3

STRIP_OFF     = 0
STRIP_RED     = 1
STRIP_GREEN   = 2
STRIP_BLUE    = 3
STRIP_ORANGE  = 4
STRIP_YELLOW  = 5
STRIP_CYAN    = 6
STRIP_MAGENTA = 7
STRIP_WHITE   = 8

STRIP_NAMES = {
    STRIP_OFF:     'off',
    STRIP_RED:     'red',
    STRIP_GREEN:   'green',
    STRIP_BLUE:    'blue',
    STRIP_ORANGE:  'orange',
    STRIP_YELLOW:  'yellow',
    STRIP_CYAN:    'cyan',
    STRIP_MAGENTA: 'magenta',
    STRIP_WHITE:   'white',
}

PATTERN_OFF            = 0
PATTERN_LARSON         = 1
PATTERN_RANDOM         = 2
PATTERN_RAINBOW        = 3
PATTERN_RETRO_COMPUTER = 4
PATTERN_CONVERGE       = 5
PATTERN_ESTOP_FLASH    = 6

PATTERN_NAMES = {
    PATTERN_OFF:            'off',
    PATTERN_LARSON:         'larson',
    PATTERN_RANDOM:         'random',
    PATTERN_RAINBOW:        'rainbow',
    PATTERN_RETRO_COMPUTER: 'retro_computer',
    PATTERN_CONVERGE:       'converge',
    PATTERN_ESTOP_FLASH:    'estop_flash',
}

NUM_LEDS = 8

# ── Test counters ─────────────────────────────────────────────────────────────

passed = 0
failed = 0


def _pass(msg):
    global passed
    passed += 1
    print(f'  PASS  {msg}')


def _fail(msg):
    global failed
    failed += 1
    print(f'  FAIL  {msg}')


# ── Protocol helpers ──────────────────────────────────────────────────────────

def _encode(cmd_code, value):
    cmd    = cmd_code + 0x20
    v_high = (value >> 4) + 0x40
    v_low  = (value & 0x0F) + 0x50
    chk    = cmd ^ v_high ^ v_low
    return bytes([SYNC, cmd, v_high, v_low, chk])


def _drain_ack(ser, timeout=0.5):
    """Wait for ACK or NAK from the Yukon, discarding any other bytes."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        b = ser.read(1)
        if b and b[0] in (ACK, NAK):
            return b[0]
    return None


def _send(ser, pkt, label):
    """Send packet, wait for ACK, report pass/fail."""
    ser.write(pkt)
    resp = _drain_ack(ser)
    if resp == ACK:
        _pass(label)
        return True
    _fail(f'{label}  (got {hex(resp) if resp else "timeout"})')
    return False


# ── Dry-run encoding tests ────────────────────────────────────────────────────

def test_encoding():
    print('\n--- Protocol encoding tests (no hardware) ---')

    # CMD_LED: all four values
    for value, label in ((LED_A_OFF, 'A off'), (LED_A_ON, 'A on'),
                         (LED_B_OFF, 'B off'), (LED_B_ON,  'B on')):
        pkt      = _encode(CMD_LED, value)
        cmd_code = pkt[1] - 0x20
        decoded  = ((pkt[2] - 0x40) << 4) | (pkt[3] - 0x50)
        chk      = pkt[1] ^ pkt[2] ^ pkt[3]
        if cmd_code == CMD_LED and decoded == value and chk == pkt[4]:
            _pass(f'CMD_LED encode  {label}')
        else:
            _fail(f'CMD_LED encode  {label}')

    # CMD_STRIP: all 9 presets
    for idx in range(9):
        pkt      = _encode(CMD_STRIP, idx)
        cmd_code = pkt[1] - 0x20
        value    = ((pkt[2] - 0x40) << 4) | (pkt[3] - 0x50)
        chk      = pkt[1] ^ pkt[2] ^ pkt[3]
        if cmd_code == CMD_STRIP and value == idx and chk == pkt[4]:
            _pass(f'CMD_STRIP encode  preset={idx} ({STRIP_NAMES.get(idx, "?")})')
        else:
            _fail(f'CMD_STRIP encode  preset={idx}')

    # CMD_PIXEL_SET: spot-check index/colour packing
    for led_idx in (0, 3, 7):
        for colour_idx in (1, 5, 8):
            value          = ((led_idx & 0x0F) << 4) | (colour_idx & 0x0F)
            pkt            = _encode(CMD_PIXEL_SET, value)
            decoded        = ((pkt[2] - 0x40) << 4) | (pkt[3] - 0x50)
            decoded_led    = decoded >> 4
            decoded_colour = decoded & 0x0F
            if decoded_led == led_idx and decoded_colour == colour_idx:
                _pass(f'CMD_PIXEL_SET encode  led={led_idx} colour={colour_idx}')
            else:
                _fail(f'CMD_PIXEL_SET encode  led={led_idx} colour={colour_idx}')

    # CMD_PIXEL_SHOW
    pkt = _encode(CMD_PIXEL_SHOW, 0)
    if pkt[1] - 0x20 == CMD_PIXEL_SHOW:
        _pass('CMD_PIXEL_SHOW encode')
    else:
        _fail('CMD_PIXEL_SHOW encode')

    # CMD_PATTERN: all patterns with and without colour nibble
    for pat in range(6):
        for colour in (0, 3, 8):
            value      = ((colour & 0x0F) << 4) | (pat & 0x0F)
            pkt        = _encode(CMD_PATTERN, value)
            v          = ((pkt[2] - 0x40) << 4) | (pkt[3] - 0x50)
            dec_pat    = v & 0x0F
            dec_colour = (v >> 4) & 0x0F
            if dec_pat == pat and dec_colour == colour:
                _pass(f'CMD_PATTERN encode  pattern={PATTERN_NAMES.get(pat, "?")} colour={colour}')
            else:
                _fail(f'CMD_PATTERN encode  pattern={pat} colour={colour}')


# ── Hardware tests ────────────────────────────────────────────────────────────

def test_hardware(ser):
    print('\n--- Onboard LED tests ---')
    _send(ser, _encode(CMD_LED, LED_A_ON),  'CMD_LED  A on')
    time.sleep(0.4)
    _send(ser, _encode(CMD_LED, LED_B_ON),  'CMD_LED  B on')
    time.sleep(0.4)
    _send(ser, _encode(CMD_LED, LED_A_OFF), 'CMD_LED  A off')
    time.sleep(0.4)
    _send(ser, _encode(CMD_LED, LED_B_OFF), 'CMD_LED  B off')
    time.sleep(0.3)

    print('\n--- Colour preset tests ---')
    for idx in range(9):
        _send(ser, _encode(CMD_STRIP, idx),
              f'CMD_STRIP  preset={idx} ({STRIP_NAMES.get(idx, "?")})')
        time.sleep(0.4)

    print('\n--- Turn strip off ---')
    _send(ser, _encode(CMD_STRIP, STRIP_OFF), 'CMD_STRIP  off')
    time.sleep(0.3)

    print('\n--- Individual pixel tests ---')
    colours = [STRIP_RED, STRIP_ORANGE, STRIP_YELLOW, STRIP_GREEN,
               STRIP_CYAN, STRIP_BLUE, STRIP_MAGENTA, STRIP_WHITE]
    for i, c in enumerate(colours):
        value = ((i & 0x0F) << 4) | (c & 0x0F)
        _send(ser, _encode(CMD_PIXEL_SET, value),
              f'CMD_PIXEL_SET  led={i} colour={STRIP_NAMES.get(c, "?")}')
    _send(ser, _encode(CMD_PIXEL_SHOW, 0), 'CMD_PIXEL_SHOW  (push to strip)')
    time.sleep(1.0)

    print('\n--- set_pixels: alternating red / off ---')
    pkt = b''.join(
        _encode(CMD_PIXEL_SET, ((i & 0x0F) << 4) | (STRIP_RED if i % 2 == 0 else STRIP_OFF))
        for i in range(NUM_LEDS)
    ) + _encode(CMD_PIXEL_SHOW, 0)
    ser.write(pkt)
    acks    = 0
    n_total = NUM_LEDS + 1
    deadline = time.monotonic() + 2.0
    while acks < n_total and time.monotonic() < deadline:
        b = ser.read(1)
        if b and b[0] == ACK:
            acks += 1
    if acks == n_total:
        _pass(f'set_pixels alternating red/off  ({acks} ACKs)')
    else:
        _fail(f'set_pixels alternating red/off  (got {acks}/{n_total} ACKs)')
    time.sleep(1.0)

    print('\n--- Pattern tests ---')
    for pat_id, pat_name in PATTERN_NAMES.items():
        if pat_id == PATTERN_OFF:
            continue
        _send(ser, _encode(CMD_PATTERN, pat_id & 0x0F), f'CMD_PATTERN  {pat_name}')
        print(f'       (watching {pat_name} for 2 s...)')
        time.sleep(2.0)

    print('\n--- ESTOP flash: red leds 0-2 flash, led 3 orange ---')
    _send(ser, _encode(CMD_PATTERN, PATTERN_ESTOP_FLASH & 0x0F),
          'CMD_PATTERN  estop_flash')
    time.sleep(3.0)

    print('\n--- Pattern with colour: converge cyan ---')
    _send(ser, _encode(CMD_PATTERN, (STRIP_CYAN << 4) | PATTERN_CONVERGE),
          'CMD_PATTERN  converge + cyan')
    time.sleep(2.0)

    print('\n--- Retro computer: magenta ---')
    _send(ser, _encode(CMD_PATTERN, (STRIP_MAGENTA << 4) | PATTERN_RETRO_COMPUTER),
          'CMD_PATTERN  retro_computer + magenta')
    time.sleep(2.0)

    print('\n--- Stop pattern ---')
    _send(ser, _encode(CMD_PATTERN, PATTERN_OFF), 'CMD_PATTERN  off')
    time.sleep(0.3)


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='LED strip tests for Yukon NeoPixel module')
    parser.add_argument('--yukon-port', default='/dev/ttyACM0',
                        help='Yukon serial port (default: /dev/ttyACM0)')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate')
    parser.add_argument('--dry-run', action='store_true',
                        help='Encoding tests only — no hardware required')
    args = parser.parse_args()

    test_encoding()

    if not args.dry_run:
        try:
            import serial
        except ImportError:
            print('\nERROR: pyserial not installed.  pip install pyserial')
            sys.exit(1)
        print(f'\nConnecting to {args.yukon_port} at {args.baud} baud...')
        try:
            ser = serial.Serial(args.yukon_port, args.baud, timeout=0.5)
            time.sleep(0.5)
            ser.reset_input_buffer()
        except Exception as e:
            print(f'ERROR: could not open {args.yukon_port}: {e}')
            sys.exit(1)
        print('Connected.\n')
        try:
            test_hardware(ser)
        finally:
            ser.close()

    total = passed + failed
    print(f'\n{"=" * 50}')
    print(f'  {passed}/{total} passed' +
          (f'  ({failed} FAILED)' if failed else '  -- all passed'))
    print(f'{"=" * 50}')
    sys.exit(0 if failed == 0 else 1)


if __name__ == '__main__':
    main()
