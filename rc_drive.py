#!/usr/bin/env python3
"""
rc_drive.py — FlySky iBUS RC control for HackyRacingRobot.

Reads iBUS channels from /dev/ttyAMA3 (GPIO 9, uart3-pi5 overlay),
applies throttle/aileron tank-mix, and sends CMD_LEFT / CMD_RIGHT
to the Yukon over USB serial (/dev/ttyACM0).

Tank mix (differential drive):
    left  = clamp(throttle - aileron, -1, 1)
    right = clamp(throttle + aileron, -1, 1)

Failsafe: if no valid iBUS packet is received for FAILSAFE_TIMEOUT seconds,
CMD_KILL is sent and motors stay stopped until signal returns.

Usage:
    python3 rc_drive.py
    python3 rc_drive.py --port /dev/ttyACM0 --ibus-port /dev/ttyAMA3
    python3 rc_drive.py --reverse-left          # flip left-side motor direction
    python3 rc_drive.py --reverse-right         # flip right-side motor direction
    python3 rc_drive.py --throttle-ch 3 --steer-ch 1   # default channel mapping
"""

import sys
import time
import queue
import threading
import argparse

from drivers.ibus import IBusReader, IBusError

# ---------------------------------------------------------------------------
# Yukon serial protocol constants (mirrors test_main.py)
# ---------------------------------------------------------------------------

SYNC = 0x7E
ACK  = 0x06
NAK  = 0x15

CMD_LEFT  = 2
CMD_RIGHT = 3
CMD_KILL  = 4

# ---------------------------------------------------------------------------
# RC / mixing config
# ---------------------------------------------------------------------------

DEADZONE         = 30    # µs either side of CH_MID treated as zero
FAILSAFE_TIMEOUT = 0.5   # seconds without a valid packet → kill motors
RAMP_RATE        = 1.0   # max speed change per second (full range in 0.5 s); 0 = instant
SPEED_CH         = 5     # CH6 — SwB speed limit: 1000=slow, 1500=mid, 2000=max
SPEED_MIN        = 0.25  # scale factor at CH6=1000

# ---------------------------------------------------------------------------
# Protocol helpers
# ---------------------------------------------------------------------------

def _encode(cmd_code, byte_value):
    cmd    = cmd_code + 0x20
    v_high = (byte_value >> 4)   + 0x40
    v_low  = (byte_value & 0x0F) + 0x50
    chk    = cmd ^ v_high ^ v_low
    return bytes([SYNC, cmd, v_high, v_low, chk])


def _speed_to_byte(speed):
    """Encode motor speed (-1.0..+1.0) to Yukon wire byte."""
    speed = max(-1.0, min(1.0, speed))
    if speed >= 0.0:
        return round(speed * 100)
    return 100 + round(abs(speed) * 100)


def _normalize(ch_val):
    """Map iBUS channel value (1000-2000) to -1.0..+1.0 with deadzone."""
    raw = ch_val - 1500
    if abs(raw) < DEADZONE:
        return 0.0
    half = 500.0
    return max(-1.0, min(1.0, raw / half))


def _speed_scale(ch_val):
    """Map CH6 (1000-2000) linearly to SPEED_MIN..1.0."""
    t = max(0.0, min(1.0, (ch_val - 1000) / 1000.0))
    return SPEED_MIN + t * (1.0 - SPEED_MIN)


def _tank_mix(throttle_raw, steer_raw, reverse_left, reverse_right):
    """
    Differential / tank mixing.
    Returns (left, right) each in range -1.0..+1.0.
    """
    thr  = _normalize(throttle_raw)
    ste  = _normalize(steer_raw)
    left  = max(-1.0, min(1.0, thr - ste))
    right = max(-1.0, min(1.0, thr + ste))
    if reverse_left:
        left = -left
    if reverse_right:
        right = -right
    return left, right


# ---------------------------------------------------------------------------
# Yukon serial link
# ---------------------------------------------------------------------------

class YukonLink:
    """Send motor commands to the Yukon; drain ACKs in the background."""

    def __init__(self, port, baud=115200):
        import serial
        # dsrdtr=False prevents DTR toggling that would reset the RP2040
        self.ser   = serial.Serial(port, baud, timeout=0.1, dsrdtr=False)
        time.sleep(1.0)
        self.ser.reset_input_buffer()
        self._ack_q = queue.Queue()
        self._stop  = threading.Event()
        self._rx    = threading.Thread(target=self._reader, daemon=True)
        self._rx.start()

    def _reader(self):
        import serial as _serial
        while not self._stop.is_set():
            try:
                data = self.ser.read(1)
            except _serial.SerialException:
                break
            if not data:
                continue
            b = data[0]
            if b in (ACK, NAK):
                self._ack_q.put(b == ACK)

    def _drain_ack(self, timeout=0.1):
        try:
            self._ack_q.get(timeout=timeout)
        except queue.Empty:
            pass

    def send(self, cmd_code, byte_value):
        self.ser.write(_encode(cmd_code, byte_value))
        self._drain_ack()

    def kill(self):
        self.ser.write(_encode(CMD_KILL, 0))
        self._drain_ack(timeout=0.5)

    def close(self):
        self._stop.set()
        try:
            self.kill()
        except Exception:
            pass
        self.ser.close()


# ---------------------------------------------------------------------------
# Main RC loop
# ---------------------------------------------------------------------------

def _ramp(current, target, max_delta):
    """Step current toward target by at most max_delta."""
    diff = target - current
    if abs(diff) <= max_delta:
        return target
    return current + (max_delta if diff > 0 else -max_delta)


def rc_loop(yukon, ibus, throttle_ch, steer_ch, reverse_left, reverse_right, ramp_rate=RAMP_RATE):
    last_left_b   = None
    last_right_b  = None
    last_packet   = time.monotonic()
    failsafe      = False
    current_left  = 0.0
    current_right = 0.0
    target_left   = 0.0
    target_right  = 0.0
    last_time     = time.monotonic()

    print("RC control active. Ctrl+C to stop.")
    print(f"  throttle=CH{throttle_ch + 1}  steer=CH{steer_ch + 1}"
          f"  reverse_left={reverse_left}  reverse_right={reverse_right}"
          f"  ramp={ramp_rate}/s")

    while True:
        # ibus.read() blocks until a valid packet arrives (~7 ms) or timeout (1 s)
        channels = ibus.read()
        now = time.monotonic()
        dt  = min(now - last_time, 0.1)   # cap to avoid a large jump after a pause
        last_time = now

        if channels:
            last_packet = now
            if failsafe:
                print("RC signal restored.")
                failsafe = False

            scale = _speed_scale(channels[SPEED_CH])
            target_left, target_right = _tank_mix(
                channels[throttle_ch], channels[steer_ch],
                reverse_left, reverse_right,
            )
            target_left  *= scale
            target_right *= scale

        elif not failsafe and (now - last_packet) > FAILSAFE_TIMEOUT:
            print("WARNING: RC signal lost — killing motors.")
            yukon.kill()
            current_left = current_right = 0.0
            target_left  = target_right  = 0.0
            last_left_b  = last_right_b  = None
            failsafe = True

        if failsafe:
            continue

        # Ramp current speeds toward target
        if ramp_rate > 0:
            max_delta     = ramp_rate * dt
            current_left  = _ramp(current_left,  target_left,  max_delta)
            current_right = _ramp(current_right, target_right, max_delta)
        else:
            current_left  = target_left
            current_right = target_right

        left_b  = _speed_to_byte(current_left)
        right_b = _speed_to_byte(current_right)

        # Only send when the byte value actually changes
        if left_b != last_left_b:
            yukon.send(CMD_LEFT, left_b)
            last_left_b = left_b
        if right_b != last_right_b:
            yukon.send(CMD_RIGHT, right_b)
            last_right_b = right_b


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def _find_yukon_port():
    import serial.tools.list_ports
    for p in serial.tools.list_ports.comports():
        if p.vid == 0x2E8A:   # Raspberry Pi / RP2040 vendor ID
            return p.device
    ports = serial.tools.list_ports.comports()
    return ports[0].device if ports else None


def main():
    parser = argparse.ArgumentParser(description="iBUS RC drive for HackyRacingRobot")
    parser.add_argument("--port",          help="Yukon USB serial port (auto-detected if omitted)")
    parser.add_argument("--ibus-port",     default="/dev/ttyAMA3",
                        help="iBUS UART device (default: /dev/ttyAMA3)")
    parser.add_argument("--throttle-ch",   type=int, default=3, metavar="N",
                        help="Throttle channel, 1-based (default: 3)")
    parser.add_argument("--steer-ch",      type=int, default=1, metavar="N",
                        help="Steering/aileron channel, 1-based (default: 1)")
    parser.add_argument("--reverse-left",  action="store_true",
                        help="Reverse left motor direction")
    parser.add_argument("--reverse-right", action="store_true",
                        help="Reverse right motor direction")
    parser.add_argument("--ramp-rate",     type=float, default=None, metavar="R",
                        help=f"Max speed change per second (default: {RAMP_RATE}); 0 = instant")
    args = parser.parse_args()

    ramp_rate = max(0.0, args.ramp_rate) if args.ramp_rate is not None else RAMP_RATE

    # Convert 1-based channel numbers to 0-based indices
    throttle_ch = args.throttle_ch - 1
    steer_ch    = args.steer_ch - 1
    if not (0 <= throttle_ch <= 13) or not (0 <= steer_ch <= 13):
        print("Channel numbers must be between 1 and 14.")
        sys.exit(1)

    try:
        import serial
    except ImportError:
        print("pyserial not installed. Run:  pip install pyserial")
        sys.exit(1)

    port = args.port or _find_yukon_port()
    if not port:
        print("No Yukon port found. Use --port /dev/ttyACM0.")
        sys.exit(1)

    print(f"Connecting to Yukon on {port} ...")
    yukon = YukonLink(port)

    print(f"Opening iBUS on {args.ibus_port} ...")
    try:
        with IBusReader(args.ibus_port) as ibus:
            try:
                rc_loop(yukon, ibus, throttle_ch, steer_ch,
                        args.reverse_left, args.reverse_right,
                        ramp_rate)
            except KeyboardInterrupt:
                print("\nStopping.")
    except IBusError as e:
        print(f"iBUS error: {e}")
        sys.exit(1)
    finally:
        yukon.close()


if __name__ == "__main__":
    main()
