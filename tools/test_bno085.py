#!/usr/bin/env python3
"""
test_bno085.py — Unit tests and hardware verification for the BNO085 IMU.

The BNO085 driver (lib/bno085.py) runs on MicroPython and cannot be imported
directly on the Pi.  This file tests:

  1. Quaternion → heading math          (no hardware needed)
  2. Angle-difference wrap arithmetic   (no hardware needed)
  3. CMD_BEARING encode/decode          (no hardware needed)
  4. Bearing hold protocol round-trip   (requires Yukon connected + IMU fitted)
  5. Live heading display               (requires Yukon connected + IMU fitted)

Usage:
    python3 tests/test_bno085.py              # unit tests only
    python3 tests/test_bno085.py --hardware   # unit tests + hardware round-trip
    python3 tests/test_bno085.py --live       # unit tests + live heading display
    python3 tests/test_bno085.py --port /dev/ttyACM0 --live
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import math
import time
import argparse


# ── Pure-Python reimplementations of the bno085.py / main.py math ────────────
# (bno085.py uses MicroPython-only modules so we duplicate the functions here)

def quaternion_to_heading(qx, qy, qz, qw):
    """Convert Game Rotation Vector quaternion to heading 0–360°."""
    yaw = math.atan2(2.0 * (qw * qz + qx * qy),
                     1.0 - 2.0 * (qy * qy + qz * qz))
    return math.degrees(yaw) % 360.0


def angle_diff(target, current):
    """Signed shortest-arc difference (target − current), −180 to +180."""
    return (target - current + 180.0) % 360.0 - 180.0


def bearing_encode(degrees):
    """Encode a bearing (0–360°) to a protocol byte 0–254 (255 = disable)."""
    return min(254, round(degrees % 360.0 * 254.0 / 359.0))


def bearing_decode(value):
    """Decode a protocol byte 0–254 back to degrees."""
    return value * 359.0 / 254.0


# ── Protocol helpers (mirrors robot.py _YukonLink) ────────────────────────────

SYNC        = 0x7E
ACK         = 0x06
NAK         = 0x15
CMD_BEARING = 6


def encode_packet(cmd_code, byte_value):
    cmd    = cmd_code + 0x20
    v_high = (byte_value >> 4)   + 0x40
    v_low  = (byte_value & 0x0F) + 0x50
    chk    = cmd ^ v_high ^ v_low
    return bytes([SYNC, cmd, v_high, v_low, chk])


# ── Unit tests ─────────────────────────────────────────────────────────────────

def run_unit_tests():
    passed = 0
    failed = 0

    def check(name, result, expected, tol=0.0):
        nonlocal passed, failed
        if tol:
            ok = abs(result - expected) <= tol
        else:
            ok = result == expected
        print(f"  {'PASS' if ok else 'FAIL'}  {name}")
        if ok:
            passed += 1
        else:
            failed += 1
            print(f"          got {result!r}, expected {expected!r}")

    # ── Quaternion → heading ─────────────────────────────────────────────────
    print("\nQuaternion → heading:")

    # Identity quaternion → 0° (no rotation)
    check("identity → 0°",
          quaternion_to_heading(0, 0, 0, 1), 0.0, tol=0.1)

    # 90° yaw CW (z-axis rotation, right-hand rule: positive yaw = CCW)
    # qw=cos(45°), qz=sin(45°) → yaw = +90° → heading = 90°
    s = math.sin(math.radians(45))
    c = math.cos(math.radians(45))
    check("+90° yaw → 90°",
          quaternion_to_heading(0, 0, s, c), 90.0, tol=0.1)

    # 180° yaw
    check("180° yaw → 180°",
          quaternion_to_heading(0, 0, 1, 0), 180.0, tol=0.1)

    # -90° yaw → 270°
    check("-90° yaw → 270°",
          quaternion_to_heading(0, 0, -s, c), 270.0, tol=0.1)

    # 45° yaw
    s45 = math.sin(math.radians(22.5))
    c45 = math.cos(math.radians(22.5))
    check("45° yaw → 45°",
          quaternion_to_heading(0, 0, s45, c45), 45.0, tol=0.5)

    # ── Angle difference ─────────────────────────────────────────────────────
    print("\nAngle difference (shortest arc):")
    check("0→0 = 0",       angle_diff(0, 0),     0.0)
    check("90→0 = +90",    angle_diff(90, 0),    90.0)
    check("0→90 = -90",    angle_diff(0, 90),   -90.0)
    check("350→10 = -20",  angle_diff(350, 10), -20.0)
    check("10→350 = +20",  angle_diff(10, 350),  20.0)
    check("180→0 = ±180",  abs(angle_diff(180, 0)),  180.0)
    check("0→180 = ±180",  abs(angle_diff(0, 180)),  180.0)
    check("270→90 = ±180", abs(angle_diff(270, 90)), 180.0)
    check("1→359 = +2",    angle_diff(1, 359),    2.0)
    check("359→1 = -2",    angle_diff(359, 1),   -2.0)

    # ── Bearing encode / decode ──────────────────────────────────────────────
    print("\nBearing encode/decode (0–254 byte, ~1.4° resolution):")
    check("encode 0°   → 0",   bearing_encode(0),   0)
    check("encode 359° → 254", bearing_encode(359), 254)
    check("encode 360° → 0  (wraps)", bearing_encode(360), 0)
    check("encode 180° → 127", bearing_encode(180), 127)

    # Round-trip within tolerance
    for deg in [0, 45, 90, 135, 180, 225, 270, 315, 359]:
        rt = bearing_decode(bearing_encode(deg))
        check(f"round-trip {deg}° within 1.5°", rt, deg, tol=1.5)

    # ── Packet encoding ──────────────────────────────────────────────────────
    print("\nCMD_BEARING packet encoding:")

    for deg in [0, 90, 180, 270, 359]:
        v = bearing_encode(deg)
        pkt = encode_packet(CMD_BEARING, v)
        # Verify structure
        cmd    = pkt[1]
        v_high = pkt[2]
        v_low  = pkt[3]
        chk    = pkt[4]
        ok = (pkt[0] == SYNC and
              cmd == CMD_BEARING + 0x20 and
              0x40 <= v_high <= 0x4F and
              0x50 <= v_low  <= 0x5F and
              chk == (cmd ^ v_high ^ v_low))
        check(f"valid packet for {deg}°", ok, True)

    # Disable (value=255)
    pkt = encode_packet(CMD_BEARING, 255)
    cmd    = pkt[1]
    v_high = pkt[2]
    v_low  = pkt[3]
    chk    = pkt[4]
    ok = (pkt[0] == SYNC and
          chk == (cmd ^ v_high ^ v_low))
    check("disable packet (value=255) valid", ok, True)

    print(f"\nUnit tests: {passed} passed, {failed} failed")
    return failed == 0


# ── Hardware tests ─────────────────────────────────────────────────────────────

def run_hardware_tests(port):
    """
    Send CMD_BEARING commands and verify responses from the Yukon.

    Expected firmware behaviour (main.py):
      value 0–254 : ACK if IMU fitted, NAK if IMU absent
      value   255 : ACK always (disable bearing hold — no IMU required)

    Result codes used below:
      PASS    ACK received (command accepted)
      NAK     NAK received (valid rejection — IMU absent for bearing-set)
      FAIL    timeout      (no response — firmware likely not uploaded)
    """
    import serial
    import queue
    import threading

    print(f"\nHardware tests on {port}")

    try:
        ser = serial.Serial(port, 115200, timeout=0.1, dsrdtr=False)
    except serial.SerialException as e:
        print(f"  FAIL  Cannot open port: {e}")
        return False

    time.sleep(3.0)   # allow Yukon startup + fault recovery to complete
    ser.reset_input_buffer()
    ack_q = queue.Queue()

    def reader():
        while True:
            try:
                b = ser.read(1)
            except Exception:
                break
            if b and b[0] == ACK:
                ack_q.put(True)
            elif b and b[0] == NAK:
                ack_q.put(False)

    threading.Thread(target=reader, daemon=True).start()

    def send_and_wait(cmd_code, value):
        """Send packet; return True (ACK), False (NAK), or None (timeout)."""
        ser.write(encode_packet(cmd_code, value))
        try:
            return ack_q.get(timeout=1.0)
        except queue.Empty:
            return None

    passed = 0
    failed = 0

    print("\nCMD_BEARING hardware round-trip:")

    # ── Probe: disable always ACKs regardless of IMU ─────────────────────────
    # Test this first; a timeout means the firmware doesn't know CMD_BEARING.
    result = send_and_wait(CMD_BEARING, 255)
    if result is None:
        print("  FAIL  disable bearing hold  (timeout — is main.py uploaded to Yukon?)")
        ser.close()
        print(f"\nHardware tests: 0 passed, 1 failed")
        return False
    elif result is False:
        print("  FAIL  disable bearing hold  (unexpected NAK)")
        failed += 1
    else:
        print("  PASS  disable bearing hold")
        passed += 1

    # ── Bearing-set commands: ACK if IMU fitted, NAK if absent ───────────────
    bearing_tests = [
        (bearing_encode(0),   "set bearing   0°"),
        (bearing_encode(90),  "set bearing  90°"),
        (bearing_encode(180), "set bearing 180°"),
        (bearing_encode(270), "set bearing 270°"),
        (bearing_encode(359), "set bearing 359°"),
    ]
    imu_absent = False
    for value, label in bearing_tests:
        result = send_and_wait(CMD_BEARING, value)
        if result is True:
            print(f"  PASS  {label}")
            passed += 1
        elif result is False:
            print(f"  NAK   {label}  (IMU absent — bearing hold disabled in firmware)")
            imu_absent = True
            failed += 1
        else:
            print(f"  FAIL  {label}  (timeout)")
            failed += 1

    if imu_absent:
        print("\n  Note: bearing-set NAKs are expected when no BNO085 is fitted.")
        print("        Fit the IMU and re-run to verify end-to-end bearing hold.")

    # Leave bearing hold disabled
    send_and_wait(CMD_BEARING, 255)

    ser.close()
    print(f"\nHardware tests: {passed} passed, {failed} failed")
    return failed == 0


# ── Live heading display ───────────────────────────────────────────────────────

def live_display(port):
    """
    Live heading readout.

    The Yukon prints periodic SENS lines to stdout; it does not yet send IMU
    heading back over the binary protocol.  Instead, this mode prints the
    raw serial output from the Yukon so you can verify the IMU is running and
    watch the heading change as you rotate the board.

    Tip: add  print(f"IMU {imu.heading():.1f}")  to motor_core in main.py
         temporarily to stream headings here.
    """
    import serial

    print(f"\nLive output from Yukon on {port}  (Ctrl+C to quit)")
    print("Rotate the robot to verify heading changes.\n")

    try:
        ser = serial.Serial(port, 115200, timeout=0.5, dsrdtr=False)
    except serial.SerialException as e:
        print(f"Cannot open port: {e}")
        return

    try:
        while True:
            line = ser.readline()
            if line:
                try:
                    print(line.decode('ascii', errors='replace').rstrip())
                except Exception:
                    pass
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        ser.close()


# ── Entry point ────────────────────────────────────────────────────────────────

def find_port():
    import glob
    candidates = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
    return candidates[0] if candidates else None


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='BNO085 IMU tests')
    parser.add_argument('--port',     default=None,  help='Yukon serial port')
    parser.add_argument('--hardware', action='store_true', help='Run hardware protocol tests')
    parser.add_argument('--live',     action='store_true', help='Live heading display')
    args = parser.parse_args()

    ok = run_unit_tests()

    if args.hardware or args.live:
        port = args.port or find_port()
        if not port:
            print("\nNo serial port found. Specify with --port /dev/ttyACMx")
            sys.exit(1)
        if args.hardware:
            ok &= run_hardware_tests(port)
        if args.live:
            live_display(port)

    sys.exit(0 if ok else 1)
