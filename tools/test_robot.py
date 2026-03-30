#!/usr/bin/env python3
"""
test_robot.py — Integration tests for robot_daemon.py using the Yukon simulator.

Uses a PTY (pseudo-terminal) to connect Robot to yukon_sim without real
hardware.  Camera, LiDAR and GPS are all disabled so only the Yukon serial
link and its subsystem threads are exercised.

Tests:
  1. Lifecycle      — start() connects to PTY; stop() shuts down cleanly
  2. get_state()    — returns RobotState with correct types and defaults
  3. Drive commands — drive() sends correct wire bytes to the simulator
  4. Kill           — kill() zeros both motor bytes in the simulator
  5. LED            — set_led_a / set_led_b update simulator state
  6. Telemetry      — 1 Hz thread populates voltage, current, temps correctly
  7. IMU heading    — heading decoded from RESP_HEADING matches sim value
  8. Estop          — estop() does not raise; mode becomes MANUAL

Usage:
    python3 tools/test_robot.py
"""

import os
import sys
import pty
import tty
import time
import threading

_REPO  = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_TOOLS = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _REPO)
sys.path.insert(0, _TOOLS)

from robot_daemon import Robot, RobotMode
from yukon_sim import (yukon_server, _state, _lock,
                        SIM_VOLTAGE, SIM_CURRENT, SIM_TEMP,
                        RC_MANUAL, RC_AUTO)


# ── Test harness ──────────────────────────────────────────────────────────────

_passed = 0
_failed = 0


def _check(name, condition, detail=""):
    global _passed, _failed
    if condition:
        print(f"  PASS  {name}")
        _passed += 1
    else:
        info = f"  ({detail})" if detail else ""
        print(f"  FAIL  {name}{info}")
        _failed += 1


def _approx(a, b, tol=1.0):
    return abs(a - b) <= tol


# ── PTY / simulator helpers ───────────────────────────────────────────────────

_SIM_HEADING = 45.0   # degrees — what the sim reports as current IMU heading


def _start_sim():
    """Open a PTY, reset sim state, start yukon_server thread.

    Returns (master_fd, slave_fd, slave_path).
    """
    master_fd, slave_fd = pty.openpty()
    tty.setraw(slave_fd)
    slave_path = os.ttyname(slave_fd)

    with _lock:
        _state['left_byte']      = None
        _state['right_byte']     = None
        _state['led_a']          = False
        _state['led_b']          = False
        _state['cmds_rx']        = 0
        _state['running']        = True
        _state['rc_mode']        = RC_MANUAL
        _state['rc_channels']    = [1500] * 14
        _state['rc_valid']       = True
        _state['imu_present']    = True
        _state['imu_heading']    = _SIM_HEADING
        _state['bearing_target'] = None
        _state['last_imu_tick']  = time.monotonic()

    t = threading.Thread(target=yukon_server, args=(master_fd,), daemon=True)
    t.start()
    return master_fd, slave_fd, slave_path


def _stop_sim(master_fd, slave_fd):
    with _lock:
        _state['running'] = False
    for fd in (master_fd, slave_fd):
        try:
            os.close(fd)
        except OSError:
            pass


def _make_robot(yukon_port):
    return Robot(
        yukon_port    = yukon_port,
        ibus_port     = '/tmp/no_ibus_for_tests',  # rc_thread logs warning + retries
        enable_camera = False,
        enable_lidar  = False,
        enable_gps    = False,
        no_motors     = False,
    )


# ── Individual test sections ──────────────────────────────────────────────────

def test_lifecycle(robot):
    print("\nLifecycle:")
    _check("robot object created",   robot is not None)
    _check("initial mode is MANUAL", robot.get_state().mode is RobotMode.MANUAL)


def test_get_state(robot):
    print("\nget_state():")
    state = robot.get_state()
    _check("returns RobotState",        state is not None)
    _check("drive.left is float",       isinstance(state.drive.left, float))
    _check("drive.right is float",      isinstance(state.drive.right, float))
    _check("initial left speed = 0",    state.drive.left  == 0.0)
    _check("initial right speed = 0",   state.drive.right == 0.0)
    _check("nav_state is str",          isinstance(state.nav_state, str))
    _check("nav_gate is int",           isinstance(state.nav_gate, int))
    _check("speed_scale > 0",           state.speed_scale > 0.0)
    _check("camera_ok = False",         state.camera_ok is False)
    _check("lidar_ok = False",          state.lidar_ok  is False)
    _check("gps_ok = False",            state.gps_ok    is False)


def test_drive(robot):
    print("\nDrive commands:")

    # Drive commands are only applied by the sim in AUTO mode (matches firmware behaviour).
    # Put the robot in AUTO and use robot.drive() so the control thread sends the values.
    # Note: control thread applies speed_scale, so we test sign/symmetry not exact bytes.
    robot.set_mode(RobotMode.AUTO)
    robot.drive(0.5, -0.5)
    time.sleep(0.15)   # wait for control thread (50 Hz) to deliver to sim

    with _lock:
        lb = _state['left_byte']
        rb = _state['right_byte']

    # left = forward (0–99), right = reverse (101–200), equal magnitudes
    _check("drive(0.5, -0.5): left byte forward (0<lb<100)",   0 < lb < 100,    f"got {lb}")
    _check("drive(0.5, -0.5): right byte reverse (100<rb≤200)", 100 < rb <= 200, f"got {rb}")
    _check("drive(0.5, -0.5): symmetric magnitudes (lb == rb-100)", lb == rb - 100, f"lb={lb} rb={rb}")

    # drive(0, 0): trigger another AUTO pulse to send zero bytes
    robot.set_mode(RobotMode.AUTO)
    robot.drive(0.0, 0.0)
    time.sleep(0.15)
    with _lock:
        lb = _state['left_byte']
        rb = _state['right_byte']
    _check("drive(0, 0): left byte = 0",   lb == 0, f"got {lb}")
    _check("drive(0, 0): right byte = 0",  rb == 0, f"got {rb}")

    robot.set_mode(RobotMode.MANUAL)   # restore
    time.sleep(0.05)


def test_kill(robot):
    print("\nKill:")
    robot._yukon.drive(0.75, 0.75)
    time.sleep(0.1)
    robot._yukon.kill()
    time.sleep(0.1)

    with _lock:
        lb = _state['left_byte']
        rb = _state['right_byte']
    _check("kill: left byte = 0",   lb == 0, f"got {lb}")
    _check("kill: right byte = 0",  rb == 0, f"got {rb}")


def test_led(robot):
    print("\nLED:")
    robot._yukon.set_led_a(True)
    time.sleep(0.1)
    with _lock:
        led_a = _state['led_a']
    _check("set_led_a(True): led_a = True",  led_a is True)

    robot._yukon.set_led_b(True)
    time.sleep(0.1)
    with _lock:
        led_b = _state['led_b']
    _check("set_led_b(True): led_b = True",  led_b is True)

    robot._yukon.set_led_a(False)
    time.sleep(0.1)
    with _lock:
        led_a = _state['led_a']
    _check("set_led_a(False): led_a = False", led_a is False)


def test_telemetry(robot):
    print("\nTelemetry (waiting up to 2 s for 1 Hz thread):")
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        t = robot.get_state().telemetry
        if t.voltage > 0:
            break
        time.sleep(0.1)

    t = robot.get_state().telemetry
    _check(f"voltage ≈ {SIM_VOLTAGE} V",
           _approx(t.voltage, SIM_VOLTAGE, tol=0.5),
           f"got {t.voltage}")
    _check(f"current ≈ {SIM_CURRENT} A",
           _approx(t.current, SIM_CURRENT, tol=0.1),
           f"got {t.current}")
    _check(f"board_temp ≈ {SIM_TEMP} °C",
           _approx(t.board_temp, SIM_TEMP, tol=2.0),
           f"got {t.board_temp}")
    _check("left_fault = False",  t.left_fault  is False)
    _check("right_fault = False", t.right_fault is False)
    _check("telemetry timestamp set", t.timestamp > 0)


_SIM_PITCH =   0.0   # degrees — sim default pitch
_SIM_ROLL  =   0.0   # degrees — sim default roll


def test_imu_heading(robot):
    print("\nIMU heading / pitch / roll:")
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        hdg = robot.get_heading()
        if hdg is not None:
            break
        time.sleep(0.1)

    hdg = robot.get_heading()
    # Heading is encoded/decoded with ~1.4° resolution; allow 3° tolerance
    _check(f"heading ≈ {_SIM_HEADING}° (±3°)",
           hdg is not None and _approx(hdg, _SIM_HEADING, tol=3.0),
           f"got {hdg}")

    state = robot.get_state()
    t = state.telemetry
    # Pitch: ~0.7° resolution; sim default 0.0°; allow 2° tolerance
    _check(f"pitch ≈ {_SIM_PITCH}° (±2°)",
           t.pitch is not None and _approx(t.pitch, _SIM_PITCH, tol=2.0),
           f"got {t.pitch}")
    # Roll: ~1.4° resolution; sim default 0.0°; allow 2° tolerance
    _check(f"roll ≈ {_SIM_ROLL}° (±2°)",
           t.roll is not None and _approx(t.roll, _SIM_ROLL, tol=2.0),
           f"got {t.roll}")

    _check("nav_bearing_err is None (navigator not running)",
           state.nav_bearing_err is None)


def test_estop(robot):
    print("\nEstop:")
    try:
        robot.estop()
        ok = True
    except Exception as e:
        ok = False
    _check("estop() does not raise", ok)
    _check("mode after estop is ESTOP",
           robot.get_state().mode is RobotMode.ESTOP)


def test_reset_estop(robot):
    print("\nReset ESTOP:")
    robot.estop()
    _check("mode is ESTOP before reset",
           robot.get_state().mode is RobotMode.ESTOP)
    robot.reset_estop()
    _check("mode returns to MANUAL after reset_estop()",
           robot.get_state().mode is RobotMode.MANUAL)


def test_auto_mode(robot):
    print("\nAUTO mode:")
    robot.reset_estop()   # ensure clean MANUAL state

    robot.set_mode(RobotMode.AUTO)
    _check("set_mode(AUTO) → mode is AUTO",
           robot.get_state().mode is RobotMode.AUTO)

    robot.set_mode(RobotMode.MANUAL)
    _check("set_mode(MANUAL) → mode is MANUAL",
           robot.get_state().mode is RobotMode.MANUAL)

    raised = False
    try:
        robot.set_mode(RobotMode.ESTOP)
    except ValueError:
        raised = True
    _check("set_mode(ESTOP) raises ValueError", raised)

    # drive() stores values regardless of mode (applied by control thread in AUTO)
    robot.set_mode(RobotMode.AUTO)
    robot.drive(0.6, -0.4)
    with robot._mode_lock:
        al = robot._auto_left
        ar = robot._auto_right
    _check("drive(0.6, -0.4): _auto_left stored correctly",
           _approx(al, 0.6), f"got {al}")
    _check("drive(0.6, -0.4): _auto_right stored correctly",
           _approx(ar, -0.4), f"got {ar}")

    # Clamp check
    robot.drive(2.0, -2.0)
    with robot._mode_lock:
        al = robot._auto_left
        ar = robot._auto_right
    _check("drive(2.0, …) clamped to 1.0",  _approx(al,  1.0), f"got {al}")
    _check("drive(…, -2.0) clamped to -1.0", _approx(ar, -1.0), f"got {ar}")

    robot.set_mode(RobotMode.MANUAL)


def test_data_log(robot):
    import tempfile, os
    print("\nData logging:")

    _check("not logging before start", not robot.is_data_logging())

    with tempfile.NamedTemporaryFile(suffix='.jsonl', delete=False) as f:
        tmp_path = f.name

    try:
        started = robot.start_data_log(path=tmp_path, hz=5.0)
        _check("start_data_log() returns True", started is True)
        _check("is_data_logging() True while active", robot.is_data_logging())

        # Second call while active should return False
        started2 = robot.start_data_log(path=tmp_path, hz=5.0)
        _check("start_data_log() returns False when already active",
               started2 is False)

        time.sleep(0.5)   # let at least 2 records be written

        returned_path = robot.stop_data_log()
        _check("stop_data_log() returns the log path",
               returned_path == tmp_path)
        _check("is_data_logging() False after stop", not robot.is_data_logging())

        size = os.path.getsize(tmp_path)
        _check("log file has content (> 0 bytes)", size > 0, f"size={size}")

        with open(tmp_path) as fh:
            import json
            lines = [l.strip() for l in fh if l.strip()]
        _check("log file has at least 1 record", len(lines) >= 1,
               f"got {len(lines)}")
        if lines:
            rec = json.loads(lines[0])
            _check("record has 'ts' field",   'ts'   in rec)
            _check("record has 'mode' field", 'mode' in rec)
            _check("record has 'drive' field",'drive' in rec)
    finally:
        try:
            os.unlink(tmp_path)
        except OSError:
            pass


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    print("=" * 50)
    print("robot.py integration tests (Yukon simulator)")
    print("=" * 50)

    master_fd, slave_fd, slave_path = _start_sim()
    print(f"\nSim PTY: {slave_path}")

    robot = _make_robot(slave_path)
    try:
        robot.start()
        time.sleep(0.3)   # let subsystem threads settle

        test_lifecycle(robot)
        test_get_state(robot)
        test_drive(robot)
        test_kill(robot)
        test_led(robot)
        test_telemetry(robot)
        test_imu_heading(robot)
        test_estop(robot)
        test_reset_estop(robot)
        test_auto_mode(robot)
        test_data_log(robot)

    finally:
        robot.stop()
        _stop_sim(master_fd, slave_fd)

    print(f"\n{'=' * 50}")
    print(f"Integration tests: {_passed} passed, {_failed} failed")
    print(f"{'=' * 50}")
    return 0 if _failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
