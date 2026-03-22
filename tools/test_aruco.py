#!/usr/bin/env python3
"""
test_aruco.py — Unit tests and live camera test for ArUco detection/navigation.

Tests:
  1. ArucoDetector helpers and init        (no camera)
  2. ArucoDetector detection on synthetic frames  (no camera)
  3. ArucoNavigator helper functions       (no hardware)
  4. ArucoNavigator state-machine          (no hardware)
  5. Live camera detection                 (--live, requires camera)

Usage:
    python3 tools/test_aruco.py              # unit tests only
    python3 tools/test_aruco.py --live       # unit tests + live camera view
    python3 tools/test_aruco.py --live --camera 0
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import math
import time
import argparse
import tempfile
import configparser

import cv2
import numpy as np

from robot.aruco_detector import ArucoDetector, ArUcoTag, ArUcoGate, ArUcoState, ARUCO_DICT
from robot.aruco_navigator import (
    ArucoNavigator, NavConfig, NavState,
    _clamp, _ramp, _angle_diff, _pixel_bearing,
)


# ── Synthetic frame helpers ────────────────────────────────────────────────────

_DICT_NAME = "DICT_4X4_1000"
_ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)


def _blank_frame(width=640, height=480):
    """Plain grey RGB frame with no markers."""
    return np.full((height, width, 3), 180, dtype=np.uint8)


def _marker_img(marker_id: int, size: int = 80) -> np.ndarray:
    """Return a (size, size, 3) RGB image of one ArUco marker."""
    gray = cv2.aruco.generateImageMarker(_ARUCO_DICT, marker_id, size)
    return cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB)


def _frame_with_markers(positions: dict, width=640, height=480, marker_size=80):
    """
    Build an RGB frame with ArUco markers stamped at given positions.

    positions : {marker_id: (x, y)}  — top-left corner of the marker square
    """
    frame = _blank_frame(width, height)
    for mid, (x, y) in positions.items():
        img = _marker_img(mid, marker_size)
        frame[y:y + marker_size, x:x + marker_size] = img
    return frame


# ── Section 1: ArucoDetector — init ───────────────────────────────────────────

def test_detector_init():
    passed = 0
    failed = 0

    def check(name, ok, detail=""):
        nonlocal passed, failed
        print(f"  {'PASS' if ok else 'FAIL'}  {name}")
        if ok:
            passed += 1
        else:
            failed += 1
            if detail:
                print(f"          {detail}")

    print("\nArucoDetector — initialisation:")

    # Valid dict names don't raise
    for name in ["DICT_4X4_50", "DICT_4X4_1000", "DICT_5X5_100", "DICT_ARUCO_ORIGINAL"]:
        try:
            ArucoDetector(dict_name=name)
            check(f"init {name}", True)
        except Exception as e:
            check(f"init {name}", False, str(e))

    # Invalid dict name raises ValueError
    try:
        ArucoDetector(dict_name="DICT_FAKE")
        check("invalid dict raises ValueError", False, "no exception raised")
    except ValueError:
        check("invalid dict raises ValueError", True)
    except Exception as e:
        check("invalid dict raises ValueError", False, f"wrong exception: {e}")

    # draw=False, show_fps=False — should construct without error
    try:
        ArucoDetector(draw=False, show_fps=False)
        check("draw=False show_fps=False", True)
    except Exception as e:
        check("draw=False show_fps=False", False, str(e))

    # calib_file that does not exist raises ValueError
    try:
        ArucoDetector(calib_file="/tmp/does_not_exist.npz")
        check("missing calib_file raises ValueError", False, "no exception raised")
    except ValueError:
        check("missing calib_file raises ValueError", True)
    except Exception as e:
        check("missing calib_file raises ValueError", False, f"wrong exception: {e}")

    return passed, failed


# ── Section 2: ArucoDetector — detection on synthetic frames ──────────────────

def test_detector_detection():
    passed = 0
    failed = 0

    def check(name, result, expected, tol=None):
        nonlocal passed, failed
        if tol is not None:
            ok = abs(result - expected) <= tol
        else:
            ok = result == expected
        print(f"  {'PASS' if ok else 'FAIL'}  {name}")
        if ok:
            passed += 1
        else:
            failed += 1
            print(f"          got {result!r}, expected {expected!r}")

    print("\nArucoDetector — synthetic frame detection:")

    det = ArucoDetector(dict_name=_DICT_NAME, draw=False, show_fps=False)

    # Blank frame → no tags, no gates
    state = det.detect(_blank_frame())
    check("blank frame: no tags",  len(state.tags),  0)
    check("blank frame: no gates", len(state.gates), 0)

    # Single marker (id=1) placed centre-left
    frame = _frame_with_markers({1: (100, 200)})
    state = det.detect(frame)
    check("single marker: 1 tag found", len(state.tags), 1)
    check("single marker: tag id=1 present", 1 in state.tags, True)
    check("single marker: no gates", len(state.gates), 0)

    # Two unrelated markers (ids 1 and 3 — not consecutive even pair)
    frame = _frame_with_markers({1: (100, 200), 3: (400, 200)})
    state = det.detect(frame)
    check("ids 1+3: 2 tags found", len(state.tags), 2)
    check("ids 1+3: no gate (not consecutive pair)", len(state.gates), 0)

    # Gate pair: markers 1 (left) and 2 (right) → gate 0, correct_dir=True
    frame = _frame_with_markers({1: (100, 200), 2: (420, 200)})
    state = det.detect(frame)
    check("gate 0: 2 tags found", len(state.tags), 2)
    check("gate 0: gate 0 present", 0 in state.gates, True)
    if 0 in state.gates:
        g = state.gates[0]
        check("gate 0: odd_tag=1",  g.odd_tag,  1)
        check("gate 0: even_tag=2", g.even_tag, 2)
        check("gate 0: correct_dir=True (1 left of 2)", g.correct_dir, True)
        # Gate centre should be between the two markers
        t1 = state.tags[1]
        t2 = state.tags[2]
        expected_cx = (t1.center_x + t2.center_x) // 2
        check("gate 0: centre_x midpoint", g.centre_x, expected_cx)

    # Gate pair with reversed direction: marker 2 left, marker 1 right
    frame = _frame_with_markers({2: (100, 200), 1: (420, 200)})
    state = det.detect(frame)
    if 0 in state.gates:
        check("gate 0 reversed: correct_dir=False", state.gates[0].correct_dir, False)
    else:
        check("gate 0 reversed: gate present", 0 in state.gates, True)

    # Multiple gates: 1+2 and 3+4
    frame = _frame_with_markers({1: (50, 100), 2: (170, 100), 3: (350, 100), 4: (470, 100)})
    state = det.detect(frame)
    check("2 gates: 4 tags found", len(state.tags), 4)
    check("2 gates: gate 0 present", 0 in state.gates, True)
    check("2 gates: gate 1 present", 1 in state.gates, True)

    # FPS is populated on second call
    det2 = ArucoDetector(draw=False, show_fps=False)
    det2.detect(_blank_frame())
    time.sleep(0.05)
    state2 = det2.detect(_blank_frame())
    check("fps > 0 after two calls", state2.fps > 0, True)

    # draw=True does not crash on blank frame
    det3 = ArucoDetector(draw=True, show_fps=True)
    try:
        det3.detect(_blank_frame())
        check("draw=True on blank frame: no crash", True, True)
    except Exception as e:
        check("draw=True on blank frame: no crash", False, True)
        print(f"          exception: {e}")

    # draw=True annotates a gate frame without crashing
    det4 = ArucoDetector(draw=True, show_fps=True)
    frame = _frame_with_markers({1: (100, 200), 2: (420, 200)})
    try:
        det4.detect(frame)
        check("draw=True on gate frame: no crash", True, True)
    except Exception as e:
        check("draw=True on gate frame: no crash", False, True)
        print(f"          exception: {e}")

    return passed, failed


# ── Section 3: ArucoNavigator — pure helper functions ─────────────────────────

def test_navigator_helpers():
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

    print("\nArucoNavigator — helper functions:")

    # _clamp
    print("  _clamp:")
    check("  clamp(0.5)  → 0.5",  _clamp(0.5),   0.5)
    check("  clamp(1.0)  → 1.0",  _clamp(1.0),   1.0)
    check("  clamp(1.5)  → 1.0",  _clamp(1.5),   1.0)
    check("  clamp(-1.5) → -1.0", _clamp(-1.5), -1.0)
    check("  clamp(0.0)  → 0.0",  _clamp(0.0),   0.0)

    # _ramp
    print("  _ramp:")
    check("  ramp(0→1, max=0.1) → 0.1",  _ramp(0.0, 1.0, 0.1), 0.1)
    check("  ramp(0→0.05, max=0.1) → 0.05", _ramp(0.0, 0.05, 0.1), 0.05)
    check("  ramp(0→-1, max=0.1) → -0.1", _ramp(0.0, -1.0, 0.1), -0.1)
    check("  ramp already at target",     _ramp(0.5, 0.5, 0.1), 0.5)

    # _angle_diff
    print("  _angle_diff:")
    check("  0→0 = 0",       _angle_diff(0, 0),     0.0)
    check("  90→0 = +90",    _angle_diff(90, 0),   90.0)
    check("  0→90 = -90",    _angle_diff(0, 90),  -90.0)
    check("  350→10 = -20",  _angle_diff(350, 10), -20.0)
    check("  10→350 = +20",  _angle_diff(10, 350),  20.0)
    check("  1→359 = +2",    _angle_diff(1, 359),    2.0)
    check("  359→1 = -2",    _angle_diff(359, 1),   -2.0)
    check("  |180→0| = 180", abs(_angle_diff(180, 0)), 180.0)

    # _pixel_bearing
    print("  _pixel_bearing:")
    check("  pixel_bearing centre → 0°",   _pixel_bearing(320, 320), 0.0)
    check("  pixel_bearing left edge → -31°", _pixel_bearing(0, 320), -31.0, tol=0.1)
    check("  pixel_bearing right edge → +31°", _pixel_bearing(640, 320), 31.0, tol=0.1)

    return passed, failed


# ── Section 4: ArucoNavigator — state machine ─────────────────────────────────

def test_navigator_state_machine():
    passed = 0
    failed = 0

    def check(name, result, expected):
        nonlocal passed, failed
        ok = result == expected
        print(f"  {'PASS' if ok else 'FAIL'}  {name}")
        if ok:
            passed += 1
        else:
            failed += 1
            print(f"          got {result!r}, expected {expected!r}")

    def check_approx(name, result, expected, tol=1e-9):
        nonlocal passed, failed
        ok = abs(result - expected) <= tol
        print(f"  {'PASS' if ok else 'FAIL'}  {name}")
        if ok:
            passed += 1
        else:
            failed += 1
            print(f"          got {result!r}, expected {expected!r}")

    print("\nArucoNavigator — state machine:")

    empty_state = ArUcoState()

    # Initial state
    nav = ArucoNavigator()
    check("initial state IDLE", nav.state, NavState.IDLE)
    check("initial gate_id=0",  nav.gate_id, 0)

    # IDLE returns (0, 0)
    l, r = nav.update(empty_state, 640)
    check_approx("IDLE update returns 0,0 (left)",  l, 0.0)
    check_approx("IDLE update returns 0,0 (right)", r, 0.0)

    # start() → SEARCHING
    nav.start()
    check("after start(): SEARCHING", nav.state, NavState.SEARCHING)
    check("after start(): gate_id=0",  nav.gate_id, 0)

    # SEARCHING with no tags → rotate (non-zero outputs)
    time.sleep(0.05)   # allow dt > 0 so ramp produces non-zero
    l, r = nav.update(empty_state, 640)
    check("searching no tags: outputs differ (rotation)", l != r, True)

    # stop() → IDLE
    nav.stop()
    check("after stop(): IDLE", nav.state, NavState.IDLE)

    # COMPLETE returns (0, 0)
    nav2 = ArucoNavigator(NavConfig(max_gates=1))
    nav2._state = NavState.COMPLETE
    l, r = nav2.update(empty_state, 640)
    check_approx("COMPLETE returns 0,0 (left)",  l, 0.0)
    check_approx("COMPLETE returns 0,0 (right)", r, 0.0)

    # ERROR returns (0, 0)
    nav3 = ArucoNavigator()
    nav3._state = NavState.ERROR
    l, r = nav3.update(empty_state, 640)
    check_approx("ERROR returns 0,0 (left)",  l, 0.0)
    check_approx("ERROR returns 0,0 (right)", r, 0.0)

    # SEARCHING with IMU heading: step-based search
    nav4 = ArucoNavigator(NavConfig(search_step_deg=45.0, search_step_pause=0.0))
    nav4.start()
    # Simulate one step's worth of rotation
    nav4._search_origin = 10.0
    time.sleep(0.02)
    # Before step complete, should still rotate
    l, r = nav4.update(empty_state, 640, heading=20.0)
    check("IMU search (10° turned): rotation output", l != r, True)

    # from_ini() — write a temp ini and parse it
    cfg_data = """
[navigator]
max_gates = 5
fwd_speed = 0.5
steer_kp  = 0.8
"""
    with tempfile.NamedTemporaryFile(mode='w', suffix='.ini', delete=False) as f:
        f.write(cfg_data)
        tmp_path = f.name

    try:
        nav5 = ArucoNavigator.from_ini(tmp_path)
        check("from_ini: max_gates=5",   nav5.cfg.max_gates,  5)
        check_approx("from_ini: fwd_speed=0.5", nav5.cfg.fwd_speed, 0.5)
        check_approx("from_ini: steer_kp=0.8",  nav5.cfg.steer_kp,  0.8)
    finally:
        os.unlink(tmp_path)

    # from_ini() with missing section → uses defaults
    with tempfile.NamedTemporaryFile(mode='w', suffix='.ini', delete=False) as f:
        f.write("[other]\nfoo = bar\n")
        tmp_path2 = f.name
    try:
        nav6 = ArucoNavigator.from_ini(tmp_path2)
        check("from_ini missing section: max_gates default",
              nav6.cfg.max_gates, NavConfig.max_gates)
    finally:
        os.unlink(tmp_path2)

    # gate advancement via _advance_gate()
    nav7 = ArucoNavigator(NavConfig(max_gates=3))
    nav7.start()
    nav7._state = NavState.PASSING
    nav7._pass_start = time.monotonic() - 999   # force timeout
    nav7._advance_gate()
    check("advance gate 0→1", nav7.gate_id, 1)
    check("after advance: SEARCHING", nav7.state, NavState.SEARCHING)

    nav7._advance_gate()
    check("advance gate 1→2", nav7.gate_id, 2)
    nav7._advance_gate()
    check("advance gate 2 → COMPLETE (max_gates=3)", nav7.state, NavState.COMPLETE)

    # _resolve_target with visible gate: checks for AttributeError bug
    # ArUcoGate does not have .bearing or .distance fields; this will raise
    # AttributeError unless the dataclass has been extended.
    print("\n  _resolve_target with gate visible (bug check):")
    gate = ArUcoGate(gate_id=0, odd_tag=1, even_tag=2,
                     centre_x=320, centre_y=240, correct_dir=True)
    state_with_gate = ArUcoState(tags={}, gates={0: gate})
    nav8 = ArucoNavigator()
    nav8.start()
    try:
        tx, dist, bearing = nav8._resolve_target(state_with_gate, 320)
        check("  _resolve_target gate: no AttributeError", True, True)
        check("  _resolve_target gate: returns centre_x", tx, 320)
    except AttributeError as e:
        # ArUcoGate is missing .bearing / .distance fields
        print(f"  FAIL  _resolve_target gate: AttributeError — {e}")
        print("        ArUcoGate in aruco_detector.py is missing .bearing and .distance fields")
        failed += 1

    # _resolve_target with single tag visible: same check for ArUcoTag
    tag = ArUcoTag(id=1, center_x=200, center_y=240, area=6400,
                   top_left=(160, 200), top_right=(240, 200),
                   bottom_right=(240, 280), bottom_left=(160, 280))
    state_with_tag = ArUcoState(tags={1: tag}, gates={})
    nav9 = ArucoNavigator()
    nav9.start()
    try:
        tx, dist, bearing = nav9._resolve_target(state_with_tag, 320)
        check("  _resolve_target single tag: no AttributeError", True, True)
        # Odd tag (id=1) → aim RIGHT of tag, so tx > tag.center_x
        check("  _resolve_target odd tag: aim right of tag", tx > tag.center_x, True)
    except AttributeError as e:
        print(f"  FAIL  _resolve_target single tag: AttributeError — {e}")
        print("        ArUcoTag in aruco_detector.py is missing .bearing and .distance fields")
        failed += 1

    return passed, failed


# ── Section 5: Live camera ─────────────────────────────────────────────────────

def live_camera(camera_index: int = 0):
    """
    Open a camera, run ArucoDetector on each frame, and display the annotated
    result in a window.  Press Q or Escape to quit.
    """
    print(f"\nLive camera (index {camera_index}) — press Q or Esc to quit\n")

    det = ArucoDetector(dict_name=_DICT_NAME, draw=True, show_fps=True)

    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        print(f"  Cannot open camera {camera_index}")
        return

    try:
        while True:
            ret, bgr = cap.read()
            if not ret:
                print("  Frame capture failed")
                break

            # robot.py frames are RGB; VideoCapture gives BGR — convert
            frame = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            state = det.detect(frame)

            # Print detections to terminal
            if state.tags:
                tag_ids = sorted(state.tags)
                gates   = [(g.gate_id, g.correct_dir) for g in state.gates.values()]
                print(f"\r  tags={tag_ids}  gates={gates}  fps={state.fps:.1f}    ",
                      end='', flush=True)
            else:
                print(f"\r  no tags  fps={state.fps:.1f}    ", end='', flush=True)

            # Display (convert back to BGR for imshow)
            display = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            cv2.imshow("ArUco Live", display)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), ord('Q'), 27):  # Q or Escape
                break
    except KeyboardInterrupt:
        pass
    finally:
        print()
        cap.release()
        cv2.destroyAllWindows()


# ── Entry point ────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='ArUco detector/navigator tests')
    parser.add_argument('--live',   action='store_true', help='Run live camera test')
    parser.add_argument('--camera', type=int, default=0, help='Camera index (default 0)')
    args = parser.parse_args()

    total_passed = 0
    total_failed = 0

    for fn in (
        test_detector_init,
        test_detector_detection,
        test_navigator_helpers,
        test_navigator_state_machine,
    ):
        p, f = fn()
        total_passed += p
        total_failed += f

    print(f"\n{'='*50}")
    print(f"Unit tests: {total_passed} passed, {total_failed} failed")

    if args.live:
        live_camera(args.camera)

    sys.exit(0 if total_failed == 0 else 1)
