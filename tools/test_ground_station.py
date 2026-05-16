#!/usr/bin/env python3
"""
test_ground_station.py — Unit tests for ground_station_v2.py.

Tests the _GsState machine (apply_cmd, handle_* packet handlers, get() snapshot),
_body_to_cmd_frame serialisation, the record_start/stop idempotency guard in
api_cmd, and the full encode→decode telemetry pipeline.

No hardware, no radio, no Flask server required.

Usage:
    python3 tools/test_ground_station.py
"""

import json
import os
import sys
import queue

_REPO  = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_TOOLS = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _REPO)
sys.path.insert(0, _TOOLS)

# Import the module under test — suppress Flask startup noise
import logging
logging.disable(logging.CRITICAL)

import ground_station_v2 as gs
from ground_station_v2 import (
    _GsState, _body_to_cmd_frame, app, _gs, _cmd_q,
)
from robot.telemetry_proto import (
    SF_CAM_RECORDING, SF_DATA_LOGGING, SF_NO_MOTORS, SF_RC_ACTIVE,
    SF_CAM_OK, SF_LIDAR_OK, SF_GPS_OK, SF_BENCH_ENABLED,
    CMD_ESTOP, CMD_RESET_ESTOP, CMD_SET_MODE, CMD_RECORD_TOGGLE,
    CMD_DATA_LOG_TOGGLE, CMD_NO_MOTORS_TOGGLE, CMD_GPS_BOOKMARK,
    CMD_NAV_RESET, CMD_NAV_PAUSE_TOGGLE, CMD_BENCH_TOGGLE,
    MODE_MANUAL, MODE_AUTO,
    encode_state, encode_telem, encode_gps, encode_nav, encode_ina,
    decode_state, decode_telem, decode_gps, decode_ina,
    TYPE_STATE, TYPE_TELEM, TYPE_GPS, TYPE_INA,
    FrameDecoder,
)

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


def _fresh() -> _GsState:
    """Return a new _GsState with default values."""
    return _GsState()


def _cmd_frame_id(body: dict) -> int | None:
    """Return the CMD byte from a _body_to_cmd_frame result, or None."""
    frame = _body_to_cmd_frame(body)
    if frame is None:
        return None
    dec = FrameDecoder()
    for ptype, payload in dec.feed(frame):
        return payload[0] if payload else None
    return None


# ── _GsState: defaults ────────────────────────────────────────────────────────

def test_defaults():
    print("\nDefault state:")
    s = _fresh()
    snap = s.get()
    _check("mode is MANUAL",          snap["mode"] == "MANUAL")
    _check("cam_recording is False",  snap["cam_recording"] is False)
    _check("data_logging is False",   snap["data_logging"]  is False)
    _check("no_motors is False",      snap["no_motors"]     is False)
    _check("rc_active is False",      snap["rc_active"]     is False)
    _check("is_recording() False",    not s.is_recording())
    _check("packets starts at 0",     s.packets == 0)
    ina = snap["ina"]
    _check("ina.ok False by default",      ina["ok"]       is False)
    _check("ina.voltage None by default",  ina["voltage"]  is None)
    _check("ina.current None by default",  ina["current"]  is None)
    _check("ina.power None by default",    ina["power"]    is None)
    _check("ina.die_temp None by default", ina["die_temp"] is None)


# ── apply_cmd: mode transitions ───────────────────────────────────────────────

def test_apply_cmd_mode():
    print("\napply_cmd — mode transitions:")
    s = _fresh()

    s.apply_cmd({'cmd': 'estop'})
    _check("estop → ESTOP",           s.get()["mode"] == "ESTOP")

    # reset in ESTOP → MANUAL
    s.apply_cmd({'cmd': 'reset'})
    _check("reset in ESTOP → MANUAL", s.get()["mode"] == "MANUAL")

    # reset in MANUAL → no-op
    s.apply_cmd({'cmd': 'reset'})
    _check("reset in MANUAL → MANUAL",s.get()["mode"] == "MANUAL")

    # set_mode AUTO when not ESTOP
    s.apply_cmd({'cmd': 'set_mode', 'mode': 'AUTO'})
    _check("set_mode AUTO",           s.get()["mode"] == "AUTO")

    # set_mode ignored when ESTOP
    s.apply_cmd({'cmd': 'estop'})
    s.apply_cmd({'cmd': 'set_mode', 'mode': 'AUTO'})
    _check("set_mode ignored in ESTOP", s.get()["mode"] == "ESTOP")


# ── apply_cmd: flag toggles ───────────────────────────────────────────────────

def test_apply_cmd_flags():
    print("\napply_cmd — flag toggles:")
    s = _fresh()

    s.apply_cmd({'cmd': 'no_motors_toggle'})
    _check("no_motors_toggle → True",    s.get()["no_motors"] is True)
    s.apply_cmd({'cmd': 'no_motors_toggle'})
    _check("no_motors_toggle → False",   s.get()["no_motors"] is False)

    s.apply_cmd({'cmd': 'data_log_toggle'})
    _check("data_log_toggle → True",     s.get()["data_logging"] is True)
    s.apply_cmd({'cmd': 'data_log_toggle'})
    _check("data_log_toggle → False",    s.get()["data_logging"] is False)

    s.apply_cmd({'cmd': 'nav_pause_toggle'})
    _check("nav_pause_toggle → True",    s.get()["nav_paused"] is True)
    s.apply_cmd({'cmd': 'nav_pause_toggle'})
    _check("nav_pause_toggle → False",   s.get()["nav_paused"] is False)


# ── apply_cmd: record start / stop / toggle ───────────────────────────────────

def test_apply_cmd_recording():
    print("\napply_cmd — recording commands:")
    s = _fresh()

    s.apply_cmd({'cmd': 'record_start'})
    _check("record_start sets cam_recording",     s.get()["cam_recording"] is True)
    _check("is_recording() True after start",     s.is_recording())

    # record_start when already recording → stays set (idempotent)
    s.apply_cmd({'cmd': 'record_start'})
    _check("record_start again stays True",       s.is_recording())

    s.apply_cmd({'cmd': 'record_stop'})
    _check("record_stop clears cam_recording",    s.get()["cam_recording"] is False)
    _check("is_recording() False after stop",     not s.is_recording())

    # record_stop when not recording → stays cleared (idempotent)
    s.apply_cmd({'cmd': 'record_stop'})
    _check("record_stop again stays False",       not s.is_recording())

    # record_toggle flips both ways
    s.apply_cmd({'cmd': 'record_toggle'})
    _check("record_toggle → True",                s.is_recording())
    s.apply_cmd({'cmd': 'record_toggle'})
    _check("record_toggle → False",               not s.is_recording())


# ── handle_state: packet handler ─────────────────────────────────────────────

def test_handle_state():
    print("\nhandle_state packet handler:")
    s = _fresh()

    flags = SF_RC_ACTIVE | SF_CAM_OK | SF_CAM_RECORDING
    s.handle_state({
        "mode": 1, "flags": flags,
        "drv_left": 0.5, "drv_right": -0.3,
        "speed_scale": 0.75,
    })
    snap = s.get()
    _check("mode AUTO from handle_state",      snap["mode"] == "AUTO")
    _check("rc_active from flags",             snap["rc_active"] is True)
    _check("camera_ok from flags",             snap["camera_ok"] is True)
    _check("cam_recording from flags",         snap["cam_recording"] is True)
    _check("speed_scale set",                  snap["speed_scale"] == 0.75)
    _check("packets incremented",              s.packets == 1)

    # ESTOP mode
    s.handle_state({"mode": 2, "flags": 0, "drv_left": 0.0, "drv_right": 0.0, "speed_scale": 0.25})
    _check("mode ESTOP from handle_state",     s.get()["mode"] == "ESTOP")


# ── handle_telem ──────────────────────────────────────────────────────────────

def test_handle_telem():
    print("\nhandle_telem packet handler:")
    s = _fresh()
    s.handle_telem({"voltage": 11.8, "current": 2.1, "board_temp": 30.0,
                    "left_temp": 28.0, "right_temp": 29.0,
                    "left_fault": False, "right_fault": False,
                    "heading": 90.0, "pitch": 1.5, "roll": -0.5})
    t = s.get()["telemetry"]
    _check("voltage",    abs(t["voltage"]    - 11.8) < 0.01)
    _check("current",    abs(t["current"]    -  2.1) < 0.01)
    _check("board_temp", abs(t["board_temp"] - 30.0) < 0.01)
    _check("heading",    abs(t["heading"]    - 90.0) < 0.01)


# ── handle_gps ────────────────────────────────────────────────────────────────

def test_handle_gps():
    print("\nhandle_gps packet handler:")
    s = _fresh()
    s.handle_gps({"lat": 52.1234, "lon": -0.9876, "alt": 42.0,
                  "speed": 3.5, "hdg": 180.0,
                  "fix": 4, "herr": 0.02, "hdop": 0.9, "sats": 12})
    g = s.get()["gps"]
    _check("latitude",        abs(g["latitude"]    - 52.1234) < 1e-4)
    _check("longitude",       abs(g["longitude"]   - (-0.9876)) < 1e-4)
    _check("fix_quality 4",   g["fix_quality"] == 4)
    _check("fix RTK Fixed",   g["fix_quality_name"] == "RTK Fixed")
    _check("satellites 12",   g["satellites"] == 12)


# ── handle_alarm ──────────────────────────────────────────────────────────────

def test_handle_alarm():
    print("\nhandle_alarm packet handler:")
    s = _fresh()
    s.handle_alarm({"alarm_name": "low_voltage", "severity": 1, "message": "11.0 V"})
    alarms = s.get()["_alarms"]
    _check("alarm count = 1",          len(alarms) == 1)
    _check("alarm name correct",       alarms[0]["name"] == "low_voltage")
    _check("alarm severity warning",   alarms[0]["severity"] == "warning")

    # Ring buffer caps at MAX_ALARMS
    for i in range(_GsState.MAX_ALARMS + 5):
        s.handle_alarm({"alarm_name": f"a{i}", "severity": 0, "message": ""})
    _check("ring buffer capped",       len(s.get()["_alarms"]) == _GsState.MAX_ALARMS)


# ── handle_ina ────────────────────────────────────────────────────────────────

def test_handle_ina():
    print("\nhandle_ina packet handler:")
    s = _fresh()

    # ok=True — all fields populated
    s.handle_ina({"ok": True, "voltage": 12.15, "current": 1.32,
                  "power": 6.25, "die_temp": 23.0})
    ina = s.get()["ina"]
    _check("ok True after good reading",        ina["ok"] is True)
    _check("voltage round-trips",               abs(ina["voltage"]  - 12.15) < 0.01)
    _check("current round-trips",               abs(ina["current"]  -  1.32) < 0.01)
    _check("power round-trips",                 abs(ina["power"]    -  6.25) < 0.01)
    _check("die_temp round-trips",              abs(ina["die_temp"] - 23.0)  < 0.5)

    # ok=False — sensor absent; values become None
    s.handle_ina({"ok": False, "voltage": None, "current": None,
                  "power": None, "die_temp": None})
    ina = s.get()["ina"]
    _check("ok False after failed reading",     ina["ok"] is False)
    _check("voltage None when not ok",          ina["voltage"]  is None)
    _check("current None when not ok",          ina["current"]  is None)
    _check("power None when not ok",            ina["power"]    is None)
    _check("die_temp None when not ok",         ina["die_temp"] is None)


# ── _body_to_cmd_frame ────────────────────────────────────────────────────────

def test_cmd_frame():
    print("\n_body_to_cmd_frame serialisation:")
    _check("estop → CMD_ESTOP",
           _cmd_frame_id({'cmd': 'estop'}) == CMD_ESTOP)
    _check("reset → CMD_RESET_ESTOP",
           _cmd_frame_id({'cmd': 'reset'}) == CMD_RESET_ESTOP)
    _check("data_log_toggle → CMD_DATA_LOG_TOGGLE",
           _cmd_frame_id({'cmd': 'data_log_toggle'}) == CMD_DATA_LOG_TOGGLE)
    _check("record_toggle → CMD_RECORD_TOGGLE",
           _cmd_frame_id({'cmd': 'record_toggle'}) == CMD_RECORD_TOGGLE)
    _check("record_start → CMD_RECORD_TOGGLE",
           _cmd_frame_id({'cmd': 'record_start'}) == CMD_RECORD_TOGGLE)
    _check("record_stop → CMD_RECORD_TOGGLE",
           _cmd_frame_id({'cmd': 'record_stop'}) == CMD_RECORD_TOGGLE)
    _check("no_motors_toggle → CMD_NO_MOTORS_TOGGLE",
           _cmd_frame_id({'cmd': 'no_motors_toggle'}) == CMD_NO_MOTORS_TOGGLE)
    _check("gps_bookmark → CMD_GPS_BOOKMARK",
           _cmd_frame_id({'cmd': 'gps_bookmark'}) == CMD_GPS_BOOKMARK)
    _check("nav_reset → CMD_NAV_RESET",
           _cmd_frame_id({'cmd': 'nav_reset'}) == CMD_NAV_RESET)
    _check("nav_pause_toggle → CMD_NAV_PAUSE_TOGGLE",
           _cmd_frame_id({'cmd': 'nav_pause_toggle'}) == CMD_NAV_PAUSE_TOGGLE)
    _check("unknown cmd → None",
           _body_to_cmd_frame({'cmd': 'banana'}) is None)

    # set_mode encodes the mode parameter
    frame_auto   = _body_to_cmd_frame({'cmd': 'set_mode', 'mode': 'AUTO'})
    frame_manual = _body_to_cmd_frame({'cmd': 'set_mode', 'mode': 'MANUAL'})
    _check("set_mode AUTO frame not None",   frame_auto   is not None)
    _check("set_mode MANUAL frame not None", frame_manual is not None)
    _check("set_mode AUTO ≠ MANUAL",         frame_auto != frame_manual)


# ── api_cmd: record_start/stop idempotency guard ─────────────────────────────

def test_api_cmd_record():
    """record_start only queues CMD when not recording; record_stop only when recording."""
    print("\napi_cmd — record start/stop idempotency:")
    client = app.test_client()

    # Reset shared gs state and queue
    gs._gs._flags = 0
    while not gs._cmd_q.empty():
        try:
            gs._cmd_q.get_nowait()
        except queue.Empty:
            break

    def queue_depth():
        return gs._cmd_q.qsize()

    # record_start when not recording → CMD queued
    before = queue_depth()
    client.post('/api/cmd', json={'cmd': 'record_start'})
    _check("record_start when idle queues CMD",
           queue_depth() == before + 1)
    _check("is_recording() True after record_start",
           gs._gs.is_recording())

    # record_start again when already recording → no CMD queued
    before = queue_depth()
    client.post('/api/cmd', json={'cmd': 'record_start'})
    _check("record_start when recording does not re-queue CMD",
           queue_depth() == before)

    # record_stop when recording → CMD queued
    before = queue_depth()
    client.post('/api/cmd', json={'cmd': 'record_stop'})
    _check("record_stop when recording queues CMD",
           queue_depth() == before + 1)
    _check("is_recording() False after record_stop",
           not gs._gs.is_recording())

    # record_stop again when not recording → no CMD queued
    before = queue_depth()
    client.post('/api/cmd', json={'cmd': 'record_stop'})
    _check("record_stop when idle does not re-queue CMD",
           queue_depth() == before)


# ── Telemetry encode→decode pipeline ─────────────────────────────────────────

def test_telemetry_pipeline():
    """encode_state / encode_telem / encode_gps / encode_ina → FrameDecoder → handle_* roundtrip."""
    print("\nTelemetry encode→decode pipeline:")

    s = _fresh()
    dec = FrameDecoder()

    flags = SF_RC_ACTIVE | SF_CAM_OK | SF_CAM_RECORDING
    state_frame = encode_state(
        mode=1,        # AUTO
        drv_left=0.4, drv_right=-0.4,
        flags=flags,
        speed_scale=0.8,
    )
    for ptype, payload in dec.feed(state_frame):
        if ptype == TYPE_STATE:
            s.handle_state(decode_state(payload))

    snap = s.get()
    _check("mode AUTO from pipeline",      snap["mode"] == "AUTO")
    _check("rc_active from pipeline",      snap["rc_active"] is True)
    _check("cam_recording from pipeline",  snap["cam_recording"] is True)

    # Telemetry
    telem_frame = encode_telem(11.5, 1.8, 28.0, 27.0, 26.0, False, False, 45.0, 2.0, -1.0)
    for ptype, payload in dec.feed(telem_frame):
        if ptype == TYPE_TELEM:
            s.handle_telem(decode_telem(payload))

    t = s.get()["telemetry"]
    _check("voltage 11.5 from pipeline",   abs(t["voltage"] - 11.5) < 0.1)
    _check("heading 45° from pipeline",    abs(t["heading"] - 45.0) < 1.0)

    # GPS
    gps_frame = encode_gps(52.0, -1.0, 100.0, 2.5, 270.0, 4, 0.02, 0.8, 8)
    for ptype, payload in dec.feed(gps_frame):
        if ptype == TYPE_GPS:
            s.handle_gps(decode_gps(payload))

    g = s.get()["gps"]
    _check("latitude 52.0 from pipeline",  abs(g["latitude"]  - 52.0) < 1e-3)
    _check("longitude -1.0 from pipeline", abs(g["longitude"] - (-1.0)) < 1e-3)
    _check("fix RTK Fixed from pipeline",  g["fix_quality_name"] == "RTK Fixed")

    # INA237
    ina_frame = encode_ina(12.18, 1.31, 6.24, 23, ok=True)
    for ptype, payload in dec.feed(ina_frame):
        if ptype == TYPE_INA:
            s.handle_ina(decode_ina(payload))

    ina = s.get()["ina"]
    _check("ina ok from pipeline",              ina["ok"] is True)
    _check("ina voltage 12.18 from pipeline",   abs(ina["voltage"]  - 12.18) < 0.01)
    _check("ina current 1.31 from pipeline",    abs(ina["current"]  -  1.31) < 0.01)
    _check("ina power 6.24 from pipeline",      abs(ina["power"]    -  6.24) < 0.05)
    _check("ina die_temp 23 from pipeline",     abs(ina["die_temp"] - 23.0)  < 0.5)

    # INA ok=False — values become None after decode
    ina_off_frame = encode_ina(0.0, 0.0, 0.0, 0, ok=False)
    for ptype, payload in dec.feed(ina_off_frame):
        if ptype == TYPE_INA:
            s.handle_ina(decode_ina(payload))

    ina = s.get()["ina"]
    _check("ina ok=False from pipeline",        ina["ok"] is False)
    _check("ina voltage None when ok=False",    ina["voltage"] is None)


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    print("=" * 52)
    print("ground_station_v2.py unit tests")
    print("=" * 52)

    test_defaults()
    test_apply_cmd_mode()
    test_apply_cmd_flags()
    test_apply_cmd_recording()
    test_handle_state()
    test_handle_telem()
    test_handle_gps()
    test_handle_alarm()
    test_handle_ina()
    test_cmd_frame()
    test_api_cmd_record()
    test_telemetry_pipeline()

    print(f"\n{'=' * 52}")
    print(f"Ground station tests: {_passed} passed, {_failed} failed")
    print(f"{'=' * 52}")
    return 0 if _failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
