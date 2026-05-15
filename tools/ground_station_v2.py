#!/usr/bin/env python3
"""
tools/ground_station_v2.py — Ground station web dashboard for HackyRacingRobot (v2).

Runs on the operator's laptop (or any machine on the LAN).
Open http://localhost:5000 in any browser.

Uses the binary framing protocol from robot/telemetry_proto.py — no JSON on
the radio link.  The SSE → browser path still sends JSON so the existing
ground_station.html frontend works unchanged.

Data flow
---------
  Downlink  SiK/TCP → binary frames → decoded → SSE → browser
  FPV       local USB capture → MJPEG → browser  (/stream/fpv)
  Uplink    browser → /api/cmd → encode_cmd() → binary frame → SiK/TCP
  RTCM      NTRIP client → encode_rtcm() → binary frame → SiK/TCP

Three serial backends (--backend)
-----------------------------------
  real      Physical SiK radio on a serial port.
              --serial-port /dev/ttyUSB0   (Linux)
              --serial-port COM5           (Windows)

  network   TCP socket — robot must run serial_telemetry_v2.py --tcp-port N
              --network-host 192.168.1.10  --network-port 5010

  fake      Generates synthetic binary telemetry locally using telemetry_proto.
            No hardware needed; exercises the full decode path.

Requires : flask pyserial
Optional : opencv-python  (FPV stream)
           robot/telemetry_proto.py must be on sys.path (repo clone, or copy)

Usage
-----
  python3 tools/ground_station_v2.py --serial-port /dev/ttyUSB0
  python3 tools/ground_station_v2.py --backend network --network-host 192.168.1.10
  python3 tools/ground_station_v2.py --backend fake
  python3 tools/ground_station_v2.py --backend fake --fpv-device 0
  python3 tools/ground_station_v2.py --web-port 8080

NTRIP credentials (preferred via env vars):
  NTRIP_USER=you@example.com NTRIP_PASSWORD=none python3 tools/ground_station_v2.py
"""

import argparse
import base64
import json
import logging
import math
import os
import queue
import signal
import socket
import sys
import threading
import time
from collections import deque
from typing import Optional

# ── Add repo root to path so robot/telemetry_proto is importable ──────────────
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    from robot.telemetry_proto import (
        FrameDecoder,
        TYPE_STATE, TYPE_TELEM, TYPE_GPS, TYPE_SYS, TYPE_NAV, TYPE_LIDAR, TYPE_ALARM, TYPE_TAGS, TYPE_INA,
        TYPE_MOD_TELEM,
        TYPE_CMD, TYPE_RTCM, TYPE_PING,
        decode_state, decode_telem, decode_gps, decode_sys, decode_nav, decode_ina,
        decode_lidar, decode_alarm, decode_tags, decode_mod_telem,
        encode_cmd, encode_rtcm, encode_ping,
        CMD_ESTOP, CMD_RESET_ESTOP, CMD_SET_MODE,
        CMD_DATA_LOG_TOGGLE, CMD_GPS_BOOKMARK,
        CMD_RECORD_TOGGLE, CMD_BENCH_TOGGLE,
        CMD_NO_MOTORS_TOGGLE, CMD_ARUCO_TOGGLE,
        CMD_NAV_RESET, CMD_NAV_PAUSE_TOGGLE,
        MODE_MANUAL, MODE_AUTO,
        # fake generator encoders
        encode_state, encode_telem, encode_mod_telem, encode_gps, encode_sys, encode_nav,
        encode_lidar, encode_alarm, encode_tags, encode_ina,
        CAM_FRONT_LEFT, CAM_FRONT_RIGHT, CAM_REAR,
        state_flags,
        SF_RC_ACTIVE, SF_LIDAR_OK, SF_GPS_OK, SF_CAM_OK,
        SF_CAM_RECORDING, SF_DATA_LOGGING, SF_NO_MOTORS,
        SF_BENCH_ENABLED, SF_FRONT_CAP, SF_REAR_CAP,
        SF_CAM_FR_OK, SF_CAM_RE_OK, SF_AUTO_TYPE_0, SF_AUTO_TYPE_1,
        ALARM_LOW_VOLTAGE, ALARM_SEV_WARNING,
        NAV_IDLE, NAV_DRIVING,
    )
except ImportError as _e:
    sys.exit(f"Cannot import robot.telemetry_proto: {_e}\n"
             f"Ensure the repo root is on sys.path or copy robot/telemetry_proto.py "
             f"to the same directory as this script.")

log = logging.getLogger("ground_station_v2")

try:
    from flask import Flask, Response, request, jsonify
except ImportError:
    sys.exit("flask not found — pip install flask")

try:
    import serial as _pyserial
    _SERIAL = True
except ImportError:
    _SERIAL = False

try:
    import cv2
    _CV2 = True
except ImportError:
    _CV2 = False

# Map fix_quality int → human-readable name
_FIX_NAMES = {0: "Invalid", 1: "GPS", 2: "DGPS", 3: "PPS",
              4: "RTK Fixed", 5: "RTK Float", 6: "Dead Reckoning"}


# ══════════════════════════════════════════════════════════════════════════════
# Ground station state  (updated incrementally from decoded packets)
# ══════════════════════════════════════════════════════════════════════════════

class _GsState:
    """
    Thread-safe store for the latest merged robot state.

    Each decoded packet type updates only the fields it owns; the rest remain
    at their defaults until a packet for that type arrives.

    get() returns a snapshot dict in the same format as robot_dashboard.py's
    _serialise() / ground_station.py's _expand() — so ground_station.html
    works without modification.
    """

    MAX_ALARMS = 50   # alarm log ring buffer depth

    def __init__(self):
        self._lock      = threading.Lock()
        self.last_rx    = 0.0
        self.packets    = 0
        self.packets_by_type: dict[int, int] = {}

        # NTRIP status (updated by NtripClient)
        self.ntrip_status = "disabled"
        self.ntrip_bytes  = 0

        # Alarm ring buffer
        self._alarms: deque = deque(maxlen=self.MAX_ALARMS)

        # State sub-dicts — set once a packet of each type arrives
        self._mode:       str   = "MANUAL"
        self._auto_type:  str   = "Camera"
        self._drv_left:   float = 0.0
        self._drv_right:  float = 0.0
        self._flags:      int   = 0
        self._speed_scale: float = 0.25

        self._telem = {
            "voltage": None, "current": None,
            "board_temp": None, "left_temp": None, "right_temp": None,
            "left_fault": False, "right_fault": False,
            "heading": None, "pitch": None, "roll": None,
            "fl_temp": 0.0, "fr_temp": 0.0, "rl_temp": 0.0, "rr_temp": 0.0,
            "fl_current": 0.0, "fr_current": 0.0, "rl_current": 0.0, "rr_current": 0.0,
            "fl_fault": False, "fr_fault": False, "rl_fault": False, "rr_fault": False,
            "applied_l": 0.0, "applied_r": 0.0,
        }
        self._gps = {
            "latitude": None, "longitude": None, "altitude": None,
            "speed": None, "heading": None,
            "fix_quality": 0, "fix_quality_name": "Invalid",
            "h_error_m": None, "hdop": None,
            "satellites": 0, "satellites_view": 0, "satellites_data": [],
            "ntrip_status": "disabled", "ntrip_bytes_recv": 0,
        }
        self._sys = {
            "cpu_percent": 0.0, "cpu_temp_c": 0.0, "cpu_freq_mhz": 0.0,
            "mem_percent": 0.0, "mem_used_mb": 0.0, "mem_total_mb": 0.0,
            "disk_percent": 0.0, "disk_used_gb": 0.0, "disk_total_gb": 0.0,
        }
        self._nav = {
            "nav_state": "IDLE", "nav_gate": 0, "nav_wp": 0,
            "nav_wp_dist": None, "nav_wp_bear": None,
            "nav_bearing_err": None, "nav_tags_visible": 0,
            "nav_paused": False,
            # Tag IDs computed by formula fallback (gate*2 / gate*2+1)
            # Updated from TYPE_NAV gate field; override with track.toml values when available
            "nav_outside_tag": 0, "nav_inside_tag": 1,
            "nav_gate_label": "",
            "nav_next_gate": 1,
            "nav_next_outside_tag": 2, "nav_next_inside_tag": 3,
            "nav_next_gate_label": "",
        }
        self._lidar = {"angles": [], "distances": []}
        self._ina = {"ok": False, "voltage": None, "current": None, "power": None, "die_temp": None}
        # ArUco tags: per-camera dict → list of tag dicts  (matches s.aruco[camKey].tags)
        self._aruco = {
            "front_left":  {"tags": [], "tag_count": 0, "fps": 0.0, "cap_w": 0, "cap_h": 0, "gates": []},
            "front_right": {"tags": [], "tag_count": 0, "fps": 0.0, "cap_w": 0, "cap_h": 0, "gates": []},
            "rear":        {"tags": [], "tag_count": 0, "fps": 0.0, "cap_w": 0, "cap_h": 0, "gates": []},
        }

    # ── Packet handlers ───────────────────────────────────────────────────────

    def handle_state(self, d: dict):
        mode_names      = {0: "MANUAL", 1: "AUTO", 2: "ESTOP"}
        auto_type_names = {0: "Camera", 1: "GPS", 2: "Cam+GPS"}
        flags = d["flags"]
        with self._lock:
            self._mode        = mode_names.get(d["mode"], "MANUAL")
            self._auto_type   = auto_type_names.get((flags >> 12) & 0x3, "Camera")
            self._drv_left    = d["drv_left"]
            self._drv_right   = d["drv_right"]
            self._flags       = flags
            self._speed_scale = d["speed_scale"]
            self._touch()

    def handle_telem(self, d: dict):
        with self._lock:
            self._telem.update(d)
            self._touch()

    def handle_mod_telem(self, d: dict):
        with self._lock:
            self._telem.update(d)
            self._touch()

    def handle_gps(self, d: dict):
        with self._lock:
            g = self._gps
            g["latitude"]        = d["lat"]
            g["longitude"]       = d["lon"]
            g["altitude"]        = d["alt"]
            g["speed"]           = d["speed"]
            g["heading"]         = d["hdg"]
            g["fix_quality"]     = d["fix"]
            g["fix_quality_name"] = _FIX_NAMES.get(d["fix"], "GPS")
            g["h_error_m"]       = d["herr"]
            g["hdop"]            = d["hdop"]
            g["satellites"]      = d["sats"]
            g["satellites_view"] = d["sats"]
            g["satellites_data"] = d.get("sat_data", [])
            g["ntrip_status"]    = self.ntrip_status
            g["ntrip_bytes_recv"]= self.ntrip_bytes
            self._touch()

    def handle_sys(self, d: dict):
        with self._lock:
            s = self._sys
            s["cpu_percent"]  = d["cpu"]
            s["cpu_temp_c"]   = d["temp"]
            s["mem_percent"]  = d["mem"]
            s["disk_percent"] = d["disk"]
            self._touch()

    def handle_ina(self, d: dict):
        with self._lock:
            self._ina.update(d)
            self._touch()

    def handle_nav(self, d: dict):
        with self._lock:
            gate = d["gate"]
            self._nav["nav_state"]      = d["nav_state_name"]
            self._nav["nav_gate"]       = gate
            self._nav["nav_wp"]         = d["wp"]
            self._nav["nav_wp_dist"]    = d["dist"]
            self._nav["nav_wp_bear"]    = d["bearing"]
            self._nav["nav_bearing_err"]= d["bearing_err"]
            self._nav["nav_tags_visible"] = d["tags"]
            # Tag IDs and labels now carried in extended NAV packet
            self._nav["nav_outside_tag"]    = d["outside_tag"]
            self._nav["nav_inside_tag"]     = d["inside_tag"]
            self._nav["nav_gate_label"]     = d["gate_label"] or f"Gate {gate}"
            next_gate = d["next_gate"]
            self._nav["nav_next_gate"]          = next_gate
            self._nav["nav_next_outside_tag"]   = d["next_outside_tag"]
            self._nav["nav_next_inside_tag"]    = d["next_inside_tag"]
            self._nav["nav_next_gate_label"]    = d["next_gate_label"] or f"Gate {next_gate}"
            self._touch()

    def handle_lidar(self, d: dict):
        with self._lock:
            self._lidar["angles"]    = [float(a) for a in d["angles"]]
            self._lidar["distances"] = [float(x) for x in d["distances"]]
            self._touch()

    def handle_tags(self, tags: list):
        """Rebuild per-camera aruco dicts from decoded TAGS packet."""
        by_cam = {"front_left": [], "front_right": [], "rear": []}
        for t in tags:
            cam = t.get("cam_name", "front_left")
            if cam in by_cam:
                by_cam[cam].append({
                    "id":       t["tag_id"],
                    "cx":       t["cx"],
                    "cy":       t["cy"],
                    "distance": t["distance"],
                    "bearing":  t["bearing"],
                    "area":     t.get("area", 0),
                })
        with self._lock:
            total = 0
            for cam, tag_list in by_cam.items():
                self._aruco[cam]["tags"]      = tag_list
                self._aruco[cam]["tag_count"] = len(tag_list)
                total += len(tag_list)
            self._nav["nav_tags_visible"] = total
            self._touch()

    def handle_alarm(self, d: dict):
        sev_names = {0: "info", 1: "warning", 2: "critical"}
        entry = {
            "ts":       round(time.monotonic(), 1),
            "name":     d["alarm_name"],
            "severity": sev_names.get(d["severity"], "warning"),
            "message":  d["message"],
        }
        with self._lock:
            self._alarms.appendleft(entry)
            self._touch()

    def apply_cmd(self, body: dict):
        """Optimistically update local state when a command is queued.

        Mirrors what the robot will do when it receives the command so the UI
        responds immediately instead of waiting for the radio round-trip.
        Real state from the robot will overwrite these on the next packet.
        """
        cmd = body.get('cmd', '')
        with self._lock:
            if cmd == 'estop':
                self._mode = 'ESTOP'
            elif cmd == 'reset':
                if self._mode == 'ESTOP':
                    self._mode = 'MANUAL'
            elif cmd == 'set_mode':
                mode_str = body.get('mode', 'MANUAL')
                if self._mode != 'ESTOP':
                    self._mode = mode_str
            elif cmd == 'no_motors_toggle':
                self._flags ^= SF_NO_MOTORS
            elif cmd == 'nav_pause_toggle':
                self._nav["nav_paused"] = not self._nav["nav_paused"]
            elif cmd == 'nav_reset':
                pass  # state updates arrive via next TYPE_NAV packet
            elif cmd == 'data_log_toggle':
                self._flags ^= SF_DATA_LOGGING
            elif cmd == 'record_toggle':
                self._flags ^= SF_CAM_RECORDING
            elif cmd == 'record_start':
                self._flags |= SF_CAM_RECORDING
            elif cmd == 'record_stop':
                self._flags &= ~SF_CAM_RECORDING

    def is_recording(self) -> bool:
        with self._lock:
            return bool(self._flags & SF_CAM_RECORDING)

    def _touch(self):
        """Must be called under self._lock."""
        self.packets  += 1
        self.last_rx   = time.monotonic()

    def record_packet_type(self, ptype: int):
        self.packets_by_type[ptype] = self.packets_by_type.get(ptype, 0) + 1

    # ── Snapshot for SSE ──────────────────────────────────────────────────────

    def get(self) -> dict:
        """Return an immutable snapshot dict in robot_dashboard / ground_station_v1 format."""
        with self._lock:
            flags = self._flags
            return {
                # Top-level
                "mode":          self._mode,
                "auto_type":     self._auto_type,
                "speed_scale":   self._speed_scale,
                "rc_active":     bool(flags & SF_RC_ACTIVE),
                "camera_ok":     bool(flags & SF_CAM_OK),
                "lidar_ok":      bool(flags & SF_LIDAR_OK),
                "gps_ok":        bool(flags & SF_GPS_OK),
                "cam_recording": bool(flags & SF_CAM_RECORDING),
                "data_logging":  bool(flags & SF_DATA_LOGGING),
                "no_motors":     bool(flags & SF_NO_MOTORS),
                "bench_enabled": bool(flags & SF_BENCH_ENABLED),
                "aruco_ok":      False,
                "aruco_enabled": {},
                "cam_rotation":  0,
                "gps_logging":   False,
                # Per-camera stubs (single camera info from flags)
                "cam_fl_ok":  bool(flags & SF_CAM_OK),
                "cam_fr_ok":  bool(flags & SF_CAM_FR_OK),
                "cam_re_ok":  bool(flags & SF_CAM_RE_OK),
                "cam_fl_rec": bool(flags & SF_CAM_RECORDING),
                "cam_fr_rec": False,
                "cam_re_rec": False,
                "cam_fl_cap": bool(flags & SF_FRONT_CAP),
                "cam_fr_cap": False,
                "cam_re_cap": bool(flags & SF_REAR_CAP),
                "gate_confirmed": False, "current_gate_id": 0,
                "cam_cap_w": {}, "cam_cap_h": {}, "aruco": dict(self._aruco),
                # Navigation (unpack flat from _nav)
                **self._nav,
                "nav_target_bearing": self._nav["nav_wp_bear"],
                "nav_target_dist":    self._nav["nav_wp_dist"],
                # Drive
                "drive": {
                    "left":  round(self._drv_left,  3),
                    "right": round(self._drv_right, 3),
                },
                # Telemetry
                "telemetry": dict(self._telem),
                # GPS
                "gps": {**self._gps,
                        "ntrip_status":    self.ntrip_status,
                        "ntrip_bytes_recv": self.ntrip_bytes},
                # LiDAR
                "lidar": dict(self._lidar),
                # System
                "system": dict(self._sys),
                # INA237 power monitor
                "ina": dict(self._ina),
                # Ground-station extras
                "_link_age":     round(self.link_age(), 1),
                "_link_packets": self.packets,
                "_ntrip_status": self.ntrip_status,
                "_ntrip_bytes":  self.ntrip_bytes,
                # Battery voltage thresholds (from CLI / robot.ini [battery])
                "batt_warn_v": _batt_warn_v,
                "batt_crit_v": _batt_crit_v,
                # Alarms log (most recent first, for the JS log panel)
                "_alarms": list(self._alarms),
            }

    def link_age(self) -> float:
        return time.monotonic() - self.last_rx if self.last_rx else 999.0


# Battery voltage thresholds — overridden by CLI args at startup
_batt_warn_v: float = 10.5
_batt_crit_v: float = 9.9

_gs = _GsState()


# ══════════════════════════════════════════════════════════════════════════════
# Dispatch decoded frames to _GsState
# ══════════════════════════════════════════════════════════════════════════════

def _dispatch(ptype: int, payload: bytes):
    _gs.record_packet_type(ptype)
    try:
        if   ptype == TYPE_STATE:     _gs.handle_state(decode_state(payload))
        elif ptype == TYPE_TELEM:     _gs.handle_telem(decode_telem(payload))
        elif ptype == TYPE_MOD_TELEM: _gs.handle_mod_telem(decode_mod_telem(payload))
        elif ptype == TYPE_GPS:       _gs.handle_gps(decode_gps(payload))
        elif ptype == TYPE_SYS:       _gs.handle_sys(decode_sys(payload))
        elif ptype == TYPE_NAV:       _gs.handle_nav(decode_nav(payload))
        elif ptype == TYPE_LIDAR:     _gs.handle_lidar(decode_lidar(payload))
        elif ptype == TYPE_ALARM:     _gs.handle_alarm(decode_alarm(payload))
        elif ptype == TYPE_TAGS:      _gs.handle_tags(decode_tags(payload))
        elif ptype == TYPE_INA:       _gs.handle_ina(decode_ina(payload))
        else:
            log.debug("Unknown downlink frame type 0x%02X (%d B)", ptype, len(payload))
    except Exception as e:
        log.warning("Decode error type=0x%02X: %s", ptype, e)


# ══════════════════════════════════════════════════════════════════════════════
# Serial link abstraction
# ══════════════════════════════════════════════════════════════════════════════

class _LinkBase:
    """
    Shared RX decoder and TX queue drainer for all backends.

    RX: feeds raw bytes into a FrameDecoder; dispatches decoded frames.
    TX: drains two queues — cmd_q (encode_cmd frames) and rtcm_q (encode_rtcm frames).
        Also sends a PING heartbeat every 5 s so the robot's link-age watchdog stays fresh.
    """

    _PING_INTERVAL  = 5.0   # seconds between keepalive PINGs to the robot
    _STATS_INTERVAL = 30.0  # seconds between RX stats log lines

    def __init__(self, rtcm_q: queue.Queue, cmd_q: queue.Queue):
        self._rtcm_q  = rtcm_q
        self._cmd_q   = cmd_q
        self._stop    = threading.Event()
        self._tx_lock = threading.Lock()
        self._rx_packets = 0
        self._rx_bytes   = 0
        self._bad_crc    = 0

    def stop(self):
        self._stop.set()

    # ── subclasses implement these ────────────────────────────────────────────

    def _read(self, n: int) -> bytes:
        raise NotImplementedError

    def _write(self, data: bytes):
        raise NotImplementedError

    # ── shared RX loop ────────────────────────────────────────────────────────

    def _rx_loop(self):
        decoder = FrameDecoder()
        while not self._stop.is_set():
            try:
                chunk = self._read(512)
            except Exception as e:
                log.warning("RX error: %s", e)
                time.sleep(1.0)
                continue
            if not chunk:
                continue
            self._rx_bytes += len(chunk)
            for ptype, payload in decoder.feed(chunk):
                self._rx_packets += 1
                _dispatch(ptype, payload)

    # ── shared TX loop ────────────────────────────────────────────────────────

    def _tx_loop(self):
        """
        Drain cmd_q and rtcm_q.  Each item in cmd_q is an already-encoded frame
        (bytes).  Each item in rtcm_q is raw RTCM3 bytes that must be wrapped.
        Also sends a PING every _PING_INTERVAL seconds and logs RX stats every
        _STATS_INTERVAL seconds.
        """
        next_ping  = time.monotonic() + self._PING_INTERVAL
        next_stats = time.monotonic() + self._STATS_INTERVAL

        while not self._stop.is_set():
            now  = time.monotonic()
            sent = False

            # CMD frames (already encoded)
            try:
                frame = self._cmd_q.get_nowait()
                self._safe_write(frame)
                sent = True
            except queue.Empty:
                pass

            # RTCM bytes — wrap and send in chunks to stay within MTU
            try:
                raw = self._rtcm_q.get_nowait()
                chunk_size = 200
                for i in range(0, len(raw), chunk_size):
                    self._safe_write(encode_rtcm(raw[i:i + chunk_size]))
                sent = True
            except queue.Empty:
                pass

            # Periodic PING — keeps robot link-age watchdog fresh
            if now >= next_ping:
                self._safe_write(encode_ping())
                next_ping = now + self._PING_INTERVAL

            # Periodic RX stats log
            if now >= next_stats:
                age = round(_gs.link_age(), 1)
                log.info("RX stats: pkts=%d  bytes=%d  link_age=%.1fs  "
                         "ntrip=%s",
                         self._rx_packets, self._rx_bytes, age,
                         _gs.ntrip_status)
                next_stats = now + self._STATS_INTERVAL

            if not sent:
                time.sleep(0.02)

    def _safe_write(self, data: bytes):
        try:
            with self._tx_lock:
                self._write(data)
        except Exception as e:
            log.warning("TX error: %s", e)

    def _start_threads(self):
        threading.Thread(target=self._rx_loop, daemon=True, name="gs_rx_v2").start()
        threading.Thread(target=self._tx_loop, daemon=True, name="gs_tx_v2").start()


# ══════════════════════════════════════════════════════════════════════════════
# Backend 1: real serial (SiK radio)
# ══════════════════════════════════════════════════════════════════════════════

class SerialLinkV2(_LinkBase):
    """SiK radio serial backend.  Opens the port in a background thread so the
    dashboard starts immediately even when the dongle is not yet plugged in.
    Reconnects automatically if the dongle is unplugged and re-inserted."""

    def __init__(self, port: str, baud: int,
                 rtcm_q: queue.Queue, cmd_q: queue.Queue):
        super().__init__(rtcm_q, cmd_q)
        if not _SERIAL:
            raise ImportError("pyserial not installed — pip install pyserial")
        self._port = port
        self._baud = baud
        self._ser: Optional[_pyserial.Serial] = None
        self._ser_lock = threading.Lock()
        log.info("SiK serial v2: %s @ %d baud", port, baud)

    def _connect(self):
        while not self._stop.is_set():
            try:
                ser = _pyserial.Serial(self._port, self._baud,
                                       timeout=0.1, write_timeout=1.0)
                time.sleep(0.3)
                ser.reset_input_buffer()
                with self._ser_lock:
                    self._ser = ser
                log.info("SiK serial opened: %s", self._port)
                return
            except Exception as e:
                log.warning("SiK serial open failed (%s) — retry in 3 s", e)
                time.sleep(3)

    def _read(self, n):
        with self._ser_lock:
            ser = self._ser
        if ser is None:
            time.sleep(0.1)
            return b""
        try:
            return ser.read(n)
        except Exception as e:
            log.warning("SiK serial read error: %s — reconnecting", e)
            with self._ser_lock:
                try: self._ser.close()
                except Exception: pass
                self._ser = None
            threading.Thread(target=self._connect, daemon=True,
                             name="gs_serial_conn").start()
            return b""

    def _write(self, data):
        with self._ser_lock:
            ser = self._ser
        if ser:
            try:
                ser.write(data)
            except Exception as e:
                log.warning("SiK serial write error: %s", e)

    def stop(self):
        super().stop()
        with self._ser_lock:
            if self._ser:
                try: self._ser.close()
                except Exception: pass
                self._ser = None

    def start(self):
        threading.Thread(target=self._connect, daemon=True,
                         name="gs_serial_conn").start()
        self._start_threads()


# ══════════════════════════════════════════════════════════════════════════════
# Backend 2: TCP network socket
# ══════════════════════════════════════════════════════════════════════════════

class NetworkLinkV2(_LinkBase):
    """
    Connects to the TCP bridge exposed by serial_telemetry_v2.py --tcp-port N.
    Reconnects automatically on disconnect.
    """

    def __init__(self, host: str, port: int,
                 rtcm_q: queue.Queue, cmd_q: queue.Queue):
        super().__init__(rtcm_q, cmd_q)
        self._host = host
        self._port = port
        self._sock: Optional[socket.socket] = None
        self._conn_lock = threading.Lock()
        log.info("Network link v2: %s:%d", host, port)

    def _connect(self):
        while not self._stop.is_set():
            try:
                s = socket.create_connection((self._host, self._port), timeout=5)
                s.settimeout(0.5)
                with self._conn_lock:
                    self._sock = s
                log.info("Network link connected: %s:%d", self._host, self._port)
                return
            except Exception as e:
                log.warning("Network connect failed: %s — retry in 3 s", e)
                time.sleep(3)

    def _read(self, n):
        with self._conn_lock:
            s = self._sock
        if s is None:
            time.sleep(0.1)
            return b""
        try:
            data = s.recv(n)
            if data == b"":
                # recv() returns empty on graceful close (FIN) — treat as disconnect
                raise ConnectionResetError("remote closed connection")
            return data
        except socket.timeout:
            return b""
        except Exception:
            with self._conn_lock:
                self._sock = None
            log.warning("Network link disconnected — reconnecting")
            threading.Thread(target=self._connect, daemon=True,
                             name="gs_net_conn_v2").start()
            return b""

    def _write(self, data):
        with self._conn_lock:
            s = self._sock
        if s:
            s.sendall(data)

    def stop(self):
        super().stop()
        with self._conn_lock:
            if self._sock:
                try: self._sock.close()
                except Exception: pass
                self._sock = None

    def start(self):
        threading.Thread(target=self._connect, daemon=True,
                         name="gs_net_conn_v2").start()
        self._start_threads()


# ══════════════════════════════════════════════════════════════════════════════
# Backend 3: fake data generator
# ══════════════════════════════════════════════════════════════════════════════

class FakeLinkV2(_LinkBase):
    """
    Generates synthetic binary telemetry at 5 Hz using the real telemetry_proto
    encoders, then feeds those bytes through the real FrameDecoder.

    This exercises the complete encode → decode → _GsState path without
    needing any hardware.

    Simulates:
      - Oscillating motor speeds
      - GPS drift around Cambridge
      - Slow IMU heading rotation
      - Battery voltage sag (resets every 60 s)
      - LiDAR wall on left side, obstacle ahead
      - RTK fix cycling
      - Low-voltage alarm when battery sags below threshold
    """

    def __init__(self, rtcm_q: queue.Queue, cmd_q: queue.Queue):
        super().__init__(rtcm_q, cmd_q)
        log.info("Fake data backend v2 — no hardware needed")

    def _read(self, n):
        time.sleep(0.1)
        return b""

    def _write(self, data):
        pass  # discard uplink

    def start(self):
        threading.Thread(target=self._tx_loop, daemon=True, name="gs_tx_v2").start()
        threading.Thread(target=self._generate, daemon=True, name="gs_fake_v2").start()

    def _generate(self):
        decoder = FrameDecoder()   # local decoder: encode → bytes → decode → _dispatch
        t0      = time.monotonic()
        lat0, lon0 = 52.204268, 0.118880   # Cambridge
        modes    = [0, 0, 1]   # MODE_MANUAL, MANUAL, AUTO
        fix_seq  = [0, 1, 1, 1, 4, 4, 4, 4, 4, 5]
        tick     = 0

        while not self._stop.is_set():
            t   = time.monotonic() - t0
            mode = modes[int(t / 5) % len(modes)]
            fix  = fix_seq[int(t / 3) % len(fix_seq)]

            drv_left  = round(math.sin(t * 0.5) * 0.6, 3)
            drv_right = round(math.sin(t * 0.5 + 0.8) * 0.6, 3)
            voltage   = round(12.6 - (t % 60) * 0.015, 2)
            heading   = (t * 8) % 360

            flags = SF_RC_ACTIVE | SF_LIDAR_OK | SF_CAM_OK
            if fix > 0:
                flags |= SF_GPS_OK

            frames = []

            # ── STATE + TELEM every tick ──────────────────────────────────
            frames.append(encode_state(mode, drv_left, drv_right, flags, 0.5))
            mod_curr = round(abs(drv_left + drv_right) * 4, 2)
            frames.append(encode_telem(
                voltage_v   = voltage,
                current_a   = round(mod_curr * 4, 2),
                board_temp  = 38.0,
                left_temp   = 32.0,
                right_temp  = 31.5,
                left_fault  = False,
                right_fault = False,
                heading     = round(heading, 1),
                pitch       = round(math.sin(t * 0.3) * 4, 1),
                roll        = round(math.cos(t * 0.4) * 3, 1),
            ))
            frames.append(encode_mod_telem(
                fl_temp  = round(30.0 + math.sin(t * 0.07) * 3, 1),
                fr_temp  = round(31.0 + math.sin(t * 0.09) * 3, 1),
                rl_temp  = round(29.5 + math.sin(t * 0.11) * 3, 1),
                rr_temp  = round(30.5 + math.sin(t * 0.13) * 3, 1),
                fl_curr  = mod_curr, fr_curr = mod_curr,
                rl_curr  = mod_curr, rr_curr = mod_curr,
                fl_fault = False, fr_fault = False,
                rl_fault = False, rr_fault = False,
            ))

            # ── Low voltage alarm ─────────────────────────────────────────
            if voltage < _batt_warn_v:
                frames.append(encode_alarm(
                    ALARM_LOW_VOLTAGE, ALARM_SEV_WARNING,
                    f"Low voltage {voltage:.2f}V"
                ))

            # ── GPS + NAV at 2 Hz ─────────────────────────────────────────
            if tick % 2 == 0:
                lat = lat0 + math.sin(t * 0.05) * 0.0002
                lon = lon0 + math.cos(t * 0.05) * 0.0002
                _fake_sats = [
                    {"svid": 1+i, "elev": 10+i*4, "azim": (i*37)%360,
                     "snr": 30+int(math.sin(t+i)*10),
                     "system": ("GPS","GLO","GAL","BDS")[i%4]}
                    for i in range(10)
                ]
                frames.append(encode_gps(
                    lat      = round(lat, 7),
                    lon      = round(lon, 7),
                    alt      = 12.5,
                    speed    = round(abs(drv_left + drv_right) * 2, 2),
                    gps_hdg  = round(heading, 1),
                    fix      = fix,
                    herr     = 0.025 if fix == 4 else (0.3 if fix == 5 else None),
                    hdop     = 0.9,
                    sats     = 10,
                    sat_data = _fake_sats,
                ))
                nav_st = NAV_DRIVING if mode == 1 else NAV_IDLE
                frames.append(encode_nav(
                    nav_state   = nav_st,
                    gate        = int(t / 10) % 5,
                    wp          = 0,
                    dist        = round(abs(math.sin(t * 0.15)) * 40, 1),
                    bearing     = round(heading + math.sin(t * 0.2) * 15, 1),
                    bearing_err = round(math.sin(t * 0.3) * 8, 1),
                    tags        = 2 if mode == 1 else 0,
                ))

            # ── SYS + LIDAR at 1 Hz ───────────────────────────────────────
            if tick % 5 == 0:
                frames.append(encode_sys(
                    cpu  = round(25 + math.sin(t * 0.1) * 10, 1),
                    mem  = 42.0,
                    disk = 18.5,
                    temp = round(52 + math.sin(t * 0.07) * 5, 1),
                ))
                frames.append(encode_ina(
                    voltage_v = round(12.1 + math.sin(t * 0.05) * 0.2, 3),
                    current_a = round(1.2  + math.sin(t * 0.13) * 0.3, 3),
                    power_w   = round(14.5 + math.sin(t * 0.09) * 3.0, 2),
                    die_temp  = round(28.0 + math.sin(t * 0.04) * 2.0, 1),
                    ok        = True,
                ))

                dists = []
                for deg in range(0, 360, 5):
                    if 85 <= deg <= 155:
                        d = int(2000 + math.sin(math.radians(deg)) * 200)
                    elif deg >= 355 or deg <= 5:
                        d = int(1500 + math.sin(t) * 200)
                    else:
                        d = int(4000 + math.sin(math.radians(deg + t * 20)) * 500)
                    dists.append(max(200, d))
                frames.append(encode_lidar(dists, step=5))

            # Feed all encoded frames through the local decoder → _dispatch
            for frame in frames:
                for ptype, payload in decoder.feed(frame):
                    _dispatch(ptype, payload)

            tick += 1
            time.sleep(0.2)   # 5 Hz


# ══════════════════════════════════════════════════════════════════════════════
# NTRIP client  (identical to ground_station.py v1)
# ══════════════════════════════════════════════════════════════════════════════

class NtripClient:
    def __init__(self, host, port, mount, user, password,
                 lat, lon, alt, rtcm_q):
        self._host, self._port     = host, port
        self._mount                = mount
        self._user, self._password = user, password
        self._lat, self._lon, self._alt = lat, lon, alt
        self._q    = rtcm_q
        self._stop = threading.Event()
        threading.Thread(target=self._run, daemon=True, name="ntrip_v2").start()

    def stop(self):
        self._stop.set()

    def _gga(self):
        lat, lon = abs(self._lat), abs(self._lon)
        ns = 'N' if self._lat >= 0 else 'S'
        ew = 'E' if self._lon >= 0 else 'W'
        gga = (f"GPGGA,{time.strftime('%H%M%S')}.00,"
               f"{int(lat):02d}{(lat-int(lat))*60:07.4f},{ns},"
               f"{int(lon):03d}{(lon-int(lon))*60:07.4f},{ew},"
               f"1,08,1.0,{self._alt:.1f},M,0.0,M,,")
        ck = 0
        for c in gga:
            ck ^= ord(c)
        return f"${gga}*{ck:02X}\r\n"

    def _run(self):
        cred = base64.b64encode(
            f"{self._user}:{self._password}".encode()).decode()
        while not self._stop.is_set():
            _gs.ntrip_status = "connecting"
            try:
                sock = socket.create_connection((self._host, self._port), timeout=10)
                req  = (f"GET /{self._mount} HTTP/1.0\r\n"
                        f"Host: {self._host}\r\n"
                        f"User-Agent: HackyRacingRobot/2.0\r\n"
                        f"Authorization: Basic {cred}\r\n"
                        f"Ntrip-Version: Ntrip/2.0\r\n\r\n")
                sock.sendall(req.encode())
                resp = b""
                while b"\r\n\r\n" not in resp:
                    resp += sock.recv(256)
                if b"200" not in resp and b"ICY 200" not in resp:
                    raise ConnectionError(f"Bad response: {resp[:60]}")
                sock.sendall(self._gga().encode())
                _gs.ntrip_status = "connected"
                last_gga = time.monotonic()
                sock.settimeout(5.0)
                while not self._stop.is_set():
                    try:
                        chunk = sock.recv(1024)
                    except socket.timeout:
                        if time.monotonic() - last_gga > 30:
                            sock.sendall(self._gga().encode())
                            last_gga = time.monotonic()
                        continue
                    if not chunk:
                        raise ConnectionError("Connection closed")
                    _gs.ntrip_bytes += len(chunk)
                    self._q.put(chunk)   # raw RTCM — TX loop wraps in encode_rtcm()
            except Exception as e:
                _gs.ntrip_status = "error"
                log.warning("NTRIP: %s", e)
                time.sleep(5)


# ══════════════════════════════════════════════════════════════════════════════
# FPV capture  (identical to ground_station.py v1)
# ══════════════════════════════════════════════════════════════════════════════

class FpvCapture:
    def __init__(self, device):
        self._device = device
        self._frame: Optional[bytes] = None
        self._lock   = threading.Lock()
        self._stop   = threading.Event()
        self.ok      = False
        if _CV2:
            threading.Thread(target=self._run, daemon=True, name="fpv_v2").start()
        else:
            log.warning("FPV disabled — pip install opencv-python")

    def stop(self):
        self._stop.set()

    def get_jpeg(self) -> Optional[bytes]:
        with self._lock:
            return self._frame

    def _run(self):
        try:
            dev = int(self._device)
        except (ValueError, TypeError):
            dev = self._device
        cap = cv2.VideoCapture(dev)
        if not cap.isOpened():
            log.warning("FPV: cannot open device %s", self._device)
            return
        self.ok = True
        log.info("FPV capture: device %s", self._device)
        while not self._stop.is_set():
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.05)
                continue
            ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                with self._lock:
                    self._frame = buf.tobytes()
        try:
            cap.release()
        except Exception:
            pass
        self.ok = False


# ══════════════════════════════════════════════════════════════════════════════
# CMD string → binary frame mapping
# ══════════════════════════════════════════════════════════════════════════════

# Maps the HTML's sendCmd('name') string to (cmd_id, default_param)
_CMD_MAP: dict[str, tuple[int, int]] = {
    'estop':            (CMD_ESTOP,            0),
    'reset':            (CMD_RESET_ESTOP,       0),
    'data_log_toggle':  (CMD_DATA_LOG_TOGGLE,   0),
    'gps_bookmark':     (CMD_GPS_BOOKMARK,      0),
    'record_toggle':    (CMD_RECORD_TOGGLE,     0),
    'record_start':     (CMD_RECORD_TOGGLE,     0),
    'record_stop':      (CMD_RECORD_TOGGLE,     0),
    'bench_toggle':     (CMD_BENCH_TOGGLE,      0),
    'no_motors_toggle': (CMD_NO_MOTORS_TOGGLE,  0),
    'aruco_toggle':     (CMD_ARUCO_TOGGLE,      0),
    'nav_reset':        (CMD_NAV_RESET,         0),
    'nav_pause_toggle': (CMD_NAV_PAUSE_TOGGLE,  0),
}


def _body_to_cmd_frame(body: dict) -> Optional[bytes]:
    """Convert a /api/cmd POST body to an encoded binary CMD frame, or None."""
    cmd  = body.get('cmd', '')
    if cmd == 'set_mode':
        mode_str = body.get('mode', 'MANUAL')
        param    = MODE_AUTO if mode_str == 'AUTO' else MODE_MANUAL
        return encode_cmd(CMD_SET_MODE, param)
    mapping = _CMD_MAP.get(cmd)
    if mapping is None:
        return None
    cmd_id, param = mapping
    return encode_cmd(cmd_id, param)


# ══════════════════════════════════════════════════════════════════════════════
# Flask application
# ══════════════════════════════════════════════════════════════════════════════

app     = Flask(__name__)
_fpv:   Optional[FpvCapture] = None
_cmd_q: queue.Queue          = queue.Queue(maxsize=20)
_HTML   = ""


@app.route('/')
def index():
    return Response(_HTML, mimetype='text/html')


@app.route('/api/state')
def api_state():
    """Single-shot state snapshot (polled by browser at ~250 ms)."""
    return jsonify(_gs.get())


@app.route('/stream/fpv')
def stream_fpv():
    """MJPEG stream from the local FPV USB capture device."""
    def _gen():
        while True:
            data = _fpv.get_jpeg() if _fpv else None
            if data:
                yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + data + b'\r\n')
                time.sleep(0.033)
            else:
                time.sleep(0.5)
    return Response(
        _gen(),
        mimetype='multipart/x-mixed-replace; boundary=frame',
        headers={'Cache-Control': 'no-store'},
    )


@app.route('/stream/<cam>')
def stream_cam(cam):
    """Stub for robot-side cameras — serves a static grey placeholder."""
    placeholder: Optional[bytes] = None
    if _CV2:
        import numpy as np
        img = np.full((240, 320, 3), 28, dtype=np.uint8)
        cv2.putText(img, f"{cam}", (10, 100),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (80, 80, 80), 1)
        cv2.putText(img, "not available at", (10, 125),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (80, 80, 80), 1)
        cv2.putText(img, "ground station", (10, 148),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (80, 80, 80), 1)
        _, buf = cv2.imencode('.jpg', img)
        placeholder = buf.tobytes()

    def _gen():
        while True:
            if placeholder:
                yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
                       + placeholder + b'\r\n')
            time.sleep(5.0)
    return Response(
        _gen(),
        mimetype='multipart/x-mixed-replace; boundary=frame',
        headers={'Cache-Control': 'no-store'},
    )


@app.route('/api/cmd', methods=['POST'])
def api_cmd():
    body = request.json or {}
    cmd  = body.get('cmd', '')

    # record_start/stop: only send the toggle when state needs to change so we
    # match dashboard behaviour (start_cam_recording / stop_cam_recording).
    if cmd == 'record_start' and _gs.is_recording():
        _gs.apply_cmd(body)  # optimistic update only; robot already recording
        return jsonify({'ok': True, 'state': _gs.get()})
    if cmd == 'record_stop' and not _gs.is_recording():
        _gs.apply_cmd(body)
        return jsonify({'ok': True, 'state': _gs.get()})

    frame = _body_to_cmd_frame(body)
    if frame is None:
        return jsonify({'ok': True, 'ignored': True,
                        'reason': f"unknown cmd '{cmd}'"})
    try:
        _cmd_q.put_nowait(frame)
        log.info("CMD queued: %s", cmd)
        # Apply optimistic local state so the UI updates immediately without
        # waiting for the radio round-trip confirmation.
        _gs.apply_cmd(body)
        return jsonify({'ok': True, 'state': _gs.get()})
    except queue.Full:
        return jsonify({'ok': False, 'error': 'TX queue full'}), 503


@app.route('/api/status')
def api_status():
    """Diagnostic endpoint — shows link stats."""
    return jsonify({
        'link_age_s':       round(_gs.link_age(), 1),
        'packets_total':    _gs.packets,
        'packets_by_type':  {f"0x{k:02X}": v for k, v in _gs.packets_by_type.items()},
        'ntrip_status':     _gs.ntrip_status,
        'ntrip_bytes':      _gs.ntrip_bytes,
        'fpv_ok':           bool(_fpv and _fpv.ok),
        'alarms':           list(_gs._alarms),
    })


@app.route('/api/logs')
def api_logs():
    return jsonify({'lines': []})


# ══════════════════════════════════════════════════════════════════════════════
# Entry point
# ══════════════════════════════════════════════════════════════════════════════

def _local_ip() -> str:
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "localhost"


def main():
    global _fpv, _HTML

    ap = argparse.ArgumentParser(
        description="HackyRacingRobot ground station v2 (binary protocol)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--backend", default="real",
                    choices=["real", "network", "fake"],
                    help="real=SiK serial, network=TCP bridge, fake=synthetic")
    # real backend
    ap.add_argument("--serial-port",  default="/dev/ttyUSB0")
    ap.add_argument("--baud",         default=57600, type=int)
    # network backend
    ap.add_argument("--network-host", default="192.168.1.1")
    ap.add_argument("--network-port", default=5010, type=int)
    # web
    ap.add_argument("--web-port",     default=5000, type=int)
    ap.add_argument("--web-host",     default="0.0.0.0")
    # FPV
    ap.add_argument("--no-fpv",       action="store_true")
    ap.add_argument("--fpv-device",   default="0",
                    help="FPV capture device index or path (default: 0)")
    # NTRIP
    ap.add_argument("--no-ntrip",     action="store_true")
    ap.add_argument("--ntrip-host",   default="www.rtk2go.com")
    ap.add_argument("--ntrip-port",   default=2101, type=int)
    ap.add_argument("--ntrip-mount",  default="CAMBRIDGE")
    ap.add_argument("--ntrip-user",
                    default=os.environ.get("NTRIP_USER", "user@example.com"))
    ap.add_argument("--ntrip-password",
                    default=os.environ.get("NTRIP_PASSWORD", "none"))
    ap.add_argument("--ntrip-lat",    default=52.2,  type=float)
    ap.add_argument("--ntrip-lon",    default=0.1,   type=float)
    ap.add_argument("--ntrip-alt",    default=20.0,  type=float)
    # Battery voltage thresholds (derived from [battery] chemistry×cells in robot.ini)
    ap.add_argument("--batt-warn-v",  default=10.5, type=float,
                    help="Pack voltage warn threshold V (default 10.5 = lipo 3S)")
    ap.add_argument("--batt-crit-v",  default=9.9,  type=float,
                    help="Pack voltage critical threshold V (default 9.9 = lipo 3S)")
    # HTML
    ap.add_argument("--html",         default=None,
                    help="Path to dashboard HTML file (default: ground_station.html "
                         "in the same directory)")
    ap.add_argument("--log-level",    default="INFO")
    args = ap.parse_args()

    global _batt_warn_v, _batt_crit_v
    _batt_warn_v = args.batt_warn_v
    _batt_crit_v = args.batt_crit_v

    logging.basicConfig(
        level=getattr(logging, args.log_level.upper(), logging.INFO),
        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    # ── HTML ──────────────────────────────────────────────────────────────────
    html_path = args.html or os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "ground_station.html")
    if not os.path.exists(html_path):
        sys.exit(
            f"HTML not found: {html_path}\n"
            "Generate it first:\n"
            "  python3 tools/build_gs_html.py\n"
            "Or pass --html /path/to/ground_station.html"
        )
    with open(html_path, encoding="utf-8") as f:
        _HTML = f.read()

    # ── Radio backend ─────────────────────────────────────────────────────────
    rtcm_q: queue.Queue = queue.Queue(maxsize=200)

    if args.backend == "real":
        if not _SERIAL:
            sys.exit("pyserial not installed — pip install pyserial")
        link = SerialLinkV2(args.serial_port, args.baud, rtcm_q, _cmd_q)
        link.start()
        log.info("Backend: serial  %s @ %d baud (will retry until port opens)",
                 args.serial_port, args.baud)

    elif args.backend == "network":
        link = NetworkLinkV2(args.network_host, args.network_port, rtcm_q, _cmd_q)
        link.start()
        log.info("Backend: network  %s:%d", args.network_host, args.network_port)

    else:  # fake
        link = FakeLinkV2(rtcm_q, _cmd_q)
        link.start()
        log.info("Backend: fake  (synthetic binary telemetry at 5 Hz)")
        args.no_ntrip = True

    # ── NTRIP ─────────────────────────────────────────────────────────────────
    ntrip = None
    if not args.no_ntrip:
        ntrip = NtripClient(
            host     = args.ntrip_host,
            port     = args.ntrip_port,
            mount    = args.ntrip_mount,
            user     = args.ntrip_user,
            password = args.ntrip_password,
            lat      = args.ntrip_lat,
            lon      = args.ntrip_lon,
            alt      = args.ntrip_alt,
            rtcm_q   = rtcm_q,
        )
        log.info("NTRIP: %s/%s", args.ntrip_host, args.ntrip_mount)
    else:
        _gs.ntrip_status = "disabled"

    # ── FPV ───────────────────────────────────────────────────────────────────
    if not args.no_fpv:
        _fpv = FpvCapture(args.fpv_device)

    # ── Shutdown ──────────────────────────────────────────────────────────────
    def _shutdown(sig, frame):
        log.info("Shutting down")
        link.stop()
        if ntrip: ntrip.stop()
        if _fpv:  _fpv.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT,  _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    # ── Print startup banner ──────────────────────────────────────────────────
    ip = _local_ip()
    print(f"\n  Ground station v2  →  http://{ip}:{args.web_port}/")
    print(f"  Protocol           :  binary (telemetry_proto v2)")
    print(f"  Backend            :  {args.backend}", end="")
    if args.backend == "real":
        print(f"  ({args.serial_port} @ {args.baud} baud)")
    elif args.backend == "network":
        print(f"  ({args.network_host}:{args.network_port})")
    else:
        print(f"  (synthetic data)")
    print(f"  FPV                :  "
          f"{'device ' + args.fpv_device if not args.no_fpv else 'disabled'}")
    print(f"  NTRIP              :  "
          f"{'disabled' if args.no_ntrip else args.ntrip_host + '/' + args.ntrip_mount}")
    print(f"  Status API         :  http://{ip}:{args.web_port}/api/status")
    print()

    app.run(host=args.web_host, port=args.web_port,
            threaded=True, use_reloader=False)


if __name__ == "__main__":
    main()
