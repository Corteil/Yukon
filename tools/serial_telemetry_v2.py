#!/usr/bin/env python3
"""
tools/serial_telemetry_v2.py — SiK radio telemetry bridge v2 for HackyRacingRobot.

Replaces tools/serial_telemetry.py with a compact binary framing protocol
defined in robot/telemetry_proto.py.  JSON is gone; every packet is a
fixed-layout struct wrapped in an 8-byte frame with CRC-16 error detection.

Downlink  (robot → ground)  rates
-----------------------------------
  STATE   0x01   5 Hz   mode, drive, flags bitmask, speed_scale
  TELEM   0x02   5 Hz   voltage, current, temps, faults, IMU hdg/pit/rol
  GPS     0x03   2 Hz   lat/lon/alt/spd/hdg/fix/herr/hdop/sats
  NAV     0x05   2 Hz   nav state, gate/wp, dist, bearing, bearing_err, tags
  SYS     0x04   1 Hz   cpu%, mem%, disk%, pi_temp
  LIDAR   0x06   1 Hz   distance array (zlib compressed)
  ALARM   0x07   event  triggered alarm (sent once on state change)

Uplink  (ground → robot)
--------------------------
  CMD   0x81   button press / control command
  RTCM  0x82   raw RTCM3 correction bytes → injected into TAU1308
  PING  0x83   keepalive (resets link-age watchdog)

Bandwidth: ~300 bytes/sec at default rates — 5% of 57600-baud budget.
Compare to v1 JSON: ~3600–5100 bytes/sec (63–88%).

robot.ini section
-----------------
  [telemetry_radio]
  disabled     = false
  port         = /dev/ttyUSB1    ; SiK radio port
  baud         = 57600           ; must match ATS1 on both radios
  telemetry_hz = 5               ; STATE+TELEM rate (2–10 Hz)
  lidar        = true            ; include LiDAR at 1 Hz
  lidar_step   = 5               ; degrees between samples (1–45)

Usage
-----
  python3 tools/serial_telemetry_v2.py
  python3 tools/serial_telemetry_v2.py --port /dev/ttyUSB1 --baud 57600
  python3 tools/serial_telemetry_v2.py --no-lidar
  python3 tools/serial_telemetry_v2.py --no-yukon --no-gps   # bench test
  python3 tools/serial_telemetry_v2.py --no-serial --tcp-port 5010  # LAN test
"""

import argparse
import configparser
import logging
import os
import signal
import sys
import threading
import time

# Add repo root to path so robot/ package is importable when running from tools/
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robot.telemetry_proto import (
    FrameDecoder,
    encode_state, encode_telem, encode_mod_telem, encode_gps, encode_sys, encode_nav, encode_ina,
    encode_lidar, encode_alarm, encode_tags, encode_cmd, encode_rtcm,
    decode_cmd,
    state_flags, lidar_to_step_array,
    CAM_FRONT_LEFT, CAM_FRONT_RIGHT, CAM_REAR,
    TYPE_CMD, TYPE_RTCM, TYPE_PING,
    NAV_IDLE, NAV_SEARCHING, NAV_ALIGNING, NAV_DRIVING, NAV_ARRIVED, NAV_COMPLETE,
    ALARM_ESTOP, ALARM_RC_LOST, ALARM_MOTOR_FAULT, ALARM_GPS_LOST,
    ALARM_LIDAR_LOST, ALARM_CAMERA_LOST, ALARM_LOW_VOLTAGE, ALARM_OVERTEMP,
    ALARM_SEV_WARNING, ALARM_SEV_CRITICAL,
    CMD_ESTOP, CMD_RESET_ESTOP, CMD_SET_MODE,
    CMD_DATA_LOG_TOGGLE, CMD_GPS_BOOKMARK,
    CMD_RECORD_TOGGLE, CMD_BENCH_TOGGLE,
    CMD_NO_MOTORS_TOGGLE, CMD_ARUCO_TOGGLE,
    CMD_NAV_RESET, CMD_NAV_PAUSE_TOGGLE,
    MODE_MANUAL, MODE_AUTO,
)

log = logging.getLogger("serial_telemetry_v2")


# ── Config helpers ─────────────────────────────────────────────────────────────

def _cfg(cfg, section, key, fallback, cast=str):
    try:
        v = cfg.get(section, key).split('#')[0].strip()
        return cast(v) if v else fallback
    except Exception:
        return fallback


def _cfg_bool(cfg, section, key, fallback: bool) -> bool:
    try:
        v = cfg.get(section, key).split('#')[0].strip().lower()
        return v not in ('false', '0', 'no', 'off')
    except Exception:
        return fallback


# ══════════════════════════════════════════════════════════════════════════════
# Alarm tracker — detects state transitions and emits ALARM packets once
# ══════════════════════════════════════════════════════════════════════════════

class _AlarmTracker:
    """
    Watches RobotState for alarm-worthy transitions and yields ALARM frames
    each time a new alarm fires.  Each alarm fires once on the rising edge
    (OK → fault) and is suppressed until it clears and re-fires.
    """

    _LOW_VOLTAGE_V = 11.5   # below this threshold triggers LOW_VOLTAGE alarm
    _OVERTEMP_C    = 70.0   # CPU temp above this triggers OVERTEMP alarm

    def __init__(self):
        self._prev: dict[int, bool] = {}  # alarm_id → was_active last tick
        self._baseline_set = False        # first call silently establishes baseline

    def check(self, state) -> list[bytes]:
        """Return a (possibly empty) list of encoded ALARM frames."""
        from robot_daemon import RobotMode

        alarms_now: dict[int, tuple[bool, int, str]] = {
            # alarm_id: (active, severity, message)
            ALARM_ESTOP:       (state.mode == RobotMode.ESTOP,
                                     ALARM_SEV_CRITICAL, "ESTOP active"),
            ALARM_RC_LOST:     (not state.rc_active,
                                     ALARM_SEV_WARNING, "RC signal lost"),
            ALARM_MOTOR_FAULT: (state.telemetry.left_fault or state.telemetry.right_fault,
                                     ALARM_SEV_CRITICAL,
                                     "Motor fault: " + (
                                         "left+right" if (state.telemetry.left_fault and
                                                          state.telemetry.right_fault)
                                         else "left" if state.telemetry.left_fault
                                         else "right")),
            ALARM_GPS_LOST:    (not state.gps_ok,
                                     ALARM_SEV_WARNING, "GPS signal lost"),
            ALARM_LIDAR_LOST:  (not state.lidar_ok,
                                     ALARM_SEV_WARNING, "LiDAR offline"),
            ALARM_CAMERA_LOST: (not state.camera_ok,
                                     ALARM_SEV_WARNING, "Camera offline"),
            ALARM_LOW_VOLTAGE: (state.telemetry.voltage > 0 and
                                     state.telemetry.voltage < self._LOW_VOLTAGE_V,
                                     ALARM_SEV_CRITICAL,
                                     f"Low voltage {state.telemetry.voltage:.2f}V"),
            ALARM_OVERTEMP:    (state.system.cpu_temp_c > self._OVERTEMP_C,
                                     ALARM_SEV_WARNING,
                                     f"CPU overtemp {state.system.cpu_temp_c:.0f}°C"),
        }

        # First call: silently record current state as baseline so we only fire
        # alarms on *transitions*, not for conditions already present at startup
        # (e.g. rc_active=False before the first CMD_RC_QUERY reply comes back).
        if not self._baseline_set:
            for alarm_id, (active, _, _) in alarms_now.items():
                self._prev[alarm_id] = active
            self._baseline_set = True
            return []

        frames = []
        for alarm_id, (active, severity, message) in alarms_now.items():
            was_active = self._prev.get(alarm_id, False)
            if active and not was_active:
                frames.append(encode_alarm(alarm_id, severity, message))
                log.warning("ALARM %s: %s", alarm_id, message)
            self._prev[alarm_id] = active
        return frames


# ══════════════════════════════════════════════════════════════════════════════
# Main bridge class
# ══════════════════════════════════════════════════════════════════════════════

class TelemetryBridgeV2:
    """
    Owns the SiK radio serial port.

    TX thread — multi-rate scheduler:
      every tick (5 Hz):  STATE, TELEM, alarm check
      every 2nd  tick:    GPS, NAV
      every 5th  tick:    SYS
      every Nth  tick:    LIDAR (1 Hz — N = ceil(hz/1))

    RX thread — decodes binary frames:
      TYPE_CMD  → dispatches robot command
      TYPE_RTCM → injects RTCM bytes into TAU1308
      TYPE_PING → updates link-age timestamp (no reply needed)
    """

    def __init__(self, robot, port: str, baud: int,
                 telemetry_hz: float, lidar_enabled: bool, lidar_step: int):
        self._robot        = robot
        self._port         = port
        self._baud         = baud
        self._hz           = max(1.0, min(20.0, telemetry_hz))
        self.lidar_enabled = lidar_enabled
        self._lidar_step   = lidar_step

        self._ser          = None
        self._stop         = threading.Event()
        self._tx_thread:  threading.Thread | None = None
        self._rx_thread:  threading.Thread | None = None
        self._tx_lock      = threading.Lock()

        self._alarm_tracker = _AlarmTracker()
        self._last_rx_ts    = 0.0   # monotonic time of last received frame

        # TCP broadcast helpers (set by _start_tcp_bridge)
        self._tcp_broadcast = None   # callable(bytes) or None

        # Stats
        self.packets_sent      = 0
        self.bytes_sent        = 0
        self.rtcm_bytes_fwd    = 0
        self.cmds_received     = 0
        self.frames_bad_crc    = 0

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def start(self, no_serial: bool = False):
        if not no_serial:
            import serial
            log.info("Opening SiK radio on %s @ %d baud", self._port, self._baud)
            self._ser = serial.Serial(self._port, self._baud,
                                      timeout=0.05, write_timeout=1.0)
            time.sleep(0.3)
            self._ser.reset_input_buffer()
            self._rx_thread = threading.Thread(target=self._rx_loop,
                                               daemon=True, name="sik_rx_v2")
            self._rx_thread.start()
        else:
            log.info("SiK radio serial disabled — TX packets go to TCP clients only")

        self._tx_thread = threading.Thread(target=self._tx_loop,
                                           daemon=True, name="sik_tx_v2")
        self._tx_thread.start()
        log.info("TelemetryBridgeV2 started  hz=%.1f  lidar=%s  step=%d°",
                 self._hz, self.lidar_enabled, self._lidar_step)

    def stop(self):
        self._stop.set()
        for t in (self._tx_thread, self._rx_thread):
            if t and t.is_alive():
                t.join(timeout=2.0)
        if self._ser:
            try:
                self._ser.close()
            except Exception:
                pass
        log.info("TelemetryBridgeV2 stopped  "
                 "pkts=%d  bytes=%d  rtcm_fwd=%d  cmds=%d  bad_crc=%d",
                 self.packets_sent, self.bytes_sent,
                 self.rtcm_bytes_fwd, self.cmds_received, self.frames_bad_crc)

    @property
    def link_age_s(self) -> float:
        """Seconds since the last uplink frame was received (any type)."""
        if self._last_rx_ts == 0.0:
            return float('inf')
        return time.monotonic() - self._last_rx_ts

    # ── TX ────────────────────────────────────────────────────────────────────

    def _send(self, frame: bytes):
        """Send one encoded frame to the radio and/or TCP clients."""
        try:
            if self._ser is not None:
                with self._tx_lock:
                    self._ser.write(frame)
            if self._tcp_broadcast is not None:
                self._tcp_broadcast(frame)
            self.packets_sent += 1
            self.bytes_sent   += len(frame)
        except Exception as e:
            log.warning("TX error: %s", e)

    def _tx_loop(self):
        from robot_daemon import RobotMode

        mode_map = {
            RobotMode.MANUAL: 0,
            RobotMode.AUTO:   1,
            RobotMode.ESTOP:  2,
        }
        nav_state_map = {
            "IDLE":      NAV_IDLE,
            "SEARCHING": NAV_SEARCHING,
            "ALIGNING":  NAV_ALIGNING,
            "DRIVING":   NAV_DRIVING,
            "ARRIVED":   NAV_ARRIVED,
            "COMPLETE":  NAV_COMPLETE,
        }

        period        = 1.0 / self._hz
        lidar_ticks   = max(1, round(self._hz))   # 1 Hz
        gps_ticks     = max(1, round(self._hz / 2))  # 2 Hz
        stats_period  = 30.0

        tick          = 0
        next_wake     = time.monotonic()
        next_stats    = time.monotonic() + stats_period

        while not self._stop.is_set():
            now = time.monotonic()

            try:
                state = self._robot.get_state()
                t     = state.telemetry
                g     = state.gps
                s     = state.system

                # ── Always: STATE + TELEM ─────────────────────────────────
                self._send(encode_state(
                    mode        = mode_map.get(state.mode, 0),
                    drv_left    = state.drive.left,
                    drv_right   = state.drive.right,
                    flags       = state_flags(state),
                    speed_scale = state.speed_scale,
                ))
                self._send(encode_telem(
                    voltage_v   = t.voltage,
                    current_a   = t.current,
                    board_temp  = t.board_temp,
                    left_temp   = t.left_temp,
                    right_temp  = t.right_temp,
                    left_fault  = t.left_fault,
                    right_fault = t.right_fault,
                    heading     = t.heading,
                    pitch       = t.pitch,
                    roll        = t.roll,
                    applied_l   = t.applied_l,
                    applied_r   = t.applied_r,
                ))
                self._send(encode_mod_telem(
                    fl_temp  = t.fl_temp,  fr_temp  = t.fr_temp,
                    rl_temp  = t.rl_temp,  rr_temp  = t.rr_temp,
                    fl_curr  = t.fl_current, fr_curr = t.fr_current,
                    rl_curr  = t.rl_current, rr_curr = t.rr_current,
                    fl_fault = t.fl_fault, fr_fault  = t.fr_fault,
                    rl_fault = t.rl_fault, rr_fault  = t.rr_fault,
                ))

                # ── Alarm check (fires only on transition) ────────────────
                for alarm_frame in self._alarm_tracker.check(state):
                    self._send(alarm_frame)

                # ── Always: TAGS (all cameras) ────────────────────────────
                all_tags = []
                for cam_name, cam_id in (('front_left',  CAM_FRONT_LEFT),
                                         ('front_right', CAM_FRONT_RIGHT),
                                         ('rear',        CAM_REAR)):
                    aruco = self._robot.get_aruco_state(cam_name)
                    if aruco is not None:
                        for tg in aruco.tags.values():
                            all_tags.append({
                                "tag_id":   tg.id,
                                "cam_id":   cam_id,
                                "cx":       tg.center_x,
                                "cy":       tg.center_y,
                                "distance": tg.distance,
                                "bearing":  tg.bearing,
                                "area":     tg.area,
                            })
                self._send(encode_tags(all_tags))

                # ── 2 Hz: GPS + NAV ───────────────────────────────────────
                if tick % gps_ticks == 0:
                    self._send(encode_gps(
                        lat      = g.latitude,
                        lon      = g.longitude,
                        alt      = g.altitude,
                        speed    = g.speed,
                        gps_hdg  = g.heading,
                        fix      = g.fix_quality,
                        herr     = g.h_error_m,
                        hdop     = g.hdop,
                        sats     = g.satellites or 0,
                        sat_data = g.satellites_data or [],
                    ))
                    nav_st = nav_state_map.get(state.nav_state, NAV_IDLE)
                    self._send(encode_nav(
                        nav_state        = nav_st,
                        gate             = state.nav_gate or 0,
                        wp               = state.nav_wp   or 0,
                        dist             = state.nav_wp_dist,
                        bearing          = state.nav_wp_bear,
                        bearing_err      = state.nav_bearing_err,
                        tags             = state.nav_tags_visible or 0,
                        outside_tag      = state.nav_outside_tag,
                        inside_tag       = state.nav_inside_tag,
                        next_outside_tag = state.nav_next_outside_tag,
                        next_inside_tag  = state.nav_next_inside_tag,
                        next_gate        = state.nav_next_gate,
                        gate_label       = state.nav_gate_label       or "",
                        next_gate_label  = state.nav_next_gate_label  or "",
                    ))

                # ── 1 Hz: SYS ────────────────────────────────────────────
                if tick % lidar_ticks == 0:
                    self._send(encode_sys(
                        cpu  = s.cpu_percent,
                        mem  = s.mem_percent,
                        disk = s.disk_percent,
                        temp = s.cpu_temp_c,
                    ))

                # ── 1 Hz: INA237 ─────────────────────────────────────────
                if tick % lidar_ticks == 0:
                    self._send(encode_ina(
                        voltage_v = state.pi_input_voltage,
                        current_a = state.pi_input_current,
                        power_w   = state.pi_input_power,
                        die_temp  = state.pi_ina_temp,
                        ok        = state.pi_ina_ok,
                    ))

                # ── 1 Hz: LIDAR ───────────────────────────────────────────
                if self.lidar_enabled and tick % lidar_ticks == 0:
                    scan  = state.lidar
                    dists = lidar_to_step_array(scan, self._lidar_step)
                    self._send(encode_lidar(dists, self._lidar_step))

            except Exception as e:
                log.warning("TX loop error: %s", e)

            # Periodic stats log
            if now >= next_stats:
                log.info("TX stats: pkts=%d  bytes=%d  rtcm_fwd=%d  "
                         "cmds=%d  bad_crc=%d  link_age=%.0fs",
                         self.packets_sent, self.bytes_sent,
                         self.rtcm_bytes_fwd, self.cmds_received,
                         self.frames_bad_crc, self.link_age_s)
                next_stats = now + stats_period

            tick      += 1
            next_wake += period
            sleep_for  = next_wake - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_wake = time.monotonic()   # fell behind — reset

    # ── RX ────────────────────────────────────────────────────────────────────

    def _rx_loop(self):
        decoder = FrameDecoder()

        while not self._stop.is_set():
            try:
                chunk = self._ser.read(512)
            except Exception as e:
                log.warning("RX serial error: %s", e)
                time.sleep(1.0)
                continue

            if not chunk:
                continue

            for ptype, payload in decoder.feed(chunk):
                self._last_rx_ts = time.monotonic()
                if ptype == TYPE_CMD:
                    self._handle_cmd(payload)
                elif ptype == TYPE_RTCM:
                    self._forward_rtcm(payload)
                elif ptype == TYPE_PING:
                    pass  # link-age already updated above
                else:
                    log.debug("Unknown uplink frame type 0x%02X (%d B)", ptype, len(payload))

    def inject_uplink(self, frame_bytes: bytes):
        """Feed a pre-decoded frame from a TCP client into the RX dispatch path.

        Used by the TCP bridge so LAN clients share the same command/RTCM path
        as the radio RX loop.
        """
        decoder = FrameDecoder()
        for ptype, payload in decoder.feed(frame_bytes):
            self._last_rx_ts = time.monotonic()
            if ptype == TYPE_CMD:
                self._handle_cmd(payload)
            elif ptype == TYPE_RTCM:
                self._forward_rtcm(payload)
            elif ptype == TYPE_PING:
                pass

    def _handle_cmd(self, payload: bytes):
        from robot_daemon import RobotMode

        d = decode_cmd(payload)
        cmd_id = d["cmd_id"]
        param  = d["param"]
        log.info("Uplink CMD %s (0x%02X) param=%d", d["cmd_name"], cmd_id, param)
        self.cmds_received += 1

        r = self._robot
        try:
            if   cmd_id == CMD_ESTOP:            r.estop()
            elif cmd_id == CMD_RESET_ESTOP:      r.reset_estop()
            elif cmd_id == CMD_SET_MODE:
                r.set_mode(RobotMode.AUTO if param == MODE_AUTO else RobotMode.MANUAL)
            elif cmd_id == CMD_DATA_LOG_TOGGLE:
                r.stop_data_log() if r.is_data_logging() else r.start_data_log()
            elif cmd_id == CMD_GPS_BOOKMARK:     r.bookmark_gps()
            elif cmd_id == CMD_RECORD_TOGGLE:
                r.stop_cam_recording() if r.is_cam_recording() else r.start_cam_recording()
            elif cmd_id == CMD_BENCH_TOGGLE:
                r.set_bench(not r.get_state().bench_enabled)
            elif cmd_id == CMD_NO_MOTORS_TOGGLE:
                r.set_no_motors(not r.get_state().no_motors)
            elif cmd_id == CMD_ARUCO_TOGGLE:      r.toggle_aruco()
            elif cmd_id == CMD_NAV_RESET:         r.reset_nav()
            elif cmd_id == CMD_NAV_PAUSE_TOGGLE:  r.toggle_nav_pause()
            else:
                log.debug("Unhandled CMD 0x%02X", cmd_id)
        except Exception as e:
            log.warning("CMD dispatch error (cmd=0x%02X): %s", cmd_id, e)

    def _forward_rtcm(self, data: bytes):
        if not data:
            return
        ok = self._robot.inject_rtcm(data)
        if ok:
            self.rtcm_bytes_fwd += len(data)
            log.debug("RTCM fwd %d B (total %d B)", len(data), self.rtcm_bytes_fwd)
        else:
            log.debug("RTCM dropped — GPS not available (%d B)", len(data))


# ══════════════════════════════════════════════════════════════════════════════
# TCP bridge  (LAN / loopback testing without physical radio)
# ══════════════════════════════════════════════════════════════════════════════

def _start_tcp_bridge(bridge: TelemetryBridgeV2, port: int):
    """
    Expose the radio link over a raw TCP socket.

    Each connected client receives every outgoing binary frame (tee'd from the
    TX thread).  Bytes sent by the client are fed through the same FrameDecoder
    path as the radio RX loop — CMD and RTCM frames are dispatched identically.

    Connect the ground station with:
        python3 tools/ground_station_v2.py --backend network --network-port <port>
    """
    import socket as _socket

    clients: list = []
    clients_lock  = threading.Lock()

    def _broadcast(frame: bytes):
        with clients_lock:
            dead = []
            for sock in clients:
                try:
                    sock.sendall(frame)
                except Exception:
                    dead.append(sock)
            for s in dead:
                clients.remove(s)
                try:
                    s.close()
                except Exception:
                    pass

    # Register broadcast callback with the bridge
    bridge._tcp_broadcast = _broadcast

    def _client_rx(sock: _socket.socket, addr):
        buf = bytearray()
        try:
            while True:
                chunk = sock.recv(512)
                if not chunk:
                    break
                buf.extend(chunk)
                bridge.inject_uplink(bytes(buf))
                buf.clear()
        except Exception:
            pass
        finally:
            with clients_lock:
                if sock in clients:
                    clients.remove(sock)
            try:
                sock.close()
            except Exception:
                pass
            log.info("TCP bridge: client disconnected %s:%d", *addr)

    def _accept_loop():
        srv = _socket.socket(_socket.AF_INET, _socket.SOCK_STREAM)
        srv.setsockopt(_socket.SOL_SOCKET, _socket.SO_REUSEADDR, 1)
        srv.bind(("0.0.0.0", port))
        srv.listen(4)
        log.info("TCP bridge listening on :%d  "
                 "(ground_station_v2.py --backend network --network-port %d)", port, port)
        while True:
            try:
                sock, addr = srv.accept()
                log.info("TCP bridge: client connected %s:%d", *addr)
                with clients_lock:
                    clients.append(sock)
                threading.Thread(target=_client_rx, args=(sock, addr),
                                 daemon=True, name=f"tcp_rx_{addr[1]}").start()
            except Exception as e:
                log.warning("TCP bridge accept error: %s", e)

    threading.Thread(target=_accept_loop, daemon=True, name="tcp_bridge_v2").start()


# ══════════════════════════════════════════════════════════════════════════════
# Entry point
# ══════════════════════════════════════════════════════════════════════════════

def main():
    ap = argparse.ArgumentParser(
        description="SiK radio telemetry bridge v2 for HackyRacingRobot")
    ap.add_argument("--config",     default="robot.ini")
    ap.add_argument("--port",       default=None,
                    help="SiK radio serial port (overrides robot.ini)")
    ap.add_argument("--baud",       default=None, type=int)
    ap.add_argument("--hz",         default=None, type=float,
                    help="STATE+TELEM rate Hz (default 5)")
    ap.add_argument("--no-lidar",   action="store_true")
    ap.add_argument("--lidar-step", default=None, type=int,
                    help="LiDAR angular step in degrees (default 5)")
    ap.add_argument("--no-camera",  action="store_true")
    ap.add_argument("--no-gps",     action="store_true")
    ap.add_argument("--no-motors",  action="store_true")
    ap.add_argument("--no-yukon",   action="store_true",
                    help="Skip Yukon connection — useful on bench")
    ap.add_argument("--no-serial",  action="store_true",
                    help="Skip opening the SiK serial port (TCP-only mode)")
    ap.add_argument("--tcp-port",   default=None, type=int,
                    help="Also expose the link over TCP on this port "
                         "(ground_station_v2.py --backend network). "
                         "Example: --tcp-port 5010")
    ap.add_argument("--log-level",  default="INFO")
    args = ap.parse_args()

    logging.basicConfig(
        level=getattr(logging, args.log_level.upper(), logging.INFO),
        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    cfg = configparser.ConfigParser(inline_comment_prefixes=(';', '#'))
    cfg.read(args.config)

    sik_port   = args.port       or _cfg(cfg, "telemetry_radio", "port",         "/dev/ttyUSB1")
    sik_baud   = args.baud       or _cfg(cfg, "telemetry_radio", "baud",         57600,  cast=int)
    telem_hz   = args.hz         or _cfg(cfg, "telemetry_radio", "telemetry_hz", 5.0,    cast=float)
    lidar_step = args.lidar_step or _cfg(cfg, "telemetry_radio", "lidar_step",   5,      cast=int)
    lidar_on   = (not args.no_lidar and
                  _cfg_bool(cfg, "telemetry_radio", "lidar", True))

    log.info("SiK port=%s  baud=%d  hz=%.1f  lidar=%s (step=%d°)",
             sik_port, sik_baud, telem_hz, lidar_on, lidar_step)

    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from robot_daemon import Robot, setup_logging
    setup_logging()

    camera_disabled = args.no_camera or _cfg_bool(cfg, "camera",  "disabled", False)
    gps_disabled    = args.no_gps    or _cfg_bool(cfg, "gps",     "disabled", False)

    yukon_port = None
    if not args.no_yukon:
        raw = _cfg(cfg, "robot", "yukon_port", "auto")
        yukon_port = None if raw.lower() in ("auto", "") else raw

    robot = Robot(
        yukon_port     = yukon_port,
        ibus_port      = _cfg(cfg, "robot", "ibus_port",           "/dev/ttyAMA3"),
        lidar_port     = _cfg(cfg, "lidar", "port",                "/dev/ttyAMA0"),
        gps_port       = _cfg(cfg, "gps",   "port",                "/dev/ttyUSB0"),
        ntrip_host     = "" if _cfg_bool(cfg, "ntrip", "disabled", False)
                            else _cfg(cfg, "ntrip", "host", ""),
        ntrip_port     = _cfg(cfg, "ntrip", "port",                2101,  int),
        ntrip_mount    = _cfg(cfg, "ntrip", "mount",               ""),
        ntrip_user     = _cfg(cfg, "ntrip", "user",                ""),
        ntrip_password = _cfg(cfg, "ntrip", "password",            ""),
        ntrip_lat      = _cfg(cfg, "ntrip", "lat",                 0.0,   float),
        ntrip_lon      = _cfg(cfg, "ntrip", "lon",                 0.0,   float),
        ntrip_height   = _cfg(cfg, "ntrip", "height",              0.0,   float),
        rtcm_port      = "",   # v2 bridge handles RTCM via uplink frames
        enable_camera  = not camera_disabled,
        enable_lidar   = not _cfg_bool(cfg, "lidar", "disabled",   False),
        enable_gps     = not gps_disabled,
        cam_width      = _cfg(cfg, "camera", "width",              640,   int),
        cam_height     = _cfg(cfg, "camera", "height",             480,   int),
        cam_fps        = _cfg(cfg, "camera", "fps",                30,    int),
        cam_rotation   = _cfg(cfg, "camera", "rotation",           0,     int),
        enable_aruco   = _cfg_bool(cfg, "aruco", "enabled",        False),
        aruco_dict     = _cfg(cfg, "aruco",  "dict",               "DICT_4X4_1000"),
        aruco_calib    = _cfg(cfg, "aruco",  "calib_file",         ""),
        aruco_tag_size = _cfg(cfg, "aruco",  "tag_size",           0.15,  float),
        aruco_area_k   = _cfg(cfg, "aruco",  "area_k",             0.0,   float),
        aruco_hfov     = _cfg(cfg, "aruco",  "hfov",               0.0,   float),
        throttle_ch    = _cfg(cfg, "rc", "throttle_ch",            3,     int),
        steer_ch       = _cfg(cfg, "rc", "steer_ch",               1,     int),
        mode_ch        = _cfg(cfg, "rc", "mode_ch",                5,     int),
        speed_ch       = _cfg(cfg, "rc", "speed_ch",               6,     int),
        auto_type_ch   = _cfg(cfg, "rc", "auto_type_ch",           7,     int),
        gps_log_ch     = _cfg(cfg, "rc", "gps_log_ch",             8,     int),
        dlog_ch        = _cfg(cfg, "rc", "dlog_ch",                0,     int),
        rec_ch         = _cfg(cfg, "rc", "rec_ch",                 0,     int),
        gps_bookmark_ch= _cfg(cfg, "rc", "gps_bookmark_ch",        10,    int),
        pause_ch       = _cfg(cfg, "rc", "pause_ch",               0,     int),
        gps_log_dir    = _cfg(cfg, "gps", "log_dir",               ""),
        gps_log_hz     = _cfg(cfg, "gps", "log_hz",                5.0,   float),
        deadzone       = _cfg(cfg, "rc", "deadzone",               30,    int),
        failsafe_s     = _cfg(cfg, "rc", "failsafe_s",             0.5,   float),
        speed_min      = _cfg(cfg, "rc", "speed_min",              0.25,  float),
        control_hz     = _cfg(cfg, "rc", "control_hz",             50,    int),
        no_motors      = args.no_motors,
        rec_dir        = _cfg(cfg, "output", "videos_dir",         ""),
        max_recording_minutes = _cfg(cfg, "output", "max_recording_minutes", 0.0, float),
        data_log_dir   = _cfg(cfg, "output", "data_log_dir",       ""),
    )

    bridge = TelemetryBridgeV2(
        robot         = robot,
        port          = sik_port,
        baud          = sik_baud,
        telemetry_hz  = telem_hz,
        lidar_enabled = lidar_on,
        lidar_step    = lidar_step,
    )

    def _shutdown(sig, frame):
        log.info("Shutting down")
        bridge.stop()
        robot.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT,  _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    robot.start()
    bridge.start(no_serial=args.no_serial)

    if args.tcp_port:
        _start_tcp_bridge(bridge, args.tcp_port)

    log.info("Running — Ctrl+C to stop")
    while True:
        time.sleep(1.0)


if __name__ == "__main__":
    main()
