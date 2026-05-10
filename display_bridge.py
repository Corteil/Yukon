#!/usr/bin/env python3
"""
display_bridge.py — JC3248W535C USB-serial telemetry bridge for HackyRacingRobot.

Runs alongside robot_daemon.py — pass it a live Robot instance and call start().
It polls get_state() at the configured rate and pushes binary packets to the
display over USB-CDC serial.  It also listens for command packets from the
display (e-stop, mode changes, etc.) and acts on them immediately.

Usage (in your launcher / robot_gui.py equivalent):

    from display_bridge import DisplayBridge
    bridge = DisplayBridge(robot)
    bridge.start()
    ...
    bridge.stop()

Command-line test (no Robot — sends canned telemetry):

    python3 display_bridge.py --test
"""

from __future__ import annotations

import argparse
import logging
import math
import os
import socket
import struct
import threading
import time
from typing import Optional

log = logging.getLogger(__name__)

# ── Protocol constants (must match display firmware) ──────────────────────────

HEADER       = b'\xAA\x55'

# Pi → Display
PKT_DRIVE    = 0x10   # drive + mode + nav state + 4-motor powers
PKT_GPS      = 0x11   # full GPS state
PKT_TAGNAV   = 0x12   # ArUco gate navigator state
PKT_INFO     = 0x13   # IPs, battery rails, system stats, motor currents
PKT_CAMERA   = 0x14   # per-camera status, fps, recording state
PKT_FAULTS   = 0x15   # active fault list + subsystem health bitmask

# Display → Pi
CMD_ESTOP        = 0x30
CMD_RESET_ESTOP  = 0x31
CMD_SET_AUTO     = 0x32
CMD_SET_MANUAL   = 0x33
CMD_REBOOT       = 0x34
CMD_SNAPSHOT     = 0x35   # value byte: 0=front_left 1=front_right 2=rear
CMD_RECORD_START = 0x36
CMD_RECORD_STOP  = 0x37
CMD_CLEAR_FAULTS = 0x38
CMD_BENCH_ON     = 0x39
CMD_BENCH_OFF    = 0x3A


def _crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) if (crc & 0x80) else (crc << 1)
        crc &= 0xFF
    return crc


def _build_packet(ptype: int, payload: bytes) -> bytes:
    hdr = HEADER + bytes([ptype, len(payload)])
    return hdr + payload + bytes([_crc8(hdr[2:] + payload)])


# ── Per-packet builders ───────────────────────────────────────────────────────

def _get_ips() -> tuple[str, str]:
    """Return (wifi_ip, hostname).  Never raises."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(('8.8.8.8', 80))
            wifi_ip = s.getsockname()[0]
    except OSError:
        wifi_ip = '0.0.0.0'
    try:
        hostname = socket.gethostname()
    except Exception:
        hostname = 'robot'
    return wifi_ip, hostname


def _pack_str(s: str, maxlen: int = 31) -> bytes:
    b = s.encode()[:maxlen]
    return bytes([len(b)]) + b


def build_drive(state) -> bytes:
    """PKT_DRIVE — drive state, mode, 4-motor power percentages.

    Yukon slot mapping (from yukon_firmware_and_software/main.py):
      SLOT1 = mod_rr = rear-right   → motor_core applies -right  (physically reversed)
      SLOT2 = mod_fr = front-right  → motor_core applies -right
      SLOT3 = mod_fl = front-left   → motor_core applies  left
      SLOT4 = mod_rl = rear-left    → motor_core applies  left

    drive.left  = _motor_sp[0] = FL + RL command  (-1..+1)
    drive.right = _motor_sp[1] = FR + RR command  (-1..+1, hardware sign-inverted)

    We display the *physical* direction as seen by the robot, so right-side
    motors show the same sign as left when driving forward.
    """
    t   = state.telemetry
    d   = state.drive
    mode_b = state.mode.name.encode()[:15]
    nav_b  = state.nav_state.encode()[:23]

    def to_pct(v): return max(-100, min(100, int(v * 100)))

    # Left side (SLOT3=FL, SLOT4=RL): direct from drive.left
    fl = to_pct(d.left)
    rl = to_pct(d.left)
    # Right side (SLOT2=FR, SLOT1=RR): motor_core applies -right to hardware,
    # so display the commanded value (positive = forward) without sign flip.
    fr = to_pct(d.right)
    rr = to_pct(d.right)

    payload = struct.pack('>ff', d.left, d.right)  # raw -1..+1
    payload += struct.pack('>bbbb', fl, fr, rl, rr) # motor % signed bytes
    payload += struct.pack('>ff', t.heading or 0.0,  # IMU heading
                                  0.0)               # yaw rate placeholder
    payload += bytes([len(mode_b)]) + mode_b
    payload += bytes([len(nav_b)]) + nav_b
    payload += bytes([1 if state.mode.name == 'ESTOP' else 0])
    payload += bytes([1 if state.no_motors else 0])
    return _build_packet(PKT_DRIVE, payload)


def build_gps(state) -> bytes:
    """PKT_GPS — full GPS state."""
    g = state.gps
    lat = g.latitude  or 0.0
    lon = g.longitude or 0.0
    alt = g.altitude  or 0.0
    spd = g.speed     or 0.0
    hdg = g.heading   or 0.0
    hdop = g.hdop     or 0.0
    sats = g.satellites or 0
    fq   = g.fix_quality

    fix_name_b  = g.fix_quality_name.encode()[:15]
    ntrip_b     = g.ntrip_status.encode()[:15]

    payload  = struct.pack('>dddffBBHff',
        lat, lon, alt,
        spd, hdg,
        int(fq), int(g.fix),
        g.ntrip_bytes_recv & 0xFFFF,
        hdop, 0.0)  # h_error placeholder
    payload += bytes([sats & 0xFF])
    payload += bytes([len(fix_name_b)]) + fix_name_b
    payload += bytes([len(ntrip_b)])    + ntrip_b
    return _build_packet(PKT_GPS, payload)


def build_tagnav(state) -> bytes:
    """PKT_TAGNAV — ArUco gate navigator state."""
    nav_state_b  = state.nav_state.encode()[:23]
    gate_label_b = state.nav_gate_label.encode()[:23]
    next_label_b = state.nav_next_gate_label.encode()[:23]

    dist  = state.nav_target_dist  or 0.0
    bear  = state.nav_target_bearing or 0.0
    berr  = state.nav_bearing_err  or 0.0
    wpdist = state.nav_wp_dist     or 0.0
    wpbear = state.nav_wp_bear     or 0.0

    payload  = struct.pack('>HHHHHHHfffff',
        state.nav_gate  & 0xFFFF,
        state.nav_outside_tag & 0xFFFF,
        state.nav_inside_tag  & 0xFFFF,
        state.nav_next_gate   & 0xFFFF,
        state.nav_next_outside_tag & 0xFFFF,
        state.nav_next_inside_tag  & 0xFFFF,
        state.nav_tags_visible & 0xFFFF,
        dist, bear, berr,
        wpdist, wpbear)
    payload += bytes([len(nav_state_b)])  + nav_state_b
    payload += bytes([len(gate_label_b)]) + gate_label_b
    payload += bytes([len(next_label_b)]) + next_label_b
    return _build_packet(PKT_TAGNAV, payload)


def build_info(state, wifi_ip: str, hostname: str) -> bytes:
    """PKT_INFO — IPs, battery, Pi system stats, per-motor current/temp.

    Per-motor telemetry fields on state.telemetry come from robot_daemon.py's
    Telemetry dataclass, which maps the Yukon RESP_* sensor IDs:
      fl_current / fl_temp  ← RESP_CURR_FL / RESP_TEMP_FL  (SLOT3)
      fr_current / fr_temp  ← RESP_CURR_FR / RESP_TEMP_FR  (SLOT2)
      rl_current / rl_temp  ← RESP_CURR_RL / RESP_TEMP_RL  (SLOT4)
      rr_current / rr_temp  ← RESP_CURR_RR / RESP_TEMP_RR  (SLOT1)
    Current values are absolute (|A|); direction is inferred from drive state.
    """
    t  = state.telemetry
    sy = state.system

    ip_b   = wifi_ip.encode()[:15]
    host_b = hostname.encode()[:23]

    payload  = struct.pack('>ffffffff',
        t.voltage,      # main pack voltage
        t.current,      # total current
        t.fl_current, t.fr_current, t.rl_current, t.rr_current,
        t.fl_temp, t.fr_temp)
    payload += struct.pack('>ffff',
        t.rl_temp, t.rr_temp,
        t.board_temp, t.bench_temp)
    payload += struct.pack('>ffffff',
        sy.cpu_percent, sy.cpu_temp_c, sy.cpu_freq_mhz,
        sy.mem_used_mb, sy.mem_total_mb, sy.mem_percent)
    payload += struct.pack('>fff',
        sy.disk_used_gb, sy.disk_total_gb, sy.disk_percent)
    payload += bytes([len(ip_b)])   + ip_b
    payload += bytes([len(host_b)]) + host_b
    # fault bitmask: bit0=fl bit1=fr bit2=rl bit3=rr bit4=bench
    fault_mask = (
        (int(t.fl_fault) << 0) |
        (int(t.fr_fault) << 1) |
        (int(t.rl_fault) << 2) |
        (int(t.rr_fault) << 3) |
        (int(t.bench_fault) << 4)
    )
    payload += bytes([fault_mask])
    payload += bytes([1 if state.bench_enabled else 0])
    return _build_packet(PKT_INFO, payload)


def build_camera(state) -> bytes:
    """PKT_CAMERA — per-camera ok/recording/cap flags."""
    flags = (
        (int(state.cam_front_left_ok)        << 0) |
        (int(state.cam_front_right_ok)       << 1) |
        (int(state.cam_rear_ok)              << 2) |
        (int(state.cam_front_left_recording) << 3) |
        (int(state.cam_front_right_recording)<< 4) |
        (int(state.cam_rear_recording)       << 5) |
        (int(state.cam_front_left_cap)       << 6) |
        (int(state.cam_front_right_cap)      << 7)
    )
    flags2 = (
        (int(state.cam_rear_cap)       << 0) |
        (int(state.aruco_ok)           << 1) |
        (int(state.robot_det_ok)       << 2) |
        (int(state.depth_ok)           << 3)
    )
    payload  = bytes([flags, flags2])
    payload += bytes([state.robot_count & 0xFF])
    return _build_packet(PKT_CAMERA, payload)


def build_faults(state) -> bytes:
    """PKT_FAULTS — subsystem health bitmask + active fault strings."""
    t = state.telemetry

    # Health bitmask (1 = ok)
    health = (
        (int(not (t.fl_fault or t.fr_fault or t.rl_fault or t.rr_fault)) << 0) |  # motors
        (int(state.gps_ok)     << 1) |
        (int(state.lidar_ok)   << 2) |
        (int(state.rc_active)  << 3) |
        (int(state.camera_ok)  << 4) |
        (int(state.aruco_ok)   << 5) |
        (int(state.robot_det_ok) << 6) |
        (int(not t.bench_fault)  << 7)
    )

    # Build fault strings
    faults = []
    if t.fl_fault:  faults.append(('E', 'Motor FL fault'))
    if t.fr_fault:  faults.append(('E', 'Motor FR fault'))
    if t.rl_fault:  faults.append(('E', 'Motor RL fault'))
    if t.rr_fault:  faults.append(('E', 'Motor RR fault'))
    if t.bench_fault: faults.append(('E', 'FPV power fault'))
    if state.mode.name == 'ESTOP':
        faults.append(('E', 'ESTOP active'))
    if not state.gps_ok:
        faults.append(('W', 'GPS unavailable'))
    if not state.rc_active:
        faults.append(('W', 'RC link lost'))
    if not state.lidar_ok:
        faults.append(('W', 'LiDAR unavailable'))

    payload  = bytes([health, len(faults)])
    for severity, msg in faults[:8]:  # cap at 8
        mb = msg.encode()[:47]
        payload += bytes([ord(severity), len(mb)]) + mb
    return _build_packet(PKT_FAULTS, payload)


# ── Incoming command handler ──────────────────────────────────────────────────

class _InboundParser:
    """Reads inbound command packets from the display on a background thread."""

    def __init__(self, robot, bridge: 'DisplayBridge'):
        self._robot  = robot
        self._bridge = bridge

    def handle(self, ptype: int, payload: bytes):
        r = self._robot
        if r is None:
            return
        try:
            if ptype == CMD_ESTOP:
                log.warning('[display] E-STOP from display')
                r.estop()
            elif ptype == CMD_RESET_ESTOP:
                log.info('[display] reset ESTOP')
                r.reset_estop()
            elif ptype == CMD_SET_AUTO:
                from robot_daemon import RobotMode
                r.set_mode(RobotMode.AUTO)
            elif ptype == CMD_SET_MANUAL:
                from robot_daemon import RobotMode
                r.set_mode(RobotMode.MANUAL)
            elif ptype == CMD_REBOOT:
                log.warning('[display] reboot requested')
                os.system('sudo reboot')
            elif ptype == CMD_SNAPSHOT:
                cam = ['front_left', 'front_right', 'rear'][payload[0] if payload else 0]
                frame = r.get_frame(cam)
                if frame is not None:
                    import cv2
                    from datetime import datetime
                    ts  = datetime.now().strftime('%Y%m%d_%H%M%S')
                    path = os.path.expanduser(f'~/Pictures/HackyRacingRobot/snap_{cam}_{ts}.jpg')
                    os.makedirs(os.path.dirname(path), exist_ok=True)
                    cv2.imwrite(path, cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
                    log.info('[display] snapshot saved: %s', path)
            elif ptype == CMD_RECORD_START:
                r.start_cam_recording()
            elif ptype == CMD_RECORD_STOP:
                r.stop_cam_recording()
            elif ptype == CMD_BENCH_ON:
                r.set_bench(True)
            elif ptype == CMD_BENCH_OFF:
                r.set_bench(False)
            elif ptype == CMD_CLEAR_FAULTS:
                # No explicit fault-clear API; reset_estop is the relevant one
                if hasattr(r, 'reset_estop'):
                    r.reset_estop()
        except Exception as e:
            log.error('[display] command %02X error: %s', ptype, e)


# ── Bridge class ──────────────────────────────────────────────────────────────

class DisplayBridge:
    """
    Thread-safe bridge between a Robot instance and the JC3248W535C display.

    Parameters
    ----------
    robot       : Robot instance (or None for test mode)
    port        : serial port, e.g. '/dev/ttyACM0' or 'auto'
    hz          : telemetry send rate (default 10 Hz)
    """

    DISPLAY_PORT = '/dev/ttyACM_display'  # set a udev rule; falls back below
    #FALLBACK_PORTS = ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyACM2']
    FALLBACK_PORTS = ['/dev/ttyACM1', '/dev/ttyACM2']
    BAUD = 115200

    def __init__(self, robot=None, port: str = 'auto', hz: float = 10.0):
        self._robot    = robot
        self._port     = port
        self._hz       = hz
        self._ser      = None
        self._lock     = threading.Lock()
        self._running  = False
        self._handler  = _InboundParser(robot, self)
        self._wifi_ip, self._hostname = _get_ips()
        self._ip_refresh = 0.0

    def start(self):
        self._running = True
        threading.Thread(target=self._connect_loop, daemon=True,
                         name='display_conn').start()
        threading.Thread(target=self._send_loop,    daemon=True,
                         name='display_tx').start()
        threading.Thread(target=self._recv_loop,    daemon=True,
                         name='display_rx').start()

    def stop(self):
        self._running = False
        with self._lock:
            if self._ser:
                try:
                    self._ser.close()
                except Exception:
                    pass
                self._ser = None

    # ── Connection management ─────────────────────────────────────────────────

    def _resolve_port(self) -> str:
        if self._port != 'auto':
            return self._port
        if os.path.exists(self.DISPLAY_PORT):
            return self.DISPLAY_PORT
        for p in self.FALLBACK_PORTS:
            if os.path.exists(p):
                return p
        return self.FALLBACK_PORTS[0]

    def _connect_loop(self):
        import serial
        while self._running:
            port = self._resolve_port()
            try:
                ser = serial.Serial(port, self.BAUD, timeout=0.1,
                                    write_timeout=0.5, dsrdtr=False)
                log.info('[display] connected on %s', port)
                with self._lock:
                    self._ser = ser
                while self._running:
                    time.sleep(0.5)
                    if not ser.isOpen():
                        break
            except Exception as e:
                log.debug('[display] %s, retrying in 2 s', e)
                with self._lock:
                    self._ser = None
                time.sleep(2)

    # ── Send loop ─────────────────────────────────────────────────────────────

    def _send(self, packet: bytes):
        with self._lock:
            if self._ser and self._ser.isOpen():
                try:
                    self._ser.write(packet)
                except Exception:
                    self._ser = None

    def _send_loop(self):
        """Cycle through all packet types at the configured rate."""
        interval = 1.0 / self._hz
        # Stagger packet types so they don't all land in the same millisecond
        types   = [PKT_DRIVE, PKT_GPS, PKT_TAGNAV, PKT_INFO, PKT_CAMERA, PKT_FAULTS]
        n       = len(types)
        tick    = 0

        while self._running:
            t0 = time.monotonic()

            if self._robot is not None:
                try:
                    state = self._robot.get_state()
                except Exception:
                    state = None

                if state is not None:
                    ptype = types[tick % n]
                    try:
                        if ptype == PKT_DRIVE:
                            self._send(build_drive(state))
                        elif ptype == PKT_GPS:
                            self._send(build_gps(state))
                        elif ptype == PKT_TAGNAV:
                            self._send(build_tagnav(state))
                        elif ptype == PKT_INFO:
                            # Refresh IPs every 30 s
                            if time.monotonic() - self._ip_refresh > 30:
                                self._wifi_ip, self._hostname = _get_ips()
                                self._ip_refresh = time.monotonic()
                            self._send(build_info(state, self._wifi_ip, self._hostname))
                        elif ptype == PKT_CAMERA:
                            self._send(build_camera(state))
                        elif ptype == PKT_FAULTS:
                            self._send(build_faults(state))
                    except Exception as e:
                        log.debug('[display] build error: %s', e)

            tick += 1
            elapsed = time.monotonic() - t0
            time.sleep(max(0.0, interval - elapsed))

    # ── Receive loop ──────────────────────────────────────────────────────────

    def _recv_loop(self):
        """Parse inbound command packets from the display."""
        buf = bytearray()
        while self._running:
            with self._lock:
                ser = self._ser
            if ser is None or not ser.isOpen():
                time.sleep(0.1)
                buf.clear()
                continue
            try:
                data = ser.read(64)
            except Exception:
                time.sleep(0.05)
                continue
            if not data:
                continue
            buf += data
            while len(buf) >= 5:
                idx = buf.find(HEADER)
                if idx < 0:
                    buf.clear()
                    break
                if idx > 0:
                    del buf[:idx]
                if len(buf) < 4:
                    break
                ptype = buf[2]
                plen  = buf[3]
                needed = 4 + plen + 1
                if len(buf) < needed:
                    break
                payload = bytes(buf[4:4 + plen])
                crc     = buf[4 + plen]
                if _crc8(bytes(buf[2:4 + plen])) == crc:
                    self._handler.handle(ptype, payload)
                del buf[:needed]


# ── CLI test harness ──────────────────────────────────────────────────────────

def _test_mode():
    """Send canned telemetry without a real Robot, for bench testing."""
    import serial, math

    port = DisplayBridge.FALLBACK_PORTS[0]
    for p in [DisplayBridge.DISPLAY_PORT] + DisplayBridge.FALLBACK_PORTS:
        if os.path.exists(p):
            port = p
            break

    print(f'Test mode → {port}')
    ser = serial.Serial(port, DisplayBridge.BAUD, timeout=0.1, write_timeout=0.5)
    t = 0.0

    # Fake state objects
    class Obj:
        def __init__(self, **kw):
            for k, v in kw.items():
                setattr(self, k, v)

    from enum import Enum, auto
    class RobotMode(Enum):
        MANUAL = auto(); AUTO = auto(); ESTOP = auto()

    while True:
        t += 0.1
        hdg = (t * 10) % 360

        telem = Obj(
            voltage=7.6, current=6.5,
            fl_current=1.8, fr_current=1.7, rl_current=1.8, rr_current=1.7,
            fl_temp=38.0,   fr_temp=37.5,   rl_temp=38.0,   rr_temp=37.5,
            board_temp=42.0, bench_temp=35.0,
            heading=hdg, pitch=1.0, roll=0.5,
            fl_fault=False, fr_fault=False, rl_fault=False, rr_fault=False,
            bench_fault=False, left_fault=False, right_fault=False,
        )
        gps = Obj(
            latitude=51.5074, longitude=-0.1278, altitude=42.3,
            speed=1.2, heading=hdg, fix=True, fix_quality=4,
            fix_quality_name='RTK Fixed', hdop=0.8, satellites=14,
            ntrip_status='connected', ntrip_bytes_recv=102400,
            h_error_m=0.02, satellites_view=18, timestamp=time.time(),
        )
        drive = Obj(left=0.7, right=0.65)
        system = Obj(
            cpu_percent=34.0, cpu_temp_c=52.0, cpu_freq_mhz=2400.0,
            mem_used_mb=1200.0, mem_total_mb=8192.0, mem_percent=14.6,
            disk_used_gb=12.0, disk_total_gb=64.0, disk_percent=18.7,
        )
        lidar = Obj(angles=[], distances=[], timestamp=time.time())

        state = Obj(
            mode=RobotMode.AUTO, drive=drive, telemetry=telem, gps=gps,
            system=system, lidar=lidar,
            rc_active=True, gps_ok=True, lidar_ok=True,
            camera_ok=True, aruco_ok=True, depth_ok=False, robot_det_ok=True,
            cam_front_left_ok=True, cam_front_right_ok=True, cam_rear_ok=True,
            cam_front_left_cap=False, cam_front_right_cap=False, cam_rear_cap=False,
            cam_front_left_recording=False, cam_front_right_recording=False,
            cam_rear_recording=False,
            nav_state='APPROACHING', nav_gate=7, nav_gate_label='Gate 7',
            nav_outside_tag=14, nav_inside_tag=15,
            nav_next_gate=8, nav_next_outside_tag=16, nav_next_inside_tag=17,
            nav_next_gate_label='Gate 8',
            nav_tags_visible=2,
            nav_target_dist=2.4, nav_target_bearing=12.0, nav_bearing_err=2.1,
            nav_wp=3, nav_wp_dist=15.2, nav_wp_bear=87.0,
            no_motors=False, bench_enabled=True,
            robot_count=1, speed_scale=0.6,
            gate_confirmed=False, current_gate_id=7,
            cam_recording=False, gps_logging=False, data_logging=False,
            nav_paused=False, auto_type=Obj(name='CAMERA'),
        )

        pkts = [
            build_drive(state),
            build_gps(state),
            build_tagnav(state),
            build_info(state, '192.168.1.100', 'hacky-racer'),
            build_camera(state),
            build_faults(state),
        ]
        for pkt in pkts:
            try:
                ser.write(pkt)
            except Exception as e:
                print(f'write error: {e}')
        time.sleep(0.1)


if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG,
                        format='%(asctime)s %(levelname)s %(message)s')
    ap = argparse.ArgumentParser()
    ap.add_argument('--test', action='store_true', help='send canned data without a Robot')
    args = ap.parse_args()
    if args.test:
        _test_mode()
    else:
        print('Import DisplayBridge and pass a Robot instance, or use --test')
