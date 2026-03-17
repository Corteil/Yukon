#!/usr/bin/env python3
"""
robot.py — Unified robot model for Yukon differential-drive robot.

Subsystems
----------
  Drive    : Yukon USB serial (CMD_LEFT / CMD_RIGHT / CMD_KILL / CMD_SENSOR)
  RC       : FlySky iBUS manual control + mode switch
  Telemetry: Yukon voltage, current, temps, faults  (1 Hz)
  Camera   : picamera2 (Pi camera) or OpenCV fallback
  Lidar    : RPLidar via rplidar library
  GPS      : gpsd or raw NMEA serial fallback

Mode switching (RC channel 6, SwB)
-----------------------------------
  CH6 ≤ 1500  →  MANUAL  (RC tank-mix drives motors directly at 50 Hz)
  CH6 > 1500  →  AUTO    (call robot.drive(left, right) from your code)
  Fault/ESTOP →  ESTOP   (motors killed; call robot.reset_estop() to recover)

Quick start
-----------
  robot = Robot()
  robot.start()
  robot.set_mode(RobotMode.AUTO)
  robot.drive(0.5, 0.5)          # forward half speed
  state = robot.get_state()
  frame = robot.get_frame()      # latest camera numpy array or None
  robot.stop()

Command-line demo
-----------------
  python3 robot.py               # print state at 2 Hz until Ctrl+C
  python3 robot.py --no-camera --no-lidar --no-gps
"""

import csv
import logging
import os
import queue
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum, auto
from typing import List, Optional

log = logging.getLogger(__name__)


# ──────────────────────────────────────────────────────────────────────────────
# Mode
# ──────────────────────────────────────────────────────────────────────────────

class RobotMode(Enum):
    MANUAL = auto()   # RC receiver drives motors
    AUTO   = auto()   # Autonomous code drives motors
    ESTOP  = auto()   # Motors killed; awaiting reset


class AutoType(Enum):
    CAMERA     = auto()   # Camera-guided autonomous
    GPS        = auto()   # GPS-guided autonomous
    CAMERA_GPS = auto()   # Camera + GPS autonomous

    @property
    def label(self) -> str:
        return {"CAMERA": "Camera", "GPS": "GPS", "CAMERA_GPS": "Cam+GPS"}[self.name]


# ──────────────────────────────────────────────────────────────────────────────
# State dataclasses
# ──────────────────────────────────────────────────────────────────────────────

@dataclass
class DriveState:
    left:  float = 0.0   # -1.0 .. +1.0
    right: float = 0.0


@dataclass
class Telemetry:
    voltage:     float = 0.0
    current:     float = 0.0
    board_temp:  float = 0.0
    left_temp:   float = 0.0
    right_temp:  float = 0.0
    left_fault:  bool  = False
    right_fault: bool  = False
    timestamp:   float = 0.0


@dataclass
class GpsState:
    latitude:        Optional[float] = None
    longitude:       Optional[float] = None
    altitude:        Optional[float] = None
    speed:           Optional[float] = None   # m/s
    heading:         Optional[float] = None   # degrees true
    fix:             bool             = False
    fix_quality:     int              = 0     # 0=none 1=GPS 2=DGPS 4=RTK Fixed 5=RTK Float
    fix_quality_name: str             = "Invalid"
    h_error_m:       Optional[float] = None  # horizontal error from GST (m)
    satellites:      Optional[int]   = None
    satellites_view: Optional[int]   = None
    hdop:            Optional[float] = None
    timestamp:       float            = 0.0


@dataclass
class LidarScan:
    angles:    List[float] = field(default_factory=list)   # degrees
    distances: List[float] = field(default_factory=list)   # mm
    timestamp: float       = 0.0


@dataclass
class SystemState:
    cpu_percent:   float = 0.0   # 0–100 %
    cpu_temp_c:    float = 0.0   # °C
    cpu_freq_mhz:  float = 0.0   # MHz
    mem_used_mb:   float = 0.0   # MB
    mem_total_mb:  float = 0.0   # MB
    mem_percent:   float = 0.0   # 0–100 %
    disk_used_gb:  float = 0.0   # GB
    disk_total_gb: float = 0.0   # GB
    disk_percent:  float = 0.0   # 0–100 %
    timestamp:     float = 0.0


@dataclass
class RobotState:
    mode:        RobotMode = RobotMode.MANUAL
    auto_type:   AutoType    = AutoType.CAMERA
    drive:       DriveState  = field(default_factory=DriveState)
    telemetry:   Telemetry   = field(default_factory=Telemetry)
    gps:         GpsState    = field(default_factory=GpsState)
    lidar:       LidarScan   = field(default_factory=LidarScan)
    system:      SystemState = field(default_factory=SystemState)
    rc_active:   bool        = False
    camera_ok:   bool        = False
    lidar_ok:    bool        = False
    gps_ok:      bool        = False
    aruco_ok:    bool        = False
    gps_logging: bool        = False
    speed_scale: float       = 0.25   # current speed limit scale (0.0–1.0)


# ──────────────────────────────────────────────────────────────────────────────
# Yukon serial link (self-contained, no dependency on rc_drive.py)
# ──────────────────────────────────────────────────────────────────────────────

class _YukonLink:
    """
    Thread-safe Yukon serial link.

    Background reader parses both ACK/NAK bytes and 5-byte sensor data packets.
    A command lock serialises every send+drain pair so concurrent callers
    (control loop + telemetry poller) can never interleave.
    """

    SYNC = 0x7E
    ACK  = 0x06
    NAK  = 0x15

    CMD_LED    = 1   # value: 0=LED_A off, 1=LED_A on
    CMD_LEFT   = 2
    CMD_RIGHT  = 3
    CMD_KILL   = 4
    CMD_SENSOR = 5

    RESP_IDS = range(7)   # 0..6

    def __init__(self, port: str, baud: int = 115200):
        import serial
        self._ser      = serial.Serial(port, baud, timeout=0.1, dsrdtr=False)
        time.sleep(0.5)
        self._ser.reset_input_buffer()
        self._cmd_lock  = threading.Lock()
        self._ack_q     = queue.Queue()
        self._sensor_q  = queue.Queue()
        self._stop      = threading.Event()
        self._rx = threading.Thread(target=self._reader, daemon=True, name="yukon_rx")
        self._rx.start()

    # ── background reader ────────────────────────────────────────────────────

    def _reader(self):
        import serial as _serial
        pkt   = []
        in_pkt = False
        s_buf  = []   # accumulate (resp_id, value) tuples for one sensor query

        while not self._stop.is_set():
            try:
                data = self._ser.read(1)
            except _serial.SerialException:
                break
            if not data:
                continue
            b = data[0]

            if b == self.ACK:
                if s_buf:
                    self._sensor_q.put(self._parse_sensor(s_buf))
                    s_buf = []
                self._ack_q.put(True)
                in_pkt = False

            elif b == self.NAK:
                s_buf  = []
                in_pkt = False
                self._ack_q.put(False)

            elif b == self.SYNC:
                pkt    = [b]
                in_pkt = True

            elif in_pkt:
                pkt.append(b)
                if len(pkt) == 5:
                    in_pkt = False
                    rtype, v_high, v_low, chk = pkt[1], pkt[2], pkt[3], pkt[4]
                    if (0x30 <= rtype <= 0x36 and
                            0x40 <= v_high <= 0x4F and
                            0x50 <= v_low  <= 0x5F and
                            chk == (rtype ^ v_high ^ v_low)):
                        resp_id = rtype - 0x30
                        value   = ((v_high - 0x40) << 4) | (v_low - 0x50)
                        s_buf.append((resp_id, value))
                    pkt = []

    @staticmethod
    def _parse_sensor(packets) -> Telemetry:
        raw = {resp_id: value for resp_id, value in packets}
        return Telemetry(
            voltage     = raw.get(0, 0) / 10.0,
            current     = raw.get(1, 0) / 100.0,
            board_temp  = raw.get(2, 0) / 3.0,
            left_temp   = raw.get(3, 0) / 3.0,
            right_temp  = raw.get(4, 0) / 3.0,
            left_fault  = bool(raw.get(5, 0)),
            right_fault = bool(raw.get(6, 0)),
            timestamp   = time.monotonic(),
        )

    # ── protocol helpers ─────────────────────────────────────────────────────

    def _encode(self, cmd_code: int, byte_value: int) -> bytes:
        cmd    = cmd_code + 0x20
        v_high = (byte_value >> 4)   + 0x40
        v_low  = (byte_value & 0x0F) + 0x50
        chk    = cmd ^ v_high ^ v_low
        return bytes([self.SYNC, cmd, v_high, v_low, chk])

    @staticmethod
    def _speed_byte(speed: float) -> int:
        speed = max(-1.0, min(1.0, speed))
        if speed >= 0.0:
            return round(speed * 100)
        return 100 + round(abs(speed) * 100)

    def _drain(self, n: int = 1, timeout: float = 0.1):
        for _ in range(n):
            try:
                self._ack_q.get(timeout=timeout)
            except queue.Empty:
                pass

    # ── public API ───────────────────────────────────────────────────────────

    def drive(self, left: float, right: float):
        """Send left+right motor speeds in one write. Thread-safe."""
        pkt = (self._encode(self.CMD_LEFT,  self._speed_byte(left)) +
               self._encode(self.CMD_RIGHT, self._speed_byte(right)))
        with self._cmd_lock:
            self._ser.write(pkt)
            self._drain(2)

    def set_led_a(self, on: bool):
        with self._cmd_lock:
            self._ser.write(self._encode(self.CMD_LED, 1 if on else 0))
            self._drain(1)

    def set_led_b(self, on: bool):
        with self._cmd_lock:
            self._ser.write(self._encode(self.CMD_LED, 3 if on else 2))
            self._drain(1)

    def kill(self):
        with self._cmd_lock:
            self._ser.write(self._encode(self.CMD_KILL, 0))
            self._drain(1, timeout=0.5)

    def query_sensor(self) -> Optional[Telemetry]:
        with self._cmd_lock:
            self._ser.write(self._encode(self.CMD_SENSOR, 0))
            self._drain(1, timeout=0.5)          # wait for ACK (arrives after data)
        try:
            return self._sensor_q.get(timeout=0.1)
        except queue.Empty:
            return None

    def close(self):
        self._stop.set()
        try:
            self.kill()
        except Exception:
            pass
        try:
            self._ser.close()
        except Exception:
            pass


# ──────────────────────────────────────────────────────────────────────────────
# Camera subsystem
# ──────────────────────────────────────────────────────────────────────────────

_CV2_ROTATIONS = {
    90:  None,   # populated after cv2 import to avoid top-level dependency
    180: None,
    270: None,
}


class _Camera:
    """Captures frames from Pi camera (picamera2) or OpenCV fallback.

    Optionally applies rotation (0/90/180/270 degrees) and runs ArUco
    marker + gate detection on every captured frame.
    """

    def __init__(self, width: int = 640, height: int = 480, fps: int = 30,
                 rotation: int = 0,
                 enable_aruco: bool = False,
                 aruco_dict:   str  = "DICT_4X4_1000"):
        self._w            = width
        self._h            = height
        self._fps          = fps
        self._rotation     = rotation
        self._aruco_dict   = aruco_dict
        self._aruco_enabled = enable_aruco   # runtime toggle flag
        self._frame        = None
        self._aruco_state  = None   # ArUcoState or None
        self._lock         = threading.Lock()
        self._stop         = threading.Event()
        self._ok           = False
        self._aruco_ok     = False

    def start(self):
        threading.Thread(target=self._run, daemon=True, name="camera").start()

    def _run(self):
        try:
            self._run_picamera2()
        except Exception:
            try:
                self._run_opencv()
            except Exception as e:
                log.warning(f"Camera unavailable: {e}")

    def _rotate(self, frame, cv2):
        """Apply configured rotation using cv2.rotate (no-op if 0)."""
        rot_map = {
            90:  cv2.ROTATE_90_CLOCKWISE,
            180: cv2.ROTATE_180,
            270: cv2.ROTATE_90_COUNTERCLOCKWISE,
        }
        code = rot_map.get(self._rotation)
        return cv2.rotate(frame, code) if code is not None else frame

    def _make_detector(self):
        """Initialise the ArucoDetector; return None on failure."""
        try:
            from aruco_detector import ArucoDetector
            det = ArucoDetector(self._aruco_dict)
            log.info(f"ArUco detector ready ({self._aruco_dict})")
            return det
        except Exception as e:
            log.warning(f"ArUco detector init failed: {e}")
            return None

    def _run_picamera2(self):
        import cv2
        from picamera2 import Picamera2
        cam = Picamera2()
        cam.configure(cam.create_video_configuration(
            main={"size": (self._w, self._h), "format": "RGB888"}
        ))
        cam.start()
        self._ok = True
        detector = self._make_detector()
        try:
            while not self._stop.is_set():
                frame = cam.capture_array()
                frame = self._rotate(frame, cv2)
                aruco_state = None
                if detector is not None and self._aruco_enabled:
                    try:
                        aruco_state = detector.detect(frame)
                    except Exception as e:
                        log.debug(f"ArUco detect error: {e}")
                with self._lock:
                    self._frame       = frame
                    self._aruco_state = aruco_state
                    self._aruco_ok    = aruco_state is not None
                time.sleep(1.0 / self._fps)
        finally:
            cam.stop()
            self._ok       = False
            self._aruco_ok = False

    def _run_opencv(self):
        import cv2
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self._w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._h)
        cap.set(cv2.CAP_PROP_FPS,          self._fps)
        if not cap.isOpened():
            raise RuntimeError("No camera found")
        self._ok = True
        detector = self._make_detector()
        try:
            while not self._stop.is_set():
                ret, frame = cap.read()
                if ret:
                    frame = frame[:, :, ::-1]   # BGR → RGB
                    frame = self._rotate(frame, cv2)
                    aruco_state = None
                    if detector is not None and self._aruco_enabled:
                        try:
                            aruco_state = detector.detect(frame)
                        except Exception as e:
                            log.debug(f"ArUco detect error: {e}")
                    with self._lock:
                        self._frame       = frame
                        self._aruco_state = aruco_state
                        self._aruco_ok    = aruco_state is not None
        finally:
            cap.release()
            self._ok       = False
            self._aruco_ok = False

    def get_frame(self):
        """Return latest camera frame (numpy array, RGB) or None."""
        with self._lock:
            return self._frame

    def get_aruco_state(self):
        """Return latest :class:`~aruco_detector.ArUcoState` or None."""
        with self._lock:
            return self._aruco_state

    @property
    def ok(self):
        return self._ok

    @property
    def aruco_ok(self):
        return self._aruco_ok

    def set_aruco_enabled(self, enabled: bool):
        """Enable or disable ArUco detection at runtime."""
        self._aruco_enabled = enabled
        if not enabled:
            with self._lock:
                self._aruco_state = None
                self._aruco_ok    = False

    def toggle_aruco(self) -> bool:
        """Toggle ArUco detection on/off. Returns the new enabled state."""
        self.set_aruco_enabled(not self._aruco_enabled)
        return self._aruco_enabled

    def get_aruco_enabled(self) -> bool:
        return self._aruco_enabled

    def set_rotation(self, rotation: int):
        """Change rotation at runtime (0, 90, 180, 270). Takes effect on next frame."""
        self._rotation = rotation % 360

    def stop(self):
        self._stop.set()


# ──────────────────────────────────────────────────────────────────────────────
# Lidar subsystem  (LDROBOT LD06)
# ──────────────────────────────────────────────────────────────────────────────

class _Lidar:
    """Thin wrapper around LD06 that matches the rest of robot.py's subsystem API."""

    def __init__(self, port: str = '/dev/ttyUSB0'):
        from ld06 import LD06
        self._driver = LD06(port)

    def start(self):
        self._driver.start()

    def stop(self):
        self._driver.stop()

    @property
    def ok(self) -> bool:
        return self._driver.ok

    def get_scan(self) -> LidarScan:
        s = self._driver.get_scan()
        return LidarScan(angles=s.angles, distances=s.distances)

    @property
    def ok(self):
        return self._ok

    def stop(self):
        self._stop.set()


# ──────────────────────────────────────────────────────────────────────────────
# GPS subsystem  (Allystar TAU1308 RTK module via tau1308.py)
# ──────────────────────────────────────────────────────────────────────────────

class _Gps:
    """
    Background reader for the Allystar TAU1308 RTK GNSS module.

    RTK corrections can be supplied via:
      - Serial port  (rtcm_port): a local base station connected by USB/UART
      - NTRIP        (ntrip_host + ntrip_mount): network caster
    Serial takes priority when both are configured.
    """

    def __init__(self, port: str = '/dev/ttyUSB0', baud: int = 115200,
                 ntrip_host: str = '', ntrip_port: int = 2101,
                 ntrip_mount: str = '', ntrip_user: str = '',
                 ntrip_password: str = '',
                 rtcm_port: str = '', rtcm_baud: int = 115200):
        self._port           = port
        self._baud           = baud
        self._ntrip_host     = ntrip_host
        self._ntrip_port     = ntrip_port
        self._ntrip_mount    = ntrip_mount
        self._ntrip_user     = ntrip_user
        self._ntrip_password = ntrip_password
        self._rtcm_port      = rtcm_port
        self._rtcm_baud      = rtcm_baud
        self._state          = GpsState()
        self._lock           = threading.Lock()
        self._stop           = threading.Event()
        self._ok             = False

    def start(self):
        threading.Thread(target=self._run, daemon=True, name="gps").start()

    def _run(self):
        try:
            import serial
            from gnss import TAU1308, NTRIPClient, RtcmSerial
        except ImportError as e:
            log.warning(f"GPS unavailable: {e}")
            return

        try:
            ser  = serial.Serial(self._port, self._baud, timeout=1)
            gnss = TAU1308(ser)
            self._ok = True
            log.info(f"TAU1308 GPS opened on {self._port}")
        except Exception as e:
            log.warning(f"GPS serial error: {e}")
            return

        correction_src = None
        if self._rtcm_port:
            correction_src = RtcmSerial(self._rtcm_port, baud=self._rtcm_baud)
            correction_src.start(gnss)
            log.info(f"RTCM serial corrections → {self._rtcm_port} @ {self._rtcm_baud}")
        elif self._ntrip_host and self._ntrip_mount:
            correction_src = NTRIPClient(
                host       = self._ntrip_host,
                port       = self._ntrip_port,
                mountpoint = self._ntrip_mount,
                username   = self._ntrip_user,
                password   = self._ntrip_password,
            )
            correction_src.start(gnss)
            log.info(f"NTRIP corrections → {self._ntrip_host}/{self._ntrip_mount}")

        try:
            while not self._stop.is_set():
                gnss.update(max_sentences=10)
                with self._lock:
                    self._state = GpsState(
                        latitude         = gnss.latitude,
                        longitude        = gnss.longitude,
                        altitude         = gnss.altitude,
                        speed            = gnss.speed_ms,
                        heading          = gnss.course,
                        fix              = gnss.has_fix,
                        fix_quality      = gnss.fix_quality,
                        fix_quality_name = gnss.fix_quality_name,
                        h_error_m        = gnss.h_error_m,
                        satellites       = gnss.satellites,
                        satellites_view  = gnss.satellites_in_view,
                        hdop             = gnss.hdop,
                        timestamp        = time.monotonic(),
                    )
                self._stop.wait(0.05)   # ~20 Hz poll, TAU1308 outputs up to 10 Hz
        finally:
            if correction_src:
                correction_src.stop()
            try:
                ser.close()
            except Exception:
                pass
            self._ok = False

    def get_state(self) -> GpsState:
        with self._lock:
            return self._state

    @property
    def ok(self):
        return self._ok

    def stop(self):
        self._stop.set()


# ──────────────────────────────────────────────────────────────────────────────
# System monitor
# ──────────────────────────────────────────────────────────────────────────────

class _System:
    """
    Polls Raspberry Pi system metrics at 2 Hz.
    Uses /proc and /sys directly — no psutil dependency.
    """

    def __init__(self):
        self._lock  = threading.Lock()
        self._state = SystemState()
        self._stop  = threading.Event()

    def start(self):
        threading.Thread(target=self._run, daemon=True, name="system").start()

    def _run(self):
        import shutil
        prev_total = prev_idle = 0

        while not self._stop.is_set():
            try:
                # CPU % (derived from /proc/stat delta)
                total, idle = self._stat()
                dt = total - prev_total
                di = idle  - prev_idle
                cpu_pct = max(0.0, min(100.0, 100.0 * (1.0 - di / dt))) if dt else 0.0
                prev_total, prev_idle = total, idle

                # CPU temperature
                try:
                    with open('/sys/class/thermal/thermal_zone0/temp') as f:
                        cpu_temp = int(f.read()) / 1000.0
                except OSError:
                    cpu_temp = 0.0

                # CPU frequency
                try:
                    with open('/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq') as f:
                        cpu_freq = int(f.read()) / 1000.0
                except OSError:
                    cpu_freq = 0.0

                # Memory
                mem       = self._meminfo()
                mem_total = mem.get('MemTotal', 0) / 1024.0
                mem_avail = mem.get('MemAvailable', mem.get('MemFree', 0)) / 1024.0
                mem_used  = mem_total - mem_avail
                mem_pct   = 100.0 * mem_used / mem_total if mem_total else 0.0

                # Disk
                du         = shutil.disk_usage('/')
                disk_total = du.total / 1e9
                disk_used  = du.used  / 1e9
                disk_pct   = 100.0 * du.used / du.total if du.total else 0.0

                with self._lock:
                    self._state = SystemState(
                        cpu_percent   = round(cpu_pct,  1),
                        cpu_temp_c    = round(cpu_temp, 1),
                        cpu_freq_mhz  = round(cpu_freq, 0),
                        mem_used_mb   = round(mem_used,  0),
                        mem_total_mb  = round(mem_total, 0),
                        mem_percent   = round(mem_pct,  1),
                        disk_used_gb  = round(disk_used,  2),
                        disk_total_gb = round(disk_total, 2),
                        disk_percent  = round(disk_pct,  1),
                        timestamp     = time.monotonic(),
                    )
            except Exception as e:
                log.debug(f"System poll error: {e}")

            self._stop.wait(0.5)   # 2 Hz

    @staticmethod
    def _stat():
        with open('/proc/stat') as f:
            parts = f.readline().split()
        vals  = list(map(int, parts[1:]))
        idle  = vals[3] + (vals[4] if len(vals) > 4 else 0)   # idle + iowait
        return sum(vals), idle

    @staticmethod
    def _meminfo():
        mem = {}
        with open('/proc/meminfo') as f:
            for line in f:
                key, _, rest = line.partition(':')
                try:
                    mem[key.strip()] = int(rest.strip().split()[0])
                except (ValueError, IndexError):
                    pass
        return mem

    def get_state(self) -> SystemState:
        with self._lock:
            return self._state

    def stop(self):
        self._stop.set()


# ──────────────────────────────────────────────────────────────────────────────
# Robot
# ──────────────────────────────────────────────────────────────────────────────

# RC channel indices (0-based)
_CH_THROTTLE  = 2    # CH3
_CH_STEER     = 0    # CH1
_CH_MODE      = 4    # CH5 — SwA: ≤1500 = MANUAL, >1500 = AUTO
_CH_SPEED     = 5    # CH6 — SwB: 1000=slow, 1500=mid, 2000=max
_CH_AUTO_TYPE = 6    # CH7 — SwC: 1000=Camera, 1500=GPS, 2000=Cam+GPS
_CH_GPS_LOG   = 7    # CH8 — SwD: ≤1500=logging off, >1500=logging on

_SPEED_MIN    = 0.25  # scale factor at CH6=1000

_DEADZONE        = 30     # µs either side of mid treated as zero
_FAILSAFE_S      = 0.5    # seconds without iBUS packet → failsafe in AUTO
_CONTROL_HZ      = 50
_TELEMETRY_HZ    = 1


def _auto_type_from_ch(ch_val: int) -> AutoType:
    """Map a 3-position switch (1000/1500/2000) to an AutoType."""
    if ch_val < 1334:
        return AutoType.CAMERA
    elif ch_val < 1667:
        return AutoType.GPS
    else:
        return AutoType.CAMERA_GPS


def _normalize(ch_val: int, deadzone: int) -> float:
    raw = ch_val - 1500
    if abs(raw) < deadzone:
        return 0.0
    return max(-1.0, min(1.0, raw / 500.0))


def _speed_scale(ch_val: int, speed_min: float) -> float:
    """Map speed channel (1000-2000) linearly to speed_min..1.0."""
    t = max(0.0, min(1.0, (ch_val - 1000) / 1000.0))
    return speed_min + t * (1.0 - speed_min)


def _tank_mix(throttle_raw: int, steer_raw: int, deadzone: int):
    thr = _normalize(throttle_raw, deadzone)
    ste = _normalize(steer_raw,    deadzone)
    return (
        max(-1.0, min(1.0, thr - ste)),   # left
        max(-1.0, min(1.0, thr + ste)),   # right
    )


class Robot:
    """
    Top-level robot controller.

    Parameters
    ----------
    yukon_port     : Yukon USB serial device (auto-detected if None)
    ibus_port      : iBUS UART device
    lidar_port     : LD06 serial device
    gps_port       : TAU1308 GPS serial device
    ntrip_host     : NTRIP caster hostname (e.g. 'www.rtk2go.com'); leave blank to disable
    ntrip_port     : NTRIP TCP port (default 2101)
    ntrip_mount    : NTRIP mountpoint name
    ntrip_user     : NTRIP username
    ntrip_password : NTRIP password
    enable_camera / enable_lidar / enable_gps : toggle optional subsystems
    throttle_ch    : RC throttle channel, 1-based (default 3)
    steer_ch       : RC steering channel, 1-based (default 1)
    mode_ch        : RC mode-switch channel, 1-based (default 5 = SwA)
    speed_ch       : RC speed-limit channel, 1-based (default 6 = SwB)
    deadzone       : RC stick deadzone in µs either side of mid (default 30)
    failsafe_s     : Seconds without iBUS packet before failsafe (default 0.5)
    speed_min      : Minimum speed scale when speed_ch is at low end (default 0.25)
    control_hz     : Motor control loop rate in Hz (default 50)
    """

    def __init__(
        self,
        yukon_port:     Optional[str] = None,
        ibus_port:      str   = '/dev/ttyAMA3',
        lidar_port:     str   = '/dev/ttyUSB0',
        gps_port:       str   = '/dev/ttyUSB0',
        ntrip_host:     str   = '',
        ntrip_port:     int   = 2101,
        ntrip_mount:    str   = '',
        ntrip_user:     str   = '',
        ntrip_password: str   = '',
        rtcm_port:      str   = '',
        rtcm_baud:      int   = 115200,
        enable_camera:  bool  = True,
        enable_lidar:   bool  = True,
        enable_gps:     bool  = True,
        cam_width:      int   = 640,
        cam_height:     int   = 480,
        cam_fps:        int   = 30,
        cam_rotation:   int   = 0,
        enable_aruco:   bool  = False,
        aruco_dict:     str   = "DICT_4X4_1000",
        throttle_ch:    int   = 3,
        steer_ch:       int   = 1,
        mode_ch:        int   = 5,
        speed_ch:       int   = 6,
        auto_type_ch:   int   = 7,
        gps_log_ch:      int   = 8,
        gps_bookmark_ch: int   = 2,
        gps_log_dir:     str   = '',
        gps_log_hz:      float = 5.0,
        deadzone:       int   = 30,
        failsafe_s:     float = 0.5,
        speed_min:      float = 0.25,
        control_hz:     int   = 50,
    ):
        self._ibus_port      = ibus_port
        self._yukon_port     = yukon_port or self._find_yukon()
        self._lidar_port     = lidar_port
        self._gps_port       = gps_port
        self._ntrip_host     = ntrip_host
        self._ntrip_port     = ntrip_port
        self._ntrip_mount    = ntrip_mount
        self._ntrip_user     = ntrip_user
        self._ntrip_password = ntrip_password
        self._rtcm_port      = rtcm_port
        self._rtcm_baud      = rtcm_baud
        self._enable         = dict(camera=enable_camera, lidar=enable_lidar, gps=enable_gps)
        self._cam_width      = cam_width
        self._cam_height     = cam_height
        self._cam_fps        = cam_fps
        self._cam_rotation   = cam_rotation
        self._enable_aruco   = enable_aruco
        self._aruco_dict     = aruco_dict
        # RC tuning (convert to 0-based channel indices)
        self._ch_throttle   = throttle_ch   - 1
        self._ch_steer      = steer_ch      - 1
        self._ch_mode       = mode_ch       - 1
        self._ch_speed      = speed_ch      - 1
        self._ch_auto_type  = auto_type_ch  - 1
        self._ch_gps_log      = gps_log_ch      - 1
        self._ch_gps_bookmark = gps_bookmark_ch - 1 if gps_bookmark_ch > 0 else None
        self._gps_log_dir   = (gps_log_dir.strip() or
                               os.path.join(os.path.dirname(os.path.abspath(__file__)), 'logs'))
        self._gps_log_interval = 1.0 / max(gps_log_hz, 0.1)
        self._deadzone    = deadzone
        self._failsafe_s  = failsafe_s
        self._speed_min   = speed_min
        self._control_hz  = control_hz

        # Subsystems (created in start())
        self._yukon:  Optional[_YukonLink] = None
        self._camera: Optional[_Camera]    = None
        self._lidar:  Optional[_Lidar]     = None
        self._gps:    Optional[_Gps]       = None
        self._system: _System              = _System()

        # Shared state
        self._mode_lock   = threading.Lock()
        self._mode        = RobotMode.MANUAL
        self._auto_type   = AutoType.CAMERA
        self._gps_logging       = False
        self._prev_bookmark_ch  = 0
        self._auto_left   = 0.0
        self._auto_right  = 0.0
        self._telemetry   = Telemetry()
        self._drive_state = DriveState()
        self._rc_channels  = [1500] * 14
        self._rc_ts        = 0.0     # monotonic time of last good iBUS packet
        self._rc_active    = False
        self._speed_scale  = 0.25

        self._stop_evt         = threading.Event()
        self._gps_bookmark_evt = threading.Event()

    # ── lifecycle ────────────────────────────────────────────────────────────

    def start(self):
        """Start all subsystems and control threads."""
        log.info(f"Connecting to Yukon on {self._yukon_port}")
        self._yukon = _YukonLink(self._yukon_port)

        if self._enable['camera']:
            self._camera = _Camera(
                width        = self._cam_width,
                height       = self._cam_height,
                fps          = self._cam_fps,
                rotation     = self._cam_rotation,
                enable_aruco = self._enable_aruco,
                aruco_dict   = self._aruco_dict,
            )
            self._camera.start()

        if self._enable['lidar']:
            self._lidar = _Lidar(self._lidar_port)
            self._lidar.start()

        if self._enable['gps']:
            self._gps = _Gps(
                port           = self._gps_port,
                ntrip_host     = self._ntrip_host,
                ntrip_port     = self._ntrip_port,
                ntrip_mount    = self._ntrip_mount,
                ntrip_user     = self._ntrip_user,
                ntrip_password = self._ntrip_password,
                rtcm_port      = self._rtcm_port,
                rtcm_baud      = self._rtcm_baud,
            )
            self._gps.start()

        self._system.start()

        threading.Thread(target=self._rc_thread,        daemon=True, name="rc_reader").start()
        threading.Thread(target=self._telemetry_thread, daemon=True, name="telemetry").start()
        threading.Thread(target=self._control_thread,   daemon=True, name="control").start()
        threading.Thread(target=self._gps_log_thread,   daemon=True, name="gps_log").start()

        log.info("Robot started.")

    def stop(self):
        """Kill motors and shut down all subsystems."""
        self._stop_evt.set()
        if self._yukon:
            self._yukon.close()
        for sub in (self._camera, self._lidar, self._gps, self._system):
            if sub:
                sub.stop()
        log.info("Robot stopped.")

    # ── mode API ─────────────────────────────────────────────────────────────

    def set_mode(self, mode: RobotMode):
        """Switch to MANUAL or AUTO. Cannot set ESTOP directly."""
        if mode is RobotMode.ESTOP:
            raise ValueError("Use reset_estop() to clear ESTOP")
        with self._mode_lock:
            self._mode = mode
        log.info(f"Mode → {mode.name}")

    def estop(self):
        """Immediately kill motors and enter ESTOP."""
        with self._mode_lock:
            self._mode = RobotMode.ESTOP
        if self._yukon:
            self._yukon.kill()
        log.warning("ESTOP triggered manually")

    def reset_estop(self):
        """Clear ESTOP and return to MANUAL."""
        with self._mode_lock:
            self._mode = RobotMode.MANUAL
        log.info("ESTOP cleared → MANUAL")

    # ── drive API (autonomous mode) ──────────────────────────────────────────

    def drive(self, left: float, right: float):
        """
        Set motor speeds for AUTO mode.
        left/right: -1.0 (full reverse) .. +1.0 (full forward)
        Has no effect in MANUAL or ESTOP.
        """
        with self._mode_lock:
            self._auto_left  = max(-1.0, min(1.0, left))
            self._auto_right = max(-1.0, min(1.0, right))

    # ── state / sensor access ─────────────────────────────────────────────────

    def get_state(self) -> RobotState:
        with self._mode_lock:
            mode      = self._mode
            auto_type = self._auto_type
            drive     = DriveState(self._drive_state.left, self._drive_state.right)
            telem     = self._telemetry
        return RobotState(
            mode      = mode,
            auto_type = auto_type,
            drive     = drive,
            telemetry = telem,
            gps       = self._gps.get_state()    if self._gps   else GpsState(),
            lidar     = self._lidar.get_scan()   if self._lidar else LidarScan(),
            system    = self._system.get_state(),
            rc_active   = self._rc_active,
            speed_scale = self._speed_scale,
            camera_ok = bool(self._camera and self._camera.ok),
            lidar_ok  = bool(self._lidar  and self._lidar.ok),
            gps_ok    = bool(self._gps    and self._gps.ok),
            aruco_ok    = bool(self._camera and self._camera.aruco_ok),
            gps_logging = self._gps_logging,
        )

    def get_frame(self):
        """Return latest camera frame (numpy array, RGB) or None."""
        return self._camera.get_frame() if self._camera else None

    def get_aruco_state(self):
        """Return latest :class:`~aruco_detector.ArUcoState` or None."""
        return self._camera.get_aruco_state() if self._camera else None

    def set_aruco_enabled(self, enabled: bool):
        """Enable or disable ArUco detection at runtime."""
        if self._camera:
            self._camera.set_aruco_enabled(enabled)

    def toggle_aruco(self) -> bool:
        """Toggle ArUco on/off. Returns the new enabled state."""
        return self._camera.toggle_aruco() if self._camera else False

    def get_aruco_enabled(self) -> bool:
        """Return True if ArUco detection is currently enabled."""
        return bool(self._camera and self._camera.get_aruco_enabled())

    def set_cam_rotation(self, rotation: int):
        """Change camera rotation at runtime (0, 90, 180, 270)."""
        self._cam_rotation = rotation % 360
        if self._camera:
            self._camera.set_rotation(rotation)

    def get_cam_rotation(self) -> int:
        """Return the current camera rotation in degrees."""
        return self._camera._rotation if self._camera else self._cam_rotation

    def get_auto_type(self) -> AutoType:
        """Return the current AutoType selected by SwC."""
        with self._mode_lock:
            return self._auto_type

    def bookmark_gps(self):
        """Insert a bookmark row into the active GPS log (no-op if not logging)."""
        if self._gps_logging:
            self._gps_bookmark_evt.set()

    # ── internal threads ─────────────────────────────────────────────────────

    def _rc_thread(self):
        """Read iBUS packets and update RC state."""
        from ibus import IBusReader, IBusError
        while not self._stop_evt.is_set():
            try:
                with IBusReader(self._ibus_port) as ibus:
                    log.info("iBUS connected.")
                    while not self._stop_evt.is_set():
                        channels = ibus.read()
                        if channels:
                            self._rc_channels = channels
                            self._rc_ts       = time.monotonic()
                            self._rc_active   = True
                        else:
                            age = time.monotonic() - self._rc_ts
                            self._rc_active = age < self._failsafe_s
            except IBusError as e:
                log.warning(f"iBUS error: {e} — retrying in 2 s")
                time.sleep(2.0)

    def _telemetry_thread(self):
        """Poll Yukon sensors and trigger ESTOP on fault."""
        interval = 1.0 / _TELEMETRY_HZ   # sensor poll is fixed at 1 Hz
        while not self._stop_evt.is_set():
            if self._yukon:
                result = self._yukon.query_sensor()
                if result:
                    with self._mode_lock:
                        self._telemetry = result
                        if result.left_fault or result.right_fault:
                            if self._mode is not RobotMode.ESTOP:
                                log.warning("Motor fault detected — ESTOP")
                                self._mode = RobotMode.ESTOP
            self._stop_evt.wait(interval)

    def _control_thread(self):
        """Control loop: apply motor speeds based on current mode."""
        dt        = 1.0 / self._control_hz
        last_left  = None
        last_right = None
        last_mode  = None
        last_led_b = None   # track last LED B state to avoid redundant serial writes

        while not self._stop_evt.is_set():
            t0 = time.monotonic()

            with self._mode_lock:
                mode       = self._mode
                auto_left  = self._auto_left
                auto_right = self._auto_right
                self._auto_type = _auto_type_from_ch(self._rc_channels[self._ch_auto_type])

            # SwD: GPS logging on/off
            gps_log_want = self._rc_channels[self._ch_gps_log] > 1500
            if gps_log_want != self._gps_logging:
                self._gps_logging = gps_log_want
                log.info(f"GPS logging {'ON' if gps_log_want else 'OFF'}")

            # RC bookmark button: rising edge (low→high) adds bookmark to GPS log
            if self._ch_gps_bookmark is not None:
                bm_val = self._rc_channels[self._ch_gps_bookmark]
                if bm_val > 1500 and self._prev_bookmark_ch <= 1500:
                    self.bookmark_gps()
                self._prev_bookmark_ch = bm_val

            # Auto-enable ArUco when AUTO + Camera or Cam+GPS; disable otherwise
            if self._camera:
                want_aruco = (mode is RobotMode.AUTO and
                              self._auto_type in (AutoType.CAMERA, AutoType.CAMERA_GPS))
                if self._camera.get_aruco_enabled() != want_aruco:
                    self._camera.set_aruco_enabled(want_aruco)
                    log.info(f"ArUco {'ON' if want_aruco else 'OFF'} "
                             f"(mode={mode.name} type={self._auto_type.label})")

            if self._yukon:
                # LED A: on = AUTO, off = MANUAL/ESTOP
                if mode is not last_mode:
                    self._yukon.set_led_a(mode is RobotMode.AUTO)
                    last_mode = mode

                # LED B: GPS status
                #   off        = no GPS / no fix
                #   slow flash = GPS fix (quality 1-3)  ~1 Hz
                #   fast flash = RTK Float (quality 5)  ~4 Hz
                #   on         = RTK Fixed (quality 4)
                gps_q = self._gps.get_state().fix_quality if self._gps and self._gps.ok else 0
                now   = t0  # time.monotonic() already captured at loop start
                if gps_q == 0:
                    led_b = False
                elif gps_q == 4:                      # RTK Fixed → solid on
                    led_b = True
                elif gps_q == 5:                      # RTK Float → fast flash ~4 Hz
                    led_b = int(now * 8) % 2 == 0
                else:                                 # GPS/DGPS/PPS → slow flash ~1 Hz
                    led_b = int(now * 2) % 2 == 0
                if led_b != last_led_b:
                    self._yukon.set_led_b(led_b)
                    last_led_b = led_b

            if mode is RobotMode.ESTOP:
                left, right = 0.0, 0.0
                if last_left != 0.0 or last_right != 0.0:
                    self._yukon.kill()
                    last_left = last_right = 0.0
                # Skip normal drive() call
                elapsed = time.monotonic() - t0
                self._stop_evt.wait(max(0.0, dt - elapsed))
                continue

            elif mode is RobotMode.MANUAL:
                ch = self._rc_channels
                # Failsafe: no RC signal → stop
                if not self._rc_active:
                    left, right = 0.0, 0.0
                else:
                    left, right = _tank_mix(ch[self._ch_throttle], ch[self._ch_steer], self._deadzone)
                    # Mode switch from transmitter
                    if ch[self._ch_mode] > 1500:
                        with self._mode_lock:
                            self._mode = RobotMode.AUTO
                        log.info("RC → AUTO")

            else:  # AUTO
                left, right = auto_left, auto_right
                # RC failsafe in AUTO: if signal lost, ESTOP
                if not self._rc_active:
                    with self._mode_lock:
                        self._mode = RobotMode.ESTOP
                    log.warning("RC signal lost in AUTO — ESTOP")
                    continue
                # RC override: switch back to manual
                if self._rc_channels[self._ch_mode] <= 1500:
                    with self._mode_lock:
                        self._mode = RobotMode.MANUAL
                    log.info("RC → MANUAL")

            # Apply speed limit from SwB
            scale              = _speed_scale(self._rc_channels[self._ch_speed], self._speed_min)
            self._speed_scale  = scale
            left               = left  * scale
            right              = right * scale

            # Only send if values changed
            left_b  = round(left  * 100)
            right_b = round(right * 100)
            if left_b != last_left or right_b != last_right:
                self._yukon.drive(left, right)
                last_left, last_right = left_b, right_b

            with self._mode_lock:
                self._drive_state = DriveState(left, right)

            elapsed = time.monotonic() - t0
            self._stop_evt.wait(max(0.0, dt - elapsed))

    # ── helpers ───────────────────────────────────────────────────────────────

    def _gps_log_thread(self):
        """Write GPS state to CSV while _gps_logging is True (toggled by SwD)."""
        log_file = None
        writer   = None

        while not self._stop_evt.is_set():
            if self._gps_logging:
                if log_file is None:
                    # Open a new timestamped CSV file
                    try:
                        os.makedirs(self._gps_log_dir, exist_ok=True)
                        fname    = datetime.now().strftime("gps_%Y%m%d_%H%M%S.csv")
                        path     = os.path.join(self._gps_log_dir, fname)
                        log_file = open(path, 'w', newline='')
                        writer   = csv.writer(log_file)
                        writer.writerow([
                            'timestamp', 'mode', 'auto_type',
                            'gps_ok', 'gps_mode',
                            'latitude', 'longitude', 'altitude_m',
                            'fix_quality', 'satellites', 'hdop', 'h_error_m',
                            'bookmark',
                        ])
                        log.info(f"GPS log started: {path}")
                    except OSError as e:
                        log.error(f"GPS log open failed: {e}")
                        self._stop_evt.wait(5.0)
                        continue

                # Snapshot GPS + mode (shared by normal row and any pending bookmark)
                g      = self._gps.get_state() if self._gps else GpsState()
                gps_ok = bool(self._gps and self._gps.ok and g.latitude is not None)
                with self._mode_lock:
                    mode      = self._mode
                    auto_type = self._auto_type

                def _write_row(bookmark=False):
                    writer.writerow([
                        datetime.now().isoformat(timespec='milliseconds'),
                        mode.name,
                        auto_type.label,
                        gps_ok,
                        g.fix_quality_name,
                        g.latitude,
                        g.longitude,
                        g.altitude,
                        g.fix_quality,
                        g.satellites,
                        g.hdop,
                        g.h_error_m,
                        bookmark,
                    ])
                    log_file.flush()

                # Bookmark requested — write it before the normal row
                if self._gps_bookmark_evt.is_set():
                    self._gps_bookmark_evt.clear()
                    _write_row(bookmark=True)
                    log.info("GPS log bookmark added")

                _write_row(bookmark=False)
            else:
                if log_file is not None:
                    log_file.close()
                    log.info("GPS log closed.")
                    log_file = None
                    writer   = None

            self._stop_evt.wait(self._gps_log_interval)

        if log_file is not None:
            log_file.close()

    @staticmethod
    def _find_yukon() -> str:
        try:
            import serial.tools.list_ports
            for p in serial.tools.list_ports.comports():
                if p.vid == 0x2E8A:
                    return p.device
            ports = serial.tools.list_ports.comports()
            if ports:
                return ports[0].device
        except Exception:
            pass
        return '/dev/ttyACM0'


# ──────────────────────────────────────────────────────────────────────────────
# CLI demo
# ──────────────────────────────────────────────────────────────────────────────

def _load_config(path):
    """Load robot.ini; return configparser instance (empty if file not found)."""
    import configparser
    cfg = configparser.ConfigParser(inline_comment_prefixes=('#',))
    cfg.read(path)
    return cfg


def _cfg(cfg, section, key, fallback, cast=str):
    """Read a value from config, casting to the required type."""
    try:
        raw = cfg.get(section, key).strip()
        return cast(raw) if raw else fallback
    except Exception:
        return fallback


def main():
    import argparse
    import os

    DEFAULT_CFG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "robot.ini")

    parser = argparse.ArgumentParser(description="Yukon robot",
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config",          default=DEFAULT_CFG, help="Config file (default: robot.ini)")
    # Overrides — all optional; config file values are used when not supplied
    parser.add_argument("--yukon-port",      default=None,  help="Yukon serial port")
    parser.add_argument("--ibus-port",       default=None)
    parser.add_argument("--lidar-port",      default=None)
    parser.add_argument("--gps-port",        default=None)
    parser.add_argument("--ntrip-host",      default=None)
    parser.add_argument("--ntrip-port",      default=None,  type=int)
    parser.add_argument("--ntrip-mount",     default=None)
    parser.add_argument("--ntrip-user",      default=None)
    parser.add_argument("--ntrip-password",  default=None)
    parser.add_argument("--rtcm-port",       default=None,  help="Serial port for RTCM input (overrides NTRIP)")
    parser.add_argument("--rtcm-baud",       default=None,  type=int)
    parser.add_argument("--no-camera",       action="store_true", default=None)
    parser.add_argument("--no-lidar",        action="store_true", default=None)
    parser.add_argument("--no-gps",          action="store_true", default=None)
    args = parser.parse_args()

    cfg = _load_config(args.config)

    def arg(cli_val, section, key, fallback, cast=str):
        """CLI wins → config file → built-in default."""
        if cli_val is not None:
            return cli_val
        return _cfg(cfg, section, key, fallback, cast)

    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")
    log.info(f"Config: {args.config}")

    robot = Robot(
        yukon_port     = arg(args.yukon_port,      "robot",  "yukon_port",     None,
                           lambda x: None if x.lower() in ("auto", "") else x),
        ibus_port      = arg(args.ibus_port,       "robot",  "ibus_port",      "/dev/ttyAMA3"),
        lidar_port     = arg(args.lidar_port,      "lidar",  "port",           "/dev/ttyUSB0"),
        gps_port       = arg(args.gps_port,        "gps",    "port",           "/dev/ttyUSB0"),
        ntrip_host     = "" if arg(None, "ntrip", "disabled", False, lambda x: x.lower() == "true") else arg(args.ntrip_host, "ntrip", "host", ""),
        ntrip_port     = arg(args.ntrip_port,      "ntrip",  "port",           2101,  int),
        ntrip_mount    = arg(args.ntrip_mount,     "ntrip",  "mount",          ""),
        ntrip_user     = arg(args.ntrip_user,      "ntrip",  "user",           ""),
        ntrip_password = arg(args.ntrip_password,  "ntrip",  "password",       ""),
        rtcm_port      = "" if arg(None, "rtcm", "disabled", False, lambda x: x.lower() == "true") else arg(args.rtcm_port, "rtcm", "port", ""),
        rtcm_baud      = arg(args.rtcm_baud,       "rtcm",   "baud",           115200, int),
        enable_camera  = not arg(args.no_camera,   "camera", "disabled",       False, lambda x: x.lower() == "true"),
        enable_lidar   = not arg(args.no_lidar,    "lidar",  "disabled",       False, lambda x: x.lower() == "true"),
        enable_gps     = not arg(args.no_gps,      "gps",    "disabled",       False, lambda x: x.lower() == "true"),
        cam_width      = _cfg(cfg, "camera", "width",    640,  int),
        cam_height     = _cfg(cfg, "camera", "height",  480,  int),
        cam_fps        = _cfg(cfg, "camera", "fps",     30,   int),
        cam_rotation   = _cfg(cfg, "camera", "rotation", 0,   int),
        enable_aruco   = _cfg(cfg, "aruco",  "enabled", False, lambda x: x.lower() == "true"),
        aruco_dict     = _cfg(cfg, "aruco",  "dict",    "DICT_4X4_1000"),
        throttle_ch    = _cfg(cfg, "rc", "throttle_ch",    3,    int),
        steer_ch       = _cfg(cfg, "rc", "steer_ch",       1,    int),
        mode_ch        = _cfg(cfg, "rc", "mode_ch",        5,    int),
        speed_ch       = _cfg(cfg, "rc", "speed_ch",       6,    int),
        auto_type_ch   = _cfg(cfg, "rc", "auto_type_ch",   7,    int),
        gps_log_ch      = _cfg(cfg, "rc", "gps_log_ch",      8,   int),
        gps_bookmark_ch = _cfg(cfg, "rc", "gps_bookmark_ch", 2,   int),
        gps_log_dir     = _cfg(cfg, "gps", "log_dir",        ""),
        gps_log_hz      = _cfg(cfg, "gps", "log_hz",         5.0, float),
        deadzone       = _cfg(cfg, "rc", "deadzone",       30,   int),
        failsafe_s     = _cfg(cfg, "rc", "failsafe_s",     0.5,  float),
        speed_min      = _cfg(cfg, "rc", "speed_min",      0.25, float),
        control_hz     = _cfg(cfg, "rc", "control_hz",     50,   int),
    )
    robot.start()

    try:
        while True:
            s = robot.get_state()
            t = s.telemetry
            g = s.gps
            print(
                f"\r[{s.mode.name:6}] "
                f"L={s.drive.left:+.2f} R={s.drive.right:+.2f}  "
                f"V={t.voltage:.1f}V I={t.current:.2f}A "
                f"T={t.board_temp:.0f}°C  "
                f"RC={'OK' if s.rc_active else '--'}  "
                f"AT={s.auto_type.label}  "
                f"CAM={'OK' if s.camera_ok else '--'}  "
                f"ACO={'OK' if s.aruco_ok  else '--'}  "
                f"LDR={'OK' if s.lidar_ok  else '--'}  "
                f"GPS={g.fix_quality_name if s.gps_ok else '--'}  "
                f"{'●LOG ' if s.gps_logging else ''}"
                f"{'FAULT-L ' if t.left_fault  else ''}"
                f"{'FAULT-R'  if t.right_fault else ''}",
                end='', flush=True,
            )
            time.sleep(0.5)
    except KeyboardInterrupt:
        print()
    finally:
        robot.stop()


if __name__ == "__main__":
    main()
