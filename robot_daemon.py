#!/usr/bin/env python3
"""
robot_daemon.py — Central Robot class for HackyRacingRobot differential-drive robot.

Subsystems
----------
  Drive    : Yukon RP2040 USB serial (CMD_LEFT / CMD_RIGHT / CMD_KILL / CMD_SENSOR)
  RC       : FlySky iBUS manual control + mode/speed-limit switches
  Telemetry: Yukon voltage, current, temps, faults, IMU heading  (1 Hz)
  Camera   : picamera2 (IMX296) or OpenCV fallback; optional ArUco detection
  LiDAR    : LDROBOT LD06 via drivers/ld06.py
  GPS      : Allystar TAU1308 RTK via gnss/ package; optional NTRIP/RTCM corrections

Mode switching (RC channel 5, SwA — configurable via [rc] mode_ch)
-------------------------------------------------------------------
  CH5 ≤ 1500  →  MANUAL  (RC tank-mix drives motors directly at 50 Hz)
  CH5 > 1500  →  AUTO    (call robot.drive(left, right) from your navigator)
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
  python3 robot_daemon.py               # print state at 2 Hz until Ctrl+C
  python3 robot_daemon.py --no-camera --no-lidar --no-gps
"""

import configparser
import csv
import json
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

# Navigator imports are deferred to avoid hard dependency when not used
try:
    from robot.aruco_navigator import ArucoNavigator, NavState
    _NAVIGATOR_AVAILABLE = True
except ImportError:
    _NAVIGATOR_AVAILABLE = False
    log.debug("aruco_navigator not found — camera auto mode disabled")

try:
    from robot.gps_navigator import GpsNavigator, GpsNavState
    _GPS_NAVIGATOR_AVAILABLE = True
except ImportError:
    _GPS_NAVIGATOR_AVAILABLE = False
    log.debug("gps_navigator not found — GPS auto mode disabled")


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
    voltage:     float          = 0.0
    current:     float          = 0.0
    board_temp:  float          = 0.0
    left_temp:   float          = 0.0
    right_temp:  float          = 0.0
    left_fault:  bool           = False
    right_fault: bool           = False
    heading:     Optional[float] = None   # IMU heading in degrees (None = IMU absent)
    timestamp:   float          = 0.0


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
    gps_logging:    bool        = False
    cam_recording:  bool        = False
    data_logging:   bool        = False
    speed_scale:    float       = 0.25   # current speed limit scale (0.0–1.0)
    nav_state:   str         = "IDLE" # Navigator state name
    nav_gate:    int         = 0      # ArUco target gate index
    nav_wp:      int         = 0      # GPS target waypoint index
    nav_wp_dist:     Optional[float] = None  # metres to current GPS waypoint
    nav_wp_bear:     Optional[float] = None  # bearing to current GPS waypoint
    nav_bearing_err: Optional[float] = None  # ArUco navigator bearing error (degrees)
    no_motors:       bool        = False  # drive commands suppressed


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

    CMD_LED     = 1   # value: 0=LED_A off, 1=LED_A on
    CMD_LEFT    = 2
    CMD_RIGHT   = 3
    CMD_KILL    = 4
    CMD_SENSOR  = 5
    CMD_BEARING = 6   # value: 0–254 = target bearing (0–359°), 255 = disable hold
    CMD_STRIP      = 7   # value: colour preset index (0=off 1=red 2=green 3=blue ...)
    CMD_PIXEL_SET  = 8   # value: high nibble = LED index, low nibble = colour index
    CMD_PIXEL_SHOW = 9   # value: ignored; pushes staged pixel data to hardware
    CMD_PATTERN    = 10  # value: high nibble = colour index (0=keep current), low nibble = pattern
                         #        patterns: 0=off 1=larson 2=random 3=rainbow 4=retro_computer 5=converge

    RESP_IDS = range(8)   # 0..7  (7 = heading)

    def __init__(self, port: str, baud: int = 115200, no_motors: bool = False):
        import serial
        self._no_motors = no_motors
        self._ser      = serial.Serial(port, baud, timeout=0.1, dsrdtr=False)
        time.sleep(0.5)
        self._ser.reset_input_buffer()
        self._cmd_lock  = threading.Lock()
        self._ack_q     = queue.Queue()
        self._sensor_q  = queue.Queue()
        self._stop      = threading.Event()
        self._rx = threading.Thread(target=self._reader, daemon=True, name="yukon_rx")
        self._rx.start()
        if no_motors:
            log.warning("NO-MOTORS mode: all drive/LED/bearing commands suppressed")

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
                    if (0x30 <= rtype <= 0x37 and
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
        # Heading: encoded as value * 254/359; 255 = IMU not available
        hdg_raw = raw.get(7, 255)
        heading = None if hdg_raw == 255 else round(hdg_raw * 359.0 / 254.0, 1)
        return Telemetry(
            voltage     = raw.get(0, 0) / 10.0,
            current     = raw.get(1, 0) / 100.0,
            board_temp  = raw.get(2, 0) / 3.0,
            left_temp   = raw.get(3, 0) / 3.0,
            right_temp  = raw.get(4, 0) / 3.0,
            left_fault  = bool(raw.get(5, 0)),
            right_fault = bool(raw.get(6, 0)),
            heading     = heading,
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
        if self._no_motors:
            return
        pkt = (self._encode(self.CMD_LEFT,  self._speed_byte(left)) +
               self._encode(self.CMD_RIGHT, self._speed_byte(right)))
        with self._cmd_lock:
            self._ser.write(pkt)
            self._drain(2)

    def set_led_a(self, on: bool):
        if self._no_motors:
            return
        with self._cmd_lock:
            self._ser.write(self._encode(self.CMD_LED, 1 if on else 0))
            self._drain(1)

    def set_led_b(self, on: bool):
        if self._no_motors:
            return
        with self._cmd_lock:
            self._ser.write(self._encode(self.CMD_LED, 3 if on else 2))
            self._drain(1)

    # Colour palette indices — mirror _STRIP_COLOURS in yukon firmware/main.py
    STRIP_OFF     = 0
    STRIP_RED     = 1
    STRIP_GREEN   = 2
    STRIP_BLUE    = 3
    STRIP_ORANGE  = 4
    STRIP_YELLOW  = 5
    STRIP_CYAN    = 6
    STRIP_MAGENTA = 7
    STRIP_WHITE   = 8

    def set_strip(self, preset: int):
        """Set all LEDs to a palette colour preset. Thread-safe."""
        if self._no_motors:
            return
        with self._cmd_lock:
            self._ser.write(self._encode(self.CMD_STRIP, max(0, min(255, preset))))
            self._drain(1)

    def set_pixel(self, index: int, colour: int):
        """Stage a single pixel colour from the palette. Call show_pixels() to update hardware."""
        if self._no_motors:
            return
        value = ((index & 0x0F) << 4) | (colour & 0x0F)
        with self._cmd_lock:
            self._ser.write(self._encode(self.CMD_PIXEL_SET, value))
            self._drain(1)

    def show_pixels(self):
        """Push all staged pixel data to the LED strip hardware."""
        if self._no_motors:
            return
        with self._cmd_lock:
            self._ser.write(self._encode(self.CMD_PIXEL_SHOW, 0))
            self._drain(1)

    PATTERN_OFF            = 0
    PATTERN_LARSON         = 1
    PATTERN_RANDOM         = 2
    PATTERN_RAINBOW        = 3
    PATTERN_RETRO_COMPUTER = 4
    PATTERN_CONVERGE       = 5
    PATTERN_ESTOP_FLASH    = 6

    def set_pattern(self, pattern: int, colour: int = 0):
        """Start a built-in LED animation (PATTERN_* constants). 0 stops and clears.

        colour: palette index to use for sparkle (0 = keep current, default white).
        """
        if self._no_motors:
            return
        value = ((colour & 0x0F) << 4) | (pattern & 0x0F)
        with self._cmd_lock:
            self._ser.write(self._encode(self.CMD_PATTERN, value))
            self._drain(1)

    # Named LED preset lookup — accepts colour or pattern names from robot.ini
    _LED_PRESETS: dict = {}   # populated below class body

    def apply_led_preset(self, name: str):
        """Apply a colour or pattern preset by name (e.g. 'larson', 'red')."""
        entry = self._LED_PRESETS.get(name.lower().strip(), ('strip', 0))
        kind, val = entry[0], entry[1]
        colour = entry[2] if len(entry) > 2 else 0
        if kind == 'pattern':
            self.set_pattern(val, colour)
        else:
            self.set_strip(val)

    def set_pixels(self, colours: list):
        """Set every pixel from a list of palette indices and push to hardware in one write."""
        if self._no_motors:
            return
        pkt = b''.join(
            self._encode(self.CMD_PIXEL_SET, ((i & 0x0F) << 4) | (c & 0x0F))
            for i, c in enumerate(colours)
        )
        pkt += self._encode(self.CMD_PIXEL_SHOW, 0)
        with self._cmd_lock:
            self._ser.write(pkt)
            self._drain(len(colours) + 1)

    def kill(self):
        if self._no_motors:
            return
        with self._cmd_lock:
            self._ser.write(self._encode(self.CMD_KILL, 0))
            self._drain(1, timeout=0.5)

    @staticmethod
    def _bearing_byte(degrees: float) -> int:
        """Encode a bearing (0–360°) as a 0–254 byte. 255 is reserved for disable."""
        return min(254, round(degrees * 254.0 / 359.0))

    def set_bearing(self, degrees: float):
        """Enable bearing hold. Yukon PID will steer toward this heading."""
        if self._no_motors:
            return
        degrees = degrees % 360.0
        with self._cmd_lock:
            self._ser.write(self._encode(self.CMD_BEARING, self._bearing_byte(degrees)))
            self._drain(1)

    def clear_bearing(self):
        """Disable bearing hold. Direct CMD_LEFT/CMD_RIGHT control resumes."""
        if self._no_motors:
            return
        with self._cmd_lock:
            self._ser.write(self._encode(self.CMD_BEARING, 255))
            self._drain(1)

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
        except (OSError, Exception):
            pass
        try:
            self._ser.close()
        except (OSError, Exception):
            pass


# Populate _YukonLink._LED_PRESETS after class body so constants are available
_YukonLink._LED_PRESETS = {
    'off':            ('strip',   _YukonLink.STRIP_OFF),
    'red':            ('strip',   _YukonLink.STRIP_RED),
    'green':          ('strip',   _YukonLink.STRIP_GREEN),
    'blue':           ('strip',   _YukonLink.STRIP_BLUE),
    'orange':         ('strip',   _YukonLink.STRIP_ORANGE),
    'yellow':         ('strip',   _YukonLink.STRIP_YELLOW),
    'cyan':           ('strip',   _YukonLink.STRIP_CYAN),
    'magenta':        ('strip',   _YukonLink.STRIP_MAGENTA),
    'white':          ('strip',   _YukonLink.STRIP_WHITE),
    'larson':         ('pattern', _YukonLink.PATTERN_LARSON),
    'random':         ('pattern', _YukonLink.PATTERN_RANDOM),
    'rainbow':        ('pattern', _YukonLink.PATTERN_RAINBOW),
    'retro_computer': ('pattern', _YukonLink.PATTERN_RETRO_COMPUTER),
    'converge':       ('pattern', _YukonLink.PATTERN_CONVERGE),
    'estop_flash':    ('pattern', _YukonLink.PATTERN_ESTOP_FLASH, _YukonLink.STRIP_YELLOW),
    'rc_lost':        ('pattern', _YukonLink.PATTERN_ESTOP_FLASH, _YukonLink.STRIP_BLUE),
    'motor_fault':    ('pattern', _YukonLink.PATTERN_ESTOP_FLASH, _YukonLink.STRIP_RED),
}


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
                 aruco_dict:   str  = "DICT_4X4_1000",
                 calib_file:   str  = None,
                 tag_size:     float = 0.15,
                 max_rec_minutes: float = 0.0,
                 rec_dir:      str  = ''):
        self._w             = width
        self._h             = height
        self._fps           = fps
        self._rotation      = rotation
        self._aruco_dict    = aruco_dict
        self._calib_file    = calib_file
        self._tag_size      = tag_size
        self._aruco_enabled = enable_aruco
        self._frame        = None
        self._aruco_state  = None   # ArUcoState or None
        self._lock         = threading.Lock()
        self._stop         = threading.Event()
        self._ok           = False
        self._aruco_ok     = False
        self._writer       = None   # cv2.VideoWriter or None
        self._rec_lock     = threading.Lock()
        self._rec_path     = ""
        self._rec_start    = 0.0
        self._max_rec_s    = max_rec_minutes * 60.0 if max_rec_minutes > 0 else 0.0
        self._rec_dir      = rec_dir

    def start(self):
        threading.Thread(target=self._run, daemon=True, name="camera").start()

    def _run(self):
        try:
            self._run_picamera2()
        except (ImportError, OSError):
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
            from robot.aruco_detector import ArucoDetector
            # Substitute {width}/{height} placeholders so robot.ini can use
            # a template like camera_cal_{width}x{height}.npz and the right
            # file is selected automatically for the configured resolution.
            calib = (self._calib_file
                     .replace('{width}',  str(self._w))
                     .replace('{height}', str(self._h))
                     ) if self._calib_file else None
            if calib and not os.path.exists(calib):
                log.warning(f"Calibration file not found: {calib} — pose estimation disabled. "
                            f"Run tools/derive_calibrations.py to generate it.")
                calib = None
            det = ArucoDetector(
                dict_name  = self._aruco_dict,
                calib_file = calib,
                tag_size   = self._tag_size,
            )
            log.info(f"ArUco detector ready ({self._aruco_dict}"
                     f"{f', calibrated ({calib})' if calib else ''})")
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
        _last_aruco_err = None
        period   = 1.0 / self._fps
        deadline = time.monotonic() + period
        try:
            while not self._stop.is_set():
                frame = cam.capture_array()[:, :, ::-1]  # BGR → RGB
                frame = self._rotate(frame, cv2)
                aruco_state = None
                if detector is not None and self._aruco_enabled:
                    try:
                        aruco_state = detector.detect(frame)
                        _last_aruco_err = None
                    except Exception as e:
                        msg = str(e)
                        if msg != _last_aruco_err:
                            log.warning(f"ArUco detect error: {e}")
                            _last_aruco_err = msg
                with self._lock:
                    self._frame       = frame
                    self._aruco_state = aruco_state
                    self._aruco_ok    = aruco_state is not None
                self._record_frame(frame)
                remaining = deadline - time.monotonic()
                if remaining > 0:
                    time.sleep(remaining)
                else:
                    deadline = time.monotonic()  # fell behind — reset, don't catch up
                deadline += period
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
        _last_aruco_err = None
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
                            _last_aruco_err = None
                        except Exception as e:
                            msg = str(e)
                            if msg != _last_aruco_err:
                                log.warning(f"ArUco detect error: {e}")
                                _last_aruco_err = msg
                    with self._lock:
                        self._frame       = frame
                        self._aruco_state = aruco_state
                        self._aruco_ok    = aruco_state is not None
                    self._record_frame(frame)
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

    def start_recording(self, path: str) -> bool:
        """Open a VideoWriter and begin saving frames to *path*. Returns True on success."""
        import cv2
        with self._rec_lock:
            if self._writer is not None:
                return False          # already recording
            # Determine actual frame size (may differ from configured if rotated 90/270)
            if self._rotation in (90, 270):
                w, h = self._h, self._w
            else:
                w, h = self._w, self._h
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            writer = cv2.VideoWriter(path, fourcc, self._fps, (w, h))
            if not writer.isOpened():
                writer.release()
                log.warning(f"VideoWriter failed to open: {path}")
                return False
            self._writer    = writer
            self._rec_path  = path
            self._rec_start = time.monotonic()
            log.info(f"Camera recording started → {path}")
            return True

    def stop_recording(self) -> str:
        """Flush and close the VideoWriter. Returns the saved file path."""
        with self._rec_lock:
            if self._writer is None:
                return ""
            self._writer.release()
            self._writer = None
            path = self._rec_path
            self._rec_path = ""
            log.info(f"Camera recording saved → {path}")
            return path

    def is_recording(self) -> bool:
        return self._writer is not None

    def _record_frame(self, frame):
        """Write a single RGB frame (with timestamp overlay) to the VideoWriter."""
        with self._rec_lock:
            if self._writer is None:
                return
            import cv2
            # Roll to a new file when max recording duration is exceeded
            if (self._max_rec_s > 0 and
                    time.monotonic() - self._rec_start >= self._max_rec_s):
                self._writer.release()
                log.info(f"Camera recording segment complete → {self._rec_path}")
                ts_str   = time.strftime("%Y%m%d_%H%M%S")
                new_path = os.path.join(self._rec_dir, f"recording_{ts_str}.mp4")
                os.makedirs(self._rec_dir, exist_ok=True)
                w, h = (self._h, self._w) if self._rotation in (90, 270) else (self._w, self._h)
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                writer = cv2.VideoWriter(new_path, fourcc, self._fps, (w, h))
                if not writer.isOpened():
                    writer.release()
                    self._writer = None
                    log.warning(f"VideoWriter failed to open new segment: {new_path}")
                    return
                self._writer    = writer
                self._rec_path  = new_path
                self._rec_start = time.monotonic()
                log.info(f"Camera recording new segment → {new_path}")
            bgr = frame[:, :, ::-1].copy()  # RGB → BGR, copy so we don't mutate the live frame
            ts  = time.strftime("%d-%m-%y  %H:%M:%S")
            # Shadow then white text for readability on any background
            cv2.putText(bgr, ts, (8, 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0),   3, cv2.LINE_AA)
            cv2.putText(bgr, ts, (8, 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
            self._writer.write(bgr)

    def stop(self):
        self.stop_recording()
        self._stop.set()


# ──────────────────────────────────────────────────────────────────────────────
# Lidar subsystem  (LDROBOT LD06)
# ──────────────────────────────────────────────────────────────────────────────

class _Lidar:
    """Thin wrapper around LD06 that matches the rest of robot.py's subsystem API."""

    def __init__(self, port: str = '/dev/ttyUSB0'):
        from drivers.ld06 import LD06
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
            except serial.SerialException:
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
# ML Data Logger
# ──────────────────────────────────────────────────────────────────────────────

class _DataLogger:
    """
    Logs all sensor readings and motor outputs to a JSONL file at a fixed rate.

    Each line is a self-contained JSON snapshot suitable for ML training:
      - Inputs : RC channels, GPS, LiDAR, IMU heading, ArUco detections
      - Outputs: motor drive commands, robot mode
    """

    def __init__(self):
        self._thread:   Optional[threading.Thread] = None
        self._stop_evt: threading.Event = threading.Event()
        self._path = ""

    def start(self, robot, path: str, hz: float = 10.0) -> bool:
        if self._thread and self._thread.is_alive():
            return False
        self._path = path
        self._stop_evt.clear()
        self._thread = threading.Thread(
            target=self._run, args=(robot, hz), daemon=True, name="data_log")
        self._thread.start()
        log.info(f"ML data logging started → {path} @ {hz:.1f} Hz")
        return True

    def stop(self):
        self._stop_evt.set()
        if self._thread:
            self._thread.join(timeout=3.0)
            self._thread = None
        log.info(f"ML data logging stopped → {self._path}")

    def is_active(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    # ── snapshot ──────────────────────────────────────────────────────────────

    @staticmethod
    def _snapshot(robot) -> dict:
        state  = robot.get_state()
        aruco  = robot.get_aruco_state()
        rc     = list(robot._rc_channels)   # 14 raw µs values

        g  = state.gps
        t  = state.telemetry
        d  = state.drive
        li = state.lidar
        s  = state.system
        now = time.time()

        aruco_data = None
        if aruco is not None:
            aruco_data = {
                'fps':   round(aruco.fps, 1),
                'tags':  [{'id': tg.id,
                           'cx': tg.center_x, 'cy': tg.center_y,
                           'area': tg.area,
                           'bearing': tg.bearing,
                           'distance': tg.distance}
                          for tg in aruco.tags.values()],
                'gates': [{'gate_id': ga.gate_id,
                           'cx': ga.centre_x, 'cy': ga.centre_y,
                           'bearing': ga.bearing,
                           'distance': ga.distance,
                           'correct_dir': ga.correct_dir}
                          for ga in aruco.gates.values()],
            }

        return {
            'ts':      round(now, 4),
            'ts_iso':  time.strftime('%Y-%m-%dT%H:%M:%S', time.localtime(now)),
            'mode':       state.mode.name,
            'auto_type':  state.auto_type.label,
            'speed_scale': round(state.speed_scale, 3),
            'rc_active':  state.rc_active,
            'rc_channels': rc,
            'drive': {
                'left':  round(d.left,  4),
                'right': round(d.right, 4),
            },
            'telemetry': {
                'voltage':     round(t.voltage,    3),
                'current':     round(t.current,    3),
                'board_temp':  round(t.board_temp, 1),
                'left_temp':   round(t.left_temp,  1),
                'right_temp':  round(t.right_temp, 1),
                'heading':     round(t.heading, 2) if t.heading is not None else None,
                'left_fault':  t.left_fault,
                'right_fault': t.right_fault,
            },
            'gps': {
                'lat':       g.latitude,
                'lon':       g.longitude,
                'alt':       g.altitude,
                'speed':     g.speed,
                'heading':   g.heading,
                'fix':       g.fix_quality,
                'fix_name':  g.fix_quality_name,
                'sats':      g.satellites,
                'sats_view': g.satellites_view,
                'hdop':      g.hdop,
                'h_error_m': g.h_error_m,
                'ts':        g.timestamp,
            },
            'lidar': {
                'angles':    li.angles,
                'distances': li.distances,
                'ts':        li.timestamp,
            },
            'aruco': aruco_data,
            'nav': {
                'state':       state.nav_state,
                'gate':        state.nav_gate,
                'wp':          state.nav_wp,
                'wp_dist':     state.nav_wp_dist,
                'wp_bear':     state.nav_wp_bear,
                'bearing_err': state.nav_bearing_err,
            },
            'system': {
                'cpu_pct':  round(s.cpu_percent,  1),
                'cpu_temp': round(s.cpu_temp_c,   1),
                'mem_pct':  round(s.mem_percent,  1),
                'disk_pct': round(s.disk_percent, 1),
            },
        }

    # ── thread ────────────────────────────────────────────────────────────────

    def _run(self, robot, hz: float):
        interval = 1.0 / max(hz, 0.1)
        try:
            with open(self._path, 'w', buffering=1) as f:   # line-buffered
                while not self._stop_evt.is_set():
                    t0 = time.monotonic()
                    try:
                        record = self._snapshot(robot)
                        f.write(json.dumps(record, separators=(',', ':')) + '\n')
                    except Exception as e:
                        log.warning(f"Data log snapshot error: {e}")
                    remaining = interval - (time.monotonic() - t0)
                    if remaining > 0:
                        self._stop_evt.wait(remaining)
        except OSError as e:
            log.error(f"Data logger file error: {e}")


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
        aruco_calib:    str   = '',        # path to camera_cal.npz
        aruco_tag_size: float = 0.15,      # physical tag side length in metres
        throttle_ch:    int   = 3,
        steer_ch:       int   = 1,
        mode_ch:        int   = 5,
        speed_ch:       int   = 6,
        auto_type_ch:   int   = 7,
        gps_log_ch:      int   = 8,
        gps_bookmark_ch: int   = 2,
        gps_log_dir:     str   = '',
        gps_log_hz:      float = 5.0,
        rec_dir:              str   = '',   # video recordings; blank = ~/Videos/HackyRacingRobot
        max_recording_minutes: float = 0.0, # 0 = unlimited; >0 rolls to new file each N minutes
        data_log_dir:    str   = '',   # ML data logs;     blank = ~/Documents/HackyRacingRobot
        deadzone:       int   = 30,
        failsafe_s:     float = 0.5,
        speed_min:      float = 0.25,
        control_hz:     int   = 50,
        no_motors:      bool  = False,  # suppress all drive commands (bench testing)
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
        self._aruco_calib    = aruco_calib.strip() or None
        self._aruco_tag_size = aruco_tag_size
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
        self._rec_dir       = (rec_dir.strip() or
                               os.path.join(os.path.expanduser('~'), 'Videos', 'HackyRacingRobot'))
        self._max_rec_minutes = max_recording_minutes
        self._data_log_dir  = (data_log_dir.strip() or
                               os.path.join(os.path.expanduser('~'), 'Documents', 'HackyRacingRobot'))
        self._gps_log_interval = 1.0 / max(gps_log_hz, 0.1)
        self._deadzone    = deadzone
        self._failsafe_s  = failsafe_s
        self._speed_min   = speed_min
        self._control_hz  = control_hz
        self._no_motors   = no_motors
        # LED strip config — read directly from robot.ini so it's consistent
        # across all frontends without each one needing to pass these values.
        _ini = configparser.ConfigParser(inline_comment_prefixes=('#',))
        _ini.read(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'robot.ini'))
        self._led_manual      = _ini.get('leds', 'manual',      fallback='larson').strip()
        self._led_auto        = _ini.get('leds', 'auto',        fallback='retro_computer').strip()
        self._led_estop       = _ini.get('leds', 'estop',       fallback='estop_flash').strip()
        self._led_rc_lost     = _ini.get('leds', 'rc_lost',     fallback='rc_lost').strip()
        self._led_motor_fault = _ini.get('leds', 'motor_fault', fallback='motor_fault').strip()
        self._fault_rotation_s = float(_ini.get('leds', 'fault_rotation_s', fallback='2.0'))

        # Subsystems (created in start())
        self._yukon:         Optional[_YukonLink] = None
        self._camera:        Optional[_Camera]    = None
        self._lidar:         Optional[_Lidar]     = None
        self._gps:           Optional[_Gps]       = None
        self._system:        _System              = _System()
        self._data_logger:   _DataLogger          = _DataLogger()
        self._navigator:     Optional[object]     = None  # ArucoNavigator when available
        self._gps_navigator: Optional[object]     = None  # GpsNavigator when available

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
        suffix = " [NO-MOTORS]" if self._no_motors else ""
        while True:
            try:
                log.info(f"Connecting to Yukon on {self._yukon_port}{suffix}")
                self._yukon = _YukonLink(self._yukon_port, no_motors=self._no_motors)
                break
            except Exception as e:
                log.warning(f"Yukon not available on {self._yukon_port} ({e}) — retrying in 3 s")
                time.sleep(3)

        if self._enable['camera']:
            self._camera = _Camera(
                width             = self._cam_width,
                height            = self._cam_height,
                fps               = self._cam_fps,
                rotation          = self._cam_rotation,
                enable_aruco      = self._enable_aruco,
                aruco_dict        = self._aruco_dict,
                calib_file        = self._aruco_calib,
                tag_size          = self._aruco_tag_size,
                max_rec_minutes   = self._max_rec_minutes,
                rec_dir           = self._rec_dir,
            )
            self._camera.start()

        if _NAVIGATOR_AVAILABLE:
            self._navigator = ArucoNavigator.from_ini(
                os.path.join(os.path.dirname(os.path.abspath(__file__)), "robot.ini")
            )
            log.info("ArucoNavigator ready")

        if _GPS_NAVIGATOR_AVAILABLE:
            self._gps_navigator = GpsNavigator.from_ini(
                os.path.join(os.path.dirname(os.path.abspath(__file__)), "robot.ini")
            )
            log.info("GpsNavigator ready")

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
        self._data_logger.stop()
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
        import serial as _serial
        with self._mode_lock:
            self._mode = RobotMode.ESTOP
        if self._yukon:
            try:
                self._yukon.kill()
            except (_serial.SerialException, OSError):
                self._reconnect_yukon()
        log.warning("ESTOP triggered manually")

    def reset_estop(self):
        """Clear ESTOP and return to MANUAL."""
        with self._mode_lock:
            self._mode = RobotMode.MANUAL
        log.info("ESTOP cleared → MANUAL (re-engage AUTO switch to restart navigator)")

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
            gps_logging   = self._gps_logging,
            cam_recording = self.is_cam_recording(),
            data_logging  = self._data_logger.is_active(),
            nav_state   = (self._navigator.state.name     if self._navigator
                           else self._gps_navigator.state.name if self._gps_navigator
                           else "IDLE"),
            nav_gate    = self._navigator.gate_id    if self._navigator else 0,
            nav_wp          = self._gps_navigator.waypoint_index if self._gps_navigator else 0,
            nav_wp_dist     = self._gps_navigator.distance_to_wp if self._gps_navigator else None,
            nav_wp_bear     = self._gps_navigator.bearing_to_wp  if self._gps_navigator else None,
            nav_bearing_err = self._navigator.bearing_err        if self._navigator     else None,
            no_motors       = self._no_motors,
        )

    def get_heading(self) -> Optional[float]:
        """Return latest IMU heading in degrees from Yukon telemetry, or None."""
        with self._mode_lock:
            return self._telemetry.heading

    def get_gps_navigator(self):
        """Return the GpsNavigator instance, or None if unavailable."""
        return self._gps_navigator

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

    def start_cam_recording(self, path: str = None) -> bool:
        """Start recording the camera feed to *path* (auto-generated if None).

        Files are saved to ``~/Videos/HackyRacingRobot/``.
        Returns True if recording started successfully.
        """
        if not self._camera:
            return False
        if path is None:
            os.makedirs(self._rec_dir, exist_ok=True)
            ts   = time.strftime("%Y%m%d_%H%M%S")
            path = os.path.join(self._rec_dir, f"recording_{ts}.mp4")
        return self._camera.start_recording(path)

    def stop_cam_recording(self) -> str:
        """Stop recording and return the saved file path (empty string if not recording)."""
        return self._camera.stop_recording() if self._camera else ""

    def is_cam_recording(self) -> bool:
        """Return True if the camera is currently recording."""
        return bool(self._camera and self._camera.is_recording())

    def start_data_log(self, path: str = None, hz: float = 10.0) -> bool:
        """Start ML data logging to *path* (auto-generated if None).

        Files are saved as JSONL to ``~/Documents/HackyRacingRobot/``.
        Returns True if logging started successfully.
        """
        if path is None:
            os.makedirs(self._data_log_dir, exist_ok=True)
            ts   = time.strftime('%Y%m%d_%H%M%S')
            path = os.path.join(self._data_log_dir, f'data_{ts}.jsonl')
        return self._data_logger.start(self, path, hz)

    def stop_data_log(self) -> str:
        """Stop ML data logging and return the saved file path."""
        path = self._data_logger._path
        self._data_logger.stop()
        return path

    def is_data_logging(self) -> bool:
        """Return True if ML data logging is currently active."""
        return self._data_logger.is_active()

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
        import serial as _serial
        from drivers.ibus import IBusReader, IBusError
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
            except (IBusError, _serial.SerialException, OSError) as e:
                log.warning(f"iBUS error: {e} — retrying in 2 s")
                time.sleep(2.0)

    def _reconnect_yukon(self):
        """Close the dead Yukon link and reconnect in a background thread."""
        try:
            if self._yukon:
                self._yukon.close()
        except (AttributeError, OSError):
            pass
        self._yukon = None
        with self._mode_lock:
            if self._mode is not RobotMode.ESTOP:
                self._mode = RobotMode.ESTOP
                log.warning("Yukon disconnected — ESTOP engaged")
        log.warning("Yukon disconnected — reconnecting…")

        def _retry():
            while not self._stop_evt.is_set():
                try:
                    link = _YukonLink(self._yukon_port, no_motors=self._no_motors)
                    self._yukon = link
                    log.info("Yukon reconnected.")
                    return
                except Exception as e:
                    log.warning(f"Yukon not available ({e}) — retrying in 3 s")
                    self._stop_evt.wait(3)

        threading.Thread(target=_retry, daemon=True, name="yukon_reconnect").start()

    def _telemetry_thread(self):
        """Poll Yukon sensors and trigger ESTOP on fault."""
        import serial as _serial
        interval = 1.0 / _TELEMETRY_HZ   # sensor poll is fixed at 1 Hz
        _MAX_SILENT_POLLS = 5   # consecutive None results before treating as disconnect
        silent_count = 0
        while not self._stop_evt.is_set():
            if self._yukon:
                try:
                    result = self._yukon.query_sensor()
                except (_serial.SerialException, OSError):
                    self._reconnect_yukon()
                    self._stop_evt.wait(interval)
                    continue
                if result:
                    silent_count = 0
                    with self._mode_lock:
                        self._telemetry = result
                        if result.left_fault or result.right_fault:
                            if self._mode is not RobotMode.ESTOP:
                                log.warning("Motor fault detected — ESTOP")
                                self._mode = RobotMode.ESTOP
                else:
                    silent_count += 1
                    if silent_count >= _MAX_SILENT_POLLS:
                        log.warning("Yukon not responding (%d silent polls) — reconnecting",
                                    silent_count)
                        silent_count = 0
                        self._reconnect_yukon()
            self._stop_evt.wait(interval)

    def _control_thread(self):
        """Control loop: apply motor speeds based on current mode."""
        import serial as _serial
        dt        = 1.0 / self._control_hz
        last_left  = None
        last_right = None
        last_mode          = None
        last_auto_type     = None
        last_led_b         = None
        rc_was_active      = True
        rc_lost_active     = False
        last_fault_list    : list  = []
        fault_rotation_idx : int   = 0
        last_fault_tick    : float = 0.0
        last_strip_mode    = None   # tracks which mode preset is on the strip

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

            # Auto-enable ArUco on mode/type transitions only, so the T-key
            # toggle is not overridden every tick.
            if self._camera and (mode is not last_mode or
                                 self._auto_type is not last_auto_type):
                want_aruco = (mode is RobotMode.AUTO and
                              self._auto_type in (AutoType.CAMERA, AutoType.CAMERA_GPS))
                self._camera.set_aruco_enabled(want_aruco)
                log.info(f"ArUco {'ON' if want_aruco else 'OFF'} "
                         f"(mode={mode.name} type={self._auto_type.label})")
                last_auto_type = self._auto_type

            # RC signal transition detection
            rc_active = self._rc_active
            if not rc_active and rc_was_active:
                rc_lost_active = True
                log.warning("RC signal lost")
            elif rc_active and not rc_was_active:
                rc_lost_active = False
                log.info("RC signal recovered")
            rc_was_active = rc_active

            if self._yukon:
                try:
                    # LED A: on = AUTO, off = MANUAL/ESTOP
                    if mode is not last_mode:
                        self._yukon.set_led_a(mode is RobotMode.AUTO)
                        last_mode = mode

                    # ── LED strip: rotate through all active faults ───────────
                    tel = self._telemetry
                    active_faults = []
                    if rc_lost_active:
                        active_faults.append(self._led_rc_lost)
                    if mode is RobotMode.ESTOP and not rc_lost_active:
                        active_faults.append(self._led_estop)
                    if tel.left_fault or tel.right_fault:
                        active_faults.append(self._led_motor_fault)

                    want_preset = None
                    if active_faults:
                        if active_faults != last_fault_list:
                            # Fault list changed — restart rotation from first entry
                            fault_rotation_idx = 0
                            last_fault_tick    = t0
                            want_preset        = active_faults[0]
                            last_strip_mode    = None  # mode preset no longer on strip
                        elif t0 - last_fault_tick >= self._fault_rotation_s:
                            fault_rotation_idx = (fault_rotation_idx + 1) % len(active_faults)
                            last_fault_tick    = t0
                            want_preset        = active_faults[fault_rotation_idx]
                    else:
                        # No faults: apply mode preset when mode changed or faults just cleared
                        if mode is not last_strip_mode:
                            want_preset     = {
                                RobotMode.MANUAL: self._led_manual,
                                RobotMode.AUTO:   self._led_auto,
                                RobotMode.ESTOP:  self._led_estop,
                            }.get(mode, 'off')
                            last_strip_mode = mode

                    last_fault_list = active_faults[:]
                    if want_preset is not None:
                        self._yukon.apply_led_preset(want_preset)

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
                except (_serial.SerialException, OSError):
                    self._reconnect_yukon()

            if mode is RobotMode.ESTOP:
                left, right = 0.0, 0.0
                if last_left != 0.0 or last_right != 0.0:
                    if self._yukon:
                        try:
                            self._yukon.kill()
                        except (_serial.SerialException, OSError):
                            self._reconnect_yukon()
                    last_left = last_right = 0.0
                if last_mode is not RobotMode.ESTOP:
                    if self._navigator:
                        self._navigator.stop()
                    if self._gps_navigator:
                        self._gps_navigator.stop()
                    last_mode = RobotMode.ESTOP
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
                        if (self._navigator and
                                self._auto_type in (AutoType.CAMERA, AutoType.CAMERA_GPS)):
                            self._navigator.start()
                        if (self._gps_navigator and
                                self._auto_type in (AutoType.GPS, AutoType.CAMERA_GPS)):
                            self._gps_navigator.start()
                        log.info("RC → AUTO")

            else:  # AUTO
                # ── Camera autonomous: feed ArUco state + IMU heading ─────────
                if (self._auto_type in (AutoType.CAMERA, AutoType.CAMERA_GPS)
                        and self._navigator is not None
                        and self._camera is not None):
                    aruco_state = self._camera.get_aruco_state()
                    with self._mode_lock:
                        heading = self._telemetry.heading
                    if aruco_state is not None:
                        nav_left, nav_right = self._navigator.update(
                            aruco_state, self._cam_width,
                            heading   = heading,
                            yukon     = self._yukon,
                        )
                        self.drive(nav_left, nav_right)
                    left, right = self._auto_left, self._auto_right

                # ── GPS autonomous: feed GPS state + IMU heading ──────────────
                elif (self._auto_type in (AutoType.GPS, AutoType.CAMERA_GPS)
                        and self._gps_navigator is not None
                        and self._gps is not None):
                    gps_state = self._gps.get_state()
                    with self._mode_lock:
                        heading = self._telemetry.heading
                    gps_left, gps_right = self._gps_navigator.update(
                        gps_state,
                        imu_heading = heading,
                        yukon       = self._yukon,
                    )
                    self.drive(gps_left, gps_right)
                    left, right = self._auto_left, self._auto_right

                else:
                    left, right = auto_left, auto_right

                # RC failsafe in AUTO: if signal lost, ESTOP
                if not self._rc_active:
                    with self._mode_lock:
                        self._mode = RobotMode.ESTOP
                    if self._navigator:
                        self._navigator.stop()
                    if self._gps_navigator:
                        self._gps_navigator.stop()
                    log.warning("RC signal lost in AUTO — ESTOP")
                    continue
                # RC override: switch back to manual
                if self._rc_channels[self._ch_mode] <= 1500:
                    if self._navigator:
                        self._navigator.stop()
                    if self._gps_navigator:
                        self._gps_navigator.stop()
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
                try:
                    self._yukon.drive(left, right)
                except (_serial.SerialException, OSError):
                    self._reconnect_yukon()
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
        except (ImportError, AttributeError):
            pass
        return '/dev/ttyACM0'


# ──────────────────────────────────────────────────────────────────────────────
# Logging setup (used by robot_gui.py, robot_web.py, robot_mobile.py)
# ──────────────────────────────────────────────────────────────────────────────

def setup_logging(log_dir: str = None) -> str:
    """
    Configure root logger with a console handler and a rotating file handler.

    Parameters
    ----------
    log_dir : directory for the log file.  Defaults to ``logs/`` next to
              this file.  Created automatically if it does not exist.

    Returns the absolute path of the log file.
    """
    from logging.handlers import RotatingFileHandler

    if log_dir is None:
        log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'logs')
    os.makedirs(log_dir, exist_ok=True)
    log_path = os.path.join(log_dir, 'robot.log')

    fmt_file    = logging.Formatter('%(asctime)s %(levelname)s %(name)s: %(message)s')
    fmt_console = logging.Formatter('%(levelname)s %(name)s: %(message)s')

    root = logging.getLogger()
    root.setLevel(logging.INFO)

    # Console
    sh = logging.StreamHandler()
    sh.setFormatter(fmt_console)
    root.addHandler(sh)

    # Rotating file — 5 MB × 3 files
    fh = RotatingFileHandler(log_path, maxBytes=5 * 1024 * 1024, backupCount=3)
    fh.setFormatter(fmt_file)
    root.addHandler(fh)

    return log_path


# ──────────────────────────────────────────────────────────────────────────────
# CLI demo
# ──────────────────────────────────────────────────────────────────────────────

def _load_config(path):
    """Load robot.ini; return configparser instance (empty if file not found)."""
    import configparser
    cfg = configparser.ConfigParser(inline_comment_prefixes=('#',))
    cfg.read(path)
    return cfg


from robot_utils import _cfg  # noqa: E402  (defined after _load_config for locality)


def main():
    import argparse
    import os

    DEFAULT_CFG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "robot.ini")

    parser = argparse.ArgumentParser(description="HackyRacingRobot",
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
    parser.add_argument("--no-motors",       action="store_true", default=False,
                        help="Suppress all motor/LED/bearing commands (bench test mode)")
    args = parser.parse_args()

    cfg = _load_config(args.config)

    _ENV_MAP = {
        ("ntrip", "host"):     "NTRIP_HOST",
        ("ntrip", "port"):     "NTRIP_PORT",
        ("ntrip", "mount"):    "NTRIP_MOUNT",
        ("ntrip", "user"):     "NTRIP_USER",
        ("ntrip", "password"): "NTRIP_PASSWORD",
    }

    def arg(cli_val, section, key, fallback, cast=str):
        """CLI wins → env var → config file → built-in default."""
        if cli_val is not None:
            return cli_val
        env_name = _ENV_MAP.get((section, key))
        if env_name:
            env_val = os.environ.get(env_name)
            if env_val is not None:
                try:
                    return cast(env_val)
                except ValueError:
                    pass
        return _cfg(cfg, section, key, fallback, cast)

    log_path = setup_logging()
    log.info(f"Log file: {log_path}")
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
        enable_aruco   = _cfg(cfg, "aruco",  "enabled",    False, lambda x: x.lower() == "true"),
        aruco_dict     = _cfg(cfg, "aruco",  "dict",       "DICT_4X4_1000"),
        aruco_calib    = _cfg(cfg, "aruco",  "calib_file", ""),
        aruco_tag_size = _cfg(cfg, "aruco",  "tag_size",   0.15, float),
        throttle_ch    = _cfg(cfg, "rc", "throttle_ch",    3,    int),
        steer_ch       = _cfg(cfg, "rc", "steer_ch",       1,    int),
        mode_ch        = _cfg(cfg, "rc", "mode_ch",        5,    int),
        speed_ch       = _cfg(cfg, "rc", "speed_ch",       6,    int),
        auto_type_ch   = _cfg(cfg, "rc", "auto_type_ch",   7,    int),
        gps_log_ch      = _cfg(cfg, "rc", "gps_log_ch",      8,   int),
        gps_bookmark_ch = _cfg(cfg, "rc", "gps_bookmark_ch", 10,  int),
        gps_log_dir     = _cfg(cfg, "gps", "log_dir",        ""),
        gps_log_hz      = _cfg(cfg, "gps", "log_hz",         5.0, float),
        deadzone       = _cfg(cfg, "rc", "deadzone",       30,   int),
        failsafe_s     = _cfg(cfg, "rc", "failsafe_s",     0.5,  float),
        speed_min      = _cfg(cfg, "rc", "speed_min",      0.25, float),
        control_hz     = _cfg(cfg, "rc", "control_hz",     50,   int),
        no_motors             = args.no_motors,
        rec_dir               = _cfg(cfg, 'output', 'videos_dir',           ''),
        max_recording_minutes = _cfg(cfg, 'output', 'max_recording_minutes', 0.0, float),
        data_log_dir          = _cfg(cfg, 'output', 'data_log_dir',          ''),
    )
    robot.start()

    import signal
    _quit = threading.Event()
    signal.signal(signal.SIGTERM, lambda sig, frame: _quit.set())
    signal.signal(signal.SIGINT,  lambda sig, frame: _quit.set())

    try:
        while not _quit.is_set():
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
            _quit.wait(0.5)
    finally:
        print()
        robot.stop()


if __name__ == "__main__":
    main()
