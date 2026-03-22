"""
robot/camera_controls.py — Shared camera constants, helpers, and utilities.

Used by camera_monitor.py (Pygame) and camera_web.py (Flask).
"""

import configparser as _cp
from pathlib import Path
from typing import Optional, Tuple

import cv2
import numpy as np
from picamera2 import Picamera2

from robot.aruco_detector import ArUcoState

# ── Paths ──────────────────────────────────────────────────────────────────────

_HERE       = Path(__file__).parent.parent   # project root
_cfg_ini    = _cp.ConfigParser(inline_comment_prefixes=('#',))
_cfg_ini.read(_HERE / "robot.ini")
_images_dir = _cfg_ini.get('output', 'images_dir', fallback='').strip()

IMAGE_DIR  = Path(_images_dir) if _images_dir else Path.home() / "Pictures" / "HackyRacingRobot"
CALIB_FILE = _HERE / "camera_cal.npz"

# ── Constants ──────────────────────────────────────────────────────────────────

CAPTURE_SIZES = [
    (640,  480),
    (1280, 720),
    (1456, 1088),   # full native IMX296
]

GAINS = [1.0, 2.0, 4.0, 8.0, 16.0]

EXP_STEPS   = [500, 1000, 2000, 4000, 8000, 16000, 33000, 66000]
EXP_DEFAULT = 4     # index into EXP_STEPS → 8000 µs

ARUCO_DICTS = [
    "DICT_4X4_50",
    "DICT_4X4_100",
    "DICT_4X4_1000",
    "DICT_5X5_100",
    "DICT_6X6_100",
]

# ── Frame processing ───────────────────────────────────────────────────────────

def sharpness(gray: np.ndarray) -> float:
    """Laplacian variance — higher = sharper.
    Downsampled to 640×480 so the value is resolution-independent."""
    small = cv2.resize(gray, (640, 480), interpolation=cv2.INTER_AREA)
    return float(cv2.Laplacian(small, cv2.CV_64F).var())


def rotate(frame: np.ndarray, degrees: int) -> np.ndarray:
    if degrees == 0:
        return frame
    return np.rot90(frame, k=-(degrees // 90))


# ── Camera factory ─────────────────────────────────────────────────────────────

def make_cam(width: int, height: int) -> Picamera2:
    cam = Picamera2()
    cam.configure(cam.create_video_configuration(
        main={"size": (width, height), "format": "RGB888"},
        controls={"FrameRate": 30},
    ))
    cam.start()
    return cam


# ── ArUco overlay ──────────────────────────────────────────────────────────────

_ACO_GREEN = (  0, 220,  80)
_ACO_RED   = (220,  40,  40)
_ACO_BLUE  = ( 40, 120, 220)
_ACO_WHITE = (230, 230, 240)
_ACO_FONT  = cv2.FONT_HERSHEY_SIMPLEX


def draw_aruco_on_frame(frame: np.ndarray, state: ArUcoState) -> None:
    """Draw ArUco tag boxes and gate lines onto an RGB numpy frame (in-place)."""
    for tag in state.tags.values():
        tl, tr = tag.top_left,     tag.top_right
        br, bl = tag.bottom_right, tag.bottom_left
        cv2.line(frame, tl, tr, _ACO_GREEN, 2)
        cv2.line(frame, tr, br, _ACO_GREEN, 2)
        cv2.line(frame, br, bl, _ACO_GREEN, 2)
        cv2.line(frame, bl, tl, _ACO_GREEN, 2)
        cv2.circle(frame, (tag.center_x, tag.center_y), 4, _ACO_RED, -1)
        cv2.putText(frame, str(tag.id),
                    (tl[0], tl[1] - 10), _ACO_FONT, 0.6, _ACO_WHITE, 2)

    for gate in state.gates.values():
        colour = _ACO_BLUE if gate.correct_dir else _ACO_RED
        cv2.line(frame,
                 (gate.centre_x, gate.centre_y - 50),
                 (gate.centre_x, gate.centre_y),
                 colour, 4)
        cv2.putText(frame, f"G{gate.gate_id}",
                    (gate.centre_x + 6, gate.centre_y - 52),
                    _ACO_FONT, 0.6, colour, 2)


# ── Calibration ────────────────────────────────────────────────────────────────

class CalibrationMaps:
    """Loads camera_cal.npz and lazily builds undistortion maps per frame size."""

    def __init__(self, calib_file: Path = CALIB_FILE):
        self._cam_mtx  = None
        self._dist     = None
        self._cal_size = None
        self._map1     = None
        self._map2     = None
        self._map_wh: Optional[Tuple[int, int]] = None

        if calib_file.exists():
            try:
                cal = np.load(calib_file)
                self._cam_mtx  = cal['camera_matrix']
                self._dist     = cal['dist_coeffs']
                self._cal_size = tuple(int(v) for v in cal['frame_size'])
            except Exception as e:
                print(f"Warning: could not load calibration {calib_file}: {e}")

    @property
    def available(self) -> bool:
        return self._cam_mtx is not None

    def get_maps(self, w: int, h: int) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """Return (map1, map2) for the given frame size, building lazily."""
        if self._cam_mtx is None:
            return None, None
        if self._map_wh != (w, h):
            cal_w, cal_h = self._cal_size
            sx, sy = w / cal_w, h / cal_h
            mtx = self._cam_mtx.copy()
            mtx[0, 0] *= sx; mtx[1, 1] *= sy
            mtx[0, 2] *= sx; mtx[1, 2] *= sy
            new_mtx, _ = cv2.getOptimalNewCameraMatrix(
                mtx, self._dist, (w, h), 1, (w, h))
            self._map1, self._map2 = cv2.initUndistortRectifyMap(
                mtx, self._dist, None, new_mtx, (w, h), cv2.CV_16SC2)
            self._map_wh = (w, h)
        return self._map1, self._map2
