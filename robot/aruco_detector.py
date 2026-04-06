#!/usr/bin/env python3
"""
aruco_detector.py — ArUco marker detection and gate identification.

Detects ArUco markers in a camera frame, draws bounding boxes / centres,
and identifies gate pairs.

Gate / tag numbering convention
--------------------------------
Each physical gate post carries two ArUco markers — one on the front face and
one on the rear face:

  Tag ID 0–99   : front face  (robot approaching from correct direction)
  Tag ID 100–199: rear face   (robot approaching from behind; ID = front_id + 100)

Post roles within a gate:
  Even base IDs (0, 2, 4 …): OUTSIDE post (right-hand side on correct approach)
  Odd  base IDs (1, 3, 5 …): INSIDE  post (left-hand  side on correct approach)

Gate N consists of outside post (base ID 2N) and inside post (base ID 2N+1):
  Gate 0 → tags 0/100 (outside) + 1/101 (inside)
  Gate 1 → tags 2/102 (outside) + 3/103 (inside)

correct_dir
-----------
  True  → front tags visible (0–99),  outside on the right — correct approach
  False → rear  tags visible (100–199), outside on the left — approaching backwards
  If only one tag type is visible the pixel positions are used as a fallback.

Frames are expected in **RGB** format (as produced by robot.py's camera).

Usage::
    from aruco_detector import ArucoDetector

    detector = ArucoDetector()
    state = detector.detect(frame)   # annotates frame in-place; returns ArUcoState
    for gate in state.gates.values():
        print(f"Gate {gate.gate_id}: centre ({gate.centre_x}, {gate.centre_y}), "
              f"correct={gate.correct_dir}")
"""

import math
import time
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple

import cv2
import numpy as np

# ── ArUco dictionary map ──────────────────────────────────────────────────────

ARUCO_DICT: Dict[str, int] = {
    "DICT_4X4_50":         cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100":        cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250":        cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000":       cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50":         cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100":        cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250":        cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000":       cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50":         cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100":        cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250":        cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000":       cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50":         cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100":        cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250":        cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000":       cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
}

# ── Colours (RGB order — robot.py frames are RGB888) ──────────────────────────

_RED   = (255, 0,   0)
_BLUE  = (0,   0,   255)
_GREEN = (0,   255, 0)
_WHITE = (255, 255, 255)

# ── Drawing constants ─────────────────────────────────────────────────────────

_MARKER_BOX_THICKNESS  = 2
_MARKER_CENTER_RADIUS  = 4
_GATE_LINE_THICKNESS   = 5
_GATE_LINE_HALF_HEIGHT = 50
_GATE_LABEL_OFFSET     = 55
_FONT                  = cv2.FONT_HERSHEY_SIMPLEX
_MARKER_FONT_SCALE     = 0.5
_MARKER_FONT_THICKNESS = 2
_FPS_FONT_SCALE        = 1.0
_FPS_FONT_THICKNESS    = 2
_FPS_POSITION          = (10, 30)


# ── Dataclasses ───────────────────────────────────────────────────────────────

@dataclass
class ArUcoTag:
    """Single detected ArUco marker with pixel geometry."""
    id:           int
    center_x:     int
    center_y:     int
    area:         int                  # bounding-box area in pixels²
    top_left:     Tuple[int, int]
    top_right:    Tuple[int, int]
    bottom_right: Tuple[int, int]
    bottom_left:  Tuple[int, int]
    distance:     float = None         # metres (None = unknown)
    bearing:      float = None         # degrees from camera centre (None = unknown)


@dataclass
class ArUcoGate:
    """
    A gate formed by an outside (even base ID) + inside (odd base ID) post pair.

    gate_id      : 0-based index  (base IDs 0+1 → gate 0, 2+3 → gate 1, …)
    outside_tag  : detected tag ID of the outside post (even base; may be 100+ for rear)
    inside_tag   : detected tag ID of the inside  post (odd  base; may be 100+ for rear)
    centre_x/y   : pixel centre between the two posts
    correct_dir  : True  → front tags visible (0–99), outside on the RIGHT — BLUE
                   False → rear  tags visible (100–199) or outside on LEFT  — RED
    distance     : metres to gate centre (None = unknown)
    bearing      : degrees from camera centre (None = unknown, use pixel fallback)
    """
    gate_id:     int
    outside_tag: int
    inside_tag:  int
    centre_x:    int
    centre_y:    int
    correct_dir: bool
    distance:    float = None
    bearing:     float = None


@dataclass
class ArUcoState:
    """Snapshot of one detection pass."""
    tags:      Dict[int, ArUcoTag]  = field(default_factory=dict)
    gates:     Dict[int, ArUcoGate] = field(default_factory=dict)
    fps:       float                = 0.0
    timestamp: float                = 0.0


# ── Detector ──────────────────────────────────────────────────────────────────

class ArucoDetector:
    """
    Detects ArUco markers and gate pairs in RGB camera frames.

    Parameters
    ----------
    dict_name  : ArUco dictionary to use (default ``"DICT_4X4_1000"``)
    draw       : Annotate frames in-place (default True)
    show_fps   : Overlay FPS counter (default True)
    calib_file : Path to camera_cal.npz produced by tools/calibrate_camera.py.
                 When supplied, frames are undistorted before detection and
                 pose estimation (distance + bearing) is performed for each
                 marker using solvePnP.
    tag_size   : Physical side length of the printed ArUco marker in metres
                 (default 0.15 m).  Used for pose estimation; ignored when
                 no calib_file is supplied.
    """

    def __init__(self,
                 dict_name:  str   = "DICT_4X4_1000",
                 draw:       bool  = True,
                 show_fps:   bool  = True,
                 calib_file: str   = None,
                 tag_size:   float = 0.15):
        if dict_name not in ARUCO_DICT:
            raise ValueError(
                f"Unknown ArUco dictionary: {dict_name!r}. "
                f"Valid options: {sorted(ARUCO_DICT)}"
            )
        self._draw     = draw
        self._show_fps = show_fps
        _dictionary    = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[dict_name])
        _parameters    = cv2.aruco.DetectorParameters()
        self._detector = cv2.aruco.ArucoDetector(_dictionary, _parameters)
        self._t_prev   = time.monotonic()

        # 3-D object points for one square marker centred at the origin.
        # Corner order matches OpenCV ArUco: TL, TR, BR, BL.
        half = tag_size / 2.0
        self._obj_pts = np.array([
            [-half,  half, 0.0],
            [ half,  half, 0.0],
            [ half, -half, 0.0],
            [-half, -half, 0.0],
        ], dtype=np.float32)

        # Lens undistortion + pose estimation (both optional, need calib_file)
        self._map1        = None
        self._map2        = None
        self._cam_mtx     = None   # camera intrinsic matrix (loaded from calib)
        self._dist_zero   = np.zeros((4, 1), dtype=np.float32)  # zero after remap
        self._calib_size  = None   # (height, width) the maps were built for
        self._calib_warned = False
        if calib_file is not None:
            try:
                cal = np.load(calib_file)
                self._map1       = cal['map1']
                self._map2       = cal['map2']
                self._cam_mtx    = cal['camera_matrix'].astype(np.float32)
                self._calib_size = (self._map1.shape[0], self._map1.shape[1])
            except Exception as e:
                raise ValueError(f"Cannot load calibration file {calib_file!r}: {e}") from e

    # ── public API ────────────────────────────────────────────────────────────

    def detect(self, frame) -> ArUcoState:
        """
        Detect markers and gate pairs in *frame* (RGB numpy array).

        If ``draw=True``, the frame is annotated in-place:

        - Green bounding box + red centre dot + ID label for each marker
        - Vertical gate-centre line (blue = correct direction, red = reversed)
        - FPS counter in top-left corner

        Returns an :class:`ArUcoState` with all detected tags and gates.
        """
        now  = time.monotonic()
        dt   = now - self._t_prev
        fps  = 1.0 / dt if dt > 0 else 0.0
        self._t_prev = now

        # Undistort frame before detection if calibration was loaded.
        # remap() operates on the raw frame (RGB) and writes back in-place so
        # that any draw annotations also appear on the corrected image.
        # Skip undistortion if the frame size doesn't match the calibration maps
        # (e.g. camera configured at a different resolution than calibration).
        # pose_ok is True only when undistortion was applied — that's when the
        # camera_matrix is valid for solvePnP (no residual lens distortion).
        pose_ok = False
        if self._map1 is not None:
            fh, fw = frame.shape[:2]
            if (fh, fw) == self._calib_size:
                undistorted = cv2.remap(frame, self._map1, self._map2, cv2.INTER_LINEAR)
                frame[:] = undistorted
                pose_ok = True
            elif not self._calib_warned:
                import warnings
                warnings.warn(
                    f"ArucoDetector: calibration maps are {self._calib_size[1]}×{self._calib_size[0]} "
                    f"but frame is {fw}×{fh} — undistortion skipped. "
                    f"Re-calibrate at {fw}×{fh} or update calib_file.",
                    UserWarning, stacklevel=2,
                )
                self._calib_warned = True

        grey = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        corners, ids, _ = self._detector.detectMarkers(grey)

        tags:  Dict[int, ArUcoTag]  = {}
        gates: Dict[int, ArUcoGate] = {}

        if ids is not None and len(ids) > 0:
            for marker_corner, marker_id in zip(corners, ids.flatten()):
                tag = self._process_marker(frame, marker_corner, int(marker_id), pose_ok)
                tags[tag.id] = tag

        if len(tags) >= 2:
            gates = self._find_gates(frame, tags)

        if self._draw and self._show_fps:
            cv2.putText(frame, f"FPS:{fps:.1f}", _FPS_POSITION,
                        _FONT, _FPS_FONT_SCALE, _WHITE, _FPS_FONT_THICKNESS)

        return ArUcoState(tags=tags, gates=gates, fps=round(fps, 1), timestamp=now)

    # ── private helpers ───────────────────────────────────────────────────────

    def _process_marker(self, frame, marker_corner, marker_id: int,
                        pose_ok: bool) -> ArUcoTag:
        pts = marker_corner.reshape((4, 2))
        tl  = (int(pts[0][0]), int(pts[0][1]))
        tr  = (int(pts[1][0]), int(pts[1][1]))
        br  = (int(pts[2][0]), int(pts[2][1]))
        bl  = (int(pts[3][0]), int(pts[3][1]))
        cx  = (tl[0] + br[0]) // 2
        cy  = (tl[1] + br[1]) // 2
        area = abs(tl[0] - br[0]) * abs(tl[1] - br[1])

        # Pose estimation — only valid after undistortion (pose_ok=True).
        # solvePnP returns the translation vector in camera coordinates:
        #   t[0] = right, t[1] = down, t[2] = forward (optical axis).
        # distance = Euclidean distance to marker centre.
        # bearing  = horizontal angle (positive = right of boresight).
        distance: Optional[float] = None
        bearing:  Optional[float] = None
        if pose_ok and self._cam_mtx is not None:
            img_pts = pts.astype(np.float32)
            ok, rvec, tvec = cv2.solvePnP(
                self._obj_pts, img_pts,
                self._cam_mtx, self._dist_zero,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
            if ok:
                t        = tvec.flatten()
                distance = float(np.linalg.norm(t))
                bearing  = float(math.degrees(math.atan2(t[0], t[2])))

        if self._draw:
            cv2.line(frame, tl, tr, _GREEN, _MARKER_BOX_THICKNESS)
            cv2.line(frame, tr, br, _GREEN, _MARKER_BOX_THICKNESS)
            cv2.line(frame, br, bl, _GREEN, _MARKER_BOX_THICKNESS)
            cv2.line(frame, bl, tl, _GREEN, _MARKER_BOX_THICKNESS)
            cv2.circle(frame, (cx, cy), _MARKER_CENTER_RADIUS, _RED, -1)
            label = str(marker_id) if distance is None else f"{marker_id} {distance:.2f}m"
            cv2.putText(frame, label,
                        (tl[0], tl[1] - 15),
                        _FONT, _MARKER_FONT_SCALE, _GREEN, _MARKER_FONT_THICKNESS)

        return ArUcoTag(id=marker_id, center_x=cx, center_y=cy, area=area,
                        top_left=tl, top_right=tr, bottom_right=br, bottom_left=bl,
                        distance=distance, bearing=bearing)

    def _find_gates(self, frame, tags: Dict[int, ArUcoTag]) -> Dict[int, ArUcoGate]:
        """
        Pair outside (even base ID) and inside (odd base ID) posts into gates.

        Each post has a front tag (ID 0–99) and a rear tag (ID 100–199).
        base_id = tag_id % 100 strips the front/rear distinction for pairing.
        Gate N is formed by base IDs 2N (outside) and 2N+1 (inside).
        We prefer the front-face tag if both faces are visible for the same post.
        """
        gates: Dict[int, ArUcoGate] = {}

        # Build a map: base_id → best tag (prefer front face, i.e. lower ID)
        base_map: Dict[int, ArUcoTag] = {}
        for tag_id, tag in tags.items():
            base = tag_id % 100
            if base not in base_map or tag_id < base_map[base].id:
                base_map[base] = tag

        # Find pairs: even base (outside) + odd base (inside)
        for base_even in sorted(base_map):
            if base_even % 2 != 0:
                continue
            base_odd = base_even + 1
            if base_odd not in base_map:
                continue

            gate_id = base_even // 2
            outside = base_map[base_even]
            inside  = base_map[base_odd]

            gcx = (outside.center_x + inside.center_x) // 2
            gcy = (outside.center_y + inside.center_y) // 2

            # correct_dir: True when approaching from the front.
            # Primary indicator: front tags (ID < 100) are visible.
            # Fallback: outside post should be to the RIGHT on correct approach.
            both_front = outside.id < 100 and inside.id < 100
            both_rear  = outside.id >= 100 and inside.id >= 100
            if both_front or both_rear:
                correct_dir = both_front
            else:
                # Mixed front/rear — use pixel position as heuristic
                correct_dir = inside.center_x < outside.center_x

            colour = _BLUE if correct_dir else _RED

            if self._draw:
                cv2.line(frame,
                         (gcx, gcy - _GATE_LINE_HALF_HEIGHT),
                         (gcx, gcy),
                         colour, _GATE_LINE_THICKNESS)
                cv2.putText(frame, str(gate_id),
                            (gcx, gcy - _GATE_LABEL_OFFSET),
                            _FONT, _MARKER_FONT_SCALE, colour, _MARKER_FONT_THICKNESS)

            # Gate distance/bearing: average of the two post values
            dists = [t.distance for t in (outside, inside) if t.distance is not None]
            bears = [t.bearing  for t in (outside, inside) if t.bearing  is not None]
            gate_dist: Optional[float] = sum(dists) / len(dists) if dists else None
            gate_bear: Optional[float] = sum(bears) / len(bears) if bears else None

            gates[gate_id] = ArUcoGate(
                gate_id=gate_id, outside_tag=outside.id, inside_tag=inside.id,
                centre_x=gcx, centre_y=gcy, correct_dir=correct_dir,
                distance=gate_dist, bearing=gate_bear,
            )
        return gates
