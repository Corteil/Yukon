#!/usr/bin/env python3
"""
aruco_detector.py — ArUco marker detection and gate identification.

Detects ArUco markers in a camera frame, draws bounding boxes / centres,
and identifies gate pairs (odd tag + consecutive even tag: 1+2, 3+4, …).

Frames are expected in **RGB** format (as produced by robot.py's camera).

Usage::
    from aruco_detector import ArucoDetector

    detector = ArucoDetector()
    state = detector.detect(frame)   # annotates frame in-place; returns ArUcoState
    for gate in state.gates.values():
        print(f"Gate {gate.gate_id}: centre ({gate.centre_x}, {gate.centre_y}), "
              f"correct={gate.correct_dir}")
"""

import time
from dataclasses import dataclass, field
from typing import Dict, Tuple

import cv2

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


@dataclass
class ArUcoGate:
    """
    A gate formed by a consecutive odd/even tag pair (e.g. tags 1+2, 3+4).

    gate_id      : 0-based index  (tags 1+2 → gate 0, tags 3+4 → gate 1, …)
    odd_tag      : odd marker id  (left post when correct_dir=True)
    even_tag     : even marker id (right post when correct_dir=True)
    centre_x/y   : pixel centre between the two posts
    correct_dir  : True  → odd tag is on the LEFT  (BLUE — correct approach)
                   False → odd tag is on the RIGHT (RED  — wrong direction)
    """
    gate_id:     int
    odd_tag:     int
    even_tag:    int
    centre_x:    int
    centre_y:    int
    correct_dir: bool


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
    dict_name : ArUco dictionary to use (default ``"DICT_4X4_1000"``)
    draw      : Annotate frames in-place (default True)
    show_fps  : Overlay FPS counter (default True)
    """

    def __init__(self,
                 dict_name: str = "DICT_4X4_1000",
                 draw:      bool = True,
                 show_fps:  bool = True):
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

        grey = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        corners, ids, _ = self._detector.detectMarkers(grey)

        tags:  Dict[int, ArUcoTag]  = {}
        gates: Dict[int, ArUcoGate] = {}

        if ids is not None and len(ids) > 0:
            for marker_corner, marker_id in zip(corners, ids.flatten()):
                tag = self._process_marker(frame, marker_corner, int(marker_id))
                tags[tag.id] = tag

        if len(tags) >= 2:
            gates = self._find_gates(frame, tags)

        if self._draw and self._show_fps:
            cv2.putText(frame, f"FPS:{fps:.1f}", _FPS_POSITION,
                        _FONT, _FPS_FONT_SCALE, _WHITE, _FPS_FONT_THICKNESS)

        return ArUcoState(tags=tags, gates=gates, fps=round(fps, 1), timestamp=now)

    # ── private helpers ───────────────────────────────────────────────────────

    def _process_marker(self, frame, marker_corner, marker_id: int) -> ArUcoTag:
        pts = marker_corner.reshape((4, 2))
        tl  = (int(pts[0][0]), int(pts[0][1]))
        tr  = (int(pts[1][0]), int(pts[1][1]))
        br  = (int(pts[2][0]), int(pts[2][1]))
        bl  = (int(pts[3][0]), int(pts[3][1]))
        cx  = (tl[0] + br[0]) // 2
        cy  = (tl[1] + br[1]) // 2
        area = abs(tl[0] - br[0]) * abs(tl[1] - br[1])

        if self._draw:
            cv2.line(frame, tl, tr, _GREEN, _MARKER_BOX_THICKNESS)
            cv2.line(frame, tr, br, _GREEN, _MARKER_BOX_THICKNESS)
            cv2.line(frame, br, bl, _GREEN, _MARKER_BOX_THICKNESS)
            cv2.line(frame, bl, tl, _GREEN, _MARKER_BOX_THICKNESS)
            cv2.circle(frame, (cx, cy), _MARKER_CENTER_RADIUS, _RED, -1)
            cv2.putText(frame, str(marker_id),
                        (tl[0], tl[1] - 15),
                        _FONT, _MARKER_FONT_SCALE, _GREEN, _MARKER_FONT_THICKNESS)

        return ArUcoTag(id=marker_id, center_x=cx, center_y=cy, area=area,
                        top_left=tl, top_right=tr, bottom_right=br, bottom_left=bl)

    def _find_gates(self, frame, tags: Dict[int, ArUcoTag]) -> Dict[int, ArUcoGate]:
        gates: Dict[int, ArUcoGate] = {}
        for tag_id in sorted(tags):
            if tag_id % 2 == 1 and (tag_id + 1) in tags:
                odd  = tags[tag_id]
                even = tags[tag_id + 1]
                gate_id     = (tag_id - 1) // 2
                correct_dir = odd.center_x < even.center_x   # even (right post) on right
                colour      = _BLUE if correct_dir else _RED
                gcx = odd.center_x + (even.center_x - odd.center_x) // 2
                gcy = (odd.center_y + even.center_y) // 2

                if self._draw:
                    cv2.line(frame,
                             (gcx, gcy - _GATE_LINE_HALF_HEIGHT),
                             (gcx, gcy),
                             colour, _GATE_LINE_THICKNESS)
                    cv2.putText(frame, str(gate_id),
                                (gcx, gcy - _GATE_LABEL_OFFSET),
                                _FONT, _MARKER_FONT_SCALE, colour, _MARKER_FONT_THICKNESS)

                gates[gate_id] = ArUcoGate(
                    gate_id=gate_id, odd_tag=tag_id, even_tag=tag_id + 1,
                    centre_x=gcx, centre_y=gcy, correct_dir=correct_dir,
                )
        return gates
