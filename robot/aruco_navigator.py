#!/usr/bin/env python3
"""
aruco_navigator.py — Autonomous gate navigation using ArUco markers + IMU.

Consumes ArUcoState snapshots from aruco_detector.py and drives the robot
through a sequence of gates (tag pairs 1+2, 3+4, … up to max_gates).

IMU integration
---------------
When a heading is supplied to update() three features are enabled:

  1. Heading hold between camera frames
     When the camera gives a bearing update the navigator sets an IMU target
     heading (_imu_target). On frames where tags are not visible but heading
     is available the robot steers to maintain that heading rather than
     relying solely on the camera bearing each frame.  This removes the
     camera frame-rate bottleneck from the steering loop entirely.

  2. Controlled search (search_mode selects between two strategies)

     "spin" (default): rotates in search_step_deg increments (default 45°),
     pausing between each step to allow the camera to catch a frame. Rotation
     stops the moment the target gate appears. After a full 360° with no
     result the counter resets and tries again.

     "serpentine": drives forward in a sweeping zigzag — suited to grass or
     any surface where the robot cannot spin on the spot. Each leg drives
     forward at search_speed with a steering bias of search_turn_bias,
     alternating direction every search_leg_time seconds. The camera sweeps
     left and right as the robot curves forward, covering a wide field.

  3. Straight-line PASSING on a fixed heading
     On entering PASSING the navigator calls yukon.set_bearing(heading) so
     the Yukon's onboard PID holds the robot dead-straight through the gate.
     clear_bearing() is called on exit.

Recovery behaviour
------------------
If the gate is lost while APPROACHING (robot was close and driving forward),
the navigator enters RECOVERING: it reverses briefly (recover_reverse_time
seconds at recover_reverse_speed) to create distance from the gate line
before transitioning back to SEARCHING.  This avoids overshooting the gate
unseen.  Gate loss from ALIGNING or SEARCHING goes straight to SEARCHING.

LiDAR obstacle stop
-------------------
If a LidarScan is passed to update(), the navigator halts (returns 0, 0)
whenever any point within the forward cone (obstacle_cone_deg wide) is
closer than obstacle_stop_dist metres.  The check is skipped during
SEARCHING and RECOVERING (robot is rotating or reversing, not driving into
an obstacle).

Gate sequencing
---------------
  - Gates are 0-based: gate 0 = tags 0+1, gate 1 = tags 2+3, etc.
  - The robot only advances to gate N+1 after passing gate N and once gate
    N+1's tags have become visible.  This enforces course order naturally.
  - Both gate tag IDs >= 100 is logged as a warning but does not stop navigation.

Single-tag fallback
-------------------
Tag IDs follow this convention:
  Base ID = tag_id % 100 (strips front/rear distinction; front = 0–99, rear = 100–199)
  Even base ID → OUTSIDE post (right side on correct approach) → aim RIGHT of tag
  Odd  base ID → INSIDE  post (left  side on correct approach) → aim LEFT  of tag

Gate N consists of outside post (base 2N) and inside post (base 2N+1):
  Gate 0 → base IDs 0 (outside) + 1 (inside)
  Gate 1 → base IDs 2 (outside) + 3 (inside)

When searching for a gate the navigator accepts either front (0–99) or rear
(100–199) variants of each post tag so the robot can locate the gate even when
approaching from behind.

  offset = TAG_OFFSET_K / distance_m  (calibrated)
         = PIXEL_OFFSET_K / sqrt(area) (pixel fallback)
"""

import logging
import math
import os
import time
from configparser import ConfigParser
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Dict, List, Optional, Tuple

try:
    import tomllib          # stdlib Python 3.11+
except ImportError:
    try:
        import tomli as tomllib   # pip install tomli
    except ImportError:
        tomllib = None            # load_track() will raise if called

from robot.aruco_detector import ArUcoState

log = logging.getLogger(__name__)


# ── Gate dataclass (pairing is the navigator's responsibility) ─────────────────

@dataclass
class ArUcoGate:
    """
    A gate formed by an outside post (even base ID) + inside post (odd base ID).

    gate_id      : 0-based gate index
    outside_tag  : detected tag ID for the outside post (0–99 front, 100–199 rear)
    inside_tag   : detected tag ID for the inside  post
    centre_x/y   : pixel midpoint between the two posts
    distance     : average metres to the two posts (None = unknown)
    bearing      : average bearing from camera centre in degrees (None = unknown)
    """
    gate_id:     int
    outside_tag: int
    inside_tag:  int
    centre_x:    int
    centre_y:    int
    distance:    float = None
    bearing:     float = None


# ── Track definition (loaded from track.toml) ─────────────────────────────────

@dataclass
class TrackGate:
    """One gate definition loaded from track.toml."""
    id:            int
    label:         str
    outside_front: int    # ArUco tag ID on outside post, front face (0–99)
    outside_rear:  int    # ArUco tag ID on outside post, rear face  (100–199)
    inside_front:  int    # ArUco tag ID on inside post,  front face (0–99)
    inside_rear:   int    # ArUco tag ID on inside post,  rear face  (100–199)
    width_m:       float  # physical gate width in metres
    heading_hint:  float  # compass heading to face this gate; −1 = no hint


@dataclass
class Track:
    """Parsed track.toml course definition."""
    name:     str
    loop:     bool
    sequence: List[int]           # ordered gate IDs to navigate
    gates:    Dict[int, TrackGate] = field(default_factory=dict)


def load_track(path: str) -> Track:
    """
    Parse a track.toml file and return a :class:`Track`.

    Raises ``RuntimeError`` if tomllib/tomli is not available.
    Raises ``ValueError`` if the file contains no [[gate]] blocks.
    Raises ``OSError`` / ``tomllib.TOMLDecodeError`` on file/parse errors.
    """
    if tomllib is None:
        raise RuntimeError(
            "tomllib not available — install tomli (pip install tomli) "
            "or upgrade to Python 3.11+"
        )
    with open(path, "rb") as fh:
        data = tomllib.load(fh)

    course = data.get("course", {})
    seq_raw = (course.get("sequence") or {}).get("gates") or []

    gates: Dict[int, TrackGate] = {}
    for g in data.get("gate", []):
        gid = int(g["id"])
        gates[gid] = TrackGate(
            id            = gid,
            label         = str(g.get("label", f"Gate {gid}")),
            outside_front = int(g.get("outside_front", gid * 2)),
            outside_rear  = int(g.get("outside_rear",  100 + gid * 2)),
            inside_front  = int(g.get("inside_front",  gid * 2 + 1)),
            inside_rear   = int(g.get("inside_rear",   100 + gid * 2 + 1)),
            width_m       = float(g.get("width_m",     1.0)),
            heading_hint  = float(g.get("heading_hint", -1.0)),
        )

    if not gates:
        raise ValueError(f"No [[gate]] blocks found in {path!r}")

    sequence = [int(x) for x in seq_raw] if seq_raw else sorted(gates.keys())

    return Track(
        name     = str(course.get("name", "Unnamed")),
        loop     = bool(course.get("loop", False)),
        sequence = sequence,
        gates    = gates,
    )


# ── State machine ─────────────────────────────────────────────────────────────

class NavState(Enum):
    IDLE        = auto()
    SEARCHING   = auto()
    ALIGNING    = auto()
    APPROACHING = auto()
    PASSING     = auto()
    RECOVERING  = auto()   # gate lost mid-approach: reverse briefly then search
    COMPLETE    = auto()
    ERROR       = auto()


# ── Config ────────────────────────────────────────────────────────────────────

@dataclass
class NavConfig:
    track_file:            str   = ""   # path to track.toml; empty = use max_gates formula
    max_gates:             int   = 10
    tag_size:              float = 0.15
    pass_distance:         float = 0.6
    pass_time:             float = 0.8
    pass_timeout:          float = 4.0   # safety net: advance after this even if tags still visible
    search_mode:           str   = "spin"  # "spin" or "serpentine"
    search_speed:          float = 0.25
    search_step_deg:       float = 45.0
    search_step_pause:     float = 0.3
    search_leg_time:       float = 4.0    # serpentine: max seconds per leg before forcing direction switch
    search_turn_bias:      float = 0.4    # serpentine fallback (no IMU): steering fraction (0=straight, 1=max curve)
    search_cone_deg:       float = 90.0   # serpentine: total sweep cone width in degrees (±half each side)
    fwd_speed:             float = 0.45
    align_fwd_speed:       float = 0.15
    steer_kp:              float = 0.6
    imu_kp:                float = 0.5
    align_deadband:        float = 3.0
    ramp_rate:             float = 2.0
    tag_offset_k:          float = 0.4
    pixel_offset_k:        float = 5000
    cam_heading_offset:    float = 0.0
    max_single_tag_offset_deg: float = 45.0  # cap on single-tag angular offset (prevents extreme corrections at close range)
    obstacle_stop_dist:    float = 0.5   # metres — halt if obstacle closer than this
    obstacle_cone_deg:     float = 60.0  # forward cone full width for LiDAR check
    recover_reverse_time:  float = 0.4   # seconds to reverse when gate lost mid-approach
    recover_reverse_speed: float = 0.20  # reverse speed during recovery


# ── Helpers ───────────────────────────────────────────────────────────────────

def _clamp(val: float, lo: float = -1.0, hi: float = 1.0) -> float:
    return max(lo, min(hi, val))


def _ramp(current: float, target: float, max_delta: float) -> float:
    diff = target - current
    if abs(diff) <= max_delta:
        return target
    return current + (max_delta if diff > 0 else -max_delta)


def _angle_diff(target: float, current: float) -> float:
    """Signed shortest-arc difference (target − current), range −180..+180."""
    return (target - current + 180.0) % 360.0 - 180.0


def _lidar_clear(lidar, cone_half_deg: float, stop_dist_m: float) -> bool:
    """Return True if the forward cone is clear, False if an obstacle is detected.

    lidar        : LidarScan (angles 0–360°, distances in mm; empty → clear)
    cone_half_deg: half-width of the forward cone in degrees (e.g. 30 for a 60° cone)
    stop_dist_m  : halt threshold in metres
    """
    if not lidar or not lidar.angles:
        return True
    stop_mm = stop_dist_m * 1000.0
    for ang, dist in zip(lidar.angles, lidar.distances):
        if dist <= 0:
            continue
        rel = (ang + 180.0) % 360.0 - 180.0   # normalise 0° = forward, −180..+180
        if abs(rel) <= cone_half_deg and dist <= stop_mm:
            return False
    return True


def _pixel_bearing(target_x: int, frame_cx: int, hfov_deg: float = 62.0) -> float:
    """Approximate bearing in degrees from frame centre to target_x pixel.
    hfov_deg: horizontal FOV of the camera (62° default for IMX296)."""
    norm = (target_x - frame_cx) / max(frame_cx, 1)
    return norm * (hfov_deg / 2.0)


# ── Navigator ─────────────────────────────────────────────────────────────────

class ArucoNavigator:
    """
    Autonomous gate navigator with IMU-assisted heading control.

    Typical usage in robot.py control thread::

        left, right = navigator.update(
            aruco_state  = camera.get_aruco_state(),
            frame_width  = cam_width,
            heading      = telemetry.heading,   # degrees or None
            yukon        = self._yukon,          # for bearing hold
        )
        robot.drive(left, right)
    """

    def __init__(self, cfg: NavConfig = None, track: Optional[Track] = None):
        self.cfg          = cfg or NavConfig()
        self._track:      Optional[Track] = track
        self._seq_idx:    int             = 0   # index into track.sequence
        self._state       = NavState.IDLE
        self._gate_id     = 0
        self._pass_start  = 0.0
        self._last_update = time.monotonic()

        # IMU / heading state
        self._imu_target:     Optional[float] = None  # camera-derived target heading
        self._pass_heading:   Optional[float] = None  # heading locked at gate entry
        self._search_origin:  Optional[float] = None  # heading at search step start
        self._search_turned:  float           = 0.0   # cumulative degrees rotated
        self._search_pausing: bool            = False
        self._search_pause_end: float         = 0.0
        # Serpentine search state
        self._serp_dir:       int             = 1              # +1 = turning right, -1 = turning left
        self._serp_leg_start: float           = 0.0            # monotonic time when current leg began
        self._serp_origin:    Optional[float] = None           # IMU heading when SEARCHING began
        self._serp_target:    Optional[float] = None           # current target heading for this leg

        # RECOVERING state timer
        self._recover_start: float = 0.0

        # Tag IDs being watched during PASSING — advance once none are visible
        self._pass_tag_ids: set = set()

        # Ramped motor outputs
        self._cur_left  = 0.0
        self._cur_right = 0.0

        # Diagnostics — readable by robot_gui.py / robot_daemon.py
        self.target_x:       Optional[int]   = None
        self.bearing_err:    Optional[float] = None
        self.tag_dist:       Optional[float] = None
        self.imu_heading:    Optional[float] = None
        # Aim-point bearing (camera-relative °, offset already applied for single tags)
        # and visible-tag count — updated every update() call; None when no target seen.
        self.target_bearing: Optional[float] = None
        self.tags_visible:   int             = 0

    # ── factory ──────────────────────────────────────────────────────────────

    @classmethod
    def from_ini(cls, path: str, section: str = "navigator") -> "ArucoNavigator":
        cp  = ConfigParser(inline_comment_prefixes=('#', ';'))
        cp.read(path)
        sec = cp[section] if cp.has_section(section) else {}

        def _f(k, d): return float(sec.get(k, d))
        def _i(k, d): return int(sec.get(k, d))

        cfg = NavConfig(
            track_file            = sec.get("track_file",        "").strip(),
            max_gates             = _i("max_gates",             NavConfig.max_gates),
            tag_size              = _f("tag_size",              NavConfig.tag_size),
            pass_distance         = _f("pass_distance",         NavConfig.pass_distance),
            pass_time             = _f("pass_time",             NavConfig.pass_time),
            pass_timeout          = _f("pass_timeout",          NavConfig.pass_timeout),
            search_mode           = sec.get("search_mode",       NavConfig.search_mode).strip(),
            search_speed          = _f("search_speed",          NavConfig.search_speed),
            search_step_deg       = _f("search_step_deg",       NavConfig.search_step_deg),
            search_step_pause     = _f("search_step_pause",     NavConfig.search_step_pause),
            search_leg_time       = _f("search_leg_time",       NavConfig.search_leg_time),
            search_turn_bias      = _f("search_turn_bias",      NavConfig.search_turn_bias),
            search_cone_deg       = _f("search_cone_deg",       NavConfig.search_cone_deg),
            fwd_speed             = _f("fwd_speed",             NavConfig.fwd_speed),
            align_fwd_speed       = _f("align_fwd_speed",       NavConfig.align_fwd_speed),
            steer_kp              = _f("steer_kp",              NavConfig.steer_kp),
            imu_kp                = _f("imu_kp",                NavConfig.imu_kp),
            align_deadband        = _f("align_deadband",        NavConfig.align_deadband),
            ramp_rate             = _f("ramp_rate",             NavConfig.ramp_rate),
            tag_offset_k          = _f("tag_offset_k",         NavConfig.tag_offset_k),
            pixel_offset_k        = _f("pixel_offset_k",       NavConfig.pixel_offset_k),
            cam_heading_offset        = _f("cam_heading_offset",         NavConfig.cam_heading_offset),
            max_single_tag_offset_deg = _f("max_single_tag_offset_deg", NavConfig.max_single_tag_offset_deg),
            obstacle_stop_dist        = _f("obstacle_stop_dist",         NavConfig.obstacle_stop_dist),
            obstacle_cone_deg     = _f("obstacle_cone_deg",    NavConfig.obstacle_cone_deg),
            recover_reverse_time  = _f("recover_reverse_time", NavConfig.recover_reverse_time),
            recover_reverse_speed = _f("recover_reverse_speed",NavConfig.recover_reverse_speed),
        )
        track: Optional[Track] = None
        if cfg.track_file:
            # Resolve relative paths relative to the ini file's directory
            track_path = cfg.track_file
            if not os.path.isabs(track_path):
                track_path = os.path.join(os.path.dirname(os.path.abspath(path)), track_path)
            try:
                track = load_track(track_path)
                log.info("Loaded track %r from %r: %d gates, sequence=%s, loop=%s",
                         track.name, track_path, len(track.gates),
                         track.sequence, track.loop)
            except Exception as exc:
                log.warning("Could not load track file %r: %s", track_path, exc)
        return cls(cfg, track=track)

    # ── lifecycle ─────────────────────────────────────────────────────────────

    def set_track(self, track: Track) -> None:
        """Replace the active track definition (call before start())."""
        self._track = track
        log.info("Track set: %r — %d gates, loop=%s",
                 track.name, len(track.sequence), track.loop)

    def start(self):
        """Begin navigation from the first gate in the sequence."""
        self._seq_idx        = 0
        self._gate_id        = self._seq_gate_id()
        self._cur_left       = 0.0
        self._cur_right      = 0.0
        self._imu_target     = None
        self._pass_heading   = None
        self._last_update    = time.monotonic()
        self._reset_search()
        self._apply_heading_hint()
        self._set_state(NavState.SEARCHING)
        log.info("Navigator started — target gate %d (%s)",
                 self._gate_id, self._gate_label())

    def stop(self):
        """Halt and return to IDLE (does NOT clear bearing — caller must if needed)."""
        self._set_state(NavState.IDLE)
        self._cur_left     = 0.0
        self._cur_right    = 0.0
        self._imu_target   = None
        self._pass_heading = None
        log.info("Navigator stopped")

    @property
    def state(self) -> NavState:
        return self._state

    @property
    def gate_id(self) -> int:
        return self._gate_id

    @property
    def gate_label(self) -> str:
        """Human-readable label for the current target gate."""
        return self._gate_label()

    @property
    def outside_tag_id(self) -> int:
        """Front-face ArUco tag ID for the outside post of the current target gate."""
        tg = self._current_track_gate()
        return tg.outside_front if tg else self._gate_id * 2

    @property
    def inside_tag_id(self) -> int:
        """Front-face ArUco tag ID for the inside post of the current target gate."""
        tg = self._current_track_gate()
        return tg.inside_front if tg else self._gate_id * 2 + 1

    @property
    def next_gate_id(self) -> int:
        """Gate ID of the next gate in sequence after the current one."""
        if self._track and self._track.sequence:
            next_idx = self._seq_idx + 1
            if next_idx < len(self._track.sequence):
                return self._track.sequence[next_idx]
        return self._gate_id + 1

    @property
    def next_outside_tag_id(self) -> int:
        """Front-face tag ID for the outside post of the next gate."""
        nid = self.next_gate_id
        if self._track:
            tg = self._track.gates.get(nid)
            if tg:
                return tg.outside_front
        return nid * 2

    @property
    def next_inside_tag_id(self) -> int:
        """Front-face tag ID for the inside post of the next gate."""
        nid = self.next_gate_id
        if self._track:
            tg = self._track.gates.get(nid)
            if tg:
                return tg.inside_front
        return nid * 2 + 1

    @property
    def next_gate_label(self) -> str:
        """Human-readable label for the next gate."""
        nid = self.next_gate_id
        if self._track:
            tg = self._track.gates.get(nid)
            if tg:
                return tg.label
        return f"Gate {nid}"

    # ── track helpers ─────────────────────────────────────────────────────────

    def _seq_gate_id(self) -> int:
        """Gate ID at the current sequence position (fallback: _gate_id)."""
        if self._track and self._track.sequence:
            return self._track.sequence[self._seq_idx % len(self._track.sequence)]
        return self._gate_id

    def _current_track_gate(self) -> Optional[TrackGate]:
        """TrackGate for the current target, or None if no track loaded."""
        if self._track:
            return self._track.gates.get(self._gate_id)
        return None

    def _gate_label(self) -> str:
        tg = self._current_track_gate()
        return tg.label if tg else f"Gate {self._gate_id}"

    def _apply_heading_hint(self) -> None:
        """Seed _imu_target from the current gate's heading_hint, if set."""
        tg = self._current_track_gate()
        if tg and tg.heading_hint >= 0.0:
            self._imu_target = tg.heading_hint
            log.info("Gate %d heading hint: %.1f°", self._gate_id, tg.heading_hint)

    def _find_gate_in_state(self, state: ArUcoState,
                            tg: TrackGate) -> Optional[object]:
        """Build an ArUcoGate from the track gate's tag IDs using state.tags directly.

        This bypasses the detector's even/odd pairing convention entirely, so
        any combination of tag IDs works — both even, both odd, arbitrary values.
        Accepts front or rear face tags for each post so the gate can be found
        whether the robot approaches from the front or the rear.

        Returns an ArUcoGate if BOTH posts are visible, otherwise None.
        """
        outside = (state.tags.get(tg.outside_front) or
                   state.tags.get(tg.outside_rear))
        inside  = (state.tags.get(tg.inside_front)  or
                   state.tags.get(tg.inside_rear))

        if outside is None or inside is None:
            return None

        gcx = (outside.center_x + inside.center_x) // 2
        gcy = (outside.center_y + inside.center_y) // 2

        dists = [t.distance for t in (outside, inside) if t.distance is not None]
        bears = [t.bearing  for t in (outside, inside) if t.bearing  is not None]

        # outside_tag = outside (even-base ID), inside_tag = inside (odd-base ID)
        return ArUcoGate(
            gate_id  = tg.id,
            outside_tag = outside.id,
            inside_tag  = inside.id,
            centre_x = gcx,
            centre_y = gcy,
            distance = sum(dists) / len(dists) if dists else None,
            bearing  = sum(bears) / len(bears) if bears else None,
        )

    # ── main update ──────────────────────────────────────────────────────────

    def update(
        self,
        aruco_state: ArUcoState,
        frame_width: int,
        heading: Optional[float] = None,
        lidar=None,
    ) -> Tuple[Optional[float], float, float]:
        """
        Process one camera frame, return (target_bearing, left, right).

        When target_bearing is not None, left == right == forward_speed and the
        caller should send CMD_BEARING(target_bearing) to the Yukon so its onboard
        PID handles differential steering.  When target_bearing is None the caller
        receives differential left/right values directly (no-IMU fallback).

        Parameters
        ----------
        aruco_state : latest ArUcoState from ArucoDetector.detect()
        frame_width : camera frame width in pixels
        heading     : current IMU heading in degrees (None = no IMU)
        lidar       : LidarScan snapshot for obstacle avoidance, or None
        """
        now = time.monotonic()
        dt  = min(now - self._last_update, 0.1)
        self._last_update = now
        self.imu_heading  = heading
        self.tags_visible = len(aruco_state.tags)  # always reflect current frame

        if self._state in (NavState.IDLE, NavState.COMPLETE, NavState.ERROR):
            return None, 0.0, 0.0

        frame_cx = frame_width // 2

        # ── RECOVERING ────────────────────────────────────────────────────────
        # Gate was lost mid-approach: reverse briefly to create distance, then
        # transition to SEARCHING so the rotation can re-acquire the tag.
        if self._state == NavState.RECOVERING:
            if (now - self._recover_start) >= self.cfg.recover_reverse_time:
                self._reset_search()
                self._set_state(NavState.SEARCHING)
            l, r = self._apply_ramp(
                -self.cfg.recover_reverse_speed,
                -self.cfg.recover_reverse_speed, dt,
            )
            return None, l, r

        # ── LiDAR obstacle check ──────────────────────────────────────────────
        # Only applied when moving forward; skipped during search rotation.
        if (lidar is not None and
                self._state in (NavState.ALIGNING, NavState.APPROACHING, NavState.PASSING) and
                not _lidar_clear(lidar,
                                 self.cfg.obstacle_cone_deg / 2.0,
                                 self.cfg.obstacle_stop_dist)):
            log.warning("Obstacle in forward cone — halting (gate %d)", self._gate_id)
            l, r = self._apply_ramp(0.0, 0.0, dt)
            return self._imu_target, l, r

        # ── PASSING ───────────────────────────────────────────────────────────
        # Drive straight until the gate tags leave the frame (robot has cleared
        # the post), with pass_time as a minimum floor and pass_timeout as a
        # safety net in case tags stay visible longer than expected.
        if self._state == NavState.PASSING:
            elapsed    = now - self._pass_start
            min_done   = elapsed >= self.cfg.pass_time
            timed_out  = elapsed >= self.cfg.pass_timeout
            gate_clear = not any(tid in aruco_state.tags
                                 for tid in self._pass_tag_ids)
            if timed_out or (min_done and gate_clear):
                if timed_out and not gate_clear:
                    log.warning("Gate %d pass timeout (%.1fs) — advancing anyway",
                                self._gate_id, elapsed)
                self._exit_passing()
                self._advance_gate()
            l, r = self._apply_ramp(self.cfg.fwd_speed, self.cfg.fwd_speed, dt)
            return self._pass_heading, l, r

        # ── Resolve aim point ─────────────────────────────────────────────────
        target_x, dist, cam_bearing, is_single_tag = self._resolve_target(aruco_state, frame_cx)

        self.target_x       = target_x
        self.tag_dist       = dist
        self.target_bearing = cam_bearing   # aim bearing with left/right offset applied

        # ── Gate lost ─────────────────────────────────────────────────────────
        if target_x is None:
            self.target_bearing = None
            if self._state == NavState.APPROACHING:
                # Lost close to the gate — reverse briefly before searching so
                # we don't overshoot into or past the gate unseen.
                log.debug("Gate %d lost during approach — recovering", self._gate_id)
                self._imu_target    = None
                self._recover_start = now
                self._set_state(NavState.RECOVERING)
                l, r = self._apply_ramp(
                    -self.cfg.recover_reverse_speed,
                    -self.cfg.recover_reverse_speed, dt,
                )
                return None, l, r
            if self._state != NavState.SEARCHING:
                log.debug("Gate %d lost — searching", self._gate_id)
                self._imu_target = None
                self._set_state(NavState.SEARCHING)
            return self._search_step(heading, dt, now)

        # ── Target visible ────────────────────────────────────────────────────
        # Update the IMU target heading from camera bearing
        if heading is not None and cam_bearing is not None:
            self._imu_target = (heading + cam_bearing + self.cfg.cam_heading_offset) % 360.0

        # Prefer IMU heading error (smooth); fall back to raw camera bearing
        if heading is not None and self._imu_target is not None:
            bearing_err = _angle_diff(self._imu_target, heading)
        else:
            bearing_err = cam_bearing if cam_bearing is not None else 0.0

        self.bearing_err = bearing_err

        # Steer: normalise ±45° → ±1.0 authority
        steer   = _clamp(self.cfg.steer_kp * (bearing_err / 45.0))
        aligned = abs(bearing_err) < self.cfg.align_deadband

        # ── State transitions ─────────────────────────────────────────────────
        if self._state == NavState.SEARCHING:
            self._reset_search()
            self._set_state(NavState.ALIGNING)

        if self._state == NavState.ALIGNING:
            log.debug("Gate %d aligning: err=%.1f° imu_target=%s heading=%s dist=%s",
                      self._gate_id, bearing_err,
                      f"{self._imu_target:.1f}" if self._imu_target is not None else "N/A",
                      f"{heading:.1f}" if heading is not None else "N/A",
                      f"{dist:.2f}m" if dist is not None else "N/A")
            if aligned:
                self._set_state(NavState.APPROACHING)

        if self._state == NavState.APPROACHING:
            pass_dist = self.cfg.pass_distance
            if dist is not None and dist <= pass_dist:
                log.info("Gate %d PASSING (dist=%.2fm heading=%s%s)",
                         self._gate_id, dist,
                         f"{heading:.1f}°" if heading is not None else "N/A",
                         " [single-tag]" if is_single_tag else "")
                self._enter_passing(heading, now)
                l, r = self._apply_ramp(self.cfg.fwd_speed, self.cfg.fwd_speed, dt)
                return self._pass_heading, l, r

        # ── Drive ─────────────────────────────────────────────────────────────
        fwd = self.cfg.align_fwd_speed if self._state == NavState.ALIGNING \
              else self.cfg.fwd_speed
        if heading is not None and self._imu_target is not None \
                and self._state != NavState.ALIGNING:
            # APPROACHING/PASSING with IMU: Yukon bearing-hold PID drives the
            # differential while Pi commands equal forward speed.
            l, r = self._apply_ramp(fwd, fwd, dt)
            return self._imu_target, l, r
        else:
            # ALIGNING (or no IMU): Pi computes differential directly.
            # steer_kp is much more aggressive than the Yukon bearing-hold KP,
            # so the robot pivots quickly to face the gate before approaching.
            l, r = self._apply_ramp(_clamp(fwd - steer), _clamp(fwd + steer), dt)
            return None, l, r

    # ── Search ────────────────────────────────────────────────────────────────

    def _search_step(
        self, heading: Optional[float], dt: float, now: float
    ) -> Tuple[Optional[float], float, float]:
        """Dispatch to the configured search strategy."""
        if self.cfg.search_mode == "serpentine":
            return self._search_serpentine(heading, dt, now)
        return self._search_spin(heading, dt, now)

    def _search_spin(
        self, heading: Optional[float], dt: float, now: float
    ) -> Tuple[Optional[float], float, float]:
        """Rotate in steps to find the target gate."""

        if heading is None:
            # No IMU — differential spin as fallback
            l, r = self._apply_ramp(-self.cfg.search_speed, self.cfg.search_speed, dt)
            return None, l, r

        # During pause between steps — hold current heading
        if self._search_pausing:
            if now < self._search_pause_end:
                l, r = self._apply_ramp(0.0, 0.0, dt)
                return heading, l, r
            self._search_pausing = False
            self._search_origin  = heading

        # Initialise step origin
        if self._search_origin is None:
            self._search_origin = heading

        step_turned = abs(_angle_diff(heading, self._search_origin))

        if step_turned >= self.cfg.search_step_deg:
            # Step complete — begin pause, hold current heading
            self._search_turned  += step_turned
            self._search_pausing  = True
            self._search_pause_end = now + self.cfg.search_step_pause
            log.debug("Search step done (%.0f° total)", self._search_turned)
            if self._search_turned >= 360.0:
                log.warning("Gate %d: full 360° without tags, retrying",
                            self._gate_id)
                self._search_turned = 0.0
            l, r = self._apply_ramp(0.0, 0.0, dt)
            return heading, l, r

        # Still rotating: Yukon PID spins toward search_target at speed=0
        search_target = (self._search_origin + self.cfg.search_step_deg) % 360.0
        l, r = self._apply_ramp(0.0, 0.0, dt)
        return search_target, l, r

    def _search_serpentine(
        self, heading: Optional[float], dt: float, now: float
    ) -> Tuple[Optional[float], float, float]:
        """Drive forward in an IMU-controlled zigzag within a ±cone/2 arc.

        Records the heading when SEARCHING begins as the centre of the sweep.
        Steers toward the ±search_cone_deg/2 target using bearing hold; switching
        direction when within 5° of the target or after search_leg_time seconds.
        Falls back to time-based differential steering when IMU is unavailable.
        """
        cone_half = self.cfg.search_cone_deg / 2.0

        if heading is None:
            # No IMU — fall back to time-based bias steering (differential)
            if self._serp_leg_start == 0.0:
                self._serp_leg_start = now
            if (now - self._serp_leg_start) >= self.cfg.search_leg_time:
                self._serp_dir       = -self._serp_dir
                self._serp_leg_start = now
                log.debug("Serpentine search: switching direction (dir=%+d, no IMU)",
                          self._serp_dir)
            steer = self.cfg.search_turn_bias * self._serp_dir
            l, r = self._apply_ramp(
                _clamp(self.cfg.search_speed - steer),
                _clamp(self.cfg.search_speed + steer), dt,
            )
            return None, l, r

        # IMU available — initialise origin and first target on first call
        if self._serp_origin is None:
            self._serp_origin    = heading
            self._serp_target    = (heading + cone_half * self._serp_dir) % 360.0
            self._serp_leg_start = now
            log.debug("Serpentine search: origin=%.1f° first target=%.1f°",
                      self._serp_origin, self._serp_target)

        err       = _angle_diff(self._serp_target, heading)
        at_target = abs(err) < 5.0
        timed_out = (now - self._serp_leg_start) >= self.cfg.search_leg_time

        if at_target or timed_out:
            self._serp_dir       = -self._serp_dir
            self._serp_target    = (self._serp_origin + cone_half * self._serp_dir) % 360.0
            self._serp_leg_start = now
            log.debug("Serpentine search: %s — new target=%.1f° (dir=%+d)",
                      "at target" if at_target else "timeout",
                      self._serp_target, self._serp_dir)

        # Bearing mode: Yukon PID steers to serp_target while driving forward
        l, r = self._apply_ramp(self.cfg.search_speed, self.cfg.search_speed, dt)
        return self._serp_target, l, r

    def _reset_search(self):
        self._search_origin   = None
        self._search_turned   = 0.0
        self._search_pausing  = False
        self._serp_dir        = 1
        self._serp_leg_start  = 0.0
        self._serp_origin     = None
        self._serp_target     = None

    # ── PASSING helpers ───────────────────────────────────────────────────────

    def _enter_passing(self, heading: Optional[float], now: float):
        self._pass_start   = now
        # Prefer the camera-derived gate-centre heading so the robot drives
        # through the gap even if the Yukon PID hasn't fully converged yet.
        self._pass_heading = self._imu_target if self._imu_target is not None else heading
        self._set_state(NavState.PASSING)
        # Record tag IDs so PASSING can wait until they leave the frame
        tg = self._current_track_gate()
        if tg:
            self._pass_tag_ids = {tg.outside_front, tg.outside_rear,
                                   tg.inside_front,  tg.inside_rear}
        else:
            gid = self._gate_id
            self._pass_tag_ids = {gid * 2, gid * 2 + 1,
                                   100 + gid * 2, 100 + gid * 2 + 1}
        if heading is not None:
            log.debug("PASSING: bearing locked at %.1f°", heading)

    def _exit_passing(self):
        self._pass_heading = None
        self._imu_target   = None

    # ── Gate advancement ──────────────────────────────────────────────────────

    def _advance_gate(self):
        if self._track and self._track.sequence:
            next_idx = self._seq_idx + 1
            if next_idx >= len(self._track.sequence):
                if self._track.loop:
                    log.info("Gate %d passed — looping back to gate %d",
                             self._gate_id, self._track.sequence[0])
                    self._seq_idx = 0
                    self._gate_id = self._seq_gate_id()
                    self._reset_search()
                    self._apply_heading_hint()
                    self._set_state(NavState.SEARCHING)
                else:
                    log.info("All %d gates complete!", len(self._track.sequence))
                    self._set_state(NavState.COMPLETE)
            else:
                self._seq_idx = next_idx
                self._gate_id = self._seq_gate_id()
                log.info("Gate passed — seeking %s (gate %d)",
                         self._gate_label(), self._gate_id)
                self._reset_search()
                self._apply_heading_hint()
                self._set_state(NavState.SEARCHING)
        else:
            # No track — original formula behaviour
            next_gate = self._gate_id + 1
            if next_gate >= self.cfg.max_gates:
                log.info("All %d gates complete!", self.cfg.max_gates)
                self._set_state(NavState.COMPLETE)
            else:
                log.info("Gate %d passed — seeking gate %d", self._gate_id, next_gate)
                self._gate_id = next_gate
                self._reset_search()
                self._set_state(NavState.SEARCHING)

    # ── Target resolution ─────────────────────────────────────────────────────

    def find_gate(self, state: ArUcoState, gate_id: int) -> Optional[ArUcoGate]:
        """Return an ArUcoGate for *gate_id* from *state*, or None if not visible.

        Uses the track file's tag-ID assignments when a track is loaded;
        falls back to the even/odd formula (gate N → tags 2N / 2N+1) otherwise.
        """
        if self._track:
            tg = self._track.gates.get(gate_id)
            if tg:
                return self._find_gate_in_state(state, tg)
        return self._pair_tags(state, gate_id)

    def _pair_tags(self, state: ArUcoState, gate_id: int) -> Optional[ArUcoGate]:
        """Pair even/odd base-ID tags from *state* into an ArUcoGate for *gate_id*.

        Used when no track.toml is loaded (formula-based tag IDs).
        Gate N uses front tags 2N (outside) and 2N+1 (inside); rear variants
        100+2N and 101+2N are also accepted.
        Returns None if both posts are not visible.
        """
        front_outside = gate_id * 2
        front_inside  = gate_id * 2 + 1
        rear_outside  = 100 + front_outside
        rear_inside   = 100 + front_inside

        outside = state.tags.get(front_outside) or state.tags.get(rear_outside)
        inside  = state.tags.get(front_inside)  or state.tags.get(rear_inside)
        if outside is None or inside is None:
            return None

        gcx   = (outside.center_x + inside.center_x) // 2
        gcy   = (outside.center_y + inside.center_y) // 2
        dists = [t.distance for t in (outside, inside) if t.distance is not None]
        bears = [t.bearing  for t in (outside, inside) if t.bearing  is not None]
        return ArUcoGate(
            gate_id    = gate_id,
            outside_tag = outside.id,
            inside_tag  = inside.id,
            centre_x   = gcx,
            centre_y   = gcy,
            distance   = sum(dists) / len(dists) if dists else None,
            bearing    = sum(bears) / len(bears) if bears else None,
        )

    def _resolve_target(
        self,
        state: ArUcoState,
        frame_cx: int,
    ) -> Tuple[Optional[int], Optional[float], Optional[float], bool]:
        """
        Work out where to aim this frame.

        Returns (target_x_pixels, distance_m, camera_bearing_degrees, is_single_tag).

        is_single_tag is True when only one post is visible (single-tag fallback),
        False when the full gate pair is resolved.

        When a track is loaded, tag IDs and outside/inside roles come from the
        TrackGate definition.  Without a track the original even/odd formula is
        used (gate N → outside front tag 2N, inside front tag 2N+1).

        Priority: full gate pair → outside-only → inside-only → None.
        Single-tag aim:
          outside post → aim RIGHT of tag (gate opening is to the right)
          inside  post → aim LEFT  of tag (gate opening is to the left)
        """
        gate_id = self._gate_id
        tg      = self._current_track_gate()

        # ── Resolve tag IDs for this gate ─────────────────────────────────────
        if tg:
            front_outside = tg.outside_front
            front_inside  = tg.inside_front
        else:
            front_outside = gate_id * 2
            front_inside  = gate_id * 2 + 1

        # ── Full gate (both posts) ────────────────────────────────────────────
        gate = (self._find_gate_in_state(state, tg)
                if tg else self._pair_tags(state, gate_id))
        if gate is not None:
            if gate.outside_tag >= 100 and gate.inside_tag >= 100:
                log.warning("Gate %d: high-ID tags visible (IDs %d/%d) — may be approaching from behind",
                            gate_id, gate.outside_tag, gate.inside_tag)
            bearing = gate.bearing if gate.bearing is not None \
                      else _pixel_bearing(gate.centre_x, frame_cx)
            return gate.centre_x, gate.distance, bearing, False

        # ── Single-tag fallback ───────────────────────────────────────────────
        # Accept both front and rear variants of each post (base_id = tag_id % 100)
        # so the robot can re-acquire after passing through from either side.
        outside_tag = (state.tags.get(front_outside)
                       or (state.tags.get(tg.outside_rear) if tg else None))
        inside_tag  = (state.tags.get(front_inside)
                       or (state.tags.get(tg.inside_rear)  if tg else None))

        if outside_tag is not None:
            tag, is_inside = outside_tag, False
        elif inside_tag is not None:
            tag, is_inside = inside_tag, True
        else:
            return None, None, None, False

        # Clockwise course: outside post is on the LEFT, inside post on the RIGHT.
        # Outside post: gate opening is to the RIGHT → aim right (add bearing offset).
        # Inside post:  gate opening is to the LEFT  → aim left (subtract bearing offset).
        if tag.bearing is not None and tag.distance is not None \
                and tag.distance > 0 and self.cfg.tag_offset_k > 0:
            # Use half the gate width from the track file when available;
            # fall back to tag_offset_k for unknown gate widths.
            tg_for_offset = self._current_track_gate()
            offset_m = (tg_for_offset.width_m / 2.0
                        if tg_for_offset is not None
                        else self.cfg.tag_offset_k)
            offset_deg = math.degrees(math.atan(offset_m / tag.distance))
            # Outside post: gate is to right → add offset
            # Inside post:  gate is to left  → subtract offset
            bearing = tag.bearing + (-offset_deg if is_inside else offset_deg)
            tx = tag.center_x  # tx only used for logging; bearing drives steering
        else:
            offset = self._single_tag_offset(tag)
            # Outside → aim RIGHT (add offset); inside → aim LEFT (subtract offset)
            tx = tag.center_x + (-offset if is_inside else offset)
            bearing = _pixel_bearing(tx, frame_cx)

        return tx, tag.distance, bearing, True

    def _single_tag_offset(self, tag) -> int:
        """Pixel offset to aim beside a single visible post (fallback when no distance).

        Uses the tag's apparent pixel area so the offset scales naturally with
        distance.  pixel_offset_k is tunable in robot.ini [navigator].
        """
        if tag.area > 0:
            return int(self.cfg.pixel_offset_k / math.sqrt(tag.area))
        return 80

    # ── Motor ramp ────────────────────────────────────────────────────────────

    def _apply_ramp(
        self, target_left: float, target_right: float, dt: float
    ) -> Tuple[float, float]:
        max_delta       = self.cfg.ramp_rate * dt
        self._cur_left  = _ramp(self._cur_left,  target_left,  max_delta)
        self._cur_right = _ramp(self._cur_right, target_right, max_delta)
        return self._cur_left, self._cur_right

    def _set_state(self, new_state: NavState):
        if new_state != self._state:
            log.info("Nav: %s → %s  (gate %d)",
                     self._state.name, new_state.name, self._gate_id)
            self._state = new_state


# ── robot.ini [navigator] reference ──────────────────────────────────────────
#
# [navigator]
# track_file            =            # path to track.toml; empty = use max_gates formula
# max_gates             = 10
# tag_size              = 0.15
# pass_distance         = 0.6
# pass_time             = 0.8
# pass_timeout          = 4.0
# search_mode           = spin         # "spin" or "serpentine" (use serpentine on grass)
# search_speed          = 0.25
# search_step_deg       = 45.0        # spin mode only: degrees per IMU-controlled step
# search_step_pause     = 0.3         # spin mode only: pause between steps (seconds)
# search_leg_time       = 4.0         # serpentine mode only: max seconds per leg (safety timeout)
# search_turn_bias      = 0.4         # serpentine mode only (no IMU fallback): steering fraction (0=straight, 1=max)
# search_cone_deg       = 90.0        # serpentine mode only: total sweep arc (±half each side of entry heading)
# fwd_speed             = 0.45
# align_fwd_speed       = 0.15
# steer_kp              = 0.3    # reduce if overshooting in ALIGNING; increase if not turning enough
# imu_kp                = 0.5
# align_deadband        = 3.0
# ramp_rate             = 2.0
# tag_offset_k          = 0.4
# pixel_offset_k        = 5000
# cam_heading_offset    = 0.0    # degrees: camera boresight offset from robot/IMU forward (+ve = camera aims right)
# max_single_tag_offset_deg = 45.0  # cap on single-tag angular offset to prevent extreme corrections at close range
# obstacle_stop_dist    = 0.5    # metres — halt if LiDAR detects obstacle closer than this
# obstacle_cone_deg     = 60.0   # full width of forward cone for obstacle check
# recover_reverse_time  = 0.4    # seconds to reverse when gate lost mid-approach
# recover_reverse_speed = 0.20   # reverse speed during recovery
#
# [aruco]
# enabled    = true
# dict       = DICT_4X4_1000
# calib_file = camera_cal.npz
# tag_size   = 0.15
