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
     heading (_imu_target). On frames where tags are visible but heading
     is available the robot steers to maintain that heading rather than
     relying solely on the camera bearing each frame.  This removes the
     camera frame-rate bottleneck from the steering loop entirely.

  2. Controlled search rotation
     Instead of spinning continuously the robot rotates in search_step_deg
     increments (default 45°), pausing between each step to allow the camera
     to catch a frame.  Rotation stops the moment the target gate appears.
     After a full 360° with no result the counter resets and tries again.

  3. Straight-line PASSING on a fixed heading
     On entering PASSING the navigator calls yukon.set_bearing(heading) so
     the Yukon's onboard PID holds the robot dead-straight through the gate.
     clear_bearing() is called on exit.

Gate sequencing
---------------
  - Gates are 0-based: gate 0 = tags 1+2, gate 1 = tags 3+4, etc.
  - The robot only advances to gate N+1 after passing gate N and once gate
    N+1's tags have become visible.  This enforces course order naturally.
  - correct_dir=False is logged as a warning but does not stop navigation.

Single-tag fallback
-------------------
  Odd tag  (left post)  → aim RIGHT:  target_x = tag.center_x + offset
  Even tag (right post) → aim LEFT:   target_x = tag.center_x − offset
  offset = TAG_OFFSET_K / distance_m  (calibrated)
         = PIXEL_OFFSET_K / sqrt(area) (pixel fallback)
"""

import logging
import math
import time
from configparser import ConfigParser
from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional, Tuple

from robot.aruco_detector import ArUcoState

log = logging.getLogger(__name__)


# ── State machine ─────────────────────────────────────────────────────────────

class NavState(Enum):
    IDLE        = auto()
    SEARCHING   = auto()
    ALIGNING    = auto()
    APPROACHING = auto()
    PASSING     = auto()
    COMPLETE    = auto()
    ERROR       = auto()


# ── Config ────────────────────────────────────────────────────────────────────

@dataclass
class NavConfig:
    max_gates:         int   = 10
    tag_size:          float = 0.15
    pass_distance:     float = 0.6
    pass_time:         float = 0.8
    search_speed:      float = 0.25
    search_step_deg:   float = 45.0
    search_step_pause: float = 0.3
    fwd_speed:         float = 0.45
    align_fwd_speed:   float = 0.15
    steer_kp:          float = 0.6
    imu_kp:            float = 0.5
    align_deadband:    float = 3.0
    ramp_rate:         float = 2.0
    tag_offset_k:      float = 0.4
    pixel_offset_k:    float = 5000


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

    def __init__(self, cfg: NavConfig = None):
        self.cfg          = cfg or NavConfig()
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

        # Ramped motor outputs
        self._cur_left  = 0.0
        self._cur_right = 0.0

        # Diagnostics — readable by robot_gui.py
        self.target_x:    Optional[int]   = None
        self.bearing_err: Optional[float] = None
        self.tag_dist:    Optional[float] = None
        self.imu_heading: Optional[float] = None

    # ── factory ──────────────────────────────────────────────────────────────

    @classmethod
    def from_ini(cls, path: str, section: str = "navigator") -> "ArucoNavigator":
        cp  = ConfigParser(inline_comment_prefixes=('#', ';'))
        cp.read(path)
        sec = cp[section] if cp.has_section(section) else {}

        def _f(k, d): return float(sec.get(k, d))
        def _i(k, d): return int(sec.get(k, d))

        cfg = NavConfig(
            max_gates         = _i("max_gates",         NavConfig.max_gates),
            tag_size          = _f("tag_size",          NavConfig.tag_size),
            pass_distance     = _f("pass_distance",     NavConfig.pass_distance),
            pass_time         = _f("pass_time",         NavConfig.pass_time),
            search_speed      = _f("search_speed",      NavConfig.search_speed),
            search_step_deg   = _f("search_step_deg",   NavConfig.search_step_deg),
            search_step_pause = _f("search_step_pause", NavConfig.search_step_pause),
            fwd_speed         = _f("fwd_speed",         NavConfig.fwd_speed),
            align_fwd_speed   = _f("align_fwd_speed",   NavConfig.align_fwd_speed),
            steer_kp          = _f("steer_kp",          NavConfig.steer_kp),
            imu_kp            = _f("imu_kp",            NavConfig.imu_kp),
            align_deadband    = _f("align_deadband",    NavConfig.align_deadband),
            ramp_rate         = _f("ramp_rate",         NavConfig.ramp_rate),
            tag_offset_k      = _f("tag_offset_k",      NavConfig.tag_offset_k),
            pixel_offset_k    = _f("pixel_offset_k",    NavConfig.pixel_offset_k),
        )
        return cls(cfg)

    # ── lifecycle ─────────────────────────────────────────────────────────────

    def start(self):
        """Begin navigation from gate 0."""
        self._gate_id        = 0
        self._cur_left       = 0.0
        self._cur_right      = 0.0
        self._imu_target     = None
        self._pass_heading   = None
        self._last_update    = time.monotonic()
        self._reset_search()
        self._set_state(NavState.SEARCHING)
        log.info("Navigator started — target gate 0")

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

    # ── main update ──────────────────────────────────────────────────────────

    def update(
        self,
        aruco_state: ArUcoState,
        frame_width: int,
        heading: Optional[float] = None,
        yukon=None,
    ) -> Tuple[float, float]:
        """
        Process one camera frame, return (left, right) motor speeds −1..+1.

        Parameters
        ----------
        aruco_state : latest ArUcoState from ArucoDetector.detect()
        frame_width : camera frame width in pixels
        heading     : current IMU heading in degrees (None = no IMU)
        yukon       : _YukonLink instance for set_bearing/clear_bearing,
                      or None to skip onboard bearing hold
        """
        now = time.monotonic()
        dt  = min(now - self._last_update, 0.1)
        self._last_update = now
        self.imu_heading  = heading

        if self._state in (NavState.IDLE, NavState.COMPLETE, NavState.ERROR):
            return 0.0, 0.0

        frame_cx = frame_width // 2

        # ── PASSING ───────────────────────────────────────────────────────────
        # Yukon's onboard bearing PID keeps us straight while we drive forward.
        # Exit after pass_time regardless (prevents stall if next gate is close).
        if self._state == NavState.PASSING:
            if (now - self._pass_start) >= self.cfg.pass_time:
                self._exit_passing(yukon)
                self._advance_gate()
            return self._apply_ramp(self.cfg.fwd_speed, self.cfg.fwd_speed, dt)

        # ── Resolve aim point ─────────────────────────────────────────────────
        target_x, dist, cam_bearing = self._resolve_target(aruco_state, frame_cx)

        self.target_x = target_x
        self.tag_dist = dist

        # ── SEARCHING ─────────────────────────────────────────────────────────
        if target_x is None:
            if self._state != NavState.SEARCHING:
                log.debug("Gate %d lost — searching", self._gate_id)
                self._imu_target = None
                self._set_state(NavState.SEARCHING)
            return self._search_step(heading, dt, now)

        # ── Target visible ────────────────────────────────────────────────────
        # Update the IMU target heading from camera bearing
        if heading is not None and cam_bearing is not None:
            self._imu_target = (heading + cam_bearing) % 360.0

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

        if self._state == NavState.ALIGNING and aligned:
            log.debug("Gate %d aligned (err=%.1f°)", self._gate_id, bearing_err)
            self._set_state(NavState.APPROACHING)

        if self._state == NavState.APPROACHING:
            if dist is not None and dist <= self.cfg.pass_distance:
                log.info("Gate %d PASSING (dist=%.2fm heading=%s)",
                         self._gate_id, dist,
                         f"{heading:.1f}°" if heading is not None else "N/A")
                self._enter_passing(heading, yukon, now)
                return self._apply_ramp(self.cfg.fwd_speed, self.cfg.fwd_speed, dt)

        # ── Motor mixing ──────────────────────────────────────────────────────
        fwd = self.cfg.align_fwd_speed if self._state == NavState.ALIGNING \
              else self.cfg.fwd_speed
        return self._apply_ramp(_clamp(fwd - steer), _clamp(fwd + steer), dt)

    # ── Search rotation ───────────────────────────────────────────────────────

    def _search_step(
        self, heading: Optional[float], dt: float, now: float
    ) -> Tuple[float, float]:
        """Rotate in steps to find the target gate."""

        if heading is None:
            # No IMU — continuous rotation
            return self._apply_ramp(
                -self.cfg.search_speed, self.cfg.search_speed, dt
            )

        # During pause between steps — hold still
        if self._search_pausing:
            if now < self._search_pause_end:
                return self._apply_ramp(0.0, 0.0, dt)
            self._search_pausing = False
            self._search_origin  = heading

        # Initialise step origin
        if self._search_origin is None:
            self._search_origin = heading

        step_turned = abs(_angle_diff(heading, self._search_origin))

        if step_turned >= self.cfg.search_step_deg:
            # Step complete — begin pause
            self._search_turned  += step_turned
            self._search_pausing  = True
            self._search_pause_end = now + self.cfg.search_step_pause
            log.debug("Search step done (%.0f° total)", self._search_turned)
            if self._search_turned >= 360.0:
                log.warning("Gate %d: full 360° without tags, retrying",
                            self._gate_id)
                self._search_turned = 0.0
            return self._apply_ramp(0.0, 0.0, dt)

        return self._apply_ramp(
            -self.cfg.search_speed, self.cfg.search_speed, dt
        )

    def _reset_search(self):
        self._search_origin   = None
        self._search_turned   = 0.0
        self._search_pausing  = False

    # ── PASSING helpers ───────────────────────────────────────────────────────

    def _enter_passing(self, heading: Optional[float], yukon, now: float):
        self._pass_start   = now
        self._pass_heading = heading
        self._set_state(NavState.PASSING)
        if heading is not None and yukon is not None:
            try:
                yukon.set_bearing(heading)
                log.debug("Bearing hold ON at %.1f°", heading)
            except Exception as e:
                log.warning("set_bearing failed: %s", e)

    def _exit_passing(self, yukon):
        if yukon is not None:
            try:
                yukon.clear_bearing()
                log.debug("Bearing hold OFF")
            except Exception as e:
                log.warning("clear_bearing failed: %s", e)
        self._pass_heading = None
        self._imu_target   = None

    # ── Gate advancement ──────────────────────────────────────────────────────

    def _advance_gate(self):
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

    def _resolve_target(
        self,
        state: ArUcoState,
        frame_cx: int,
    ) -> Tuple[Optional[int], Optional[float], Optional[float]]:
        """
        Work out where to aim this frame.

        Returns (target_x_pixels, distance_m, camera_bearing_degrees).
        Priority: full gate → odd tag only → even tag only → None.
        """
        gate_id = self._gate_id
        odd_id  = gate_id * 2 + 1
        even_id = gate_id * 2 + 2

        gate = state.gates.get(gate_id)
        odd  = state.tags.get(odd_id)
        even = state.tags.get(even_id)

        # Full gate visible
        if gate is not None:
            if not gate.correct_dir:
                log.warning("Gate %d: correct_dir=False", gate_id)
            bearing = gate.bearing if gate.bearing is not None \
                      else _pixel_bearing(gate.centre_x, frame_cx)
            return gate.centre_x, gate.distance, bearing

        # Single tag visible
        tag = odd if odd is not None else (even if even is not None else None)
        if tag is None:
            return None, None, None

        is_odd = (tag.id % 2 == 1)
        offset = self._single_tag_offset(tag)
        tx     = tag.center_x + (offset if is_odd else -offset)

        if tag.bearing is not None:
            # Shift bearing by the pixel offset angle
            lateral_norm = (tx - tag.center_x) / max(frame_cx, 1)
            bearing = tag.bearing + math.degrees(math.atan(lateral_norm))
        else:
            bearing = _pixel_bearing(tx, frame_cx)

        return tx, tag.distance, bearing

    def _single_tag_offset(self, tag) -> int:
        """Pixel offset to aim beside a single visible post."""
        if tag.distance is not None and tag.distance > 0.01:
            return int(self.cfg.tag_offset_k / tag.distance)
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
# max_gates         = 10
# tag_size          = 0.15
# pass_distance     = 0.6
# pass_time         = 0.8
# search_speed      = 0.25
# search_step_deg   = 45.0
# search_step_pause = 0.3
# fwd_speed         = 0.45
# align_fwd_speed   = 0.15
# steer_kp          = 0.6
# imu_kp            = 0.5
# align_deadband    = 3.0
# ramp_rate         = 2.0
# tag_offset_k      = 0.4
# pixel_offset_k    = 5000
#
# [aruco]
# enabled    = true
# dict       = DICT_4X4_1000
# calib_file = camera_cal.npz
# tag_size   = 0.15
