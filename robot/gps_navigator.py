#!/usr/bin/env python3
"""
gps_navigator.py — Waypoint GPS navigator with IMU heading hold.

Navigates a differential-drive robot through a sequence of GPS waypoints.
The TAU1308 RTK module outputs at up to 10 Hz; the IMU (BNO085 via Yukon)
runs at 50 Hz.  Between GPS fixes the IMU maintains a stable heading so
the robot drives straight rather than drifting.

Architecture
------------
  GPS fix  → compute bearing + distance to next waypoint
           → set _imu_target heading
  50 Hz loop → steer to _imu_target using IMU heading error (smooth)
             → fall back to GPS bearing if no IMU

State machine
-------------
  IDLE
   │  start() called
   ▼
  WAITING_FIX   — no GPS fix yet, holding still
   │  valid fix received
   ▼
  NAVIGATING    — driving toward current waypoint
   │  within arrival_radius of waypoint
   ▼
  ARRIVED       — brief pause at waypoint
   │  pause elapsed
   ▼
  NAVIGATING    (next waypoint) ──► COMPLETE when all done

  ESTOP / RC override → IDLE from any state

Sensor fusion
-------------
  Bearing to waypoint is computed from GPS lat/lon (Haversine).
  _imu_target is set each time a new GPS fix arrives.
  Between fixes the control loop steers by IMU heading error alone —
  this gives smooth 50 Hz steering vs the jerky 10 Hz GPS-only approach.
  If IMU is absent the loop falls back to GPS bearing directly.

  Distance is always from GPS (IMU has no position).  The robot advances
  toward the waypoint until GPS says it's within arrival_radius.

Waypoint file format (JSON)
---------------------------
  {
    "version": 1,
    "waypoints": [
      {"lat": 52.123456, "lon": -0.123456, "label": "Start"},
      {"lat": 52.123789, "lon": -0.123789, "label": "Gate 1"},
      ...
    ]
  }

  Waypoints can also be passed directly as a list of dicts or
  (lat, lon) tuples to GpsNavigator.load_waypoints().

Look-ahead smoothing
--------------------
  When within lookahead_m metres of the current waypoint (and a next
  waypoint exists), the target bearing is blended toward the next
  waypoint's bearing.  Blend fraction = 1 − (dist / lookahead_m), so
  the turn starts gradually and the robot arrives already facing the
  right direction.

LiDAR obstacle stop
--------------------
  If a LidarScan is passed to update(), the navigator halts whenever any
  point within the forward cone (obstacle_cone_deg wide) is closer than
  obstacle_stop_dist metres.

Configuration (robot.ini [gps_navigator])
-----------------------------------------
  waypoints_file    = waypoints.json
  arrival_radius    = 2.0     # metres — waypoint reached threshold
  min_fix_quality   = 1       # minimum fix quality to navigate (1=GPS, 4=RTK)
  fwd_speed         = 0.5     # forward speed (0–1)
  steer_kp          = 0.8     # proportional steering gain on heading error
  steer_max         = 0.7     # maximum steering authority (clamps steer output)
  arrival_pause     = 1.0     # seconds to pause at each waypoint
  ramp_rate         = 2.0     # motor ramp rate (speed units/second)
  imu_kp            = 1.0     # IMU heading error gain (usually = steer_kp)
  spin_speed        = 0.3     # rotation speed when bearing error is large
  spin_threshold    = 45.0    # degrees — rotate on the spot if error > this
  loop              = false   # repeat waypoints from the beginning when done
  lookahead_m       = 5.0     # metres — start blending toward next wp bearing
  obstacle_stop_dist = 0.5    # metres — halt if obstacle detected closer than this
  obstacle_cone_deg  = 60.0   # forward cone full width for obstacle check
"""

import json
import logging
import math
import time
from configparser import ConfigParser
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import List, Optional, Tuple

log = logging.getLogger(__name__)

# ── Earth geometry ────────────────────────────────────────────────────────────

_EARTH_R = 6_371_000.0   # metres


def _haversine(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Return distance in metres between two WGS-84 positions."""
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi  = math.radians(lat2 - lat1)
    dlam  = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlam/2)**2
    return 2 * _EARTH_R * math.asin(math.sqrt(a))


def _bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Return initial bearing in degrees (0=N, 90=E) from point 1 to point 2."""
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dlam       = math.radians(lon2 - lon1)
    x = math.sin(dlam) * math.cos(phi2)
    y = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlam)
    return (math.degrees(math.atan2(x, y)) + 360) % 360


def _angle_diff(target: float, current: float) -> float:
    """Signed shortest-arc difference (target − current), range −180..+180."""
    return (target - current + 180.0) % 360.0 - 180.0


# ── Waypoint ──────────────────────────────────────────────────────────────────

@dataclass
class Waypoint:
    lat:   float
    lon:   float
    label: str = ""

    @classmethod
    def from_dict(cls, d) -> "Waypoint":
        return cls(lat=float(d["lat"]), lon=float(d["lon"]),
                   label=str(d.get("label", "")))

    @classmethod
    def from_tuple(cls, t) -> "Waypoint":
        return cls(lat=float(t[0]), lon=float(t[1]))


# ── State ─────────────────────────────────────────────────────────────────────

class GpsNavState(Enum):
    IDLE        = auto()
    WAITING_FIX = auto()   # waiting for a valid GPS fix before moving
    NAVIGATING  = auto()   # driving toward current waypoint
    ARRIVED     = auto()   # pausing at waypoint before advancing
    COMPLETE    = auto()   # all waypoints visited
    ERROR       = auto()


# ── Config ────────────────────────────────────────────────────────────────────

@dataclass
class GpsNavConfig:
    waypoints_file:   str   = "waypoints.json"
    arrival_radius:   float = 2.0
    min_fix_quality:  int   = 1
    fwd_speed:        float = 0.5
    steer_kp:         float = 0.8
    steer_max:        float = 0.7
    arrival_pause:    float = 1.0
    ramp_rate:        float = 2.0
    imu_kp:           float = 1.0
    spin_speed:       float = 0.3
    spin_threshold:   float = 45.0
    loop:             bool  = False
    lookahead_m:      float = 5.0   # metres — blend toward next wp bearing when within this distance
    obstacle_stop_dist: float = 0.5  # metres — halt if LiDAR detects obstacle closer than this
    obstacle_cone_deg:  float = 60.0 # forward cone full width for LiDAR check


# ── Helpers ───────────────────────────────────────────────────────────────────

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


def _clamp(v: float, lo: float = -1.0, hi: float = 1.0) -> float:
    return max(lo, min(hi, v))


def _ramp(current: float, target: float, max_delta: float) -> float:
    diff = target - current
    if abs(diff) <= max_delta:
        return target
    return current + (max_delta if diff > 0 else -max_delta)


# ── Navigator ─────────────────────────────────────────────────────────────────

class GpsNavigator:
    """
    GPS waypoint navigator with IMU heading hold.

    Typical usage in robot.py control thread::

        gps_state = robot._gps.get_state() if robot._gps else GpsState()
        imu_heading = robot.get_heading()
        left, right = gps_nav.update(gps_state, imu_heading, yukon=robot._yukon)
        robot.drive(left, right)

    Waypoints can be loaded from a JSON file or set programmatically::

        nav = GpsNavigator.from_ini("robot.ini")
        nav.load_waypoints("waypoints.json")
        nav.start()
    """

    def __init__(self, cfg: GpsNavConfig = None):
        self.cfg         = cfg or GpsNavConfig()
        self._state      = GpsNavState.IDLE
        self._waypoints: List[Waypoint] = []
        self._wp_idx     = 0
        self._arrive_t   = 0.0
        self._last_update= time.monotonic()

        # Sensor fusion state
        self._imu_target:   Optional[float] = None   # target heading set from GPS bearing
        self._gps_bearing:  Optional[float] = None   # last computed GPS bearing to waypoint
        self._gps_distance: Optional[float] = None   # last computed distance to waypoint
        self._last_gps_ts:  float           = 0.0    # timestamp of last GPS fix used

        # Ramped motor outputs
        self._cur_left  = 0.0
        self._cur_right = 0.0

        # Diagnostics — readable externally
        self.bearing_to_wp:  Optional[float] = None
        self.distance_to_wp: Optional[float] = None
        self.imu_heading:    Optional[float] = None
        self.heading_err:    Optional[float] = None

    # ── factory ──────────────────────────────────────────────────────────────

    @classmethod
    def from_ini(cls, path: str, section: str = "gps_navigator") -> "GpsNavigator":
        cp = ConfigParser(inline_comment_prefixes=('#', ';'))
        cp.read(path)
        sec = cp[section] if cp.has_section(section) else {}

        def _f(k, d): return float(sec.get(k, d))
        def _i(k, d): return int(sec.get(k, d))
        def _b(k, d): return sec.get(k, str(d)).lower() == "true"

        cfg = GpsNavConfig(
            waypoints_file    = sec.get("waypoints_file", GpsNavConfig.waypoints_file),
            arrival_radius    = _f("arrival_radius",   GpsNavConfig.arrival_radius),
            min_fix_quality   = _i("min_fix_quality",  GpsNavConfig.min_fix_quality),
            fwd_speed         = _f("fwd_speed",         GpsNavConfig.fwd_speed),
            steer_kp          = _f("steer_kp",          GpsNavConfig.steer_kp),
            steer_max         = _f("steer_max",          GpsNavConfig.steer_max),
            arrival_pause     = _f("arrival_pause",     GpsNavConfig.arrival_pause),
            ramp_rate         = _f("ramp_rate",         GpsNavConfig.ramp_rate),
            imu_kp            = _f("imu_kp",            GpsNavConfig.imu_kp),
            spin_speed        = _f("spin_speed",        GpsNavConfig.spin_speed),
            spin_threshold    = _f("spin_threshold",    GpsNavConfig.spin_threshold),
            loop              = _b("loop",              GpsNavConfig.loop),
            lookahead_m       = _f("lookahead_m",       GpsNavConfig.lookahead_m),
            obstacle_stop_dist = _f("obstacle_stop_dist", GpsNavConfig.obstacle_stop_dist),
            obstacle_cone_deg  = _f("obstacle_cone_deg",  GpsNavConfig.obstacle_cone_deg),
        )
        nav = cls(cfg)

        # Auto-load waypoints file if specified
        wf = cfg.waypoints_file.strip()
        if wf and wf != "waypoints.json":
            try:
                nav.load_waypoints(wf)
            except Exception as e:
                log.warning("Could not auto-load waypoints from %s: %s", wf, e)

        return nav

    # ── waypoint management ───────────────────────────────────────────────────

    def load_waypoints(self, source):
        """
        Load waypoints from a JSON file path, a list of dicts, or a list of
        (lat, lon) tuples.  Clears any existing waypoints.
        """
        if isinstance(source, str):
            with open(source) as f:
                data = json.load(f)
            self._waypoints = [Waypoint.from_dict(d) for d in data['waypoints']]
            log.info("Loaded %d waypoints from %s", len(self._waypoints), source)
        else:
            wps = []
            for item in source:
                if isinstance(item, dict):
                    wps.append(Waypoint.from_dict(item))
                elif isinstance(item, (list, tuple)) and len(item) >= 2:
                    wps.append(Waypoint.from_tuple(item))
                else:
                    raise ValueError(f"Unrecognised waypoint format: {item!r}")
            self._waypoints = wps
            log.info("Loaded %d waypoints", len(self._waypoints))

    def add_waypoint(self, lat: float, lon: float, label: str = ""):
        """Append a single waypoint."""
        self._waypoints.append(Waypoint(lat=lat, lon=lon, label=label))

    def clear_waypoints(self):
        self._waypoints.clear()

    @property
    def waypoints(self) -> List[Waypoint]:
        return list(self._waypoints)

    @property
    def current_waypoint(self) -> Optional[Waypoint]:
        if 0 <= self._wp_idx < len(self._waypoints):
            return self._waypoints[self._wp_idx]
        return None

    @property
    def waypoint_index(self) -> int:
        return self._wp_idx

    # ── lifecycle ─────────────────────────────────────────────────────────────

    def start(self):
        """Begin navigation from the first waypoint."""
        if not self._waypoints:
            log.error("No waypoints loaded — cannot start GPS navigator")
            self._set_state(GpsNavState.ERROR)
            return
        self._wp_idx     = 0
        self._cur_left   = 0.0
        self._cur_right  = 0.0
        self._imu_target = None
        self._last_update= time.monotonic()
        self._set_state(GpsNavState.WAITING_FIX)
        log.info("GPS navigator started — %d waypoints", len(self._waypoints))

    def stop(self):
        """Halt and return to IDLE."""
        self._set_state(GpsNavState.IDLE)
        self._cur_left   = 0.0
        self._cur_right  = 0.0
        self._imu_target = None
        log.info("GPS navigator stopped")

    @property
    def state(self) -> GpsNavState:
        return self._state

    # ── main update ──────────────────────────────────────────────────────────

    def update(
        self,
        gps,                          # GpsState from robot._gps.get_state()
        imu_heading: Optional[float] = None,
        yukon=None,                   # _YukonLink for set_bearing/clear_bearing
        lidar=None,                   # LidarScan for obstacle avoidance, or None
    ) -> Tuple[float, float]:
        """
        Called at the control loop rate (50 Hz).  Returns (left, right) −1..+1.

        Parameters
        ----------
        gps         : latest GpsState snapshot
        imu_heading : current IMU heading in degrees, or None
        yukon       : _YukonLink instance for bearing hold, or None
        lidar       : LidarScan snapshot for obstacle avoidance, or None
        """
        now = time.monotonic()
        dt  = min(now - self._last_update, 0.1)
        self._last_update = now
        self.imu_heading  = imu_heading

        if self._state in (GpsNavState.IDLE, GpsNavState.COMPLETE, GpsNavState.ERROR):
            return 0.0, 0.0

        wp = self.current_waypoint
        if wp is None:
            self._set_state(GpsNavState.COMPLETE)
            return 0.0, 0.0

        # ── GPS fix quality check ─────────────────────────────────────────────
        has_fix = (gps.fix and
                   gps.latitude is not None and
                   gps.longitude is not None and
                   gps.fix_quality >= self.cfg.min_fix_quality)

        # ── WAITING_FIX ───────────────────────────────────────────────────────
        if self._state == GpsNavState.WAITING_FIX:
            if has_fix:
                log.info("GPS fix acquired (quality=%s) — navigating to wp %d: %s",
                         gps.fix_quality_name, self._wp_idx,
                         wp.label or f"{wp.lat:.6f},{wp.lon:.6f}")
                self._set_state(GpsNavState.NAVIGATING)
            else:
                return self._apply_ramp(0.0, 0.0, dt)

        # ── ARRIVED — pause before advancing ─────────────────────────────────
        if self._state == GpsNavState.ARRIVED:
            if now - self._arrive_t >= self.cfg.arrival_pause:
                self._advance_waypoint(yukon)
            return self._apply_ramp(0.0, 0.0, dt)

        # ── LiDAR obstacle check ──────────────────────────────────────────────
        if (lidar is not None and
                not _lidar_clear(lidar,
                                 self.cfg.obstacle_cone_deg / 2.0,
                                 self.cfg.obstacle_stop_dist)):
            log.warning("Obstacle in forward cone — halting (wp %d)", self._wp_idx)
            return self._apply_ramp(0.0, 0.0, dt)

        # ── NAVIGATING ────────────────────────────────────────────────────────

        # Update GPS bearing + distance on every new fix
        if has_fix and gps.timestamp != self._last_gps_ts:
            self._last_gps_ts   = gps.timestamp
            dist = _haversine(gps.latitude, gps.longitude, wp.lat, wp.lon)
            bear = _bearing (gps.latitude, gps.longitude, wp.lat, wp.lon)

            self._gps_distance  = dist
            self._gps_bearing   = bear
            self.distance_to_wp = dist
            self.bearing_to_wp  = bear

            # Look-ahead smoothing: when close to the current waypoint, blend
            # in the bearing toward the next waypoint so the robot starts
            # turning early and avoids oscillation at waypoint transitions.
            next_idx = self._wp_idx + 1
            if (self.cfg.lookahead_m > 0 and
                    dist < self.cfg.lookahead_m and
                    next_idx < len(self._waypoints)):
                nwp     = self._waypoints[next_idx]
                nb      = _bearing(wp.lat, wp.lon, nwp.lat, nwp.lon)
                blend   = 1.0 - (dist / self.cfg.lookahead_m)
                bear    = (bear + blend * _angle_diff(nb, bear)) % 360.0
                log.debug("Look-ahead blend=%.2f toward wp %d: target=%.1f°",
                          blend, next_idx, bear)

            # Update IMU target from (possibly blended) bearing
            self._imu_target = bear
            log.debug("WP %d: dist=%.1fm bear=%.1f°", self._wp_idx, dist, bear)

            # Arrived?
            if dist <= self.cfg.arrival_radius:
                log.info("WP %d reached (dist=%.1fm)", self._wp_idx, dist)
                self._arrive_t = now
                if yukon:
                    try: yukon.clear_bearing()
                    except (AttributeError, OSError): pass
                self._set_state(GpsNavState.ARRIVED)
                return self._apply_ramp(0.0, 0.0, dt)

        elif not has_fix:
            # Lost fix — keep heading but log it
            log.debug("GPS fix lost — holding IMU heading")

        # ── Compute heading error ─────────────────────────────────────────────
        #
        # Priority:
        #   1. IMU heading error against _imu_target  (smooth, 50 Hz)
        #   2. GPS bearing directly                   (10 Hz, jerky)
        #   3. No info → drive straight

        if imu_heading is not None and self._imu_target is not None:
            heading_err = _angle_diff(self._imu_target, imu_heading)
            source      = "IMU"
        elif self._gps_bearing is not None:
            # Approximate heading error from GPS course (if available)
            if gps.heading is not None:
                heading_err = _angle_diff(self._gps_bearing, gps.heading)
            else:
                heading_err = 0.0   # can't steer, just go forward
            source = "GPS"
        else:
            heading_err = 0.0
            source      = "none"

        self.heading_err = heading_err

        # ── Motor output ──────────────────────────────────────────────────────
        abs_err = abs(heading_err)

        if abs_err > self.cfg.spin_threshold:
            # Large error — spin on the spot
            sign = 1.0 if heading_err > 0 else -1.0
            target_left  = -self.cfg.spin_speed * sign
            target_right =  self.cfg.spin_speed * sign
            log.debug("Spinning %.0f° (%s)", heading_err, source)

            # Engage bearing hold for the spin so Yukon PID assists
            if imu_heading is not None and self._imu_target is not None and yukon:
                try: yukon.set_bearing(self._imu_target)
                except (AttributeError, OSError): pass

        else:
            # Normal approach — proportional steering
            steer = _clamp(
                self.cfg.steer_kp * heading_err / 90.0,
                -self.cfg.steer_max, self.cfg.steer_max
            )
            target_left  = _clamp(self.cfg.fwd_speed - steer)
            target_right = _clamp(self.cfg.fwd_speed + steer)

            # Engage Yukon bearing hold for straight sections
            if imu_heading is not None and self._imu_target is not None and yukon:
                try: yukon.set_bearing(self._imu_target)
                except (AttributeError, OSError): pass

        return self._apply_ramp(target_left, target_right, dt)

    # ── helpers ───────────────────────────────────────────────────────────────

    def _advance_waypoint(self, yukon=None):
        next_idx = self._wp_idx + 1
        if next_idx >= len(self._waypoints):
            if self.cfg.loop:
                log.info("All waypoints complete — looping back to start")
                self._wp_idx     = 0
                self._imu_target = None
                self._set_state(GpsNavState.WAITING_FIX)
            else:
                log.info("All %d waypoints complete!", len(self._waypoints))
                if yukon:
                    try: yukon.clear_bearing()
                    except (AttributeError, OSError): pass
                self._set_state(GpsNavState.COMPLETE)
        else:
            wp = self._waypoints[next_idx]
            log.info("Advancing to wp %d: %s", next_idx,
                     wp.label or f"{wp.lat:.6f},{wp.lon:.6f}")
            self._wp_idx     = next_idx
            self._imu_target = None   # will be set on next GPS fix
            self._set_state(GpsNavState.NAVIGATING)

    def _apply_ramp(self, tl: float, tr: float, dt: float) -> Tuple[float, float]:
        max_d = self.cfg.ramp_rate * dt
        self._cur_left  = _ramp(self._cur_left,  tl, max_d)
        self._cur_right = _ramp(self._cur_right, tr, max_d)
        return self._cur_left, self._cur_right

    def _set_state(self, new: GpsNavState):
        if new != self._state:
            log.info("GPS nav: %s → %s  (wp %d/%d)",
                     self._state.name, new.name,
                     self._wp_idx, len(self._waypoints))
            self._state = new

    # ── status summary ────────────────────────────────────────────────────────

    def status_str(self) -> str:
        """One-line status for display in GUIs."""
        wp = self.current_waypoint
        wp_lbl = wp.label if (wp and wp.label) else f"WP{self._wp_idx}"
        dist = f"{self.distance_to_wp:.1f}m" if self.distance_to_wp is not None else "--m"
        bear = f"{self.bearing_to_wp:.0f}°"  if self.bearing_to_wp  is not None else "--°"
        err  = f"{self.heading_err:+.1f}°"   if self.heading_err    is not None else "--°"
        hdg  = f"{self.imu_heading:.0f}°"    if self.imu_heading    is not None else "no IMU"
        return (f"{self._state.name}  {wp_lbl}  "
                f"dist={dist}  bear={bear}  err={err}  imu={hdg}")


# ── Waypoint file helpers ─────────────────────────────────────────────────────

def save_waypoints(waypoints: List[Waypoint], path: str):
    """Save a list of Waypoint objects to a JSON file (versioned format)."""
    data = {"version": 1,
            "waypoints": [{"lat": wp.lat, "lon": wp.lon, "label": wp.label}
                          for wp in waypoints]}
    with open(path, "w") as f:
        json.dump(data, f, indent=2)
    log.info("Saved %d waypoints to %s", len(waypoints), path)


def load_waypoints(path: str) -> List[Waypoint]:
    """Load waypoints from a JSON file."""
    with open(path) as f:
        data = json.load(f)
    return [Waypoint.from_dict(d) for d in data['waypoints']]


# ── robot.ini [gps_navigator] reference ──────────────────────────────────────
#
# [gps_navigator]
# waypoints_file    = waypoints.json
# arrival_radius    = 2.0
# min_fix_quality   = 1
# fwd_speed         = 0.5
# steer_kp          = 0.8
# steer_max         = 0.7
# arrival_pause     = 1.0
# ramp_rate         = 2.0
# imu_kp            = 1.0
# spin_speed        = 0.3
# spin_threshold    = 45.0
# loop              = false
# lookahead_m       = 5.0    # metres — blend toward next wp bearing when within this distance
# obstacle_stop_dist = 0.5   # metres — halt if LiDAR detects obstacle closer than this
# obstacle_cone_deg  = 60.0  # forward cone full width for LiDAR obstacle check
