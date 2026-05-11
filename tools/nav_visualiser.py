#!/usr/bin/env python3
"""
nav_visualiser.py — Real-time overhead view of robot navigation.

Connects to a running robot_daemon / Robot instance (via direct import or
the --sim PTY mode) and draws a top-down 2D view showing:

  • Robot body with heading arrow
  • Visible ArUco tags (circles, labelled with ID and distance)
  • Gate centre markers and gate lines
  • Camera bearing cone from the robot
  • Planned aim point (target_x projected as a direction ray)
  • Dead-reckoned trail of robot positions
  • Navigator state, gate ID, bearing error
  • Motor bar indicators (left / right)
  • IMU heading compass
  • LiDAR obstacle returns (if available)

Three run modes
---------------
  --live        Poll the /api/state SSE stream of a running frontend
                (robot_web.py on :5000, or robot_mobile.py on :5001).
                No second Robot instance is created — read-only.
                Use --url to point at a different address or port.

  --sim         Spin up an internal closed-loop simulation: a synthetic
                navigator + fake ArUco state driven by scripted tag layouts.
                Useful for pure unit / visual testing with zero hardware.

  --udp         Listen for JSON telemetry packets broadcast by
                test_tag_approach.py (or any other script that calls
                VisEmitter).  Run the test in one terminal, the visualiser
                in another — they share no ports or Robot instances.

Usage
-----
  # Alongside robot_web.py (default port 5000):
  python3 tools/nav_visualiser.py --live

  # Alongside robot_mobile.py (port 5001):
  python3 tools/nav_visualiser.py --live --url http://localhost:5001/api/state

  # Receive live telemetry from test_tag_approach.py:
  python3 tools/nav_visualiser.py --udp

  # UDP on a different port or from a remote Pi:
  python3 tools/nav_visualiser.py --udp --udp-port 5555 --udp-host 192.168.1.42

  # Simulation mode (no hardware at all):
  python3 tools/nav_visualiser.py --sim

  # Sim with custom gate layout:
  python3 tools/nav_visualiser.py --sim --gates 5

Keys
----
  Q / Esc     : quit
  R           : reset dead-reckoning trail
  +  / -      : zoom in / out
  Arrow keys  : pan view
  S           : toggle trail
  H           : toggle heading-hold cone
  G           : toggle gate lines
  L           : toggle LiDAR returns
  Space       : pause / resume (sim mode only)
"""

import argparse
import json
import math
import os
import socket
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import pygame

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# ── Optional live import ──────────────────────────────────────────────────────

def _try_import_robot():
    try:
        from robot_daemon import Robot, RobotMode   # noqa: F401
        from robot.aruco_detector import ArUcoState, ArUcoTag
        from robot.aruco_navigator import ArucoNavigator, NavConfig, NavState, ArUcoGate
        return True
    except ImportError:
        return False


# ── Colours ───────────────────────────────────────────────────────────────────

C_BG          = ( 15,  20,  30)
C_GRID        = ( 30,  40,  55)
C_ROBOT       = ( 60, 200, 100)
C_ROBOT_DARK  = ( 20,  80,  40)
C_HEADING     = (255, 255,  60)
C_TAG_ODD     = ( 60, 140, 255)
C_TAG_EVEN    = (255, 140,  60)
C_TAG_LABEL   = (220, 220, 220)
C_GATE_LINE   = ( 60, 220, 220)
C_GATE_OK     = ( 60, 220,  80)
C_GATE_WRONG  = (220,  60,  60)
C_GATE_LABEL  = (200, 220, 255)
C_AIM_RAY     = (255, 220,   0)
C_CAM_CONE    = ( 80, 100,  60)
C_TRAIL       = ( 40, 100, 160)
C_TRAIL_OLD   = ( 25,  50,  80)
C_LIDAR       = (200,  60,  60)
C_PANEL_BG    = ( 22,  28,  42)
C_PANEL_BD    = ( 50,  65,  90)
C_WHITE       = (230, 230, 240)
C_GRAY        = (120, 130, 150)
C_CYAN        = ( 60, 200, 220)
C_ORANGE      = (240, 160,  40)
C_RED         = (220,  60,  60)
C_GREEN       = ( 60, 220,  80)
C_YELLOW      = (240, 200,  40)

STATE_COLOURS = {
    "IDLE":        C_GRAY,
    "SEARCHING":   C_YELLOW,
    "ALIGNING":    C_ORANGE,
    "APPROACHING": C_GREEN,
    "PASSING":     C_CYAN,
    "RECOVERING":  C_RED,
    "COMPLETE":    C_GREEN,
    "ERROR":       C_RED,
}

# ── Layout ────────────────────────────────────────────────────────────────────

W, H        = 1200, 800
PANEL_W     = 280          # right-side info panel width
VIEW_W      = W - PANEL_W  # viewport for the overhead map

PIXELS_PER_M_DEFAULT = 80   # 1 m = 80 px at default zoom

# ── Fonts ─────────────────────────────────────────────────────────────────────

pygame.init()
FONT_SM  = pygame.font.SysFont("monospace", 13)
FONT_MD  = pygame.font.SysFont("monospace", 15, bold=True)
FONT_LG  = pygame.font.SysFont("monospace", 20, bold=True)
FONT_XL  = pygame.font.SysFont("monospace", 28, bold=True)

# ── Simulation data structures ────────────────────────────────────────────────

@dataclass
class TagLayout:
    """A physical ArUco tag placed in the simulated arena (world coords, metres)."""
    id:  int
    x:   float   # world x (right = positive)
    y:   float   # world y (up = positive in world, down = positive on screen)


@dataclass
class VisFrame:
    """
    Everything the visualiser needs for one display frame.
    This is the exchange type between the data source (live or sim) and the renderer.
    """
    # Robot pose (world coords)
    robot_x:       float = 0.0
    robot_y:       float = 0.0
    heading_deg:   float = 0.0          # 0 = North (up), clockwise positive

    # Motor outputs
    left_speed:    float = 0.0
    right_speed:   float = 0.0

    # Navigator status
    nav_state:     str   = "IDLE"
    gate_id:       int   = 0
    bearing_err:   Optional[float] = None   # degrees
    imu_target:    Optional[float] = None   # degrees

    # ArUco detections  (world coords if available, else robot-relative)
    tags:          Dict[int, Tuple[float, float, float, float]] = field(
        default_factory=dict)
    # tag_id → (world_x, world_y, distance_m, bearing_deg)

    gates:         Dict[int, Tuple[float, float]] = field(
        default_factory=dict)
    # gate_id → (world_x, world_y)

    aim_bearing:   Optional[float] = None   # camera bearing to aim point
    cam_fov_deg:   float = 62.0             # horizontal FOV

    # LiDAR (robot-relative: angle in degrees, distance in metres)
    lidar_angles:  List[float] = field(default_factory=list)
    lidar_dists:   List[float] = field(default_factory=list)

    # Timestamp
    ts:            float = 0.0


# ── Coordinate transform ──────────────────────────────────────────────────────

class ViewTransform:
    """World ↔ screen coordinate transforms with pan and zoom."""

    def __init__(self, ppm: float = PIXELS_PER_M_DEFAULT):
        self.ppm    = ppm        # pixels per metre
        self.origin = [VIEW_W // 2, H // 2]   # screen pixel for world (0,0)

    def w2s(self, wx: float, wy: float) -> Tuple[int, int]:
        """World coords → screen pixels."""
        sx = int(self.origin[0] + wx * self.ppm)
        sy = int(self.origin[1] - wy * self.ppm)   # y-flip
        return sx, sy

    def s2w(self, sx: int, sy: int) -> Tuple[float, float]:
        wx = (sx - self.origin[0]) / self.ppm
        wy = -(sy - self.origin[1]) / self.ppm
        return wx, wy

    def zoom(self, factor: float):
        self.ppm = max(20.0, min(300.0, self.ppm * factor))

    def pan(self, dx: int, dy: int):
        self.origin[0] += dx
        self.origin[1] += dy

    def reset(self):
        self.origin = [VIEW_W // 2, H // 2]
        self.ppm    = PIXELS_PER_M_DEFAULT


# ── Drawing helpers ───────────────────────────────────────────────────────────

def _heading_vec(heading_deg: float) -> Tuple[float, float]:
    """Unit vector in the direction of heading (North=0°, clockwise)."""
    r = math.radians(heading_deg)
    return math.sin(r), -math.cos(r)   # screen coords: x right, y down


def _draw_robot(surf: pygame.Surface, tx: ViewTransform,
                frame: VisFrame):
    sx, sy  = tx.w2s(frame.robot_x, frame.robot_y)
    radius  = max(8, int(tx.ppm * 0.18))

    # Body
    pygame.draw.circle(surf, C_ROBOT_DARK, (sx, sy), radius)
    pygame.draw.circle(surf, C_ROBOT,      (sx, sy), radius, 2)

    # Heading arrow
    hx, hy = _heading_vec(frame.heading_deg)
    arrow_len = radius + int(tx.ppm * 0.25)
    ex = sx + int(hx * arrow_len)
    ey = sy + int(hy * arrow_len)
    pygame.draw.line(surf, C_HEADING, (sx, sy), (ex, ey), 3)
    # arrowhead
    perp = (-hy * 0.3, hx * 0.3)
    tip_len = max(6, radius // 2)
    p1 = (ex + int(perp[0] * tip_len), ey + int(perp[1] * tip_len))
    p2 = (ex - int(perp[0] * tip_len), ey - int(perp[1] * tip_len))
    pygame.draw.polygon(surf, C_HEADING, [(ex, ey), p1, p2])


def _draw_camera_cone(surf: pygame.Surface, tx: ViewTransform,
                      frame: VisFrame, show_cone: bool):
    if not show_cone:
        return
    sx, sy = tx.w2s(frame.robot_x, frame.robot_y)
    fov    = frame.cam_fov_deg
    reach  = int(tx.ppm * 3.5)
    base   = frame.heading_deg

    # Semi-transparent filled cone
    cone_surf = pygame.Surface((VIEW_W, H), pygame.SRCALPHA)
    left_deg  = base - fov / 2
    right_deg = base + fov / 2
    lx, ly    = _heading_vec(left_deg)
    rx, ry    = _heading_vec(right_deg)
    pts       = [
        (sx, sy),
        (sx + int(lx * reach), sy + int(ly * reach)),
        (sx + int(rx * reach), sy + int(ry * reach)),
    ]
    pygame.draw.polygon(cone_surf, (*C_CAM_CONE, 50), pts)
    surf.blit(cone_surf, (0, 0))

    # Cone edges
    pygame.draw.line(surf, C_CAM_CONE,
                     (sx, sy),
                     (sx + int(lx * reach), sy + int(ly * reach)), 1)
    pygame.draw.line(surf, C_CAM_CONE,
                     (sx, sy),
                     (sx + int(rx * reach), sy + int(ry * reach)), 1)

    # Aim ray
    if frame.aim_bearing is not None:
        aim_deg = frame.heading_deg + frame.aim_bearing
        ax, ay  = _heading_vec(aim_deg)
        aim_len = int(tx.ppm * 3.0)
        pygame.draw.line(surf, C_AIM_RAY,
                         (sx, sy),
                         (sx + int(ax * aim_len), sy + int(ay * aim_len)), 2)


def _draw_tags(surf: pygame.Surface, tx: ViewTransform, frame: VisFrame):
    for tid, (wx, wy, dist, bear) in frame.tags.items():
        sx, sy = tx.w2s(wx, wy)
        colour = C_TAG_ODD if tid % 2 == 1 else C_TAG_EVEN
        r      = max(6, int(tx.ppm * 0.12))

        pygame.draw.circle(surf, colour, (sx, sy), r)
        pygame.draw.circle(surf, C_WHITE, (sx, sy), r, 1)

        label = f"T{tid}"
        if dist > 0:
            label += f"\n{dist:.2f}m"
        for i, line in enumerate(label.split("\n")):
            t = FONT_SM.render(line, True, C_TAG_LABEL)
            surf.blit(t, (sx + r + 3, sy - 8 + i * 14))


def _draw_gates(surf: pygame.Surface, tx: ViewTransform,
                frame: VisFrame, show_gates: bool):
    if not show_gates:
        return
    for gid, (wx, wy) in frame.gates.items():
        sx, sy = tx.w2s(wx, wy)
        colour = C_GATE_OK

        # Cross marker at gate centre
        cs = max(8, int(tx.ppm * 0.12))
        pygame.draw.line(surf, colour, (sx - cs, sy), (sx + cs, sy), 2)
        pygame.draw.line(surf, colour, (sx, sy - cs), (sx, sy + cs), 2)

        # Gate label
        lbl = FONT_SM.render(f"G{gid}", True, C_GATE_LABEL)
        surf.blit(lbl, (sx + 6, sy - 16))


def _draw_trail(surf: pygame.Surface, tx: ViewTransform,
                trail: deque, show_trail: bool):
    if not show_trail or len(trail) < 2:
        return
    pts = list(trail)
    for i in range(1, len(pts)):
        age_frac = i / len(pts)
        c = tuple(int(a + (b - a) * age_frac)
                  for a, b in zip(C_TRAIL_OLD, C_TRAIL))
        a = tx.w2s(*pts[i - 1])
        b = tx.w2s(*pts[i])
        if 0 < a[0] < VIEW_W and 0 < b[0] < VIEW_W:
            pygame.draw.line(surf, c, a, b, 2)


def _draw_lidar(surf: pygame.Surface, tx: ViewTransform,
                frame: VisFrame, show_lidar: bool):
    if not show_lidar:
        return
    rx, ry  = frame.robot_x, frame.robot_y
    base    = frame.heading_deg
    for ang, dist in zip(frame.lidar_angles, frame.lidar_dists):
        world_ang = base + ang
        lx, ly    = _heading_vec(world_ang)
        wx = rx + lx * dist
        wy = ry - ly * dist   # sign: heading_vec y already flipped
        # Recompute properly:
        rad  = math.radians(world_ang)
        wx   = rx + math.sin(rad) * dist
        wy   = ry + math.cos(rad) * dist
        sx, sy = tx.w2s(wx, wy)
        if 0 < sx < VIEW_W and 0 < sy < H:
            pygame.draw.circle(surf, C_LIDAR, (sx, sy), 2)


def _draw_grid(surf: pygame.Surface, tx: ViewTransform):
    # Minor 0.5 m
    cols = int(VIEW_W / (tx.ppm * 0.5)) + 2
    rows = int(H      / (tx.ppm * 0.5)) + 2
    ox, oy = int(tx.origin[0] % (tx.ppm * 0.5)), int(tx.origin[1] % (tx.ppm * 0.5))
    for i in range(-1, cols + 1):
        x = ox + int(i * tx.ppm * 0.5)
        pygame.draw.line(surf, C_GRID, (x, 0), (x, H), 1)
    for j in range(-1, rows + 1):
        y = oy + int(j * tx.ppm * 0.5)
        pygame.draw.line(surf, C_GRID, (0, y), (VIEW_W, y), 1)

    # Origin cross
    ox2, oy2 = tx.w2s(0, 0)
    pygame.draw.line(surf, (50, 60, 80), (ox2 - 20, oy2), (ox2 + 20, oy2), 1)
    pygame.draw.line(surf, (50, 60, 80), (ox2, oy2 - 20), (ox2, oy2 + 20), 1)


# ── Info panel ────────────────────────────────────────────────────────────────

def _panel_rect() -> pygame.Rect:
    return pygame.Rect(VIEW_W, 0, PANEL_W, H)


def _draw_panel(surf: pygame.Surface, frame: VisFrame, fps: float):
    r = _panel_rect()
    pygame.draw.rect(surf, C_PANEL_BG, r)
    pygame.draw.line(surf, C_PANEL_BD, (VIEW_W, 0), (VIEW_W, H), 2)

    x   = VIEW_W + 12
    y   = 14
    dy  = 18

    def txt(text, color=C_WHITE, font=FONT_SM):
        nonlocal y
        surf.blit(font.render(text, True, color), (x, y))
        y += dy

    def sep():
        nonlocal y
        pygame.draw.line(surf, C_PANEL_BD, (VIEW_W + 6, y), (W - 6, y), 1)
        y += 6

    # Title
    surf.blit(FONT_LG.render("NAV VISUALISER", True, C_CYAN), (x, y))
    y += 28
    sep()

    # Navigator state
    state_c = STATE_COLOURS.get(frame.nav_state, C_GRAY)
    surf.blit(FONT_MD.render("STATE", True, C_GRAY), (x, y))
    y += 18
    surf.blit(FONT_LG.render(frame.nav_state, True, state_c), (x, y))
    y += 26

    txt(f"Gate target : {frame.gate_id}", C_WHITE)
    err_str = f"{frame.bearing_err:+.1f}°" if frame.bearing_err is not None else "—"
    txt(f"Bearing err : {err_str}",
        C_GREEN if (frame.bearing_err is not None and abs(frame.bearing_err) < 5)
        else C_ORANGE)
    imu_str = f"{frame.imu_target:.1f}°" if frame.imu_target is not None else "—"
    txt(f"IMU target  : {imu_str}", C_CYAN)
    sep()

    # Robot pose
    txt("POSE", C_GRAY, FONT_MD)
    txt(f"X : {frame.robot_x:+.2f} m")
    txt(f"Y : {frame.robot_y:+.2f} m")
    txt(f"Hdg : {frame.heading_deg:.1f}°", C_YELLOW)
    sep()

    # Motor bars
    txt("MOTORS", C_GRAY, FONT_MD)
    _draw_motor_bar(surf, VIEW_W + 12, y, PANEL_W - 24, 18, frame.left_speed, "L")
    y += 22
    _draw_motor_bar(surf, VIEW_W + 12, y, PANEL_W - 24, 18, frame.right_speed, "R")
    y += 26
    sep()

    # Visible tags
    txt("TAGS VISIBLE", C_GRAY, FONT_MD)
    if frame.tags:
        for tid, (wx, wy, dist, bear) in sorted(frame.tags.items()):
            col = C_TAG_ODD if tid % 2 == 1 else C_TAG_EVEN
            txt(f"  T{tid:2d}  {dist:.2f}m  {bear:+.1f}°", col)
    else:
        txt("  (none)", C_GRAY)
    sep()

    # Gates visible
    txt("GATES VISIBLE", C_GRAY, FONT_MD)
    if frame.gates:
        for gid, (wx, wy) in sorted(frame.gates.items()):
            txt(f"  G{gid}", C_GATE_OK)
    else:
        txt("  (none)", C_GRAY)
    sep()

    # Compass
    _draw_compass(surf, VIEW_W + PANEL_W // 2, y + 52, 42,
                  frame.heading_deg, frame.imu_target)
    y += 112
    sep()

    # Stats
    txt(f"Display FPS : {fps:.0f}", C_GRAY)
    txt(f"Tags total  : {len(frame.tags)}", C_GRAY)
    txt(f"Aim bearing : {frame.aim_bearing:+.1f}°"
        if frame.aim_bearing is not None else "Aim bearing : —", C_GRAY)

    # Key hints at bottom
    yh = H - 120
    for line in ["Q/Esc quit   R reset trail",
                 "+/-   zoom   Arrows pan",
                 "S trail  H cone  G gates  L lidar",
                 "Space pause (sim)"]:
        surf.blit(FONT_SM.render(line, True, C_GRID), (x, yh))
        yh += 16


def _draw_motor_bar(surf, x, y, w, h, speed, label):
    mid = w // 2
    pygame.draw.rect(surf, C_PANEL_BD, pygame.Rect(x, y, w, h), 1)
    # Bar
    bar_len = int(abs(speed) * mid)
    if speed >= 0:
        rect = pygame.Rect(x + mid, y + 1, bar_len, h - 2)
        colour = C_GREEN
    else:
        rect = pygame.Rect(x + mid - bar_len, y + 1, bar_len, h - 2)
        colour = C_RED
    if bar_len > 0:
        pygame.draw.rect(surf, colour, rect)
    # Centre line
    pygame.draw.line(surf, C_GRAY, (x + mid, y), (x + mid, y + h), 1)
    surf.blit(FONT_SM.render(f"{label} {speed:+.2f}", True, C_WHITE),
              (x + 2, y + 2))


def _draw_compass(surf, cx, cy, r, heading, imu_target):
    pygame.draw.circle(surf, C_PANEL_BD, (cx, cy), r, 1)
    for deg, lbl in ((0, "N"), (90, "E"), (180, "S"), (270, "W")):
        rad = math.radians(deg)
        tx  = cx + int(math.sin(rad) * (r + 10))
        ty  = cy - int(math.cos(rad) * (r + 10))
        t   = FONT_SM.render(lbl, True, C_GRAY)
        surf.blit(t, (tx - t.get_width() // 2, ty - t.get_height() // 2))

    # IMU target
    if imu_target is not None:
        rad = math.radians(imu_target)
        ex  = cx + int(math.sin(rad) * (r - 2))
        ey  = cy - int(math.cos(rad) * (r - 2))
        pygame.draw.line(surf, C_CYAN, (cx, cy), (ex, ey), 1)

    # Heading needle
    rad = math.radians(heading)
    nx  = cx + int(math.sin(rad) * (r - 4))
    ny  = cy - int(math.cos(rad) * (r - 4))
    pygame.draw.line(surf, C_YELLOW, (cx, cy), (nx, ny), 3)
    pygame.draw.circle(surf, C_YELLOW, (cx, cy), 3)


# ── Simulation data source ────────────────────────────────────────────────────

class Simulator:
    """
    Closed-loop sim: robot starts at origin heading North,
    drives through a row of gates placed 2 m apart.
    The ArUco navigator runs internally; robot position is dead-reckoned
    from motor outputs with a simple unicycle model.
    """

    WHEELBASE = 0.20   # metres between wheel centrelines
    VEL_SCALE = 1.0    # max_speed metres/second at motor=1.0

    def __init__(self, num_gates: int = 4):
        from robot.aruco_navigator import ArucoNavigator, NavConfig, NavState
        from robot.aruco_detector  import ArUcoState, ArUcoTag

        self._NavState   = NavState
        self._ArUcoState = ArUcoState
        self._ArUcoTag   = ArUcoTag

        self.num_gates  = num_gates
        self.cfg        = NavConfig(
            max_gates      = num_gates,
            fwd_speed      = 0.45,
            pass_distance  = 0.55,
            pass_time      = 0.6,
            search_speed   = 0.25,
            align_deadband = 4.0,
        )
        self.nav        = ArucoNavigator(self.cfg)

        # Physical gate layout: gates 2 m apart, 1.5 m wide, aligned along Y axis
        self.gate_layouts: List[Tuple[float, float, float]] = []  # (cx, cy, half_width)
        for i in range(num_gates):
            self.gate_layouts.append((0.0, 2.0 + i * 2.5, 0.75))

        # Robot state
        self.rx:  float = 0.0
        self.ry:  float = 0.0
        self.hdg: float = 0.0   # degrees, North=0

        self._left  = 0.0
        self._right = 0.0
        self._lock  = threading.Lock()
        self._frame = VisFrame()
        self._paused = False
        self._running = True
        self._t = threading.Thread(target=self._loop, daemon=True)

    def start(self):
        self.nav.start()
        self._t.start()

    def stop(self):
        self._running = False

    def pause_toggle(self):
        self._paused = not self._paused

    def get_frame(self) -> VisFrame:
        with self._lock:
            return self._frame

    # ── Simulation loop ───────────────────────────────────────────────────────

    def _loop(self):
        last = time.monotonic()
        while self._running:
            now = time.monotonic()
            dt  = min(now - last, 0.05)
            last = now

            if not self._paused:
                aruco_state = self._sense()
                left, right = self.nav.update(aruco_state, 640,
                                              heading=self.hdg)
                self._integrate(left, right, dt)
                self._left, self._right = left, right
                self._publish(aruco_state)

            time.sleep(0.02)   # ~50 Hz

    def _sense(self):
        """
        Build a synthetic ArUcoState from the robot's current pose
        relative to gate post positions.
        """
        gate_id = self.nav.gate_id
        tags:  Dict = {}

        if gate_id >= self.num_gates:
            return self._ArUcoState(tags=tags, fps=50.0, timestamp=time.monotonic())

        cx, cy, hw = self.gate_layouts[gate_id]

        outside_wx = cx - hw    # left post (outside/even tag, gate_id*2)
        outside_wy = cy
        inside_wx  = cx + hw    # right post (inside/odd tag, gate_id*2+1)
        inside_wy  = cy

        # Transform posts into robot-relative polar
        def _to_robot(wx, wy):
            dx, dy = wx - self.rx, wy - self.ry
            dist   = math.hypot(dx, dy)
            # Bearing relative to camera: world angle then subtract heading
            world_ang = math.degrees(math.atan2(dx, dy))   # N=0, clockwise
            bearing   = (world_ang - self.hdg + 180) % 360 - 180
            return dist, bearing

        outside_dist, outside_bear = _to_robot(outside_wx, outside_wy)
        inside_dist,  inside_bear  = _to_robot(inside_wx,  inside_wy)

        FOV         = 60.0  # ±30° visible
        outside_id  = gate_id * 2
        inside_id   = gate_id * 2 + 1

        if abs(outside_bear) <= FOV / 2 and outside_dist < 5.0:
            tags[outside_id] = self._ArUcoTag(
                id=outside_id,
                center_x=int(320 + outside_bear / FOV * 640),
                center_y=240,
                area=int(4000 / max(0.1, outside_dist ** 2)),
                top_left=(300, 220), top_right=(340, 220),
                bottom_right=(340, 260), bottom_left=(300, 260),
                distance=outside_dist, bearing=outside_bear,
            )

        if abs(inside_bear) <= FOV / 2 and inside_dist < 5.0:
            tags[inside_id] = self._ArUcoTag(
                id=inside_id,
                center_x=int(320 + inside_bear / FOV * 640),
                center_y=240,
                area=int(4000 / max(0.1, inside_dist ** 2)),
                top_left=(380, 220), top_right=(420, 220),
                bottom_right=(420, 260), bottom_left=(380, 260),
                distance=inside_dist, bearing=inside_bear,
            )

        return self._ArUcoState(tags=tags, fps=50.0, timestamp=time.monotonic())

    def _integrate(self, left: float, right: float, dt: float):
        v_l = left  * self.VEL_SCALE
        v_r = right * self.VEL_SCALE
        v   = (v_l + v_r) / 2.0
        w   = (v_r - v_l) / self.WHEELBASE

        heading_rad = math.radians(self.hdg)
        self.rx  += v * math.sin(heading_rad) * dt
        self.ry  += v * math.cos(heading_rad) * dt
        self.hdg  = (self.hdg + math.degrees(w) * dt) % 360

    def _publish(self, aruco_state):
        # Build tags dict for VisFrame (world coords via dead-reckoning)
        vis_tags: Dict[int, Tuple[float, float, float, float]] = {}
        for tid, tag in aruco_state.tags.items():
            if tag.distance is not None and tag.bearing is not None:
                world_ang = (self.hdg + tag.bearing) % 360
                rad = math.radians(world_ang)
                wx  = self.rx + math.sin(rad) * tag.distance
                wy  = self.ry + math.cos(rad) * tag.distance
                vis_tags[tid] = (wx, wy, tag.distance, tag.bearing)

        vis_gates: Dict[int, Tuple[float, float]] = {}
        gate = self.nav.find_gate(aruco_state, self.nav.gate_id)
        if gate is not None and gate.distance is not None and gate.bearing is not None:
            world_ang = (self.hdg + gate.bearing) % 360
            rad = math.radians(world_ang)
            wx  = self.rx + math.sin(rad) * gate.distance
            wy  = self.ry + math.cos(rad) * gate.distance
            vis_gates[self.nav.gate_id] = (wx, wy)

        aim_bear = None
        if hasattr(self.nav, 'bearing_err') and self.nav.bearing_err is not None:
            aim_bear = self.nav.bearing_err

        frame = VisFrame(
            robot_x     = self.rx,
            robot_y     = self.ry,
            heading_deg = self.hdg,
            left_speed  = self._left,
            right_speed = self._right,
            nav_state   = self.nav.state.name,
            gate_id     = self.nav.gate_id,
            bearing_err = getattr(self.nav, 'bearing_err', None),
            imu_target  = self.nav._imu_target,
            tags        = vis_tags,
            gates       = vis_gates,
            aim_bearing = aim_bear,
            ts          = time.monotonic(),
        )
        with self._lock:
            self._frame = frame


# ── HTTP source — polls /api/state SSE from a running frontend ───────────────

class HttpSource:
    """
    Reads robot state from the SSE endpoint already served by whichever
    frontend is running (robot_gui.py, robot_web.py, or robot_mobile.py).

    Default URLs:
      robot_web.py    → http://localhost:5000/api/state
      robot_mobile.py → http://localhost:5001/api/state

    Dead-reckons robot position from motor outputs between IMU heading
    updates (same approach as UdpSource).

    The frontend keeps running unmodified — this source is read-only.
    """

    WHEELBASE   = 0.20
    VEL_SCALE   = 0.70
    STALE_SECS  = 3.0
    RETRY_SECS  = 3.0

    def __init__(self, url: str = "http://localhost:5000/api/state"):
        self._url     = url
        self._rx      = 0.0
        self._ry      = 0.0
        self._hdg     = 0.0
        self._frame   = VisFrame()
        self._lock    = threading.Lock()
        self._running = True
        self._last_rx = 0.0
        self._last_t  = time.monotonic()
        self._t = threading.Thread(target=self._loop, daemon=True)

    def start(self):
        self._t.start()
        print(f"  HTTP source polling {self._url}")
        print("  (frontend must be running — robot_web.py or robot_mobile.py)")

    def stop(self):
        self._running = False

    def get_frame(self) -> VisFrame:
        with self._lock:
            f = self._frame
        if time.monotonic() - self._last_rx > self.STALE_SECS:
            f.nav_state = "WAITING"
        return f

    def _loop(self):
        import urllib.request
        import urllib.error

        while self._running:
            try:
                req = urllib.request.Request(
                    self._url,
                    headers={"Accept": "text/event-stream",
                             "Cache-Control": "no-cache"},
                )
                with urllib.request.urlopen(req, timeout=5) as resp:
                    buf = b""
                    while self._running:
                        chunk = resp.read(4096)
                        if not chunk:
                            break
                        buf += chunk
                        # SSE frames end with \n\n
                        while b"\n\n" in buf:
                            frame_bytes, buf = buf.split(b"\n\n", 1)
                            for line in frame_bytes.split(b"\n"):
                                if line.startswith(b"data:"):
                                    payload = line[5:].strip()
                                    try:
                                        self._handle(json.loads(payload))
                                    except (json.JSONDecodeError, KeyError):
                                        pass

            except (urllib.error.URLError, OSError, TimeoutError) as e:
                if self._running:
                    print(f"\r  HTTP source: {e} — retrying in "
                          f"{self.RETRY_SECS:.0f}s...", end="", flush=True)
                    time.sleep(self.RETRY_SECS)

    def _handle(self, d: dict):
        now = time.monotonic()
        dt  = min(now - self._last_t, 0.1)
        self._last_t = now
        self._last_rx = now

        # IMU heading (use if available, else dead-reckon)
        t = d.get("telemetry", {})
        hdg = t.get("heading")
        if hdg is not None:
            self._hdg = hdg

        # Motor outputs → dead-reckon position (prefer actual applied speeds)
        drive = d.get("drive", {})
        telem = d.get("telemetry", {})
        al, ar = telem.get("applied_l", 0.0), telem.get("applied_r", 0.0)
        left  = al if (al != 0.0 or ar != 0.0) else drive.get("left",  0.0)
        right = ar if (al != 0.0 or ar != 0.0) else drive.get("right", 0.0)
        self._integrate(left, right, dt)
        if hdg is None:
            pass   # heading already updated above if present

        # ArUco tags → world coords
        vis_tags:  Dict = {}
        vis_gates: Dict = {}

        for tag in d.get("aruco", {}).get("tags", []):
            tid  = tag.get("id")
            dist = tag.get("distance")
            bear = tag.get("bearing")
            if tid is not None and dist is not None and bear is not None:
                world_ang = (self._hdg + bear) % 360
                rad = math.radians(world_ang)
                wx  = self._rx + math.sin(rad) * dist
                wy  = self._ry + math.cos(rad) * dist
                vis_tags[tid] = (wx, wy, dist, bear)

        nav_state   = d.get("nav_state",     "IDLE") or "IDLE"
        gate_id     = d.get("nav_gate",      0)
        bearing_err = d.get("nav_bearing_err")

        frame = VisFrame(
            robot_x     = self._rx,
            robot_y     = self._ry,
            heading_deg = self._hdg,
            left_speed  = left,
            right_speed = right,
            nav_state   = nav_state,
            gate_id     = gate_id,
            bearing_err = bearing_err,
            tags        = vis_tags,
            gates       = vis_gates,
            ts          = now,
        )
        with self._lock:
            self._frame = frame

    def _integrate(self, left: float, right: float, dt: float):
        v_l = left  * self.VEL_SCALE
        v_r = right * self.VEL_SCALE
        v   = (v_l + v_r) / 2.0
        w   = (v_r - v_l) / self.WHEELBASE
        rad = math.radians(self._hdg)
        self._rx  += v * math.sin(rad) * dt
        self._ry  += v * math.cos(rad) * dt
        self._hdg  = (self._hdg + math.degrees(w) * dt) % 360


# ── UDP telemetry source ──────────────────────────────────────────────────────

UDP_PORT_DEFAULT = 5005


class UdpSource:
    """
    Receives VisFrame JSON datagrams broadcast by VisEmitter (in
    test_tag_approach.py).  Runs in its own thread; get_frame() is
    always safe to call from the render loop.

    Packet format (JSON, max ~1400 bytes):
    {
      "rx": 0.0, "ry": 0.0, "hdg": 0.0,
      "lspd": 0.0, "rspd": 0.0,
      "state": "SEARCH", "bear": 12.3,
      "tags": {"1": [wx, wy, dist, bear], ...},
      "ts": 1234567.8
    }
    """

    STALE_TIMEOUT = 2.0   # seconds without a packet → show "waiting" state

    def __init__(self, host: str = "0.0.0.0", port: int = UDP_PORT_DEFAULT):
        self._host    = host
        self._port    = port
        self._frame   = VisFrame()
        self._lock    = threading.Lock()
        self._running = True
        self._last_rx = 0.0
        self._t = threading.Thread(target=self._loop, daemon=True)

    def start(self):
        self._t.start()
        print(f"  UDP source listening on {self._host}:{self._port}")
        print("  Waiting for telemetry from test_tag_approach.py ...")

    def stop(self):
        self._running = False

    def get_frame(self) -> VisFrame:
        with self._lock:
            f = self._frame
        # If no packet recently, show a "waiting" state
        if time.monotonic() - self._last_rx > self.STALE_TIMEOUT:
            f.nav_state = "WAITING"
        return f

    def _loop(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self._host, self._port))
        sock.settimeout(0.5)

        while self._running:
            try:
                data, _ = sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break

            try:
                d = json.loads(data.decode())
            except (json.JSONDecodeError, UnicodeDecodeError):
                continue

            # Rebuild tags dict  {int(id): (wx, wy, dist, bear)}
            tags = {}
            for k, v in d.get("tags", {}).items():
                try:
                    tags[int(k)] = tuple(v)
                except (ValueError, TypeError):
                    pass

            frame = VisFrame(
                robot_x     = d.get("rx",    0.0),
                robot_y     = d.get("ry",    0.0),
                heading_deg = d.get("hdg",   0.0),
                left_speed  = d.get("lspd",  0.0),
                right_speed = d.get("rspd",  0.0),
                nav_state   = d.get("state", "IDLE"),
                bearing_err = d.get("bear",  None),
                tags        = tags,
                ts          = d.get("ts",    time.monotonic()),
            )

            self._last_rx = time.monotonic()
            with self._lock:
                self._frame = frame

        sock.close()


# ── Main loop ─────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="HackyRacingRobot overhead visualiser")
    group  = parser.add_mutually_exclusive_group()
    group.add_argument("--sim",  action="store_true",
                       help="Internal closed-loop simulation (no hardware)")
    group.add_argument("--live", action="store_true",
                       help="Poll /api/state from robot_web.py or robot_mobile.py")
    group.add_argument("--udp",  action="store_true",
                       help="Receive UDP telemetry from test_tag_approach.py")
    parser.add_argument("--url",      default="http://localhost:5000/api/state",
                        help="Frontend SSE URL for --live mode "
                             "(default http://localhost:5000/api/state)")
    parser.add_argument("--gates",    type=int, default=4,
                        help="Number of gates in simulation (default 4)")
    parser.add_argument("--udp-port", type=int, default=UDP_PORT_DEFAULT,
                        help=f"UDP listen port (default {UDP_PORT_DEFAULT})")
    parser.add_argument("--udp-host", default="0.0.0.0",
                        help="UDP bind address (default 0.0.0.0)")
    args = parser.parse_args()

    if not args.sim and not args.live and not args.udp:
        args.sim = True   # default to sim

    # ── Data source ────────────────────────────────────────────────────────────
    if args.sim:
        print(f"Starting simulation with {args.gates} gates...")
        source = Simulator(num_gates=args.gates)
    elif args.live:
        source = HttpSource(url=args.url)
    elif args.udp:
        source = UdpSource(host=args.udp_host, port=args.udp_port)
    else:
        source = Simulator()   # fallback
    source.start()

    # ── Pygame setup ───────────────────────────────────────────────────────────
    screen = pygame.display.set_mode((W, H))
    mode_label = "SIM" if args.sim else ("UDP" if args.udp else "LIVE")
    pygame.display.set_caption(f"HackyRacingRobot — Nav Visualiser [{mode_label}]")
    clock  = pygame.time.Clock()

    tx        = ViewTransform()
    trail     = deque(maxlen=500)
    last_pos  = None

    show_trail = True
    show_cone  = True
    show_gates = True
    show_lidar = True

    fps_display = 30.0

    running = True
    while running:
        dt_ms = clock.tick(50)
        fps_display = 0.9 * fps_display + 0.1 * (1000.0 / max(1, dt_ms))

        # ── Events ─────────────────────────────────────────────────────────────
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                k = event.key
                if k in (pygame.K_q, pygame.K_ESCAPE):
                    running = False
                elif k == pygame.K_r:
                    trail.clear(); last_pos = None; tx.reset()
                elif k == pygame.K_EQUALS or k == pygame.K_PLUS:
                    tx.zoom(1.2)
                elif k == pygame.K_MINUS:
                    tx.zoom(1 / 1.2)
                elif k == pygame.K_LEFT:
                    tx.pan(-30, 0)
                elif k == pygame.K_RIGHT:
                    tx.pan(30, 0)
                elif k == pygame.K_UP:
                    tx.pan(0, -30)
                elif k == pygame.K_DOWN:
                    tx.pan(0, 30)
                elif k == pygame.K_s:
                    show_trail = not show_trail
                elif k == pygame.K_h:
                    show_cone  = not show_cone
                elif k == pygame.K_g:
                    show_gates = not show_gates
                elif k == pygame.K_l:
                    show_lidar = not show_lidar
                elif k == pygame.K_SPACE:
                    if hasattr(source, 'pause_toggle'):
                        source.pause_toggle()

        # Arrow key hold for smooth pan
        keys = pygame.key.get_pressed()
        if keys[pygame.K_LEFT]:  tx.pan(-5, 0)
        if keys[pygame.K_RIGHT]: tx.pan( 5, 0)
        if keys[pygame.K_UP]:    tx.pan(0, -5)
        if keys[pygame.K_DOWN]:  tx.pan(0,  5)

        # ── Data ───────────────────────────────────────────────────────────────
        frame = source.get_frame()

        # Update trail
        pos = (frame.robot_x, frame.robot_y)
        if last_pos is None or math.hypot(pos[0] - last_pos[0],
                                          pos[1] - last_pos[1]) > 0.05:
            trail.append(pos)
            last_pos = pos

        # ── Render ─────────────────────────────────────────────────────────────
        screen.fill(C_BG)

        # Grid
        _draw_grid(screen, tx)

        # Gate layouts (simulated physical posts)
        if args.sim and show_gates:
            for i, (cx, cy, hw) in enumerate(source.gate_layouts):
                sp1 = tx.w2s(cx - hw, cy)
                sp2 = tx.w2s(cx + hw, cy)
                gate_col = C_GATE_OK if i == frame.gate_id else C_PANEL_BD
                pygame.draw.line(screen, gate_col, sp1, sp2, 3)
                # Post markers
                for px, py in [(cx - hw, cy), (cx + hw, cy)]:
                    sx, sy = tx.w2s(px, py)
                    pygame.draw.circle(screen, gate_col, (sx, sy), 6)
                # Gate number
                lbl = FONT_SM.render(f"G{i}", True, C_GATE_LABEL)
                gsx, gsy = tx.w2s(cx + hw + 0.1, cy)
                screen.blit(lbl, (gsx + 4, gsy - 8))

        # Trail
        _draw_trail(screen, tx, trail, show_trail)

        # LiDAR
        _draw_lidar(screen, tx, frame, show_lidar)

        # Camera cone + aim ray
        _draw_camera_cone(screen, tx, frame, show_cone)

        # Detected gates
        _draw_gates(screen, tx, frame, show_gates)

        # Detected tags
        _draw_tags(screen, tx, frame)

        # Robot
        _draw_robot(screen, tx, frame)

        # Scale bar
        bar_world = 1.0   # 1 metre
        bar_px    = int(tx.ppm * bar_world)
        bx, by    = VIEW_W - 20 - bar_px, H - 30
        pygame.draw.line(screen, C_WHITE, (bx, by), (bx + bar_px, by), 2)
        pygame.draw.line(screen, C_WHITE, (bx, by - 4), (bx, by + 4), 2)
        pygame.draw.line(screen, C_WHITE, (bx + bar_px, by - 4), (bx + bar_px, by + 4), 2)
        lbl = FONT_SM.render("1 m", True, C_WHITE)
        screen.blit(lbl, (bx + bar_px // 2 - lbl.get_width() // 2, by - 18))

        # Info panel
        _draw_panel(screen, frame, fps_display)

        pygame.display.flip()

    source.stop()
    pygame.quit()
    print("Visualiser closed.")


if __name__ == "__main__":
    main()
