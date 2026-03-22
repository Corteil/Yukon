#!/usr/bin/env python3
"""
robot_gui.py — Pygame status display for HackyRacingRobot.

Panels
------
  Top bar   : title + current mode (colour-coded)
  Drive     : left/right motor speed bars
  Telemetry : voltage, current, temperatures, fault indicators
  GPS       : position, fix quality, accuracy, satellite count
  Camera    : live frame (if enabled) with optional bearing overlay
  Lidar     : polar distance scan (if enabled)
  Footer    : subsystem health badges

Keyboard
--------
  E         : ESTOP (kill motors)
  R         : Reset ESTOP → MANUAL
  [ / ]     : Rotate camera −90° / +90°
  T         : Toggle ArUco detection
  B         : Toggle bearing overlay on camera
  C         : Open config overlay
  Q / Esc   : Quit

Bearing overlay (toggle with B)
--------------------------------
  Each visible ArUco tag gets:
    - A line from frame centre to tag centre
    - Distance label (metres, if calibrated) or pixel-area proxy
    - Bearing angle label (degrees from camera centre)
  Active gate aim point gets a cyan crosshair + aim line
  IMU heading compass arc shown at bottom of camera panel
  Navigator state + target gate shown when in AUTO·Camera mode

Usage
-----
  python3 robot_gui.py
  python3 robot_gui.py --config robot.ini
  python3 robot_gui.py --fps 15
"""

import argparse
import configparser
import logging
import math
import os
import re
import sys

import pygame

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from robot_daemon import Robot, RobotMode, AutoType, setup_logging

log = logging.getLogger(__name__)

# ── Colours ───────────────────────────────────────────────────────────────────

C_BG     = ( 18,  18,  30)
C_PANEL  = ( 28,  28,  45)
C_BORDER = ( 60,  60,  90)
C_WHITE  = (230, 230, 240)
C_GRAY   = (130, 130, 150)
C_GREEN  = ( 60, 220,  80)
C_YELLOW = (240, 200,  40)
C_ORANGE = (240, 140,  40)
C_RED    = (220,  60,  60)
C_CYAN   = ( 60, 200, 220)
C_PURPLE = (180,  80, 220)

MODE_COLORS = {
    RobotMode.MANUAL: C_YELLOW,
    RobotMode.AUTO:   C_GREEN,
    RobotMode.ESTOP:  C_RED,
}

FIX_COLORS = {
    0: C_RED,
    1: C_YELLOW,
    2: C_ORANGE,
    3: C_ORANGE,
    4: C_GREEN,
    5: C_CYAN,
}

# ── Config helpers ────────────────────────────────────────────────────────────

def _load_config(path):
    cfg = configparser.ConfigParser(inline_comment_prefixes=('#',))
    cfg.read(path)
    return cfg

from robot_utils import _cfg

# ── Drawing helpers ───────────────────────────────────────────────────────────

def _panel(surf, rect, title=None, border=C_BORDER):
    pygame.draw.rect(surf, C_PANEL, rect, border_radius=8)
    pygame.draw.rect(surf, border,  rect, width=1, border_radius=8)
    if title:
        t = FONT_SM.render(title, True, C_GRAY)
        surf.blit(t, (rect.x + 10, rect.y + 6))

def _field(surf, label, value, x, y, color=C_WHITE):
    surf.blit(FONT_SM.render(label, True, C_GRAY), (x, y))
    surf.blit(FONT_MD.render(str(value), True, color), (x, y + 16))

def _draw_drive(surf, rect, left, right):
    _panel(surf, rect, "Drive")
    bar_h = 46
    bar_w = rect.width - 20
    bx    = rect.x + 10
    for label, speed, by in (
        ("L", left,  rect.y + 30),
        ("R", right, rect.y + 90),
    ):
        bar_rect = pygame.Rect(bx, by, bar_w, bar_h)
        pygame.draw.rect(surf, C_BG, bar_rect, border_radius=4)
        half = bar_w // 2
        cx   = bx + half
        fill = int(abs(speed) * half)
        color = C_GREEN if speed >= 0 else C_ORANGE
        if fill > 0:
            if speed >= 0:
                pygame.draw.rect(surf, color,
                    pygame.Rect(cx, by + 4, fill, bar_h - 8), border_radius=3)
            else:
                pygame.draw.rect(surf, color,
                    pygame.Rect(cx - fill, by + 4, fill, bar_h - 8), border_radius=3)
        pygame.draw.line(surf, C_BORDER, (cx, by + 2), (cx, by + bar_h - 2), 1)
        lbl = FONT_SM.render(label, True, C_GRAY)
        val = FONT_SM.render(f"{speed:+.2f}", True, color if speed != 0 else C_GRAY)
        surf.blit(lbl, (bx + 4,        by + (bar_h - lbl.get_height()) // 2))
        surf.blit(val, (bx + bar_w - val.get_width() - 4,
                        by + (bar_h - val.get_height()) // 2))

def _draw_telemetry(surf, rect, t):
    fault = t.left_fault or t.right_fault
    _panel(surf, rect, "Telemetry", border=C_RED if fault else C_BORDER)
    mx, my = rect.x + 10, rect.y + 26
    _field(surf, "Voltage", f"{t.voltage:.1f} V",   mx,       my)
    _field(surf, "Current", f"{t.current:.2f} A",   mx + 130, my)
    _field(surf, "Board",   f"{t.board_temp:.0f}°C", mx,       my + 50)
    _field(surf, "Left",    f"{t.left_temp:.0f}°C",  mx + 100, my + 50)
    _field(surf, "Right",   f"{t.right_temp:.0f}°C", mx + 180, my + 50)
    # Heading row
    hdg_str = f"{t.heading:.1f}°" if t.heading is not None else "---"
    _field(surf, "IMU Hdg", hdg_str, mx, my + 100,
           color=C_CYAN if t.heading is not None else C_GRAY)
    fault_str = (("FAULT-L " if t.left_fault else "") +
                 ("FAULT-R"  if t.right_fault else "")) or "OK"
    _field(surf, "Faults", fault_str, mx + 130, my + 100,
           color=C_RED if fault else C_GREEN)

def _draw_gps(surf, rect, g, gps_ok, gps_logging=False):
    fix_color = FIX_COLORS.get(g.fix_quality, C_GRAY) if gps_ok else C_GRAY
    border    = C_CYAN if gps_logging else fix_color
    _panel(surf, rect, "GPS", border=border)
    gx, gy = rect.x + 10, rect.y + 26
    lat_str  = f"{g.latitude:.7f}"  if g.latitude  is not None else "---"
    lon_str  = f"{g.longitude:.7f}" if g.longitude is not None else "---"
    alt_str  = f"{g.altitude:.1f} m" if g.altitude is not None else "---"
    herr_str = f"{g.h_error_m:.3f} m" if g.h_error_m is not None else "---"
    hdop_str = f"{g.hdop:.2f}" if g.hdop is not None else "---"
    sats_str = str(g.satellites or "--")
    if g.satellites_view:
        sats_str += f"/{g.satellites_view}"
    half = (rect.width - 20) // 2
    _field(surf, "Latitude",  lat_str,  gx,        gy)
    _field(surf, "Longitude", lon_str,  gx,        gy + 50)
    _field(surf, "Altitude",  alt_str,  gx + half, gy)
    _field(surf, "H-Error",   herr_str, gx + half, gy + 50)
    _field(surf, "Fix",  g.fix_quality_name if gps_ok else "No GPS",
           gx,        gy + 100, color=fix_color)
    _field(surf, "Sats",      sats_str, gx + half,       gy + 100)
    _field(surf, "HDOP",      hdop_str, gx + half + 100, gy + 100)

def _draw_system(surf, rect, s):
    temp_color = (C_GREEN if s.cpu_temp_c < 60
                  else C_YELLOW if s.cpu_temp_c < 75 else C_RED)
    _panel(surf, rect, "System",
           border=temp_color if s.cpu_temp_c >= 60 else C_BORDER)
    bx = rect.x + 10
    by = rect.y + 26
    bw = rect.width - 20

    def _bar_row(y, label, pct, detail, color):
        surf.blit(FONT_TINY.render(label, True, C_GRAY), (bx, y))
        bar = pygame.Rect(bx, y + 13, bw, 13)
        pygame.draw.rect(surf, C_BG, bar, border_radius=3)
        fill_w = max(0, int(bw * pct / 100))
        if fill_w:
            pygame.draw.rect(surf, color,
                pygame.Rect(bx, y + 13, fill_w, 13), border_radius=3)
        val = FONT_TINY.render(detail, True, color)
        surf.blit(val, (bx + bw - val.get_width() - 2, y + 14))

    cpu_color  = C_GREEN if s.cpu_percent  < 60 else C_YELLOW if s.cpu_percent  < 85 else C_RED
    mem_color  = C_GREEN if s.mem_percent  < 70 else C_YELLOW if s.mem_percent  < 90 else C_RED
    disk_color = C_GREEN if s.disk_percent < 70 else C_YELLOW if s.disk_percent < 90 else C_RED

    _bar_row(by,        "CPU",  s.cpu_percent,
             f"{s.cpu_percent:.0f}% {s.cpu_temp_c:.0f}°C {s.cpu_freq_mhz:.0f}MHz", cpu_color)
    _bar_row(by + 50,   "Mem",  s.mem_percent,
             f"{s.mem_percent:.0f}% {s.mem_used_mb/1024:.1f}/{s.mem_total_mb/1024:.1f}GB", mem_color)
    _bar_row(by + 100,  "Disk", s.disk_percent,
             f"{s.disk_percent:.0f}% {s.disk_used_gb:.1f}/{s.disk_total_gb:.1f}GB", disk_color)


# ── Bearing overlay helpers ───────────────────────────────────────────────────

def _draw_bearing_line(surf, inner, tag_cx, tag_cy, label, color, aim=False):
    """Draw a line from the frame centre to (tag_cx, tag_cy) with a label.

    tag_cx/tag_cy are in *frame* pixel coords; we scale them to the inner rect.
    If aim=True, draw a crosshair at the endpoint instead of a dot.
    """
    # tag coords are already in frame space — they come straight from ArUcoTag
    # which uses the original (unscaled) frame.  We scale to inner rect.
    ix, iy = inner.x, inner.y
    iw, ih = inner.width, inner.height

    # These will be set by the caller in scaled coords — see _draw_camera
    ex, ey = tag_cx, tag_cy

    fcx = ix + iw // 2
    fcy = iy + ih // 2

    pygame.draw.line(surf, color, (fcx, fcy), (ex, ey), 1)

    if aim:
        sz = 10
        pygame.draw.line(surf, color, (ex - sz, ey), (ex + sz, ey), 2)
        pygame.draw.line(surf, color, (ex, ey - sz), (ex, ey + sz), 2)
        pygame.draw.circle(surf, color, (ex, ey), sz, 1)
    else:
        pygame.draw.circle(surf, color, (ex, ey), 4, 2)

    lbl = FONT_TINY.render(label, True, color)
    lx  = ex + 6 if ex < fcx + iw // 3 else ex - lbl.get_width() - 6
    ly  = ey - lbl.get_height() - 2
    # Keep label within inner bounds
    lx  = max(ix + 2, min(lx, ix + iw - lbl.get_width() - 2))
    ly  = max(iy + 2, min(ly, iy + ih - lbl.get_height() - 2))
    # Dark background for readability
    bg  = pygame.Rect(lx - 2, ly - 1, lbl.get_width() + 4, lbl.get_height() + 2)
    pygame.draw.rect(surf, C_BG, bg)
    surf.blit(lbl, (lx, ly))


def _draw_compass(surf, inner, heading):
    """Draw a small compass arc at the bottom-centre of the camera panel."""
    cx  = inner.x + inner.width // 2
    cy  = inner.bottom - 22
    r   = 18
    pygame.draw.circle(surf, C_BG,    (cx, cy), r + 2)
    pygame.draw.circle(surf, C_BORDER,(cx, cy), r,    1)
    # North tick
    pygame.draw.line(surf, C_RED, (cx, cy - r + 3), (cx, cy - r + 9), 2)
    # Heading needle
    rad = math.radians(heading - 90)
    nx  = cx + int(r * math.cos(rad))
    ny  = cy + int(r * math.sin(rad))
    pygame.draw.line(surf, C_CYAN, (cx, cy), (nx, ny), 2)
    lbl = FONT_TINY.render(f"{heading:.0f}°", True, C_CYAN)
    surf.blit(lbl, (cx - lbl.get_width() // 2, cy - r - 14))


def _draw_nav_status(surf, inner, nav_state, nav_gate, bearing_err,
                     auto_type=None, nav_wp=0, nav_wp_dist=None, nav_wp_bear=None):
    """Draw navigator state badge in top-right of camera panel."""
    state_colors = {
        "IDLE":         C_GRAY,
        "WAITING_FIX":  C_YELLOW,
        "SEARCHING":    C_YELLOW,
        "ALIGNING":     C_ORANGE,
        "APPROACHING":  C_GREEN,
        "NAVIGATING":   C_GREEN,
        "PASSING":      C_CYAN,
        "ARRIVED":      C_CYAN,
        "COMPLETE":     C_GREEN,
        "ERROR":        C_RED,
    }
    color = state_colors.get(nav_state, C_GRAY)

    # Build label depending on navigator type
    from robot_daemon import AutoType as _AT
    is_gps = (auto_type is not None and
              auto_type in (_AT.GPS, _AT.CAMERA_GPS) and
              nav_state not in ("IDLE",))

    if is_gps:
        parts = [f"GPS:{nav_state}  WP:{nav_wp}"]
        if nav_wp_dist is not None:
            parts.append(f"{nav_wp_dist:.1f}m")
        if nav_wp_bear is not None:
            parts.append(f"{nav_wp_bear:.0f}°")
    else:
        parts = [f"NAV:{nav_state}  Gate:{nav_gate}"]
        if bearing_err is not None:
            parts.append(f"Err:{bearing_err:+.1f}°")

    txt = "  ".join(parts)
    lbl = FONT_TINY.render(txt, True, color)
    x   = inner.right  - lbl.get_width() - 8
    y   = inner.y + 6
    bg  = pygame.Rect(x - 3, y - 2, lbl.get_width() + 6, lbl.get_height() + 4)
    pygame.draw.rect(surf, C_BG, bg, border_radius=3)
    surf.blit(lbl, (x, y))


# ── Camera panel ──────────────────────────────────────────────────────────────

def _draw_camera(surf, rect, frame, rotation=0,
                 aruco_state=None, aruco_enabled=False,
                 show_bearing=False,
                 nav_state="IDLE", nav_gate=0,
                 nav_bearing_err=None,
                 imu_heading=None,
                 auto_mode=False,
                 frame_w=640, frame_h=480,
                 target_gate_id=0,
                 auto_type=None,
                 nav_wp=0, nav_wp_dist=None, nav_wp_bear=None):
    title = f"Camera [{rotation}°]" if rotation else "Camera"
    if show_bearing:
        title += "  [B=bearing ON]"
    _panel(surf, rect, title)
    inner = pygame.Rect(rect.x + 2, rect.y + 22, rect.width - 4, rect.height - 24)

    if frame is None:
        msg = FONT_SM.render("No camera", True, C_GRAY)
        surf.blit(msg, (rect.centerx - msg.get_width() // 2,
                        rect.centery - msg.get_height() // 2))
    else:
        try:
            cam_surf = pygame.surfarray.make_surface(frame.swapaxes(0, 1))
            scaled   = pygame.transform.scale(cam_surf, (inner.width, inner.height))
            surf.blit(scaled, inner.topleft)
        except Exception as e:
            msg = FONT_SM.render(str(e)[:50], True, C_RED)
            surf.blit(msg, (rect.x + 10, rect.centery))

        # ── Bearing overlay ───────────────────────────────────────────────────
        if show_bearing and aruco_state is not None:
            # Scale factors: frame coords → inner rect coords
            sx = inner.width  / max(frame_w, 1)
            sy = inner.height / max(frame_h, 1)

            odd_id  = target_gate_id * 2 + 1
            even_id = target_gate_id * 2 + 2

            for tag in aruco_state.tags.values():
                # Scale tag centre to inner rect
                tx = inner.x + int(tag.center_x * sx)
                ty = inner.y + int(tag.center_y * sy)

                is_target = tag.id in (odd_id, even_id)
                color = C_CYAN if is_target else C_YELLOW

                # Build label
                parts = [f"#{tag.id}"]
                if tag.bearing is not None:
                    parts.append(f"{tag.bearing:+.1f}°")
                if tag.distance is not None:
                    parts.append(f"{tag.distance:.2f}m")
                label = " ".join(parts)

                _draw_bearing_line(surf, inner, tx, ty, label, color, aim=False)

            # Draw aim point for active gate if both posts visible
            gate = aruco_state.gates.get(target_gate_id)
            if gate is not None:
                gx = inner.x + int(gate.centre_x * sx)
                gy = inner.y + int(gate.centre_y * sy)
                parts = ["AIM"]
                if gate.bearing is not None:
                    parts.append(f"{gate.bearing:+.1f}°")
                if gate.distance is not None:
                    parts.append(f"{gate.distance:.2f}m")
                _draw_bearing_line(surf, inner, gx, gy,
                                   " ".join(parts), C_GREEN, aim=True)

            # Centre crosshair (camera boresight)
            fcx = inner.x + inner.width  // 2
            fcy = inner.y + inner.height // 2
            pygame.draw.line(surf, C_GRAY, (fcx - 12, fcy), (fcx + 12, fcy), 1)
            pygame.draw.line(surf, C_GRAY, (fcx, fcy - 12), (fcx, fcy + 12), 1)
            pygame.draw.circle(surf, C_GRAY, (fcx, fcy), 4, 1)

            # IMU compass
            if imu_heading is not None:
                _draw_compass(surf, inner, imu_heading)

        # ── Nav state badge (always shown in AUTO·Camera) ─────────────────────
        if auto_mode:
            _draw_nav_status(surf, inner, nav_state, nav_gate, nav_bearing_err,
                             auto_type=auto_type, nav_wp=nav_wp,
                             nav_wp_dist=nav_wp_dist, nav_wp_bear=nav_wp_bear)

    # ── ArUco status bar (bottom-left) ────────────────────────────────────────
    if aruco_enabled:
        if aruco_state is not None:
            info  = (f"ACO tags:{len(aruco_state.tags)}"
                     f" gates:{len(aruco_state.gates)}"
                     f" {aruco_state.fps:.0f}fps")
            color = C_GREEN
        else:
            info, color = "ACO: initialising...", C_YELLOW
    else:
        info, color = "ACO: OFF", C_GRAY

    info_surf = FONT_TINY.render(info, True, color)
    ix = inner.x + 6
    iy = inner.bottom - info_surf.get_height() - 6
    bg = pygame.Rect(ix - 3, iy - 2,
                     info_surf.get_width() + 6, info_surf.get_height() + 4)
    pygame.draw.rect(surf, C_BG, bg, border_radius=2)
    surf.blit(info_surf, (ix, iy))


# ── Lidar panel ───────────────────────────────────────────────────────────────

def _draw_lidar(surf, rect, scan):
    _panel(surf, rect, "Lidar")
    if not scan.angles:
        msg = FONT_SM.render("No lidar data", True, C_GRAY)
        surf.blit(msg, (rect.centerx - msg.get_width() // 2,
                        rect.centery - msg.get_height() // 2))
        return

    cx       = rect.centerx
    cy       = rect.centery + 10
    r        = min(rect.width, rect.height) // 2 - 26
    max_dist = max(scan.distances) if scan.distances else 1

    for frac in (0.25, 0.5, 0.75, 1.0):
        ring_r = int(r * frac)
        pygame.draw.circle(surf, C_BORDER, (cx, cy), ring_r, 1)
        lbl = FONT_TINY.render(f"{max_dist * frac / 1000:.1f}m", True, C_BORDER)
        surf.blit(lbl, (cx + ring_r + 2, cy - 7))

    pygame.draw.line(surf, C_BORDER, (cx, cy - r), (cx, cy + r), 1)
    pygame.draw.line(surf, C_BORDER, (cx - r, cy), (cx + r, cy), 1)
    for lbl_txt, dx, dy in (("N", 0, -r - 14), ("S", 0, r + 2),
                             ("E", r + 2, -6),  ("W", -r - 14, -6)):
        t = FONT_TINY.render(lbl_txt, True, C_GRAY)
        surf.blit(t, (cx + dx, cy + dy))

    for angle, dist in zip(scan.angles, scan.distances):
        if dist <= 0:
            continue
        frac = min(dist / max_dist, 1.0)
        pr   = frac * r
        rad  = math.radians(angle)
        px   = cx + int(pr * math.sin(rad))
        py   = cy - int(pr * math.cos(rad))
        color = (int(220 * (1 - frac) + 60 * frac),
                 int( 60 * (1 - frac) + 200 * frac),
                 int( 60 * (1 - frac) + 220 * frac))
        pygame.draw.circle(surf, color, (px, py), 2)

    pygame.draw.circle(surf, C_GREEN, (cx, cy), 5)


def _badge(surf, text, color, x, y):
    t = FONT_SM.render(text, True, color)
    surf.blit(t, (x, y))
    return x + t.get_width() + 20


# ── Config overlay ────────────────────────────────────────────────────────────

_CFG_FIELDS = [
    ("robot", ["yukon_port", "ibus_port"]),
    ("rc",    ["throttle_ch", "steer_ch", "mode_ch", "speed_ch",
               "auto_type_ch", "gps_log_ch", "deadzone",
               "failsafe_s", "speed_min", "control_hz"]),
    ("gui",   ["fps"]),
    ("camera",["disabled", "width", "height", "fps", "rotation"]),
    ("aruco", ["enabled", "dict", "calib_file", "tag_size"]),
    ("lidar", ["disabled", "port"]),
    ("gps",   ["disabled", "port", "log_dir", "log_hz"]),
    ("ntrip", ["disabled", "host", "port", "mount", "user", "password"]),
    ("rtcm",  ["disabled", "port", "baud"]),
    ("navigator", ["max_gates", "pass_distance", "pass_time",
                   "search_speed", "search_step_deg", "fwd_speed",
                   "steer_kp", "imu_kp", "align_deadband"]),
]

_CFG_LEFT_SECTS = 3


def _cfg_flat(cfg):
    rows = []
    for section, keys in _CFG_FIELDS:
        for key in keys:
            try:
                val = cfg.get(section, key).strip()
            except (configparser.NoSectionError, configparser.NoOptionError):
                val = ""
            rows.append((section, key, val))
    return rows


def _save_config(path, cfg):
    with open(path) as f:
        lines = f.readlines()
    current_section = None
    result = []
    for line in lines:
        stripped = line.strip()
        m = re.match(r'^\[(\w+)\]', stripped)
        if m:
            current_section = m.group(1).lower()
            result.append(line)
            continue
        if (current_section and '=' in stripped
                and not stripped.startswith('#')):
            key = stripped.split('=')[0].strip()
            if cfg.has_option(current_section, key):
                new_val = cfg.get(current_section, key)
                indent  = len(line) - len(line.lstrip())
                result.append(' ' * indent + f"{key:<16}= {new_val}\n")
                continue
        result.append(line)
    with open(path, 'w') as f:
        f.writelines(result)


def _draw_config_overlay(surf, W, H, config_path, cfg,
                         sel_idx, editing, edit_text, dirty):
    overlay = pygame.Rect(20, 20, W - 40, H - 40)
    pygame.draw.rect(surf, C_PANEL, overlay, border_radius=8)
    pygame.draw.rect(surf, C_BORDER, overlay, width=1, border_radius=8)

    dirty_mark = " [unsaved]" if dirty else ""
    title = FONT_BIG.render(
        f"Config: {os.path.basename(config_path)}{dirty_mark}", True,
        C_YELLOW if dirty else C_WHITE)
    surf.blit(title, (overlay.x + 16, overlay.y + 12))

    hint_txt = ("Enter=confirm Esc=cancel" if editing
                else "↑↓=select ←→=toggle bool Enter=edit S=save C/Esc=close")
    hint = FONT_TINY.render(hint_txt, True, C_GRAY)
    surf.blit(hint, (overlay.right - hint.get_width() - 16, overlay.y + 18))
    pygame.draw.line(surf, C_BORDER,
                     (overlay.x + 10, overlay.y + 44),
                     (overlay.right - 10, overlay.y + 44), 1)

    ROW_H  = 20
    GAP    = 8
    col_w  = (overlay.width - 48) // 2
    left_x = overlay.x + 16
    right_x= left_x + col_w + 16
    start_y= overlay.y + 54
    mid_x  = left_x + col_w + 8

    pygame.draw.line(surf, C_BORDER,
                     (mid_x, start_y - 4), (mid_x, overlay.bottom - 12), 1)

    flat      = _cfg_flat(cfg)
    item_rects= []
    flat_idx  = 0

    for col_idx, (col_x, sect_range) in enumerate((
        (left_x,  range(0, _CFG_LEFT_SECTS)),
        (right_x, range(_CFG_LEFT_SECTS, len(_CFG_FIELDS)))
    )):
        y = start_y
        for si in sect_range:
            section, keys = _CFG_FIELDS[si]
            hdr = FONT_SM.render(section.capitalize(), True, C_CYAN)
            surf.blit(hdr, (col_x, y))
            y += ROW_H
            for key in keys:
                _, _, val = flat[flat_idx]
                row_rect  = pygame.Rect(col_x, y, col_w, ROW_H - 2)
                if flat_idx == sel_idx:
                    pygame.draw.rect(surf, (45, 45, 70), row_rect, border_radius=3)
                key_color = C_GRAY if flat_idx != sel_idx else C_WHITE
                surf.blit(FONT_TINY.render(f" {key}", True, key_color), (col_x, y))
                val_x = col_x + col_w // 2
                if flat_idx == sel_idx and editing:
                    cursor  = "_" if (pygame.time.get_ticks() // 500) % 2 == 0 else " "
                    display = edit_text + cursor
                    surf.blit(FONT_TINY.render(display, True, C_YELLOW), (val_x, y))
                else:
                    disp_val = str(val)
                    if len(disp_val) > 26:
                        disp_val = disp_val[:23] + "…"
                    color = C_YELLOW if flat_idx == sel_idx else C_WHITE
                    surf.blit(FONT_TINY.render(disp_val, True, color), (val_x, y))
                item_rects.append((section, key, row_rect))
                flat_idx += 1
                y += ROW_H - 2
            y += GAP
    return item_rects


# ── Main GUI loop ─────────────────────────────────────────────────────────────

def run_gui(robot: Robot, fps: int = 10, initial_rotation: int = 0,
            config_path: str = "", cfg=None):

    global FONT_BIG, FONT_MD, FONT_SM, FONT_TINY

    pygame.init()
    W, H = 1024, 768
    screen = pygame.display.set_mode((W, H))
    pygame.display.set_caption("HackyRacingRobot Monitor")
    clock = pygame.time.Clock()

    FONT_BIG  = pygame.font.SysFont("monospace", 28, bold=True)
    FONT_MD   = pygame.font.SysFont("monospace", 18)
    FONT_SM   = pygame.font.SysFont("monospace", 14)
    FONT_TINY = pygame.font.SysFont("monospace", 11)

    TITLE_H = 72
    PANEL_H = 190
    PANEL_Y = TITLE_H + 10
    BODY_Y  = PANEL_Y + PANEL_H + 10
    FOOTER_H= 30
    BODY_H  = H - BODY_Y - FOOTER_H - 8

    running       = True
    show_config   = False
    show_bearing  = False      # ← bearing overlay toggle
    cfg_sel       = 0
    cfg_editing   = False
    cfg_edit_text = ""
    cfg_dirty     = False
    cfg_n_items   = sum(len(keys) for _, keys in _CFG_FIELDS)

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.KEYDOWN:

                if show_config and cfg_editing:
                    if event.key == pygame.K_RETURN:
                        flat = _cfg_flat(cfg)
                        section, key, _ = flat[cfg_sel]
                        cfg.set(section, key, cfg_edit_text)
                        cfg_dirty  = True
                        cfg_editing = False
                    elif event.key == pygame.K_ESCAPE:
                        cfg_editing = False
                    elif event.key == pygame.K_BACKSPACE:
                        cfg_edit_text = cfg_edit_text[:-1]
                    else:
                        if event.unicode and event.unicode.isprintable():
                            cfg_edit_text += event.unicode

                elif show_config:
                    if event.key in (pygame.K_c, pygame.K_ESCAPE):
                        show_config = False
                    elif event.key in (pygame.K_UP, pygame.K_DOWN):
                        delta   = -1 if event.key == pygame.K_UP else 1
                        cfg_sel = (cfg_sel + delta) % cfg_n_items
                    elif event.key in (pygame.K_LEFT, pygame.K_RIGHT):
                        flat = _cfg_flat(cfg)
                        val  = flat[cfg_sel][2].lower()
                        if val in ("true", "false"):
                            section, key, _ = flat[cfg_sel]
                            cfg.set(section, key,
                                    "false" if val == "true" else "true")
                            cfg_dirty = True
                    elif event.key == pygame.K_RETURN:
                        flat          = _cfg_flat(cfg)
                        cfg_edit_text = flat[cfg_sel][2]
                        cfg_editing   = True
                    elif event.key == pygame.K_s and cfg is not None and config_path:
                        _save_config(config_path, cfg)
                        cfg_dirty = False

                else:
                    if event.key == pygame.K_c:
                        show_config = True
                    elif event.key in (pygame.K_q, pygame.K_ESCAPE):
                        running = False
                    elif event.key == pygame.K_e:
                        robot.estop()
                    elif event.key == pygame.K_r:
                        robot.reset_estop()
                    elif event.key == pygame.K_LEFTBRACKET:
                        robot.set_cam_rotation((robot.get_cam_rotation() - 90) % 360)
                    elif event.key == pygame.K_RIGHTBRACKET:
                        robot.set_cam_rotation((robot.get_cam_rotation() + 90) % 360)
                    elif event.key == pygame.K_t:
                        robot.toggle_aruco()
                    elif event.key == pygame.K_b:
                        show_bearing = not show_bearing   # ← bearing toggle

        state       = robot.get_state()
        frame       = robot.get_frame()
        aruco_state = robot.get_aruco_state()
        cam_rotation= robot.get_cam_rotation()
        imu_heading = robot.get_heading()
        mode_color  = MODE_COLORS.get(state.mode, C_GRAY)

        nav_bearing_err = state.nav_bearing_err
        auto_cam = (state.mode is RobotMode.AUTO and
                    state.auto_type in (AutoType.CAMERA, AutoType.CAMERA_GPS))

        screen.fill(C_BG)

        # ── Title bar ─────────────────────────────────────────────────────────
        title_rect = pygame.Rect(10, 8, W - 20, TITLE_H - 8)
        pygame.draw.rect(screen, C_PANEL,      title_rect, border_radius=6)
        pygame.draw.rect(screen, mode_color,   title_rect, width=2, border_radius=6)
        screen.blit(FONT_BIG.render("HackyRacingRobot Monitor", True, C_WHITE), (22, 14))

        sc = state.speed_scale
        if sc < 0.45:   spd_label, spd_color = "Slow",   C_CYAN
        elif sc < 0.80: spd_label, spd_color = "Medium", C_YELLOW
        else:           spd_label, spd_color = "Fast",   C_GREEN

        mode_display = (f"AUTO·{state.auto_type.label}"
                        if state.mode is RobotMode.AUTO else state.mode.name)
        mode_txt = FONT_BIG.render(mode_display, True, mode_color)
        mode_lbl = FONT_BIG.render("Mode: ",     True, C_GRAY)
        spd_txt  = FONT_BIG.render(spd_label,    True, spd_color)
        spd_lbl  = FONT_BIG.render("Speed: ",    True, C_GRAY)

        rx = W - 22
        screen.blit(mode_txt, (rx - mode_txt.get_width(), 14))
        rx -= mode_txt.get_width()
        screen.blit(mode_lbl, (rx - mode_lbl.get_width(), 14))
        rx -= mode_lbl.get_width() + 30
        screen.blit(spd_txt,  (rx - spd_txt.get_width(),  14))
        rx -= spd_txt.get_width()
        screen.blit(spd_lbl,  (rx - spd_lbl.get_width(),  14))

        no_motors_warn = "  *** NO-MOTORS ***" if state.no_motors else ""
        hint = FONT_SM.render(
            "E=ESTOP R=Reset [/]=Cam T=ArUco B=Bearing C=Config Q=Quit" + no_motors_warn,
            True, C_YELLOW if state.no_motors else C_GRAY)
        screen.blit(hint, (W // 2 - hint.get_width() // 2, 50))

        # ── Status panels ──────────────────────────────────────────────────────
        DRIVE_W = 200
        TELEM_W = 260    # slightly wider to fit heading field
        SYS_W   = 210
        GPS_W   = W - 20 - DRIVE_W - TELEM_W - SYS_W - 30
        x0      = 10

        drive_rect = pygame.Rect(x0,                                   PANEL_Y, DRIVE_W, PANEL_H)
        telem_rect = pygame.Rect(x0 + DRIVE_W + 10,                    PANEL_Y, TELEM_W, PANEL_H)
        gps_rect   = pygame.Rect(x0 + DRIVE_W + TELEM_W + 20,         PANEL_Y, GPS_W,   PANEL_H)
        sys_rect   = pygame.Rect(x0 + DRIVE_W + TELEM_W + GPS_W + 30, PANEL_Y, SYS_W,   PANEL_H)

        _draw_drive    (screen, drive_rect, state.drive.left, state.drive.right)
        _draw_telemetry(screen, telem_rect, state.telemetry)
        _draw_gps      (screen, gps_rect,   state.gps, state.gps_ok, state.gps_logging)
        _draw_system   (screen, sys_rect,   state.system)

        # ── Camera & Lidar ─────────────────────────────────────────────────────
        cam_w     = (W - 30) // 2
        cam_rect  = pygame.Rect(10,          BODY_Y, cam_w,         BODY_H)
        lidar_rect= pygame.Rect(20 + cam_w,  BODY_Y, W - cam_w - 30, BODY_H)

        _draw_camera(
            screen, cam_rect,
            frame if state.camera_ok else None,
            rotation     = cam_rotation,
            aruco_state  = aruco_state,
            aruco_enabled= robot.get_aruco_enabled(),
            show_bearing = show_bearing,
            nav_state    = state.nav_state,
            nav_gate     = state.nav_gate,
            nav_bearing_err = nav_bearing_err,
            imu_heading  = imu_heading,
            auto_mode    = auto_cam,
            frame_w      = robot._cam_width,
            frame_h      = robot._cam_height,
            target_gate_id = state.nav_gate,
            auto_type    = state.auto_type,
            nav_wp       = state.nav_wp,
            nav_wp_dist  = state.nav_wp_dist,
            nav_wp_bear  = state.nav_wp_bear,
        )
        _draw_lidar(screen, lidar_rect, state.lidar)

        # ── Footer ─────────────────────────────────────────────────────────────
        fy = H - FOOTER_H + 4
        pygame.draw.line(screen, C_BORDER, (10, fy - 6), (W - 10, fy - 6), 1)
        bx = 14
        g  = state.gps
        fix_color = FIX_COLORS.get(g.fix_quality, C_GRAY)
        aco_on    = robot.get_aruco_enabled()

        bx = _badge(screen, f"RC: {'OK' if state.rc_active else '--'}",
                    C_GREEN if state.rc_active else C_GRAY, bx, fy)
        bx = _badge(screen, f"CAM: {'OK' if state.camera_ok else '--'}",
                    C_GREEN if state.camera_ok else C_GRAY, bx, fy)
        bx = _badge(screen, f"ACO: {'ON' if aco_on else 'OFF'}",
                    C_GREEN if state.aruco_ok else (C_YELLOW if aco_on else C_GRAY), bx, fy)
        bx = _badge(screen, f"LDR: {'OK' if state.lidar_ok else '--'}",
                    C_GREEN if state.lidar_ok else C_GRAY, bx, fy)
        bx = _badge(screen, f"GPS: {g.fix_quality_name if state.gps_ok else '--'}",
                    fix_color if state.gps_ok else C_GRAY, bx, fy)
        if state.gps_ok and g.h_error_m is not None:
            bx = _badge(screen, f"H-err: {g.h_error_m:.3f}m", C_CYAN, bx, fy)
        bx = _badge(screen, f"LOG: {'ON' if state.gps_logging else 'OFF'}",
                    C_CYAN if state.gps_logging else C_GRAY, bx, fy)

        # IMU heading badge
        if imu_heading is not None:
            bx = _badge(screen, f"HDG: {imu_heading:.1f}°", C_CYAN, bx, fy)

        # Nav state badge
        if state.nav_state not in ("IDLE",):
            nav_colors = {"SEARCHING": C_YELLOW, "ALIGNING": C_ORANGE,
                          "APPROACHING": C_GREEN, "PASSING": C_CYAN,
                          "NAVIGATING": C_GREEN, "WAITING_FIX": C_YELLOW,
                          "ARRIVED": C_CYAN, "COMPLETE": C_GREEN, "ERROR": C_RED}
            nc = nav_colors.get(state.nav_state, C_GRAY)
            from robot_daemon import AutoType as _AT
            if state.auto_type in (_AT.GPS, _AT.CAMERA_GPS):
                nav_txt = f"GPS: {state.nav_state}  WP{state.nav_wp}"
                if state.nav_wp_dist is not None:
                    nav_txt += f"  {state.nav_wp_dist:.1f}m"
            else:
                nav_txt = f"NAV: {state.nav_state}  G{state.nav_gate}"
            bx = _badge(screen, nav_txt, nc, bx, fy)

        if config_path:
            cfg_lbl = FONT_SM.render(os.path.basename(config_path), True, C_GRAY)
            screen.blit(cfg_lbl, (W - cfg_lbl.get_width() - 14, fy))

        if state.no_motors:
            nm = FONT_SM.render("NO-MOTORS", True, C_RED)
            # Right-aligned, just left of config filename
            nm_x = W - nm.get_width() - (
                FONT_SM.size(os.path.basename(config_path))[0] + 28
                if config_path else 14
            )
            pygame.draw.rect(screen, (60, 0, 0),
                             pygame.Rect(nm_x - 4, fy - 2,
                                         nm.get_width() + 8,
                                         nm.get_height() + 4),
                             border_radius=3)
            screen.blit(nm, (nm_x, fy))

        # ── Config overlay ─────────────────────────────────────────────────────
        if show_config and cfg is not None:
            _draw_config_overlay(screen, W, H, config_path, cfg,
                                 cfg_sel, cfg_editing, cfg_edit_text, cfg_dirty)

        pygame.display.flip()
        clock.tick(fps)


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    DEFAULT_CFG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "robot.ini")

    parser = argparse.ArgumentParser(description="HackyRacingRobot pygame GUI")
    parser.add_argument("--config",          default=DEFAULT_CFG)
    parser.add_argument("--fps",             default=None, type=int)
    parser.add_argument("--yukon-port",      default=None)
    parser.add_argument("--ibus-port",       default=None)
    parser.add_argument("--gps-port",        default=None)
    parser.add_argument("--lidar-port",      default=None)
    parser.add_argument("--ntrip-host",      default=None)
    parser.add_argument("--ntrip-port",      default=None, type=int)
    parser.add_argument("--ntrip-mount",     default=None)
    parser.add_argument("--ntrip-user",      default=None)
    parser.add_argument("--ntrip-password",  default=None)
    parser.add_argument("--rtcm-port",       default=None)
    parser.add_argument("--rtcm-baud",       default=None, type=int)
    parser.add_argument("--no-camera",       action="store_true", default=False)
    parser.add_argument("--no-lidar",        action="store_true", default=False)
    parser.add_argument("--no-gps",          action="store_true", default=False)
    parser.add_argument("--no-motors",       action="store_true", default=False,
                        help="Suppress all motor/LED/bearing commands (bench test mode)")
    args = parser.parse_args()

    cfg      = _load_config(args.config)
    bool_val = lambda x: x.lower() == "true"
    port_val = lambda x: None if x.lower() in ("auto", "") else x

    log_path = setup_logging()
    log.info(f"Log file: {log_path}")
    log.info(f"Config: {args.config}")

    fps = args.fps if args.fps is not None else _cfg(cfg, "gui", "fps", 10, int)

    def arg(cli_val, section, key, fallback, cast=str):
        if cli_val is not None:
            return cli_val
        return _cfg(cfg, section, key, fallback, cast)

    robot = Robot(
        yukon_port     = arg(args.yukon_port,      "robot", "yukon_port",     None, port_val),
        ibus_port      = arg(args.ibus_port,        "robot", "ibus_port",      "/dev/ttyAMA3"),
        lidar_port     = arg(args.lidar_port,       "lidar", "port",           "/dev/ttyUSB0"),
        gps_port       = arg(args.gps_port,         "gps",   "port",           "/dev/ttyUSB0"),
        ntrip_host     = "" if _cfg(cfg, "ntrip", "disabled", False, bool_val)
                         else arg(args.ntrip_host,  "ntrip", "host",           ""),
        ntrip_port     = arg(args.ntrip_port,       "ntrip", "port",           2101,  int),
        ntrip_mount    = arg(args.ntrip_mount,      "ntrip", "mount",          ""),
        ntrip_user     = arg(args.ntrip_user,       "ntrip", "user",           ""),
        ntrip_password = arg(args.ntrip_password,   "ntrip", "password",       ""),
        rtcm_port      = "" if _cfg(cfg, "rtcm", "disabled", False, bool_val)
                         else arg(args.rtcm_port,   "rtcm",  "port",           ""),
        rtcm_baud      = arg(args.rtcm_baud,        "rtcm",  "baud",           115200, int),
        enable_camera  = not (args.no_camera or _cfg(cfg, "camera", "disabled", False, bool_val)),
        enable_lidar   = not (args.no_lidar  or _cfg(cfg, "lidar",  "disabled", False, bool_val)),
        enable_gps     = not (args.no_gps    or _cfg(cfg, "gps",    "disabled", False, bool_val)),
        cam_width      = _cfg(cfg, "camera",     "width",       640,  int),
        cam_height     = _cfg(cfg, "camera",     "height",      480,  int),
        cam_fps        = _cfg(cfg, "camera",     "fps",         30,   int),
        cam_rotation   = _cfg(cfg, "camera",     "rotation",    0,    int),
        enable_aruco   = _cfg(cfg, "aruco",      "enabled",     False, bool_val),
        aruco_dict     = _cfg(cfg, "aruco",      "dict",        "DICT_4X4_1000"),
        aruco_calib    = _cfg(cfg, "aruco",      "calib_file",  ""),
        aruco_tag_size = _cfg(cfg, "aruco",      "tag_size",    0.15,  float),
        throttle_ch    = _cfg(cfg, "rc",         "throttle_ch", 3,    int),
        steer_ch       = _cfg(cfg, "rc",         "steer_ch",    1,    int),
        mode_ch        = _cfg(cfg, "rc",         "mode_ch",     5,    int),
        speed_ch       = _cfg(cfg, "rc",         "speed_ch",    6,    int),
        auto_type_ch   = _cfg(cfg, "rc",         "auto_type_ch",7,    int),
        gps_log_ch     = _cfg(cfg, "rc",         "gps_log_ch",  8,    int),
        gps_bookmark_ch= _cfg(cfg, "rc",         "gps_bookmark_ch", 2, int),
        gps_log_dir    = _cfg(cfg, "gps",        "log_dir",     ""),
        gps_log_hz     = _cfg(cfg, "gps",        "log_hz",      5.0,  float),
        deadzone       = _cfg(cfg, "rc",         "deadzone",    30,   int),
        failsafe_s     = _cfg(cfg, "rc",         "failsafe_s",  0.5,  float),
        speed_min      = _cfg(cfg, "rc",         "speed_min",   0.25, float),
        control_hz     = _cfg(cfg, "rc",         "control_hz",  50,   int),
        no_motors      = args.no_motors,
    )
    robot.start()

    try:
        run_gui(robot, fps=fps, config_path=args.config, cfg=cfg)
    finally:
        robot.stop()
        pygame.quit()


if __name__ == "__main__":
    main()
