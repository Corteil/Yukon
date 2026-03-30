#!/usr/bin/env python3
"""
robot_quad_gui.py — Quad-panel touch GUI for HackyRacingRobot.

Layout (1280 × 720 landscape touchscreen, configurable)
--------------------------------------------------------
  Full-width top bar  : ESTOP · RESET · MODE · RC · GPS · BATTERY · FAULTS
  2×2 quad grid       : swappable panels (camera feeds, lidar, GPS, telemetry…)
  Side button strip   : ESTOP · RESET · MODE · ARUCO · FREEZE · ML LOG ·
                        REC ALL · layout presets

Quad interaction
----------------
  Single tap   : enter SELECT mode — strip shows panel-type picker
  Double tap   : expand / collapse that quad to fill the whole grid area
  Tap outside strip (in SELECT mode) : cancel, return to NORMAL

Panel types
-----------
  front_left   IMX296 camera + ArUco overlay (CAM0)
  front_right  IMX296 camera + ArUco overlay (CAM1)
  rear         IMX477 camera + gate-confirmed overlay
  lidar        LD06 polar scan
  gps_sky      GPS satellite sky-view (elevation / azimuth plot)
  gps_track    GPS position track
  depth_map    AI Kit depth output (placeholder)
  telemetry    Drive bars + voltage / temp
  system       CPU / RAM / disk bars
  imu          BNO085 compass rose + roll / pitch

Keyboard shortcuts (press H to show overlay)
--------------------------------------------
  E            ESTOP
  R            Reset ESTOP
  M            Cycle mode
  A            Toggle ArUco (all cameras)
  F            Freeze all cameras
  L            Toggle ML data log
  V            Toggle REC ALL
  1–4          Select quad 1–4
  Enter        Expand / collapse selected quad
  Esc          Cancel select / close help
  H            Toggle key-hint overlay
  Q            Quit

Usage
-----
  python3 robot_quad_gui.py
  python3 robot_quad_gui.py --display 1280x720
  python3 robot_quad_gui.py --display fullscreen
  python3 robot_quad_gui.py --config robot.ini
  python3 robot_quad_gui.py --no-camera --no-lidar
"""

import argparse
import configparser
import logging
import math
import os
import sys
import time

import pygame

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from robot_daemon import Robot, RobotMode, AutoType, setup_logging
from robot_utils import _cfg

log = logging.getLogger(__name__)

# ── Colour palette ────────────────────────────────────────────────────────────

C_BG        = ( 18,  18,  30)
C_PANEL     = ( 28,  28,  45)
C_PANEL2    = ( 35,  35,  55)
C_BORDER    = ( 60,  60,  90)
C_BORDER_HI = (100, 100, 140)
C_WHITE     = (230, 230, 240)
C_GRAY      = (130, 130, 150)
C_GRAY2     = ( 80,  80, 100)
C_GREEN     = ( 60, 220,  80)
C_YELLOW    = (240, 200,  40)
C_ORANGE    = (240, 140,  40)
C_RED       = (220,  60,  60)
C_CYAN      = ( 60, 200, 220)
C_PURPLE    = (180,  80, 220)
C_BLUE      = ( 60, 120, 220)

C_SELECT    = (100, 180, 255)   # selected quad border
C_EXPANDED  = ( 60, 200, 220)  # expanded quad border

MODE_COLOR = {
    RobotMode.MANUAL: C_YELLOW,
    RobotMode.AUTO:   C_GREEN,
    RobotMode.ESTOP:  C_RED,
}

FIX_COLOR = {0: C_RED, 1: C_YELLOW, 2: C_ORANGE,
             3: C_ORANGE, 4: C_GREEN, 5: C_CYAN}

# ── Panel type registry ───────────────────────────────────────────────────────

ALL_PANEL_TYPES = [
    'front_left', 'front_right', 'rear',
    'lidar', 'gps_sky', 'gps_track',
    'depth_map', 'telemetry', 'system', 'imu',
]

PANEL_LABELS = {
    'front_left':  'FRONT LEFT',
    'front_right': 'FRONT RIGHT',
    'rear':        'REAR',
    'lidar':       'LIDAR',
    'gps_sky':     'GPS SKY',
    'gps_track':   'GPS TRACK',
    'depth_map':   'DEPTH MAP',
    'telemetry':   'TELEMETRY',
    'system':      'SYSTEM',
    'imu':         'IMU',
}

CAMERA_PANELS = {'front_left', 'front_right', 'rear'}

# ── Config helpers ────────────────────────────────────────────────────────────

def _load_config(path):
    cfg = configparser.ConfigParser(inline_comment_prefixes=('#',))
    cfg.read(path)
    return cfg


def _resolve_display(mode: str):
    """Return (width, height, flags) from a display_mode string."""
    m = mode.strip().lower()
    if m == 'fullscreen':
        pygame.display.init()
        info = pygame.display.Info()
        return info.current_w, info.current_h, pygame.FULLSCREEN
    if m in ('windowed', ''):
        return 1280, 800, 0
    if m == 'touchscreen':
        return 1280, 720, 0
    if 'x' in m:
        try:
            w, h = m.split('x')
            return int(w), int(h), 0
        except ValueError:
            pass
    return 1280, 720, 0


def _voltage_thresholds(cfg):
    CHEM = {
        'lipo':  {'max': 4.20, 'warn': 3.50, 'critical': 3.30},
        'liion': {'max': 4.20, 'warn': 3.40, 'critical': 3.20},
        'nimh':  {'max': 1.45, 'warn': 1.10, 'critical': 1.00},
        'lfe':   {'max': 3.65, 'warn': 3.00, 'critical': 2.80},
    }
    chem  = cfg.get('battery', 'chemistry', fallback='lipo').strip().lower()
    cells = int(cfg.get('battery', 'cells', fallback='3'))
    defs  = CHEM.get(chem, CHEM['lipo'])
    scale = cells
    return {
        'max':      float(cfg.get('battery', 'voltage_max',      fallback='') or defs['max']      * scale),
        'warn':     float(cfg.get('battery', 'voltage_warn',     fallback='') or defs['warn']     * scale),
        'critical': float(cfg.get('battery', 'voltage_critical', fallback='') or defs['critical'] * scale),
    }


def _voltage_color(v, thresholds):
    if v <= 0:
        return C_GRAY
    if v <= thresholds['critical']:
        return C_RED
    if v <= thresholds['warn']:
        return C_YELLOW
    return C_GREEN


# ── Drawing primitives ────────────────────────────────────────────────────────

def _panel_bg(surf, rect, border=C_BORDER, radius=6):
    pygame.draw.rect(surf, C_PANEL, rect, border_radius=radius)
    pygame.draw.rect(surf, border,  rect, width=1, border_radius=radius)


def _text(surf, font, txt, x, y, color=C_WHITE, anchor='topleft'):
    s = font.render(str(txt), True, color)
    r = s.get_rect()
    setattr(r, anchor, (x, y))
    surf.blit(s, r)
    return r


def _bar_h(surf, rect, frac, color, bg=C_BG):
    pygame.draw.rect(surf, bg, rect, border_radius=3)
    if frac > 0:
        filled = rect.copy()
        filled.width = max(2, int(rect.width * min(frac, 1.0)))
        pygame.draw.rect(surf, color, filled, border_radius=3)


def _motor_bar(surf, rect, value, font):
    """Centred bidirectional motor bar."""
    pygame.draw.rect(surf, C_BG, rect, border_radius=4)
    mid = rect.centerx
    pygame.draw.line(surf, C_BORDER, (mid, rect.top + 3), (mid, rect.bottom - 3), 1)
    if abs(value) > 0.01:
        w = int(abs(value) * (rect.width // 2 - 2))
        color = C_GREEN if value >= 0 else C_RED
        if value >= 0:
            r = pygame.Rect(mid, rect.top + 3, w, rect.height - 6)
        else:
            r = pygame.Rect(mid - w, rect.top + 3, w, rect.height - 6)
        pygame.draw.rect(surf, color, r, border_radius=3)
    pct = f'{value * 100:+.0f}%'
    _text(surf, font, pct, rect.right - 4, rect.centery, C_WHITE, 'midright')


# ── Panel renderers ───────────────────────────────────────────────────────────

def _render_camera(surf, rect, frame, label, state, cam_key,
                   recording, frozen, gate_confirmed, fonts):
    """Draw a camera quad: header bar + live frame + optional overlays."""
    F_SM, F_MD, F_BIG, F_TINY = fonts['sm'], fonts['md'], fonts['big'], fonts['tiny']
    HEADER_H = 30
    FOOTER_H = 22 if recording else 0

    header = pygame.Rect(rect.x, rect.y, rect.width, HEADER_H)
    body   = pygame.Rect(rect.x, rect.y + HEADER_H,
                         rect.width, rect.height - HEADER_H - FOOTER_H)

    # Header
    pygame.draw.rect(surf, C_PANEL2, header, border_radius=4)
    _text(surf, F_SM, label, header.x + 8, header.centery, C_GRAY, 'midleft')

    # Sensor info (small, top-right of header)
    sensor = {'front_left': 'IMX296', 'front_right': 'IMX296', 'rear': 'IMX477'}.get(cam_key, '')
    if sensor:
        _text(surf, F_TINY, sensor, header.right - 8, header.centery, C_GRAY2, 'midright')

    # Body — camera frame or placeholder
    pygame.draw.rect(surf, (10, 10, 20), body)
    if frame is not None:
        try:
            import numpy as np
            h, w = frame.shape[:2]
            # Letterbox into body
            scale = min(body.width / w, body.height / h)
            sw, sh = int(w * scale), int(h * scale)
            import cv2
            scaled = cv2.resize(frame, (sw, sh), interpolation=cv2.INTER_LINEAR)
            surf_frame = pygame.surfarray.make_surface(scaled.swapaxes(0, 1))
            ox = body.x + (body.width - sw) // 2
            oy = body.y + (body.height - sh) // 2
            surf.blit(surf_frame, (ox, oy))
        except Exception:
            pass
    else:
        # No signal placeholder
        msg = 'FROZEN' if frozen else 'NO SIGNAL'
        col = C_CYAN if frozen else C_GRAY2
        _text(surf, F_MD, msg, body.centerx, body.centery, col, 'center')

    # Gate confirmed overlay (rear camera)
    if gate_confirmed and cam_key == 'rear':
        t = time.monotonic()
        if int(t * 2) % 2 == 0:
            s = F_BIG.render('GATE ✓ CONFIRMED', True, C_GREEN)
            surf.blit(s, s.get_rect(center=(body.centerx, body.bottom - 30)))

    # Recording footer
    if recording:
        footer = pygame.Rect(rect.x, rect.bottom - FOOTER_H, rect.width, FOOTER_H)
        pygame.draw.rect(surf, (40, 10, 10), footer)
        t = time.monotonic()
        dot = '●' if int(t * 2) % 2 == 0 else '○'
        _text(surf, F_TINY, f'{dot} REC', footer.x + 6, footer.centery,
              C_RED, 'midleft')


def _render_lidar(surf, rect, scan, fonts):
    """Polar lidar plot."""
    F_TINY = fonts['tiny']
    cx, cy = rect.centerx, rect.centery
    r_max  = min(rect.width, rect.height) // 2 - 10

    pygame.draw.circle(surf, (10, 10, 20), (cx, cy), r_max)

    if scan is None or len(scan.distances) == 0:
        _text(surf, fonts['md'], 'NO LIDAR', cx, cy, C_GRAY2, 'center')
        return

    # Grid rings
    for frac in (0.25, 0.5, 0.75, 1.0):
        pygame.draw.circle(surf, C_BORDER, (cx, cy), int(r_max * frac), 1)

    # Scan points
    angles, dists = scan.angles, scan.distances
    if len(angles) > 0:
        d_max = max(d for d in dists if d > 0) if any(d > 0 for d in dists) else 1
        for ang, d in zip(angles, dists):
            if d <= 0:
                continue
            frac = min(d / d_max, 1.0)
            rad  = math.radians(ang - 90)
            px   = cx + int(r_max * frac * math.cos(rad))
            py   = cy + int(r_max * frac * math.sin(rad))
            # Colour: near=red, far=cyan
            g = int(frac * 200)
            color = (220 - g, g, 220)
            pygame.draw.circle(surf, color, (px, py), 2)

    # Forward arrow
    pygame.draw.line(surf, C_CYAN, (cx, cy), (cx, cy - r_max + 5), 1)
    _text(surf, F_TINY, 'FWD', cx, cy - r_max + 6, C_CYAN, 'midbottom')

    # Range label
    if len(dists) > 0:
        near = min((d for d in dists if d > 0), default=0)
        _text(surf, F_TINY, f'near {near/1000:.1f}m', rect.x + 4, rect.bottom - 4,
              C_YELLOW, 'bottomleft')


def _render_telemetry(surf, rect, state, fonts):
    """Drive bars + voltage / current / temps."""
    F_SM, F_TINY = fonts['sm'], fonts['tiny']
    t = state.telemetry
    d = state.drive
    pad = 8
    x, y = rect.x + pad, rect.y + pad
    w = rect.width - 2 * pad

    # Motor bars
    _text(surf, F_TINY, 'LEFT',  x, y, C_GRAY)
    _motor_bar(surf, pygame.Rect(x, y + 14, w, 22), d.left,  F_TINY)
    y += 44
    _text(surf, F_TINY, 'RIGHT', x, y, C_GRAY)
    _motor_bar(surf, pygame.Rect(x, y + 14, w, 22), d.right, F_TINY)
    y += 50

    # Telemetry fields
    fields = [
        ('VOLT',  f'{t.voltage:.1f} V',   C_GREEN if t.voltage > 10 else C_RED),
        ('CURR',  f'{t.current:.2f} A',   C_WHITE),
        ('BOARD', f'{t.board_temp:.0f}°C', C_YELLOW if t.board_temp > 60 else C_WHITE),
        ('L-MOT', f'{t.left_temp:.0f}°C',  C_RED if t.left_fault  else C_WHITE),
        ('R-MOT', f'{t.right_temp:.0f}°C', C_RED if t.right_fault else C_WHITE),
    ]
    col_w = w // 2
    for i, (lbl, val, col) in enumerate(fields):
        fx = x + (i % 2) * col_w
        fy = y + (i // 2) * 36
        _text(surf, F_TINY, lbl, fx, fy, C_GRAY)
        _text(surf, F_SM,   val, fx, fy + 14, col)


def _render_gps_sky(surf, rect, gps, fonts):
    """GPS satellite sky view — azimuth/elevation polar plot."""
    F_TINY = fonts['tiny']
    cx = rect.centerx
    cy = rect.centery
    r  = min(rect.width, rect.height) // 2 - 10

    pygame.draw.circle(surf, (10, 10, 20), (cx, cy), r)
    for frac in (0.33, 0.66, 1.0):
        pygame.draw.circle(surf, C_BORDER, (cx, cy), int(r * frac), 1)

    compass = ['N', 'E', 'S', 'W']
    for i, lbl in enumerate(compass):
        ang = math.radians(i * 90 - 90)
        px = cx + int((r + 8) * math.cos(ang))
        py = cy + int((r + 8) * math.sin(ang))
        _text(surf, F_TINY, lbl, px, py, C_GRAY2, 'center')

    if gps and hasattr(gps, 'satellites_data') and gps.satellites_data:
        for sat in gps.satellites_data:
            elev = sat.get('elev', 0)
            azim = sat.get('azim', 0)
            snr  = sat.get('snr',  0) or 0
            frac = 1.0 - (elev / 90.0)
            rad  = math.radians(azim - 90)
            px   = cx + int(r * frac * math.cos(rad))
            py   = cy + int(r * frac * math.sin(rad))
            col  = C_GREEN if snr > 30 else C_YELLOW if snr > 20 else C_RED
            pygame.draw.circle(surf, col, (px, py), 5)
            _text(surf, F_TINY, str(sat.get('svid', '')), px + 6, py, C_GRAY2)
    else:
        _text(surf, fonts['md'], 'NO GPS', cx, cy, C_GRAY2, 'center')

    if gps:
        fix_name = getattr(gps, 'fix_quality_name', 'NO FIX')
        sats = getattr(gps, 'satellites', 0) or 0
        _text(surf, F_TINY, f'{fix_name}  {sats} sats',
              rect.x + 4, rect.bottom - 4, FIX_COLOR.get(getattr(gps, 'fix_quality', 0), C_GRAY),
              'bottomleft')


def _render_gps_track(surf, rect, gps, track_pts, fonts):
    """GPS position track map."""
    F_TINY = fonts['tiny']
    pad = 10
    inner = rect.inflate(-pad * 2, -pad * 2)

    if len(track_pts) < 2:
        _text(surf, fonts['md'], 'NO TRACK', rect.centerx, rect.centery, C_GRAY2, 'center')
        if gps and gps.latitude is not None:
            _text(surf, F_TINY,
                  f'{gps.latitude:.6f}, {gps.longitude:.6f}',
                  rect.centerx, rect.centery + 24, C_GRAY, 'center')
        return

    lats = [p[0] for p in track_pts]
    lons = [p[1] for p in track_pts]
    lat_min, lat_max = min(lats), max(lats)
    lon_min, lon_max = min(lons), max(lons)
    lat_span = max(lat_max - lat_min, 1e-7)
    lon_span = max(lon_max - lon_min, 1e-7)

    def _proj(lat, lon):
        x = inner.x + int((lon - lon_min) / lon_span * inner.width)
        y = inner.bottom - int((lat - lat_min) / lat_span * inner.height)
        return x, y

    pts = [_proj(p[0], p[1]) for p in track_pts]
    if len(pts) >= 2:
        pygame.draw.lines(surf, C_CYAN, False, pts, 2)
    # Current position dot
    pygame.draw.circle(surf, C_GREEN, pts[-1], 5)
    _text(surf, F_TINY,
          f'{gps.latitude:.6f}, {gps.longitude:.6f}' if gps.latitude else 'NO FIX',
          rect.x + 4, rect.bottom - 4, C_GRAY, 'bottomleft')


def _render_system(surf, rect, sys_state, fonts):
    """CPU / RAM / disk usage bars."""
    F_TINY, F_SM = fonts['tiny'], fonts['sm']
    pad = 8
    x, y = rect.x + pad, rect.y + pad
    w = rect.width - 2 * pad

    rows = [
        ('CPU',  sys_state.cpu_pct,  C_CYAN),
        ('MEM',  sys_state.mem_pct,  C_BLUE),
        ('DISK', sys_state.disk_pct, C_PURPLE),
    ]
    for lbl, pct, col in rows:
        _text(surf, F_TINY, lbl, x, y, C_GRAY)
        bar = pygame.Rect(x, y + 14, w, 16)
        _bar_h(surf, bar, pct / 100, col)
        _text(surf, F_TINY, f'{pct:.0f}%', bar.right - 2, bar.centery, C_WHITE, 'midright')
        y += 40

    # CPU temp
    temp_col = C_GREEN if sys_state.cpu_temp_c < 60 else C_YELLOW if sys_state.cpu_temp_c < 75 else C_RED
    _text(surf, F_SM, f'CPU {sys_state.cpu_temp_c:.0f}°C', x, y, temp_col)


def _render_imu(surf, rect, telemetry, fonts):
    """BNO085 compass rose + roll/pitch/gyro readout."""
    F_SM, F_TINY, F_MD = fonts['sm'], fonts['tiny'], fonts['md']
    heading = getattr(telemetry, 'heading', None)

    cx = rect.x + rect.width // 2
    compass_r = min(rect.width // 2 - 20, 80)
    cy = rect.y + compass_r + 20

    # Compass circle
    pygame.draw.circle(surf, (15, 15, 28), (cx, cy), compass_r)
    pygame.draw.circle(surf, C_BORDER, (cx, cy), compass_r, 1)

    # Cardinal labels
    for ang_deg, lbl in ((0, 'N'), (90, 'E'), (180, 'S'), (270, 'W')):
        rad = math.radians(ang_deg - 90)
        px = cx + int((compass_r - 10) * math.cos(rad))
        py = cy + int((compass_r - 10) * math.sin(rad))
        _text(surf, F_TINY, lbl, px, py, C_GRAY2, 'center')

    if heading is not None:
        # Heading arrow
        rad = math.radians(heading - 90)
        ax  = cx + int((compass_r - 5) * math.cos(rad))
        ay  = cy + int((compass_r - 5) * math.sin(rad))
        pygame.draw.line(surf, C_CYAN, (cx, cy), (ax, ay), 2)
        pygame.draw.circle(surf, C_CYAN, (cx, cy), 3)
        _text(surf, F_SM, f'{heading:.1f}°', cx, cy + compass_r + 6,
              C_WHITE, 'midtop')
    else:
        _text(surf, F_MD, '-- IMU --', cx, cy, C_GRAY2, 'center')

    # Data fields below compass
    y = rect.y + compass_r * 2 + 36
    pad = rect.x + 8
    w2  = rect.width // 2

    roll  = getattr(telemetry, 'roll',  None)
    pitch = getattr(telemetry, 'pitch', None)

    _text(surf, F_TINY, 'ROLL',  pad,       y,      C_GRAY)
    _text(surf, F_SM,   f'{roll:.1f}°'  if roll  is not None else '--',
          pad,       y + 14, C_WHITE)
    _text(surf, F_TINY, 'PITCH', pad + w2,  y,      C_GRAY)
    _text(surf, F_SM,   f'{pitch:.1f}°' if pitch is not None else '--',
          pad + w2,  y + 14, C_WHITE)


def _render_depth_map(surf, rect, fonts):
    """Placeholder for AI Kit depth map output."""
    _text(surf, fonts['md'], 'DEPTH MAP', rect.centerx, rect.centery - 10, C_GRAY2, 'center')
    _text(surf, fonts['tiny'], '(AI Kit — not yet connected)',
          rect.centerx, rect.centery + 14, C_GRAY2, 'center')


# ── Top bar ───────────────────────────────────────────────────────────────────

def _render_topbar(surf, rect, state, v_thresh, tick, fonts, estop_rect_out, reset_rect_out):
    """Full-width top bar with ESTOP, RESET, MODE, RC, GPS, BAT, FAULTS."""
    F_SM, F_MD, F_TINY, F_BIG = fonts['sm'], fonts['md'], fonts['tiny'], fonts['big']

    pygame.draw.rect(surf, C_PANEL, rect)
    pygame.draw.line(surf, C_BORDER, (rect.x, rect.bottom), (rect.right, rect.bottom), 1)

    cx = rect.x + 8
    cy = rect.centery

    # ESTOP button
    btn_w, btn_h = 72, rect.height - 10
    estop_rect = pygame.Rect(cx, cy - btn_h // 2, btn_w, btn_h)
    ecol = C_RED if state.mode == RobotMode.ESTOP else (140, 30, 30)
    pygame.draw.rect(surf, ecol, estop_rect, border_radius=5)
    pygame.draw.rect(surf, C_RED, estop_rect, width=1, border_radius=5)
    _text(surf, F_SM, 'ESTOP', estop_rect.centerx, estop_rect.centery, C_WHITE, 'center')
    estop_rect_out.update(estop_rect)
    cx += btn_w + 6

    # RESET button
    reset_rect = pygame.Rect(cx, cy - btn_h // 2, btn_w - 10, btn_h)
    pygame.draw.rect(surf, (20, 60, 20), reset_rect, border_radius=5)
    pygame.draw.rect(surf, C_GREEN, reset_rect, width=1, border_radius=5)
    _text(surf, F_SM, 'RESET', reset_rect.centerx, reset_rect.centery, C_WHITE, 'center')
    reset_rect_out.update(reset_rect)
    cx += btn_w + 4

    # Separator
    pygame.draw.line(surf, C_BORDER, (cx, rect.y + 6), (cx, rect.bottom - 6), 1)
    cx += 10

    # MODE
    mode_str = state.mode.name
    if state.mode == RobotMode.AUTO:
        mode_str = f'AUTO·{state.auto_type.label}'
    mcol = MODE_COLOR.get(state.mode, C_WHITE)
    _text(surf, F_TINY, 'MODE', cx, cy - 10, C_GRAY)
    _text(surf, F_MD,   mode_str, cx, cy + 2, mcol)
    cx += 130

    # RC status
    if state.rc_active:
        rc_col, rc_sym = C_GREEN, '● RC'
    else:
        rc_col = C_RED if int(tick / 5) % 2 == 0 else C_GRAY
        rc_sym = '● RC'
    _text(surf, F_TINY, 'RC',  cx, cy - 10, C_GRAY)
    _text(surf, F_SM,   rc_sym, cx, cy + 2, rc_col)
    cx += 70

    # GPS
    gps = state.gps
    fix_q = getattr(gps, 'fix_quality', 0)
    fix_name = getattr(gps, 'fix_quality_name', 'NO FIX') if gps else 'NO GPS'
    gcol = FIX_COLOR.get(fix_q, C_GRAY) if state.gps_ok else C_GRAY
    _text(surf, F_TINY, 'GPS', cx, cy - 10, C_GRAY)
    _text(surf, F_SM,   fix_name, cx, cy + 2, gcol)
    cx += 100

    # Battery
    v   = state.telemetry.voltage
    amp = state.telemetry.current
    vcol = _voltage_color(v, v_thresh)
    _text(surf, F_TINY, 'BAT',      cx, cy - 10, C_GRAY)
    _text(surf, F_SM,   f'{v:.1f}V', cx, cy + 2,  vcol)
    cx += 70
    _text(surf, F_SM, f'{amp:.1f}A', cx, cy + 2, C_WHITE)
    cx += 60

    # Separator
    pygame.draw.line(surf, C_BORDER, (cx, rect.y + 6), (cx, rect.bottom - 6), 1)
    cx += 10

    # Faults — rotating
    faults = []
    if state.telemetry.left_fault:  faults.append('L-MOTOR FAULT')
    if state.telemetry.right_fault: faults.append('R-MOTOR FAULT')
    if not state.gps_ok:            faults.append('NO GPS')
    if not state.lidar_ok:          faults.append('NO LIDAR')
    if not state.rc_active:         faults.append('RC LOST')
    v_crit = state.telemetry.voltage > 0 and state.telemetry.voltage <= v_thresh['critical']
    if v_crit:                      faults.append('LOW BATTERY')
    if state.mode == RobotMode.ESTOP: faults.append('ESTOP')

    if faults:
        fi = int(time.monotonic() / 2.0) % len(faults)
        fault_str = faults[fi]
        blink = int(time.monotonic() * 2) % 2 == 0
        if blink:
            _text(surf, F_MD, f'⚠ {fault_str}', cx, cy + 2, C_RED)
    else:
        _text(surf, F_SM, 'OK', cx, cy + 2, C_GREEN)


# ── Side button strip ─────────────────────────────────────────────────────────

class ButtonStrip:
    """Manages the side strip buttons in NORMAL and SELECT modes."""

    NORMAL_BUTTONS = [
        ('ESTOP',   'E', C_RED),
        ('RESET',   'R', C_GREEN),
        ('MODE',    'M', C_YELLOW),
        ('ARUCO',   'A', C_CYAN),
        ('FREEZE',  'F', C_BLUE),
        ('ML LOG',  'L', C_PURPLE),
        ('REC ALL', 'V', C_RED),
    ]

    def __init__(self, rect, cfg, fonts):
        self._rect    = rect
        self._fonts   = fonts
        self._presets = self._load_presets(cfg)
        self._mode    = 'normal'   # 'normal' | 'select'
        self._buttons = []         # list of (label, key, color, rect, action)
        self._rebuild()

    def _load_presets(self, cfg):
        presets = {}
        if cfg.has_section('layout_presets'):
            for k, v in cfg.items('layout_presets'):
                if k == 'active_preset':
                    continue
                panels = [p.strip() for p in v.split(',')]
                presets[k.title()] = panels
        if not presets:
            presets = {'Race': ['front_left', 'front_right', 'rear', 'lidar']}
        return presets

    def set_mode(self, mode):
        self._mode = mode
        self._rebuild()

    def _rebuild(self):
        self._buttons = []
        r = self._rect
        pad = 6
        bw  = r.width - 2 * pad
        bh  = 44
        by  = r.y + pad

        if self._mode == 'normal':
            items = list(self.NORMAL_BUTTONS)
            # Preset buttons
            for name in self._presets:
                items.append((name, '', C_BORDER_HI))
            for label, key, color in items:
                btn_rect = pygame.Rect(r.x + pad, by, bw, bh)
                self._buttons.append((label, key, color, btn_rect))
                by += bh + 4
        else:  # select
            self._buttons.append(('← BACK', 'Esc', C_GRAY,
                                   pygame.Rect(r.x + pad, by, bw, bh)))
            by += bh + 8
            for ptype in ALL_PANEL_TYPES:
                btn_rect = pygame.Rect(r.x + pad, by, bw, bh)
                self._buttons.append((PANEL_LABELS[ptype], ptype, C_PANEL2, btn_rect))
                by += bh + 4

    def draw(self, surf, state):
        F_TINY, F_SM = self._fonts['tiny'], self._fonts['sm']
        aruco_on = getattr(state, 'aruco_ok', False)
        rec_any  = state.cam_recording

        for label, key, color, btn_rect in self._buttons:
            # Dynamic colour for toggle buttons
            if label == 'ARUCO':
                color = C_CYAN if aruco_on else C_GRAY2
            elif label == 'REC ALL':
                color = C_RED if rec_any else C_GRAY2
            elif label == 'ML LOG':
                color = C_PURPLE if state.data_logging else C_GRAY2

            pygame.draw.rect(surf, C_PANEL2, btn_rect, border_radius=6)
            pygame.draw.rect(surf, color,    btn_rect, width=1, border_radius=6)
            _text(surf, F_SM, label, btn_rect.centerx, btn_rect.centery - 5,
                  C_WHITE, 'center')
            if key:
                _text(surf, F_TINY, f'[{key}]', btn_rect.centerx,
                      btn_rect.centery + 8, C_GRAY2, 'center')

    def hit_test(self, pos):
        """Return (label, key, action) for button under pos, or None."""
        for label, key, color, btn_rect in self._buttons:
            if btn_rect.collidepoint(pos):
                return label, key
        return None

    @property
    def rect(self):
        return self._rect

    @property
    def presets(self):
        return self._presets


# ── Key hints overlay ─────────────────────────────────────────────────────────

def _render_hints(surf, W, H, fonts):
    overlay = pygame.Surface((W, H), pygame.SRCALPHA)
    overlay.fill((0, 0, 15, 210))
    surf.blit(overlay, (0, 0))

    F_BIG, F_SM, F_TINY = fonts['big'], fonts['sm'], fonts['tiny']
    pw, ph = 560, 420
    px, py = (W - pw) // 2, (H - ph) // 2
    panel  = pygame.Rect(px, py, pw, ph)
    pygame.draw.rect(surf, C_PANEL, panel, border_radius=12)
    pygame.draw.rect(surf, C_BORDER, panel, width=2, border_radius=12)

    _text(surf, F_BIG, 'Keyboard Shortcuts', panel.centerx, py + 16, C_WHITE, 'midtop')

    hints = [
        ('E',        'ESTOP — kill motors immediately'),
        ('R',        'Reset ESTOP → MANUAL'),
        ('M',        'Cycle mode (MANUAL / AUTO)'),
        ('A',        'Toggle ArUco detection (all cameras)'),
        ('F',        'Freeze / unfreeze all camera frames'),
        ('L',        'Toggle ML data log'),
        ('V',        'Toggle REC ALL (all cameras)'),
        ('1 – 4',    'Select quad panel 1–4'),
        ('Enter',    'Expand / collapse selected quad'),
        ('Esc',      'Cancel selection / close help'),
        ('H',        'Toggle this help overlay'),
        ('Q',        'Quit'),
        ('',         ''),
        ('Touch',    'Single tap quad → select panel type'),
        ('',         'Double tap quad → expand / collapse'),
    ]

    lx, rx = px + 20, px + 180
    y = py + 52
    for key, desc in hints:
        if key == '':
            y += 6
            continue
        _text(surf, F_SM,   key,  lx, y, C_CYAN)
        _text(surf, F_TINY, desc, rx, y + 3, C_WHITE)
        y += 24

    _text(surf, F_TINY, 'Press H or Esc to close',
          panel.centerx, panel.bottom - 16, C_GRAY, 'midbottom')


# ── Main GUI loop ─────────────────────────────────────────────────────────────

def run_quad_gui(robot: Robot, cfg, fps: int = 10, log_path: str = ''):
    bool_val = lambda x: x.lower() == 'true'

    # Display setup
    display_mode = _cfg(cfg, 'gui', 'display_mode', 'touchscreen')
    W, H, disp_flags = _resolve_display(display_mode)

    strip_side  = _cfg(cfg, 'gui', 'strip_side',  'right').strip().lower()
    strip_w     = min(int(_cfg(cfg, 'gui', 'strip_width', '100')), 120)
    topbar_h    = int(_cfg(cfg, 'gui', 'topbar_height', '52'))
    dbl_tap_ms  = int(_cfg(cfg, 'gui', 'double_tap_ms', '300'))

    pygame.init()
    screen = pygame.display.set_mode((W, H), disp_flags)
    pygame.display.set_caption('Yukon Robot — Quad GUI')
    clock = pygame.time.Clock()

    # Fonts
    fonts = {
        'big':  pygame.font.SysFont('monospace', 20, bold=True),
        'md':   pygame.font.SysFont('monospace', 16),
        'sm':   pygame.font.SysFont('monospace', 13),
        'tiny': pygame.font.SysFont('monospace', 11),
    }

    # Voltage thresholds
    v_thresh = _voltage_thresholds(cfg)

    # Layout regions
    if strip_side == 'left':
        strip_rect = pygame.Rect(0,       topbar_h, strip_w,     H - topbar_h)
        grid_rect  = pygame.Rect(strip_w, topbar_h, W - strip_w, H - topbar_h)
    else:
        grid_rect  = pygame.Rect(0,             topbar_h, W - strip_w, H - topbar_h)
        strip_rect = pygame.Rect(W - strip_w,   topbar_h, strip_w,     H - topbar_h)
    topbar_rect = pygame.Rect(0, 0, W, topbar_h)

    # Button strip
    strip = ButtonStrip(strip_rect, cfg, fonts)

    # Active panel layout — load from [layout_presets] active_preset
    active_preset = _cfg(cfg, 'layout_presets', 'active_preset', 'Race')
    presets = strip.presets
    default_layout = presets.get(active_preset, list(presets.values())[0] if presets else
                                  ['front_left', 'front_right', 'rear', 'lidar'])
    # Ensure exactly 4 panels
    while len(default_layout) < 4:
        default_layout.append('telemetry')
    quad_panels = list(default_layout[:4])

    # Interaction state
    selected_quad  = None   # 0-3 or None
    expanded_quad  = None   # 0-3 or None
    show_hints     = False
    frozen         = False
    strip_mode     = 'normal'

    # Double-tap tracking
    last_tap_quad  = None
    last_tap_time  = 0.0

    # GPS track history
    track_pts = []

    # Reusable rects for top bar button hit-testing
    topbar_estop_rect = pygame.Rect(0, 0, 0, 0)
    topbar_reset_rect = pygame.Rect(0, 0, 0, 0)
    # Per-quad REC button rects — updated each frame, hit-tested in MOUSEBUTTONDOWN
    quad_rec_rects = [pygame.Rect(0, 0, 0, 0)] * 4

    tick = 0

    def _quad_rects():
        """Return list of 4 pygame.Rect for the quad cells (or 1 expanded)."""
        if expanded_quad is not None:
            return [grid_rect.copy() if i == expanded_quad
                    else pygame.Rect(0, 0, 0, 0) for i in range(4)]
        gw, gh = grid_rect.width, grid_rect.height
        hw, hh = gw // 2, gh // 2
        gx, gy = grid_rect.x, grid_rect.y
        return [
            pygame.Rect(gx,      gy,      hw, hh),  # 0 top-left
            pygame.Rect(gx + hw, gy,      hw, hh),  # 1 top-right
            pygame.Rect(gx,      gy + hh, hw, hh),  # 2 bottom-left
            pygame.Rect(gx + hw, gy + hh, hw, hh),  # 3 bottom-right
        ]

    def _set_strip_mode(mode):
        nonlocal strip_mode
        strip_mode = mode
        strip.set_mode(mode)

    def _handle_button(label):
        nonlocal selected_quad, expanded_quad, frozen, show_hints
        if label == '← BACK' or label == 'Esc':
            selected_quad = None
            _set_strip_mode('normal')
        elif label == 'ESTOP':
            robot.estop()
        elif label == 'RESET':
            robot.reset_estop()
        elif label == 'MODE':
            s = robot.get_state()
            if s.mode == RobotMode.MANUAL:
                robot.set_mode(RobotMode.AUTO)
            else:
                robot.set_mode(RobotMode.MANUAL)
        elif label == 'ARUCO':
            robot.toggle_aruco()
        elif label == 'FREEZE':
            frozen = not frozen
        elif label == 'ML LOG':
            s = robot.get_state()
            if s.data_logging:
                robot.stop_data_log()
            else:
                robot.start_data_log()
        elif label == 'REC ALL':
            s = robot.get_state()
            if s.cam_recording:
                robot.stop_cam_recording('all')
            else:
                robot.start_cam_recording('all')
        elif label in PANEL_LABELS.values():
            # Panel-type selection in SELECT mode
            ptype = next((k for k, v in PANEL_LABELS.items() if v == label), None)
            if ptype and selected_quad is not None:
                quad_panels[selected_quad] = ptype
            selected_quad = None
            _set_strip_mode('normal')
        else:
            # Preset button
            panels = strip.presets.get(label)
            if panels:
                while len(panels) < 4:
                    panels.append('telemetry')
                for i in range(4):
                    quad_panels[i] = panels[i]
                expanded_quad = None
                selected_quad = None
                _set_strip_mode('normal')

    def _handle_quad_tap(qi, double_tap=False):
        nonlocal selected_quad, expanded_quad
        if double_tap:
            if expanded_quad == qi:
                expanded_quad = None
            else:
                expanded_quad = qi
            selected_quad = None
            _set_strip_mode('normal')
        else:
            if strip_mode == 'normal':
                selected_quad = qi
                _set_strip_mode('select')
            elif strip_mode == 'select' and selected_quad == qi:
                # Second tap on same quad — cancel
                selected_quad = None
                _set_strip_mode('normal')

    running = True
    while running:
        tick += 1
        state = robot.get_state()

        # Update GPS track
        if state.gps.latitude is not None and state.gps_ok:
            if not track_pts or abs(state.gps.latitude  - track_pts[-1][0]) > 1e-6 or \
                               abs(state.gps.longitude - track_pts[-1][1]) > 1e-6:
                track_pts.append((state.gps.latitude, state.gps.longitude))
                if len(track_pts) > 500:
                    track_pts.pop(0)

        # ── Events ────────────────────────────────────────────────────────────
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                running = False

            elif ev.type == pygame.MOUSEBUTTONDOWN and ev.button == 1:
                pos = ev.pos

                # Top-bar buttons
                if topbar_estop_rect.collidepoint(pos):
                    robot.estop()
                    continue
                if topbar_reset_rect.collidepoint(pos):
                    robot.reset_estop()
                    continue

                # Strip buttons
                hit = strip.hit_test(pos)
                if hit:
                    _handle_button(hit[0])
                    continue

                # Per-quad REC buttons
                _rec_hit = False
                for qi, rb in enumerate(quad_rec_rects):
                    if rb.width > 0 and rb.collidepoint(pos):
                        ptype = quad_panels[qi]
                        if ptype == 'front_left':
                            if state.cam_front_left_recording:
                                robot.stop_cam_recording('front_left')
                            else:
                                robot.start_cam_recording('front_left')
                        elif ptype == 'front_right':
                            if state.cam_front_right_recording:
                                robot.stop_cam_recording('front_right')
                            else:
                                robot.start_cam_recording('front_right')
                        elif ptype == 'rear':
                            if state.cam_rear_recording:
                                robot.stop_cam_recording('rear')
                            else:
                                robot.start_cam_recording('rear')
                        _rec_hit = True
                        break
                if _rec_hit:
                    continue

                # Quad taps
                if show_hints:
                    show_hints = False
                    continue
                if strip_mode == 'select' and not strip_rect.collidepoint(pos):
                    # Check for double-tap before cancelling — a quick second tap
                    # on the same quad should expand it, not just deselect.
                    rects = _quad_rects()
                    dbl_handled = False
                    for qi, qr in enumerate(rects):
                        if qr.width > 0 and qr.collidepoint(pos):
                            now = time.monotonic()
                            if (last_tap_quad == qi and
                                    (now - last_tap_time) * 1000 < dbl_tap_ms):
                                last_tap_quad = qi
                                last_tap_time = now
                                _handle_quad_tap(qi, double_tap=True)
                                dbl_handled = True
                            break
                    if not dbl_handled:
                        selected_quad = None
                        _set_strip_mode('normal')
                    continue

                rects = _quad_rects()
                for qi, qr in enumerate(rects):
                    if qr.width > 0 and qr.collidepoint(pos):
                        now = time.monotonic()
                        double = (last_tap_quad == qi and
                                  (now - last_tap_time) * 1000 < dbl_tap_ms)
                        last_tap_quad = qi
                        last_tap_time = now
                        _handle_quad_tap(qi, double_tap=double)
                        break

            elif ev.type == pygame.KEYDOWN:
                key = ev.key
                if show_hints:
                    show_hints = False
                    continue
                if key == pygame.K_q or key == pygame.K_ESCAPE:
                    if strip_mode == 'select':
                        selected_quad = None
                        _set_strip_mode('normal')
                    elif expanded_quad is not None:
                        expanded_quad = None
                    else:
                        running = False
                elif key == pygame.K_h:
                    show_hints = not show_hints
                elif key == pygame.K_e:
                    robot.estop()
                elif key == pygame.K_r:
                    robot.reset_estop()
                elif key == pygame.K_m:
                    _handle_button('MODE')
                elif key == pygame.K_a:
                    robot.toggle_aruco()
                elif key == pygame.K_f:
                    frozen = not frozen
                elif key == pygame.K_l:
                    _handle_button('ML LOG')
                elif key == pygame.K_v:
                    _handle_button('REC ALL')
                elif key == pygame.K_RETURN:
                    if selected_quad is not None:
                        _handle_quad_tap(selected_quad, double_tap=True)
                elif key in (pygame.K_1, pygame.K_2, pygame.K_3, pygame.K_4):
                    qi = key - pygame.K_1
                    _handle_quad_tap(qi)

        # ── Render ────────────────────────────────────────────────────────────
        screen.fill(C_BG)

        # Top bar
        _render_topbar(screen, topbar_rect, state, v_thresh, tick, fonts,
                       topbar_estop_rect, topbar_reset_rect)

        # Quads
        rects = _quad_rects()
        for qi, (qr, ptype) in enumerate(zip(rects, quad_panels)):
            if qr.width == 0:
                continue

            # Clipping
            screen.set_clip(qr)

            # Panel background
            border_col = (C_SELECT   if qi == selected_quad else
                          C_EXPANDED if qi == expanded_quad else
                          C_BORDER)
            _panel_bg(screen, qr.inflate(-2, -2), border=border_col)

            inner = qr.inflate(-4, -4)

            if ptype in CAMERA_PANELS:
                cam_map = {'front_left': 'front_left',
                           'front_right': 'front_right',
                           'rear': 'rear'}
                cam_key = cam_map[ptype]
                frame   = None if frozen else robot.get_frame(cam_key)
                rec     = state.cam_front_left_recording  if ptype == 'front_left'  else \
                          state.cam_front_right_recording if ptype == 'front_right' else \
                          state.cam_rear_recording
                cam_ok  = state.cam_front_left_ok  if ptype == 'front_left'  else \
                          state.cam_front_right_ok if ptype == 'front_right' else \
                          state.cam_rear_ok
                _render_camera(screen, inner, frame if cam_ok or frame else None,
                               PANEL_LABELS[ptype], state, cam_key,
                               rec, frozen, state.gate_confirmed, fonts)

                # Per-camera REC button (top-right of panel)
                rbw, rbh = max(strip_w - 16, 72), 28
                rb = pygame.Rect(inner.right - rbw - 4, inner.y + 34, rbw, rbh)
                rcol = C_RED if rec else C_GRAY2
                pygame.draw.rect(screen, rcol, rb, border_radius=5)
                pygame.draw.rect(screen, C_BORDER, rb, width=1, border_radius=5)
                rlbl = '■ STOP' if rec else '● REC'
                _text(screen, fonts['tiny'], rlbl, rb.centerx, rb.centery,
                      C_WHITE, 'center')
                quad_rec_rects[qi] = rb

            elif ptype == 'lidar':
                _render_lidar(screen, inner, state.lidar if state.lidar_ok else None, fonts)
                _text(screen, fonts['sm'], 'LIDAR',
                      inner.x + 8, inner.y + 6, C_GRAY)

            elif ptype == 'gps_sky':
                _text(screen, fonts['sm'], 'GPS SKY',
                      inner.x + 8, inner.y + 6, C_GRAY)
                _render_gps_sky(screen, inner.inflate(0, -24).move(0, 24),
                                state.gps, fonts)

            elif ptype == 'gps_track':
                _text(screen, fonts['sm'], 'GPS TRACK',
                      inner.x + 8, inner.y + 6, C_GRAY)
                _render_gps_track(screen, inner.inflate(0, -24).move(0, 24),
                                  state.gps, track_pts, fonts)

            elif ptype == 'telemetry':
                _text(screen, fonts['sm'], 'TELEMETRY',
                      inner.x + 8, inner.y + 6, C_GRAY)
                _render_telemetry(screen, inner.inflate(-4, -28).move(0, 28),
                                  state, fonts)

            elif ptype == 'system':
                _text(screen, fonts['sm'], 'SYSTEM',
                      inner.x + 8, inner.y + 6, C_GRAY)
                _render_system(screen, inner.inflate(-4, -28).move(0, 28),
                               state.system, fonts)

            elif ptype == 'imu':
                _text(screen, fonts['sm'], 'IMU',
                      inner.x + 8, inner.y + 6, C_GRAY)
                _render_imu(screen, inner.inflate(-4, -28).move(0, 28),
                            state.telemetry, fonts)

            elif ptype == 'depth_map':
                _text(screen, fonts['sm'], 'DEPTH MAP',
                      inner.x + 8, inner.y + 6, C_GRAY)
                _render_depth_map(screen, inner, fonts)

            # Quad number hint (bottom-right, small)
            _text(screen, fonts['tiny'], str(qi + 1),
                  inner.right - 4, inner.bottom - 4, C_GRAY2, 'bottomright')

            screen.set_clip(None)

        # Side strip
        screen.set_clip(strip_rect)
        pygame.draw.rect(screen, C_PANEL, strip_rect)
        pygame.draw.line(screen, C_BORDER,
                         (strip_rect.x if strip_side == 'right' else strip_rect.right,
                          topbar_h),
                         (strip_rect.x if strip_side == 'right' else strip_rect.right,
                          H), 1)
        strip.draw(screen, state)
        screen.set_clip(None)

        # Frozen badge
        if frozen:
            badge = fonts['md'].render('❄ FROZEN', True, C_CYAN)
            screen.blit(badge, (topbar_rect.right - badge.get_width() - 8,
                                topbar_h + 4))

        # Key hints overlay
        if show_hints:
            _render_hints(screen, W, H, fonts)

        pygame.display.flip()
        clock.tick(fps)


# ── main() ───────────────────────────────────────────────────────────────────

def main():
    DEFAULT_CFG = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'robot.ini')

    parser = argparse.ArgumentParser(description='HackyRacingRobot — Quad GUI')
    parser.add_argument('--config',         default=DEFAULT_CFG)
    parser.add_argument('--fps',            default=None,  type=int)
    parser.add_argument('--display',        default=None,
                        help='Display mode: touchscreen | windowed | fullscreen | WxH')
    parser.add_argument('--yukon-port',     default=None)
    parser.add_argument('--ibus-port',      default=None)
    parser.add_argument('--gps-port',       default=None)
    parser.add_argument('--lidar-port',     default=None)
    parser.add_argument('--ntrip-host',     default=None)
    parser.add_argument('--ntrip-port',     default=None, type=int)
    parser.add_argument('--ntrip-mount',    default=None)
    parser.add_argument('--ntrip-user',     default=None)
    parser.add_argument('--ntrip-password', default=None)
    parser.add_argument('--rtcm-port',      default=None)
    parser.add_argument('--rtcm-baud',      default=None, type=int)
    parser.add_argument('--no-camera',      action='store_true', default=False)
    parser.add_argument('--no-lidar',       action='store_true', default=False)
    parser.add_argument('--no-gps',         action='store_true', default=False)
    parser.add_argument('--no-motors',      action='store_true', default=False)
    args = parser.parse_args()

    cfg      = _load_config(args.config)
    bool_val = lambda x: x.lower() == 'true'
    port_val = lambda x: None if x.lower() in ('auto', '') else x

    log_path = setup_logging()
    log.info(f'Log file: {log_path}')
    log.info(f'Config:   {args.config}')

    fps = args.fps if args.fps is not None else _cfg(cfg, 'gui', 'fps', 10, int)

    # Override display_mode from CLI if supplied
    if args.display:
        if not cfg.has_section('gui'):
            cfg.add_section('gui')
        cfg.set('gui', 'display_mode', args.display)

    def arg(cli_val, section, key, fallback, cast=str):
        if cli_val is not None:
            return cli_val
        return _cfg(cfg, section, key, fallback, cast)

    robot = Robot(
        yukon_port     = arg(args.yukon_port,      'robot', 'yukon_port',  None, port_val),
        ibus_port      = arg(args.ibus_port,        'robot', 'ibus_port',   '/dev/ttyAMA3'),
        lidar_port     = arg(args.lidar_port,       'lidar', 'port',        '/dev/ttyAMA0'),
        gps_port       = arg(args.gps_port,         'gps',   'port',        '/dev/ttyUSB0'),
        ntrip_host     = '' if _cfg(cfg, 'ntrip', 'disabled', False, bool_val)
                         else arg(args.ntrip_host,  'ntrip', 'host',        ''),
        ntrip_port     = arg(args.ntrip_port,       'ntrip', 'port',        2101,  int),
        ntrip_mount    = arg(args.ntrip_mount,      'ntrip', 'mount',       ''),
        ntrip_user     = arg(args.ntrip_user,       'ntrip', 'user',        ''),
        ntrip_password = arg(args.ntrip_password,   'ntrip', 'password',    ''),
        rtcm_port      = '' if _cfg(cfg, 'rtcm', 'disabled', False, bool_val)
                         else arg(args.rtcm_port,   'rtcm',  'port',        ''),
        rtcm_baud      = arg(args.rtcm_baud,        'rtcm',  'baud',        115200, int),
        enable_camera  = not (args.no_camera or _cfg(cfg, 'camera', 'disabled', False, bool_val)),
        enable_lidar   = not (args.no_lidar  or _cfg(cfg, 'lidar',  'disabled', False, bool_val)),
        enable_gps     = not (args.no_gps    or _cfg(cfg, 'gps',    'disabled', False, bool_val)),
        cam_width      = _cfg(cfg, 'camera', 'width',    640,  int),
        cam_height     = _cfg(cfg, 'camera', 'height',   480,  int),
        cam_fps        = _cfg(cfg, 'camera', 'fps',      30,   int),
        cam_rotation   = _cfg(cfg, 'camera', 'rotation', 0,    int),
        enable_aruco   = _cfg(cfg, 'aruco',  'enabled',  False, bool_val),
        aruco_dict     = _cfg(cfg, 'aruco',  'dict',     'DICT_4X4_1000'),
        aruco_calib    = _cfg(cfg, 'aruco',  'calib_file', ''),
        aruco_tag_size = _cfg(cfg, 'aruco',  'tag_size', 0.15, float),
        throttle_ch    = _cfg(cfg, 'rc',     'throttle_ch', 3,   int),
        steer_ch       = _cfg(cfg, 'rc',     'steer_ch',    1,   int),
        mode_ch        = _cfg(cfg, 'rc',     'mode_ch',     5,   int),
        speed_ch       = _cfg(cfg, 'rc',     'speed_ch',    6,   int),
        auto_type_ch   = _cfg(cfg, 'rc',     'auto_type_ch', 7,  int),
        gps_log_ch     = _cfg(cfg, 'rc',     'gps_log_ch',  8,   int),
        gps_bookmark_ch= _cfg(cfg, 'rc',     'gps_bookmark_ch', 2, int),
        gps_log_dir    = _cfg(cfg, 'gps',    'log_dir',     ''),
        gps_log_hz     = _cfg(cfg, 'gps',    'log_hz',      5.0, float),
        deadzone       = _cfg(cfg, 'rc',     'deadzone',    30,  int),
        failsafe_s     = _cfg(cfg, 'rc',     'failsafe_s',  0.5, float),
        speed_min      = _cfg(cfg, 'rc',     'speed_min',   0.25, float),
        control_hz     = _cfg(cfg, 'rc',     'control_hz',  50,  int),
        no_motors      = args.no_motors,
        rec_dir               = _cfg(cfg, 'output', 'videos_dir',            ''),
        max_recording_minutes = _cfg(cfg, 'output', 'max_recording_minutes', 0.0, float),
        data_log_dir          = _cfg(cfg, 'output', 'data_log_dir',          ''),
    )
    robot.start()

    try:
        run_quad_gui(robot, cfg=cfg, fps=fps, log_path=log_path)
    except KeyboardInterrupt:
        pass
    finally:
        robot.stop()
        pygame.quit()


if __name__ == '__main__':
    main()
