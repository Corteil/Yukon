#!/usr/bin/env python3
"""
lidar_gui.py — Standalone pygame visualiser for the LD06 LiDAR.

Controls
--------
  Mouse wheel / + -  : zoom (change max display range)
  F                  : freeze / unfreeze scan
  G                  : toggle grid
  Q / Esc            : quit

Usage
-----
  python3 lidar_gui.py [PORT]          # default /dev/ttyAMA0
"""

import math
import sys
import time
from typing import Optional

import pygame

from drivers.ld06 import LD06, LidarScan

# ── Colours (matches robot_gui.py) ───────────────────────────────────────────

C_BG     = ( 18,  18,  30)
C_PANEL  = ( 28,  28,  45)
C_BORDER = ( 60,  60,  90)
C_WHITE  = (230, 230, 240)
C_GRAY   = (130, 130, 150)
C_GREEN  = ( 60, 220,  80)
C_YELLOW = (240, 200,  40)
C_RED    = (220,  60,  60)
C_CYAN   = ( 60, 200, 220)
C_DARK   = ( 10,  10,  20)   # inside the scan circle

# ── Layout ────────────────────────────────────────────────────────────────────

W, H      = 900, 900
BAR_H     = 54
POLAR_CX  = W // 2
POLAR_CY  = H // 2
POLAR_R   = 340   # px — leaves ~110 px above/below for the bars

# ── Zoom presets (mm) ────────────────────────────────────────────────────────

_ZOOM = [500, 1000, 1500, 2000, 3000, 4000, 5000, 6000, 8000, 10000, 12000]
_ZOOM_DEFAULT = 4000


# ── Helpers ───────────────────────────────────────────────────────────────────

def _pt_color(frac: float):
    """Near=red → mid=yellow → far=cyan, matching robot_gui.py."""
    r = int(220 * (1 - frac) + 60  * frac)
    g = int(60  * (1 - frac) + 200 * frac)
    b = int(60  * (1 - frac) + 220 * frac)
    return (r, g, b)


def _zoom_idx(v: int) -> int:
    try:
        return _ZOOM.index(v)
    except ValueError:
        for i, s in enumerate(_ZOOM):
            if s >= v:
                return i
        return len(_ZOOM) - 1


# ── Polar drawing ─────────────────────────────────────────────────────────────

def _draw_polar(surf, scan: LidarScan, trail: Optional[LidarScan],
                max_mm: int, show_grid: bool, frozen: bool,
                font_sm, font_tiny):

    cx, cy, r = POLAR_CX, POLAR_CY, POLAR_R

    # Dark filled circle
    pygame.draw.circle(surf, C_DARK, (cx, cy), r)

    if show_grid:
        # Radial spokes every 30°
        for deg in range(0, 360, 30):
            rad = math.radians(deg)
            x2  = cx + int(r * math.sin(rad))
            y2  = cy - int(r * math.cos(rad))
            pygame.draw.line(surf, C_BORDER, (cx, cy), (x2, y2), 1)

        # Range rings
        for i in range(1, 5):
            ring_r = r * i // 4
            pygame.draw.circle(surf, C_BORDER, (cx, cy), ring_r, 1)
            dist_m = max_mm * i / 4 / 1000
            lbl = font_tiny.render(f"{dist_m:.1f}m", True, C_BORDER)
            surf.blit(lbl, (cx + ring_r + 3, cy - lbl.get_height() // 2))

    # Outer border
    pygame.draw.circle(surf, C_BORDER, (cx, cy), r, 1)

    # Cardinal labels (outside circle)
    for txt, deg in (("N",   0), ("NE",  45), ("E",   90), ("SE", 135),
                     ("S", 180), ("SW", 225), ("W",  270), ("NW", 315)):
        rad    = math.radians(deg)
        offset = r + 18
        lx = cx + int(offset * math.sin(rad))
        ly = cy - int(offset * math.cos(rad))
        lbl = font_sm.render(txt, True, C_GRAY)
        surf.blit(lbl, (lx - lbl.get_width() // 2, ly - lbl.get_height() // 2))

    # Trail — previous scan, dimmed
    if trail and trail.angles:
        for angle, dist in zip(trail.angles, trail.distances):
            if dist <= 0 or dist > max_mm:
                continue
            frac = dist / max_mm
            rad  = math.radians(angle)
            px   = cx + int(frac * r * math.sin(rad))
            py   = cy - int(frac * r * math.cos(rad))
            pygame.draw.circle(surf, (30, 70, 80), (px, py), 2)

    # Current scan — colour by distance
    for angle, dist in zip(scan.angles, scan.distances):
        if dist <= 0 or dist > max_mm:
            continue
        frac = min(dist / max_mm, 1.0)
        rad  = math.radians(angle)
        px   = cx + int(frac * r * math.sin(rad))
        py   = cy - int(frac * r * math.cos(rad))
        pygame.draw.circle(surf, _pt_color(frac), (px, py), 2)

    # Robot marker + forward pointer
    pygame.draw.circle(surf, C_GREEN, (cx, cy), 5)
    pygame.draw.line(surf, C_GREEN, (cx, cy), (cx, cy - 24), 2)

    # FROZEN overlay
    if frozen:
        lbl = font_sm.render("FROZEN", True, C_CYAN)
        surf.blit(lbl, (cx - lbl.get_width() // 2, cy + r - 22))


# ── GUI loop ──────────────────────────────────────────────────────────────────

def run_gui(port: str, fps: int = 20):
    pygame.init()
    screen = pygame.display.set_mode((W, H))
    pygame.display.set_caption("LD06 LiDAR")
    clock = pygame.time.Clock()

    FONT_BIG  = pygame.font.SysFont("monospace", 24, bold=True)
    FONT_MD   = pygame.font.SysFont("monospace", 18)
    FONT_SM   = pygame.font.SysFont("monospace", 14)
    FONT_TINY = pygame.font.SysFont("monospace", 11)

    lidar = LD06(port)
    lidar.start()

    scan     = LidarScan()
    trail    = None
    frozen   = False
    show_grid = True
    max_mm   = _ZOOM_DEFAULT

    last_pts   = -1
    scan_count = 0
    hz_t0      = time.time()
    display_hz = 0.0

    try:
        while True:
            # ── Events ───────────────────────────────────────────────────────
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    return
                elif ev.type == pygame.KEYDOWN:
                    if ev.key in (pygame.K_q, pygame.K_ESCAPE):
                        return
                    elif ev.key == pygame.K_f:
                        frozen = not frozen
                    elif ev.key == pygame.K_g:
                        show_grid = not show_grid
                    elif ev.key in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
                        i = _zoom_idx(max_mm)
                        if i > 0:
                            max_mm = _ZOOM[i - 1]
                    elif ev.key in (pygame.K_MINUS, pygame.K_KP_MINUS):
                        i = _zoom_idx(max_mm)
                        if i < len(_ZOOM) - 1:
                            max_mm = _ZOOM[i + 1]
                elif ev.type == pygame.MOUSEWHEEL:
                    i = _zoom_idx(max_mm)
                    i = max(0, min(len(_ZOOM) - 1, i - ev.y))
                    max_mm = _ZOOM[i]

            # ── Poll lidar ───────────────────────────────────────────────────
            if not frozen:
                new = lidar.get_scan()
                if len(new.angles) != last_pts:
                    trail    = scan
                    scan     = new
                    last_pts = len(new.angles)
                    scan_count += 1
                    now = time.time()
                    dt  = now - hz_t0
                    if dt >= 1.0:
                        display_hz = scan_count / dt
                        scan_count = 0
                        hz_t0      = now

            # ── Render ───────────────────────────────────────────────────────
            screen.fill(C_BG)

            # Polar plot (behind the bars — bars float over it)
            _draw_polar(screen, scan, trail, max_mm, show_grid, frozen,
                        FONT_SM, FONT_TINY)

            # Top bar
            top_rect = pygame.Rect(10, 8, W - 20, BAR_H - 10)
            pygame.draw.rect(screen, C_PANEL,  top_rect, border_radius=6)
            pygame.draw.rect(screen, C_BORDER, top_rect, width=1, border_radius=6)

            screen.blit(FONT_BIG.render("LD06 LiDAR", True, C_WHITE), (22, 16))

            status_txt   = "OK"        if lidar.ok else "NO SIGNAL"
            status_color = C_GREEN     if lidar.ok else C_RED
            st = FONT_MD.render(status_txt, True, status_color)
            screen.blit(st, (W - st.get_width() - 20, 20))

            hints = FONT_TINY.render(
                "Scroll / + − : zoom     F : freeze     G : grid     Q : quit",
                True, C_GRAY)
            screen.blit(hints, (W // 2 - hints.get_width() // 2, 28))

            # Bottom stats bar
            bot_y = H - BAR_H + 4
            pygame.draw.line(screen, C_BORDER, (10, bot_y - 4), (W - 10, bot_y - 4), 1)

            min_dist = min((d for d in scan.distances if d > 0), default=0)

            stats = [
                ("RPM",    f"{scan.rpm:.1f}"),
                ("Scan Hz", f"{display_hz:.1f}"),
                ("Points",  f"{len(scan.angles)}"),
                ("Range",   f"{max_mm / 1000:.1f} m"),
                ("Nearest", f"{min_dist / 1000:.2f} m" if min_dist else "---"),
            ]

            sx = 20
            for label, value in stats:
                lbl_s = FONT_TINY.render(label, True, C_GRAY)
                val_s = FONT_MD.render(value,   True, C_WHITE)
                screen.blit(lbl_s, (sx, bot_y))
                screen.blit(val_s, (sx, bot_y + lbl_s.get_height() + 2))
                sx += max(lbl_s.get_width(), val_s.get_width()) + 30

            pygame.display.flip()
            clock.tick(fps)

    finally:
        lidar.stop()
        pygame.quit()


if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyAMA0"
    run_gui(port)
