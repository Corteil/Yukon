#!/usr/bin/env python3
"""
gps_route_builder.py — GPS route builder for HackyRacingRobot.

Build, edit and export GPS waypoint routes using an interactive map.

Features
--------
  Map          : OpenStreetMap tiles (cached to disk); grid fallback when offline
  Add          : left-click empty map → add waypoint
  Move         : drag waypoint to reposition
  Delete       : right-click waypoint
  Reorder      : drag rows in the sidebar list
  Labels       : click label in sidebar to edit inline
  Import       : load waypoints.json (versioned format) or GPS log CSV
  Export       : save waypoints.json (versioned format: {"version":1, "waypoints":[...]})
  Live mode    : connect to robot_daemon.py → show live GPS position + accuracy ring,
                 record track, optionally send route to GPS navigator

Controls
--------
  Left-click map    : add waypoint (if not over existing)
  Left-drag wp      : move waypoint
  Right-click wp    : delete waypoint
  Scroll            : zoom in / out
  Middle-drag       : pan map
  Right-drag (empty): pan map
  C                 : centre map on all waypoints
  R                 : centre map on robot position (live mode)
  Ctrl+S            : save waypoints.json
  Ctrl+O            : open waypoints.json
  Escape            : quit

Usage
-----
  python3 gps_route_builder.py
  python3 gps_route_builder.py --waypoints my_route.json
  python3 gps_route_builder.py --live      # connect to robot_daemon.py

Dependencies
------------
  pip install pygame requests
"""

import argparse
import csv
import io
import json
import math
import os
import queue
import sys
import threading
import time
import urllib.request
import urllib.error
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Tuple

import pygame
import pygame.freetype

# ── Constants ─────────────────────────────────────────────────────────────────

W, H        = 1280, 780
TILE_SIZE   = 256
TILE_CACHE  = Path.home() / ".cache" / "yukon_map_tiles"
OSM_URL     = "https://tile.openstreetmap.org/{z}/{x}/{y}.png"
USER_AGENT  = "HackyRacingRobotRouteBuilder/1.0 (robot project)"
MAX_ZOOM    = 19
MIN_ZOOM    = 2
DEFAULT_ZOOM= 17
WP_RADIUS   = 12      # waypoint circle radius px
SNAP_DIST   = 18      # px — click within this of WP to select it

# ── Colours ───────────────────────────────────────────────────────────────────

C_BG        = ( 15,  15,  22)
C_SIDEBAR   = ( 20,  20,  32)
C_PANEL     = ( 28,  28,  44)
C_BORDER    = ( 55,  55,  80)
C_WHITE     = (230, 232, 245)
C_GRAY      = (110, 115, 140)
C_LGRAY     = (160, 165, 185)
C_GREEN     = ( 50, 210,  80)
C_YELLOW    = (240, 195,  40)
C_ORANGE    = (240, 130,  40)
C_RED       = (215,  55,  55)
C_CYAN      = ( 50, 195, 220)
C_BLUE      = ( 60, 120, 230)
C_PURPLE    = (160,  80, 220)
C_ROUTE     = ( 50, 195, 220)       # route line
C_WP_FILL   = ( 28,  28,  44)
C_WP_RING   = ( 50, 195, 220)
C_WP_SEL    = (240, 195,  40)
C_TRACK     = (100, 220, 100)       # recorded GPS track
C_ROBOT     = (240, 195,  40)       # live robot position
C_ACCURACY  = ( 50, 195, 220)       # GPS accuracy ring
C_GRID_LINE = ( 40,  44,  60)
C_OFFLINE   = ( 30,  32,  45)

SIDEBAR_W   = 320
MAP_W       = W - SIDEBAR_W

# ── Tile maths (Web Mercator) ─────────────────────────────────────────────────

def _lat_lon_to_tile(lat, lon, zoom):
    n  = 2 ** zoom
    xt = (lon + 180) / 360 * n
    yt = (1 - math.log(math.tan(math.radians(lat)) +
                        1 / math.cos(math.radians(lat))) / math.pi) / 2 * n
    return xt, yt


def _tile_to_lat_lon(xt, yt, zoom):
    n   = 2 ** zoom
    lon = xt / n * 360 - 180
    lat = math.degrees(math.atan(math.sinh(math.pi * (1 - 2 * yt / n))))
    return lat, lon


def _haversine(lat1, lon1, lat2, lon2):
    R = 6_371_000
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a  = math.sin(dp/2)**2 + math.cos(p1)*math.cos(p2)*math.sin(dl/2)**2
    return 2 * R * math.asin(math.sqrt(a))

# ── Waypoint ──────────────────────────────────────────────────────────────────

@dataclass
class Waypoint:
    lat:   float
    lon:   float
    label: str = ""

    def to_dict(self):
        return {"lat": self.lat, "lon": self.lon, "label": self.label}

    @classmethod
    def from_dict(cls, d):
        return cls(lat=float(d["lat"]), lon=float(d["lon"]),
                   label=str(d.get("label", "")))

# ── Tile cache ────────────────────────────────────────────────────────────────

class TileCache:
    def __init__(self):
        TILE_CACHE.mkdir(parents=True, exist_ok=True)
        self._mem:  dict = {}          # (z,x,y) → Surface
        self._fail: set  = set()       # (z,x,y) → skip (404 / timeout)
        self._q:    queue.Queue = queue.Queue()
        self._downloading: set = set()
        t = threading.Thread(target=self._worker, daemon=True)
        t.start()

    def get(self, z, x, y) -> Optional[pygame.Surface]:
        key = (z, x, y)
        if key in self._mem:
            return self._mem[key]
        if key in self._fail:
            return None
        # Check disk
        path = TILE_CACHE / f"{z}_{x}_{y}.png"
        if path.exists():
            try:
                surf = pygame.image.load(str(path))
                self._mem[key] = surf
                return surf
            except Exception:
                pass
        # Enqueue download
        if key not in self._downloading:
            self._downloading.add(key)
            self._q.put(key)
        return None

    def _worker(self):
        while True:
            key = self._q.get()
            z, x, y = key
            url  = OSM_URL.format(z=z, x=x, y=y)
            path = TILE_CACHE / f"{z}_{x}_{y}.png"
            try:
                req = urllib.request.Request(url,
                      headers={"User-Agent": USER_AGENT})
                with urllib.request.urlopen(req, timeout=8) as resp:
                    data = resp.read()
                path.write_bytes(data)
                surf = pygame.image.load(io.BytesIO(data))
                self._mem[key] = surf
            except Exception:
                self._fail.add(key)
            finally:
                self._downloading.discard(key)
            self._q.task_done()

# ── Map view ──────────────────────────────────────────────────────────────────

class MapView:
    """Manages zoom + pan and converts between pixel and lat/lon coords."""

    def __init__(self, cx_lat=52.2053, cx_lon=0.1218, zoom=DEFAULT_ZOOM):
        self.zoom   = zoom
        self._cx_tx, self._cx_ty = _lat_lon_to_tile(cx_lat, cx_lon, zoom)

    # Map-rect centre in tile coords
    @property
    def _origin_tx(self):
        return self._cx_tx - (MAP_W / 2) / TILE_SIZE

    @property
    def _origin_ty(self):
        return self._cx_ty - (H / 2) / TILE_SIZE

    def ll_to_px(self, lat, lon) -> Tuple[int, int]:
        tx, ty = _lat_lon_to_tile(lat, lon, self.zoom)
        px = int((tx - self._origin_tx) * TILE_SIZE)
        py = int((ty - self._origin_ty) * TILE_SIZE)
        return px, py

    def px_to_ll(self, px, py) -> Tuple[float, float]:
        tx = self._origin_tx + px / TILE_SIZE
        ty = self._origin_ty + py / TILE_SIZE
        return _tile_to_lat_lon(tx, ty, self.zoom)

    def pan(self, dpx, dpy):
        self._cx_tx -= dpx / TILE_SIZE
        self._cx_ty -= dpy / TILE_SIZE

    def zoom_at(self, px, py, dz):
        lat, lon = self.px_to_ll(px, py)
        self.zoom = max(MIN_ZOOM, min(MAX_ZOOM, self.zoom + dz))
        self._cx_tx, self._cx_ty = _lat_lon_to_tile(lat, lon, self.zoom)
        # Keep the pixel under cursor fixed
        tx2, ty2 = _lat_lon_to_tile(lat, lon, self.zoom)
        self._cx_tx += (px - MAP_W / 2) / TILE_SIZE - (tx2 - self._cx_tx)
        self._cx_ty += (py - H    / 2) / TILE_SIZE - (ty2 - self._cx_ty)
        # Simpler: recentre so cursor point stays put
        self._cx_tx = tx2 - (px - MAP_W / 2) / TILE_SIZE
        self._cx_ty = ty2 - (py - H    / 2) / TILE_SIZE

    def centre_on(self, lat, lon):
        self._cx_tx, self._cx_ty = _lat_lon_to_tile(lat, lon, self.zoom)

    def fit_waypoints(self, waypoints: List[Waypoint]):
        if not waypoints:
            return
        lats = [w.lat for w in waypoints]
        lons = [w.lon for w in waypoints]
        clat = (min(lats) + max(lats)) / 2
        clon = (min(lons) + max(lons)) / 2
        self.centre_on(clat, clon)
        # Zoom out until all WPs fit with margin
        for _ in range(MAX_ZOOM - MIN_ZOOM):
            pxs = [self.ll_to_px(w.lat, w.lon) for w in waypoints]
            xs  = [p[0] for p in pxs]
            ys  = [p[1] for p in pxs]
            margin = 60
            if (min(xs) > margin and max(xs) < MAP_W - margin and
                    min(ys) > margin and max(ys) < H - margin):
                break
            self.zoom = max(MIN_ZOOM, self.zoom - 1)
            self.centre_on(clat, clon)

# ── Sidebar ───────────────────────────────────────────────────────────────────

class Sidebar:
    ROW_H     = 44
    SCROLL_W  = 8

    def __init__(self, x, font_sm, font_tiny):
        self.x        = x
        self.font_sm  = font_sm
        self.font_tiny= font_tiny
        self._scroll  = 0      # px offset
        self._drag_row: Optional[int] = None
        self._drag_y0:  int = 0
        self._drag_orig: int = 0
        self._edit_idx: Optional[int] = None
        self._edit_text: str = ""

    # ── drawing ──────────────────────────────────────────────────────────────

    def draw(self, surf, waypoints, selected_idx, live_state):
        rect = pygame.Rect(self.x, 0, SIDEBAR_W, H)
        pygame.draw.rect(surf, C_SIDEBAR, rect)
        pygame.draw.line(surf, C_BORDER, (self.x, 0), (self.x, H), 1)

        y = 12
        # Title
        t = self.font_sm.render("ROUTE BUILDER", True, C_CYAN)
        surf.blit(t, (self.x + 14, y))
        y += 24

        # Live status
        if live_state:
            gps   = live_state.get('gps')
            mode  = live_state.get('mode', '--')
            nav   = live_state.get('nav_state', 'IDLE')
            col   = C_GREEN if mode == 'AUTO' else C_YELLOW
            t2 = self.font_tiny.render(
                f"LIVE  {mode}  NAV:{nav}", True, col)
            surf.blit(t2, (self.x + 14, y))
            y += 16
            if gps and gps.get('latitude') is not None:
                t3 = self.font_tiny.render(
                    f"GPS {gps['fix_quality_name']}  "
                    f"{gps['latitude']:.6f}, {gps['longitude']:.6f}",
                    True, C_LGRAY)
                surf.blit(t3, (self.x + 14, y))
            y += 18
        else:
            t2 = self.font_tiny.render("Offline (no robot)", True, C_GRAY)
            surf.blit(t2, (self.x + 14, y))
            y += 18

        pygame.draw.line(surf, C_BORDER, (self.x + 8, y), (self.x + SIDEBAR_W - 8, y), 1)
        y += 8

        # Waypoint count
        t = self.font_tiny.render(
            f"{len(waypoints)} waypoint{'s' if len(waypoints) != 1 else ''}  "
            f"{'  total: ' + _fmt_distance(waypoints) if len(waypoints) > 1 else ''}",
            True, C_LGRAY)
        surf.blit(t, (self.x + 14, y))
        y += 20

        # List area
        list_top = y
        list_h   = H - list_top - 160
        clip     = pygame.Rect(self.x, list_top, SIDEBAR_W, list_h)
        surf.set_clip(clip)

        for i, wp in enumerate(waypoints):
            ry = list_top + i * self.ROW_H - self._scroll
            if ry + self.ROW_H < list_top or ry > list_top + list_h:
                continue
            row_rect = pygame.Rect(self.x + 4, ry + 2,
                                   SIDEBAR_W - 12, self.ROW_H - 4)
            is_sel = (i == selected_idx)
            is_drag = (i == self._drag_row)
            bg = (40, 44, 62) if is_sel else (28, 28, 44)
            pygame.draw.rect(surf, bg, row_rect, border_radius=6)
            if is_sel:
                pygame.draw.rect(surf, C_WP_SEL, row_rect, width=1, border_radius=6)

            # Number badge
            badge = pygame.Rect(self.x + 10, ry + 10, 24, 24)
            pygame.draw.circle(surf, C_WP_RING,
                               badge.center, 12)
            nt = self.font_tiny.render(str(i + 1), True, C_BG)
            surf.blit(nt, (badge.centerx - nt.get_width()//2,
                           badge.centery - nt.get_height()//2))

            # Label / edit field
            lx = self.x + 40
            if i == self._edit_idx:
                cursor = "|" if (pygame.time.get_ticks() // 400) % 2 == 0 else " "
                et = self.font_sm.render(self._edit_text + cursor, True, C_YELLOW)
                surf.blit(et, (lx, ry + 8))
            else:
                label = wp.label or f"WP {i+1}"
                lt    = self.font_sm.render(label, True, C_WHITE if is_sel else C_LGRAY)
                surf.blit(lt, (lx, ry + 7))
                coords = self.font_tiny.render(
                    f"{wp.lat:.6f}, {wp.lon:.6f}", True, C_GRAY)
                surf.blit(coords, (lx, ry + 24))

            # Drag handle (≡)
            hx = self.x + SIDEBAR_W - 24
            for dy in (-4, 0, 4):
                pygame.draw.line(surf, C_BORDER,
                                 (hx, ry + self.ROW_H//2 + dy),
                                 (hx + 12, ry + self.ROW_H//2 + dy), 1)

        surf.set_clip(None)

        # Scroll bar
        total_h = len(waypoints) * self.ROW_H
        if total_h > list_h:
            bar_h  = max(20, int(list_h * list_h / total_h))
            bar_y  = list_top + int(self._scroll / total_h * list_h)
            bar_r  = pygame.Rect(self.x + SIDEBAR_W - self.SCROLL_W - 2,
                                  bar_y, self.SCROLL_W, bar_h)
            pygame.draw.rect(surf, C_BORDER, bar_r, border_radius=4)

        # Separator
        btn_y = H - 155
        pygame.draw.line(surf, C_BORDER,
                         (self.x + 8, btn_y), (self.x + SIDEBAR_W - 8, btn_y), 1)

        return list_top, list_h

    # ── button drawing (called by main) ──────────────────────────────────────

    def draw_buttons(self, surf, live_connected):
        bx = self.x + 10
        bw = (SIDEBAR_W - 28) // 2
        rows = [
            (H - 148, [("Import JSON", C_CYAN,   'import_json'),
                       ("Import CSV",  C_CYAN,   'import_csv')]),
            (H - 100, [("Export JSON", C_GREEN,  'export'),
                       ("Clear All",   C_RED,    'clear')]),
            (H -  52, [("Connect" if not live_connected else "Disconnect",
                        C_GREEN if not live_connected else C_ORANGE,
                        'connect'),
                       ("Send Route",  C_YELLOW, 'send_route')]),
        ]
        btn_rects = {}
        for row_y, buttons in rows:
            x = bx
            for label, color, key in buttons:
                r = pygame.Rect(x, row_y, bw, 36)
                pygame.draw.rect(surf, C_PANEL,  r, border_radius=6)
                pygame.draw.rect(surf, color,    r, width=1, border_radius=6)
                t = self.font_tiny.render(label, True, color)
                surf.blit(t, (r.centerx - t.get_width()//2,
                              r.centery - t.get_height()//2))
                btn_rects[key] = r
                x += bw + 8
        return btn_rects

    # ── hit testing ──────────────────────────────────────────────────────────

    def row_at(self, mx, my, list_top, list_h, n_waypoints):
        if not (self.x < mx < self.x + SIDEBAR_W):
            return None
        if not (list_top < my < list_top + list_h):
            return None
        i = (my - list_top + self._scroll) // self.ROW_H
        if 0 <= i < n_waypoints:
            return i
        return None

    def scroll_by(self, dy, n_waypoints, list_h):
        total = n_waypoints * self.ROW_H
        self._scroll = max(0, min(self._scroll - dy * 30,
                                  max(0, total - list_h)))

    # ── label editing ─────────────────────────────────────────────────────────

    def start_edit(self, idx, current_label):
        self._edit_idx  = idx
        self._edit_text = current_label

    def commit_edit(self, waypoints):
        if self._edit_idx is not None and 0 <= self._edit_idx < len(waypoints):
            waypoints[self._edit_idx].label = self._edit_text
        self._edit_idx = None

    def cancel_edit(self):
        self._edit_idx = None

    def feed_key(self, event):
        if self._edit_idx is None:
            return
        if event.key == pygame.K_RETURN:
            return 'commit'
        elif event.key == pygame.K_ESCAPE:
            return 'cancel'
        elif event.key == pygame.K_BACKSPACE:
            self._edit_text = self._edit_text[:-1]
        elif event.unicode and event.unicode.isprintable():
            self._edit_text += event.unicode
        return None


def _fmt_distance(waypoints):
    if len(waypoints) < 2:
        return ""
    total = sum(_haversine(waypoints[i].lat, waypoints[i].lon,
                           waypoints[i+1].lat, waypoints[i+1].lon)
                for i in range(len(waypoints) - 1))
    if total >= 1000:
        return f"{total/1000:.2f} km"
    return f"{total:.0f} m"

# ── Live robot thread ─────────────────────────────────────────────────────────

class LiveRobot:
    """Polls robot_daemon.py Robot state in a background thread."""

    def __init__(self):
        self._state    = None
        self._track:   List[Tuple[float,float]] = []
        self._lock     = threading.Lock()
        self._running  = False
        self._robot    = None

    def connect(self):
        try:
            sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
            from robot_daemon import Robot, RobotMode
            self._robot   = None   # use existing instance if running
            self._running = True
            t = threading.Thread(target=self._poll, daemon=True)
            t.start()
            return True
        except ImportError as e:
            print(f"Cannot import robot_daemon.py: {e}", file=sys.stderr)
            return False

    def connect_to(self, robot_instance):
        """Connect to an already-running Robot instance."""
        self._robot   = robot_instance
        self._running = True
        t = threading.Thread(target=self._poll, daemon=True)
        t.start()

    def disconnect(self):
        self._running = False
        self._robot   = None

    def _poll(self):
        while self._running:
            if self._robot is None:
                time.sleep(0.5)
                continue
            try:
                state = self._robot.get_state()
                g     = state.gps
                s = {
                    'mode':             state.mode.name,
                    'nav_state':        state.nav_state,
                    'nav_wp':           state.nav_wp,
                    'nav_wp_dist':      state.nav_wp_dist,
                    'gps': {
                        'latitude':         g.latitude,
                        'longitude':        g.longitude,
                        'fix_quality':      g.fix_quality,
                        'fix_quality_name': g.fix_quality_name,
                        'h_error_m':        g.h_error_m,
                        'heading':          g.heading,
                    } if g else None,
                }
                with self._lock:
                    self._state = s
                    if g and g.latitude is not None:
                        self._track.append((g.latitude, g.longitude))
                        if len(self._track) > 5000:
                            self._track = self._track[-5000:]
            except Exception:
                pass
            time.sleep(0.1)

    def get_state(self):
        with self._lock:
            return self._state

    def get_track(self):
        with self._lock:
            return list(self._track)

    @property
    def connected(self):
        return self._running

# ── File I/O ──────────────────────────────────────────────────────────────────

def load_json(path) -> List[Waypoint]:
    with open(path) as f:
        data = json.load(f)
    return [Waypoint.from_dict(d) for d in data['waypoints']]


def save_json(waypoints, path):
    with open(path, 'w') as f:
        json.dump({'version': 1, 'waypoints': [w.to_dict() for w in waypoints]}, f, indent=2)


def load_csv(path) -> List[Waypoint]:
    """Import from robot_daemon.py GPS log CSV. Uses latitude/longitude columns."""
    wps   = []
    seen  = set()
    with open(path, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                lat = float(row.get('latitude') or row.get('lat') or 0)
                lon = float(row.get('longitude') or row.get('lon') or 0)
            except (ValueError, TypeError):
                continue
            if lat == 0 and lon == 0:
                continue
            # Deduplicate points closer than 1 m
            key = (round(lat, 5), round(lon, 5))
            if key in seen:
                continue
            seen.add(key)
            bm  = row.get('bookmark', 'false').lower() == 'true'
            lbl = 'Bookmark' if bm else ''
            wps.append(Waypoint(lat=lat, lon=lon, label=lbl))
    return wps


def file_dialog_open(exts=('.json', '.csv')):
    """Simple path prompt (no tkinter dependency)."""
    # Try tkinter first
    try:
        import tkinter as tk
        from tkinter import filedialog
        root = tk.Tk(); root.withdraw()
        path = filedialog.askopenfilename(
            filetypes=[("Route files", " ".join(f"*{e}" for e in exts)),
                       ("All files", "*.*")])
        root.destroy()
        return path or None
    except Exception:
        pass
    return None


def file_dialog_save(default='waypoints.json'):
    try:
        import tkinter as tk
        from tkinter import filedialog
        root = tk.Tk(); root.withdraw()
        path = filedialog.asksaveasfilename(
            defaultextension='.json',
            initialfile=default,
            filetypes=[("JSON route", "*.json"), ("All files", "*.*")])
        root.destroy()
        return path or None
    except Exception:
        return default

# ── Toast notification ────────────────────────────────────────────────────────

class Toast:
    def __init__(self):
        self._msg    = ""
        self._color  = C_GREEN
        self._until  = 0.0

    def show(self, msg, color=C_GREEN, duration=2.5):
        self._msg   = msg
        self._color = color
        self._until = time.monotonic() + duration

    def draw(self, surf, font):
        if time.monotonic() > self._until:
            return
        t   = font.render(self._msg, True, self._color)
        pad = 14
        r   = pygame.Rect(MAP_W//2 - t.get_width()//2 - pad,
                          H - 60, t.get_width() + pad*2, 36)
        pygame.draw.rect(surf, C_PANEL,  r, border_radius=8)
        pygame.draw.rect(surf, self._color, r, width=1, border_radius=8)
        surf.blit(t, (r.x + pad, r.centery - t.get_height()//2))

# ── Main app ──────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="HackyRacingRobot GPS route builder")
    parser.add_argument("--waypoints", default=None, help="Load waypoints JSON on start")
    parser.add_argument("--live",      action="store_true", help="Connect to robot.py on start")
    args = parser.parse_args()

    pygame.init()
    screen = pygame.display.set_mode((W, H))
    pygame.display.set_caption("HackyRacingRobot GPS Route Builder")
    clock  = pygame.time.Clock()

    font_big  = pygame.font.SysFont("monospace", 22, bold=True)
    font_md   = pygame.font.SysFont("monospace", 16)
    font_sm   = pygame.font.SysFont("monospace", 13)
    font_tiny = pygame.font.SysFont("monospace", 11)

    tiles     = TileCache()
    view      = MapView(zoom=DEFAULT_ZOOM)
    sidebar   = Sidebar(MAP_W, font_sm, font_tiny)
    toast     = Toast()
    live      = LiveRobot()
    waypoints: List[Waypoint] = []
    selected  = -1

    # Interaction state
    dragging_wp    = -1       # index of WP being dragged on map
    drag_offset    = (0, 0)
    panning        = False
    pan_start      = (0, 0)
    pan_origin_tx  = 0.0
    pan_origin_ty  = 0.0
    list_top       = 100
    list_h         = H - 260
    drag_row       = -1       # sidebar list row being reordered
    drag_row_y     = 0
    drag_row_orig  = -1

    def wp_at_px(px, py):
        for i, wp in enumerate(waypoints):
            wx, wy = view.ll_to_px(wp.lat, wp.lon)
            if math.hypot(px - wx, py - wy) <= SNAP_DIST:
                return i
        return -1

    # Load initial waypoints
    if args.waypoints:
        try:
            waypoints = load_json(args.waypoints)
            view.fit_waypoints(waypoints)
            toast.show(f"Loaded {len(waypoints)} waypoints")
        except Exception as e:
            toast.show(f"Load failed: {e}", C_RED)

    if args.live:
        live.connect()

    # ── Offline grid surface (drawn once per zoom) ────────────────────────────
    _grid_surf = None
    _grid_zoom = -1

    def get_grid():
        nonlocal _grid_surf, _grid_zoom
        if view.zoom != _grid_zoom:
            _grid_surf = pygame.Surface((MAP_W, H))
            _grid_surf.fill(C_OFFLINE)
            step = TILE_SIZE
            for gx in range(0, MAP_W, step):
                pygame.draw.line(_grid_surf, C_GRID_LINE, (gx, 0), (gx, H), 1)
            for gy in range(0, H, step):
                pygame.draw.line(_grid_surf, C_GRID_LINE, (0, gy), (MAP_W, gy), 1)
            _grid_zoom = view.zoom
        return _grid_surf

    running = True
    while running:
        clock.tick(30)
        live_state = live.get_state() if live.connected else None

        # ── Events ───────────────────────────────────────────────────────────
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            # ── Keyboard ─────────────────────────────────────────────────────
            elif event.type == pygame.KEYDOWN:

                # Editing label
                result = sidebar.feed_key(event)
                if result == 'commit':
                    sidebar.commit_edit(waypoints)
                elif result == 'cancel':
                    sidebar.cancel_edit()
                elif result is not None:
                    pass  # key consumed by edit
                else:
                    # Normal keys
                    mods = pygame.key.get_mods()
                    if event.key in (pygame.K_q, pygame.K_ESCAPE):
                        running = False
                    elif event.key == pygame.K_c:
                        if waypoints:
                            view.fit_waypoints(waypoints)
                    elif event.key == pygame.K_r and live_state:
                        g = live_state.get('gps') or {}
                        if g.get('latitude'):
                            view.centre_on(g['latitude'], g['longitude'])
                    elif event.key == pygame.K_s and mods & pygame.KMOD_CTRL:
                        path = file_dialog_save()
                        if path:
                            try:
                                save_json(waypoints, path)
                                toast.show(f"Saved {len(waypoints)} waypoints → {Path(path).name}")
                            except Exception as e:
                                toast.show(f"Save failed: {e}", C_RED)
                    elif event.key == pygame.K_o and mods & pygame.KMOD_CTRL:
                        path = file_dialog_open()
                        if path:
                            try:
                                new = load_json(path) if path.endswith('.json') \
                                      else load_csv(path)
                                waypoints = new
                                view.fit_waypoints(waypoints)
                                toast.show(f"Loaded {len(waypoints)} waypoints")
                            except Exception as e:
                                toast.show(f"Load failed: {e}", C_RED)
                    elif event.key == pygame.K_DELETE and 0 <= selected < len(waypoints):
                        waypoints.pop(selected)
                        selected = min(selected, len(waypoints) - 1)

            # ── Mouse button down ─────────────────────────────────────────────
            elif event.type == pygame.MOUSEBUTTONDOWN:
                mx, my = event.pos
                on_map = mx < MAP_W

                if event.button == 4:   # scroll up
                    if on_map:
                        view.zoom_at(mx, my, +1)
                    else:
                        sidebar.scroll_by(+1, len(waypoints), list_h)
                elif event.button == 5: # scroll down
                    if on_map:
                        view.zoom_at(mx, my, -1)
                    else:
                        sidebar.scroll_by(-1, len(waypoints), list_h)

                elif event.button == 1 and on_map:
                    # Check if clicking existing WP
                    idx = wp_at_px(mx, my)
                    if idx >= 0:
                        selected    = idx
                        dragging_wp = idx
                        wx, wy      = view.ll_to_px(waypoints[idx].lat,
                                                     waypoints[idx].lon)
                        drag_offset = (mx - wx, my - wy)
                        sidebar.cancel_edit()
                    else:
                        # Add new waypoint
                        lat, lon = view.px_to_ll(mx, my)
                        waypoints.append(Waypoint(lat=lat, lon=lon,
                                                   label=f"WP {len(waypoints)+1}"))
                        selected    = len(waypoints) - 1
                        dragging_wp = selected
                        drag_offset = (0, 0)

                elif event.button == 1 and not on_map:
                    sidebar.cancel_edit()
                    row = sidebar.row_at(mx, my, list_top, list_h, len(waypoints))
                    if row is not None:
                        selected      = row
                        drag_row      = row
                        drag_row_orig = row
                        drag_row_y    = my
                        # Double-click: start label edit
                        # (single click selects, handled above)

                elif event.button == 2 or (event.button == 3 and on_map):
                    # Pan
                    panning       = True
                    pan_start     = (mx, my)
                    pan_origin_tx = view._cx_tx
                    pan_origin_ty = view._cx_ty

                elif event.button == 3 and on_map:
                    pass  # handled below

            # ── Mouse button up ───────────────────────────────────────────────
            elif event.type == pygame.MOUSEBUTTONUP:
                mx, my = event.pos
                on_map = mx < MAP_W

                if event.button == 1:
                    dragging_wp = -1
                    if drag_row >= 0:
                        drag_row = -1

                elif event.button == 3 and on_map:
                    # Right-click: delete WP if over one
                    idx = wp_at_px(mx, my)
                    if idx >= 0:
                        waypoints.pop(idx)
                        selected = min(selected, len(waypoints) - 1)

                if event.button in (2, 3):
                    panning = False

            # ── Mouse motion ──────────────────────────────────────────────────
            elif event.type == pygame.MOUSEMOTION:
                mx, my = event.pos
                on_map = mx < MAP_W

                if dragging_wp >= 0 and on_map:
                    tx = mx - drag_offset[0]
                    ty = my - drag_offset[1]
                    lat, lon = view.px_to_ll(tx, ty)
                    waypoints[dragging_wp].lat = lat
                    waypoints[dragging_wp].lon = lon

                elif panning:
                    dx = mx - pan_start[0]
                    dy = my - pan_start[1]
                    view._cx_tx = pan_origin_tx - dx / TILE_SIZE
                    view._cx_ty = pan_origin_ty - dy / TILE_SIZE

                elif drag_row >= 0:
                    delta = (my - drag_row_y) // sidebar.ROW_H
                    new_row = max(0, min(len(waypoints)-1, drag_row_orig + delta))
                    if new_row != drag_row and len(waypoints) > 1:
                        waypoints.insert(new_row, waypoints.pop(drag_row))
                        selected  = new_row
                        drag_row  = new_row

            # ── Double-click for label edit ───────────────────────────────────
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                pass  # handled above; pygame doesn't have native dblclick

        # ── Button clicks (checked each frame via rect test) ──────────────────
        # (drawn and returned each frame)

        # ── Draw map ──────────────────────────────────────────────────────────
        # Tiles
        ox = int(view._origin_tx * TILE_SIZE)
        oy = int(view._origin_ty * TILE_SIZE)
        tx0 = int(view._origin_tx)
        ty0 = int(view._origin_ty)
        tx1 = int(view._origin_tx + MAP_W / TILE_SIZE) + 1
        ty1 = int(view._origin_ty + H    / TILE_SIZE) + 1
        n   = 2 ** view.zoom

        any_tile = False
        screen.set_clip(pygame.Rect(0, 0, MAP_W, H))
        for tx in range(tx0, tx1 + 1):
            for ty in range(ty0, ty1 + 1):
                if not (0 <= tx < n and 0 <= ty < n):
                    continue
                tile = tiles.get(view.zoom, tx, ty)
                px   = tx * TILE_SIZE - ox
                py   = ty * TILE_SIZE - oy
                if tile:
                    any_tile = True
                    screen.blit(tile, (px, py))
                else:
                    pygame.draw.rect(screen, C_OFFLINE,
                                     pygame.Rect(px, py, TILE_SIZE, TILE_SIZE))
                    pygame.draw.rect(screen, C_GRID_LINE,
                                     pygame.Rect(px, py, TILE_SIZE, TILE_SIZE), 1)

        if not any_tile:
            screen.blit(get_grid(), (0, 0))

        screen.set_clip(None)

        # Dark overlay on map (subtle — keeps colours readable)
        overlay = pygame.Surface((MAP_W, H), pygame.SRCALPHA)
        overlay.fill((0, 0, 10, 60))
        screen.blit(overlay, (0, 0))

        # ── Draw GPS track ────────────────────────────────────────────────────
        if live.connected:
            track = live.get_track()
            if len(track) > 1:
                pts = [view.ll_to_px(lat, lon) for lat, lon in track]
                pts = [(x, y) for x, y in pts if -10 <= x <= MAP_W+10
                       and -10 <= y <= H+10]
                if len(pts) > 1:
                    pygame.draw.lines(screen, C_TRACK, False, pts, 2)

        # ── Draw route line ───────────────────────────────────────────────────
        if len(waypoints) > 1:
            pts = [view.ll_to_px(w.lat, w.lon) for w in waypoints]
            pygame.draw.lines(screen, C_ROUTE, False, pts, 2)
            # Distance labels on segments
            for i in range(len(waypoints) - 1):
                d   = _haversine(waypoints[i].lat, waypoints[i].lon,
                                  waypoints[i+1].lat, waypoints[i+1].lon)
                p1  = view.ll_to_px(waypoints[i].lat, waypoints[i].lon)
                p2  = view.ll_to_px(waypoints[i+1].lat, waypoints[i+1].lon)
                mx_ = (p1[0] + p2[0]) // 2
                my_ = (p1[1] + p2[1]) // 2
                if 0 <= mx_ <= MAP_W and 0 <= my_ <= H:
                    ds   = f"{d:.0f}m" if d < 1000 else f"{d/1000:.2f}km"
                    dlbl = font_tiny.render(ds, True, C_ROUTE)
                    # Small bg
                    br = pygame.Rect(mx_ - dlbl.get_width()//2 - 3,
                                     my_ - 8,
                                     dlbl.get_width() + 6, 14)
                    pygame.draw.rect(screen, (0, 0, 0, 160), br, border_radius=3)
                    screen.blit(dlbl, (mx_ - dlbl.get_width()//2, my_ - 7))

        # ── Draw waypoints ────────────────────────────────────────────────────
        for i, wp in enumerate(waypoints):
            px_, py_ = view.ll_to_px(wp.lat, wp.lon)
            if -20 <= px_ <= MAP_W+20 and -20 <= py_ <= H+20:
                is_sel  = (i == selected)
                is_next = (live_state and
                           live_state.get('nav_wp') == i and
                           live_state.get('nav_state') not in ('IDLE', 'COMPLETE'))
                ring_col = C_WP_SEL if is_sel else (C_GREEN if is_next else C_WP_RING)
                r        = WP_RADIUS + (2 if is_sel else 0)

                # Shadow
                pygame.draw.circle(screen, (0, 0, 0, 120),
                                   (px_ + 2, py_ + 2), r)
                # Fill
                pygame.draw.circle(screen, C_WP_FILL, (px_, py_), r)
                # Ring
                pygame.draw.circle(screen, ring_col,  (px_, py_), r, 2)

                # Number
                nt = font_tiny.render(str(i + 1), True, ring_col)
                screen.blit(nt, (px_ - nt.get_width()//2,
                                  py_ - nt.get_height()//2))

                # Label above
                label = wp.label or f"WP {i+1}"
                lt    = font_tiny.render(label, True, C_WHITE)
                lbg   = pygame.Rect(px_ - lt.get_width()//2 - 3,
                                     py_ - r - 16,
                                     lt.get_width() + 6, 13)
                pygame.draw.rect(screen, (0, 0, 0, 180), lbg, border_radius=2)
                screen.blit(lt, (px_ - lt.get_width()//2, py_ - r - 15))

        # ── Draw live robot position ──────────────────────────────────────────
        if live_state:
            g = live_state.get('gps') or {}
            if g.get('latitude') is not None:
                rpx, rpy = view.ll_to_px(g['latitude'], g['longitude'])
                if 0 <= rpx <= MAP_W and 0 <= rpy <= H:
                    # Accuracy ring
                    if g.get('h_error_m'):
                        # Convert metres to pixels at current zoom
                        metres_per_px = (40075016.686 *
                                         abs(math.cos(math.radians(g['latitude'])))
                                         / (2 ** view.zoom * TILE_SIZE))
                        acc_r = max(4, int(g['h_error_m'] / metres_per_px))
                        pygame.draw.circle(screen, (*C_ACCURACY, 40),
                                           (rpx, rpy), acc_r)
                        pygame.draw.circle(screen, C_ACCURACY,
                                           (rpx, rpy), acc_r, 1)

                    # Heading arrow
                    hdg = g.get('heading')
                    if hdg is not None:
                        rad = math.radians(hdg - 90)
                        ax  = rpx + int(18 * math.cos(rad))
                        ay  = rpy + int(18 * math.sin(rad))
                        pygame.draw.line(screen, C_ROBOT, (rpx, rpy), (ax, ay), 3)
                        # Arrowhead
                        for side in (-25, 25):
                            sr  = math.radians(hdg - 90 + side + 180)
                            bx_ = ax + int(7 * math.cos(sr))
                            by_ = ay + int(7 * math.sin(sr))
                            pygame.draw.line(screen, C_ROBOT, (ax, ay), (bx_, by_), 2)

                    pygame.draw.circle(screen, C_ROBOT, (rpx, rpy), 7)
                    pygame.draw.circle(screen, C_BG,    (rpx, rpy), 4)

        # ── Zoom indicator ────────────────────────────────────────────────────
        zt = font_tiny.render(f"Zoom {view.zoom}", True, C_GRAY)
        screen.blit(zt, (8, H - 18))

        # Attribution
        at = font_tiny.render("© OpenStreetMap contributors", True, C_GRAY)
        screen.blit(at, (MAP_W - at.get_width() - 6, H - 16))

        # ── Sidebar ───────────────────────────────────────────────────────────
        list_top, list_h = sidebar.draw(screen, waypoints, selected, live_state)
        btn_rects        = sidebar.draw_buttons(screen, live.connected)

        # ── Handle button clicks ──────────────────────────────────────────────
        mouse_pos    = pygame.mouse.get_pos()
        mouse_just   = pygame.mouse.get_pressed()

        # Process one-shot button clicks via event queue above.
        # We check MOUSEBUTTONUP events in a second pass for buttons:
        for event in pygame.event.get(pygame.MOUSEBUTTONUP):
            if event.button != 1:
                continue
            mx, my = event.pos
            if btn_rects.get('import_json', pygame.Rect(0,0,0,0)).collidepoint(mx, my):
                path = file_dialog_open(('.json',))
                if path:
                    try:
                        waypoints = load_json(path)
                        view.fit_waypoints(waypoints)
                        toast.show(f"Loaded {len(waypoints)} waypoints")
                    except Exception as e:
                        toast.show(f"Load failed: {e}", C_RED)

            elif btn_rects.get('import_csv', pygame.Rect(0,0,0,0)).collidepoint(mx, my):
                path = file_dialog_open(('.csv',))
                if path:
                    try:
                        wps = load_csv(path)
                        waypoints = wps
                        view.fit_waypoints(waypoints)
                        toast.show(f"Imported {len(waypoints)} points from CSV")
                    except Exception as e:
                        toast.show(f"Import failed: {e}", C_RED)

            elif btn_rects.get('export', pygame.Rect(0,0,0,0)).collidepoint(mx, my):
                path = file_dialog_save()
                if path:
                    try:
                        save_json(waypoints, path)
                        toast.show(f"Saved → {Path(path).name}", C_GREEN)
                    except Exception as e:
                        toast.show(f"Save failed: {e}", C_RED)

            elif btn_rects.get('clear', pygame.Rect(0,0,0,0)).collidepoint(mx, my):
                waypoints = []
                selected  = -1
                toast.show("Cleared", C_ORANGE)

            elif btn_rects.get('connect', pygame.Rect(0,0,0,0)).collidepoint(mx, my):
                if live.connected:
                    live.disconnect()
                    toast.show("Disconnected", C_ORANGE)
                else:
                    ok = live.connect()
                    toast.show("Connected to robot.py" if ok
                               else "Could not connect — is robot.py importable?",
                               C_GREEN if ok else C_RED)

            elif btn_rects.get('send_route', pygame.Rect(0,0,0,0)).collidepoint(mx, my):
                if not live.connected or live._robot is None:
                    toast.show("Not connected to robot", C_RED)
                elif not waypoints:
                    toast.show("No waypoints to send", C_ORANGE)
                else:
                    try:
                        nav = live._robot.get_gps_navigator()
                        if nav is None:
                            toast.show("GPS navigator not available", C_RED)
                        else:
                            nav.load_waypoints(waypoints)
                            toast.show(f"Sent {len(waypoints)} waypoints to navigator",
                                       C_GREEN)
                    except Exception as e:
                        toast.show(f"Send failed: {e}", C_RED)

            # Sidebar row double-click → label edit
            row = sidebar.row_at(mx, my, list_top, list_h, len(waypoints))
            if row is not None and row == selected:
                sidebar.start_edit(row, waypoints[row].label)

        # ── Toast ─────────────────────────────────────────────────────────────
        toast.draw(screen, font_sm)

        pygame.display.flip()

    pygame.quit()
    live.disconnect()


if __name__ == "__main__":
    main()
