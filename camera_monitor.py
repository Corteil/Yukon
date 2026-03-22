#!/usr/bin/env python3
"""
camera_monitor.py — Live camera monitor with focus assist and display options.

Controls
--------
  M          : cycle colour mode
  S          : cycle capture resolution
  R          : rotate 90° clockwise
  H          : toggle histogram overlay
  ↑ / ↓     : exposure +/- (switches to manual; A to restore auto)
  [ / ]      : analogue gain down / up
  A          : auto exposure / gain
  Q / Esc    : quit

Colour modes
------------
  Colour · Greyscale · False Colour · Edges · Focus Peak

Focus Peak overlays red on the sharpest edges — move the lens until the
red highlights are strongest on your target subject.
The Sharpness score (Laplacian variance) in the status bar rises as focus
improves.

Usage
-----
  python3 camera_monitor.py

  T          : toggle ArUco tag detection
  D          : cycle ArUco dictionary
"""

import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import pygame
from picamera2 import Picamera2

from robot.aruco_detector import ArucoDetector, ArUcoState, ARUCO_DICT

# ── Colour palette ────────────────────────────────────────────────────────────

C_BG     = ( 18,  18,  30)
C_PANEL  = ( 28,  28,  45)
C_BORDER = ( 60,  60,  90)
C_WHITE  = (230, 230, 240)
C_GRAY   = (130, 130, 150)
C_GREEN  = ( 60, 220,  80)
C_YELLOW = (240, 200,  40)
C_RED    = (220,  60,  60)
C_CYAN   = ( 60, 200, 220)

# ── Options ───────────────────────────────────────────────────────────────────

COLOUR_MODES = ["Colour", "Greyscale", "False Colour", "Edges", "Focus Peak"]

CAPTURE_SIZES = [
    (640,  480),
    (1280, 720),
    (1456, 1088),   # full native IMX296
]

GAINS = [1.0, 2.0, 4.0, 8.0, 16.0]

ARUCO_DICTS = [
    "DICT_4X4_50",
    "DICT_4X4_100",
    "DICT_4X4_1000",
    "DICT_5X5_100",
    "DICT_6X6_100",
]

IMAGE_DIR  = Path(__file__).parent / "saved_images"
CALIB_FILE = Path(__file__).parent / "camera_cal.npz"

# Exposure steps in µs (range 1–66 666)
EXP_STEPS = [500, 1000, 2000, 4000, 8000, 16000, 33000, 66000]
EXP_DEFAULT = 4     # index into EXP_STEPS → 8000 µs

# ── Layout ────────────────────────────────────────────────────────────────────

W, H     = 1024, 768
BAR_H    = 72
PANEL_W  = 214          # right-hand info panel
IMG_Y    = BAR_H
IMG_H    = H - BAR_H * 2
IMG_W    = W - PANEL_W  # image area width (810 px)


# ── Frame processing ──────────────────────────────────────────────────────────

def _apply_mode(frame_rgb: np.ndarray, mode: str) -> np.ndarray:
    gray = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2GRAY)
    if mode == "Colour":
        return frame_rgb
    if mode == "Greyscale":
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB)
    if mode == "False Colour":
        mapped = cv2.applyColorMap(gray, cv2.COLORMAP_JET)
        return cv2.cvtColor(mapped, cv2.COLOR_BGR2RGB)
    if mode == "Edges":
        edges = cv2.Canny(gray, 50, 150)
        return cv2.cvtColor(edges, cv2.COLOR_GRAY2RGB)
    if mode == "Focus Peak":
        base  = cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB)
        edges = cv2.Canny(gray, 80, 200)
        base[edges > 0] = (220, 40, 40)
        return base
    return frame_rgb


def _sharpness(gray: np.ndarray) -> float:
    """Laplacian variance — higher = sharper.
    Downsampled to 640×480 before scoring so the value is
    resolution-independent and thresholds stay consistent."""
    small = cv2.resize(gray, (640, 480), interpolation=cv2.INTER_AREA)
    return float(cv2.Laplacian(small, cv2.CV_64F).var())


def _rotate(frame: np.ndarray, degrees: int) -> np.ndarray:
    if degrees == 0:
        return frame
    return np.rot90(frame, k=-(degrees // 90))


def _letterbox(frame: np.ndarray, max_w: int, max_h: int) -> np.ndarray:
    fh, fw = frame.shape[:2]
    scale  = min(max_w / fw, max_h / fh)
    nw, nh = int(fw * scale), int(fh * scale)
    return cv2.resize(frame, (nw, nh), interpolation=cv2.INTER_LINEAR)


# ── Histogram ─────────────────────────────────────────────────────────────────

def _draw_histogram(surf, frame_rgb: np.ndarray, rect: pygame.Rect):
    pygame.draw.rect(surf, (8, 8, 18), rect)
    pygame.draw.rect(surf, C_BORDER, rect, 1)
    bar_w = max(1, rect.width // 64)
    for ch, color in enumerate([(200, 60, 60), (60, 200, 60), (60, 100, 220)]):
        hist = cv2.calcHist([frame_rgb], [ch], None, [64], [0, 256]).flatten()
        mx   = hist.max() or 1
        for i, v in enumerate(hist):
            bh = int((v / mx) * (rect.height - 2))
            if bh:
                pygame.draw.rect(surf, color,
                                 (rect.x + i * bar_w,
                                  rect.bottom - bh - 1,
                                  bar_w, bh))


# ── ArUco overlay ─────────────────────────────────────────────────────────────

_ACO_GREEN = (  0, 220,  80)
_ACO_RED   = (220,  40,  40)
_ACO_BLUE  = ( 40, 120, 220)
_ACO_WHITE = (230, 230, 240)


def _draw_aruco_on_frame(frame: np.ndarray, state: ArUcoState):
    """Draw tag boxes and gate lines onto an RGB numpy frame."""
    font      = cv2.FONT_HERSHEY_SIMPLEX
    for tag in state.tags.values():
        tl, tr = tag.top_left,     tag.top_right
        br, bl = tag.bottom_right, tag.bottom_left
        cv2.line(frame, tl, tr, _ACO_GREEN, 2)
        cv2.line(frame, tr, br, _ACO_GREEN, 2)
        cv2.line(frame, br, bl, _ACO_GREEN, 2)
        cv2.line(frame, bl, tl, _ACO_GREEN, 2)
        cv2.circle(frame, (tag.center_x, tag.center_y), 4, _ACO_RED, -1)
        cv2.putText(frame, str(tag.id),
                    (tl[0], tl[1] - 10), font, 0.55, _ACO_WHITE, 2)

    for gate in state.gates.values():
        colour = _ACO_BLUE if gate.correct_dir else _ACO_RED
        cv2.line(frame,
                 (gate.centre_x, gate.centre_y - 50),
                 (gate.centre_x, gate.centre_y),
                 colour, 4)
        cv2.putText(frame, f"G{gate.gate_id}",
                    (gate.centre_x + 6, gate.centre_y - 52),
                    font, 0.55, colour, 2)


_SCROLL_STEP = 18   # pixels per arrow-key press when frozen


def _draw_right_panel(surf, rect: pygame.Rect,
                      show_aruco: bool, state: ArUcoState, dict_name: str,
                      frozen: bool, scroll: int, font_sm, font_tiny):
    """Draw the fixed right-hand info panel.

    *scroll* is a pixel offset applied only to the scrollable body
    (tag list + gates).  The header and frozen badge stay fixed.
    """
    pygame.draw.rect(surf, C_PANEL,  rect, border_radius=6)
    pygame.draw.rect(surf, C_BORDER, rect, 1, border_radius=6)

    old_clip = surf.get_clip()
    surf.set_clip(rect.inflate(-2, -2))

    px    = rect.x + 8
    row_h = 18

    # ── Fixed header ──────────────────────────────────────────────────────
    py = rect.y + 8
    hdr_color = C_CYAN if show_aruco else C_GRAY
    surf.blit(font_sm.render("ArUco", True, hdr_color), (px, py))
    py += row_h + 2
    pygame.draw.line(surf, C_BORDER,
                     (rect.x + 4, py), (rect.right - 4, py), 1)
    py += 6

    if not show_aruco:
        surf.blit(font_tiny.render("OFF  (T to enable)", True, C_GRAY), (px, py))
        surf.set_clip(old_clip)
        return

    d_short = dict_name.replace("DICT_", "")
    surf.blit(font_tiny.render(f"Dict: {d_short}", True, C_GRAY), (px, py))
    py += row_h
    n = len(state.tags)
    surf.blit(font_tiny.render(
        f"Tags: {n}", True, C_GREEN if n else C_GRAY), (px, py))
    py += row_h + 4

    # ── Scrollable body — tighter clip from here to bottom of panel ───────
    frozen_badge_h = row_h + 12 if frozen else 0
    body_top    = py
    body_bottom = rect.bottom - frozen_badge_h - 4
    body_rect   = pygame.Rect(rect.x + 2, body_top, rect.width - 4,
                               max(0, body_bottom - body_top))
    surf.set_clip(body_rect)

    py = body_top - scroll   # apply scroll offset

    if state.tags:
        for tag in sorted(state.tags.values(), key=lambda t: t.id):
            surf.blit(font_tiny.render(
                f"  ID {tag.id}", True, C_WHITE), (px, py))
            surf.blit(font_tiny.render(
                f"  {tag.center_x},{tag.center_y}", True, C_GRAY),
                (px, py + row_h - 4))
            surf.blit(font_tiny.render(
                f"  {tag.area} px²", True, C_GRAY),
                (px, py + (row_h - 4) * 2))
            py += row_h * 3 - 4
    else:
        surf.blit(font_tiny.render("  (none)", True, C_GRAY), (px, py))
        py += row_h

    if state.gates:
        py += 4
        surf.blit(font_sm.render("Gates", True, C_CYAN), (px, py))
        py += row_h + 2
        pygame.draw.line(surf, C_BORDER,
                         (rect.x + 4, py), (rect.right - 4, py), 1)
        py += 6
        for gate in sorted(state.gates.values(), key=lambda g: g.gate_id):
            dir_txt   = "correct"  if gate.correct_dir else "reversed"
            dir_color = C_GREEN    if gate.correct_dir else C_RED
            surf.blit(font_tiny.render(
                f"  G{gate.gate_id}  {gate.odd_tag}+{gate.even_tag}",
                True, C_WHITE), (px, py))
            py += row_h - 4
            surf.blit(font_tiny.render(
                f"  {dir_txt}", True, dir_color), (px, py))
            py += row_h

    # Scroll indicators
    if scroll > 0:
        surf.blit(font_tiny.render("▲ more", True, C_GRAY),
                  (px, body_top + 2))
    content_end = py + scroll   # py is already offset; undo to get true end
    if content_end > body_bottom:
        surf.blit(font_tiny.render("▼ more", True, C_GRAY),
                  (px, body_bottom - row_h))

    # ── Freeze indicator (fixed, below scrollable area) ───────────────────
    surf.set_clip(rect.inflate(-2, -2))
    if frozen:
        hint = font_tiny.render("↑↓ scroll", True, C_GRAY)
        lbl  = font_sm.render("FROZEN", True, C_CYAN)
        surf.blit(hint, (rect.centerx - hint.get_width() // 2,
                         rect.bottom - lbl.get_height() - hint.get_height() - 8))
        surf.blit(lbl,  (rect.centerx - lbl.get_width() // 2,
                         rect.bottom - lbl.get_height() - 6))

    surf.set_clip(old_clip)


# ── Camera helpers ────────────────────────────────────────────────────────────

def _make_cam(width: int, height: int) -> Picamera2:
    cam = Picamera2()
    cam.configure(cam.create_video_configuration(
        main={"size": (width, height), "format": "RGB888"},
        controls={"FrameRate": 30},
    ))
    cam.start()
    return cam


# ── Main loop ─────────────────────────────────────────────────────────────────

def run():
    pygame.init()
    screen = pygame.display.set_mode((W, H))
    pygame.display.set_caption("Camera Monitor")
    clock  = pygame.time.Clock()

    FONT_BIG  = pygame.font.SysFont("monospace", 22, bold=True)
    FONT_MD   = pygame.font.SysFont("monospace", 16)
    FONT_SM   = pygame.font.SysFont("monospace", 14)
    FONT_TINY = pygame.font.SysFont("monospace", 11)

    # ── State ─────────────────────────────────────────────────────────────
    mode_idx   = 0
    size_idx   = 2          # start at native 1456×1088 (IMX296 global shutter)
    rotation   = 0
    gain_idx   = 0
    exp_idx    = EXP_DEFAULT
    auto_exp   = True        # AeEnable
    show_hist  = False
    show_aruco = False
    dict_idx   = 0
    frozen        = False
    tag_scroll    = 0
    capture_flash = 0.0   # timestamp of last save (for on-screen flash)

    # ── Calibration (optional undistortion) ───────────────────────────────
    _cal_map1 = _cal_map2 = _cal_size = None
    use_calib = False
    if CALIB_FILE.exists():
        try:
            _cal = np.load(CALIB_FILE)
            _cam_mtx  = _cal['camera_matrix']
            _dist     = _cal['dist_coeffs']
            _cal_size = tuple(int(v) for v in _cal['frame_size'])
        except Exception as e:
            print(f"Warning: could not load calibration: {e}")

    def _get_cal_maps(w, h):
        """Return (map1, map2) for the given frame size, building lazily."""
        nonlocal _cal_map1, _cal_map2, _cal_size
        if _cam_mtx is None:
            return None, None
        if (_cal_map1 is None or (_cal_map1.shape[1], _cal_map1.shape[0]) != (w, h)):
            cal_w, cal_h = _cal_size
            sx, sy = w / cal_w, h / cal_h
            mtx = _cam_mtx.copy()
            mtx[0, 0] *= sx; mtx[1, 1] *= sy
            mtx[0, 2] *= sx; mtx[1, 2] *= sy
            new_mtx, _ = cv2.getOptimalNewCameraMatrix(mtx, _dist, (w, h), 1, (w, h))
            _cal_map1, _cal_map2 = cv2.initUndistortRectifyMap(
                mtx, _dist, None, new_mtx, (w, h), cv2.CV_16SC2)
        return _cal_map1, _cal_map2

    detector   = ArucoDetector(ARUCO_DICTS[dict_idx], draw=False)
    aruco_state: ArUcoState = ArUcoState()

    cap_w, cap_h = CAPTURE_SIZES[size_idx]
    cam   = _make_cam(cap_w, cap_h)

    raw_frame  = None
    disp_frame = None
    sharpness  = 0.0
    fps_count  = 0
    fps_t0     = time.time()
    display_fps = 0.0

    def restart_cam():
        nonlocal cam, cap_w, cap_h
        cam.stop()
        cam.close()
        cap_w, cap_h = CAPTURE_SIZES[size_idx]
        cam = _make_cam(cap_w, cap_h)
        _apply_controls()

    def _apply_controls():
        if auto_exp:
            cam.set_controls({"AeEnable": True,
                               "AnalogueGain": GAINS[gain_idx]})
        else:
            cam.set_controls({"AeEnable":     False,
                               "ExposureTime": EXP_STEPS[exp_idx],
                               "AnalogueGain": GAINS[gain_idx]})

    running = True
    while running:
        # ── Events ────────────────────────────────────────────────────────
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                running = False
            elif ev.type == pygame.KEYDOWN:
                if ev.key in (pygame.K_q, pygame.K_ESCAPE):
                    running = False
                elif ev.key == pygame.K_m:
                    mode_idx = (mode_idx + 1) % len(COLOUR_MODES)
                elif ev.key == pygame.K_s:
                    size_idx = (size_idx + 1) % len(CAPTURE_SIZES)
                    restart_cam()
                elif ev.key == pygame.K_r:
                    rotation = (rotation + 90) % 360
                elif ev.key == pygame.K_h:
                    show_hist = not show_hist
                elif ev.key == pygame.K_a:
                    auto_exp = True
                    _apply_controls()
                elif ev.key == pygame.K_UP:
                    if frozen:
                        tag_scroll = max(0, tag_scroll - _SCROLL_STEP)
                    else:
                        auto_exp = False
                        exp_idx  = min(exp_idx + 1, len(EXP_STEPS) - 1)
                        _apply_controls()
                elif ev.key == pygame.K_DOWN:
                    if frozen:
                        tag_scroll += _SCROLL_STEP
                    else:
                        auto_exp = False
                        exp_idx  = max(exp_idx - 1, 0)
                        _apply_controls()
                elif ev.key == pygame.K_RIGHTBRACKET:
                    gain_idx = min(gain_idx + 1, len(GAINS) - 1)
                    _apply_controls()
                elif ev.key == pygame.K_LEFTBRACKET:
                    gain_idx = max(gain_idx - 1, 0)
                    _apply_controls()
                elif ev.key == pygame.K_t:
                    show_aruco = not show_aruco
                elif ev.key == pygame.K_d:
                    dict_idx = (dict_idx + 1) % len(ARUCO_DICTS)
                    detector  = ArucoDetector(ARUCO_DICTS[dict_idx], draw=False)
                elif ev.key == pygame.K_f:
                    frozen = not frozen
                    if not frozen:
                        tag_scroll = 0
                elif ev.key == pygame.K_k:
                    if _cam_mtx is not None:
                        use_calib = not use_calib
                    else:
                        print("No calibration file found — run calibrate_camera.py first")
                elif ev.key == pygame.K_SPACE:
                    frame_to_save = disp_frame if disp_frame is not None else raw_frame
                    if frame_to_save is not None:
                        IMAGE_DIR.mkdir(exist_ok=True)
                        ts   = datetime.now().strftime("%Y%m%d_%H%M%S")
                        path = IMAGE_DIR / f"capture_{ts}.png"
                        cv2.imwrite(str(path),
                                    cv2.cvtColor(frame_to_save, cv2.COLOR_RGB2BGR))
                        capture_flash = time.time()
                        print(f"Saved: {path}")

        # ── Capture ───────────────────────────────────────────────────────
        if frozen:
            raw = None
        else:
            try:
                raw = cam.capture_array()[:, :, ::-1]  # BGR → RGB
            except Exception:
                raw = None

        if raw is not None:
            rotated = _rotate(raw, rotation)
            if use_calib:
                h, w = rotated.shape[:2]
                m1, m2 = _get_cal_maps(w, h)
                if m1 is not None:
                    rotated = cv2.remap(rotated, m1, m2, cv2.INTER_LINEAR)
            gray      = cv2.cvtColor(rotated, cv2.COLOR_RGB2GRAY)
            sharpness = _sharpness(gray)
            raw_frame = rotated

            if show_aruco:
                aruco_state = detector.detect(rotated.copy())
                annotated   = rotated.copy()
                _draw_aruco_on_frame(annotated, aruco_state)
                disp_frame  = _apply_mode(annotated, COLOUR_MODES[mode_idx])
            else:
                aruco_state = ArUcoState()
                disp_frame  = _apply_mode(rotated, COLOUR_MODES[mode_idx])
            fps_count += 1
            now = time.time()
            if now - fps_t0 >= 1.0:
                display_fps = fps_count / (now - fps_t0)
                fps_count   = 0
                fps_t0      = now

        # ── Render ────────────────────────────────────────────────────────
        screen.fill(C_BG)

        if disp_frame is not None:
            scaled = _letterbox(disp_frame, IMG_W, IMG_H)
            sh, sw = scaled.shape[:2]
            ox = (IMG_W - sw) // 2
            oy = IMG_Y + (IMG_H - sh) // 2
            surf = pygame.surfarray.make_surface(scaled.swapaxes(0, 1))
            screen.blit(surf, (ox, oy))

            # Centre crosshair
            cx, cy = ox + sw // 2, oy + sh // 2
            arm = 20
            pygame.draw.line(screen, C_GREEN, (cx - arm, cy), (cx + arm, cy), 1)
            pygame.draw.line(screen, C_GREEN, (cx, cy - arm), (cx, cy + arm), 1)
            pygame.draw.rect(screen, C_GREEN,
                             pygame.Rect(cx - 10, cy - 10, 20, 20), 1)

            if show_hist and raw_frame is not None:
                hist_rect = pygame.Rect(ox + sw - 134, oy + sh - 74, 132, 72)
                _draw_histogram(screen, raw_frame, hist_rect)

            if frozen:
                lbl = FONT_MD.render("FROZEN", True, C_CYAN)
                screen.blit(lbl, (ox + sw // 2 - lbl.get_width() // 2,
                                  oy + sh - lbl.get_height() - 8))

            if time.time() - capture_flash < 2.0:
                ts_str = datetime.fromtimestamp(capture_flash).strftime("%H:%M:%S")
                msg = FONT_MD.render(f"Saved  {ts_str}", True, C_GREEN)
                screen.blit(msg, (ox + 8, oy + 8))

        # ── Right panel (always present) ─────────────────────────────────
        panel_rect = pygame.Rect(IMG_W + 4, IMG_Y + 2, PANEL_W - 8, IMG_H - 4)
        _draw_right_panel(screen, panel_rect,
                          show_aruco, aruco_state, ARUCO_DICTS[dict_idx],
                          frozen, tag_scroll, FONT_SM, FONT_TINY)

        # ── Top bar ───────────────────────────────────────────────────────
        top_rect = pygame.Rect(10, 6, W - 20, BAR_H - 10)
        pygame.draw.rect(screen, C_PANEL,  top_rect, border_radius=6)
        pygame.draw.rect(screen, C_BORDER, top_rect, width=1, border_radius=6)

        screen.blit(FONT_BIG.render("Camera Monitor", True, C_WHITE), (20, 13))

        h1 = FONT_SM.render(
            "M=mode   S=size   R=rotate   H=hist   K=calib   Spc=save   F=freeze   Q=quit",
            True, C_GRAY)
        h2 = FONT_SM.render(
            "T=ArUco   D=dict   ↑↓=exposure   [ ]=gain   A=auto-exp",
            True, C_GRAY)
        screen.blit(h1, (W // 2 - h1.get_width() // 2, 28))
        screen.blit(h2, (W // 2 - h2.get_width() // 2, 46))

        # ── Bottom stats bar ──────────────────────────────────────────────
        bot_y = H - BAR_H + 6
        pygame.draw.line(screen, C_BORDER, (10, bot_y - 4), (W - 10, bot_y - 4), 1)

        sharp_color = (C_GREEN  if sharpness > 300
                       else C_YELLOW if sharpness > 80
                       else C_RED)

        exp_str = ("auto" if auto_exp
                   else f"{EXP_STEPS[exp_idx] / 1000:.1f} ms")

        aruco_str = (f"{len(aruco_state.tags)} tags"
                     + (f"  {len(aruco_state.gates)} gates" if aruco_state.gates else "")
                     if show_aruco else "OFF")

        calib_str = ("ON" if use_calib else "OFF") if _cam_mtx is not None else "none"
        stats = [
            ("Mode",      COLOUR_MODES[mode_idx]),
            ("Size",      f"{cap_w}×{cap_h}"),
            ("Rotation",  f"{rotation}°"),
            ("Exposure",  exp_str),
            ("Gain",      f"{GAINS[gain_idx]:.0f}×"),
            ("FPS",       f"{display_fps:.1f}"),
            ("Sharpness", f"{sharpness:.0f}"),
            ("Calib",     calib_str),
            ("ArUco",     aruco_str),
        ]

        sx = 20
        for label, value in stats:
            lbl_s = FONT_TINY.render(label, True, C_GRAY)
            color = (sharp_color if label == "Sharpness"
                     else C_GREEN  if label == "Calib" and value == "ON"
                     else C_GRAY   if label == "Calib" and value == "none"
                     else C_WHITE)
            val_s = FONT_MD.render(value, True, color)
            screen.blit(lbl_s, (sx, bot_y))
            screen.blit(val_s, (sx, bot_y + lbl_s.get_height() + 2))
            sx += max(lbl_s.get_width(), val_s.get_width()) + 22

        # Sharpness bar (right side of bottom bar)
        bar_rect = pygame.Rect(W - 164, bot_y + 2, 150, 16)
        pygame.draw.rect(screen, C_BG,    bar_rect, border_radius=3)
        pygame.draw.rect(screen, C_BORDER, bar_rect, 1, border_radius=3)
        fill = int(min(sharpness / 600, 1.0) * 148)
        if fill:
            pygame.draw.rect(screen, sharp_color,
                             pygame.Rect(bar_rect.x + 1, bar_rect.y + 1,
                                         fill, 14), border_radius=3)

        pygame.display.flip()
        clock.tick(30)

    cam.stop()
    cam.close()
    pygame.quit()


if __name__ == "__main__":
    run()
