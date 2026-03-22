#!/usr/bin/env python3
"""
calibrate_camera.py — Camera lens calibration tool (pygame GUI).

Captures frames with a checkerboard pattern, computes the camera matrix and
distortion coefficients, and saves them to camera_cal.npz.

Controls:
  SPACE  — capture current frame as a calibration sample
  C      — compute calibration from captured samples (need ≥ 6)
  U      — toggle undistort preview (only after calibration)
  R      — rotate frame 90° clockwise
  Q/ESC  — quit (saves calibration if computed)

Checkerboard:
  Print docs/checkerboard_9x6.pdf at 100% scale (no "fit to page").
  Default board: 9×6 inner corners, 25 mm squares.
  Capture ≥ 15 views at different positions, angles and distances,
  making sure to cover all four corners of the image.

Output:
  camera_cal.npz in the project root, containing camera_matrix,
  dist_coeffs, precomputed undistortion maps (map1/map2), rms, frame_size.
"""

import argparse
import configparser
import sys
import os
import time
import ctypes

import cv2
import numpy as np
import pygame

# ── Suppress GLib-GObject-CRITICAL from picamera2/GStreamer teardown ──────────
try:
    _glib    = ctypes.CDLL('libglib-2.0.so.0')
    _LogFunc = ctypes.CFUNCTYPE(
        None, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_void_p)
    _null_log = _LogFunc(lambda *_: None)
    _glib.g_log_set_handler(b'GLib-GObject', 1 << 3, _null_log, None)
except Exception:
    pass

try:
    from picamera2 import Picamera2
    _USE_PICAMERA2 = True
except ImportError:
    _USE_PICAMERA2 = False

# ── Board geometry (must match make_checkerboard_pdf.py) ──────────────────────
COLS      = 9    # inner corners horizontally
ROWS      = 6    # inner corners vertically
SQUARE_MM = 25   # physical square size in mm

ROTATION  = 180  # default frame rotation: 0, 90, 180, 270

# ── Output file ───────────────────────────────────────────────────────────────
_HERE    = os.path.dirname(os.path.abspath(__file__))
_CAL_DIR = os.path.join(_HERE, '..')

def _cal_path(frame_size):
    """Return the calibration output path for a given (width, height)."""
    w, h = frame_size
    return os.path.normpath(os.path.join(_CAL_DIR, f'camera_cal_{w}x{h}.npz'))

# ── Calibration internals ─────────────────────────────────────────────────────
_CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

_OBJP = np.zeros((ROWS * COLS, 3), np.float32)
_OBJP[:, :2] = np.mgrid[0:COLS, 0:ROWS].T.reshape(-1, 2) * SQUARE_MM

# ── Colours (RGB) ─────────────────────────────────────────────────────────────
_WHITE  = (255, 255, 255)
_BLACK  = (  0,   0,   0)
_GREEN  = (  0, 210,   0)
_RED    = (220,   0,   0)
_YELLOW = (255, 210,   0)
_CYAN   = (  0, 200, 240)


# ── Camera wrapper ────────────────────────────────────────────────────────────

class _Cam:
    def __init__(self, width: int, height: int):
        self._pc2 = None
        self._cap = None
        if _USE_PICAMERA2:
            print(f"Using picamera2 backend ({width}×{height})")
            pc2 = Picamera2()
            pc2.configure(pc2.create_video_configuration(
                main={"size": (width, height), "format": "RGB888"}
            ))
            pc2.start()
            time.sleep(0.5)   # let AGC settle
            self._pc2 = pc2
        else:
            idx = int(os.environ.get('CAM_INDEX', 0))
            print(f"Using OpenCV backend (index {idx}, {width}×{height})")
            cap = cv2.VideoCapture(idx)
            if not cap.isOpened():
                sys.exit(f"ERROR: cannot open camera {idx}")
            cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self._cap = cap

    def read(self, rotation=0):
        """Return a BGR numpy array (rotated), or None on failure."""
        if self._pc2 is not None:
            frame = self._pc2.capture_array()  # BGR despite RGB888 label
        else:
            ret, frame = self._cap.read()
            if not ret:
                return None
        if rotation == 90:
            return cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        if rotation == 180:
            return cv2.rotate(frame, cv2.ROTATE_180)
        if rotation == 270:
            return cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        return frame

    def release(self):
        if self._pc2 is not None:
            self._pc2.stop()
        if self._cap is not None:
            self._cap.release()


# ── Helpers ───────────────────────────────────────────────────────────────────

def _find_corners(grey):
    ok, corners = cv2.findChessboardCorners(grey, (COLS, ROWS), None)
    if not ok:
        return None
    return cv2.cornerSubPix(grey, corners, (11, 11), (-1, -1), _CRITERIA)


def _to_surface(bgr):
    """Convert BGR numpy array → pygame Surface."""
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    h, w = rgb.shape[:2]
    return pygame.image.frombuffer(rgb.tobytes(), (w, h), 'RGB')


def _label(font, text, colour):
    return font.render(text, True, colour)


def _make_zones(fw, fh):
    """Return a list of (pygame.Rect, label) covering the 3×3 grid.

    Order prioritises calibration quality: corners first, then edges, then
    centre.  Covering all 9 zones gives good spatial coverage.
    """
    cw, ch = fw // 3, fh // 3
    # (col, row) in 0-based 3×3 grid
    order = [
        (0, 0, "TL"),  (2, 0, "TR"),  (0, 2, "BL"),  (2, 2, "BR"),
        (1, 0, "Top"), (1, 2, "Bot"), (0, 1, "Left"), (2, 1, "Right"),
        (1, 1, "Ctr"),
    ]
    zones = []
    for col, row, lbl in order:
        x = col * cw
        y = row * ch
        w = fw - x if col == 2 else cw
        h = fh - y if row == 2 else ch
        zones.append((pygame.Rect(x, y, w, h), lbl))
    return zones


def _draw_zones(screen, zones, img_points, font, fw=0, mirrored=False):
    """Overlay coverage zones and captured position dots.

    All zones shown in yellow; green once covered.  Next suggested zone
    gets a filled highlight and a label.  Coordinates are mirrored when
    *mirrored* is True so overlays match the flipped display image.
    """
    def mx(x):
        return fw - x if mirrored and fw else x

    def mirror_rect(rect):
        return pygame.Rect(mx(rect.x + rect.w), rect.y, rect.w, rect.h) if mirrored and fw else rect

    # Determine which zones are covered (always in original coords)
    covered = set()
    for pts in img_points:
        cx = int(np.mean(pts[:, 0, 0]))
        cy = int(np.mean(pts[:, 0, 1]))
        for i, (rect, _) in enumerate(zones):
            if rect.collidepoint(cx, cy):
                covered.add(i)

    next_zone = next((i for i in range(len(zones)) if i not in covered), None)

    for i, (rect, lbl) in enumerate(zones):
        drect = mirror_rect(rect)
        if i in covered:
            s = pygame.Surface((drect.w, drect.h), pygame.SRCALPHA)
            s.fill((0, 200, 0, 35))
            screen.blit(s, drect.topleft)
            pygame.draw.rect(screen, (0, 180, 0), drect, 2)
        elif i == next_zone:
            s = pygame.Surface((drect.w, drect.h), pygame.SRCALPHA)
            s.fill((255, 200, 0, 35))
            screen.blit(s, drect.topleft)
            pygame.draw.rect(screen, (255, 200, 0), drect, 2)
            hint = font.render(f"hold here  ({lbl})", True, (255, 200, 0))
            screen.blit(hint, (drect.centerx - hint.get_width() // 2,
                               drect.centery - hint.get_height() // 2))
        else:
            pygame.draw.rect(screen, (180, 150, 0), drect, 1)

    # Cyan dots — mirrored to match display
    for pts in img_points:
        cx = int(np.mean(pts[:, 0, 0]))
        cy = int(np.mean(pts[:, 0, 1]))
        pygame.draw.circle(screen, _CYAN, (mx(cx), cy), 7)
        pygame.draw.circle(screen, _BLACK, (mx(cx), cy), 7, 1)


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    _HERE_MAIN = os.path.dirname(os.path.abspath(__file__))
    default_ini = os.path.join(_HERE_MAIN, '..', 'robot.ini')

    parser = argparse.ArgumentParser(description="Camera lens calibration tool")
    parser.add_argument('--width',  type=int, default=None,
                        help='Capture width  (overrides robot.ini)')
    parser.add_argument('--height', type=int, default=None,
                        help='Capture height (overrides robot.ini)')
    parser.add_argument('--config', default=default_ini,
                        help='Path to robot.ini (default: ../robot.ini)')
    args = parser.parse_args()

    # Resolution: CLI > robot.ini [camera] > built-in default 640×480
    cfg = configparser.ConfigParser()
    cfg.read(args.config)
    ini_w = int(cfg.get('camera', 'width',  fallback=640))
    ini_h = int(cfg.get('camera', 'height', fallback=480))
    width  = args.width  if args.width  is not None else ini_w
    height = args.height if args.height is not None else ini_h

    pygame.init()
    font    = pygame.font.SysFont('monospace', 16)
    font_lg = pygame.font.SysFont('monospace', 20, bold=True)

    cam = _Cam(width, height)

    obj_points    = []
    img_points    = []
    frame_size    = None
    camera_matrix = None   # raw calibration output (kept for display only)
    dist_coeffs   = None
    opt_mtx       = None   # post-remap intrinsics — saved to file, used for solvePnP
    map1 = map2   = None
    rms           = 0.0
    calibrated    = False
    undistort_on  = False
    mirrored      = False
    rotation      = ROTATION
    capture_now   = False
    flash_until   = 0.0

    screen       = None
    zones        = []
    auto_capture    = True    # toggled with A
    stable_since    = None    # monotonic time when board first held in zone
    STABLE_HOLD     = 1.2     # seconds to hold still before auto-capture
    AUTO_COOLDOWN   = 2.0     # seconds before capturing again in same zone
    last_zone       = None    # which zone the board was in last frame
    last_capture_t  = 0.0     # monotonic time of last auto-capture

    print("\nCamera Calibration")
    print(f"  Resolution: {width}×{height}")
    print(f"  Board   : {COLS}×{ROWS} inner corners, {SQUARE_MM} mm squares")
    print(f"  Output  : {os.path.normpath(CAL_FILE)}")
    print(f"  Controls: SPACE capture · A auto-capture · C calibrate · U undistort · R rotate · Q quit\n")

    clock = pygame.time.Clock()

    while True:
        # ── Events ────────────────────────────────────────────────────────────
        capture_now = False
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                cam.release()
                _save(calibrated, opt_mtx, map1, map2, rms, frame_size)
                return

            if event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_q, pygame.K_ESCAPE):
                    pygame.quit()
                    cam.release()
                    _save(calibrated, opt_mtx, map1, map2, rms, frame_size)
                    return

                elif event.key == pygame.K_SPACE:
                    capture_now = True

                elif event.key == pygame.K_a:
                    auto_capture = not auto_capture
                    stable_since = None
                    print(f"  Auto-capture: {'ON' if auto_capture else 'OFF'}")

                elif event.key == pygame.K_r:
                    rotation   = (rotation + 90) % 360
                    frame_size = None  # dimensions may change
                    print(f"  Rotation: {rotation}°")

                elif event.key == pygame.K_m:
                    mirrored = not mirrored
                    print(f"  Mirror: {'ON' if mirrored else 'OFF'}")

                elif event.key == pygame.K_c:
                    if len(obj_points) < 6:
                        print(f"  Need ≥6 samples (have {len(obj_points)})")
                    else:
                        print(f"  Calibrating from {len(obj_points)} samples…",
                              end='', flush=True)
                        rms, camera_matrix, dist_coeffs, _, _ = \
                            cv2.calibrateCamera(
                                obj_points, img_points, frame_size, None, None)
                        new_mtx, _ = cv2.getOptimalNewCameraMatrix(
                            camera_matrix, dist_coeffs, frame_size, 1, frame_size)
                        map1, map2 = cv2.initUndistortRectifyMap(
                            camera_matrix, dist_coeffs, None, new_mtx,
                            frame_size, cv2.CV_16SC2)
                        opt_mtx      = new_mtx   # save post-remap intrinsics
                        calibrated   = True
                        undistort_on = True
                        q = ("EXCELLENT" if rms < 0.5 else
                             "ACCEPTABLE" if rms < 1.0 else "POOR")
                        print(f" RMS={rms:.4f} px  [{q}]")

                elif event.key == pygame.K_u:
                    if calibrated:
                        undistort_on = not undistort_on
                        print(f"  Undistort: {'ON' if undistort_on else 'OFF'}")
                    else:
                        print("  No calibration yet — press C first")

        # ── Capture frame ─────────────────────────────────────────────────────
        frame = cam.read(rotation)
        if frame is None:
            time.sleep(0.05)
            continue

        if frame_size is None:
            h, w   = frame.shape[:2]
            frame_size = (w, h)
            zones  = _make_zones(w, h)
            screen = pygame.display.set_mode((w, h))
            pygame.display.set_caption("Camera Calibration")

        # ── Process ───────────────────────────────────────────────────────────
        if undistort_on and map1 is not None:
            display = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)
        else:
            display = frame.copy()

        grey    = cv2.cvtColor(display, cv2.COLOR_BGR2GRAY)
        corners = _find_corners(grey)

        if corners is not None:
            cv2.drawChessboardCorners(display, (COLS, ROWS), corners, True)

        # ── Auto-capture logic ────────────────────────────────────────────────
        now = time.monotonic()
        hold_progress = 0.0   # 0.0–1.0 for the progress bar

        if auto_capture and corners is not None and zones:
            cx = int(np.mean(corners[:, 0, 0]))
            cy = int(np.mean(corners[:, 0, 1]))
            current_zone = next((i for i, (rect, _) in enumerate(zones)
                                 if rect.collidepoint(cx, cy)), None)

            if current_zone != last_zone:
                stable_since = now   # board moved — reset hold timer
                last_zone    = current_zone

            # Capture if held still long enough AND cooldown has passed
            if stable_since is not None and current_zone is not None:
                elapsed = now - stable_since
                hold_progress = min(elapsed / STABLE_HOLD, 1.0)
                if elapsed >= STABLE_HOLD and now - last_capture_t >= AUTO_COOLDOWN:
                    capture_now     = True
                    stable_since    = now    # reset so it can capture again after cooldown
                    last_capture_t  = now
        else:
            stable_since  = None
            last_zone     = None

        # ── Capture sample ────────────────────────────────────────────────────
        if capture_now:
            if corners is None:
                print("  No board visible — move checkerboard into frame")
            else:
                obj_points.append(_OBJP.copy())
                img_points.append(corners)
                flash_until    = now + 0.2
                last_capture_t = now
                print(f"  Captured sample {len(obj_points)}")

        # ── Draw ──────────────────────────────────────────────────────────────
        render = cv2.flip(display, 1) if mirrored else display
        screen.blit(_to_surface(render), (0, 0))

        # Green flash on capture
        if time.monotonic() < flash_until:
            flash = pygame.Surface(screen.get_size(), pygame.SRCALPHA)
            flash.fill((0, 255, 0, 70))
            screen.blit(flash, (0, 0))

        # Coverage zones + captured position dots
        _draw_zones(screen, zones, img_points, font,
                    fw=frame_size[0] if frame_size else 0, mirrored=mirrored)

        # Board-found indicator (top-right)
        if corners is not None:
            ind = _label(font_lg, "BOARD FOUND", _GREEN)
        else:
            ind = _label(font_lg, "no board", _RED)
        w_ind = ind.get_width()
        pygame.draw.rect(screen, (0, 0, 0),
                         (frame_size[0] - w_ind - 16, 6, w_ind + 12, 28))
        screen.blit(ind, (frame_size[0] - w_ind - 10, 10))

        # Status bar (bottom)
        BAR = 44
        bar = pygame.Surface((frame_size[0], BAR), pygame.SRCALPHA)
        bar.fill((0, 0, 0, 190))
        screen.blit(bar, (0, frame_size[1] - BAR))

        by = frame_size[1] - BAR + 4

        # Line 1: sample count + RMS / progress
        screen.blit(_label(font, f"Samples: {len(obj_points)}", _WHITE), (10, by))
        if calibrated:
            q_col = _GREEN if rms < 0.5 else _YELLOW if rms < 1.0 else _RED
            screen.blit(_label(font, f"RMS={rms:.3f} px", q_col), (130, by))
            ud_col = _GREEN if undistort_on else _WHITE
            screen.blit(_label(font, f"Undistort: {'ON' if undistort_on else 'OFF'}", ud_col),
                        (280, by))
        else:
            need = max(0, 6 - len(obj_points))
            msg  = f"Need {need} more sample{'s' if need != 1 else ''} before calibrating"
            screen.blit(_label(font, msg, _YELLOW), (130, by))

        # Auto-capture status + hold progress bar
        auto_col = _GREEN if auto_capture else (160, 160, 160)
        screen.blit(_label(font, f"Auto: {'ON' if auto_capture else 'OFF'}", auto_col),
                    (frame_size[0] - 120, by))
        if auto_capture and hold_progress > 0:
            bar_w = 110
            bar_x = frame_size[0] - bar_w - 5
            bar_y = by + 20
            pygame.draw.rect(screen, (60, 60, 60),   (bar_x, bar_y, bar_w, 10))
            pygame.draw.rect(screen, (255, 200, 0),  (bar_x, bar_y, int(bar_w * hold_progress), 10))

        # Line 2: key hints
        hints = (f"SPACE=capture  A=auto  C=calibrate  U=undistort  "
                 f"M=mirror  R=rotate({rotation}°)  Q=quit")
        screen.blit(_label(font, hints, (180, 180, 180)), (10, by + 20))

        pygame.display.flip()
        clock.tick(30)


def _save(calibrated, opt_mtx, map1, map2, rms, frame_size):
    if calibrated:
        out = _cal_path(frame_size)
        os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
        # camera_matrix saved here is the post-remap optimal matrix.
        # After cv2.remap(map1, map2), the undistorted image has these intrinsics
        # with zero distortion — correct inputs for solvePnP pose estimation.
        np.savez(out,
                 camera_matrix=opt_mtx,
                 dist_coeffs=np.zeros((4, 1), dtype=np.float32),
                 map1=map1,
                 map2=map2,
                 rms=np.array([rms]),
                 frame_size=np.array(frame_size))
        q = ("EXCELLENT" if rms < 0.5 else
             "ACCEPTABLE" if rms < 1.0 else "POOR — recapture with more varied views")
        print(f"\nCalibration saved → {out}")
        print(f"  RMS={rms:.4f} px  [{q}]")
        print(f"  Frame size: {frame_size[0]}×{frame_size[1]}")
        print(f"\n  Run tools/derive_calibrations.py to generate files for other resolutions.")
    else:
        print("\nNo calibration computed — nothing saved.")


if __name__ == '__main__':
    main()
