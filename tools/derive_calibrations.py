#!/usr/bin/env python3
"""
derive_calibrations.py — Derive per-resolution calibration files from a master.

Scales the camera intrinsic matrix to each target resolution and recomputes
the undistortion maps.  Distortion coefficients are resolution-independent
and are carried over unchanged, so a single capture session at full sensor
resolution generates valid calibration for every resolution you run the
camera at.

Usage::
    # Resolutions from [camera_calibrations] resolutions in robot.ini
    python3 tools/derive_calibrations.py camera_cal_1456x1088.npz

    # Explicit resolutions on the command line
    python3 tools/derive_calibrations.py camera_cal_1456x1088.npz 640x480 1280x720

    # Use a different config file
    python3 tools/derive_calibrations.py --config path/to/robot.ini camera_cal_1456x1088.npz

Output files are written alongside the master file, e.g.::
    camera_cal_640x480.npz
    camera_cal_1280x720.npz
"""

import argparse
import configparser
import os
import sys

import cv2
import numpy as np


def _parse_resolution(s: str):
    """Parse 'WxH' or 'W×H' string → (width, height) ints."""
    for sep in ('x', 'X', '×'):
        if sep in s:
            w, h = s.split(sep, 1)
            return int(w.strip()), int(h.strip())
    raise ValueError(f"Cannot parse resolution {s!r} — expected WxH (e.g. 640x480)")


def derive(master_path: str, target_w: int, target_h: int) -> str:
    """
    Derive a calibration file for *target_w* × *target_h* from *master_path*.

    Returns the path of the written file.
    """
    cal = np.load(master_path)
    src_w, src_h = int(cal['frame_size'][0]), int(cal['frame_size'][1])

    sx = target_w / src_w
    sy = target_h / src_h

    # Scale the original (pre-optimal) intrinsic matrix.
    # We load the master's camera_matrix which is already the post-remap optimal
    # matrix at the source resolution — scale it to the target resolution.
    src_mtx = cal['camera_matrix'].astype(np.float64)
    scaled_mtx = src_mtx.copy()
    scaled_mtx[0, 0] *= sx   # fx
    scaled_mtx[1, 1] *= sy   # fy
    scaled_mtx[0, 2] *= sx   # cx
    scaled_mtx[1, 2] *= sy   # cy

    # dist_coeffs are normalised — they're the same at any resolution.
    # The master already saves zeros (post-remap), so this stays zeros.
    dist = cal['dist_coeffs'].astype(np.float64)

    new_size = (target_w, target_h)

    # Recompute the optimal matrix and undistortion maps at the new resolution.
    opt_mtx, _ = cv2.getOptimalNewCameraMatrix(
        scaled_mtx, dist, new_size, 1, new_size)
    map1, map2 = cv2.initUndistortRectifyMap(
        scaled_mtx, dist, None, opt_mtx, new_size, cv2.CV_16SC2)

    out_dir  = os.path.dirname(os.path.abspath(master_path))
    out_path = os.path.join(out_dir, f'camera_cal_{target_w}x{target_h}.npz')

    np.savez(out_path,
             camera_matrix=opt_mtx.astype(np.float32),
             dist_coeffs=np.zeros((4, 1), dtype=np.float32),
             map1=map1,
             map2=map2,
             rms=cal['rms'],
             frame_size=np.array([target_w, target_h]))

    return out_path


def main():
    _HERE = os.path.dirname(os.path.abspath(__file__))
    default_ini = os.path.join(_HERE, '..', 'robot.ini')

    parser = argparse.ArgumentParser(
        description="Derive per-resolution calibration files from a master .npz")
    parser.add_argument('master',
                        help='Master calibration file (e.g. camera_cal_1456x1088.npz)')
    parser.add_argument('resolutions', nargs='*', metavar='WxH',
                        help='Target resolutions (e.g. 640x480 1280x720). '
                             'Defaults to [camera_calibrations] resolutions in robot.ini')
    parser.add_argument('--config', default=default_ini,
                        help='Path to robot.ini (default: ../robot.ini)')
    args = parser.parse_args()

    if not os.path.exists(args.master):
        sys.exit(f"ERROR: master file not found: {args.master}")

    # Load master to show source resolution
    cal = np.load(args.master)
    src_w, src_h = int(cal['frame_size'][0]), int(cal['frame_size'][1])
    print(f"Master : {args.master}  ({src_w}×{src_h})")

    # Resolve target resolutions: CLI > robot.ini > error
    if args.resolutions:
        targets = [_parse_resolution(r) for r in args.resolutions]
    else:
        cfg = configparser.ConfigParser()
        cfg.read(args.config)
        raw = cfg.get('camera_calibrations', 'resolutions', fallback='').strip()
        if not raw:
            sys.exit(
                "No resolutions specified.\n"
                "Pass them on the command line (e.g. 640x480 1280x720)\n"
                "or add [camera_calibrations] resolutions = 640x480, 1280x720 to robot.ini"
            )
        targets = [_parse_resolution(r) for r in raw.split(',')]

    print(f"Targets: {', '.join(f'{w}×{h}' for w, h in targets)}\n")

    for w, h in targets:
        if (w, h) == (src_w, src_h):
            print(f"  {w}×{h}  — same as master, skipped")
            continue
        out = derive(args.master, w, h)
        print(f"  {w}×{h}  → {os.path.basename(out)}")

    print("\nDone.  Update [aruco] calib_file in robot.ini to:")
    print(f"  calib_file = camera_cal_{{width}}x{{height}}.npz")
    print("robot.py will substitute the camera resolution automatically.")


if __name__ == '__main__':
    main()
