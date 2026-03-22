# Camera Calibration

This document covers the full pipeline from capturing calibration images to using corrected frames for ArUco gate detection.

---

## Why calibrate?

The IMX296 global-shutter module fitted with a 2.1 mm wide-angle lens (~100° FOV) introduces significant barrel distortion. Without calibration:

- Straight lines at the frame edges appear curved.
- ArUco marker corner positions are biased outward, causing distance and bearing estimates to be wrong.
- The effect is worst at wide resolutions and near the frame edges — exactly where gates appear during an approach.

Calibration computes a **camera matrix** (focal lengths, principal point) and **distortion coefficients** from a known target. These are used to build lookup maps that remap every pixel, producing an undistorted frame.

---

## Step 1 — Print a calibration target

```bash
python3 tools/make_checkerboard_pdf.py
```

This writes `docs/checkerboard_9x6.pdf` — a 9×6 inner-corner checkerboard sized for A4 paper. Print it, mount it flat on a rigid backing (cardboard or clipboard), and measure the actual square size in millimetres; the default is **25 mm per square**.

> Keep the target flat. Bowing or curling introduces systematic error.

---

## Step 2 — Capture calibration images

Run the calibration tool with the camera at its **maximum sensor resolution** (1456×1088). The intrinsic matrix scales cleanly to lower resolutions; distortion coefficients are resolution-independent.

```bash
python3 tools/calibrate_camera.py
```

### Key controls

| Key | Action |
|-----|--------|
| `A` | Toggle auto-capture (default **on** — waits for stillness, then captures) |
| `M` | Mirror the display (useful when you are holding the board yourself) |
| `C` | Compute calibration (available once ≥ 6 captures are collected) |
| `Q` / `Esc` | Quit and save (calibration is saved automatically after `C`) |

### Zone guidance

The overlay divides the frame into 9 zones (3×3 grid). A zone turns green once a capture with the target fully inside it is accepted. Cover all 9 zones from a range of angles (tilted, rotated, close, far) — **aim for 20–30 captures total** for a stable result.

The tool prints the reprojection RMS error after computing. Values below **1.0 px** are good; below **0.5 px** is excellent.

### Output

```
camera_cal_1456x1088.npz
```

Saved in the repo root. Contents:

| Array | Description |
|-------|-------------|
| `camera_matrix` | 3×3 intrinsic matrix (optimal, post-remap) |
| `dist_coeffs`   | Distortion coefficients (zeros post-remap, kept for reference) |
| `map1`, `map2`  | Pre-computed undistortion lookup maps for 1456×1088 |
| `rms`           | Reprojection RMS error (pixels) |
| `frame_size`    | `[width, height]` this calibration was computed at |

> `camera_cal.npz` (no resolution suffix) is also written for backwards-compat, but the suffixed file should be used going forward.

---

## Step 3 — Derive per-resolution files

The robot typically runs the camera at a lower resolution (default **640×480** in `robot.ini`). Rather than re-capturing at every resolution, `derive_calibrations.py` scales the master calibration mathematically:

```bash
# Single resolution
python3 tools/derive_calibrations.py camera_cal_1456x1088.npz 640x480

# Multiple resolutions at once
python3 tools/derive_calibrations.py camera_cal_1456x1088.npz 640x480 1280x720 1456x1088

# Read target resolutions from robot.ini [camera_calibrations] resolutions
python3 tools/derive_calibrations.py camera_cal_1456x1088.npz
```

Each target gets its own file:

```
camera_cal_640x480.npz
camera_cal_1280x720.npz
```

The tool prints the output paths and reminds you to update `robot.ini`.

---

## Step 4 — Configure robot.ini

Set the `calib_file` key in the `[aruco]` section to the file matching your capture resolution:

```ini
[aruco]
calib_file = camera_cal_640x480.npz
```

`CalibrationMaps` (in `robot/camera_controls.py`) loads this file lazily on the first call to `get_maps(w, h)` and caches the maps. If the file is missing or the resolution doesn't match, undistortion is skipped silently and a warning is logged.

---

## Step 5 — Verify in live view

Start the camera tool:

```bash
python3 camera_monitor.py   # Pygame desktop
python3 camera_web.py       # web, port 8080
```

Press `K` (or toggle **Calib** in the web UI) to enable undistortion. Hold the checkerboard up to the lens and verify that straight lines are straight across the whole frame, including the corners.

---

## ArUco detection pipeline

Once calibration is active, the full detection pipeline is:

```
picamera2 capture_array()
    │  BGR → RGB ([:, :, ::-1])
    │  rotate (robot.ini [camera] rotate_degrees)
    │
    ▼
CalibrationMaps.get_maps(w, h)
    │  cv2.remap(frame, map1, map2) → undistorted frame
    │
    ▼
ArucoDetector.detect(frame)
    │  cv2.aruco.detectMarkers()
    │  solvePnP (tag_size from robot.ini [aruco] tag_size_m)
    │    → rvec, tvec
    │  distance = ‖tvec‖
    │  bearing  = atan2(tvec[0], tvec[2])  (degrees, + = right)
    │
    ▼
ArucoNavigator / robot.get_aruco_state()
```

Key configuration in `robot.ini`:

| Key | Section | Default | Notes |
|-----|---------|---------|-------|
| `calib_file` | `[aruco]` | `camera_cal.npz` | Path to the per-resolution `.npz` |
| `tag_size_m` | `[aruco]` | `0.15` | Physical side length of printed tags (metres) |
| `dictionary` | `[aruco]` | `DICT_4X4_50` | Must match the dictionary used to generate the tags |
| `rotate_degrees` | `[camera]` | `0` | Applied before detection; use `180` if camera is mounted upside-down |

---

## Troubleshooting

**No Calib button / toggle appears**
The calibration file path in `[aruco] calib_file` does not exist. Check the path and run `derive_calibrations.py`.

**Undistortion looks wrong / image is cropped**
The calibration was captured at a different resolution. Re-derive for your actual capture resolution using `derive_calibrations.py`.

**RMS error above 2.0 px**
Too few captures, uneven zone coverage, or a bent target board. Recapture with 25+ images covering all zones and angles. Discard outlier captures (tool shows per-image error after computing).

**ArUco distance estimates are off**
Check `tag_size_m` in `robot.ini` — it must match the actual printed tag size. Also verify calibration is enabled (log shows `CalibrationMaps loaded` at startup).
