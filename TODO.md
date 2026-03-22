# HackyRacingRobot — To-Do List

---

## For the AI Assistant

Suggested improvements based on codebase analysis, roughly ordered by priority.

### Code & Architecture
- [x] Commit the currently uncommitted changes — done (commit `18a273f`)
- [x] Decide whether `camera_cal.npz` should be committed — added to `.gitignore`
- [x] NTRIP credentials — env var overrides (`NTRIP_HOST/PORT/MOUNT/USER/PASSWORD`) implemented
- [x] `camera_monitor.py` and `camera_web.py` share logic — extracted into `robot/camera_controls.py` (constants, sharpness, rotate, make_cam, draw_aruco_on_frame, CalibrationMaps)

### Testing
- [x] Add unit tests for the `gnss/` package — `tools/test_gnss.py` (57 tests, no hardware)
- [x] Expand `tools/test_robot.py` — added AUTO mode, reset_estop, and data log tests (52 tests total)
- [x] Add `--dry-run` flag to `tools/test_gps.py` — runs 20 NMEA parsing tests without hardware or GUI

### Navigation & Autonomy
- [x] LiDAR data (`LidarScan`) is collected but not used by either navigator — add basic obstacle-stop logic to `aruco_navigator.py` and `gps_navigator.py` (halt if any point inside a configurable forward cone is closer than N metres)
- [x] `gps_navigator.py` drives straight between waypoints — add a simple look-ahead / smoothing step to reduce oscillation on longer straights
- [x] The ArUco gate navigator has no recovery behaviour when it loses sight of the target gate — add a slow rotation search state

### Documentation
- [x] `docs/ARCHITECTURE.md` and `docs/SETUP.md` may be out of date following the rename from Yukon to HackyRacingRobot — review and update device paths, file names, and wiring notes
- [x] Add a `docs/CALIBRATION.md` walk-through covering the full camera calibration → ArUco detection pipeline

### Minor / Quality-of-Life
- [x] Video recordings have no maximum duration or size cap — long runs could fill the SD card; add a rolling max-minutes config option to `[output]`
- [x] `robot_web.py` and `robot_mobile.py` both start their own `Robot` instance; if both are launched together they fight over serial ports — document clearly that only one frontend should run at a time, or add a shared-daemon IPC layer
- [x] `tools/gps_route_builder.py` saves routes as plain JSON with no schema version field — add a `"version": 1` key now before the format evolves

---

## For the Owner

_Add your own tasks below._

- [ ]
- [ ]
- [ ]
- [ ]
- [ ]
