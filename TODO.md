# HackyRacingRobot — To-Do List

---

## For the AI Assistant

Suggested improvements based on codebase analysis, roughly ordered by priority.

### Testing
- [ ] Add unit tests for `robot/aruco_navigator.py` state machine — synthetic `ArUcoState` inputs, verify state transitions (SEARCHING→ALIGNING→APPROACHING→RECOVERING etc.), no hardware needed
- [ ] Add unit tests for `robot/gps_navigator.py` — synthetic NMEA fixes, verify waypoint sequencing, arrival detection, look-ahead blending, no hardware needed
- [x] Extend `tools/yukon_sim.py` to simulate IMU heading responses (CMD_SENSOR reply ID 7) — already implemented; `_state['imu_heading']` is encoded and returned for `RESP_HEADING`
- [ ] Add `--dry-run` mode to `tools/test_leds.py` so it runs in CI without a Yukon connected (pattern/preset encoding tests only)
- [ ] Add `tools/test_robot.py` bearing-hold test — use the existing IMU simulation to verify `CMD_BEARING` sets a target and the sim converges toward it

### Code & Architecture
- [ ] `robot_daemon.py` is ~2400 lines — consider splitting subsystem classes (`_Camera`, `_Gps`, `_YukonLink`, `_Lidar`) into separate files under `robot/` to reduce merge conflicts and improve readability
- [ ] ArUco navigator COMPLETE state just stops — add a `loop` config option to restart from gate 0 for circuit racing
- [ ] GPS navigator has no fallback when IMU is unavailable between fixes — add a timed dead-reckoning step (hold last bearing for up to N seconds) to avoid stalling mid-straight
- [ ] Obstacle stop in both navigators is binary (halt/go) — add a configurable `obstacle_steer` mode that biases steering away from the nearest obstacle instead of fully halting
- [ ] Add a `--lock` / single-instance guard to `robot_daemon.py` (write PID to `/tmp/robot.pid`, refuse to start if another instance holds it) to enforce the single-frontend constraint at runtime rather than just in docs
- [ ] `tools/yukon_sim_gui.py` has dead code: `_orig_server = _sim.yukon_server` is assigned but never used — remove it
- [ ] `tools/yukon_sim.py` `_tick_manual_drive()` uses hardcoded channel indices and deadzone (CH1/CH3/CH6, deadzone=30, speed_min=0.25) — read from `robot.ini` so the sim stays in sync with actual robot config

### Tooling & Developer Experience
- [ ] Add a `systemd` service file (`robot.service`) for auto-start on boot — `ExecStart=python3 /home/pi/Code/HackyRacingRobot/robot_web.py`, with restart-on-failure
- [x] Add a `Makefile` (or `justfile`) with common tasks: `make upload` (firmware), `make test` (all dry-run tests), `make logs` (tail robot.log)
- [ ] `tools/read_data_log.py` has no export — add a `/api/export.csv` endpoint that streams the current log as CSV for offline analysis

### Documentation
- [ ] Add `docs/AUTONOMOUS.md` — how to extend `AutoType`, wire a new navigator into `_control_thread`, and the state-machine contract (`update()` signature, return values)
- [ ] `docs/SETUP.md` has no "first run" checklist — add a step-by-step section: flash firmware → verify serial ports → test RC → test motors (`--no-motors` bench mode) → calibrate camera → first autonomous run

---

## For the Owner

_Add your own tasks below._

- [ ] Dust cap detection
- [ ]
- [ ]
- [ ]
- [ ]
