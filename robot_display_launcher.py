"""
robot_display_launcher.py — Minimal example showing how to integrate
DisplayBridge alongside robot_daemon.Robot in your existing launcher.

If you already have robot_dashboard.py or robot_gui.py running, just add
the four DisplayBridge lines to that file instead of using this one.

Usage:
    python3 robot_display_launcher.py
    python3 robot_display_launcher.py --no-camera --no-lidar
    python3 robot_display_launcher.py --display-port /dev/ttyACM0

The display board will appear as /dev/ttyACM0 (or ACM1 if Yukon is also
enumerated as ACM0).  Use:
    ls /dev/ttyACM*
to identify which port is the display, then add a udev rule so it's stable:

    # /etc/udev/rules.d/99-hacky-robot.rules
    SUBSYSTEM=="tty", ATTRS{idVendor}=="303a", ATTRS{idProduct}=="4001", \
        SYMLINK+="ttyACM_display"
    # reload: sudo udevadm control --reload && sudo udevadm trigger

The ESP32-S3 native USB vendor/product IDs above are the defaults for
Espressif CDC devices; verify with: udevadm info /dev/ttyACM0
"""

import argparse
import logging
import signal
import time

from robot_daemon import Robot, RobotMode
from display_bridge import DisplayBridge

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s %(levelname)-8s %(name)s: %(message)s',
)
log = logging.getLogger(__name__)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--no-camera',  action='store_true')
    ap.add_argument('--no-lidar',   action='store_true')
    ap.add_argument('--no-gps',     action='store_true')
    ap.add_argument('--no-motors',  action='store_true')
    ap.add_argument('--display-port', default='auto',
                    help='Serial port for JC3248W535C (default: auto)')
    ap.add_argument('--display-hz', type=float, default=10.0,
                    help='Telemetry send rate to display (default: 10 Hz)')
    args = ap.parse_args()

    # ── Start robot_daemon ────────────────────────────────────────────────
    robot = Robot(
        no_camera = args.no_camera,
        no_lidar  = args.no_lidar,
        no_gps    = args.no_gps,
        no_motors = args.no_motors,
    )
    robot.start()
    log.info('Robot daemon started')

    # ── Start display bridge ──────────────────────────────────────────────
    bridge = DisplayBridge(robot,
                           port=args.display_port,
                           hz=args.display_hz)
    bridge.start()
    log.info('Display bridge started (port=%s, hz=%.0f)',
             args.display_port, args.display_hz)

    # ── Graceful shutdown on Ctrl+C / SIGTERM ─────────────────────────────
    stop = [False]

    def _shutdown(sig, frame):
        log.info('Shutting down …')
        stop[0] = True

    signal.signal(signal.SIGINT,  _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    try:
        while not stop[0]:
            time.sleep(0.5)
    finally:
        bridge.stop()
        robot.stop()
        log.info('Clean exit.')


if __name__ == '__main__':
    main()
