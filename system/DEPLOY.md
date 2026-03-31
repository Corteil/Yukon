# Deploying the Robot Service

This directory contains the systemd unit file for running the robot dashboard
automatically on boot.

---

## Prerequisites

Complete `docs/SETUP.md` first (device tree overlays, udev rules, Python
dependencies, firmware upload).

---

## 1 — Adjust the install path (if needed)

The service file assumes the repository lives at:

```
/home/pi/Code/HackyRacingRobot/
```

If your path differs, edit `robot.service` before copying:

```bash
nano system/robot.service
# update WorkingDirectory and ExecStart to match your actual path
```

---

## 2 — Add the `pi` user to required groups

The robot process needs access to serial ports, GPIO, and (if using hardware
PWM for the LiDAR motor) the `pwm` sysfs node:

```bash
sudo usermod -aG dialout,gpio pi
```

Log out and back in (or reboot) for group membership to take effect.

---

## 3 — Install and enable the service

```bash
# Copy the unit file
sudo cp system/robot.service /etc/systemd/system/robot.service

# Reload systemd so it sees the new file
sudo systemctl daemon-reload

# Enable at boot
sudo systemctl enable robot.service

# Start immediately (without rebooting)
sudo systemctl start robot.service
```

---

## 4 — Verify it is running

```bash
sudo systemctl status robot.service
```

The web dashboard should be reachable at `http://<pi-ip>:5000/` within a few
seconds of the service starting.

---

## Common management commands

| Action | Command |
|--------|---------|
| Start | `sudo systemctl start robot` |
| Stop | `sudo systemctl stop robot` |
| Restart | `sudo systemctl restart robot` |
| Disable autostart | `sudo systemctl disable robot` |
| View live logs | `sudo journalctl -u robot -f` |
| View last 100 lines | `sudo journalctl -u robot -n 100` |

The daemon also writes to `logs/robot.log` inside the repository (rotating,
5 MB × 3 files). Use `make tail` from the repo root to follow that file.

---

## Switching frontends

The service starts `robot_web.py` (port 5000) by default. To use the mobile
dashboard instead, change the `ExecStart` line in the unit file:

```ini
ExecStart=/usr/bin/python3 /home/pi/Code/HackyRacingRobot/robot_mobile.py
```

Then reload and restart:

```bash
sudo systemctl daemon-reload
sudo systemctl restart robot.service
```

> **Important:** Never run more than one frontend at a time. Each frontend
> creates its own `Robot` instance and claims the same hardware ports (Yukon
> USB serial, LiDAR UART, GPS USB serial). Running two simultaneously causes
> port-conflict errors.

---

## Updating the service file

After editing `system/robot.service` in the repository, re-copy and reload:

```bash
sudo cp system/robot.service /etc/systemd/system/robot.service
sudo systemctl daemon-reload
sudo systemctl restart robot.service
```
