#!/usr/bin/env python3
"""
ld06.py — Driver for the LDROBOT LD06 DTOF LiDAR.

Hardware setup (Raspberry Pi 5)
--------------------------------
  GPIO 15 (RX)  ← LD06 Tx   (UART0 @ 230400 8N1)
  GPIO 18 (PWM) → LD06 PWM  (30 kHz, 40% duty ≈ 10 Hz scan rate)
  3.3 V / GND   as required

Required /boot/firmware/config.txt overlays::

    dtoverlay=uart0-pi5                 # GPIO 14/15 → /dev/ttyAMA0
    dtoverlay=pwm,pin=18,func=2         # GPIO 18 → PWM0 (pwmchip0/pwm0)

Packet layout (47 bytes per packet, 12 points):
  Byte  0    : 0x54  header
  Byte  1    : 0x2C  VerLen (version=2, data_len=12)
  Bytes 2-3  : speed        uint16 LE  (°/s × 100)
  Bytes 4-5  : start_angle  uint16 LE  (° × 100)
  Bytes 6-41 : 12 × { distance uint16 LE mm, intensity uint8 }
  Bytes 42-43: end_angle    uint16 LE  (° × 100)
  Bytes 44-45: timestamp    uint16 LE  (ms, wraps at 30 000)
  Byte  46   : CRC8  (poly 0x4D, over bytes 0-45)

Usage::
    from ld06 import LD06

    lidar = LD06('/dev/ttyAMA0')
    lidar.start()
    scan = lidar.get_scan()     # LidarScan(angles, distances, rpm)
    lidar.stop()
"""

import logging
import os
import struct
import threading
from dataclasses import dataclass, field
from typing import List, Optional

log = logging.getLogger(__name__)

# ── Constants ─────────────────────────────────────────────────────────────────

_HEADER     = 0x54
_VERLEN     = 0x2C
_PACKET_LEN = 47
_N_POINTS   = 12
_BAUD       = 230400

# PWM defaults: 30 kHz, 40% duty → ~10 Hz scan rate
_PWM_FREQ_HZ   = 30_000
_PWM_DUTY_PCT  = 40
_PWM_PERIOD_NS = 1_000_000_000 // _PWM_FREQ_HZ          # 33 333 ns
_PWM_DUTY_NS   = _PWM_PERIOD_NS * _PWM_DUTY_PCT // 100  # 13 333 ns

# CRC-8 lookup table (polynomial 0x4D)
_CRC_TABLE: List[int] = []
for _i in range(256):
    _crc = _i
    for _ in range(8):
        _crc = ((_crc << 1) ^ 0x4D) & 0xFF if (_crc & 0x80) else (_crc << 1) & 0xFF
    _CRC_TABLE.append(_crc)


def _crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc = _CRC_TABLE[crc ^ b]
    return crc


# ── Sysfs PWM helper ──────────────────────────────────────────────────────────

class _SysfsPWM:
    """
    Minimal sysfs PWM controller.

    Requires the pwm overlay to have exported the channel already via
    ``dtoverlay=pwm,pin=18,func=2`` (creates /sys/class/pwm/pwmchip0/pwm0).
    """

    def __init__(self, chip: int = 0, channel: int = 0):
        self._root    = f"/sys/class/pwm/pwmchip{chip}/pwm{channel}"
        self._export  = f"/sys/class/pwm/pwmchip{chip}/export"
        self._channel = channel

    def start(self, period_ns: int = _PWM_PERIOD_NS, duty_ns: int = _PWM_DUTY_NS):
        try:
            if not os.path.exists(self._root):
                self._write(self._export, self._channel)
            # Must set period before duty_cycle
            self._write(f"{self._root}/period",     period_ns)
            self._write(f"{self._root}/duty_cycle", duty_ns)
            self._write(f"{self._root}/enable",     1)
            log.info("LD06 PWM started: %s  %d Hz  %d%% duty",
                     self._root, _PWM_FREQ_HZ, _PWM_DUTY_PCT)
        except OSError as e:
            log.warning("LD06 PWM setup failed: %s "
                        "(check dtoverlay=pwm,pin=18,func=2 in config.txt)", e)

    def stop(self):
        try:
            self._write(f"{self._root}/enable", 0)
        except OSError:
            pass

    @staticmethod
    def _write(path: str, value):
        with open(path, 'w') as f:
            f.write(str(value))


# ── Public data type ──────────────────────────────────────────────────────────

@dataclass
class LidarScan:
    """One full 360° accumulation of LD06 measurements."""
    angles:    List[float] = field(default_factory=list)   # degrees [0, 360)
    distances: List[int]   = field(default_factory=list)   # mm  (0 = invalid)
    rpm:       float       = 0.0


# ── Driver ────────────────────────────────────────────────────────────────────

class LD06:
    """
    Streaming driver for the LD06 LiDAR.

    Spawns a background reader thread on ``start()``.
    Motor speed is controlled via hardware PWM on GPIO 18 (sysfs).
    Call ``get_scan()`` from any thread to retrieve the latest complete scan.

    Parameters
    ----------
    port        : UART device, default ``/dev/ttyAMA0`` (GPIO 15)
    pwm_chip    : sysfs PWM chip number, default 0  (pwmchip0)
    pwm_channel : sysfs PWM channel,    default 0  (pwm0 → GPIO 18)
    """

    def __init__(self,
                 port:        str = '/dev/ttyAMA0',
                 pwm_chip:    int = 0,
                 pwm_channel: int = 0):
        self._port     = port
        self._pwm      = _SysfsPWM(pwm_chip, pwm_channel)
        self._lock     = threading.Lock()
        self._scan     = LidarScan()
        self._ok       = False
        self._stop_evt = threading.Event()

    # ── public API ────────────────────────────────────────────────────────────

    def start(self):
        self._pwm.start()
        threading.Thread(target=self._run, daemon=True, name="ld06").start()

    def stop(self):
        self._stop_evt.set()
        self._pwm.stop()

    @property
    def ok(self) -> bool:
        return self._ok

    def get_scan(self) -> LidarScan:
        with self._lock:
            return self._scan

    # ── background reader ─────────────────────────────────────────────────────

    def _run(self):
        import serial
        ser = None

        acc_angles:    List[float] = []
        acc_distances: List[int]   = []
        last_angle: float = -1.0

        while not self._stop_evt.is_set():
            try:
                ser = serial.Serial(self._port, _BAUD, timeout=1.0)
                log.info("LD06 opened on %s", self._port)
                self._ok = True
                buf = b''

                while not self._stop_evt.is_set():
                    buf += ser.read(max(1, ser.in_waiting))

                    while len(buf) >= _PACKET_LEN:
                        idx = buf.find(bytes([_HEADER, _VERLEN]))
                        if idx < 0:
                            buf = buf[-1:]
                            break
                        if idx > 0:
                            buf = buf[idx:]
                            continue
                        if len(buf) < _PACKET_LEN:
                            break

                        pkt = buf[:_PACKET_LEN]
                        if _crc8(pkt[:-1]) != pkt[-1]:
                            log.debug("LD06 CRC error — skipping byte")
                            buf = buf[1:]
                            continue

                        buf = buf[_PACKET_LEN:]
                        angles, distances, rpm = self._parse_packet(pkt)

                        # Publish scan on each revolution wrap-through-0°
                        if last_angle > 270 and angles and angles[0] < 90:
                            with self._lock:
                                self._scan = LidarScan(
                                    angles    = list(acc_angles),
                                    distances = list(acc_distances),
                                    rpm       = rpm,
                                )
                            acc_angles    = []
                            acc_distances = []

                        acc_angles.extend(angles)
                        acc_distances.extend(distances)
                        if angles:
                            last_angle = angles[-1]

            except Exception as exc:
                self._ok = False
                log.warning("LD06 error: %s — retrying in 3 s", exc)
                if ser:
                    try:
                        ser.close()
                    except Exception:
                        pass
                    ser = None
                self._stop_evt.wait(3.0)

        self._ok = False
        if ser:
            try:
                ser.close()
            except Exception:
                pass

    # ── packet parser ─────────────────────────────────────────────────────────

    @staticmethod
    def _parse_packet(pkt: bytes):
        speed_raw,  = struct.unpack_from('<H', pkt, 2)
        start_raw,  = struct.unpack_from('<H', pkt, 4)
        end_raw,    = struct.unpack_from('<H', pkt, 42)

        start_deg = start_raw / 100.0
        end_deg   = end_raw   / 100.0
        rpm       = speed_raw / 100.0 / 6.0   # °/s → RPM

        span = end_deg - start_deg
        if span < 0:
            span += 360.0
        step = span / (_N_POINTS - 1) if _N_POINTS > 1 else 0.0

        angles:    List[float] = []
        distances: List[int]   = []

        for i in range(_N_POINTS):
            offset = 6 + i * 3
            dist, intensity = struct.unpack_from('<HB', pkt, offset)
            angle = (start_deg + i * step) % 360.0
            angles.append(round(angle, 2))
            distances.append(dist if intensity > 0 else 0)

        return angles, distances, rpm
