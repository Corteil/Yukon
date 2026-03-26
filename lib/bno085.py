"""
bno085.py — Minimal BNO085 SHTP/I2C driver for MicroPython.

Enables the Game Rotation Vector report (accelerometer + gyroscope, no
magnetometer).  Heading is relative to the orientation at power-on, not
compass north.  This is ideal for straight-line bearing hold; for absolute
compass bearing swap _REPORT_GAME for _REPORT_GEOMAG (0x09) once the
magnetometer is calibrated.

Wiring — Adafruit BNO085 breakout to Yukon Qw/ST port (I2C0 at 400 kHz):
  BNO085 VIN  → 3.3 V
  BNO085 GND  → GND
  BNO085 SDA  → Qw/ST SDA
  BNO085 SCL  → Qw/ST SCL
  (P0 and P1 are pulled LOW by default on the Adafruit breakout → I2C mode, no wiring needed)
  (DI pin selects I2C address: unconnected/LOW → 0x4A, HIGH → 0x4B)

Usage:
  imu = BNO085(yukon.i2c)
  while True:
      imu.update()        # call at loop rate (≥20 Hz)
      h = imu.heading()   # 0.0 – 360.0 degrees (relative to startup)
"""

import struct
import math
from utime import sleep_ms

# SHTP constants
_CH_EXE      = 1
_CH_CONTROL  = 2
_CH_REPORTS  = 3
_REPORT_GAME        = 0x08   # Game Rotation Vector (accel + gyro, no magnetometer)
_REPORT_GEOMAG      = 0x09   # Geomagnetic Rotation Vector (accel + mag, no gyro)
_REPORT_GYRO        = 0x02   # Calibrated gyroscope (rad/s)
_REPORT_LINEAR_ACCEL = 0x04  # Linear acceleration — gravity removed (m/s²)
_REPORT_STABILITY   = 0x13   # Stability classifier (0=unknown 1=on_table 2=stationary 3=stable 4=motion)
_SET_FEATURE = 0xFD   # Set Feature command
_ID_REQUEST  = 0xF9   # Product ID Request
_ID_RESPONSE = 0xF8   # Product ID Response
_Q14         = 1 << 14


class BNO085:
    def __init__(self, i2c, addr=0x4A, interval_ms=20):
        """
        i2c         — machine.I2C instance (e.g. yukon.i2c)
        addr        — I2C address: 0x4A (ADDR→GND) or 0x4B (ADDR→3.3V)
        interval_ms — report interval; 20 ms = 50 Hz
        """
        self._i2c  = i2c
        self._addr = addr
        self._seq  = bytearray(8)            # per-channel sequence counters
        self._quat       = (0.0, 0.0, 0.0, 1.0)  # qx, qy, qz, qw (identity)
        self._lin_accel  = (0.0, 0.0, 0.0)        # linear accel x/y/z (m/s²)
        self._gyro_xyz   = (0.0, 0.0, 0.0)        # calibrated gyro x/y/z (rad/s)
        self._stability  = 0                       # stability classifier value
        self._data_received = False                # set True once a valid report is parsed

        self._reset()
        self._enable_report(_REPORT_GAME,         interval_ms * 1000)
        self._enable_report(_REPORT_GYRO,         interval_ms * 1000)
        self._enable_report(_REPORT_LINEAR_ACCEL, interval_ms * 1000)
        self._enable_report(_REPORT_STABILITY,    50000)  # 50 ms — lightweight
        sleep_ms(150)                        # let first reports arrive

    def _reset(self):
        """Soft-reset the BNO085 and wait for it to identify itself."""
        # Send reset command to the executable channel
        self._write_packet(_CH_EXE, [1])
        sleep_ms(500)
        # Drain startup packets (advertisement, reset notice, etc.).
        # Each readfrom() call issues a STOP which causes the BNO085 to
        # dequeue exactly one SHTP packet — so one call per packet.
        # The advertisement is ~5 packets; drain 15 to be safe.
        for _ in range(15):
            self._read_packet()
            sleep_ms(5)
        # Request product ID — BNO085 won't reliably accept Set Feature
        # commands until this handshake is complete.
        self._write_packet(_CH_CONTROL, [_ID_REQUEST, 0])
        deadline = 500
        while deadline > 0:
            ch, payload = self._read_packet()
            if ch == _CH_CONTROL and payload and payload[0] == _ID_RESPONSE:
                break
            sleep_ms(10)
            deadline -= 10

    # ── SHTP I2C transport ─────────────────────────────────────────────────

    # Bytes per I2C read transaction.
    # On RP2040 MicroPython, every readfrom() ends with a STOP condition.
    # The BNO085 treats STOP as "packet consumed — advance to next packet",
    # so TWO readfrom() calls would dequeue TWO packets, discarding the
    # payload of the first.  Fix: read header + payload in ONE call.
    #
    # Size calculation:
    #   4  SHTP header
    #   5  Base Timestamp Reference prefix (0xFB + 4-byte timestamp)
    #  14  Game Rotation Vector payload (report_id … accuracy)
    # ──   ──
    #  23  total; use 28 for headroom
    _READ_SIZE = 28

    def _read_packet(self):
        """Return (channel, payload_bytes) or (None, None) if nothing ready."""
        try:
            buf = self._i2c.readfrom(self._addr, self._READ_SIZE)
        except (OSError, MemoryError):
            return None, None

        length = struct.unpack_from('<H', buf)[0] & 0x7FFF
        payload_len = length - 4
        if payload_len <= 0:
            return None, None          # empty / no data

        channel = buf[2]
        # Clamp to what we actually read; large packets (e.g. advertisement)
        # are partially read — unread tail bytes are discarded on STOP.
        payload = buf[4:4 + min(payload_len, self._READ_SIZE - 4)]
        return channel, payload

    def _write_packet(self, channel, data):
        length = len(data) + 4
        header = struct.pack('<HBB', length, channel, self._seq[channel])
        self._seq[channel] = (self._seq[channel] + 1) & 0xFF
        try:
            self._i2c.writeto(self._addr, header + bytes(data))
        except OSError:
            pass

    def _enable_report(self, report_id, interval_us):
        b = struct.pack('<I', int(interval_us))
        cmd = [_SET_FEATURE, report_id,
               0, 0, 0,           # feature flags, change sensitivity (unused)
               b[0], b[1], b[2], b[3],   # report interval µs
               0, 0, 0, 0,        # batch interval (disabled)
               0, 0, 0, 0]        # sensor-specific config
        self._write_packet(_CH_CONTROL, cmd)

    # ── public API ─────────────────────────────────────────────────────────

    def update(self):
        """Read one pending sensor packet. Call once per motor-loop iteration."""
        channel, payload = self._read_packet()
        if channel != _CH_REPORTS or not payload:
            return
        # Channel 3 payloads may be prefixed by timing sub-records:
        #   0xFB  Base Timestamp Reference — skip 5 bytes (id + 4-byte ts)
        #   0xFE  Delta / wake timestamp   — skip 4 bytes (id + 3-byte data)
        #   0x3F  Compact timing marker    — skip 1 byte
        # Scan forward until we find the GRV report (0x08).
        i = 0
        while i < len(payload):
            rid = payload[i]
            if rid == 0xFB:          # base timestamp: 5-byte record
                i += 5
            elif rid == 0xFE:        # delta/wake timestamp: 4-byte record
                i += 4
            elif rid == 0x3F:        # compact marker: 1-byte record
                i += 1
            elif rid == _REPORT_GAME:
                # 12 bytes: id(1) seq(1) status(1) delay(1) qx(2) qy(2) qz(2) qw(2)
                if len(payload) < i + 12:
                    return
                _Q = 1 << 14
                qx = struct.unpack_from('<h', payload, i + 4)[0]  / _Q
                qy = struct.unpack_from('<h', payload, i + 6)[0]  / _Q
                qz = struct.unpack_from('<h', payload, i + 8)[0]  / _Q
                qw = struct.unpack_from('<h', payload, i + 10)[0] / _Q
                self._quat = (qx, qy, qz, qw)
                self._data_received = True
                return
            elif rid == _REPORT_GYRO:
                # 10 bytes: id(1) seq(1) status(1) delay(1) x(2) y(2) z(2)
                # Q9 fixed-point: scale = 1/512
                if len(payload) < i + 10:
                    return
                _QG = 1 << 9
                gx = struct.unpack_from('<h', payload, i + 4)[0] / _QG
                gy = struct.unpack_from('<h', payload, i + 6)[0] / _QG
                gz = struct.unpack_from('<h', payload, i + 8)[0] / _QG
                self._gyro_xyz = (gx, gy, gz)
                return
            elif rid == _REPORT_LINEAR_ACCEL:
                # 10 bytes: id(1) seq(1) status(1) delay(1) x(2) y(2) z(2)
                # Q8 fixed-point: scale = 1/256
                if len(payload) < i + 10:
                    return
                _QA = 1 << 8
                ax = struct.unpack_from('<h', payload, i + 4)[0] / _QA
                ay = struct.unpack_from('<h', payload, i + 6)[0] / _QA
                az = struct.unpack_from('<h', payload, i + 8)[0] / _QA
                self._lin_accel = (ax, ay, az)
                return
            elif rid == _REPORT_STABILITY:
                # 6 bytes: id(1) seq(1) status(1) delay(1) classifier(1) reserved(1)
                if len(payload) < i + 5:
                    return
                self._stability = payload[i + 4]
                return
            else:
                return  # unrecognised sub-record

    def heading(self):
        """Return heading in degrees (0.0 – 360.0), relative to startup orientation."""
        qx, qy, qz, qw = self._quat
        yaw = math.atan2(2.0 * (qw * qz + qx * qy),
                         1.0 - 2.0 * (qy * qy + qz * qz))
        return math.degrees(yaw) % 360.0

    def roll(self):
        """Return roll in degrees (-180.0 to +180.0), derived from stored quaternion.

        Uses standard aerospace ZYX Euler convention, matching heading().
        Positive roll = right side down.
        """
        qx, qy, qz, qw = self._quat
        sinr_cosp = 2.0 * (qw * qx + qy * qz)
        cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
        return math.degrees(math.atan2(sinr_cosp, cosr_cosp))

    def pitch(self):
        """Return pitch in degrees (-90.0 to +90.0), derived from stored quaternion.

        Positive pitch = nose up.
        """
        qx, qy, qz, qw = self._quat
        sinp = 2.0 * (qw * qy - qz * qx)
        sinp = max(-1.0, min(1.0, sinp))   # clamp for numerical safety at ±90°
        return math.degrees(math.asin(sinp))

    def linear_acceleration(self):
        """Return linear acceleration as (x, y, z) tuple in m/s², gravity removed.

        Useful for wheel-slip detection (motor commanded but near-zero acceleration)
        and collision detection (sudden negative-Z spike on impact).
        Returns (0.0, 0.0, 0.0) until the first report is received.
        """
        return self._lin_accel

    def gyro(self):
        """Return calibrated angular velocity as (x, y, z) tuple in rad/s.

        Useful for turn-rate logging and bearing-rate-of-change computation.
        Returns (0.0, 0.0, 0.0) until the first report is received.
        """
        return self._gyro_xyz

    def stability(self):
        """Return the stability classifier value (int).

        0 = unknown
        1 = on table  (completely stationary, flat surface)
        2 = stationary (stationary but may be held)
        3 = stable    (slow, smooth motion)
        4 = motion    (significant motion detected)

        Returns 0 until the first report is received.
        """
        return self._stability

    def is_moving(self):
        """Return True if the stability classifier reports motion (value == 4)."""
        return self._stability == 4

    def is_still(self):
        """Return True if the classifier reports stationary or on-table (value <= 2)."""
        return 1 <= self._stability <= 2
