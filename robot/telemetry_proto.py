"""
robot/telemetry_proto.py — Binary telemetry protocol for HackyRacingRobot SiK radio link.

Shared by the robot-side bridge (tools/serial_telemetry_v2.py) and the ground station
(tools/ground_station_v2.py).  Has NO dependency on robot_daemon — safe to import on the
laptop-side ground station without the full robot stack.

Frame format
------------
  [SYNC:2][TYPE:1][FLAGS:1][LEN:2 LE][PAYLOAD:N][CRC16:2 LE]

  SYNC   = 0x7E 0x42
  FLAGS  bit 0 = payload is zlib-compressed (decompress before parsing)
          bits 1-7 reserved, must be zero
  LEN    payload length in bytes (little-endian uint16)
  CRC16  CCITT over TYPE + FLAGS + LEN[0] + LEN[1] + PAYLOAD

Frame overhead: 8 bytes per packet.

Downlink packet types (robot → ground)
---------------------------------------
  0x01  STATE   6 B   5 Hz   mode, drive, flags, speed_scale
  0x02  TELEM  14 B   5 Hz   voltage, current, temps, faults, IMU heading/pitch/roll
  0x03  GPS    19 B   2 Hz   lat, lon, alt, speed, heading, fix, h_error, hdop, sats
  0x04  SYS     4 B   1 Hz   cpu, mem, disk, pi_temp
  0x05  NAV    10 B   2 Hz   nav state, gate, wp, dist, bearing, bearing_error, tags
  0x06  LIDAR   var   1 Hz   distance array, zlib compressed
  0x07  ALARM   var   event  alarm_id, severity, message string
  0x08  TAGS    var   5 Hz   visible ArUco tags: id, cam, cx, cy, distance, bearing
  0x09  MOD_TELEM 9 B 5 Hz  per-module: fl/fr/rl/rr temp(°C), current(A×10), faults(bits)
  0x0A  INA      8 B 1 Hz  INA237 input power: voltage(mV), current(mA), power(10mW), die_temp(°C), flags

Uplink packet types (ground → robot)
--------------------------------------
  0x81  CMD     2 B   on demand   cmd_id, param
  0x82  RTCM    var   on demand   raw RTCM3 bytes → forwarded to TAU1308
  0x83  PING    0 B   keepalive

Null sentinels
--------------
  u16 fields that can be absent → 0xFFFF
  i16 fields that can be absent → 0x7FFF  (= +3276.7°, physically impossible)
  u8  heading/bearing absent    → 0xFF

See PROTO_V2.md (docs/) for the rationale behind each encoding choice.
"""

import struct
import zlib
from typing import Generator, Optional, Tuple

# ── Constants ─────────────────────────────────────────────────────────────────

SYNC_0 = 0x7E
SYNC_1 = 0x42

# Downlink types
TYPE_STATE  = 0x01
TYPE_TELEM  = 0x02
TYPE_GPS    = 0x03
TYPE_SYS    = 0x04
TYPE_NAV    = 0x05
TYPE_LIDAR  = 0x06
TYPE_ALARM  = 0x07
TYPE_TAGS   = 0x08
TYPE_MOD_TELEM = 0x09  # per-module telemetry (4 × BigMotorModule)
TYPE_INA       = 0x0A  # INA237 input power monitor

# Camera ID constants for TAGS packet
CAM_FRONT_LEFT  = 0
CAM_FRONT_RIGHT = 1
CAM_REAR        = 2
_CAM_NAMES = {CAM_FRONT_LEFT: 'front_left', CAM_FRONT_RIGHT: 'front_right', CAM_REAR: 'rear'}
_CAM_IDS   = {v: k for k, v in _CAM_NAMES.items()}

# Uplink types
TYPE_CMD    = 0x81
TYPE_RTCM   = 0x82
TYPE_PING   = 0x83

# FLAGS byte bits
FLAG_ZLIB   = 0x01   # payload is zlib-compressed

# Null sentinels
NULL_U16    = 0xFFFF
NULL_I16    = 0x7FFF
NULL_U8     = 0xFF

# ── RobotMode encoding ────────────────────────────────────────────────────────

MODE_MANUAL = 0
MODE_AUTO   = 1
MODE_ESTOP  = 2

# ── NavState encoding ─────────────────────────────────────────────────────────

NAV_IDLE       = 0
NAV_SEARCHING  = 1
NAV_ALIGNING   = 2
NAV_DRIVING    = 3
NAV_ARRIVED    = 4
NAV_COMPLETE   = 5

NAV_STATE_NAMES = {
    NAV_IDLE:      "IDLE",
    NAV_SEARCHING: "SEARCHING",
    NAV_ALIGNING:  "ALIGNING",
    NAV_DRIVING:   "DRIVING",
    NAV_ARRIVED:   "ARRIVED",
    NAV_COMPLETE:  "COMPLETE",
}

# ── Alarm IDs ─────────────────────────────────────────────────────────────────

ALARM_ESTOP          = 0x01
ALARM_RC_LOST        = 0x02
ALARM_MOTOR_FAULT    = 0x03
ALARM_GPS_LOST       = 0x04
ALARM_LIDAR_LOST     = 0x05
ALARM_CAMERA_LOST    = 0x06
ALARM_LOW_VOLTAGE    = 0x07
ALARM_OVERTEMP       = 0x08
ALARM_NTRIP_DISC     = 0x09

ALARM_SEV_INFO     = 0
ALARM_SEV_WARNING  = 1
ALARM_SEV_CRITICAL = 2

ALARM_NAMES = {
    ALARM_ESTOP:       "ESTOP",
    ALARM_RC_LOST:     "RC_LOST",
    ALARM_MOTOR_FAULT: "MOTOR_FAULT",
    ALARM_GPS_LOST:    "GPS_LOST",
    ALARM_LIDAR_LOST:  "LIDAR_LOST",
    ALARM_CAMERA_LOST: "CAMERA_LOST",
    ALARM_LOW_VOLTAGE: "LOW_VOLTAGE",
    ALARM_OVERTEMP:    "OVERTEMP",
    ALARM_NTRIP_DISC:  "NTRIP_DISC",
}

# ── CMD IDs ───────────────────────────────────────────────────────────────────

CMD_ESTOP             = 0x01
CMD_RESET_ESTOP       = 0x02
CMD_SET_MODE          = 0x03   # param: MODE_MANUAL / MODE_AUTO
CMD_DATA_LOG_TOGGLE   = 0x04
CMD_GPS_BOOKMARK      = 0x05
CMD_RECORD_TOGGLE     = 0x06
CMD_BENCH_TOGGLE      = 0x07
CMD_NO_MOTORS_TOGGLE  = 0x08
CMD_ARUCO_TOGGLE      = 0x09
CMD_NAV_RESET         = 0x0A
CMD_NAV_PAUSE_TOGGLE  = 0x0B

CMD_NAMES = {
    CMD_ESTOP:            "ESTOP",
    CMD_RESET_ESTOP:      "RESET_ESTOP",
    CMD_SET_MODE:         "SET_MODE",
    CMD_DATA_LOG_TOGGLE:  "DATA_LOG_TOGGLE",
    CMD_GPS_BOOKMARK:     "GPS_BOOKMARK",
    CMD_RECORD_TOGGLE:    "RECORD_TOGGLE",
    CMD_BENCH_TOGGLE:     "BENCH_TOGGLE",
    CMD_NO_MOTORS_TOGGLE: "NO_MOTORS_TOGGLE",
    CMD_ARUCO_TOGGLE:     "ARUCO_TOGGLE",
    CMD_NAV_RESET:        "NAV_RESET",
    CMD_NAV_PAUSE_TOGGLE: "NAV_PAUSE_TOGGLE",
}

# ── FLAGS bitmask (STATE packet) ──────────────────────────────────────────────

SF_RC_ACTIVE      = 1 << 0
SF_LIDAR_OK       = 1 << 1
SF_GPS_OK         = 1 << 2
SF_CAM_OK         = 1 << 3   # front-left (primary) camera OK
SF_CAM_RECORDING  = 1 << 4
SF_DATA_LOGGING   = 1 << 5
SF_NO_MOTORS      = 1 << 6
SF_BENCH_ENABLED  = 1 << 7
SF_FRONT_CAP      = 1 << 8
SF_REAR_CAP       = 1 << 9
SF_CAM_FR_OK      = 1 << 10  # front-right camera OK
SF_CAM_RE_OK      = 1 << 11  # rear camera OK
# Auto-type (2 bits, only meaningful when mode == AUTO)
#   00 = Camera   01 = GPS   10 = Cam+GPS
SF_AUTO_TYPE_0    = 1 << 12
SF_AUTO_TYPE_1    = 1 << 13


# ══════════════════════════════════════════════════════════════════════════════
# CRC-16/CCITT-FALSE  (init=0xFFFF, poly=0x1021, refin=False, refout=False)
# ══════════════════════════════════════════════════════════════════════════════

def _crc16(data: bytes | bytearray) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc


# ══════════════════════════════════════════════════════════════════════════════
# Frame builder
# ══════════════════════════════════════════════════════════════════════════════

def encode_frame(ptype: int, payload: bytes, compress: bool = False) -> bytes:
    """Wrap *payload* in a framed packet of type *ptype*.

    If *compress* is True (and the payload is non-trivial), zlib-deflate the
    payload and set FLAG_ZLIB.  The caller should only set compress=True for
    variable-length payloads (LIDAR, RTCM) — fixed small payloads gain nothing.
    """
    flags = 0
    if compress and len(payload) > 16:
        compressed = zlib.compress(payload, level=6)
        if len(compressed) < len(payload):
            payload = compressed
            flags |= FLAG_ZLIB

    length = len(payload)
    len_bytes = struct.pack('<H', length)
    crc_data = bytes([ptype, flags]) + len_bytes + payload
    crc = _crc16(crc_data)

    return (
        bytes([SYNC_0, SYNC_1, ptype, flags])
        + len_bytes
        + payload
        + struct.pack('<H', crc)
    )


# ══════════════════════════════════════════════════════════════════════════════
# Frame decoder
# ══════════════════════════════════════════════════════════════════════════════

class FrameDecoder:
    """
    Incremental byte-stream frame decoder.

    Feed raw bytes from the serial port (or TCP socket) with feed().
    Each call yields zero or more (ptype, payload) tuples where *payload* has
    already been decompressed if FLAG_ZLIB was set.

    Corrupt frames (bad CRC, invalid length) are silently dropped and the
    scanner re-syncs on the next SYNC_0/SYNC_1 pair.

    Usage::

        decoder = FrameDecoder()
        for ptype, payload in decoder.feed(chunk):
            handle(ptype, payload)
    """

    _HEADER_LEN = 6   # SYNC[2] + TYPE[1] + FLAGS[1] + LEN[2]

    def __init__(self):
        self._buf = bytearray()

    def feed(self, data: bytes | bytearray) -> Generator[Tuple[int, bytes], None, None]:
        self._buf.extend(data)
        while True:
            result = self._try_decode()
            if result is None:
                break
            yield result

    def _try_decode(self) -> Optional[Tuple[int, bytes]]:
        buf = self._buf

        # Find sync
        while len(buf) >= 2:
            if buf[0] == SYNC_0 and buf[1] == SYNC_1:
                break
            buf.pop(0)
        else:
            # Less than 2 bytes — can't decide yet
            return None

        # Need at least a full header
        if len(buf) < self._HEADER_LEN:
            return None

        ptype  = buf[2]
        flags  = buf[3]
        length = struct.unpack_from('<H', buf, 4)[0]

        total = self._HEADER_LEN + length + 2   # +2 for CRC
        if len(buf) < total:
            return None

        payload_raw = bytes(buf[self._HEADER_LEN : self._HEADER_LEN + length])
        crc_recv    = struct.unpack_from('<H', buf, self._HEADER_LEN + length)[0]

        # Verify CRC
        crc_calc = _crc16(bytes([ptype, flags]) + struct.pack('<H', length) + payload_raw)
        if crc_calc != crc_recv:
            # Bad frame — drop first byte and re-scan
            buf.pop(0)
            return None

        # Good frame — consume it
        del buf[:total]

        # Decompress if needed
        if flags & FLAG_ZLIB:
            try:
                payload = zlib.decompress(payload_raw)
            except zlib.error:
                return None
        else:
            payload = payload_raw

        return (ptype, payload)


# ══════════════════════════════════════════════════════════════════════════════
# Downlink encoders  (robot → ground)
# ══════════════════════════════════════════════════════════════════════════════

# STATE — 6 bytes
_FMT_STATE = struct.Struct('<BbbHB')   # mode, drv_l, drv_r, flags, speed_pct

def encode_state(mode: int, drv_left: float, drv_right: float,
                 flags: int, speed_scale: float) -> bytes:
    """Encode a STATE packet.

    *drv_left* / *drv_right*  float -1.0…+1.0
    *speed_scale*             float  0.0…1.0
    """
    drv_l = max(-100, min(100, int(round(drv_left  * 100))))
    drv_r = max(-100, min(100, int(round(drv_right * 100))))
    spd   = max(0,    min(100, int(round(speed_scale * 100))))
    return encode_frame(TYPE_STATE, _FMT_STATE.pack(mode, drv_l, drv_r, flags, spd))


def decode_state(payload: bytes) -> dict:
    mode, drv_l, drv_r, flags, spd = _FMT_STATE.unpack(payload[:_FMT_STATE.size])
    return {
        "mode":        mode,
        "drv_left":    drv_l  / 100.0,
        "drv_right":   drv_r  / 100.0,
        "flags":       flags,
        "speed_scale": spd    / 100.0,
        # Convenience bool unpacking
        "rc_active":     bool(flags & SF_RC_ACTIVE),
        "lidar_ok":      bool(flags & SF_LIDAR_OK),
        "gps_ok":        bool(flags & SF_GPS_OK),
        "cam_ok":        bool(flags & SF_CAM_OK),
        "cam_recording": bool(flags & SF_CAM_RECORDING),
        "data_logging":  bool(flags & SF_DATA_LOGGING),
        "no_motors":     bool(flags & SF_NO_MOTORS),
        "bench_enabled": bool(flags & SF_BENCH_ENABLED),
        "front_cap":     bool(flags & SF_FRONT_CAP),
        "rear_cap":      bool(flags & SF_REAR_CAP),
    }


# TELEM — 16 bytes
# voltage:u16(mV) current:u16(mA) board_temp:i8 left_temp:i8 right_temp:i8
# faults:u8 heading:u16(0.1°) pitch:i16(0.1°) roll:i16(0.1°)
# applied_l:i8(×100) applied_r:i8(×100)  — firmware v5+; 0 when unavailable
_FMT_TELEM = struct.Struct('<HHbbbBHhhbb')

def encode_telem(voltage_v: float, current_a: float,
                 board_temp: float, left_temp: float, right_temp: float,
                 left_fault: bool, right_fault: bool,
                 heading: Optional[float], pitch: Optional[float],
                 roll: Optional[float],
                 applied_l: float = 0.0, applied_r: float = 0.0) -> bytes:
    v_mv  = max(0,      min(65534, int(round(voltage_v  * 1000))))
    i_ma  = max(0,      min(65534, int(round(current_a  * 1000))))
    bt    = max(-128,   min(127,   int(round(board_temp))))
    lt    = max(-128,   min(127,   int(round(left_temp))))
    rt    = max(-128,   min(127,   int(round(right_temp))))
    faults = (1 if left_fault else 0) | (2 if right_fault else 0)
    hdg   = NULL_U16 if heading is None else max(0,     min(3599, int(round(heading * 10))))
    pit   = NULL_I16 if pitch   is None else max(-1800, min(1800, int(round(pitch   * 10))))
    rol   = NULL_I16 if roll    is None else max(-1800, min(1800, int(round(roll    * 10))))
    al    = max(-100,  min(100,  int(round(applied_l * 100))))
    ar    = max(-100,  min(100,  int(round(applied_r * 100))))
    return encode_frame(TYPE_TELEM, _FMT_TELEM.pack(v_mv, i_ma, bt, lt, rt, faults, hdg, pit, rol, al, ar))


def decode_telem(payload: bytes) -> dict:
    v_mv, i_ma, bt, lt, rt, faults, hdg, pit, rol, al, ar = _FMT_TELEM.unpack(payload[:_FMT_TELEM.size])
    return {
        "voltage":     v_mv  / 1000.0,
        "current":     i_ma  / 1000.0,
        "board_temp":  float(bt),
        "left_temp":   float(lt),
        "right_temp":  float(rt),
        "left_fault":  bool(faults & 1),
        "right_fault": bool(faults & 2),
        "heading":     None if hdg == NULL_U16 else hdg / 10.0,
        "pitch":       None if pit == NULL_I16 else pit / 10.0,
        "roll":        None if rol == NULL_I16 else rol / 10.0,
        "applied_l":   al / 100.0,
        "applied_r":   ar / 100.0,
    }


# MOD_TELEM — 9 bytes
# fl_temp:i8 fr_temp:i8 rl_temp:i8 rr_temp:i8
# fl_curr:u8 fr_curr:u8 rl_curr:u8 rr_curr:u8  (|A| × 10, 0–25.5 A)
# faults:u8  bit0=FL, bit1=FR, bit2=RL, bit3=RR
_FMT_MOD_TELEM = struct.Struct('<bbbbBBBBB')

def encode_mod_telem(fl_temp: float, fr_temp: float,
                     rl_temp: float, rr_temp: float,
                     fl_curr: float, fr_curr: float,
                     rl_curr: float, rr_curr: float,
                     fl_fault: bool, fr_fault: bool,
                     rl_fault: bool, rr_fault: bool) -> bytes:
    def _t(v): return max(-128, min(127, int(round(v))))
    def _c(v): return max(0, min(255, int(round(abs(v) * 10))))
    faults = ((1 if fl_fault else 0) | (2 if fr_fault else 0) |
              (4 if rl_fault else 0) | (8 if rr_fault else 0))
    return encode_frame(TYPE_MOD_TELEM, _FMT_MOD_TELEM.pack(
        _t(fl_temp), _t(fr_temp), _t(rl_temp), _t(rr_temp),
        _c(fl_curr), _c(fr_curr), _c(rl_curr), _c(rr_curr),
        faults,
    ))


def decode_mod_telem(payload: bytes) -> dict:
    ft_fl, ft_fr, ft_rl, ft_rr, fc_fl, fc_fr, fc_rl, fc_rr, faults = (
        _FMT_MOD_TELEM.unpack(payload[:_FMT_MOD_TELEM.size]))
    return {
        "fl_temp":  float(ft_fl), "fr_temp":  float(ft_fr),
        "rl_temp":  float(ft_rl), "rr_temp":  float(ft_rr),
        "fl_current": fc_fl / 10.0, "fr_current": fc_fr / 10.0,
        "rl_current": fc_rl / 10.0, "rr_current": fc_rr / 10.0,
        "fl_fault": bool(faults & 1), "fr_fault": bool(faults & 2),
        "rl_fault": bool(faults & 4), "rr_fault": bool(faults & 8),
    }


# INA — 8 bytes
# voltage:u16(mV)  current:u16(mA)  power:u16(10mW)  die_temp:i8(°C)  flags:u8(bit0=ok)
_FMT_INA = struct.Struct('<HHHbB')

def encode_ina(voltage_v: float, current_a: float, power_w: float,
               die_temp: float, ok: bool) -> bytes:
    v  = max(0, min(65534, int(round(voltage_v * 1000))))
    i  = max(0, min(65534, int(round(current_a * 1000))))
    p  = max(0, min(65534, int(round(power_w   * 100))))
    t  = max(-128, min(127, int(round(die_temp))))
    f  = 0x01 if ok else 0x00
    return encode_frame(TYPE_INA, _FMT_INA.pack(v, i, p, t, f))


def decode_ina(payload: bytes) -> dict:
    v, i, p, t, f = _FMT_INA.unpack(payload[:_FMT_INA.size])
    ok = bool(f & 0x01)
    return {
        "ok":       ok,
        "voltage":  None if not ok else v / 1000.0,
        "current":  None if not ok else i / 1000.0,
        "power":    None if not ok else p / 100.0,
        "die_temp": None if not ok else float(t),
    }


# GPS — fixed header (19 bytes) + variable satellite records (5 bytes each, max 24)
# Fixed: lat:i32(1e-7°) lon:i32(1e-7°) alt:u16(dm) speed:u16(mm/s)
#        gps_hdg:u16(0.1°) fix:u8 herr:u16(mm) hdop:u8(0.1) sats:u8
# Satellite record: svid:u8 elev:u8(°) azim:u8(°/2→×2 on decode) snr:u8 sys:u8
#   sys: 0=GPS 1=GLO 2=GAL 3=BDS 4=QZSS
_FMT_GPS = struct.Struct('<iiHHHBHBB')
_FMT_SAT = struct.Struct('<BBBBB')   # svid, elev, azim_half, snr, sys
_SAT_MAX  = 24
_SYS_NAMES = ("GPS", "GLO", "GAL", "BDS", "QZSS")
_SYS_IDS   = {v: i for i, v in enumerate(_SYS_NAMES)}

def encode_gps(lat: Optional[float], lon: Optional[float],
               alt: Optional[float], speed: Optional[float],
               gps_hdg: Optional[float], fix: int,
               herr: Optional[float], hdop: Optional[float],
               sats: int,
               sat_data: Optional[list] = None) -> bytes:
    _lat   = 0         if lat     is None else max(-900000000, min(900000000, int(round(lat  * 1e7))))
    _lon   = 0         if lon     is None else max(-1800000000, min(1800000000, int(round(lon * 1e7))))
    _alt   = NULL_U16  if alt     is None else max(0, min(65534, int(round(alt * 10))))
    _spd   = NULL_U16  if speed   is None else max(0, min(65534, int(round(speed * 1000))))
    _hdg   = NULL_U16  if gps_hdg is None else max(0, min(3599, int(round(gps_hdg * 10))))
    _herr  = NULL_U16  if herr    is None else max(0, min(65534, int(round(herr  * 1000))))
    _hdop  = NULL_U8   if hdop    is None else max(0, min(254,   int(round(hdop  * 10))))
    _sats  = max(0, min(255, sats))
    payload = bytearray(_FMT_GPS.pack(_lat, _lon, _alt, _spd, _hdg, fix, _herr, _hdop, _sats))
    if sat_data:
        for sv in sat_data[:_SAT_MAX]:
            payload += _FMT_SAT.pack(
                max(0, min(255, int(sv.get("svid", 0)))),
                max(0, min(90,  int(sv.get("elev", 0)))),
                max(0, min(179, int(sv.get("azim", 0)) // 2)),
                max(0, min(255, int(sv.get("snr",  0)))),
                _SYS_IDS.get(sv.get("system", "GPS"), 0),
            )
    return encode_frame(TYPE_GPS, bytes(payload))


def decode_gps(payload: bytes) -> dict:
    lat, lon, alt, spd, hdg, fix, herr, hdop, sats = _FMT_GPS.unpack(payload[:_FMT_GPS.size])
    # Parse variable-length satellite records appended after fixed header
    sat_data = []
    offset = _FMT_GPS.size
    while offset + _FMT_SAT.size <= len(payload):
        svid, elev, azim_half, snr, sys_id = _FMT_SAT.unpack(payload[offset:offset + _FMT_SAT.size])
        sat_data.append({
            "svid":   svid,
            "elev":   elev,
            "azim":   azim_half * 2,
            "snr":    snr,
            "system": _SYS_NAMES[sys_id] if sys_id < len(_SYS_NAMES) else "GPS",
        })
        offset += _FMT_SAT.size
    return {
        "lat":      lat  / 1e7,
        "lon":      lon  / 1e7,
        "alt":      None if alt  == NULL_U16 else alt  / 10.0,
        "speed":    None if spd  == NULL_U16 else spd  / 1000.0,
        "hdg":      None if hdg  == NULL_U16 else hdg  / 10.0,
        "fix":      fix,
        "herr":     None if herr == NULL_U16 else herr / 1000.0,
        "hdop":     None if hdop == NULL_U8  else hdop / 10.0,
        "sats":     sats,
        "sat_data": sat_data,
    }


# SYS — 4 bytes
_FMT_SYS = struct.Struct('<BBBB')   # cpu%, mem%, disk%, pi_temp°C

def encode_sys(cpu: float, mem: float, disk: float, temp: float) -> bytes:
    return encode_frame(TYPE_SYS, _FMT_SYS.pack(
        max(0, min(100, int(round(cpu)))),
        max(0, min(100, int(round(mem)))),
        max(0, min(100, int(round(disk)))),
        max(0, min(255, int(round(temp)))),
    ))


def decode_sys(payload: bytes) -> dict:
    cpu, mem, disk, temp = _FMT_SYS.unpack(payload[:_FMT_SYS.size])
    return {"cpu": float(cpu), "mem": float(mem), "disk": float(disk), "temp": float(temp)}


# NAV — variable length
# Fixed header (14 bytes):
#   nav_state:u8 gate:u8 wp:u8 dist:u16(0.1m) bearing:u16(0.1°) bearing_err:i16(0.1°) tags:u8
#   outside_tag:u8 inside_tag:u8 next_outside_tag:u8 next_inside_tag:u8 next_gate:u8
# Variable tail (optional, for backward compat):
#   gate_label_len:u8  gate_label:N bytes  (UTF-8, max 31 chars, no NUL)
#   next_label_len:u8  next_label:M bytes  (UTF-8, max 31 chars, no NUL)
_FMT_NAV        = struct.Struct('<BBBHHhB')      # 10-byte legacy base (used for size sentinel)
_FMT_NAV_EXT    = struct.Struct('<BBBHHhBBBBBB') # 15-byte extended fixed header
_NAV_EXT_V1_SIZE = 14                            # v1 had no next_gate field
_NAV_LABEL_MAX = 31

def encode_nav(nav_state: int, gate: int, wp: int,
               dist: Optional[float], bearing: Optional[float],
               bearing_err: Optional[float], tags: int,
               outside_tag: int = 0xFF, inside_tag: int = 0xFF,
               next_outside_tag: int = 0xFF, next_inside_tag: int = 0xFF,
               gate_label: str = "", next_gate_label: str = "",
               next_gate: int = 0xFF) -> bytes:
    _dist = NULL_U16 if dist        is None else max(0,     min(65534, int(round(dist        * 10))))
    _bear = NULL_U16 if bearing     is None else max(0,     min(3599,  int(round(bearing     * 10))))
    _berr = NULL_I16 if bearing_err is None else max(-1800, min(1800,  int(round(bearing_err * 10))))
    fixed = _FMT_NAV_EXT.pack(
        nav_state & 0xFF, gate & 0xFF, wp & 0xFF, _dist, _bear, _berr, tags & 0xFF,
        outside_tag & 0xFF, inside_tag & 0xFF,
        next_outside_tag & 0xFF, next_inside_tag & 0xFF,
        next_gate & 0xFF,
    )
    def _lbl(s: str) -> bytes:
        enc = s.encode("utf-8")[:_NAV_LABEL_MAX]
        return bytes([len(enc)]) + enc
    return encode_frame(TYPE_NAV, fixed + _lbl(gate_label) + _lbl(next_gate_label))


def decode_nav(payload: bytes) -> dict:
    base_size = _FMT_NAV.size  # 10 bytes
    st, gate, wp, dist, bear, berr, tags = _FMT_NAV.unpack(payload[:base_size])
    # Extended fixed header (tag IDs + next_gate)
    if len(payload) >= _FMT_NAV_EXT.size:
        *_, ot, it, not_, nit, ng = _FMT_NAV_EXT.unpack(payload[:_FMT_NAV_EXT.size])
    elif len(payload) >= _NAV_EXT_V1_SIZE:
        # v1 extended header (no next_gate field)
        _, _, _, _, _, _, _, ot, it, not_, nit = struct.unpack('<BBBHHhBBBBB', payload[:_NAV_EXT_V1_SIZE])
        ng = 0xFF  # sentinel: use formula fallback
    else:
        ot = gate * 2;       it  = gate * 2 + 1
        not_ = (gate+1) * 2; nit = (gate+1) * 2 + 1
        ng = 0xFF
    # Sentinel 0xFF means "use formula fallback"
    if ot  == 0xFF: ot  = gate * 2
    if it  == 0xFF: it  = gate * 2 + 1
    if ng  == 0xFF: ng  = gate + 1
    if not_== 0xFF: not_= ng * 2
    if nit == 0xFF: nit = ng * 2 + 1
    # Variable-length labels
    def _read_lbl(buf: bytes, off: int) -> tuple:
        if off >= len(buf):
            return "", off
        n = buf[off]; off += 1
        label = buf[off:off + n].decode("utf-8", errors="replace")
        return label, off + n
    off = _FMT_NAV_EXT.size
    gate_lbl,  off = _read_lbl(payload, off)
    next_lbl,  _   = _read_lbl(payload, off)
    return {
        "nav_state":        st,
        "nav_state_name":   NAV_STATE_NAMES.get(st, "?"),
        "gate":             gate,
        "wp":               wp,
        "dist":             None if dist == NULL_U16 else dist / 10.0,
        "bearing":          None if bear == NULL_U16 else bear / 10.0,
        "bearing_err":      None if berr == NULL_I16 else berr / 10.0,
        "tags":             tags,
        "outside_tag":      ot,
        "inside_tag":       it,
        "next_gate":        ng,
        "next_outside_tag": not_,
        "next_inside_tag":  nit,
        "gate_label":       gate_lbl,
        "next_gate_label":  next_lbl,
    }


# LIDAR — variable, zlib compressed
# step:u8 followed by N uint16 distances (N = 360 / step)
# Angles are implicit: index i → angle = i * step degrees

def encode_lidar(distances: list[int], step: int) -> bytes:
    """Encode a LiDAR scan.

    *distances*  list of N uint16 distances in mm where N = 360 / step.
                 Index i corresponds to angle = i * step degrees.
                 0 means no return / out of range.
    *step*       degrees between samples (e.g. 5 → 72 samples).
    """
    n = 360 // step
    dists = distances[:n]
    while len(dists) < n:
        dists.append(0)
    payload = struct.pack('B', step) + struct.pack(f'<{n}H', *dists)
    return encode_frame(TYPE_LIDAR, payload, compress=True)


def decode_lidar(payload: bytes) -> dict:
    step = payload[0]
    n    = 360 // step
    dists = list(struct.unpack_from(f'<{n}H', payload, 1))
    angles = [i * step for i in range(n)]
    return {"step": step, "angles": angles, "distances": dists}


# ALARM — variable length
# alarm_id:u8 severity:u8 message:utf-8 bytes

def encode_alarm(alarm_id: int, severity: int, message: str) -> bytes:
    payload = struct.pack('BB', alarm_id, severity) + message.encode('utf-8')
    return encode_frame(TYPE_ALARM, payload)


def decode_alarm(payload: bytes) -> dict:
    alarm_id = payload[0]
    severity = payload[1]
    message  = payload[2:].decode('utf-8', errors='replace')
    return {
        "alarm_id":   alarm_id,
        "alarm_name": ALARM_NAMES.get(alarm_id, f"ALARM_{alarm_id:02X}"),
        "severity":   severity,
        "message":    message,
    }


# ══════════════════════════════════════════════════════════════════════════════
# Uplink encoders  (ground → robot)
# ══════════════════════════════════════════════════════════════════════════════

def encode_cmd(cmd_id: int, param: int = 0) -> bytes:
    return encode_frame(TYPE_CMD, struct.pack('BB', cmd_id, param))


def decode_cmd(payload: bytes) -> dict:
    cmd_id = payload[0]
    param  = payload[1] if len(payload) > 1 else 0
    return {
        "cmd_id":   cmd_id,
        "cmd_name": CMD_NAMES.get(cmd_id, f"CMD_{cmd_id:02X}"),
        "param":    param,
    }


def encode_rtcm(data: bytes) -> bytes:
    """Wrap raw RTCM3 bytes in a TYPE_RTCM frame (no compression — already binary)."""
    return encode_frame(TYPE_RTCM, data)


def decode_rtcm(payload: bytes) -> bytes:
    """Return raw RTCM3 bytes from a decoded TYPE_RTCM frame."""
    return payload


def encode_ping() -> bytes:
    return encode_frame(TYPE_PING, b'')


# TAGS — variable length (14 bytes per tag, max 16 tags across all cameras)
# tag_id:u8  cam_id:u8  cx:u16  cy:u16  dist:u16(mm,null=0xFFFF)  bearing:i16(0.1°,null=0x7FFF)  area:u32(px²)
_FMT_TAG  = struct.Struct('<BBHHHhI')
_TAGS_MAX = 16

def encode_tags(tags: list) -> bytes:
    """
    Encode visible ArUco tags from all cameras.

    Each tag dict must have:
      tag_id  : int
      cam_id  : int  (CAM_FRONT_LEFT=0, CAM_FRONT_RIGHT=1, CAM_REAR=2)
      cx      : int  (pixel centre x)
      cy      : int  (pixel centre y)
      distance: float | None  (metres)
      bearing : float | None  (degrees)
      area    : int           (bounding-box area in pixels²)
    """
    payload = bytearray()
    for t in tags[:_TAGS_MAX]:
        _dist = NULL_U16 if t.get("distance") is None else max(0, min(65534, int(round(t["distance"] * 1000))))
        _bear = NULL_I16 if t.get("bearing")  is None else max(-1800, min(1800, int(round(t["bearing"] * 10))))
        payload += _FMT_TAG.pack(
            t["tag_id"] & 0xFF,
            t["cam_id"] & 0xFF,
            max(0, min(65535, int(t.get("cx", 0)))),
            max(0, min(65535, int(t.get("cy", 0)))),
            _dist,
            _bear,
            max(0, min(0xFFFFFFFF, int(t.get("area", 0)))),
        )
    return encode_frame(TYPE_TAGS, bytes(payload))


def decode_tags(payload: bytes) -> list:
    """Return list of tag dicts, each with tag_id, cam_id, cam_name, cx, cy, distance, bearing."""
    tags = []
    offset = 0
    while offset + _FMT_TAG.size <= len(payload):
        tid, cam, cx, cy, dist, bear, area = _FMT_TAG.unpack(payload[offset:offset + _FMT_TAG.size])
        tags.append({
            "tag_id":   tid,
            "id":       tid,   # alias for HTML compatibility
            "cam_id":   cam,
            "cam_name": _CAM_NAMES.get(cam, "unknown"),
            "cx":       cx,
            "cy":       cy,
            "distance": None if dist == NULL_U16 else dist / 1000.0,
            "bearing":  None if bear == NULL_I16 else bear / 10.0,
            "area":     area,
        })
        offset += _FMT_TAG.size
    return tags


# ══════════════════════════════════════════════════════════════════════════════
# Convenience: build STATE flags from a RobotState
# ══════════════════════════════════════════════════════════════════════════════

def state_flags(robot_state) -> int:
    """Build the STATE flags uint16 from a RobotState dataclass instance."""
    s = robot_state
    flags = 0
    if getattr(s, 'rc_active',            False): flags |= SF_RC_ACTIVE
    if getattr(s, 'lidar_ok',             False): flags |= SF_LIDAR_OK
    if getattr(s, 'gps_ok',               False): flags |= SF_GPS_OK
    if getattr(s, 'camera_ok',            False): flags |= SF_CAM_OK
    if getattr(s, 'cam_recording',        False): flags |= SF_CAM_RECORDING
    if getattr(s, 'data_logging',         False): flags |= SF_DATA_LOGGING
    if getattr(s, 'no_motors',            False): flags |= SF_NO_MOTORS
    if getattr(s, 'bench_enabled',        False): flags |= SF_BENCH_ENABLED
    # Lens cap: either front camera triggers SF_FRONT_CAP
    front_cap = (getattr(s, 'cam_front_left_cap',  False) or
                 getattr(s, 'cam_front_right_cap', False))
    if front_cap:                                 flags |= SF_FRONT_CAP
    if getattr(s, 'cam_rear_cap',         False): flags |= SF_REAR_CAP
    # Per-camera OK (front-left is covered by SF_CAM_OK above)
    if getattr(s, 'cam_front_right_ok',   False): flags |= SF_CAM_FR_OK
    if getattr(s, 'cam_rear_ok',          False): flags |= SF_CAM_RE_OK
    # Auto-type: encode 2-bit value in bits 12-13
    # auto_type.name is one of CAMERA / GPS / CAMERA_GPS
    auto_type_name = getattr(getattr(s, 'auto_type', None), 'name', 'CAMERA')
    auto_type_bits = {'CAMERA': 0, 'GPS': 1, 'CAMERA_GPS': 2}.get(auto_type_name, 0)
    flags |= (auto_type_bits & 0x3) << 12
    return flags


# ══════════════════════════════════════════════════════════════════════════════
# Convenience: encode all downlink packets from a RobotState in one call
# ══════════════════════════════════════════════════════════════════════════════

def encode_all_downlink(robot_state, include_gps=True, include_nav=True,
                        include_sys=True) -> list[bytes]:
    """Return a list of framed packets to send for one telemetry tick.

    Caller is responsible for rate-limiting (only call include_gps=True every
    2nd 5 Hz tick, include_sys=True every 5th tick, etc.).
    """
    s   = robot_state
    t   = s.telemetry
    g   = s.gps
    sys = s.system

    from robot_daemon import RobotMode   # late import — not needed at ground station
    mode_map = {RobotMode.MANUAL: MODE_MANUAL,
                RobotMode.AUTO:   MODE_AUTO,
                RobotMode.ESTOP:  MODE_ESTOP}
    mode = mode_map.get(s.mode, MODE_MANUAL)

    packets = []

    # STATE
    packets.append(encode_state(
        mode         = mode,
        drv_left     = s.drive.left,
        drv_right    = s.drive.right,
        flags        = state_flags(s),
        speed_scale  = getattr(s, 'speed_scale', 1.0),
    ))

    # TELEM
    packets.append(encode_telem(
        voltage_v   = t.voltage,
        current_a   = t.current,
        board_temp  = t.board_temp,
        left_temp   = t.left_temp,
        right_temp  = t.right_temp,
        left_fault  = t.left_fault,
        right_fault = t.right_fault,
        heading     = t.heading,
        pitch       = t.pitch,
        roll        = t.roll,
    ))

    if include_gps:
        packets.append(encode_gps(
            lat     = g.latitude,
            lon     = g.longitude,
            alt     = g.altitude,
            speed   = g.speed,
            gps_hdg = g.heading,
            fix     = g.fix_quality,
            herr    = g.h_error_m,
            hdop    = g.hdop,
            sats    = g.satellites or 0,
        ))

    if include_sys:
        packets.append(encode_sys(
            cpu  = sys.cpu_percent,
            mem  = sys.mem_percent,
            disk = sys.disk_percent,
            temp = sys.cpu_temp_c,
        ))

    if include_nav:
        nav_state_map = {
            "IDLE":      NAV_IDLE,
            "SEARCHING": NAV_SEARCHING,
            "ALIGNING":  NAV_ALIGNING,
            "DRIVING":   NAV_DRIVING,
            "ARRIVED":   NAV_ARRIVED,
            "COMPLETE":  NAV_COMPLETE,
        }
        nav_st = nav_state_map.get(getattr(s, 'nav_state', 'IDLE'), NAV_IDLE)
        packets.append(encode_nav(
            nav_state   = nav_st,
            gate        = getattr(s, 'nav_gate', 0) or 0,
            wp          = getattr(s, 'nav_wp',   0) or 0,
            dist        = getattr(s, 'nav_wp_dist', None),
            bearing     = getattr(s, 'nav_wp_bear', None),
            bearing_err = getattr(s, 'nav_bearing_err', None),
            tags        = getattr(s, 'nav_tags_visible', 0) or 0,
        ))

    return packets


# ══════════════════════════════════════════════════════════════════════════════
# LiDAR helper: subsample a raw scan into a fixed-step distance array
# ══════════════════════════════════════════════════════════════════════════════

def lidar_to_step_array(scan, step: int) -> list[int]:
    """Convert a LidarScan dataclass to a fixed-step distance list.

    Returns a list of N uint16 distances (mm) where N = 360 // step.
    Index i → angle i * step degrees.  Nearest reading wins per bucket.
    """
    n = 360 // step
    out = [0] * n
    best = [float('inf')] * n

    if not (scan.angles and scan.distances):
        return out

    for a, d in zip(scan.angles, scan.distances):
        bucket = int(round(a)) % 360 // step
        if bucket < n and d < best[bucket]:
            best[bucket] = d
            out[bucket] = int(d)

    return out


# ══════════════════════════════════════════════════════════════════════════════
# Self-test
# ══════════════════════════════════════════════════════════════════════════════

if __name__ == '__main__':
    import sys

    ok = 0
    fail = 0

    def check(name, got, expect):
        global ok, fail
        if got == expect:
            print(f"  PASS  {name}")
            ok += 1
        else:
            print(f"  FAIL  {name}: got {got!r}, expected {expect!r}")
            fail += 1

    def check_close(name, got, expect, tol=0.01):
        global ok, fail
        if isinstance(got, dict) and isinstance(expect, dict):
            for k in expect:
                v_got = got.get(k)
                v_exp = expect[k]
                if v_exp is None:
                    if v_got is None:
                        continue
                    else:
                        print(f"  FAIL  {name}[{k}]: got {v_got!r}, expected None")
                        fail += 1
                        return
                if isinstance(v_exp, float):
                    if abs(v_got - v_exp) > tol:
                        print(f"  FAIL  {name}[{k}]: got {v_got!r}, expected {v_exp!r} (tol={tol})")
                        fail += 1
                        return
            print(f"  PASS  {name}")
            ok += 1
        else:
            check(name, got, expect)

    print("--- telemetry_proto self-test ---")

    # CRC sanity
    crc = _crc16(b'\x01\x00\x06\x00')
    check("crc16_nonzero", crc != 0, True)
    check("crc16_stable",  _crc16(b'\x01\x00\x06\x00'), crc)

    # Round-trip STATE
    frame = encode_state(MODE_AUTO, 0.75, -0.5, SF_RC_ACTIVE | SF_GPS_OK, 0.8)
    check("state_frame_starts_sync", frame[:2], bytes([SYNC_0, SYNC_1]))
    dec = FrameDecoder()
    frames = list(dec.feed(frame))
    check("state_one_frame", len(frames), 1)
    ptype, payload = frames[0]
    check("state_ptype", ptype, TYPE_STATE)
    d = decode_state(payload)
    check_close("state_drv_left",  {"drv_left":  d["drv_left"]},  {"drv_left":  0.75})
    check_close("state_drv_right", {"drv_right": d["drv_right"]}, {"drv_right": -0.5})
    check("state_mode", d["mode"], MODE_AUTO)
    check("state_flag_rc", d["rc_active"], True)
    check("state_flag_gps", d["gps_ok"], True)

    # Round-trip TELEM with nulls
    frame2 = encode_telem(12.34, 1.5, 38.0, 32.5, 31.0, False, True, None, 2.5, -1.5)
    ptype2, payload2 = list(FrameDecoder().feed(frame2))[0]
    check("telem_ptype", ptype2, TYPE_TELEM)
    d2 = decode_telem(payload2)
    check_close("telem_voltage", {"voltage": d2["voltage"]}, {"voltage": 12.34}, tol=0.002)
    check("telem_heading_null", d2["heading"], None)
    check_close("telem_pitch",  {"pitch": d2["pitch"]},  {"pitch": 2.5},  tol=0.05)
    check("telem_right_fault", d2["right_fault"], True)

    # Round-trip GPS with nulls
    frame3 = encode_gps(52.1234567, -0.9876543, 45.3, 1.5, None, 4, 0.025, 0.8, 12)
    ptype3, payload3 = list(FrameDecoder().feed(frame3))[0]
    check("gps_ptype", ptype3, TYPE_GPS)
    d3 = decode_gps(payload3)
    check_close("gps_lat", {"lat": d3["lat"]}, {"lat": 52.1234567}, tol=1e-5)
    check_close("gps_lon", {"lon": d3["lon"]}, {"lon": -0.9876543}, tol=1e-5)
    check("gps_hdg_null", d3["hdg"], None)
    check("gps_fix", d3["fix"], 4)

    # Round-trip SYS
    frame4 = encode_sys(45.2, 62.1, 18.5, 57.0)
    d4 = decode_sys(list(FrameDecoder().feed(frame4))[0][1])
    check_close("sys_cpu",  {"cpu":  d4["cpu"]},  {"cpu":  45.0}, tol=1.0)
    check_close("sys_temp", {"temp": d4["temp"]}, {"temp": 57.0}, tol=1.0)

    # Round-trip NAV with nulls
    frame5 = encode_nav(NAV_DRIVING, 2, 0, 15.5, 270.3, -5.2, 3)
    d5 = decode_nav(list(FrameDecoder().feed(frame5))[0][1])
    check("nav_state", d5["nav_state"], NAV_DRIVING)
    check_close("nav_dist", {"dist": d5["dist"]}, {"dist": 15.5}, tol=0.1)
    check_close("nav_berr", {"bearing_err": d5["bearing_err"]}, {"bearing_err": -5.2}, tol=0.1)

    # Round-trip LIDAR (compressed)
    # Use a realistic scan pattern (nearby wall on one side) which zlib compresses well
    dists_in = [800] * 20 + [1200] * 10 + [3000] * 22 + [1200] * 10 + [800] * 10
    frame6 = encode_lidar(dists_in, step=5)
    check("lidar_flag_zlib", frame6[3] & FLAG_ZLIB, FLAG_ZLIB)
    d6 = decode_lidar(list(FrameDecoder().feed(frame6))[0][1])
    check("lidar_step", d6["step"], 5)
    check("lidar_n_angles", len(d6["angles"]), 72)
    check("lidar_dists_match", d6["distances"], dists_in)

    # Round-trip ALARM
    frame7 = encode_alarm(ALARM_LOW_VOLTAGE, ALARM_SEV_WARNING, "12.1V < 12.5V threshold")
    d7 = decode_alarm(list(FrameDecoder().feed(frame7))[0][1])
    check("alarm_id",   d7["alarm_id"],   ALARM_LOW_VOLTAGE)
    check("alarm_name", d7["alarm_name"], "LOW_VOLTAGE")
    check("alarm_msg",  d7["message"],    "12.1V < 12.5V threshold")

    # Round-trip CMD
    frame8 = encode_cmd(CMD_SET_MODE, MODE_AUTO)
    d8 = decode_cmd(list(FrameDecoder().feed(frame8))[0][1])
    check("cmd_id",    d8["cmd_id"],   CMD_SET_MODE)
    check("cmd_param", d8["param"],    MODE_AUTO)
    check("cmd_name",  d8["cmd_name"], "SET_MODE")

    # Round-trip RTCM
    rtcm_bytes = bytes([0xD3, 0x00, 0x13, 0x3E, 0xD0]) * 10
    frame9 = encode_rtcm(rtcm_bytes)
    ptype9, payload9 = list(FrameDecoder().feed(frame9))[0]
    check("rtcm_ptype",   ptype9,            TYPE_RTCM)
    check("rtcm_payload", decode_rtcm(payload9), rtcm_bytes)

    # Frame decoder noise resilience
    noise  = bytes([0xFF, 0xAB, 0x00, 0x7E, 0x00])
    frame_a = encode_state(MODE_MANUAL, 0.0, 0.0, 0, 1.0)
    junk_stream = noise + frame_a + noise + frame_a
    recovered = list(FrameDecoder().feed(junk_stream))
    check("noise_resilience_count", len(recovered), 2)
    check("noise_resilience_type",  recovered[0][0], TYPE_STATE)

    # Bandwidth estimate
    all_frames = [
        encode_state(MODE_MANUAL, 0.0, 0.0, 0, 1.0),
        encode_telem(12.0, 1.0, 35.0, 30.0, 30.0, False, False, 180.0, 0.0, 0.0),
        encode_gps(52.0, 0.0, 10.0, 0.0, None, 1, None, 1.2, 8),
        encode_sys(20.0, 40.0, 15.0, 55.0),
        encode_nav(NAV_IDLE, 0, 0, None, None, None, 0),
        encode_lidar([500] * 72, step=5),
    ]
    total_bytes = sum(len(f) for f in all_frames)
    # STATE+TELEM at 5Hz, GPS+NAV at 2Hz, SYS+LIDAR at 1Hz
    bps = (len(all_frames[0]) + len(all_frames[1])) * 5 + \
          (len(all_frames[2]) + len(all_frames[4])) * 2 + \
          (len(all_frames[3]) + len(all_frames[5])) * 1
    print(f"\n  Bandwidth estimate: {bps} bytes/sec  "
          f"({bps/5760*100:.1f}% of 57600-baud budget)")
    print(f"  Per-packet sizes: "
          + ", ".join(f"{n}={len(f)}B" for n, f in zip(
              ['STATE','TELEM','GPS','SYS','NAV','LIDAR'], all_frames)))

    print(f"\n{'PASS' if fail == 0 else 'FAIL'}  {ok} passed, {fail} failed")
    sys.exit(0 if fail == 0 else 1)
