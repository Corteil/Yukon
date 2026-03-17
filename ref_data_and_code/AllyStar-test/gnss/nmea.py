"""
NMEA 0183 protocol helpers and fix quality constants.
Shared by all GNSS drivers.
"""

# ---------------------------------------------------------------------------
# MicroPython compat
# ---------------------------------------------------------------------------

def _const(x):
    return x

try:
    from micropython import const
except ImportError:
    const = _const

# ---------------------------------------------------------------------------
# Fix quality constants  (GGA field 6)
# ---------------------------------------------------------------------------

FIX_INVALID    = const(0)
FIX_GPS        = const(1)   # Autonomous GNSS
FIX_DGPS       = const(2)   # SBAS / RTCM differential
FIX_PPS        = const(3)
FIX_RTK_FIXED  = const(4)   # RTK Fixed — centimetre accuracy
FIX_RTK_FLOAT  = const(5)   # RTK Float
FIX_ESTIMATED  = const(6)
FIX_MANUAL     = const(7)
FIX_SIMULATION = const(8)

FIX_QUALITY_NAMES = {
    0: "Invalid",
    1: "GPS",
    2: "DGPS",
    3: "PPS",
    4: "RTK Fixed",
    5: "RTK Float",
    6: "Estimated",
    7: "Manual",
    8: "Simulation",
}

# ---------------------------------------------------------------------------
# NMEA message class / sub-IDs
# Class 0xF0 is used by both Allystar HD and UBX CFG-MSG commands.
# ---------------------------------------------------------------------------

_MSG_GROUP_NMEA = const(0xF0)
_NMEA_GGA       = const(0x00)
_NMEA_GLL       = const(0x01)
_NMEA_GSA       = const(0x02)
_NMEA_GSV       = const(0x03)
_NMEA_RMC       = const(0x04)
_NMEA_VTG       = const(0x05)
_NMEA_GST       = const(0x07)
_NMEA_ZDA       = const(0x08)

# ---------------------------------------------------------------------------
# NMEA helper functions
# ---------------------------------------------------------------------------

def _parse_lat(val, hem):
    if not val:
        return None
    try:
        d = float(val)
        deg = int(d / 100)
        result = deg + (d - deg * 100) / 60.0
        return -result if hem == "S" else result
    except (ValueError, TypeError):
        return None


def _parse_lon(val, hem):
    if not val:
        return None
    try:
        d = float(val)
        deg = int(d / 100)
        result = deg + (d - deg * 100) / 60.0
        return -result if hem == "W" else result
    except (ValueError, TypeError):
        return None


def _safe_float(val):
    try:
        return float(val) if val else None
    except (ValueError, TypeError):
        return None


def _safe_int(val):
    try:
        return int(val) if val else None
    except (ValueError, TypeError):
        return None


def _nmea_checksum(sentence):
    chk = 0
    for ch in sentence:
        chk ^= ord(ch)
    return "{:02X}".format(chk)


def _verify_nmea_checksum(raw):
    try:
        if raw.startswith("$") and "*" in raw:
            body, chk = raw[1:].rsplit("*", 1)
            return _nmea_checksum(body) == chk.strip().upper()
    except Exception:
        pass
    return False
