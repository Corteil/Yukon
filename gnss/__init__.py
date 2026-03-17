"""
gnss — multi-driver GNSS library.

Supported hardware
------------------
TAU1308    Allystar single-band RTK (GPS/BDS/GLONASS/Galileo) — Allystar HD protocol
UBlox7     u-blox 7 series (NEO-7M, NEO-7N, MAX-7C)          — UBX protocol, no RTK
UBloxM8P   u-blox NEO-M8P RTK rover                           — UBX protocol, RTK

All drivers share the same public interface (GNSSBase) and work with
the same NTRIPClient for RTCM corrections.

Typical imports
---------------
    from gnss.tau1308  import TAU1308,   configure_rover
    from gnss.ublox7   import UBlox7,    configure_rover
    from gnss.ubloxm8p import UBloxM8P,  configure_rover
    from gnss.ntrip    import NTRIPClient
"""

from .nmea      import (FIX_INVALID, FIX_GPS, FIX_DGPS, FIX_PPS,
                         FIX_RTK_FIXED, FIX_RTK_FLOAT, FIX_ESTIMATED,
                         FIX_MANUAL, FIX_SIMULATION, FIX_QUALITY_NAMES)
from .base      import GNSSBase
from .ntrip       import NTRIPClient, fetch_ntrip_sourcetable
from .rtcm_serial import RtcmSerial
from .tau1308     import TAU1308
from .ublox7    import UBlox7
from .ubloxm8p  import UBloxM8P

__all__ = [
    "GNSSBase",
    "TAU1308", "UBlox7", "UBloxM8P",
    "NTRIPClient", "fetch_ntrip_sourcetable", "RtcmSerial",
    "FIX_INVALID", "FIX_GPS", "FIX_DGPS", "FIX_PPS",
    "FIX_RTK_FIXED", "FIX_RTK_FLOAT", "FIX_ESTIMATED",
    "FIX_MANUAL", "FIX_SIMULATION", "FIX_QUALITY_NAMES",
]
