"""
GNSSBase — shared GNSS state, NMEA parsing, and public interface.

Subclass this for each hardware driver.  Override only the binary
configuration methods (_send_binary / _send_ubx etc.) — all NMEA
parsing and public properties live here and are inherited for free.
"""

from .nmea import (
    FIX_INVALID, FIX_RTK_FIXED, FIX_RTK_FLOAT, FIX_QUALITY_NAMES,
    _parse_lat, _parse_lon, _safe_float, _safe_int,
    _verify_nmea_checksum,
)

# ---------------------------------------------------------------------------
# Binary utilities  (shared by Allystar HD and UBX protocols)
# ---------------------------------------------------------------------------

def _pack_u8(val):
    return bytes([val & 0xFF])


def _pack_u16(val):
    return bytes([val & 0xFF, (val >> 8) & 0xFF])


def _pack_u32(val):
    return bytes([
        val & 0xFF, (val >> 8) & 0xFF,
        (val >> 16) & 0xFF, (val >> 24) & 0xFF,
    ])


def _fletcher8(data):
    """Fletcher-8 checksum — identical algorithm used by Allystar HD and UBX."""
    ck_a = ck_b = 0
    for b in data:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return ck_a, ck_b


# ---------------------------------------------------------------------------
# GNSSBase
# ---------------------------------------------------------------------------

class GNSSBase:
    """
    Abstract base class for GNSS drivers.

    Handles all NMEA parsing and exposes a uniform public interface.
    Hardware-specific subclasses add only the binary configuration
    commands for their protocol (Allystar HD, UBX, etc.).

    Parameters
    ----------
    uart : serial-like object
        pyserial Serial (Python 3), machine.UART (MicroPython), busio.UART (CircuitPython).
    debug : bool
        Print raw NMEA lines and binary frames.
    validate_checksum : bool
        Reject sentences with bad NMEA checksums.
    """

    def __init__(self, uart, debug=False, validate_checksum=True):
        self._uart     = uart
        self._debug    = debug
        self._validate = validate_checksum

        # --- GGA ---
        self.latitude    = None
        self.longitude   = None
        self.altitude    = None
        self.fix_quality = FIX_INVALID
        self.satellites  = None
        self.hdop        = None
        self.geoid_sep   = None
        self.dgps_age    = None
        self.utc_time    = None

        # --- RMC ---
        self.speed_knots = None
        self.speed_ms    = None
        self.course      = None
        self.date        = None
        self.rmc_valid   = False

        # --- GSA ---
        self.fix_type = None
        self.pdop     = None
        self.vdop     = None
        self.sat_ids  = []

        # --- GST (position error estimates) ---
        self.rms_error = None
        self.std_lat   = None
        self.std_lon   = None
        self.std_alt   = None

        # --- GSV ---
        self.satellites_in_view = None
        self.satellites_data    = []   # list of {svid, elev, azim, snr, system}
        self._gsv_buffer        = {}   # keyed by talker prefix

        # --- ZDA ---
        self.utc_datetime = None

        # --- Antenna status ---
        self.antenna_status = "UNKNOWN"   # OK / OPEN / SHORT / UNKNOWN

        # Counters
        self._sentences_parsed = 0
        self._sentences_errors = 0
        self._rtcm_bytes_sent  = 0

    # -----------------------------------------------------------------------
    # Public interface
    # -----------------------------------------------------------------------

    def update(self, max_sentences=20):
        """Read and parse up to max_sentences NMEA lines.  Call in main loop."""
        parsed = 0
        for _ in range(max_sentences):
            line = self._readline()
            if not line:
                break
            if self._parse_sentence(line):
                parsed += 1
        return parsed

    @property
    def has_fix(self):
        return self.fix_quality > FIX_INVALID

    @property
    def has_rtk_fixed(self):
        return self.fix_quality == FIX_RTK_FIXED

    @property
    def has_rtk_float(self):
        return self.fix_quality == FIX_RTK_FLOAT

    @property
    def fix_quality_name(self):
        return FIX_QUALITY_NAMES.get(self.fix_quality, "Unknown")

    @property
    def position(self):
        return (self.latitude, self.longitude)

    @property
    def position_3d(self):
        return (self.latitude, self.longitude, self.altitude)

    @property
    def h_error_m(self):
        """Horizontal position error in metres from GST.  None if unavailable."""
        if self.std_lat is not None and self.std_lon is not None:
            return (self.std_lat ** 2 + self.std_lon ** 2) ** 0.5
        return None

    @property
    def stats(self):
        return {
            "parsed":     self._sentences_parsed,
            "errors":     self._sentences_errors,
            "rtcm_bytes": self._rtcm_bytes_sent,
        }

    def send_rtcm(self, data):
        """Forward raw RTCM correction bytes directly to the module."""
        if data:
            self._uart.write(data)
            self._rtcm_bytes_sent += len(data)
            if self._debug:
                print("[{}] RTCM {} bytes -> module".format(
                    self.__class__.__name__, len(data)))

    # -----------------------------------------------------------------------
    # NMEA parsing
    # -----------------------------------------------------------------------

    def _readline(self):
        try:
            if hasattr(self._uart, "readline"):
                raw = self._uart.readline()
            else:
                raw = b""
                while True:
                    b = self._uart.read(1)
                    if not b:
                        break
                    raw += b
                    if b == b"\n":
                        break
            if raw:
                line = raw.decode("ascii", "ignore").strip()
                if self._debug and line:
                    print("[{}]".format(self.__class__.__name__), line)
                return line
        except Exception as e:
            if self._debug:
                print("[{}] readline error:".format(self.__class__.__name__), e)
        return None

    def _parse_sentence(self, sentence):
        if not sentence.startswith("$"):
            return False
        if self._validate and not _verify_nmea_checksum(sentence):
            self._sentences_errors += 1
            return False
        if "*" in sentence:
            sentence = sentence[:sentence.rfind("*")]
        parts = sentence[1:].split(",")
        if not parts:
            return False
        suffix = parts[0].upper()[-3:] if len(parts[0]) >= 3 else ""
        handlers = {
            "GGA": self._parse_gga,
            "RMC": self._parse_rmc,
            "GSA": self._parse_gsa,
            "GSV": self._parse_gsv,
            "GST": self._parse_gst,
            "VTG": self._parse_vtg,
            "ZDA": self._parse_zda,
            "GLL": self._parse_gll,
            "TXT": self._parse_txt,
        }
        handler = handlers.get(suffix)
        if handler:
            try:
                handler(parts)
                self._sentences_parsed += 1
                return True
            except Exception as e:
                self._sentences_errors += 1
                if self._debug:
                    print("[{}] parse error:".format(self.__class__.__name__), e)
        return False

    def _parse_gga(self, p):
        if len(p) < 10:
            return
        self.utc_time    = p[1]
        self.latitude    = _parse_lat(p[2], p[3])
        self.longitude   = _parse_lon(p[4], p[5])
        self.fix_quality = _safe_int(p[6]) or FIX_INVALID
        self.satellites  = _safe_int(p[7])
        self.hdop        = _safe_float(p[8])
        self.altitude    = _safe_float(p[9])
        if len(p) > 11:
            self.geoid_sep = _safe_float(p[11])
        if len(p) > 13:
            self.dgps_age = _safe_float(p[13])

    def _parse_rmc(self, p):
        if len(p) < 9:
            return
        self.utc_time    = p[1]
        self.rmc_valid   = (p[2].upper() == "A")
        self.latitude    = _parse_lat(p[3], p[4])
        self.longitude   = _parse_lon(p[5], p[6])
        self.speed_knots = _safe_float(p[7])
        if self.speed_knots is not None:
            self.speed_ms = self.speed_knots * 0.514444
        self.course = _safe_float(p[8])
        if len(p) > 9:
            self.date = p[9]

    def _parse_gsa(self, p):
        if len(p) < 18:
            return
        self.fix_type = _safe_int(p[2])
        self.sat_ids  = [s for s in p[3:15] if s]
        self.pdop     = _safe_float(p[15])
        self.hdop     = _safe_float(p[16])
        self.vdop     = _safe_float(p[17])

    def _parse_gsv(self, p):
        if len(p) < 4:
            return
        self.satellites_in_view = _safe_int(p[3])
        talker = p[0][:2].upper()
        system = {"GP": "GPS", "GL": "GLO", "GA": "GAL",
                  "GB": "BDS", "BD": "BDS", "GN": "GNS"}.get(talker, talker)
        total_msgs = _safe_int(p[1]) or 1
        msg_num    = _safe_int(p[2]) or 1
        if msg_num == 1:
            self._gsv_buffer[system] = []
        sats = self._gsv_buffer.setdefault(system, [])
        i = 4
        while i + 3 < len(p):
            svid = _safe_int(p[i])
            elev = _safe_int(p[i + 1])
            azim = _safe_int(p[i + 2])
            snr_raw = p[i + 3].split(",")[0] if p[i + 3] else ""
            snr = _safe_int("".join(c for c in snr_raw if c.isdigit()))
            if svid is not None:
                sats.append({"svid": svid, "elev": elev or 0,
                              "azim": azim or 0, "snr": snr or 0,
                              "system": system})
            i += 4
        if msg_num == total_msgs:
            all_sats = []
            for v in self._gsv_buffer.values():
                all_sats.extend(v)
            self.satellites_data = all_sats

    def _parse_gst(self, p):
        if len(p) < 9:
            return
        self.rms_error = _safe_float(p[2])
        self.std_lat   = _safe_float(p[6])
        self.std_lon   = _safe_float(p[7])
        self.std_alt   = _safe_float(p[8])

    def _parse_vtg(self, p):
        if len(p) >= 8:
            self.course      = _safe_float(p[1])
            self.speed_knots = _safe_float(p[5])
            speed_kmh = _safe_float(p[7])
            if speed_kmh is not None:
                self.speed_ms = speed_kmh / 3.6

    def _parse_zda(self, p):
        if len(p) >= 5:
            self.utc_datetime = "{}/{}/{} {}".format(p[2], p[3], p[4], p[1])
            day, mon, yr = p[2], p[3], p[4]
            if day and mon and len(yr) >= 4:
                self.date = "{}{}{}".format(day.zfill(2), mon.zfill(2), yr[2:4])

    def _parse_gll(self, p):
        if len(p) >= 5:
            lat = _parse_lat(p[1], p[2])
            lon = _parse_lon(p[3], p[4])
            if lat is not None:
                self.latitude = lat
            if lon is not None:
                self.longitude = lon

    def _parse_txt(self, p):
        if len(p) >= 5:
            msg = p[4].upper()
            if "ANT_OK" in msg:
                self.antenna_status = "OK"
            elif "ANT_OPEN" in msg:
                self.antenna_status = "OPEN"
            elif "ANT_SHORT" in msg:
                self.antenna_status = "SHORT"

    def __repr__(self):
        return "{}(fix={}, lat={}, lon={}, alt={}m, sats={})".format(
            self.__class__.__name__,
            self.fix_quality_name, self.latitude, self.longitude,
            self.altitude, self.satellites)
