"""
Allystar TAU1308 GNSS driver.

TAU1308 — single-band RTK module (GPS L1C/A, BDS B1I, GLONASS L1OF, Galileo E1)
UART: 115200 baud, 8N1
RTK accuracy: 1.0 cm + 1 ppm (H), 2.5 cm + 1 ppm (V)
Binary protocol: Allystar HD — sync bytes 0xF1 0xD9, Fletcher-8 checksum

Only the HD binary configuration commands live here.
All NMEA parsing and public properties are inherited from GNSSBase.
"""

import time
from .base import GNSSBase, _pack_u16, _pack_u32, _fletcher8
from .nmea import (
    _MSG_GROUP_NMEA, _NMEA_GGA, _NMEA_GLL, _NMEA_GSA,
    _NMEA_GST, _NMEA_GSV, _NMEA_RMC, _NMEA_VTG,
)

# ---------------------------------------------------------------------------
# Allystar HD binary protocol constants
# ---------------------------------------------------------------------------

_HD_SYNC1 = 0xF1
_HD_SYNC2 = 0xD9

# ---------------------------------------------------------------------------
# TAU1308 driver
# ---------------------------------------------------------------------------

class TAU1308(GNSSBase):
    """
    Driver for the Allystar TAU1308 single-band RTK GNSS module.

    Inherits all NMEA parsing and public properties from GNSSBase.
    Adds Allystar HD binary configuration commands.

    Quick start:
        import serial
        from gnss.tau1308 import TAU1308, configure_rover
        from gnss.ntrip import NTRIPClient

        ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        gnss = TAU1308(ser)
        configure_rover(gnss)

        ntrip = NTRIPClient('ntrip.example.com', 2101, 'MOUNT', ...)
        ntrip.start(gnss)

        while True:
            gnss.update()
            print(gnss.fix_quality_name, gnss.latitude, gnss.longitude)
    """

    # -----------------------------------------------------------------------
    # Configuration  (Allystar HD binary protocol)
    # -----------------------------------------------------------------------

    def set_update_rate(self, hz=5):
        """Set position update rate (1–10 Hz).  Rover recommended: 5 Hz."""
        if not (1 <= hz <= 10):
            raise ValueError("Rate must be 1-10 Hz")
        period_ms = 1000 // hz
        payload = _pack_u16(period_ms) + _pack_u16(1) + _pack_u16(1)
        self._send_binary(0x06, 0x08, payload)

    def set_baud_rate(self, baud=115200):
        """Change UART baud rate on the module.  Update host port to match."""
        valid = (9600, 38400, 115200, 230400, 460800)
        if baud not in valid:
            raise ValueError("Baud must be one of {}".format(valid))
        payload = (
            bytes([0x01, 0x00])
            + _pack_u16(0x0000)
            + _pack_u32(0x000008C0)
            + _pack_u32(baud)
            + _pack_u16(0x0007)
            + _pack_u16(0x0003)
            + _pack_u16(0x0000)
            + _pack_u16(0x0000)
        )
        self._send_binary(0x06, 0x00, payload)

    def enable_nmea_sentence(self, nmea_sub_id, rate=1):
        """Enable/disable an NMEA sentence.  rate=0 disables."""
        payload = bytes([_MSG_GROUP_NMEA, nmea_sub_id] + [rate] * 6)
        self._send_binary(0x06, 0x01, payload)

    def disable_nmea_sentence(self, nmea_sub_id):
        self.enable_nmea_sentence(nmea_sub_id, 0)

    def enable_gst(self):
        """Enable GST sentences for RTK position error statistics."""
        self.enable_nmea_sentence(_NMEA_GST, 1)

    def save_config(self):
        """Persist current configuration to non-volatile memory."""
        payload = (
            _pack_u32(0x00000000)
            + _pack_u32(0x00001F1F)
            + _pack_u32(0x00000000)
            + bytes([0x03])
        )
        self._send_binary(0x06, 0x09, payload)

    def cold_start(self):
        """Clear all satellite data and force a cold start."""
        payload = _pack_u16(0xFFFF) + bytes([0x01, 0x00])
        self._send_binary(0x06, 0x04, payload)

    def hot_start(self):
        """Restart navigation using existing satellite data."""
        payload = _pack_u16(0x0000) + bytes([0x01, 0x00])
        self._send_binary(0x06, 0x04, payload)

    # -----------------------------------------------------------------------
    # Allystar HD binary frame builder
    # -----------------------------------------------------------------------

    def _send_binary(self, cls, msg_id, payload=b""):
        """Build and transmit an Allystar HD binary frame."""
        length = len(payload)
        header = bytes([_HD_SYNC1, _HD_SYNC2, cls, msg_id]) + _pack_u16(length)
        body   = bytes([cls, msg_id]) + _pack_u16(length) + payload
        ck_a, ck_b = _fletcher8(body)
        frame = header + payload + bytes([ck_a, ck_b])
        self._uart.write(frame)
        if self._debug:
            print("[TAU1308] TX:", " ".join("{:02X}".format(b) for b in frame))


# ---------------------------------------------------------------------------
# Rover configuration helper
# ---------------------------------------------------------------------------

def configure_rover(gnss, update_hz=5, save=True):
    """
    Apply the recommended RTK rover configuration to a TAU1308 instance.

    - 5 Hz update rate
    - NMEA: GGA, RMC, GSA, GST, GSV enabled; VTG and GLL disabled
    - Saves to flash by default

    Parameters
    ----------
    gnss : TAU1308
    update_hz : int   1–10 Hz (default 5)
    save : bool       Save to non-volatile memory after configuring
    """
    print("[rover] Setting update rate to {} Hz".format(update_hz))
    gnss.set_update_rate(update_hz)
    time.sleep(0.1)

    print("[rover] Enabling NMEA: GGA RMC GSA GST GSV")
    gnss.enable_nmea_sentence(_NMEA_GGA, 1)
    gnss.enable_nmea_sentence(_NMEA_RMC, 1)
    gnss.enable_nmea_sentence(_NMEA_GSA, 1)
    gnss.enable_nmea_sentence(_NMEA_GST, 1)
    gnss.enable_nmea_sentence(_NMEA_GSV, 1)

    print("[rover] Disabling NMEA: VTG GLL")
    gnss.disable_nmea_sentence(_NMEA_VTG)
    gnss.disable_nmea_sentence(_NMEA_GLL)
    time.sleep(0.1)

    if save:
        print("[rover] Saving config to flash")
        gnss.save_config()

    print("[rover] Configuration complete.")
