"""
UBloxBase — shared UBX binary protocol layer for all u-blox modules.

Inherited by UBlox7, UBloxM8P, and any future u-blox drivers.
Only the UBX config commands common to all u-blox generations live here;
model-specific additions (SBAS, DGNSS, etc.) stay in the subclass.
"""

import time
from .base import GNSSBase, _pack_u8, _pack_u16, _pack_u32, _fletcher8
from .nmea import _MSG_GROUP_NMEA

# ---------------------------------------------------------------------------
# UBX protocol constants
# ---------------------------------------------------------------------------

_UBX_SYNC1 = 0xB5
_UBX_SYNC2 = 0x62

_CLS_CFG = 0x06

_CFG_PRT  = 0x00   # Port / baud rate
_CFG_MSG  = 0x01   # Enable/disable messages
_CFG_RST  = 0x04   # Reset
_CFG_RATE = 0x08   # Measurement/navigation rate
_CFG_CFG  = 0x09   # Save / clear / load config
_CFG_SBAS = 0x16   # SBAS (u-blox 7 / M8)
_CFG_GNSS = 0x3E   # Constellation config (M8 and later)
_CFG_DGNSS = 0x70  # Differential GNSS / RTK mode (M8P)


class UBloxBase(GNSSBase):
    """
    Shared UBX binary protocol layer for u-blox GNSS modules.

    Provides configuration methods common to all u-blox generations.
    Subclasses override set_baud_rate (to set the correct inProtoMask)
    and add model-specific commands.
    """

    # -----------------------------------------------------------------------
    # Common UBX configuration
    # -----------------------------------------------------------------------

    def set_update_rate(self, hz=5):
        """Set navigation/NMEA output rate (1–10 Hz).  Sends UBX CFG-RATE."""
        if not (1 <= hz <= 10):
            raise ValueError("Rate must be 1-10 Hz")
        period_ms = 1000 // hz
        payload = (
            _pack_u16(period_ms)   # measRate — measurement period in ms
            + _pack_u16(1)         # navRate  — 1 solution per measurement
            + _pack_u16(1)         # timeRef  — 1 = GPS time
        )
        self._send_ubx(_CLS_CFG, _CFG_RATE, payload)

    def set_baud_rate(self, baud=115200):
        """
        Change UART1 baud rate.  Module switches immediately; update host port.
        Subclasses override this to set the correct inProtoMask for their model.
        """
        valid = (9600, 19200, 38400, 57600, 115200, 230400, 460800)
        if baud not in valid:
            raise ValueError("Baud must be one of {}".format(valid))
        self._send_cfg_prt(baud, in_proto=0x0007, out_proto=0x0003)

    def enable_nmea_sentence(self, nmea_sub_id, rate=1):
        """Enable/disable an NMEA sentence on UART1 and USB.  rate=0 disables."""
        # CFG-MSG: [class, id, rate × 6 ports: DDC UART1 UART2 USB SPI reserved]
        payload = bytes([_MSG_GROUP_NMEA, nmea_sub_id, 0, rate, 0, rate, 0, 0])
        self._send_ubx(_CLS_CFG, _CFG_MSG, payload)

    def disable_nmea_sentence(self, nmea_sub_id):
        self.enable_nmea_sentence(nmea_sub_id, 0)

    def save_config(self):
        """Persist configuration to BBR and Flash."""
        payload = (
            _pack_u32(0x00000000)   # clearMask
            + _pack_u32(0x00001F1F) # saveMask — all sections
            + _pack_u32(0x00000000) # loadMask
            + _pack_u8(0x03)        # deviceMask: BBR | Flash
        )
        self._send_ubx(_CLS_CFG, _CFG_CFG, payload)

    def cold_start(self):
        """Clear all satellite data and force a cold start."""
        payload = _pack_u16(0xFFFF) + _pack_u8(0x01) + _pack_u8(0x00)
        self._send_ubx(_CLS_CFG, _CFG_RST, payload)

    def hot_start(self):
        """Restart navigation using existing satellite data (fastest TTFF)."""
        payload = _pack_u16(0x0000) + _pack_u8(0x01) + _pack_u8(0x00)
        self._send_ubx(_CLS_CFG, _CFG_RST, payload)

    def warm_start(self):
        """Restart keeping almanac but discarding ephemeris."""
        payload = _pack_u16(0x0001) + _pack_u8(0x01) + _pack_u8(0x00)
        self._send_ubx(_CLS_CFG, _CFG_RST, payload)

    # -----------------------------------------------------------------------
    # Internal helpers
    # -----------------------------------------------------------------------

    def _send_cfg_prt(self, baud, in_proto, out_proto):
        """Send CFG-PRT for UART1 with the given baud rate and proto masks."""
        payload = (
            _pack_u8(0x01)          # portID = 1 (UART1)
            + _pack_u8(0x00)        # reserved
            + _pack_u16(0x0000)     # txReady = disabled
            + _pack_u32(0x000008C0) # mode: 8N1
            + _pack_u32(baud)       # baudRate
            + _pack_u16(in_proto)   # inProtoMask
            + _pack_u16(out_proto)  # outProtoMask
            + _pack_u16(0x0000)     # flags
            + _pack_u16(0x0000)     # reserved
        )
        self._send_ubx(_CLS_CFG, _CFG_PRT, payload)

    def _send_ubx(self, cls, msg_id, payload=b""):
        """Build and transmit a UBX binary frame."""
        length = len(payload)
        chk_data = bytes([cls, msg_id]) + _pack_u16(length) + payload
        ck_a, ck_b = _fletcher8(chk_data)
        frame = (
            bytes([_UBX_SYNC1, _UBX_SYNC2, cls, msg_id])
            + _pack_u16(length)
            + payload
            + bytes([ck_a, ck_b])
        )
        self._uart.write(frame)
        if self._debug:
            print("[{}] TX UBX:".format(self.__class__.__name__),
                  " ".join("{:02X}".format(b) for b in frame))
