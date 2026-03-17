"""
u-blox NEO-M8P GNSS RTK driver.

Supports: NEO-M8P-0 (rover), NEO-M8P-2 (rover + base)
UART: 9600 baud default (configurable to 115200 via configure_rover)
Constellations: GPS + GLONASS (default) or GPS + BeiDou
RTK accuracy: 1 cm + 1 ppm (H), 2 cm + 1 ppm (V) with RTK Fixed
RTCM input: 3.x (types 1001–1013, 1071–1127)

Key difference from u-blox 7: full RTK support via RTCM 3.x corrections.
has_rtk_fixed / has_rtk_float work from GGA fix quality — no overrides needed.

Inherits shared UBX protocol config from UBloxBase.
Inherits all NMEA parsing and public properties from GNSSBase.
"""

import time
from .ublox_base import UBloxBase, _CLS_CFG, _CFG_DGNSS, _CFG_GNSS
from .base import _pack_u8, _pack_u16, _pack_u32
from .nmea import (
    _NMEA_GGA, _NMEA_GLL, _NMEA_GSA, _NMEA_GST,
    _NMEA_GSV, _NMEA_RMC, _NMEA_VTG, _NMEA_ZDA,
)

# inProtoMask for M8P rover: UBX(1) | NMEA(2) | RTCM3(0x20)
_IN_PROTO_M8P  = 0x0023
_OUT_PROTO_M8P = 0x0003   # UBX | NMEA


class UBloxM8P(UBloxBase):
    """
    Driver for the u-blox NEO-M8P RTK GNSS module.

    Rover mode: receives RTCM 3.x corrections via send_rtcm() and achieves
    RTK Fixed (fix quality 4) or RTK Float (fix quality 5).

    Quick start:
        import serial
        from gnss.ubloxm8p import UBloxM8P, configure_rover
        from gnss.ntrip import NTRIPClient

        ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        gnss = UBloxM8P(ser)
        configure_rover(gnss)
        ser.baudrate = 115200

        ntrip = NTRIPClient('ntrip.example.com', 2101, 'MOUNT', ...)
        ntrip.start(gnss)

        while True:
            gnss.update()
            print(gnss.fix_quality_name, gnss.latitude, gnss.longitude)
    """

    # has_rtk_fixed / has_rtk_float: inherited from GNSSBase — check fix_quality.
    # h_error_m: inherited from GNSSBase — uses GST std_lat/std_lon directly.
    # M8P reliably outputs $GPGST when GST is enabled, so no HDOP fallback needed.

    def set_baud_rate(self, baud=115200):
        """
        Change UART1 baud rate with RTCM3 input enabled.
        Module switches immediately; set ser.baudrate to match before next I/O.
        """
        valid = (9600, 19200, 38400, 57600, 115200, 230400, 460800)
        if baud not in valid:
            raise ValueError("Baud must be one of {}".format(valid))
        # inProtoMask includes RTCM3 (0x20) so the module accepts correction data
        self._send_cfg_prt(baud, in_proto=_IN_PROTO_M8P, out_proto=_OUT_PROTO_M8P)

    def set_dgnss_mode(self, mode=3):
        """
        Configure the differential / RTK operating mode (CFG-DGNSS).

        Parameters
        ----------
        mode : int
            2 = RTK Float only (carrier phase corrections, float solution)
            3 = RTK Fixed + Float (fixed when possible, float as fallback) ← default
        """
        if mode not in (2, 3):
            raise ValueError("mode must be 2 (float) or 3 (fixed+float)")
        payload = _pack_u8(mode) + bytes([0x00, 0x00, 0x00])   # mode + 3 reserved
        self._send_ubx(_CLS_CFG, _CFG_DGNSS, payload)

    def set_constellations(self, gps=True, glonass=True, beidou=False, galileo=False):
        """
        Configure which GNSS constellations are enabled (CFG-GNSS).

        M8P supports simultaneous GPS + one of GLONASS / BeiDou.
        Enabling both GLONASS and BeiDou together is not supported.

        Parameters
        ----------
        gps     : bool   GPS L1C/A — always recommended
        glonass : bool   GLONASS L1OF (default on)
        beidou  : bool   BeiDou B1I (mutually exclusive with GLONASS)
        galileo : bool   Galileo E1 (M8P firmware ≥3.01 only)
        """
        if glonass and beidou:
            raise ValueError("M8P cannot use GLONASS and BeiDou simultaneously")

        # Each block: gnssId(1) resTrkCh(1) maxTrkCh(1) reserved(1) flags(4)
        # flags bit 0 = enable; bits 24-31 = sigCfgMask (1 = L1)
        def _block(gnss_id, res_ch, max_ch, enabled):
            flags = (0x01 | 0x01000000) if enabled else 0x00
            return (
                _pack_u8(gnss_id)
                + _pack_u8(res_ch)
                + _pack_u8(max_ch)
                + _pack_u8(0x00)
                + _pack_u32(flags)
            )

        blocks = (
            _block(0, 8, 16, gps)      # GPS
            + _block(2, 4, 12, galileo)  # Galileo
            + _block(3, 8, 16, beidou)   # BeiDou
            + _block(6, 8, 14, glonass)  # GLONASS
        )
        num_blocks = 4
        header = (
            _pack_u8(0x00)         # msgVer
            + _pack_u8(0x00)       # numTrkChHw (read-only, set to 0)
            + _pack_u8(0x00)       # numTrkChUse (0 = auto)
            + _pack_u8(num_blocks) # numConfigBlocks
        )
        self._send_ubx(_CLS_CFG, _CFG_GNSS, header + blocks)


def configure_rover(gnss, update_hz=5, baud=115200, save=True):
    """
    Apply recommended RTK rover configuration to a UBloxM8P instance.

    1. Change UART1 baud to ``baud`` (RTCM3 input enabled in inProtoMask).
       Set ser.baudrate = baud before the next update() call.
    2. Set navigation rate to ``update_hz`` Hz.
    3. Set DGNSS mode 3 (RTK Fixed + Float).
    4. Enable NMEA: GGA, RMC, GSA, GST, GSV, ZDA.
    5. Disable NMEA: GLL, VTG.
    6. Save to non-volatile memory.

    Parameters
    ----------
    gnss : UBloxM8P
    update_hz : int   1–8 Hz recommended for RTK (default 5)
    baud : int        Target baud rate (default 115200)
    save : bool       Save to BBR / Flash
    """
    print("[rover] Setting baud rate to {} (RTCM3 input enabled)".format(baud))
    gnss.set_baud_rate(baud)
    time.sleep(0.1)

    print("[rover] Setting update rate to {} Hz".format(update_hz))
    gnss.set_update_rate(update_hz)
    time.sleep(0.1)

    print("[rover] Setting DGNSS mode 3 (RTK Fixed + Float)")
    gnss.set_dgnss_mode(mode=3)
    time.sleep(0.1)

    print("[rover] Enabling NMEA: GGA RMC GSA GST GSV ZDA")
    gnss.enable_nmea_sentence(_NMEA_GGA, 1)
    gnss.enable_nmea_sentence(_NMEA_RMC, 1)
    gnss.enable_nmea_sentence(_NMEA_GSA, 1)
    gnss.enable_nmea_sentence(_NMEA_GST, 1)
    gnss.enable_nmea_sentence(_NMEA_GSV, 1)
    gnss.enable_nmea_sentence(_NMEA_ZDA, 1)

    print("[rover] Disabling NMEA: GLL VTG")
    gnss.disable_nmea_sentence(_NMEA_GLL)
    gnss.disable_nmea_sentence(_NMEA_VTG)
    time.sleep(0.1)

    if save:
        print("[rover] Saving config to flash / BBR")
        gnss.save_config()

    print("[rover] Done. Set ser.baudrate = {} before next update()".format(baud))
    print("[rover] Feed RTCM 3.x corrections via send_rtcm() or NTRIPClient")
