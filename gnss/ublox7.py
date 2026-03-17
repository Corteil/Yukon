"""
u-blox 7 GNSS driver.

Supports: NEO-7M, NEO-7N, MAX-7C, LEA-7x
UART: 9600 baud default
Accuracy: 2.5 m CEP standalone GPS, ~1 m with SBAS/DGPS
No RTK support — use UBloxM8P for RTK.

Inherits shared UBX protocol config from UBloxBase.
Inherits all NMEA parsing and public properties from GNSSBase.
"""

import time
from .ublox_base import UBloxBase, _CLS_CFG, _CFG_SBAS
from .nmea import (
    _NMEA_GGA, _NMEA_GLL, _NMEA_GSA, _NMEA_GST,
    _NMEA_GSV, _NMEA_RMC, _NMEA_VTG, _NMEA_ZDA,
)


class UBlox7(UBloxBase):
    """
    Driver for u-blox 7 series GNSS modules (NEO-7M, NEO-7N, MAX-7C, LEA-7x).

    Quick start:
        import serial
        from gnss.ublox7 import UBlox7, configure_rover
        from gnss.ntrip import NTRIPClient

        ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        gnss = UBlox7(ser)
        configure_rover(gnss)
        ser.baudrate = 115200
    """

    # u-blox 7 does not support RTK
    @property
    def has_rtk_fixed(self):
        return False

    @property
    def has_rtk_float(self):
        return False

    # Fall back to HDOP estimate when GST is unavailable
    @property
    def h_error_m(self):
        if self.std_lat is not None and self.std_lon is not None:
            return (self.std_lat ** 2 + self.std_lon ** 2) ** 0.5
        if self.hdop is not None:
            return self.hdop * 1.2   # rough 1-sigma from 2.5 m CEP spec
        return None

    def enable_sbas(self, enabled=True):
        """Enable SBAS (EGNOS / WAAS / MSAS) for DGPS-level accuracy (~1 m)."""
        mode = 0x01 if enabled else 0x00
        payload = (
            bytes([mode])
            + bytes([0x07])             # usage: ranging + corrections + integrity
            + bytes([3])                # maxSBAS
            + bytes([0x00])             # reserved
            + bytes([0x00] * 4)         # scanmode2 = auto
            + bytes([0x51, 0x00, 0x00, 0x00])  # scanmode1: PRN 120+133 (EGNOS)
        )
        self._send_ubx(_CLS_CFG, _CFG_SBAS, payload)


def configure_rover(gnss, update_hz=5, baud=115200, save=True):
    """
    Apply recommended configuration to a UBlox7 instance.

    1. Change UART1 baud to ``baud`` — set ser.baudrate after this returns.
    2. Set navigation rate to ``update_hz`` Hz.
    3. Enable NMEA: GGA, RMC, GSA, GST, GSV, ZDA.
    4. Disable NMEA: GLL, VTG.
    5. Enable SBAS (EGNOS/WAAS).
    6. Save to non-volatile memory.
    """
    print("[rover] Setting baud rate to {}".format(baud))
    gnss.set_baud_rate(baud)
    time.sleep(0.1)

    print("[rover] Setting update rate to {} Hz".format(update_hz))
    gnss.set_update_rate(update_hz)
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

    print("[rover] Enabling SBAS (EGNOS/WAAS)")
    gnss.enable_sbas(True)
    time.sleep(0.1)

    if save:
        print("[rover] Saving config to flash / BBR")
        gnss.save_config()

    print("[rover] Done. Set ser.baudrate = {} before next update()".format(baud))
