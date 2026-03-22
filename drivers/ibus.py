"""
ibus.py — FlySky iBUS receiver protocol reader.

Protocol (115200 8N1, packet every ~7ms):
  Byte  0     : 0x20  (length/header)
  Byte  1     : 0x40  (command/header)
  Bytes 2..29 : 14 channels × 2 bytes, little-endian, range 1000-2000
  Bytes 30-31 : checksum = 0xFFFF - sum(bytes[0:30]), little-endian

Usage:
    with IBusReader('/dev/ttyS0') as ibus:
        while True:
            channels = ibus.read()
            if channels:
                print(channels)
"""

import struct
import serial

PACKET_LEN   = 32
HEADER       = b'\x20\x40'
NUM_CHANNELS = 14
CH_MIN       = 1000
CH_MAX       = 2000
CH_MID       = 1500


class IBusError(Exception):
    pass


class IBusReader:
    def __init__(self, port, timeout=1.0):
        self.ser = serial.Serial(port, baudrate=115200, bytesize=8,
                                 parity='N', stopbits=1, timeout=timeout,
                                 dsrdtr=False, rtscts=False)
        self._channels = [CH_MID] * NUM_CHANNELS
        self.packets_ok  = 0
        self.packets_bad = 0

    # ── context manager ──────────────────────────────────────────────────────

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    def close(self):
        if self.ser.is_open:
            self.ser.close()

    # ── public API ───────────────────────────────────────────────────────────

    @property
    def channels(self):
        """Last successfully decoded channel values (list of 14 ints, 1000-2000)."""
        return list(self._channels)

    def read(self):
        """
        Block until one valid iBUS packet is received.

        Returns a list of 14 channel values (ints, 1000-2000),
        or None on checksum error / timeout.
        Raises IBusError on serial fault.
        """
        try:
            packet = self._sync_and_read()
        except serial.SerialException as e:
            raise IBusError(f"Serial error: {e}") from e

        if packet is None:
            return None          # timeout

        if not self._verify(packet):
            self.packets_bad += 1
            return None

        self.packets_ok += 1
        self._channels = [
            struct.unpack_from('<H', packet, 2 + i * 2)[0]
            for i in range(NUM_CHANNELS)
        ]
        return list(self._channels)

    # ── internals ────────────────────────────────────────────────────────────

    def _sync_and_read(self):
        """
        Scan the byte stream for the 0x20 0x40 header, then read the
        remaining 30 bytes.  Returns the full 32-byte packet or None on timeout.
        """
        # Read until we see the first header byte
        while True:
            b = self.ser.read(1)
            if not b:
                return None        # timeout
            if b[0] != 0x20:
                continue

            # Peek at the next byte
            b2 = self.ser.read(1)
            if not b2:
                return None
            if b2[0] != 0x40:
                # Not a valid header — but b2 might be a 0x20 itself
                if b2[0] == 0x20:
                    # Peek one more
                    b3 = self.ser.read(1)
                    if not b3:
                        return None
                    if b3[0] == 0x40:
                        # Found header at b2/b3 position
                        rest = self.ser.read(PACKET_LEN - 2)
                        if len(rest) < PACKET_LEN - 2:
                            return None
                        return b2 + b3 + rest
                continue

            # Read the remaining 30 bytes
            rest = self.ser.read(PACKET_LEN - 2)
            if len(rest) < PACKET_LEN - 2:
                return None
            return b + b2 + rest

    @staticmethod
    def _verify(packet):
        """Return True if checksum is valid."""
        expected = 0xFFFF - sum(packet[:30])
        stored   = struct.unpack_from('<H', packet, 30)[0]
        return (expected & 0xFFFF) == stored
