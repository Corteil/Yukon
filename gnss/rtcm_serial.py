"""
RtcmSerial — feed RTCM3 corrections from a local serial port to a GNSS driver.

Works with any GNSS driver that implements gnss.send_rtcm(data).

Typical use: a local RTK base station (e.g. second TAU1308 or u-blox ZED-F9P)
connected via USB/UART instead of a network NTRIP caster.

Usage
-----
    from gnss.rtcm_serial import RtcmSerial

    rtcm = RtcmSerial('/dev/ttyUSB1', baud=115200)
    rtcm.start(gnss)      # gnss is any GNSSBase instance (the rover)

    print(rtcm.status)         # 'connected' / 'error' / 'disconnected'
    print(rtcm.bytes_received)
    rtcm.stop()
"""

import threading
import time

RTCM_DISCONNECTED = "disconnected"
RTCM_CONNECTED    = "connected"
RTCM_ERROR        = "error"


class RtcmSerial:
    """
    Read RTCM3 correction bytes from a serial port and forward them to a
    GNSS rover driver.  Runs in a background daemon thread.

    Parameters
    ----------
    port : str      Serial device (e.g. '/dev/ttyUSB1')
    baud : int      Baud rate (default 115200)
    debug : bool    Print byte counts as data arrives
    """

    def __init__(self, port, baud=115200, debug=False):
        self.port           = port
        self.baud           = baud
        self.debug          = debug

        self.status         = RTCM_DISCONNECTED
        self.bytes_received = 0
        self.last_error     = None

        self._gnss    = None
        self._running = False
        self._thread  = None

    # ------------------------------------------------------------------
    # Public API  (matches NTRIPClient interface)
    # ------------------------------------------------------------------

    def start(self, gnss):
        """Start the background forwarding thread, feeding RTCM to gnss."""
        self._gnss    = gnss
        self._running = True
        self._thread  = threading.Thread(
            target=self._run_loop, daemon=True, name="RTCM-serial"
        )
        self._thread.start()
        print("[RTCM] Background thread started → {} @ {} baud".format(
            self.port, self.baud))

    def stop(self):
        """Stop the background thread."""
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=3.0)
        self.status = RTCM_DISCONNECTED
        print("[RTCM] Stopped.")

    # ------------------------------------------------------------------
    # Background thread
    # ------------------------------------------------------------------

    def _run_loop(self):
        while self._running:
            try:
                self._open_and_forward()
            except Exception as e:
                self.last_error = str(e)
                self.status     = RTCM_ERROR
                print("[RTCM] Error:", e)

            if self._running:
                print("[RTCM] Reconnecting in 5 s...")
                self.status = RTCM_DISCONNECTED
                time.sleep(5.0)

    def _open_and_forward(self):
        import serial

        print("[RTCM] Opening {} @ {} baud".format(self.port, self.baud))
        with serial.Serial(self.port, self.baud, timeout=1.0) as ser:
            self.status     = RTCM_CONNECTED
            self.last_error = None
            print("[RTCM] Connected — forwarding corrections to rover")

            while self._running:
                data = ser.read(512)
                if not data:
                    continue
                self.bytes_received += len(data)
                if self._gnss:
                    self._gnss.send_rtcm(data)
                if self.debug:
                    print("[RTCM] {} bytes → rover (total {})".format(
                        len(data), self.bytes_received))

    def __repr__(self):
        return "RtcmSerial({}, status={}, rx={}B)".format(
            self.port, self.status, self.bytes_received)
