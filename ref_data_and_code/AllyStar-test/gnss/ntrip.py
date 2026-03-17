"""
NTRIP client — protocol-agnostic RTCM correction stream.

Works with any GNSS driver that implements:
    gnss.send_rtcm(data)   — forwards bytes to the module
    gnss.has_fix           — bool
    gnss.latitude          — float or None
    gnss.longitude         — float or None
    gnss.altitude          — float or None
"""

import time
from .nmea import _nmea_checksum, _safe_float

# ---------------------------------------------------------------------------
# Connection state constants
# ---------------------------------------------------------------------------

NTRIP_DISCONNECTED = "disconnected"
NTRIP_CONNECTING   = "connecting"
NTRIP_CONNECTED    = "connected"
NTRIP_ERROR        = "error"


# ---------------------------------------------------------------------------
# NTRIPClient
# ---------------------------------------------------------------------------

class NTRIPClient:
    """
    NTRIP v1/v2 client that pulls RTCM3 corrections from a caster and feeds
    them to a GNSS driver over UART.

    Runs in a background daemon thread so your main loop stays clean.
    Automatically reconnects on disconnect.
    Sends GGA back to caster at a configurable interval (required for VRS).

    Parameters
    ----------
    host : str          NTRIP caster hostname or IP
    port : int          TCP port (typically 2101)
    mountpoint : str    Stream mountpoint (e.g. 'NEAR', 'MSM7_GPS')
    username : str      NTRIP credentials (empty string if none required)
    password : str
    lat : float         Approximate rover latitude  (for VRS / caster GGA)
    lon : float         Approximate rover longitude
    height : float      Approximate height above MSL in metres
    reconnect_s : float Seconds to wait before reconnecting after a drop
    gga_interval_s : float  How often to send GGA back to caster (VRS, default 10 s)
    ntrip_version : int 1 or 2
    debug : bool

    Usage
    -----
        ntrip = NTRIPClient('ntrip.example.com', 2101, 'MOUNT1',
                            username='user', password='pass',
                            lat=51.5, lon=-0.1, height=42.0)
        ntrip.start(gnss)

        print(ntrip.status)         # 'connected' / 'connecting' / 'error'
        print(ntrip.bytes_received)
        ntrip.stop()
    """

    def __init__(self, host, port=2101, mountpoint="", username="", password="",
                 lat=0.0, lon=0.0, height=0.0,
                 reconnect_s=5.0, gga_interval_s=10.0, ntrip_version=1, debug=False):
        self.host           = host
        self.port           = port
        self.mountpoint     = mountpoint
        self.username       = username
        self.password       = password
        self.ntrip_version  = ntrip_version
        self.lat            = lat
        self.lon            = lon
        self.height         = height
        self.reconnect_s    = reconnect_s
        self.gga_interval_s = gga_interval_s
        self.debug          = debug

        self.status         = NTRIP_DISCONNECTED
        self.bytes_received = 0
        self.last_error     = None
        self.connected_at   = None

        self._gnss    = None
        self._sock    = None
        self._running = False
        self._thread  = None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def start(self, gnss):
        """Start the background correction thread, feeding RTCM to gnss."""
        import threading
        self._gnss    = gnss
        self._running = True
        self._thread  = threading.Thread(
            target=self._run_loop, daemon=True, name="NTRIP"
        )
        self._thread.start()
        print("[NTRIP] Background thread started -> {}:{}/{}".format(
            self.host, self.port, self.mountpoint))

    def stop(self):
        """Stop the background thread and close the socket."""
        self._running = False
        self._close_socket()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=3.0)
        self.status = NTRIP_DISCONNECTED
        print("[NTRIP] Stopped.")

    def update_position(self, lat, lon, height=0.0):
        """Update rover position sent back to the caster (VRS support).
        Safe to call from the main thread while NTRIP is running."""
        self.lat    = lat
        self.lon    = lon
        self.height = height

    @property
    def uptime_s(self):
        """Seconds since last successful connection, or None if not connected."""
        return time.time() - self.connected_at if self.connected_at else None

    # ------------------------------------------------------------------
    # Background thread
    # ------------------------------------------------------------------

    def _run_loop(self):
        while self._running:
            try:
                self._connect_and_stream()
            except Exception as e:
                self.last_error = str(e)
                self.status     = NTRIP_ERROR
                if self.debug:
                    print("[NTRIP] Error:", e)
                else:
                    print("[NTRIP] Connection lost:", e)

            if self._running:
                print("[NTRIP] Reconnecting in {}s...".format(self.reconnect_s))
                self.status = NTRIP_CONNECTING
                time.sleep(self.reconnect_s)

    def _connect_and_stream(self):
        import socket

        self.status = NTRIP_CONNECTING
        print("[NTRIP] Connecting to {}:{}/{}".format(
            self.host, self.port, self.mountpoint))

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.settimeout(15.0)
        self._sock.connect((self.host, self.port))

        self._sock.sendall(self._build_request().encode("ascii"))

        header = self._read_http_header()
        if self.debug:
            print("[NTRIP] Response:", header[:150])

        if "200 OK" not in header and "ICY 200 OK" not in header:
            raise ConnectionError("Rejected by caster: {}".format(header[:120]))

        self._sock.settimeout(10.0)
        self.status       = NTRIP_CONNECTED
        self.connected_at = time.time()
        self.last_error   = None
        print("[NTRIP] Connected OK - receiving RTCM corrections")

        self._send_gga_to_caster()
        last_gga = time.time()

        if getattr(self, "_header_remainder", b""):
            if self._gnss:
                self._gnss.send_rtcm(self._header_remainder)
            self.bytes_received += len(self._header_remainder)
            self._header_remainder = b""

        while self._running:
            try:
                data = self._sock.recv(4096)
            except socket.timeout:
                self._send_gga_to_caster()
                continue

            if not data:
                raise ConnectionError("Server closed the connection")

            self.bytes_received += len(data)

            if self._gnss:
                self._gnss.send_rtcm(data)

            now = time.time()
            if now - last_gga >= self.gga_interval_s:
                if self._gnss and self._gnss.has_fix:
                    self.update_position(
                        self._gnss.latitude,
                        self._gnss.longitude,
                        self._gnss.altitude or self.height,
                    )
                self._send_gga_to_caster()
                last_gga = now

            if self.debug:
                print("[NTRIP] {} RTCM bytes -> module (total {})".format(
                    len(data), self.bytes_received))

    def _build_request(self):
        import base64
        creds = base64.b64encode(
            "{}:{}".format(self.username, self.password).encode()
        ).decode()
        if self.ntrip_version == 1:
            return (
                "GET /{mp} HTTP/1.1\r\n"
                "Host: {host}\r\n"
                "User-Agent: NTRIP GNSS-Python/1.0\r\n"
                "Authorization: Basic {creds}\r\n"
                "Connection: close\r\n"
                "\r\n"
            ).format(mp=self.mountpoint, host=self.host, creds=creds)
        else:
            gga = self._build_gga_sentence()
            return (
                "GET /{mp} HTTP/1.0\r\n"
                "Host: {host}\r\n"
                "Ntrip-Version: Ntrip/2.0\r\n"
                "User-Agent: NTRIP GNSS-Python/2.0\r\n"
                "Authorization: Basic {creds}\r\n"
                "Ntrip-GGA: {gga}\r\n"
                "\r\n"
            ).format(mp=self.mountpoint, host=self.host, creds=creds, gga=gga)

    def _read_http_header(self):
        """Read HTTP/ICY response header, handling CRLF, LF-only, and headerless streams."""
        self._header_remainder = b""
        buf = b""
        while True:
            try:
                chunk = self._sock.recv(256)
            except Exception:
                break
            if not chunk:
                break
            buf += chunk
            if b"\r\n\r\n" in buf:
                idx = buf.index(b"\r\n\r\n") + 4
                self._header_remainder = buf[idx:]
                buf = buf[:idx]
                break
            if b"\n\n" in buf:
                idx = buf.index(b"\n\n") + 2
                self._header_remainder = buf[idx:]
                buf = buf[:idx]
                break
            # Caster streaming binary immediately with no HTTP header
            if len(buf) > 4096:
                self._header_remainder = buf
                return "ICY 200 OK"
        return buf.decode("ascii", "ignore")

    def _send_gga_to_caster(self):
        try:
            gga = self._build_gga_sentence() + "\r\n"
            self._sock.sendall(gga.encode("ascii"))
            if self.debug:
                print("[NTRIP] Sent GGA to caster:", gga.strip())
        except Exception as e:
            if self.debug:
                print("[NTRIP] GGA send failed:", e)

    def _build_gga_sentence(self):
        lat = abs(self.lat)
        lon = abs(self.lon)
        lat_deg = int(lat)
        lon_deg = int(lon)
        lat_str = "{:02d}{:09.6f}".format(lat_deg, (lat - lat_deg) * 60)
        lon_str = "{:03d}{:09.6f}".format(lon_deg, (lon - lon_deg) * 60)
        ns = "N" if self.lat >= 0 else "S"
        ew = "E" if self.lon >= 0 else "W"
        try:
            t = time.gmtime()
            utc = "{:02d}{:02d}{:06.3f}".format(t.tm_hour, t.tm_min, float(t.tm_sec))
        except Exception:
            utc = "000000.000"
        body = "GPGGA,{utc},{lat},{ns},{lon},{ew},1,08,1.0,{alt:.3f},M,0.0,M,,".format(
            utc=utc, lat=lat_str, ns=ns, lon=lon_str, ew=ew, alt=self.height)
        return "${}*{}".format(body, _nmea_checksum(body))

    def _close_socket(self):
        try:
            if self._sock:
                self._sock.close()
        except Exception:
            pass
        self._sock = None

    def __repr__(self):
        return "NTRIPClient({}:{}/{}, status={}, rx={}B)".format(
            self.host, self.port, self.mountpoint,
            self.status, self.bytes_received)


# ---------------------------------------------------------------------------
# Utility: fetch and list NTRIP source table
# ---------------------------------------------------------------------------

def fetch_ntrip_sourcetable(host, port=2101, timeout=10):
    """
    Fetch the NTRIP caster source table and return a list of available streams.

    Each stream is a dict with keys:
        mountpoint, identifier, format, format_detail, carrier,
        nav_system, network, country, lat, lon, nmea, solution
    """
    import socket

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(timeout)
    try:
        sock.connect((host, port))
        req = (
            "GET / HTTP/1.0\r\n"
            "Host: {}\r\n"
            "Ntrip-Version: Ntrip/2.0\r\n"
            "User-Agent: GNSS-Python/2.0\r\n"
            "\r\n"
        ).format(host)
        sock.sendall(req.encode())
        data = b""
        while True:
            chunk = sock.recv(4096)
            if not chunk:
                break
            data += chunk
    finally:
        sock.close()

    streams = []
    for line in data.decode("ascii", "ignore").splitlines():
        if not line.startswith("STR;"):
            continue
        parts = line.split(";")
        if len(parts) < 10:
            continue
        try:
            streams.append({
                "mountpoint":    parts[1],
                "identifier":    parts[2],
                "format":        parts[3],
                "format_detail": parts[4],
                "carrier":       parts[5],
                "nav_system":    parts[6],
                "network":       parts[7],
                "country":       parts[8],
                "lat":           _safe_float(parts[9]),
                "lon":           _safe_float(parts[10]) if len(parts) > 10 else None,
                "nmea":          parts[11] == "1" if len(parts) > 11 else False,
                "solution":      parts[12] if len(parts) > 12 else "",
            })
        except Exception:
            continue
    return streams
