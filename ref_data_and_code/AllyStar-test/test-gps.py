import sys
import time
import argparse
import configparser
import os
import serial
import pygame
from gnss.ntrip import NTRIPClient

# ---------------------------------------------------------------------------
# Config file  (config.ini next to this script, or --config path)
# ---------------------------------------------------------------------------

CONFIG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config.ini")

def _load_config(path):
    cfg = configparser.ConfigParser()
    cfg.read(path)
    return cfg

def _cfg_get(cfg, section, key, fallback):
    try:
        val = cfg.get(section, key)
        return val if val.strip() != "" else fallback
    except (configparser.NoSectionError, configparser.NoOptionError):
        return fallback

# ---------------------------------------------------------------------------
# CLI arguments  (override config file values)
# ---------------------------------------------------------------------------

parser = argparse.ArgumentParser(
    description="TAU1308 RTK rover with pygame GUI",
    formatter_class=argparse.RawDescriptionHelpFormatter,
    epilog="Settings are loaded from config.ini; CLI flags override them.",
)
parser.add_argument("--config",     default=CONFIG_PATH,  help="Path to config file (default: config.ini)")
parser.add_argument("--port",       default=None,          help="Serial port")
parser.add_argument("--baud",       default=None,          type=int)
parser.add_argument("--host",       default=None,          help="NTRIP caster hostname")
parser.add_argument("--ntrip-port", default=None,          type=int, dest="ntrip_port")
parser.add_argument("--mount",      default=None,          help="NTRIP mountpoint")
parser.add_argument("--user",       default=None,          help="NTRIP username")
parser.add_argument("--password",   default=None,          help="NTRIP password")
parser.add_argument("--lat",        default=None,          type=float)
parser.add_argument("--lon",        default=None,          type=float)
parser.add_argument("--height",     default=None,          type=float)
parser.add_argument("--hz",         default=None,          type=int,   help="Update rate Hz")
parser.add_argument("--no-config",  action="store_true",               help="Skip module configuration")
parser.add_argument("--debug",      action="store_true",               help="Debug NMEA/RTCM output")
_args = parser.parse_args()

cfg = _load_config(_args.config)

# Merge: CLI wins over config file, config file wins over built-in defaults
def _arg(cli_val, section, key, fallback, cast=str):
    if cli_val is not None:
        return cli_val
    raw = _cfg_get(cfg, section, key, None)
    if raw is not None:
        try:
            return cast(raw)
        except ValueError:
            pass
    return fallback

class args:
    port       = _arg(_args.port,       "gnss",  "port",      "/dev/ttyUSB0")
    baud       = _arg(_args.baud,       "gnss",  "baud",      115200,  int)
    hz         = _arg(_args.hz,         "gnss",  "hz",        5,       int)
    debug      = _args.debug or _cfg_get(cfg, "gnss", "debug", "false").lower() == "true"
    no_config  = _args.no_config or _cfg_get(cfg, "gnss", "no_config", "false").lower() == "true"
    host       = _arg(_args.host,       "ntrip", "host",      "")
    ntrip_port = _arg(_args.ntrip_port, "ntrip", "port",      2101,    int)
    mount      = _arg(_args.mount,      "ntrip", "mount",     "")
    user       = _arg(_args.user,       "ntrip", "user",      "")
    password   = _arg(_args.password,   "ntrip", "password",  "")
    lat           = _arg(_args.lat,        "ntrip", "lat",           0.0,  float)
    lon           = _arg(_args.lon,        "ntrip", "lon",           0.0,  float)
    height        = _arg(_args.height,     "ntrip", "height",        0.0,  float)
    ntrip_version = _arg(None,             "ntrip", "ntrip_version", 1,    int)

print("Config: {}".format(_args.config))

# ---------------------------------------------------------------------------
# Load GNSS driver  (set driver = tau1308 or ublox7 in config.ini [gnss])
# ---------------------------------------------------------------------------

_driver_name = _cfg_get(cfg, "gnss", "driver", "tau1308").lower()
if _driver_name == "ublox7":
    from gnss.ublox7 import UBlox7 as TAU1308, configure_rover
    print("Driver: UBlox7")
elif _driver_name == "ubloxm8p":
    from gnss.ubloxm8p import UBloxM8P as TAU1308, configure_rover
    print("Driver: UBloxM8P")
else:
    from gnss.tau1308 import TAU1308, configure_rover
    print("Driver: TAU1308")

# ---------------------------------------------------------------------------
# Open serial port
# ---------------------------------------------------------------------------

demo_mode = False

def _find_serial_port(preferred, baud):
    """Try preferred port first, then scan ttyUSB* and ttyACM* devices."""
    import glob
    candidates = [preferred] + sorted(
        p for p in glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
        if p != preferred
    )
    for port in candidates:
        try:
            s = serial.Serial(port, baud, timeout=1)
            print("Opened {} @ {} baud".format(port, baud))
            return s, port
        except Exception as e:
            print("Could not open {}: {}".format(port, e))
    return None, None

ser, active_port = _find_serial_port(args.port, args.baud)
if ser is None:
    print("[Demo mode] Using canned NMEA sentences\n")
    demo_mode = True
    active_port = args.port

    class _FakeUART:
        _lines = [
            b"$GNGGA,123519.00,5152.12345,N,00023.45678,W,5,12,0.6,42.3,M,47.0,M,1.0,0000*1A\r\n",
            b"$GNRMC,123519.00,A,5152.12345,N,00023.45678,W,0.02,88.0,120326,,,A*7C\r\n",
            b"$GNGST,123519.00,0.4,0.3,0.2,45.0,0.008,0.007,0.018*7A\r\n",
            b"$GNGSA,A,3,01,04,07,09,11,17,28,,,,,,,0.9,0.6,0.7*3A\r\n",
        ]
        _i = 0
        def readline(self):
            if self._i < len(self._lines):
                l = self._lines[self._i]; self._i += 1; return l
            return b""
        def write(self, d): pass
    ser = _FakeUART()

# ---------------------------------------------------------------------------
# Init TAU1308 driver
# ---------------------------------------------------------------------------

gnss = TAU1308(ser, debug=args.debug, validate_checksum=not demo_mode)

if not demo_mode and not args.no_config:
    configure_rover(gnss, update_hz=args.hz, save=True)
    time.sleep(0.5)

# Read startup sentences to capture $GNTXT antenna status (sent once at power-on)
print("Reading startup messages...")
_t0 = time.time()
while time.time() - _t0 < 2.0:
    gnss.update(max_sentences=10)
    if gnss.antenna_status != "UNKNOWN":
        break
    time.sleep(0.1)
print("Antenna: {}".format(gnss.antenna_status))

# ---------------------------------------------------------------------------
# Start NTRIP
# ---------------------------------------------------------------------------

ntrip = None
if args.host and args.mount:
    ntrip = NTRIPClient(
        host=args.host,
        port=args.ntrip_port,
        mountpoint=args.mount,
        username=args.user,
        password=args.password,
        lat=args.lat,
        lon=args.lon,
        height=args.height,
        reconnect_s=5.0,
        gga_interval_s=10.0,
        ntrip_version=args.ntrip_version,
        debug=args.debug,
    )
    ntrip.start(gnss)
    print("[NTRIP] Correction stream started")
elif args.host:
    print("WARNING: --mount not specified. NTRIP disabled.")

# ---------------------------------------------------------------------------
# Pygame GUI
# ---------------------------------------------------------------------------

pygame.init()

W, H = 1024, 768
screen = pygame.display.set_mode((W, H))
pygame.display.set_caption("RTK GNSS Monitor")
clock = pygame.time.Clock()

# Fonts
FONT_BIG   = pygame.font.SysFont("monospace", 28, bold=True)
FONT_MED   = pygame.font.SysFont("monospace", 20)
FONT_SMALL = pygame.font.SysFont("monospace", 15)

# Colors
C_BG        = (18,  18,  30)
C_PANEL     = (28,  28,  45)
C_BORDER    = (60,  60,  90)
C_WHITE     = (230, 230, 240)
C_GRAY      = (130, 130, 150)
C_GREEN     = ( 60, 220,  80)
C_YELLOW    = (240, 200,  40)
C_ORANGE    = (240, 140,  40)
C_RED       = (220,  60,  60)
C_CYAN      = ( 60, 200, 220)
C_PURPLE    = (160,  80, 220)

FIX_COLORS = {
    0: C_RED,
    1: C_YELLOW,
    2: C_ORANGE,
    3: C_ORANGE,
    4: C_GREEN,
    5: C_CYAN,
    6: C_GRAY,
    7: C_GRAY,
    8: C_GRAY,
}

# Track history (last N positions)
MAX_TRACK = 200
track = []  # list of (lat, lon)

# View toggle: False = track, True = scatter
show_scatter = False

# ---------------------------------------------------------------------------
# Drawing helpers
# ---------------------------------------------------------------------------

def draw_panel(surf, rect, title=None):
    pygame.draw.rect(surf, C_PANEL, rect, border_radius=8)
    pygame.draw.rect(surf, C_BORDER, rect, width=1, border_radius=8)
    if title:
        t = FONT_SMALL.render(title, True, C_GRAY)
        surf.blit(t, (rect.x + 10, rect.y + 6))


def draw_field(surf, label, value, x, y, color=C_WHITE, label_color=C_GRAY):
    lbl = FONT_SMALL.render(label, True, label_color)
    val = FONT_MED.render(str(value), True, color)
    surf.blit(lbl, (x, y))
    surf.blit(val, (x, y + 16))


def draw_track(surf, rect, points):
    """Draw position track scaled to bounding box."""
    pygame.draw.rect(surf, C_PANEL, rect, border_radius=8)
    pygame.draw.rect(surf, C_BORDER, rect, width=1, border_radius=8)
    label = FONT_SMALL.render("Position Track", True, C_GRAY)
    surf.blit(label, (rect.x + 10, rect.y + 6))

    if len(points) < 2:
        msg = FONT_SMALL.render("No track data", True, C_GRAY)
        surf.blit(msg, (rect.x + rect.width // 2 - 50, rect.y + rect.height // 2))
        return

    lats = [p[0] for p in points]
    lons = [p[1] for p in points]
    min_lat, max_lat = min(lats), max(lats)
    min_lon, max_lon = min(lons), max(lons)

    pad = 20
    bx = rect.x + pad
    by = rect.y + pad + 16
    bw = rect.width  - pad * 2
    bh = rect.height - pad * 2 - 16

    dlat = max_lat - min_lat or 1e-9
    dlon = max_lon - min_lon or 1e-9

    def to_screen(lat, lon):
        sx = bx + int((lon - min_lon) / dlon * bw)
        sy = by + int((max_lat - lat)  / dlat * bh)
        return (sx, sy)

    pts = [to_screen(p[0], p[1]) for p in points]
    if len(pts) >= 2:
        pygame.draw.lines(surf, C_CYAN, False, pts, 2)
    pygame.draw.circle(surf, C_GREEN, pts[-1], 5)


def draw_scatter(surf, rect, points):
    """Draw a position scatter plot in local East/North metres."""
    import math
    pygame.draw.rect(surf, C_PANEL, rect, border_radius=8)
    pygame.draw.rect(surf, C_BORDER, rect, width=1, border_radius=8)
    label = FONT_SMALL.render("Position Scatter", True, C_GRAY)
    surf.blit(label, (rect.x + 10, rect.y + 6))

    if len(points) < 2:
        msg = FONT_SMALL.render("No scatter data", True, C_GRAY)
        surf.blit(msg, (rect.x + rect.width // 2 - 50, rect.y + rect.height // 2))
        return

    lats = [p[0] for p in points]
    lons = [p[1] for p in points]
    mean_lat = sum(lats) / len(lats)
    mean_lon = sum(lons) / len(lons)
    cos_lat  = math.cos(math.radians(mean_lat))
    M_PER_DEG = 111111.0

    # Convert to metres East/North relative to mean
    east  = [(lon - mean_lon) * M_PER_DEG * cos_lat for lon in lons]
    north = [(lat - mean_lat) * M_PER_DEG            for lat in lats]

    max_r = max(max(abs(e) for e in east), max(abs(n) for n in north), 0.001)
    # Pad scale by 20 %
    scale = max_r * 1.2

    pad  = 24
    bx   = rect.x + pad
    by   = rect.y + pad + 16
    bw   = rect.width  - pad * 2
    bh   = rect.height - pad * 2 - 16
    cx   = bx + bw // 2
    cy   = by + bh // 2

    # Crosshair
    pygame.draw.line(surf, C_BORDER, (bx, cy), (bx + bw, cy), 1)
    pygame.draw.line(surf, C_BORDER, (cx, by), (cx, by + bh), 1)

    def to_px(e, n):
        sx = cx + int(e / scale * (bw // 2))
        sy = cy - int(n / scale * (bh // 2))
        return (sx, sy)

    # Draw all points; newest is brightest
    n_pts = len(points)
    for i, (e, n) in enumerate(zip(east, north)):
        brightness = int(80 + 175 * i / max(n_pts - 1, 1))
        color = (0, brightness // 2, brightness)
        pygame.draw.circle(surf, color, to_px(e, n), 2)

    # Highlight newest point
    pygame.draw.circle(surf, C_GREEN, to_px(east[-1], north[-1]), 4)

    # RMS
    rms = math.sqrt(sum(e**2 + n**2 for e, n in zip(east, north)) / len(east))
    rms_str = "RMS {:.4f} m  N={}".format(rms, len(points))
    rms_surf = FONT_SMALL.render(rms_str, True, C_CYAN)
    surf.blit(rms_surf, (rect.x + 10, rect.y + rect.height - 18))

    # Scale label
    scale_str = "±{:.4f} m".format(scale)
    sc_surf = FONT_SMALL.render(scale_str, True, C_GRAY)
    surf.blit(sc_surf, (rect.x + rect.width - sc_surf.get_width() - 6,
                        rect.y + rect.height - 18))


def draw_skymap(surf, rect, sats):
    """Draw a polar sky map of satellite positions."""
    import math
    draw_panel(surf, rect, "Sky Map")

    cx = rect.x + rect.width  // 2
    cy = rect.y + rect.height // 2 + 8
    r  = min(rect.width, rect.height) // 2 - 22

    # Elevation rings: 0°, 30°, 60° (outer=horizon, inner=zenith)
    for elev_ring in (0, 30, 60):
        ring_r = int(r * (90 - elev_ring) / 90)
        pygame.draw.circle(surf, C_BORDER, (cx, cy), ring_r, 1)
        lbl = FONT_SMALL.render("{}°".format(elev_ring), True, C_BORDER)
        surf.blit(lbl, (cx + ring_r + 2, cy - 8))

    # Cardinal lines
    pygame.draw.line(surf, C_BORDER, (cx, cy - r), (cx, cy + r), 1)
    pygame.draw.line(surf, C_BORDER, (cx - r, cy), (cx + r, cy), 1)

    # Cardinal labels
    for label, dx, dy in (("N", 0, -r-14), ("S", 0, r+2),
                           ("E", r+2, 0),  ("W", -r-14, 0)):
        t = FONT_SMALL.render(label, True, C_GRAY)
        surf.blit(t, (cx + dx, cy + dy))

    # Satellite colours by system
    SYS_COLORS = {"GPS": C_GREEN, "GLO": C_CYAN, "GAL": C_YELLOW,
                  "BDS": C_ORANGE, "GNS": C_WHITE}

    for sat in sats:
        elev = sat.get("elev", 0)
        azim = sat.get("azim", 0)
        snr  = sat.get("snr",  0)
        svid = sat.get("svid", 0)
        sys  = sat.get("system", "GPS")

        sat_r = int(r * (90 - elev) / 90)
        rad   = math.radians(azim)
        sx    = cx + int(sat_r * math.sin(rad))
        sy    = cy - int(sat_r * math.cos(rad))

        color = SYS_COLORS.get(sys, C_WHITE)
        dot_r = 6 if snr >= 30 else (4 if snr >= 15 else 3)
        alpha = 255 if snr >= 15 else 120

        # Dim dot for weak signals
        draw_color = color if snr >= 15 else C_GRAY
        pygame.draw.circle(surf, draw_color, (sx, sy), dot_r)
        if snr >= 15:
            pygame.draw.circle(surf, C_BG, (sx, sy), dot_r - 2)
            pygame.draw.circle(surf, draw_color, (sx, sy), dot_r)

        lbl = FONT_SMALL.render(str(svid), True, draw_color)
        surf.blit(lbl, (sx + dot_r + 1, sy - 6))

    # Legend
    ly = rect.y + rect.height - 18
    lx = rect.x + 8
    for sys, col in SYS_COLORS.items():
        t = FONT_SMALL.render(sys, True, col)
        surf.blit(t, (lx, ly))
        lx += t.get_width() + 10


def draw_signal_bars(surf, rect, sats):
    """Draw SNR bar chart for all satellites."""
    draw_panel(surf, rect, "Signal Strength (SNR dB)")
    if not sats:
        msg = FONT_SMALL.render("No satellite data", True, C_GRAY)
        surf.blit(msg, (rect.x + 10, rect.y + rect.height // 2))
        return

    SYS_COLORS = {"GPS": C_GREEN, "GLO": C_CYAN, "GAL": C_YELLOW,
                  "BDS": C_ORANGE, "GNS": C_WHITE}
    MAX_SNR  = 50
    pad_x    = 8
    pad_top  = 22
    pad_bot  = 28
    bar_area_h = rect.height - pad_top - pad_bot
    avail_w    = rect.width - pad_x * 2
    n          = len(sats)
    bar_w      = max(4, min(20, avail_w // max(n, 1) - 2))
    spacing    = avail_w // max(n, 1)

    # SNR = 0 reference line
    base_y = rect.y + pad_top + bar_area_h
    pygame.draw.line(surf, C_BORDER,
                     (rect.x + pad_x, base_y),
                     (rect.x + rect.width - pad_x, base_y), 1)

    # Guide lines at 20 and 40 dB
    for snr_guide in (20, 40):
        gy = base_y - int(bar_area_h * snr_guide / MAX_SNR)
        pygame.draw.line(surf, C_BORDER,
                         (rect.x + pad_x, gy),
                         (rect.x + rect.width - pad_x, gy), 1)
        gl = FONT_SMALL.render(str(snr_guide), True, C_BORDER)
        surf.blit(gl, (rect.x + pad_x, gy - 12))

    for i, sat in enumerate(sats):
        snr  = sat.get("snr", 0) or 0
        svid = sat.get("svid", 0)
        sys  = sat.get("system", "GPS")
        color = SYS_COLORS.get(sys, C_WHITE)

        bx = rect.x + pad_x + i * spacing + (spacing - bar_w) // 2
        bh = int(bar_area_h * min(snr, MAX_SNR) / MAX_SNR)
        by = base_y - bh

        if bh > 0:
            pygame.draw.rect(surf, color, (bx, by, bar_w, bh))

        # PRN label below bar
        lbl = FONT_SMALL.render(str(svid), True, C_GRAY)
        surf.blit(lbl, (bx + bar_w // 2 - lbl.get_width() // 2,
                        base_y + 2))

        # SNR value above bar
        if snr > 0:
            snr_lbl = FONT_SMALL.render(str(snr), True, color)
            surf.blit(snr_lbl, (bx + bar_w // 2 - snr_lbl.get_width() // 2,
                                max(by - 14, rect.y + pad_top)))


def sat_bar(surf, x, y, count, max_count=20):
    """Draw a simple satellite count bar."""
    bar_w = 120
    bar_h = 12
    pygame.draw.rect(surf, C_BORDER, (x, y, bar_w, bar_h), border_radius=3)
    fill = int(bar_w * min(count or 0, max_count) / max_count)
    if fill > 0:
        pygame.draw.rect(surf, C_CYAN, (x, y, fill, bar_h), border_radius=3)


# ---------------------------------------------------------------------------
# Config menu
# ---------------------------------------------------------------------------

class ConfigMenu:
    # (section, key, display_label, type)
    # type: "str" | "int" | "float" | "bool" | "password" | "choice:<a,b,c>"
    FIELDS = [
        ("gnss",  "driver",        "Driver",        "choice:tau1308,ublox7,ubloxm8p"),
        ("gnss",  "port",          "Serial Port",   "str"),
        ("gnss",  "baud",          "Baud Rate",     "int"),
        ("gnss",  "hz",            "Update Hz",     "int"),
        ("gnss",  "no_config",     "Skip Config",   "bool"),
        ("gnss",  "debug",         "Debug Output",  "bool"),
        ("ntrip", "host",          "Host",          "str"),
        ("ntrip", "port",          "Port",          "int"),
        ("ntrip", "mount",         "Mountpoint",    "str"),
        ("ntrip", "user",          "Username",      "str"),
        ("ntrip", "password",      "Password",      "str"),
        ("ntrip", "lat",           "Latitude",      "float"),
        ("ntrip", "lon",           "Longitude",     "float"),
        ("ntrip", "height",        "Height (m)",    "float"),
        ("ntrip", "ntrip_version", "NTRIP Version", "int"),
    ]

    _DEFAULTS = {
        ("gnss",  "driver"):        "tau1308",
        ("gnss",  "port"):          "/dev/ttyUSB0",
        ("gnss",  "baud"):          "115200",
        ("gnss",  "hz"):            "5",
        ("gnss",  "no_config"):     "false",
        ("gnss",  "debug"):         "false",
        ("ntrip", "host"):          "",
        ("ntrip", "port"):          "2101",
        ("ntrip", "mount"):         "",
        ("ntrip", "user"):          "",
        ("ntrip", "password"):      "",
        ("ntrip", "lat"):           "0.0",
        ("ntrip", "lon"):           "0.0",
        ("ntrip", "height"):        "0.0",
        ("ntrip", "ntrip_version"): "1",
    }

    def __init__(self, config_path):
        self.config_path  = config_path
        self.active       = False
        self.selected     = 0
        self.editing      = False
        self.edit_buffer  = ""
        self.edit_cursor  = 0
        self._blink_timer = 0   # incremented each draw; cursor visible when < BLINK_ON
        self.values       = {}   # {(section, key): str}
        self._scroll      = 0    # index of first visible field
        self._save_timer  = 0    # frames to show "Saved" message

    # ------------------------------------------------------------------

    def open(self):
        self._reload()
        self.active  = True
        self.editing = False
        self._clamp_scroll()

    def close(self):
        self.active  = False
        self.editing = False

    def _reload(self):
        cfg = _load_config(self.config_path)
        for sec, key, _label, _ftype in self.FIELDS:
            self.values[(sec, key)] = _cfg_get(
                cfg, sec, key, self._DEFAULTS.get((sec, key), ""))

    def save(self):
        cfg = configparser.ConfigParser()
        cfg.read(self.config_path)
        for (sec, key), val in self.values.items():
            if not cfg.has_section(sec):
                cfg.add_section(sec)
            cfg.set(sec, key, val)
        with open(self.config_path, "w") as f:
            cfg.write(f)
        self._save_timer = 150

    # ------------------------------------------------------------------

    def handle_event(self, event):
        """Process a pygame event.  Returns True if consumed."""
        if not self.active:
            return False
        if event.type != pygame.KEYDOWN:
            return True   # swallow mouse etc. while menu is open

        if self.editing:
            if event.key == pygame.K_RETURN:
                sec, key = self.FIELDS[self.selected][:2]
                self.values[(sec, key)] = self.edit_buffer
                self.editing = False
            elif event.key == pygame.K_ESCAPE:
                self.editing = False
            elif event.key == pygame.K_LEFT:
                self.edit_cursor = max(0, self.edit_cursor - 1)
                self._blink_timer = 0
            elif event.key == pygame.K_RIGHT:
                self.edit_cursor = min(len(self.edit_buffer), self.edit_cursor + 1)
            elif event.key == pygame.K_HOME:
                self.edit_cursor = 0
            elif event.key == pygame.K_END:
                self.edit_cursor = len(self.edit_buffer)
            elif event.key == pygame.K_BACKSPACE:
                if self.edit_cursor > 0:
                    self.edit_buffer = (self.edit_buffer[:self.edit_cursor - 1]
                                        + self.edit_buffer[self.edit_cursor:])
                    self.edit_cursor -= 1
            elif event.key == pygame.K_DELETE:
                self.edit_buffer = (self.edit_buffer[:self.edit_cursor]
                                    + self.edit_buffer[self.edit_cursor + 1:])
            elif event.unicode and event.unicode.isprintable():
                self.edit_buffer = (self.edit_buffer[:self.edit_cursor]
                                    + event.unicode
                                    + self.edit_buffer[self.edit_cursor:])
                self.edit_cursor += 1
        else:
            if event.key == pygame.K_ESCAPE:
                self.close()
            elif event.key == pygame.K_UP:
                self.selected = (self.selected - 1) % len(self.FIELDS)
                self._clamp_scroll()
            elif event.key == pygame.K_DOWN:
                self.selected = (self.selected + 1) % len(self.FIELDS)
                self._clamp_scroll()
            elif event.key in (pygame.K_RETURN, pygame.K_SPACE):
                self._activate_field()
            elif event.key == pygame.K_s:
                self.save()
        return True

    def _activate_field(self):
        sec, key, _label, ftype = self.FIELDS[self.selected]
        if ftype == "bool":
            cur = self.values.get((sec, key), "false").lower()
            self.values[(sec, key)] = "false" if cur == "true" else "true"
        elif ftype.startswith("choice:"):
            choices = ftype.split(":", 1)[1].split(",")
            cur = self.values.get((sec, key), choices[0])
            idx = choices.index(cur) if cur in choices else 0
            self.values[(sec, key)] = choices[(idx + 1) % len(choices)]
        else:
            self.edit_buffer  = self.values.get((sec, key), "")
            self.edit_cursor  = len(self.edit_buffer)
            self._blink_timer = 0
            self.editing = True

    def _clamp_scroll(self):
        visible = self._visible_rows()
        if self.selected < self._scroll:
            self._scroll = self.selected
        elif self.selected >= self._scroll + visible:
            self._scroll = self.selected - visible + 1
        self._scroll = max(0, min(self._scroll, len(self.FIELDS) - visible))

    def _visible_rows(self):
        # How many field rows fit in the content area
        MH = H - 120
        content_h = MH - 56 - 38   # minus header and footer
        return max(1, content_h // 34)

    # ------------------------------------------------------------------

    def draw(self, surf):
        if not self.active:
            return

        # Dim overlay
        overlay = pygame.Surface((W, H), pygame.SRCALPHA)
        overlay.fill((0, 0, 15, 210))
        surf.blit(overlay, (0, 0))

        MX, MY = 80, 55
        MW, MH = W - 160, H - 110
        panel = pygame.Rect(MX, MY, MW, MH)
        pygame.draw.rect(surf, C_PANEL,  panel, border_radius=10)
        pygame.draw.rect(surf, C_BORDER, panel, width=2, border_radius=10)

        # Header
        t_cfg  = FONT_BIG.render("Configuration", True, C_WHITE)
        t_note = FONT_SMALL.render(
            "Changes apply on restart   |   {}".format(
                os.path.basename(self.config_path)), True, C_GRAY)
        surf.blit(t_cfg,  (MX + 18, MY + 10))
        surf.blit(t_note, (MX + MW - t_note.get_width() - 14, MY + 18))
        pygame.draw.line(surf, C_BORDER,
                         (MX + 8, MY + 46), (MX + MW - 8, MY + 46), 1)

        # Columns
        LABEL_X  = MX + 22
        VALUE_X  = MX + 22 + (MW - 44) // 2
        ROW_H    = 34
        SEC_H    = 22
        start_y  = MY + 54

        y = start_y
        cur_sec = None
        visible = self._visible_rows()

        for i in range(self._scroll, min(self._scroll + visible + 1, len(self.FIELDS))):
            if y > MY + MH - 40:
                break
            sec, key, label, ftype = self.FIELDS[i]

            # Section header when section changes
            if sec != cur_sec:
                cur_sec = sec
                sh = FONT_SMALL.render("[  {}  ]".format(sec.upper()), True, C_CYAN)
                surf.blit(sh, (LABEL_X, y + 2))
                y += SEC_H

            is_sel = (i == self.selected)
            row_rect = pygame.Rect(LABEL_X - 6, y, MW - 32, ROW_H - 2)

            if is_sel:
                pygame.draw.rect(surf, (38, 52, 72), row_rect, border_radius=5)
                pygame.draw.rect(surf, C_BORDER,     row_rect, width=1, border_radius=5)

            # Label
            lbl_surf = FONT_MED.render(label, True, C_WHITE if is_sel else C_GRAY)
            surf.blit(lbl_surf, (LABEL_X, y + 8))

            # Value
            val = self.values.get((sec, key), "")
            if is_sel and self.editing:
                BLINK_PERIOD = 10   # frames per full on/off cycle
                self._blink_timer = (self._blink_timer + 1) % BLINK_PERIOD
                cursor = "|" if self._blink_timer < BLINK_PERIOD // 2 else " "
                display  = (self.edit_buffer[:self.edit_cursor]
                            + cursor
                            + self.edit_buffer[self.edit_cursor:])
                val_col  = C_YELLOW
            elif ftype == "bool":
                on       = val.lower() == "true"
                display  = "ON" if on else "OFF"
                val_col  = C_GREEN if on else C_RED
            elif ftype.startswith("choice:"):
                display  = val if val else "(empty)"
                val_col  = C_CYAN if is_sel else C_WHITE
            else:
                display  = val if val else "(empty)"
                val_col  = C_WHITE if is_sel else C_GRAY

            val_surf = FONT_MED.render(display, True, val_col)
            surf.blit(val_surf, (VALUE_X, y + 8))

            y += ROW_H

        # Scroll indicators
        if self._scroll > 0:
            up_surf = FONT_SMALL.render("▲ more", True, C_GRAY)
            surf.blit(up_surf, (MX + MW - up_surf.get_width() - 14, MY + 54))
        if self._scroll + visible < len(self.FIELDS):
            dn_surf = FONT_SMALL.render("▼ more", True, C_GRAY)
            surf.blit(dn_surf, (MX + MW - dn_surf.get_width() - 14, MY + MH - 46))

        # Footer
        footer_y = MY + MH - 28
        pygame.draw.line(surf, C_BORDER,
                         (MX + 8, footer_y - 6), (MX + MW - 8, footer_y - 6), 1)
        if self._save_timer > 0:
            self._save_timer -= 1
            msg = FONT_MED.render(
                "Saved to {}".format(os.path.basename(self.config_path)), True, C_GREEN)
            surf.blit(msg, (MX + MW // 2 - msg.get_width() // 2, footer_y - 2))
        else:
            hint = FONT_SMALL.render(
                "Up/Down: navigate    Enter/Space: edit / toggle    S: save    Esc: close",
                True, C_GRAY)
            surf.blit(hint, (MX + MW // 2 - hint.get_width() // 2, footer_y - 2))


config_menu = ConfigMenu(_args.config)

# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------

running = True
loop_interval = 1.0 / args.hz

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
            continue
        if config_menu.active:
            config_menu.handle_event(event)
            continue
        if event.type == pygame.KEYDOWN:
            if event.key in (pygame.K_q, pygame.K_ESCAPE):
                running = False
            elif event.key == pygame.K_s:
                show_scatter = not show_scatter
            elif event.key == pygame.K_c:
                config_menu.open()

    # --- Update GNSS ---
    gnss.update()

    # Append to track if we have a valid fix
    if gnss.has_fix and gnss.latitude and gnss.longitude:
        if not track or (gnss.latitude, gnss.longitude) != track[-1]:
            track.append((gnss.latitude, gnss.longitude))
            if len(track) > MAX_TRACK:
                track.pop(0)

    # --- Draw background ---
    screen.fill(C_BG)

    fix_color = FIX_COLORS.get(gnss.fix_quality, C_GRAY)
    fix_name  = gnss.fix_quality_name

    # Layout constants
    LX = 10          # left column x
    LW = 490          # left column width
    RX = 520          # right column x
    RW = W - RX - 10  # right column width
    M  = 10           # margin inside panels

    # ---- Title bar ----
    title_rect = pygame.Rect(10, 8, W - 20, 44)
    pygame.draw.rect(screen, C_PANEL, title_rect, border_radius=6)
    pygame.draw.rect(screen, fix_color, title_rect, width=2, border_radius=6)
    title_txt = FONT_BIG.render("RTK GNSS Monitor", True, C_WHITE)
    screen.blit(title_txt, (22, 16))
    fix_txt    = FONT_BIG.render(fix_name, True, fix_color)
    mode_label = FONT_BIG.render("Mode: ", True, C_GRAY)
    total_w    = mode_label.get_width() + fix_txt.get_width()
    screen.blit(mode_label, (W - total_w - 22, 16))
    screen.blit(fix_txt,    (W - fix_txt.get_width() - 22, 16))

    # ---- Position panel  (left, row 1) ----
    pos_rect = pygame.Rect(LX, 62, LW, 150)
    draw_panel(screen, pos_rect, "Position")
    lat_str = "{:.8f}".format(gnss.latitude)   if gnss.latitude  is not None else "---"
    lon_str = "{:.8f}".format(gnss.longitude)  if gnss.longitude is not None else "---"
    alt_str = "{:.3f} m".format(gnss.altitude) if gnss.altitude  is not None else "---"
    draw_field(screen, "Latitude",  lat_str, LX+M,       84)
    draw_field(screen, "Longitude", lon_str, LX+M,       122)
    draw_field(screen, "Altitude",  alt_str, LX+M+260,   84, color=C_YELLOW)

    # ---- Accuracy / DOP panel  (left, row 2) ----
    acc_rect = pygame.Rect(LX, 222, LW, 150)
    draw_panel(screen, acc_rect, "Accuracy / DOP")
    h_err    = gnss.h_error_m
    herr_str = "{:.4f} m".format(h_err)          if h_err           is not None else "---"
    verr_str = "{:.4f} m".format(gnss.std_alt)   if gnss.std_alt    is not None else "---"
    hdop_str = "{:.2f}".format(gnss.hdop)         if gnss.hdop       is not None else "---"
    pdop_str = "{:.2f}".format(gnss.pdop)         if gnss.pdop       is not None else "---"
    vdop_str = "{:.2f}".format(gnss.vdop)         if gnss.vdop       is not None else "---"
    draw_field(screen, "H-Error", herr_str, LX+M,     244, color=C_GREEN if h_err and h_err < 0.05 else C_YELLOW)
    draw_field(screen, "V-Error", verr_str, LX+M,     282, color=C_CYAN)
    draw_field(screen, "HDOP",    hdop_str, LX+M+200, 244)
    draw_field(screen, "PDOP",    pdop_str, LX+M+320, 244)
    draw_field(screen, "VDOP",    vdop_str, LX+M+200, 282)

    # ---- Satellites panel  (left, row 3) ----
    sat_rect = pygame.Rect(LX, 382, LW, 100)
    draw_panel(screen, sat_rect, "Satellites")
    sats_used = gnss.satellites or 0
    sats_view = gnss.satellites_in_view or 0
    draw_field(screen, "Used", str(sats_used), LX+M,     402)
    draw_field(screen, "In view", str(sats_view), LX+M+120, 402)
    if gnss.antenna_status != "UNKNOWN":
        ant_label = gnss.antenna_status
        ant_color = {"OK": C_GREEN, "OPEN": C_RED, "SHORT": C_RED}.get(gnss.antenna_status, C_GRAY)
    elif (gnss.satellites or 0) >= 4:
        ant_label = "OK"
        ant_color = C_GREEN
    else:
        ant_label = "?"
        ant_color = C_GRAY
    draw_field(screen, "Antenna", ant_label, LX+M+260, 402, color=ant_color)
    sat_bar(screen, LX+M, 458, sats_used)

    # ---- Time / Motion panel  (left, row 4) ----
    time_rect = pygame.Rect(LX, 492, LW, 100)
    draw_panel(screen, time_rect, "Time / Motion")
    # Format UTC time HHMMSS.sss -> HH:MM:SS
    raw_t = gnss.utc_time or ""
    if len(raw_t) >= 6 and raw_t[:6].isdigit():
        utc_str = "{}:{}:{}".format(raw_t[0:2], raw_t[2:4], raw_t[4:6])
    else:
        utc_str = raw_t or "---"
    # Format date DDMMYY -> DD/MM/20YY
    raw_d = gnss.date or ""
    if len(raw_d) == 6 and raw_d.isdigit():
        date_str = "{}/{}/20{}".format(raw_d[0:2], raw_d[2:4], raw_d[4:6])
    else:
        date_str = raw_d or "---"
    speed_str = "{:.2f} m/s".format(gnss.speed_ms) if gnss.speed_ms is not None else "---"
    course_str= "{:.1f} deg".format(gnss.course)   if gnss.course   is not None else "---"
    draw_field(screen, "UTC",    utc_str,   LX+M,     512)
    draw_field(screen, "Date",   date_str,  LX+M+220, 512)
    draw_field(screen, "Speed",  speed_str, LX+M,     550)
    draw_field(screen, "Course", course_str,LX+M+260, 550)

    # ---- NTRIP panel  (right, row 1) ----
    ntrip_rect = pygame.Rect(RX, 62, RW, 190)
    draw_panel(screen, ntrip_rect, "NTRIP Corrections")
    if ntrip:
        ntrip_color = {
            "connected":    C_GREEN,
            "connecting":   C_YELLOW,
            "error":        C_RED,
            "disconnected": C_GRAY,
        }.get(ntrip.status, C_GRAY)
        draw_field(screen, "Status",     ntrip.status.upper(),
                   RX+M, 82, color=ntrip_color)
        draw_field(screen, "Server",
                   "{}:{}/{}".format(ntrip.host, ntrip.port, ntrip.mountpoint),
                   RX+M, 120)
        draw_field(screen, "RTCM RX",   "{} bytes".format(ntrip.bytes_received),
                   RX+M, 158, color=C_CYAN)
        uptime  = ntrip.uptime_s
        up_str  = "{}s".format(int(uptime)) if uptime else "---"
        draw_field(screen, "Uptime", up_str, RX+M+220, 158)
        if ntrip.last_error:
            err_surf = FONT_SMALL.render("Err: " + ntrip.last_error[:45], True, C_RED)
            screen.blit(err_surf, (RX+M, 222))
    else:
        draw_field(screen, "Status", "Not configured", RX+M, 100, color=C_GRAY)
        hint = FONT_SMALL.render("Use --host / --mount to enable RTK", True, C_GRAY)
        screen.blit(hint, (RX+M, 148))

    # ---- Stats panel  (right, row 2) ----
    stats_rect = pygame.Rect(RX, 262, RW, 100)
    draw_panel(screen, stats_rect, "Parser Stats")
    stats = gnss.stats
    draw_field(screen, "Sentences", str(stats["parsed"]),      RX+M,     282)
    draw_field(screen, "Errors",    str(stats["errors"]),      RX+M+180, 282)
    draw_field(screen, "RTCM sent", "{} B".format(stats["rtcm_bytes"]), RX+M, 320)

    # ---- Sky map + Track  (right, row 3, side by side) ----
    bottom_y = 372
    bottom_h = H - bottom_y - 30
    half_w   = (RW - 8) // 2
    skymap_rect = pygame.Rect(RX,              bottom_y, half_w, bottom_h)
    track_rect  = pygame.Rect(RX + half_w + 8, bottom_y, half_w, bottom_h)
    draw_skymap(screen, skymap_rect, gnss.satellites_data)
    if show_scatter:
        draw_scatter(screen, track_rect, track)
    else:
        draw_track(screen, track_rect, track)

    # ---- Signal strength bars  (left, below Time/Motion) ----
    sig_rect = pygame.Rect(LX, 602, LW, H - 602 - 30)
    draw_signal_bars(screen, sig_rect, gnss.satellites_data)

    # ---- Footer ----
    footer_txt = FONT_SMALL.render(
        "Q: quit   S: scatter/track   C: config   demo={}   port={}   {}Hz   cfg={}".format(
            demo_mode, active_port, args.hz, os.path.basename(_args.config)),
        True, C_GRAY)
    screen.blit(footer_txt, (10, H - 20))

    config_menu.draw(screen)
    pygame.display.flip()
    clock.tick(args.hz)

# ---------------------------------------------------------------------------
# Shutdown
# ---------------------------------------------------------------------------

if ntrip:
    ntrip.stop()
pygame.quit()
print("Final stats:", gnss.stats)
