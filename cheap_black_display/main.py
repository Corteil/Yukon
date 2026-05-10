import sys
import uos

class _Null:
    def write(self, *a): return 0
    def flush(self): pass
    def read(self, *a): return b''
    def readinto(self, *a): return 0

uos.dupterm(_Null(), 0)

import gc
import struct
import time
import _thread
from micropython import const

import lvgl as lv
from machine import Pin, SPI
import lcd_bus
import axs15231b
import axs15231
from i2c import I2C as I2CBus

Pin(1, Pin.OUT).value(1)

spi_bus = SPI.Bus(
    host=1,
    sck=47,
    quad_pins=(21, 48, 40, 39),
)

display_bus = lcd_bus.SPIBusFast(
    spi_bus=spi_bus,
    dc=8,
    cs=45,
    freq=40000000,
    spi_mode=3,
    quad=True,
)

W = const(320)
H = const(480)

display = axs15231b.AXS15231B(
    display_bus,
    W, H,
    backlight_pin=1,
    color_space=lv.COLOR_FORMAT.RGB565,
    rgb565_byte_swap=True,
    backlight_on_state=axs15231b.STATE_PWM,
)
display.set_power(True)
display.set_backlight(100)
display.init()
# wait for USB-CDC startup messages to drain before serial thread starts
import time as _t
_t.sleep_ms(1000)

i2c_bus   = I2CBus.Bus(host=1, sda=4, scl=8)
touch_i2c = I2CBus.Device(i2c_bus, axs15231.I2C_ADDR, axs15231.BITS)
indev     = axs15231.AXS15231(touch_i2c, debug=False)
indev.enable_input_priority()

HEADER = b'\xAA\x55'
PKT_DRIVE   = const(0x10)
PKT_GPS     = const(0x11)
PKT_TAGNAV  = const(0x12)
PKT_INFO    = const(0x13)
PKT_CAMERA  = const(0x14)
PKT_FAULTS  = const(0x15)
CMD_ESTOP        = const(0x30)
CMD_RESET_ESTOP  = const(0x31)
CMD_SET_AUTO     = const(0x32)
CMD_SET_MANUAL   = const(0x33)
CMD_REBOOT       = const(0x34)
CMD_SNAPSHOT     = const(0x35)
CMD_RECORD_START = const(0x36)
CMD_RECORD_STOP  = const(0x37)
CMD_CLEAR_FAULTS = const(0x38)
CMD_BENCH_ON     = const(0x39)
CMD_BENCH_OFF    = const(0x3A)

def _crc8(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) if (crc & 0x80) else (crc << 1)
        crc &= 0xFF
    return crc

def _build_cmd(ptype, payload=b''):
    hdr = HEADER + bytes([ptype, len(payload)])
    return hdr + payload + bytes([_crc8(hdr[2:] + payload)])

_stream = sys.stdin.buffer

def _send_cmd(ptype, payload=b''):
    try:
        _stream.write(_build_cmd(ptype, payload))
    except Exception:
        pass

st = {
    'left': 0.0, 'right': 0.0,
    'fl': 0, 'fr': 0, 'rl': 0, 'rr': 0,
    'heading': 0.0, 'yaw': 0.0,
    'mode': 'MANUAL', 'nav_state': 'IDLE',
    'estop': False, 'no_motors': False,
    'lat': 0.0, 'lon': 0.0, 'alt': 0.0,
    'gps_spd': 0.0, 'gps_hdg': 0.0,
    'fix_qual': 0, 'fix': False, 'hdop': 0.0, 'sats': 0,
    'fix_name': 'No fix', 'ntrip': '', 'ntrip_kb': 0,
    'gate': 0, 'out_tag': 0, 'in_tag': 0,
    'next_gate': 0, 'next_out': 0, 'next_in': 0,
    'tags_vis': 0,
    'dist': 0.0, 'bear': 0.0, 'berr': 0.0,
    'wp_dist': 0.0, 'wp_bear': 0.0,
    'nav_label': '', 'next_label': '',
    'voltage': 0.0, 'pack_cur': 0.0,
    'fl_cur': 0.0, 'fr_cur': 0.0, 'rl_cur': 0.0, 'rr_cur': 0.0,
    'fl_tmp': 0.0, 'fr_tmp': 0.0, 'rl_tmp': 0.0, 'rr_tmp': 0.0,
    'board_tmp': 0.0, 'bench_tmp': 0.0,
    'cpu_pct': 0.0, 'cpu_tmp': 0.0, 'cpu_mhz': 0.0,
    'mem_used': 0.0, 'mem_tot': 0.0,
    'disk_used': 0.0, 'disk_tot': 0.0,
    'wifi_ip': '---', 'hostname': '---',
    'fault_mask': 0, 'bench_on': True,
    'cam_flags': 0, 'cam_flags2': 0, 'robot_count': 0,
    'health': 0xFF, 'faults': [],
    'last_pkt': -10000,
}
_lock = _thread.allocate_lock()

def _upd(**kw):
    with _lock:
        st.update(kw)

def _parse_drive(p):
    left, right = struct.unpack_from('>ff', p, 0)
    fl, fr, rl, rr = struct.unpack_from('>bbbb', p, 8)
    hdg, yaw = struct.unpack_from('>ff', p, 12)
    o = 20
    mlen = p[o]; o += 1; mode = p[o:o+mlen].decode(); o += mlen
    nlen = p[o]; o += 1; nav  = p[o:o+nlen].decode(); o += nlen
    estop     = bool(p[o]); o += 1
    no_motors = bool(p[o])
    _upd(left=left, right=right,
         fl=int(fl), fr=int(fr), rl=int(rl), rr=int(rr),
         heading=hdg, yaw=yaw, mode=mode, nav_state=nav,
         estop=estop, no_motors=no_motors)

def _parse_gps(p):
    lat, lon, alt, spd, ghdg = struct.unpack_from('>dddff', p, 0)
    fq, fix, nkb, hdop, _ = struct.unpack_from('>BBHff', p, 28)
    o = 40
    sats = p[o]; o += 1
    flen = p[o]; o += 1; fix_name = p[o:o+flen].decode(); o += flen
    nlen = p[o]; o += 1; ntrip    = p[o:o+nlen].decode()
    _upd(lat=lat, lon=lon, alt=alt, gps_spd=spd, gps_hdg=ghdg,
         fix_qual=fq, fix=bool(fix), hdop=hdop, sats=sats,
         fix_name=fix_name, ntrip=ntrip, ntrip_kb=nkb)

def _parse_tagnav(p):
    gate, out, inn, ng, no, ni, tv = struct.unpack_from('>HHHHHHH', p, 0)
    dist, bear, berr, wpd, wpb = struct.unpack_from('>fffff', p, 14)
    o = 34
    nlen = p[o]; o += 1; nav_s    = p[o:o+nlen].decode(); o += nlen
    glen = p[o]; o += 1; gate_lbl = p[o:o+glen].decode(); o += glen
    xlen = p[o]; o += 1; next_lbl = p[o:o+xlen].decode()
    _upd(gate=gate, out_tag=out, in_tag=inn,
         next_gate=ng, next_out=no, next_in=ni, tags_vis=tv,
         dist=dist, bear=bear, berr=berr, wp_dist=wpd, wp_bear=wpb,
         nav_state=nav_s, nav_label=gate_lbl, next_label=next_lbl)

def _parse_info(p):
    v, c, flc, frc, rlc, rrc, flt, frt = struct.unpack_from('>ffffffff', p, 0)
    rlt, rrt, bdt, bet = struct.unpack_from('>ffff', p, 32)
    cpp, cpt, cpmhz, mmu, mmt, mmp = struct.unpack_from('>ffffff', p, 48)
    dgu, dgt, dgp = struct.unpack_from('>fff', p, 72)
    o = 84
    ilen = p[o]; o += 1; ip   = p[o:o+ilen].decode(); o += ilen
    hlen = p[o]; o += 1; host = p[o:o+hlen].decode(); o += hlen
    fm = p[o]; o += 1
    bon = bool(p[o])
    _upd(voltage=v, pack_cur=c,
         fl_cur=flc, fr_cur=frc, rl_cur=rlc, rr_cur=rrc,
         fl_tmp=flt, fr_tmp=frt, rl_tmp=rlt, rr_tmp=rrt,
         board_tmp=bdt, bench_tmp=bet,
         cpu_pct=cpp, cpu_tmp=cpt, cpu_mhz=cpmhz,
         mem_used=mmu, mem_tot=mmt, disk_used=dgu, disk_tot=dgt,
         wifi_ip=ip, hostname=host, fault_mask=fm, bench_on=bon)

def _parse_camera(p):
    _upd(cam_flags=p[0], cam_flags2=p[1], robot_count=p[2])

def _parse_faults(p):
    health = p[0]; n = p[1]; faults = []; o = 2
    for _ in range(n):
        sev = chr(p[o]); o += 1
        mlen = p[o]; o += 1
        msg  = p[o:o+mlen].decode(); o += mlen
        faults.append((sev, msg))
    _upd(health=health, faults=faults)

_parsers = {
    PKT_DRIVE:  _parse_drive,  PKT_GPS:    _parse_gps,
    PKT_TAGNAV: _parse_tagnav, PKT_INFO:   _parse_info,
    PKT_CAMERA: _parse_camera, PKT_FAULTS: _parse_faults,
}

def _serial_thread():
    buf = bytearray()
    while True:
        try:
            b = _stream.read(1)
            if not b:
                continue
            buf += b
            idx = buf.find(HEADER)
            if idx < 0:
                buf = bytearray(); continue
            if idx > 0:
                del buf[:idx]
            if len(buf) < 4:
                continue
            ptype = buf[2]; plen = buf[3]
            needed = 4 + plen + 1
            if len(buf) < needed:
                rest = _stream.read(needed - len(buf))
                if rest:
                    buf += rest
            if len(buf) < needed:
                continue
            payload = bytes(buf[4:4+plen])
            crc     = buf[4+plen]
            if _crc8(bytes(buf[2:4+plen])) == crc:
                fn = _parsers.get(ptype)
                if fn:
                    fn(payload)
                    with _lock:
                        st['last_pkt'] = time.ticks_ms()
            del buf[:needed]
        except Exception:
            time.sleep_ms(5)

_thread.start_new_thread(_serial_thread, ())

C_BG     = lv.color_hex(0x080810)
C_PANEL  = lv.color_hex(0x0d0d20)
C_ACCENT = lv.color_hex(0x00d4ff)
C_GREEN  = lv.color_hex(0x00ff88)
C_YELLOW = lv.color_hex(0xffcc00)
C_AMBER  = lv.color_hex(0xffaa00)
C_RED    = lv.color_hex(0xff4444)
C_DIM    = lv.color_hex(0x444466)
C_WHITE  = lv.color_hex(0xffffff)
C_SEP    = lv.color_hex(0x111130)
C_TAB_BG = lv.color_hex(0x0a0a1e)

TAB_H     = const(32)
CONTENT_Y = TAB_H + 1
CONTENT_H = H - CONTENT_Y

def _lbl(parent, text, x, y, color=None, font=None):
    if color is None:
        color = C_WHITE
    if font is None:
        font = lv.font_montserrat_12
    lb = lv.label(parent)
    lb.set_text(text)
    lb.set_style_text_color(color, 0)
    lb.set_style_text_font(font, 0)
    lb.set_pos(x, y)
    return lb

def _sec(parent, text, x, y):
    return _lbl(parent, text, x, y, C_DIM, lv.font_montserrat_10)

def _divider(parent, y):
    line = lv.line(parent)
    pts  = [{'x': 0, 'y': 0}, {'x': W - 10, 'y': 0}]
    line.set_points(pts, 2)
    line.set_pos(4, y)
    line.set_style_line_color(C_SEP, 0)
    line.set_style_line_width(1, 0)
    return line

def _bar(parent, x, y, w, h):
    b = lv.bar(parent)
    b.set_size(w, h); b.set_pos(x, y)
    b.set_range(-100, 100); b.set_value(0, lv.ANIM.OFF)
    b.set_style_bg_color(C_PANEL, lv.PART.MAIN)
    b.set_style_bg_color(C_ACCENT, lv.PART.INDICATOR)
    b.set_style_radius(2, 0)
    return b

def _btn(parent, text, x, y, w, h, cb):
    b = lv.button(parent)
    b.set_pos(x, y); b.set_size(w, h)
    b.set_style_bg_color(C_PANEL, 0)
    b.set_style_border_color(C_DIM, 0)
    b.set_style_border_width(1, 0)
    b.set_style_radius(3, 0)
    lb = lv.label(b); lb.center()
    lb.set_text(text)
    lb.set_style_text_font(lv.font_montserrat_10, 0)
    b.add_event_cb(cb, lv.EVENT.CLICKED, None)
    return b

root = lv.screen_active()
root.set_style_bg_color(C_BG, 0)

tab_panel = lv.obj(root)
tab_panel.set_size(W, TAB_H)
tab_panel.set_pos(0, 0)
tab_panel.set_style_bg_color(C_TAB_BG, 0)
tab_panel.set_style_border_width(0, 0)
tab_panel.set_style_pad_all(0, 0)
tab_panel.remove_flag(lv.obj.FLAG.SCROLLABLE)

sep = lv.obj(root)
sep.set_size(W, 1); sep.set_pos(0, TAB_H)
sep.set_style_bg_color(C_SEP, 0)
sep.set_style_border_width(0, 0)

content = lv.obj(root)
content.set_size(W, CONTENT_H)
content.set_pos(0, CONTENT_Y)
content.set_style_bg_color(C_BG, 0)
content.set_style_border_width(0, 0)
content.set_style_pad_all(0, 0)
content.remove_flag(lv.obj.FLAG.SCROLLABLE)

lbl_link = lv.label(root)
lbl_link.set_text('*')
lbl_link.set_style_text_color(C_GREEN, 0)
lbl_link.set_style_text_font(lv.font_montserrat_12, 0)
lbl_link.set_pos(W - 14, 10)

SCREEN_IDS = ['drive', 'gps', 'tagnav', 'info', 'cam', 'faults']
screens = {}
for sid in SCREEN_IDS:
    pg = lv.obj(content)
    pg.set_size(W, CONTENT_H)
    pg.set_pos(0, 0)
    pg.set_style_bg_color(C_BG, 0)
    pg.set_style_border_width(0, 0)
    pg.set_style_pad_all(5, 0)
    pg.remove_flag(lv.obj.FLAG.SCROLLABLE)
    pg.add_flag(lv.obj.FLAG.HIDDEN)
    screens[sid] = pg

active_screen = ['drive']
_tabs = []

def _show(sid):
    active_screen[0] = sid
    for k, pg in screens.items():
        if k == sid:
            pg.remove_flag(lv.obj.FLAG.HIDDEN)
        else:
            pg.add_flag(lv.obj.FLAG.HIDDEN)
    for tb in _tabs:
        active = tb['id'] == sid
        tb['lbl'].set_style_text_color(C_ACCENT if active else C_DIM, 0)
        tb['obj'].set_style_bg_color(C_BG if active else C_TAB_BG, 0)

TAB_LABELS = ['Drive', 'GPS', 'TagNav', 'Info', 'Cam', 'Faults']
tab_w = W // len(SCREEN_IDS)

for i, (sid, tlabel) in enumerate(zip(SCREEN_IDS, TAB_LABELS)):
    tb = lv.obj(tab_panel)
    tb.set_size(tab_w, TAB_H)
    tb.set_pos(i * tab_w, 0)
    tb.set_style_bg_color(C_TAB_BG, 0)
    tb.set_style_border_width(0, 0)
    tb.set_style_pad_all(0, 0)
    tb.remove_flag(lv.obj.FLAG.SCROLLABLE)
    lb = lv.label(tb)
    lb.set_style_text_font(lv.font_montserrat_10, 0)
    lb.set_style_text_color(C_DIM, 0)
    lb.set_text(tlabel)
    lb.center()
    def _make_cb(s=sid):
        def cb(e): _show(s)
        return cb
    tb.add_event_cb(_make_cb(), lv.EVENT.CLICKED, None)
    _tabs.append({'id': sid, 'obj': tb, 'lbl': lb})

pg = screens['drive']
arc = lv.arc(pg)
arc.set_size(100, 100); arc.set_pos(0, 0)
arc.set_range(0, 360); arc.set_value(0)
arc.remove_style(None, lv.PART.KNOB)
arc.set_style_arc_color(C_ACCENT, lv.PART.INDICATOR)
arc.set_style_arc_color(C_PANEL,  lv.PART.MAIN)
_sec(pg, 'HDG', 34, 38)
lbl_hdg_val = _lbl(pg, '000', 22, 51, C_WHITE, lv.font_montserrat_14)
_sec(pg, 'SPEED', 108, 2)
lbl_spd       = _lbl(pg, '0.0', 106, 13, C_ACCENT, lv.font_montserrat_24)
_lbl(pg, 'm/s', 156, 21, C_DIM, lv.font_montserrat_10)
_sec(pg, 'MODE',  108, 48)
lbl_mode      = _lbl(pg, 'MANUAL', 106, 59, C_YELLOW, lv.font_montserrat_12)
_sec(pg, 'STATE', 108, 74)
lbl_nav_state = _lbl(pg, 'IDLE',   106, 85, C_GREEN,  lv.font_montserrat_12)
_divider(pg, 104)
_sec(pg, 'MOTORS  FL  FR  RL  RR', 4, 108)
MOT_X   = [18, 90, 162, 234]
MOT_KEY = ['fl', 'fr', 'rl', 'rr']
MOT_LBL = ['FL', 'FR', 'RL', 'RR']
mot_bars     = []
lbl_mot_vals = []
for mx, mk, ml in zip(MOT_X, MOT_KEY, MOT_LBL):
    _lbl(pg, ml, mx + 3, 118, C_DIM, lv.font_montserrat_10)
    b = _bar(pg, mx, 130, 22, 78)
    mot_bars.append(b)
    lv_tmp = _lbl(pg, '0', mx, 210, C_ACCENT, lv.font_montserrat_10)
    lbl_mot_vals.append(lv_tmp)
_divider(pg, 224)
_sec(pg, 'GPS',  4,  228)
lbl_gps_foot = _lbl(pg, '--',   4,  239, C_DIM, lv.font_montserrat_12)
_sec(pg, 'BAT', 116, 228)
lbl_bat_foot = _lbl(pg, '--',  116, 239, C_DIM, lv.font_montserrat_12)
_sec(pg, 'CPU', 220, 228)
lbl_cpu_foot = _lbl(pg, '--',  220, 239, C_DIM, lv.font_montserrat_12)
_divider(pg, 255)
lbl_coords   = _lbl(pg, '--,--', 4, 259, C_DIM, lv.font_montserrat_10)
_sec(pg, 'RC',  4,  273)
lbl_rc_foot  = _lbl(pg, '--',   4,  284, C_DIM, lv.font_montserrat_12)
_sec(pg, 'YAW', 116, 273)
lbl_yaw_foot = _lbl(pg, '--',  116, 284, C_DIM, lv.font_montserrat_12)

pg = screens['gps']
_sec(pg, 'POSITION', 4, 0)
lbl_lat      = _lbl(pg, 'Lat  ---',  4,  11, C_WHITE, lv.font_montserrat_12)
lbl_lon      = _lbl(pg, 'Lon  ---',  4,  25, C_WHITE, lv.font_montserrat_12)
lbl_alt      = _lbl(pg, 'Alt  ---',  4,  39, C_WHITE, lv.font_montserrat_12)
_divider(pg, 54)
_sec(pg, 'QUALITY', 4, 58)
lbl_fixname  = _lbl(pg, 'No fix',    4,  69, C_RED,   lv.font_montserrat_12)
lbl_sats     = _lbl(pg, 'Sats  --',  4,  83, C_WHITE, lv.font_montserrat_12)
lbl_hdop     = _lbl(pg, 'HDOP  --',  4,  97, C_WHITE, lv.font_montserrat_12)
_divider(pg, 112)
_sec(pg, 'VELOCITY', 4, 116)
lbl_gspd     = _lbl(pg, 'Speed  --', 4, 127, C_WHITE, lv.font_montserrat_12)
lbl_gtrack   = _lbl(pg, 'Track  --', 4, 141, C_WHITE, lv.font_montserrat_12)
_divider(pg, 156)
_sec(pg, 'NTRIP', 4, 160)
lbl_ntrip    = _lbl(pg, '--',        4, 171, C_DIM,   lv.font_montserrat_12)
lbl_ntrip_kb = _lbl(pg, '0 B',       4, 185, C_DIM,   lv.font_montserrat_10)
_divider(pg, 200)
_sec(pg, 'TRACK MAP', 4, 204)
gps_canvas = lv.canvas(pg)
gps_canvas.set_size(W - 10, 230)
gps_canvas.set_pos(4, 214)
_gps_cbuf = bytearray((W - 10) * 230 * 2)
gps_canvas.set_buffer(_gps_cbuf, W - 10, 230, lv.COLOR_FORMAT.RGB565)
gps_canvas.fill_bg(C_PANEL, lv.OPA.COVER)

pg = screens['tagnav']
_sec(pg, 'TARGET GATE', 4, 0)
lbl_gate_id   = _lbl(pg, '#--',       4,  11, C_ACCENT, lv.font_montserrat_16)
lbl_tn_state  = _lbl(pg, 'IDLE',      70, 15, C_YELLOW, lv.font_montserrat_12)
lbl_tn_tags   = _lbl(pg, 'T--/T--',   4,  32, C_DIM,   lv.font_montserrat_10)
lbl_tn_vis    = _lbl(pg, '0 vis',    160, 32, C_GREEN,  lv.font_montserrat_10)
lbl_tn_dist   = _lbl(pg, 'Dist  --',  4,  44, C_WHITE,  lv.font_montserrat_12)
lbl_tn_bear   = _lbl(pg, 'Bear  --',  4,  58, C_WHITE,  lv.font_montserrat_12)
lbl_tn_berr   = _lbl(pg, 'Err   --',  4,  72, C_WHITE,  lv.font_montserrat_12)
_divider(pg, 87)
_sec(pg, 'NEXT GATE', 4, 91)
lbl_next_gate = _lbl(pg, '#--',        4, 102, C_DIM,   lv.font_montserrat_12)
_divider(pg, 117)
_sec(pg, 'WAYPOINT', 4, 121)
lbl_wp_dist   = _lbl(pg, 'WP dist --', 4, 132, C_WHITE, lv.font_montserrat_12)
lbl_wp_bear   = _lbl(pg, 'WP bear --', 4, 146, C_WHITE, lv.font_montserrat_12)
_divider(pg, 161)
_sec(pg, 'CAMERA VIEW', 4, 165)
tn_canvas = lv.canvas(pg)
tn_canvas.set_size(W - 10, 270)
tn_canvas.set_pos(4, 175)
_tn_cbuf = bytearray((W - 10) * 270 * 2)
tn_canvas.set_buffer(_tn_cbuf, W - 10, 270, lv.COLOR_FORMAT.RGB565)
tn_canvas.fill_bg(C_PANEL, lv.OPA.COVER)

pg = screens['info']
_sec(pg, 'NETWORK', 4, 0)
lbl_ip       = _lbl(pg, 'IP    ---',     4,  11, C_WHITE, lv.font_montserrat_12)
lbl_host     = _lbl(pg, 'Host  ---',     4,  25, C_WHITE, lv.font_montserrat_12)
_divider(pg, 40)
_sec(pg, 'POWER', 4, 44)
lbl_volt     = _lbl(pg, 'Main  --V',     4,  55, C_GREEN, lv.font_montserrat_12)
lbl_cur      = _lbl(pg, 'Total --A',     4,  69, C_WHITE, lv.font_montserrat_12)
_divider(pg, 84)
_sec(pg, 'MOTORS  FL  FR  RL  RR', 4, 88)
_lbl(pg, 'FL',  14, 100, C_DIM, lv.font_montserrat_10)
_lbl(pg, 'FR',  94, 100, C_DIM, lv.font_montserrat_10)
_lbl(pg, 'RL', 174, 100, C_DIM, lv.font_montserrat_10)
_lbl(pg, 'RR', 254, 100, C_DIM, lv.font_montserrat_10)
lbl_fl_cur   = _lbl(pg, '--A',  4,  112, C_ACCENT, lv.font_montserrat_12)
lbl_fr_cur   = _lbl(pg, '--A',  84, 112, C_ACCENT, lv.font_montserrat_12)
lbl_rl_cur   = _lbl(pg, '--A', 164, 112, C_ACCENT, lv.font_montserrat_12)
lbl_rr_cur   = _lbl(pg, '--A', 244, 112, C_ACCENT, lv.font_montserrat_12)
lbl_fl_tmp   = _lbl(pg, '--',  4,  126, C_WHITE,  lv.font_montserrat_12)
lbl_fr_tmp   = _lbl(pg, '--',  84, 126, C_WHITE,  lv.font_montserrat_12)
lbl_rl_tmp   = _lbl(pg, '--', 164, 126, C_WHITE,  lv.font_montserrat_12)
lbl_rr_tmp   = _lbl(pg, '--', 244, 126, C_WHITE,  lv.font_montserrat_12)
lbl_flt_flags = _lbl(pg, 'FL FR RL RR FPV', 4, 140, C_GREEN, lv.font_montserrat_10)
_divider(pg, 154)
_sec(pg, 'PI 5', 4, 158)
lbl_cpu_i    = _lbl(pg, 'CPU  --%  --C', 4, 169, C_WHITE, lv.font_montserrat_12)
lbl_mem_i    = _lbl(pg, 'RAM  --/-- MB', 4, 183, C_WHITE, lv.font_montserrat_12)
lbl_disk_i   = _lbl(pg, 'Disk --/-- GB', 4, 197, C_WHITE, lv.font_montserrat_12)
_divider(pg, 212)
_sec(pg, 'BOARD', 4, 216)
lbl_brd_tmp  = _lbl(pg, 'Yukon  --C',   4, 227, C_WHITE, lv.font_montserrat_12)
lbl_bnch_tmp = _lbl(pg, 'FPV    --C',   4, 241, C_WHITE, lv.font_montserrat_12)

pg = screens['cam']
_sec(pg, 'CAMERAS', 4, 0)
CAM_NAMES  = ['Front L (IMX296)', 'Front R (IMX296)', 'Rear (IMX477)']
cam_ok_lbl  = []
cam_rec_lbl = []
for i, cn in enumerate(CAM_NAMES):
    y = 11 + i * 30
    _lbl(pg, cn, 4, y, C_DIM, lv.font_montserrat_10)
    ok  = _lbl(pg, '-- --',  4,  y + 12, C_DIM, lv.font_montserrat_12)
    rec = _lbl(pg, '',     160, y + 12, C_RED,  lv.font_montserrat_10)
    cam_ok_lbl.append(ok)
    cam_rec_lbl.append(rec)
_divider(pg, 104)
_sec(pg, 'DETECTION', 4, 108)
lbl_aruco_ok  = _lbl(pg, 'ArUco      --',  4, 119, C_DIM, lv.font_montserrat_12)
lbl_robot_det = _lbl(pg, 'Robot det  --',  4, 133, C_DIM, lv.font_montserrat_12)
lbl_robot_cnt = _lbl(pg, 'Robots seen  0', 4, 147, C_WHITE, lv.font_montserrat_12)
_divider(pg, 162)
_sec(pg, 'CONTROLS', 4, 166)

def _cb_snap_fl(e):  _send_cmd(CMD_SNAPSHOT,     bytes([0]))
def _cb_snap_fr(e):  _send_cmd(CMD_SNAPSHOT,     bytes([1]))
def _cb_snap_r(e):   _send_cmd(CMD_SNAPSHOT,     bytes([2]))
def _cb_rec_on(e):   _send_cmd(CMD_RECORD_START)
def _cb_rec_off(e):  _send_cmd(CMD_RECORD_STOP)
def _cb_fpv_on(e):   _send_cmd(CMD_BENCH_ON)

BW2 = (W - 14) // 3
_btn(pg, 'Snap FL',    4,              178, BW2, 30, _cb_snap_fl)
_btn(pg, 'Snap FR',    4 + BW2 + 3,   178, BW2, 30, _cb_snap_fr)
_btn(pg, 'Snap Rear',  4 + (BW2+3)*2, 178, BW2, 30, _cb_snap_r)
_btn(pg, 'Rec all',    4,              212, BW2, 30, _cb_rec_on)
_btn(pg, 'Stop rec',   4 + BW2 + 3,   212, BW2, 30, _cb_rec_off)
_btn(pg, 'FPV on',     4 + (BW2+3)*2, 212, BW2, 30, _cb_fpv_on)

pg = screens['faults']
_sec(pg, 'ACTIVE FAULTS', 4, 0)
lbl_fault_count = _lbl(pg, '', 180, 0, C_AMBER, lv.font_montserrat_10)
fault_rows = []
for i in range(7):
    y = 11 + i * 26
    dot = _lbl(pg, 'o', 4,  y, C_DIM, lv.font_montserrat_12)
    msg = _lbl(pg, '--', 20, y, C_DIM, lv.font_montserrat_12)
    fault_rows.append((dot, msg))
_divider(pg, 195)
_sec(pg, 'SYSTEM HEALTH', 4, 199)
HEALTH_NAMES = ['Motors', 'GPS', 'LiDAR', 'RC', 'Camera', 'ArUco', 'Hailo', 'FPV']
health_dots = []
for i, name in enumerate(HEALTH_NAMES):
    col = i % 2; row = i // 2
    x = 4 + col * 155; y = 210 + row * 18
    _lbl(pg, name, x + 16, y, C_DIM, lv.font_montserrat_10)
    d = _lbl(pg, 'o', x, y, C_DIM, lv.font_montserrat_12)
    health_dots.append(d)

def _cb_clear(e): _send_cmd(CMD_CLEAR_FAULTS)
_btn(pg, 'Clear faults', 4, 290, W - 10, 28, _cb_clear)

overlay = lv.obj(root)
overlay.set_size(W, H)
overlay.set_pos(0, 0)
overlay.set_style_bg_color(C_BG, 0)
overlay.set_style_bg_opa(240, 0)
overlay.set_style_border_width(0, 0)
overlay.set_style_pad_all(6, 0)
overlay.remove_flag(lv.obj.FLAG.SCROLLABLE)
overlay.add_flag(lv.obj.FLAG.HIDDEN)

_lbl(overlay, 'CONTROL PANEL', 4, 4, C_DIM, lv.font_montserrat_10)

def _hide_overlay(e):
    overlay.add_flag(lv.obj.FLAG.HIDDEN)

btn_estop = _btn(overlay, 'E-STOP', 4, 18, W - 10, 52, lambda e: None)
btn_estop.set_style_border_color(C_RED, 0)
btn_estop.set_style_border_width(2, 0)
lv.label(btn_estop).set_style_text_color(C_RED, 0)
lv.label(btn_estop).set_style_text_font(lv.font_montserrat_14, 0)

_estop_active = [False]

def _cb_estop_btn(e):
    if not _estop_active[0]:
        _send_cmd(CMD_ESTOP)
        _estop_active[0] = True
        btn_estop.set_style_border_color(C_GREEN, 0)
        lv.label(btn_estop).set_style_text_color(C_GREEN, 0)
        lv.label(btn_estop).set_text('RESUME')
    else:
        _send_cmd(CMD_RESET_ESTOP)
        _estop_active[0] = False
        btn_estop.set_style_border_color(C_RED, 0)
        lv.label(btn_estop).set_style_text_color(C_RED, 0)
        lv.label(btn_estop).set_text('E-STOP')

btn_estop.add_event_cb(_cb_estop_btn, lv.EVENT.CLICKED, None)

BW3 = (W - 16) // 3

def _ov_btn(text, x, y, cmd, payload=b''):
    def cb(e): _send_cmd(cmd, payload)
    return _btn(overlay, text, x, y, BW3, 36, cb)

_ov_btn('Auto',       4,              76, CMD_SET_AUTO)
_ov_btn('Manual',     4 + BW3 + 3,   76, CMD_SET_MANUAL)
_ov_btn('Reset stop', 4+(BW3+3)*2,   76, CMD_RESET_ESTOP)
_ov_btn('FPV on',     4,             116, CMD_BENCH_ON)
_ov_btn('FPV off',    4 + BW3 + 3,  116, CMD_BENCH_OFF)
_ov_btn('Snap FL',    4+(BW3+3)*2,  116, CMD_SNAPSHOT, bytes([0]))
_ov_btn('Rec all',    4,             156, CMD_RECORD_START)
_ov_btn('Stop rec',   4 + BW3 + 3,  156, CMD_RECORD_STOP)
_ov_btn('Reboot',     4+(BW3+3)*2,  156, CMD_REBOOT)
_btn(overlay, 'close', 4, 200, W - 10, 34, _hide_overlay)

def _open_overlay(e):
    overlay.remove_flag(lv.obj.FLAG.HIDDEN)

content.add_event_cb(_open_overlay, lv.EVENT.CLICKED, None)

def _motor_colour(pct):
    a = abs(pct)
    if a > 90: return C_RED
    if a > 70: return C_AMBER
    return C_ACCENT

def _update_ui(timer):
    with _lock:
        s = dict(st)
    age = time.ticks_diff(time.ticks_ms(), s['last_pkt'])
    lbl_link.set_style_text_color(C_GREEN if age < 1500 else C_RED, 0)
    sid = active_screen[0]

    if sid == 'drive':
        hdg = int(s['heading']) % 360
        arc.set_value(hdg)
        lbl_hdg_val.set_text(str(hdg))
        spd = (abs(s['left']) + abs(s['right'])) / 2
        lbl_spd.set_text('{:.1f}'.format(spd))
        lbl_mode.set_text(s['mode'])
        lbl_mode.set_style_text_color(
            C_RED if s['estop'] else (C_GREEN if s['mode'] == 'AUTO' else C_YELLOW), 0)
        lbl_nav_state.set_text(s['nav_state'])
        for bar, lbl_v, key in zip(mot_bars, lbl_mot_vals, MOT_KEY):
            pct = s[key]
            bar.set_value(pct, lv.ANIM.OFF)
            lbl_v.set_text(str(pct))
            lbl_v.set_style_text_color(_motor_colour(pct), 0)
        lbl_gps_foot.set_text(s['fix_name'] if s['fix'] else 'NO FIX')
        lbl_gps_foot.set_style_text_color(C_GREEN if s['fix'] else C_RED, 0)
        lbl_bat_foot.set_text('{:.1f}V'.format(s['voltage']))
        lbl_cpu_foot.set_text('{:.0f}C'.format(s['cpu_tmp']))
        lbl_coords.set_text('{:.4f},{:.4f}'.format(s['lat'], s['lon']))
        lbl_rc_foot.set_text('ok' if s['health'] & 0x08 else 'lost')
        lbl_rc_foot.set_style_text_color(C_GREEN if s['health'] & 0x08 else C_RED, 0)
        lbl_yaw_foot.set_text('{:.1f}/s'.format(s['yaw']))

    elif sid == 'gps':
        lbl_lat.set_text('Lat  {:.6f}'.format(s['lat']))
        lbl_lon.set_text('Lon  {:.6f}'.format(s['lon']))
        lbl_alt.set_text('Alt  {:.1f}m'.format(s['alt']))
        lbl_fixname.set_text(s['fix_name'])
        lbl_fixname.set_style_text_color(
            C_GREEN if s['fix_qual'] >= 4 else
            (C_AMBER if s['fix_qual'] >= 1 else C_RED), 0)
        lbl_sats.set_text('Sats  {}'.format(s['sats']))
        lbl_hdop.set_text('HDOP  {:.1f}'.format(s['hdop']))
        lbl_gspd.set_text('Speed  {:.2f}m/s'.format(s['gps_spd']))
        lbl_gtrack.set_text('Track  {:.0f}'.format(s['gps_hdg']))
        nc = (C_GREEN if s['ntrip'] == 'connected' else
              C_AMBER if s['ntrip'] == 'connecting' else C_RED)
        lbl_ntrip.set_text(s['ntrip'] or 'disabled')
        lbl_ntrip.set_style_text_color(nc, 0)
        kb = s['ntrip_kb']
        lbl_ntrip_kb.set_text('{} KB'.format(kb//1024) if kb >= 1024 else '{} B'.format(kb))

    elif sid == 'tagnav':
        lbl_gate_id.set_text('#{}'.format(s['gate']))
        lbl_tn_state.set_text(s['nav_state'])
        lbl_tn_tags.set_text('T{}/T{}'.format(s['out_tag'], s['in_tag']))
        lbl_tn_vis.set_text('{} vis'.format(s['tags_vis']))
        lbl_tn_vis.set_style_text_color(C_GREEN if s['tags_vis'] > 0 else C_DIM, 0)
        lbl_tn_dist.set_text('Dist  {:.2f}m'.format(s['dist']))
        lbl_tn_bear.set_text('Bear  {:.1f}'.format(s['bear']))
        lbl_tn_berr.set_text('Err   {:.1f}'.format(s['berr']))
        lbl_next_gate.set_text('#{} T{}/T{} {}'.format(
            s['next_gate'], s['next_out'], s['next_in'], s['next_label']))
        lbl_wp_dist.set_text('WP dist  {:.1f}m'.format(s['wp_dist']))
        lbl_wp_bear.set_text('WP bear  {:.0f}'.format(s['wp_bear']))

    elif sid == 'info':
        lbl_ip.set_text('IP    {}'.format(s['wifi_ip']))
        lbl_host.set_text('Host  {}'.format(s['hostname']))
        v = s['voltage']
        lbl_volt.set_text('Main  {:.2f}V'.format(v))
        lbl_volt.set_style_text_color(
            C_RED if v < 10.5 else (C_AMBER if v < 11.1 else C_GREEN), 0)
        lbl_cur.set_text('Total {:.1f}A'.format(s['pack_cur']))
        lbl_fl_cur.set_text('{:.1f}A'.format(s['fl_cur']))
        lbl_fr_cur.set_text('{:.1f}A'.format(s['fr_cur']))
        lbl_rl_cur.set_text('{:.1f}A'.format(s['rl_cur']))
        lbl_rr_cur.set_text('{:.1f}A'.format(s['rr_cur']))
        lbl_fl_tmp.set_text('{:.0f}C'.format(s['fl_tmp']))
        lbl_fr_tmp.set_text('{:.0f}C'.format(s['fr_tmp']))
        lbl_rl_tmp.set_text('{:.0f}C'.format(s['rl_tmp']))
        lbl_rr_tmp.set_text('{:.0f}C'.format(s['rr_tmp']))
        lbl_flt_flags.set_style_text_color(
            C_RED if s['fault_mask'] else C_GREEN, 0)
        lbl_cpu_i.set_text('CPU  {:.0f}%  {:.0f}C'.format(s['cpu_pct'], s['cpu_tmp']))
        lbl_mem_i.set_text('RAM  {:.0f}/{:.0f}MB'.format(s['mem_used'], s['mem_tot']))
        lbl_disk_i.set_text('Disk {:.1f}/{:.0f}GB'.format(s['disk_used'], s['disk_tot']))
        lbl_brd_tmp.set_text('Yukon  {:.0f}C'.format(s['board_tmp']))
        lbl_bnch_tmp.set_text('FPV    {:.0f}C'.format(s['bench_tmp']))

    elif sid == 'cam':
        flags = s['cam_flags']; flags2 = s['cam_flags2']
        for i in range(3):
            ok  = bool(flags & (1 << i))
            rec = bool(flags & (1 << (i + 3)))
            cam_ok_lbl[i].set_text('OK' if ok else '--')
            cam_ok_lbl[i].set_style_text_color(C_GREEN if ok else C_DIM, 0)
            cam_rec_lbl[i].set_text('REC' if rec else '')
        aruco_ok = bool(flags2 & 0x02)
        rdet_ok  = bool(flags2 & 0x04)
        lbl_aruco_ok.set_text('ArUco      {}'.format('OK' if aruco_ok else '--'))
        lbl_aruco_ok.set_style_text_color(C_GREEN if aruco_ok else C_DIM, 0)
        lbl_robot_det.set_text('Robot det  {}'.format('OK' if rdet_ok else '--'))
        lbl_robot_det.set_style_text_color(C_GREEN if rdet_ok else C_DIM, 0)
        lbl_robot_cnt.set_text('Robots seen  {}'.format(s['robot_count']))

    elif sid == 'faults':
        faults = s['faults']
        active_n = sum(1 for sv, _ in faults if sv in ('E', 'W'))
        lbl_fault_count.set_text('{} active'.format(active_n) if active_n else '')
        for i, (dot, msg) in enumerate(fault_rows):
            if i < len(faults):
                sev, txt = faults[i]
                col = C_RED if sev == 'E' else C_AMBER
                dot.set_style_text_color(col, 0)
                msg.set_text(txt[:34])
                msg.set_style_text_color(col, 0)
            else:
                dot.set_style_text_color(C_DIM, 0)
                msg.set_text('--')
                msg.set_style_text_color(C_DIM, 0)
        h = s['health']
        for i, d in enumerate(health_dots):
            d.set_style_text_color(C_GREEN if (h >> i) & 1 else C_RED, 0)

    gc.collect()

lv.timer_create(_update_ui, 100, None)
_show('drive')

while True:
    lv.task_handler()
    time.sleep_ms(5)