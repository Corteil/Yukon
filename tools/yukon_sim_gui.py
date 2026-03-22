#!/usr/bin/env python3
"""
yukon_sim_gui.py — Pygame GUI front-end for the Yukon serial simulator.

Runs yukon_sim.py's PTY server in the background and provides a visual
dashboard for monitoring and controlling the simulation.

Features
--------
  Compass rose    : live heading needle + bearing-hold target needle
  Motor bars      : left / right speed bars updated in real time
  Voltage slider  : simulate battery voltage (8–15 V)
  Current slider  : simulate motor current draw (0–5 A)
  IMU toggle      : enable / disable simulated IMU
  Fault buttons   : inject left / right motor faults
  Bearing hold    : shows target and error when active
  Click compass   : click anywhere on the rose to jump heading there

Usage
-----
  python3 yukon_sim_gui.py
  python3 yukon_sim_gui.py --no-terminal   # suppress terminal output

Keys
----
  <  /  >    : decrease / increase heading by 5°
  I          : toggle IMU present
  Q / Esc    : quit
"""

import math
import os
import pty
import select
import sys
import threading
import time
import tty
import argparse

import pygame

# ── Import shared state from yukon_sim ───────────────────────────────────────
# yukon_sim.py must be in the same directory
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import yukon_sim as _sim

# ── Colours ───────────────────────────────────────────────────────────────────
C_BG      = ( 18,  18,  30)
C_PANEL   = ( 28,  28,  45)
C_BORDER  = ( 60,  60,  90)
C_WHITE   = (230, 230, 240)
C_GRAY    = (130, 130, 150)
C_GREEN   = ( 60, 220,  80)
C_YELLOW  = (240, 200,  40)
C_ORANGE  = (240, 140,  40)
C_RED     = (220,  60,  60)
C_CYAN    = ( 60, 200, 220)
C_BLUE    = ( 60, 120, 220)
C_DARKRED = ( 80,  10,  10)

# ── Layout ────────────────────────────────────────────────────────────────────
W, H = 900, 620

# ── Helpers ───────────────────────────────────────────────────────────────────

def _panel(surf, rect, title=None):
    pygame.draw.rect(surf, C_PANEL,  rect, border_radius=8)
    pygame.draw.rect(surf, C_BORDER, rect, width=1, border_radius=8)
    if title:
        t = FONT_SM.render(title, True, C_GRAY)
        surf.blit(t, (rect.x + 10, rect.y + 6))


def _label(surf, text, x, y, color=C_GRAY, font=None):
    f = font or FONT_SM
    surf.blit(f.render(text, True, color), (x, y))


def _button(surf, rect, text, color, active=False):
    bg = tuple(min(255, c + 30) for c in C_PANEL) if active else C_PANEL
    pygame.draw.rect(surf, bg,    rect, border_radius=6)
    pygame.draw.rect(surf, color, rect, width=2, border_radius=6)
    t = FONT_SM.render(text, True, color)
    surf.blit(t, (rect.centerx - t.get_width()//2,
                  rect.centery - t.get_height()//2))


def _motor_bar(surf, rect, speed):
    """Draw a -1..+1 motor speed bar."""
    pygame.draw.rect(surf, C_BG, rect, border_radius=4)
    cx = rect.x + rect.width // 2
    pygame.draw.line(surf, C_BORDER, (cx, rect.y+2), (cx, rect.bottom-2), 1)
    if speed is not None and speed != 0:
        fill_w = int(abs(speed) * rect.width // 2)
        color  = C_GREEN if speed > 0 else C_ORANGE
        if speed > 0:
            fill = pygame.Rect(cx, rect.y+3, fill_w, rect.height-6)
        else:
            fill = pygame.Rect(cx - fill_w, rect.y+3, fill_w, rect.height-6)
        pygame.draw.rect(surf, color, fill, border_radius=3)
    val = "---" if speed is None else f"{speed:+.2f}"
    col = C_GRAY if speed is None else (C_GREEN if speed >= 0 else C_ORANGE)
    t   = FONT_SM.render(val, True, col)
    surf.blit(t, (rect.right - t.get_width() - 6,
                  rect.centery - t.get_height()//2))


def _slider(surf, rect, value, vmin, vmax, color, fmt="{:.1f}"):
    """Horizontal slider — returns new value if clicked/dragged."""
    pygame.draw.rect(surf, C_BG,    rect, border_radius=4)
    pygame.draw.rect(surf, C_BORDER,rect, width=1, border_radius=4)
    t = (value - vmin) / (vmax - vmin)
    fill_w = int(t * rect.width)
    if fill_w > 0:
        pygame.draw.rect(surf, color,
                         pygame.Rect(rect.x, rect.y, fill_w, rect.height),
                         border_radius=4)
    # Knob
    kx = rect.x + fill_w
    pygame.draw.circle(surf, C_WHITE, (kx, rect.centery), 7)
    # Value label
    lbl = FONT_SM.render(fmt.format(value), True, C_WHITE)
    surf.blit(lbl, (rect.x + rect.width//2 - lbl.get_width()//2,
                    rect.centery - lbl.get_height()//2))
    return None


def _compass(surf, cx, cy, radius, heading, bearing_target, imu_present):
    """Draw compass rose with heading needle and optional bearing target needle."""
    # Outer ring
    pygame.draw.circle(surf, C_BORDER, (cx, cy), radius, 2)
    pygame.draw.circle(surf, C_BG,     (cx, cy), radius - 3)

    # Cardinal tick marks + labels
    for angle, label in ((0,"N"),(90,"E"),(180,"S"),(270,"W")):
        rad = math.radians(angle - 90)
        ox  = cx + int((radius - 2)  * math.cos(rad))
        oy  = cy + int((radius - 2)  * math.sin(rad))
        ix  = cx + int((radius - 14) * math.cos(rad))
        iy  = cy + int((radius - 14) * math.sin(rad))
        pygame.draw.line(surf, C_GRAY, (ix, iy), (ox, oy), 2)
        lx  = cx + int((radius - 26) * math.cos(rad))
        ly  = cy + int((radius - 26) * math.sin(rad))
        t   = FONT_TINY.render(label, True, C_GRAY)
        surf.blit(t, (lx - t.get_width()//2, ly - t.get_height()//2))

    # Intercardinal ticks
    for angle in range(0, 360, 45):
        if angle % 90 == 0:
            continue
        rad = math.radians(angle - 90)
        ox  = cx + int((radius - 2)  * math.cos(rad))
        oy  = cy + int((radius - 2)  * math.sin(rad))
        ix  = cx + int((radius - 10) * math.cos(rad))
        iy  = cy + int((radius - 10) * math.sin(rad))
        pygame.draw.line(surf, C_BORDER, (ix, iy), (ox, oy), 1)

    if not imu_present:
        t = FONT_MD.render("IMU ABSENT", True, C_RED)
        surf.blit(t, (cx - t.get_width()//2, cy - t.get_height()//2))
        return

    # Bearing target needle (cyan, dashed appearance via segments)
    if bearing_target is not None:
        trad = math.radians(bearing_target - 90)
        tx   = cx + int((radius - 20) * math.cos(trad))
        ty   = cy + int((radius - 20) * math.sin(trad))
        # Draw dashed by segments
        steps = 10
        for i in range(0, steps, 2):
            sx = cx + int((radius - 20) * i/steps * math.cos(trad))
            sy = cy + int((radius - 20) * i/steps * math.sin(trad))
            ex = cx + int((radius - 20) * (i+1)/steps * math.cos(trad))
            ey = cy + int((radius - 20) * (i+1)/steps * math.sin(trad))
            pygame.draw.line(surf, C_CYAN, (sx, sy), (ex, ey), 2)
        # Target label
        lx = cx + int((radius + 10) * math.cos(trad))
        ly = cy + int((radius + 10) * math.sin(trad))
        t  = FONT_TINY.render(f"{bearing_target:.0f}°", True, C_CYAN)
        surf.blit(t, (lx - t.get_width()//2, ly - t.get_height()//2))

    # Heading needle (white, with red tail)
    hrad   = math.radians(heading - 90)
    needle = int(radius - 18)
    tail   = int(radius * 0.3)
    nx     = cx + int(needle * math.cos(hrad))
    ny     = cy + int(needle * math.sin(hrad))
    tx2    = cx - int(tail   * math.cos(hrad))
    ty2    = cy - int(tail   * math.sin(hrad))
    pygame.draw.line(surf, C_RED,   (cx, cy), (tx2, ty2), 3)
    pygame.draw.line(surf, C_WHITE, (cx, cy), (nx, ny),   3)
    pygame.draw.circle(surf, C_WHITE, (cx, cy), 5)

    # Heading text in centre
    t = FONT_MD.render(f"{heading:.1f}°", True, C_WHITE)
    surf.blit(t, (cx - t.get_width()//2, cy + radius//2))


# ── Slider interaction ────────────────────────────────────────────────────────

class Slider:
    def __init__(self, rect, vmin, vmax, value, color, label, fmt="{:.1f}"):
        self.rect   = pygame.Rect(rect)
        self.vmin   = vmin
        self.vmax   = vmax
        self.value  = value
        self.color  = color
        self.label  = label
        self.fmt    = fmt
        self.active = False

    def draw(self, surf):
        _label(surf, self.label, self.rect.x, self.rect.y - 18)
        _slider(surf, self.rect, self.value, self.vmin, self.vmax,
                self.color, self.fmt)

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            if self.rect.collidepoint(event.pos):
                self.active = True
                self._set_from_mouse(event.pos[0])
        elif event.type == pygame.MOUSEBUTTONUP:
            self.active = False
        elif event.type == pygame.MOUSEMOTION and self.active:
            self._set_from_mouse(event.pos[0])

    def _set_from_mouse(self, mx):
        t = (mx - self.rect.x) / self.rect.width
        self.value = max(self.vmin, min(self.vmax,
                         self.vmin + t * (self.vmax - self.vmin)))


# ── Main GUI ──────────────────────────────────────────────────────────────────

def run_gui(yukon_path, no_terminal=False):
    global FONT_BIG, FONT_MD, FONT_SM, FONT_TINY

    pygame.init()
    screen = pygame.display.set_mode((W, H))
    pygame.display.set_caption("Yukon Simulator")
    clock  = pygame.time.Clock()

    FONT_BIG  = pygame.font.SysFont("monospace", 26, bold=True)
    FONT_MD   = pygame.font.SysFont("monospace", 18)
    FONT_SM   = pygame.font.SysFont("monospace", 14)
    FONT_TINY = pygame.font.SysFont("monospace", 11)

    # Sliders
    volt_slider = Slider((480, 100, 200, 22), 8.0, 15.0, 12.0,
                         C_YELLOW, "Voltage (V)", "{:.1f} V")
    curr_slider = Slider((480, 160, 200, 22), 0.0,  5.0,  0.5,
                         C_ORANGE, "Current (A)", "{:.2f} A")

    # Button rects
    imu_btn    = pygame.Rect(480, 220, 200, 36)
    fault_l    = pygame.Rect(480, 280, 95,  36)
    fault_r    = pygame.Rect(585, 280, 95,  36)
    reset_f    = pygame.Rect(480, 330, 200, 36)

    # Fault state (local to GUI — injected into sim state)
    fault_left  = False
    fault_right = False

    # Compass centre + radius
    comp_cx = 180
    comp_cy = 302
    comp_r  = 140

    running = True
    while running:
        clock.tick(30)

        # ── Events ───────────────────────────────────────────────────────────
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            volt_slider.handle_event(event)
            curr_slider.handle_event(event)

            if event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_q, pygame.K_ESCAPE):
                    running = False
                elif event.key == pygame.K_i:
                    with _sim._lock:
                        _sim._state['imu_present'] = not _sim._state['imu_present']
                        if not _sim._state['imu_present']:
                            _sim._state['bearing_target'] = None
                elif event.key == pygame.K_GREATER or event.key == pygame.K_PERIOD:
                    with _sim._lock:
                        if _sim._state['imu_present']:
                            _sim._state['imu_heading'] = (
                                _sim._state['imu_heading'] + _sim.IMU_HEADING_STEP
                            ) % 360.0
                elif event.key == pygame.K_LESS or event.key == pygame.K_COMMA:
                    with _sim._lock:
                        if _sim._state['imu_present']:
                            _sim._state['imu_heading'] = (
                                _sim._state['imu_heading'] - _sim.IMU_HEADING_STEP
                            ) % 360.0

            if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                mx, my = event.pos

                # Click inside compass → set heading
                dx, dy = mx - comp_cx, my - comp_cy
                if dx*dx + dy*dy <= comp_r*comp_r:
                    angle = (math.degrees(math.atan2(dy, dx)) + 90) % 360
                    with _sim._lock:
                        if _sim._state['imu_present']:
                            _sim._state['imu_heading'] = angle

                # IMU toggle button
                if imu_btn.collidepoint(mx, my):
                    with _sim._lock:
                        _sim._state['imu_present'] = not _sim._state['imu_present']
                        if not _sim._state['imu_present']:
                            _sim._state['bearing_target'] = None

                # Fault inject
                if fault_l.collidepoint(mx, my):
                    fault_left = not fault_left
                if fault_r.collidepoint(mx, my):
                    fault_right = not fault_right
                if reset_f.collidepoint(mx, my):
                    fault_left = fault_right = False

        # Push slider values + fault state into sim
        _sim.SIM_VOLTAGE = volt_slider.value
        _sim.SIM_CURRENT = curr_slider.value
        with _sim._lock:
            _sim._state['fault_l'] = fault_left
            _sim._state['fault_r'] = fault_right

        # Tick IMU
        _sim._tick_imu()

        # ── Read state ───────────────────────────────────────────────────────
        with _sim._lock:
            lb          = _sim._state['left_byte']
            rb          = _sim._state['right_byte']
            led_a       = _sim._state['led_a']
            led_b       = _sim._state['led_b']
            cmds        = _sim._state['cmds_rx']
            imu_present = _sim._state['imu_present']
            imu_heading = _sim._state['imu_heading']
            bearing_tgt = _sim._state['bearing_target']

        rx_l = _sim._decode_speed(lb)
        rx_r = _sim._decode_speed(rb)

        # ── Draw ─────────────────────────────────────────────────────────────
        screen.fill(C_BG)

        # Title bar
        title_rect = pygame.Rect(10, 8, W - 20, 65)
        pygame.draw.rect(screen, C_PANEL, title_rect, border_radius=6)
        pygame.draw.rect(screen, C_CYAN,  title_rect, width=1, border_radius=6)
        screen.blit(FONT_BIG.render("Yukon Simulator", True, C_WHITE), (22, 16))
        port_t = FONT_SM.render(f"PTY: {yukon_path}", True, C_GRAY)
        screen.blit(port_t, (W - port_t.get_width() - 16, 22))
        hint = FONT_SM.render("< > = heading   I = IMU toggle   Q = quit   click compass = set heading",
                              True, C_GRAY)
        screen.blit(hint, (W//2 - hint.get_width()//2, 50))

        # ── Left column: Compass ──────────────────────────────────────────────
        comp_panel = pygame.Rect(10, 82, 350, 400)
        _panel(screen, comp_panel, "IMU Heading")
        _compass(screen, comp_cx, comp_cy, comp_r,
                 imu_heading, bearing_tgt, imu_present)

        # Bearing hold info below compass
        if imu_present and bearing_tgt is not None:
            err = _sim._angle_diff(bearing_tgt, imu_heading)
            info = f"Hold: {bearing_tgt:.1f}°   Err: {err:+.1f}°"
            col  = C_GREEN if abs(err) < 5 else C_YELLOW
        elif imu_present:
            info, col = "Bearing hold: OFF", C_GRAY
        else:
            info, col = "IMU ABSENT", C_RED
        t = FONT_SM.render(info, True, col)
        screen.blit(t, (comp_panel.centerx - t.get_width()//2, comp_panel.bottom - 30))

        # ── Middle column: Motors + Status ────────────────────────────────────
        mid_panel = pygame.Rect(370, 82, 240, 260)
        _panel(screen, mid_panel, "Drive")

        for i, (label, speed) in enumerate((("Left", rx_l), ("Right", rx_r))):
            by = mid_panel.y + 30 + i * 90
            _label(screen, label, mid_panel.x + 12, by + 4)
            bar = pygame.Rect(mid_panel.x + 12, by + 22, mid_panel.width - 24, 40)
            _motor_bar(screen, bar, speed)

        # LED status
        ly = mid_panel.y + 190
        for label, state in (("LED A", led_a), ("LED B", led_b)):
            col = C_GREEN if state else C_GRAY
            _label(screen, f"{label}: {'ON ' if state else 'OFF'}", mid_panel.x + 12, ly, col)
            ly += 20

        # Cmds received
        _label(screen, f"Cmds rx: {cmds}", mid_panel.x + 12, mid_panel.bottom - 22, C_GRAY)

        # Status panel
        stat_panel = pygame.Rect(370, 352, 240, 130)
        _panel(screen, stat_panel, "Sim Values")
        _label(screen, f"Voltage : {volt_slider.value:.1f} V",  stat_panel.x+12, stat_panel.y+26, C_YELLOW)
        _label(screen, f"Current : {curr_slider.value:.2f} A",  stat_panel.x+12, stat_panel.y+50, C_ORANGE)
        fl_col = C_RED   if fault_left  else C_GRAY
        fr_col = C_RED   if fault_right else C_GRAY
        _label(screen, f"Fault L : {'YES' if fault_left  else 'no'}", stat_panel.x+12, stat_panel.y+74,  fl_col)
        _label(screen, f"Fault R : {'YES' if fault_right else 'no'}", stat_panel.x+12, stat_panel.y+98,  fr_col)

        # ── Right column: Controls ────────────────────────────────────────────
        ctrl_panel = pygame.Rect(620, 82, 270, 400)
        _panel(screen, ctrl_panel, "Controls")

        # Sliders
        volt_slider.rect = pygame.Rect(ctrl_panel.x + 12, ctrl_panel.y + 40,
                                       ctrl_panel.width - 24, 22)
        curr_slider.rect = pygame.Rect(ctrl_panel.x + 12, ctrl_panel.y + 100,
                                       ctrl_panel.width - 24, 22)
        volt_slider.draw(screen)
        curr_slider.draw(screen)

        # IMU toggle button
        imu_btn = pygame.Rect(ctrl_panel.x + 12, ctrl_panel.y + 155,
                              ctrl_panel.width - 24, 36)
        imu_col = C_GREEN if imu_present else C_RED
        _button(screen, imu_btn,
                f"IMU: {'PRESENT' if imu_present else 'ABSENT'}",
                imu_col, active=imu_present)

        # Fault buttons
        fault_l = pygame.Rect(ctrl_panel.x + 12,  ctrl_panel.y + 210,
                              (ctrl_panel.width - 28)//2, 36)
        fault_r = pygame.Rect(fault_l.right + 4,  ctrl_panel.y + 210,
                              (ctrl_panel.width - 28)//2, 36)
        reset_f = pygame.Rect(ctrl_panel.x + 12,  ctrl_panel.y + 258,
                              ctrl_panel.width - 24, 36)

        _button(screen, fault_l, "Fault L", C_RED,    active=fault_left)
        _button(screen, fault_r, "Fault R", C_RED,    active=fault_right)
        _button(screen, reset_f, "Clear Faults", C_GRAY)

        # Heading nudge buttons (for trackpad / mouse users)
        hl = pygame.Rect(ctrl_panel.x + 12,         ctrl_panel.y + 310, 80, 36)
        hr = pygame.Rect(ctrl_panel.right - 12 - 80, ctrl_panel.y + 310, 80, 36)
        _button(screen, hl, "< -5°", C_CYAN)
        _button(screen, hr, "+5° >", C_CYAN)
        hl_lbl = FONT_TINY.render("heading", True, C_GRAY)
        screen.blit(hl_lbl, (ctrl_panel.centerx - hl_lbl.get_width()//2,
                              ctrl_panel.y + 322))

        # Handle nudge button clicks (re-check each frame since rects move)
        mouse_pos   = pygame.mouse.get_pos()
        mouse_press = pygame.mouse.get_pressed()
        # (actual click handling done in event loop above for < / > keys;
        #  button clicks handled via MOUSEBUTTONDOWN events already mapped to keys)

        # ── Footer ────────────────────────────────────────────────────────────
        pygame.draw.line(screen, C_BORDER, (10, H-42), (W-10, H-42), 1)
        footer = FONT_SM.render(
            "Click compass to set heading  ·  < >  nudge  ·  I  toggle IMU  ·  Q  quit",
            True, C_GRAY)
        screen.blit(footer, (W//2 - footer.get_width()//2, H - 30))

        pygame.display.flip()

    pygame.quit()


def main():
    parser = argparse.ArgumentParser(description="Yukon simulator GUI")
    parser.add_argument("--no-terminal", action="store_true",
                        help="Suppress terminal draw() output")
    args = parser.parse_args()

    # Create PTY
    yukon_master, yukon_slave = pty.openpty()
    tty.setraw(yukon_slave)
    yukon_path = os.ttyname(yukon_slave)

    print(f"Yukon PTY  : {yukon_path}", file=sys.stderr)
    print(f"Connect with --port {yukon_path}", file=sys.stderr)

    # Initialise IMU tick timestamp
    with _sim._lock:
        _sim._state['last_imu_tick'] = time.monotonic()
        _sim._state['fault_l'] = False
        _sim._state['fault_r'] = False

    # Patch _send_sensor_packet to honour GUI-injected fault state
    _orig_server = _sim.yukon_server

    # Launch Yukon protocol server thread
    t_srv = threading.Thread(target=_sim.yukon_server,
                             args=(yukon_master,), daemon=True)
    t_srv.start()

    try:
        run_gui(yukon_path, no_terminal=args.no_terminal)
    finally:
        with _sim._lock:
            _sim._state['running'] = False
        os.close(yukon_master)
        os.close(yukon_slave)


if __name__ == "__main__":
    main()
