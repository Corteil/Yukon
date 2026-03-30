"""
test_neopixel.py -- NeoPixel LED strip hardware test for Yukon RP2040.

Exercises the LEDStripModule on SLOT3 directly — no Pi serial protocol needed.
Useful for confirming wiring, colour order, and module power before flashing main.py.

Run:  mpremote run yukon_firmware_and_software/test_neopixel.py
"""

from pimoroni_yukon import Yukon, SLOT3
from pimoroni_yukon.modules import LEDStripModule
from utime import sleep_ms
import math

NUM_LEDS = 8

COLOURS = [
    ("off",     0,   0,   0  ),
    ("red",     255, 0,   0  ),
    ("green",   0,   255, 0  ),
    ("blue",    0,   0,   255),
    ("orange",  255, 80,  0  ),
    ("yellow",  255, 200, 0  ),
    ("cyan",    0,   255, 255),
    ("magenta", 255, 0,   255),
    ("white",   255, 255, 255),
]


# ── Yukon init ────────────────────────────────────────────────────────────────

yukon = Yukon()
strip_module = LEDStripModule()
yukon.register_with_slot(strip_module, SLOT3)
yukon.verify_and_initialise()
yukon.enable_main_output()
strip = strip_module.strip
print("Yukon + LEDStripModule on SLOT3 initialised OK")
print(f"Strip reports {strip.num_leds} LEDs\n")


# ── Helpers ───────────────────────────────────────────────────────────────────

def fill(r, g, b):
    for i in range(NUM_LEDS):
        strip.set_rgb(i, r, g, b)
    strip.update()


def clear():
    fill(0, 0, 0)


def hsv_to_rgb(h, s=1.0, v=1.0):
    """h in 0..360, s/v in 0..1 -> (r, g, b) 0..255."""
    h = h % 360
    hi = int(h / 60) % 6
    f  = (h / 60) - int(h / 60)
    p  = int(v * (1 - s) * 255)
    q  = int(v * (1 - f * s) * 255)
    t  = int(v * (1 - (1 - f) * s) * 255)
    vi = int(v * 255)
    return [(vi, t, p), (q, vi, p), (p, vi, t), (p, q, vi), (t, p, vi), (vi, p, q)][hi]


# ── Test 1: Colour presets ────────────────────────────────────────────────────

print("=== 1. Colour presets (0.5 s each) ===")
for name, r, g, b in COLOURS:
    print(f"  {name:10s}  rgb({r:3d}, {g:3d}, {b:3d})")
    fill(r, g, b)
    sleep_ms(500)
clear()
sleep_ms(300)


# ── Test 2: Individual pixels ─────────────────────────────────────────────────

print("\n=== 2. Individual pixels (rainbow chase) ===")
hues = [i * (360 // NUM_LEDS) for i in range(NUM_LEDS)]
for _ in range(NUM_LEDS * 2):
    for i in range(NUM_LEDS):
        r, g, b = hsv_to_rgb(hues[i])
        strip.set_rgb(i, r, g, b)
    strip.update()
    hues = [(h + 20) % 360 for h in hues]
    sleep_ms(80)
clear()
sleep_ms(300)


# ── Test 3: Larson scanner ────────────────────────────────────────────────────

print("\n=== 3. Larson scanner (2 s) ===")
pos = 0
direction = 1
t_end = 2000
elapsed = 0
while elapsed < t_end:
    for i in range(NUM_LEDS):
        d = abs(i - pos)
        if d == 0:
            strip.set_rgb(i, 255, 0, 0)
        elif d == 1:
            strip.set_rgb(i, 48, 0, 0)
        elif d == 2:
            strip.set_rgb(i, 8, 0, 0)
        else:
            strip.set_rgb(i, 0, 0, 0)
    strip.update()
    pos += direction
    if pos >= NUM_LEDS - 1:
        direction = -1
    elif pos <= 0:
        direction = 1
    sleep_ms(60)
    elapsed += 60
clear()
sleep_ms(300)


# ── Test 4: Brightness ramp ───────────────────────────────────────────────────

print("\n=== 4. Brightness ramp — white (up then down) ===")
for level in list(range(0, 256, 8)) + list(range(255, -1, -8)):
    fill(level, level, level)
    sleep_ms(10)
clear()
sleep_ms(300)


# ── Test 5: Converge ──────────────────────────────────────────────────────────

print("\n=== 5. Converge (2 s) ===")
elapsed = 0
step = 0
while elapsed < 2000:
    mid = NUM_LEDS // 2
    for i in range(NUM_LEDS):
        dist = abs(i - mid + 0.5)
        if abs(dist - step % mid) < 1:
            strip.set_rgb(i, 0, 200, 255)
        else:
            strip.set_rgb(i, 0, 0, 0)
    strip.update()
    step += 1
    sleep_ms(80)
    elapsed += 80
clear()
sleep_ms(300)


# ── Test 6: Alternating pixels ────────────────────────────────────────────────

print("\n=== 6. Alternating red / blue (4 blinks) ===")
for phase in range(4):
    for i in range(NUM_LEDS):
        if (i + phase) % 2 == 0:
            strip.set_rgb(i, 255, 0, 0)
        else:
            strip.set_rgb(i, 0, 0, 255)
    strip.update()
    sleep_ms(400)
clear()
sleep_ms(300)


# ── Done ──────────────────────────────────────────────────────────────────────

print("\nAll tests done. Strip cleared.")
yukon.disable_main_output()
