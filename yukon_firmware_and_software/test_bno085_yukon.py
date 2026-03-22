"""
test_bno085_yukon.py — BNO085 driver tests, runs directly on the Yukon.

Upload this file AND lib/bno085.py to the Yukon with Thonny, then run it
from the Thonny shell.  Do NOT save as main.py.

Tests:
  1. I2C scan — BNO085 visible at expected address
  2. Driver init — BNO085() constructs without error
  3. Data flow — update() delivers fresh quaternion data within 500 ms
  4. Heading range — heading() returns a value in 0.0–360.0
  5. Live display — rotating heading bar, press BOOT to exit

Wiring (Adafruit BNO085 breakout → Yukon Qw/ST port):
  VIN → 3.3 V    GND → GND    SDA → Qw/ST SDA    SCL → Qw/ST SCL
  DI unconnected → I2C address 0x4A  (tie DI HIGH for 0x4B)
"""

from pimoroni_yukon import Yukon
from utime import sleep_ms, ticks_ms, ticks_diff
import sys

# Change to 0x4B if DI pin is tied HIGH on the breakout
BNO085_ADDR = 0x4A

# ---------------------------------------------------------------------------

yukon = Yukon()

passed = 0
failed = 0

def _result(ok, name, detail=""):
    global passed, failed
    if ok:
        passed += 1
        print(f"  PASS  {name}")
    else:
        failed += 1
        print(f"  FAIL  {name}" + (f" — {detail}" if detail else ""))
    return ok


# ---------------------------------------------------------------------------
# Test 1: I2C scan
# ---------------------------------------------------------------------------
print("\n── Test 1: I2C scan ──────────────────────────────────")

found = yukon.i2c.scan()
addr_found = BNO085_ADDR in found
_result(addr_found,
        f"BNO085 visible at 0x{BNO085_ADDR:02X}",
        f"devices on bus: {[hex(a) for a in found]}")

if not addr_found:
    print("\nNo BNO085 found — check wiring and address.  Aborting.\n")
    yukon.reset()
    sys.exit(1)


# ---------------------------------------------------------------------------
# Test 2: Driver init
# ---------------------------------------------------------------------------
print("\n── Test 2: Driver init ───────────────────────────────")

imu = None
try:
    from bno085 import BNO085
    imu = BNO085(yukon.i2c, addr=BNO085_ADDR)
    _result(True, "BNO085() constructed without error")
except Exception as e:
    _result(False, "BNO085() constructed without error", str(e))
    print("\nDriver init failed — aborting.\n")
    yukon.reset()
    sys.exit(1)


# ---------------------------------------------------------------------------
# Test 3: Data flow — quaternion updates within 500 ms
# ---------------------------------------------------------------------------
print("\n── Test 3: Data flow ─────────────────────────────────")

deadline = ticks_ms() + 500
data_arrived = False

while ticks_diff(deadline, ticks_ms()) > 0:
    imu.update()
    if imu._data_received:
        data_arrived = True
        break
    sleep_ms(10)

_result(data_arrived, "Quaternion updated within 500 ms",
        "no data — is IMU powered and report interval <= 100 ms?")


# ---------------------------------------------------------------------------
# Test 4: Heading in range
# ---------------------------------------------------------------------------
print("\n── Test 4: Heading range ─────────────────────────────")

# Collect 20 readings and verify every one is in [0, 360)
out_of_range = []
for _ in range(20):
    imu.update()
    h = imu.heading()
    if not (0.0 <= h < 360.0):
        out_of_range.append(h)
    sleep_ms(25)

_result(len(out_of_range) == 0,
        "All 20 heading samples in [0, 360)",
        f"bad values: {out_of_range}")

# ---------------------------------------------------------------------------
# Test 5: Heading stability — 1 second of readings, std-dev sanity check
# ---------------------------------------------------------------------------
print("\n── Test 5: Heading stability (1 s, still sensor) ────")

samples = []
t_end = ticks_ms() + 1000
while ticks_diff(t_end, ticks_ms()) > 0:
    imu.update()
    samples.append(imu.heading())
    sleep_ms(20)

if len(samples) >= 5:
    mean = sum(samples) / len(samples)
    # Variance using circular-safe differences relative to first sample
    ref = samples[0]
    diffs = [((s - ref + 180) % 360 - 180) for s in samples]
    variance = sum(d * d for d in diffs) / len(diffs)
    std_dev = variance ** 0.5
    # A stationary IMU on a stable surface should drift < 5° over 1 s
    _result(std_dev < 5.0,
            f"Heading stable over 1 s (std dev {std_dev:.2f}° < 5°)",
            f"std dev = {std_dev:.2f}° — sensor may be moving or noisy")
else:
    _result(False, "Heading stability", "not enough samples")


# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
total = passed + failed
print(f"\n{'='*52}")
print(f"  Results: {passed}/{total} passed" +
      (f"  ({failed} FAILED)" if failed else "  — all passed"))
print(f"{'='*52}")


# ---------------------------------------------------------------------------
# Live heading display (press BOOT to exit)
# ---------------------------------------------------------------------------
print("\nLive heading display — rotate the sensor, press BOOT to exit.\n")

BAR_WIDTH = 40

def _compass_bar(heading):
    """40-char bar with a marker showing heading direction."""
    pos = int(heading / 360.0 * BAR_WIDTH) % BAR_WIDTH
    bar = ['-'] * BAR_WIDTH
    bar[pos] = '|'
    return ''.join(bar)

def _cardinal(h):
    dirs = ["N", "NE", "E", "SE", "S", "SW", "W", "NW", "N"]
    return dirs[int((h + 22.5) / 45) % 8]

try:
    while not yukon.is_boot_pressed():
        imu.update()
        h = imu.heading()
        bar = _compass_bar(h)
        card = _cardinal(h)
        # Overwrite same line
        sys.stdout.write(f"\r  [{bar}]  {h:6.1f}°  {card}   ")
        sleep_ms(50)
except KeyboardInterrupt:
    pass

print("\n\nDone.")
yukon.reset()
