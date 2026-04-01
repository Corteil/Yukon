"""
test_bench_power.py — BenchPowerModule diagnostic for Yukon RP2040

Run this directly on the Yukon (upload via Thonny, run as main or execute in REPL).
Steps through voltages from 1 V to 15 V in 0.5 V increments, printing the
set-point and reading temperature/power-good each time.  Measure the actual
output voltage with a multimeter to find the real scaling factor.

Press Ctrl-C to stop.
"""

from utime import sleep
from pimoroni_yukon import Yukon, SLOT1
from pimoroni_yukon.modules import BenchPowerModule

SLOT = SLOT1   # change if your bench module is in a different slot

yukon = Yukon()
bench = BenchPowerModule()

print("Registering BenchPowerModule in", SLOT)
yukon.register_with_slot(bench, SLOT)
yukon.verify_and_initialise(allow_unregistered=True)

print("Enabling main output (battery supply)...")
yukon.enable_main_output()
sleep(0.5)

print("Enabling bench power module...")
bench.enable()
sleep(0.2)

print()
print("Sweeping set-point voltages.  Measure output with a multimeter.")
print(f"{'Set (V)':>10}  {'Pwr Good':>10}  {'Temp (C)':>10}")
print("-" * 36)

try:
    # Start low and work up so you can spot where output tracks the set-point
    v = 1.0
    while v <= 15.5:
        bench.set_voltage(v)
        sleep(0.5)  # settle
        try:
            pwr_good = bench.read_power_good()
            temp     = bench.read_temperature()
        except Exception as e:
            pwr_good = "ERR"
            temp     = str(e)
        print(f"{v:10.1f}  {str(pwr_good):>10}  {str(temp):>10}")
        v += 0.5

    print()
    print("Sweep complete.  Leaving at last set-point.")
    print("Press Ctrl-C to disable and exit.")
    while True:
        sleep(1)

except KeyboardInterrupt:
    print("\nDisabling bench module and main output.")
    bench.disable()
    yukon.disable_main_output()
    print("Done.")
