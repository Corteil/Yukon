"""
test_bench_power.py — BenchPowerModule diagnostic for Yukon RP2040

Registers ALL modules (matching main.py slot assignments) before enabling,
so the Yukon does not reject initialisation when all slots are populated.

Slot layout:
  SLOT4 — BenchPowerModule
  SLOT2 — DualMotorModule  (left motors)
  SLOT3 — LEDStripModule   (NeoPixel, 8 LEDs)
  SLOT5 — DualMotorModule  (right motors)

Steps through voltages from 1 V to 15 V in 0.5 V increments, printing the
set-point and reading temperature/power-good each time.  Measure the actual
output voltage with a multimeter to verify.

Press Ctrl-C to stop.
"""

from utime import sleep
from pimoroni_yukon import Yukon, SLOT2, SLOT3, SLOT4, SLOT5
from pimoroni_yukon.modules import BenchPowerModule, DualMotorModule, LEDStripModule

NUM_LEDS = 8

yukon      = Yukon()
bench      = BenchPowerModule()
led_strip  = LEDStripModule(LEDStripModule.NEOPIXEL, pio=0, sm=0, num_leds=NUM_LEDS)
motors_l   = DualMotorModule()   # SLOT2 — left motors
motors_r   = DualMotorModule()   # SLOT5 — right motors

print("Registering all modules...")
yukon.register_with_slot(bench,     SLOT4)
yukon.register_with_slot(motors_l,  SLOT2)
yukon.register_with_slot(led_strip, SLOT3)
yukon.register_with_slot(motors_r,  SLOT5)

yukon.verify_and_initialise()

print("Enabling main output (battery supply)...")
yukon.enable_main_output()
sleep(0.5)

print("Enabling all modules...")
motors_l.enable()
motors_r.enable()
led_strip.enable()
bench.enable()          # enable() MUST come before set_voltage()
sleep(0.5)

print()
print("Sweeping set-point voltages.  Measure output with a multimeter.")
print(f"{'Set (V)':>10}  {'Pwr Good':>10}  {'Temp (C)':>10}")
print("-" * 36)

try:
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
    print("\nDisabling all modules and main output.")
    bench.disable()
    motors_l.disable()
    motors_r.disable()
    led_strip.disable()
    yukon.disable_main_output()
    print("Done.")
