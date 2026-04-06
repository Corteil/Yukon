"""
test_dual_output.py — DualOutputModule diagnostic for Yukon RP2040

Run directly in Thonny (NOT alongside main.py).

Registers all modules matching main.py slot layout, then toggles both
outputs of the Dual Switched Output module in SLOT4, printing power-good
status each iteration so you can see which physical terminal responds.

Slot layout:
  SLOT4 — DualOutputModule  (dual switched output)
  SLOT2 — DualMotorModule   (left motors)
  SLOT3 — LEDStripModule    (NeoPixel, 8 LEDs)
  SLOT5 — DualMotorModule   (right motors)

Press Ctrl-C to stop and disable all outputs.
"""

from utime import sleep
from pimoroni_yukon import Yukon, SLOT2, SLOT3, SLOT4, SLOT5
from pimoroni_yukon.modules import DualOutputModule, DualMotorModule, LEDStripModule

NUM_LEDS = 8

yukon    = Yukon()
dual_out = DualOutputModule()
led      = LEDStripModule(LEDStripModule.NEOPIXEL, pio=0, sm=0, num_leds=NUM_LEDS)
motors_l = DualMotorModule()
motors_r = DualMotorModule()

print("Registering modules...")
yukon.register_with_slot(dual_out, SLOT4)
yukon.register_with_slot(motors_l, SLOT2)
yukon.register_with_slot(led,      SLOT3)
yukon.register_with_slot(motors_r, SLOT5)

yukon.verify_and_initialise()
print("Verified OK")

yukon.enable_main_output()
sleep(0.5)

motors_l.enable()
motors_r.enable()
led.enable()

print()
print("Testing DualOutputModule outputs.")
print(f"{'Output':>8}  {'SW_EN':>6}  {'OUT_PIN':>8}  {'PGood1':>8}  {'PGood2':>8}  {'Temp':>6}")
print("-" * 56)

def read_status():
    try:
        pg1 = dual_out.read_power_good1()
        pg2 = dual_out.read_power_good2()
        t   = dual_out.read_temperature()
    except Exception as e:
        return str(e), str(e), str(e)
    return pg1, pg2, round(t, 1)

try:
    while True:
        # --- Test output 0 ON, output 1 OFF ---
        dual_out.enable(0)
        dual_out.outputs[0].value(True)
        dual_out.disable(1)
        dual_out.outputs[1].value(False)
        sleep(0.2)
        pg1, pg2, t = read_status()
        print(f"{'OUT0':>8}  {'True':>6}  {'True':>8}  {str(pg1):>8}  {str(pg2):>8}  {str(t):>6}")
        sleep(1.8)

        # --- Both OFF ---
        dual_out.outputs[0].value(False)
        dual_out.disable(0)
        dual_out.outputs[1].value(False)
        dual_out.disable(1)
        sleep(0.2)
        pg1, pg2, t = read_status()
        print(f"{'OFF':>8}  {'False':>6}  {'False':>8}  {str(pg1):>8}  {str(pg2):>8}  {str(t):>6}")
        sleep(0.8)

        # --- Test output 1 ON, output 0 OFF ---
        dual_out.enable(1)
        dual_out.outputs[1].value(True)
        dual_out.disable(0)
        dual_out.outputs[0].value(False)
        sleep(0.2)
        pg1, pg2, t = read_status()
        print(f"{'OUT1':>8}  {'True':>6}  {'True':>8}  {str(pg1):>8}  {str(pg2):>8}  {str(t):>6}")
        sleep(1.8)

        # --- Both OFF ---
        dual_out.outputs[0].value(False)
        dual_out.disable(0)
        dual_out.outputs[1].value(False)
        dual_out.disable(1)
        sleep(0.2)
        pg1, pg2, t = read_status()
        print(f"{'OFF':>8}  {'False':>6}  {'False':>8}  {str(pg1):>8}  {str(pg2):>8}  {str(t):>6}")
        sleep(0.8)

except KeyboardInterrupt:
    print("\nDisabling all outputs.")
    dual_out.outputs[0].value(False)
    dual_out.outputs[1].value(False)
    dual_out.disable()
    motors_l.disable()
    motors_r.disable()
    led.disable()
    yukon.disable_main_output()
    print("Done.")
