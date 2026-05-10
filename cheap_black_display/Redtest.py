from machine import Pin, SPI
import lcd_bus, axs15231b, lvgl as lv

Pin(1, Pin.OUT).value(1)

spi_bus = SPI.Bus(host=1, sck=47, quad_pins=(21, 48, 40, 39))
display_bus = lcd_bus.SPIBusFast(spi_bus=spi_bus, dc=8, cs=45,
    freq=40000000, spi_mode=3, quad=True)

display = axs15231b.AXS15231B(display_bus, 320, 480,
    backlight_pin=1,
    color_space=lv.COLOR_FORMAT.RGB565,
    rgb565_byte_swap=True,
    backlight_on_state=axs15231b.STATE_PWM)

display.set_power(True)
display.set_backlight(100)
display.init()

scr = lv.screen_active()
scr.set_style_bg_color(lv.color_hex(0xff0000), 0)
scr.invalidate()

import time
while True:
    lv.task_handler()
    time.sleep_ms(5)