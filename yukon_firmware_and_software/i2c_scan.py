"""
i2c_scan.py — I2C bus scanner for the Pimoroni Yukon.

Scans the Yukon's built-in Qw/ST I2C bus (I2C0, 400 kHz) and any additional
buses you configure below.  Run this to find device addresses before writing
drivers, or to verify wiring.

Upload to the Yukon with Thonny and run directly (do not save as main.py).

Wiring (Qw/ST port):
  Device VIN → 3.3 V
  Device GND → GND
  Device SDA → Qw/ST SDA
  Device SCL → Qw/ST SCL
"""

from pimoroni_yukon import Yukon
from machine import I2C, Pin
from utime import sleep_ms

# ---------------------------------------------------------------------------
# Known I2C device addresses — add your own as needed
# ---------------------------------------------------------------------------
KNOWN_DEVICES = {
    0x18: "LIS3DH accelerometer",
    0x19: "LIS3DH accelerometer (alt)",
    0x1C: "MMA8452 accelerometer",
    0x1D: "ADXL345 accelerometer",
    0x20: "MCP23017 GPIO expander",
    0x23: "BH1750 light sensor",
    0x28: "BNO055 IMU",
    0x29: "VL53L0X ToF / TCS34725 colour",
    0x38: "AHT10/AHT20 humidity",
    0x3C: "SSD1306 OLED (128x64)",
    0x3D: "SSD1306 OLED (128x32, alt)",
    0x40: "INA219 power monitor / PCA9685 PWM",
    0x41: "INA219 (alt)",
    0x44: "SHT30/SHT31 humidity",
    0x48: "ADS1015/ADS1115 ADC / TMP102 temp",
    0x49: "ADS1115 (alt) / AS7341 spectral",
    0x4A: "BNO085 IMU / ADS1015 (alt)",
    0x4B: "BNO085 IMU (alt addr) / ADS1115 (alt)",
    0x53: "ADXL345 accelerometer (alt)",
    0x57: "MAX3010x pulse-ox EEPROM",
    0x5A: "MPR121 capacitive touch / MLX90614 IR",
    0x5B: "MPR121 (alt)",
    0x60: "MPL3115A2 pressure / Si5351 clock",
    0x68: "MPU-6050 IMU / DS3231 RTC",
    0x69: "MPU-6050 (alt) / ICM-20649",
    0x6A: "LSM6DS IMU",
    0x6B: "LSM6DS IMU (alt)",
    0x70: "HT16K33 LED matrix",
    0x76: "BME280/BMP280 pressure+humidity",
    0x77: "BME280/BMP280 (alt) / BMP180",
}

# ---------------------------------------------------------------------------
# Extra I2C buses to scan in addition to yukon.i2c
# Uncomment and edit if you have devices on other pins.
# ---------------------------------------------------------------------------
EXTRA_BUSES = [
    # {"label": "I2C1 (GP6/GP7)", "sda": 6, "scl": 7, "freq": 400_000},
]

# ---------------------------------------------------------------------------

def scan_bus(i2c, label):
    """Scan one I2C bus and print results."""
    print(f"\n{'='*52}")
    print(f"  {label}")
    print(f"{'='*52}")

    try:
        found = i2c.scan()
    except Exception as e:
        print(f"  ERROR scanning bus: {e}")
        return []

    if not found:
        print("  No devices found.")
        return []

    print(f"  Found {len(found)} device(s):\n")
    print(f"  {'Address':>12}   Description")
    print(f"  {'-'*12}   {'-'*32}")

    for addr in sorted(found):
        desc = KNOWN_DEVICES.get(addr, "unknown device")
        print(f"  0x{addr:02X}  ({addr:3d})   {desc}")

    return found


def main():
    yukon = Yukon()

    print("\nYukon I2C Scanner")
    print("Press BOOT to exit at any time.\n")

    sleep_ms(200)   # allow devices to power up

    all_found = {}

    # Scan the built-in Qw/ST bus
    found = scan_bus(yukon.i2c, "Qw/ST port  (I2C0, 400 kHz)")
    if found:
        all_found["Qw/ST"] = found

    # Scan any extra buses
    for bus_cfg in EXTRA_BUSES:
        try:
            bus = I2C(
                1,
                sda=Pin(bus_cfg["sda"]),
                scl=Pin(bus_cfg["scl"]),
                freq=bus_cfg.get("freq", 400_000),
            )
            found = scan_bus(bus, bus_cfg["label"])
            if found:
                all_found[bus_cfg["label"]] = found
        except Exception as e:
            print(f"\nCould not initialise {bus_cfg['label']}: {e}")

    # Summary
    total = sum(len(v) for v in all_found.values())
    print(f"\n{'='*52}")
    print(f"  Scan complete — {total} device(s) found across {len(all_found)} bus(es)")
    print(f"{'='*52}\n")

    yukon.reset()


main()
