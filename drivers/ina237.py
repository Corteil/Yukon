"""INA237 power monitor — thin wrapper around adafruit-circuitpython-ina23x.

Install dependency:
    pip3 install adafruit-circuitpython-ina23x

Wiring (I2C1 on Pi 5):
    VCC  → 3.3 V  (pin 1)
    GND  → GND    (pin 6)
    SDA  → GPIO 2 (pin 3)
    SCL  → GPIO 3 (pin 5)
    VIN+ → supply +ve (before DC-DC converter)
    VIN- → supply −ve / GND
"""

from __future__ import annotations


class INA237:
    """INA237 / INA238 power monitor.

    Args:
        address:     I2C address (default 0x40, A0+A1 unconnected)
        r_shunt:     Shunt resistor in ohms (Adafruit breakout = 0.1 Ω)
        max_current: Maximum expected current in amps (sets ADC resolution)
    """

    def __init__(self, address: int = 0x40,
                 r_shunt: float = 0.1, max_current: float = 2.0) -> None:
        import board
        import adafruit_ina23x

        i2c = board.I2C()
        self._dev = adafruit_ina23x.INA23X(i2c, address=address)
        self._dev.set_calibration(r_shunt, max_current)

    @property
    def voltage(self) -> float:
        """Bus voltage in volts."""
        return self._dev.bus_voltage

    @property
    def current(self) -> float:
        """Current in amps."""
        return self._dev.current

    @property
    def power(self) -> float:
        """Power in watts."""
        return self._dev.power

    @property
    def temperature(self) -> float:
        """Die temperature in °C."""
        return self._dev.die_temperature

    def read_all(self) -> tuple[float, float, float]:
        """Return (voltage_V, current_A, power_W)."""
        return self._dev.bus_voltage, self._dev.current, self._dev.power
