# GPIO Pin Allocation — HackyRacingRobot

Raspberry Pi 5 — 40-pin header

---

## Allocated Pins

| GPIO | Pin | Function              | Direction | Notes                                      |
|------|-----|-----------------------|-----------|--------------------------------------------|
| 2    | 3   | I2C1 SDA              | Bidir     | Display + sensors (hardware I2C1)          |
| 3    | 5   | I2C1 SCL              | Out       | Display + sensors; 400 kHz fast mode       |
| 12   | 32  | LiDAR PWM             | Out       | Hardware PWM0 ch0; 30 kHz, 40% duty → LD06 motor |
| 14   | 8   | LiDAR UART0 TX        | Out       | `/dev/ttyAMA0`; reserved, unused by LD06   |
| 15   | 10  | LiDAR UART0 RX        | In        | `/dev/ttyAMA0`; LD06 data @ 230400 baud    |
| 17   | 11  | ESTOP reset button    | In        | Pull-up, active low, interrupt-driven      |
| 18   | 12  | I2S BCLK              | Out       | Audio bit clock                            |
| 19   | 35  | I2S LRCLK             | Out       | Audio word select (left/right clock)       |
| 20   | 38  | I2S RX (DOUT)         | In        | Audio data from DAC/ADC                    |
| 21   | 40  | I2S TX (DIN)          | Out       | Audio data to DAC/ADC                      |
| 22   | 15  | Status LED — ESTOP    | Out       | Red; active high                           |
| 23   | 16  | Status LED — AUTO     | Out       | Green; active high                         |
| 24   | 18  | Camera stereo sync    | Out       | Pulse to XVS pins on both IMX296 modules   |
| 25   | 22  | Status LED — MANUAL   | Out       | Blue or amber; active high                 |
| 27   | 13  | Start/stop button     | In        | Pull-up, active low                        |

---

## `/boot/firmware/config.txt` Overlays Required

```
# UART0 on GPIO 14/15 → /dev/ttyAMA0 (LD06 LiDAR)
dtoverlay=uart0-pi5

# PWM0 on GPIO 12 → LD06 motor speed control
dtoverlay=pwm,pin=12,func=4

# I2S audio
dtoverlay=hifiberry-dac          # replace with your actual audio overlay
```

> **Note:** GPIO 12 uses `func=4` (ALT4) for PWM on the Pi 5, not `func=2`.
> Verify with `pinctrl get 12` after applying the overlay.

---

## Pins to Avoid

| GPIO  | Reason                                              |
|-------|-----------------------------------------------------|
| 0, 1  | Reserved for HAT ID EEPROM                          |
| 4     | 1-Wire; leave free for future temperature sensors   |
| 6, 13 | Adjacent to PWM; avoid to minimise noise near LiDAR |

---

## Free GPIO (available for future expansion)

5, 7, 8, 9, 10, 11, 16, 26

---

## Off-header / USB Devices

These devices do **not** consume GPIO pins:

| Device              | Interface      | Path             |
|---------------------|----------------|------------------|
| Pimoroni Yukon      | USB serial     | `/dev/ttyACM0`   |
| Allystar TAU1308    | USB serial     | `/dev/ttyUSB0`   |
| IMX296 camera       | CSI / USB      | picamera2        |
| BNO085 IMU          | Yukon I2C      | via Yukon        |
| Tufty 2350          | USB / WiFi / BT| `/dev/ttyACM1` or wireless |

---

## Notes

- All buttons wired **active low**: connect between GPIO pin and GND; rely on Pi internal pull-up (`gpiod` `BIAS_PULL_UP`).
- Status LEDs: use a 330 Ω series resistor per LED to limit current to ~10 mA.
- I2C1 devices share GPIO 2/3; ensure all have unique 7-bit addresses. Use `i2cdetect -y 1` to scan.
- I2S overlay depends on the specific DAC/amp board used; replace `hifiberry-dac` with the correct overlay for your hardware.
