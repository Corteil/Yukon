import _thread
from utime import sleep
import sys
import select
from pimoroni_yukon import Yukon, SLOT2, SLOT5
from pimoroni_yukon.modules import DualMotorModule
from pimoroni_yukon.timing import ticks_ms, ticks_add
from pimoroni_yukon.errors import FaultError

# Hardware constants
LED_A = 'A'
LED_B = 'B'
UPDATES = 50
CURRENT_LIMIT = 2
SENSOR_PERIOD = 1000   # ms between sensor reports

# Protocol: 5-byte packets, all bytes are printable ASCII — no REPL interference
#   Host→Device  [SYNC, CMD, V_HIGH, V_LOW, CHK]
#   SYNC   = 0x7E (126, '~') — unique, never appears in other fields
#   CMD    = cmd_code + 0x20  → 0x21-0x25 (33-37, '!' to '%')
#   V_HIGH = (value >> 4) + 0x40 → 0x40-0x4F (64-79, '@' to 'O')
#   V_LOW  = (value & 0xF) + 0x50 → 0x50-0x5F (80-95, 'P' to '_')
#   CHK    = CMD ^ V_HIGH ^ V_LOW  (printable, never equals SYNC)
#   Response: ACK (0x06) on success, NAK (0x15) on any error
#
#   CMD_SENSOR response (Device→Host): N data packets then ACK
#   Data packet: [SYNC, RESP_TYPE, V_HIGH, V_LOW, CHK]
#   RESP_TYPE = sensor_id + 0x30 → 0x30-0x36 ('0' to '6')
#   CHK       = RESP_TYPE ^ V_HIGH ^ V_LOW → 0x20-0x2F (never equals SYNC/ACK/NAK)
SYNC = 0x7E
ACK  = 0x06
NAK  = 0x15

CMD_LED    = 1   # value: 0=LED_A off, 1=LED_A on, 2=LED_B off, 3=LED_B on
CMD_LEFT   = 2   # value: motor speed byte
CMD_RIGHT  = 3
CMD_KILL   = 4   # value: ignored
CMD_SENSOR = 5   # value: ignored; replies with sensor data packets then ACK

# Sensor data packet RESP_TYPE IDs (resp_id + 0x30)
RESP_VOLTAGE = 0   # input voltage × 10  (e.g. 11.2 V → 112)
RESP_CURRENT = 1   # current × 100       (e.g. 1.21 A → 121)
RESP_TEMP    = 2   # board temp × 3      (e.g. 22.5 °C → 67)
RESP_TEMP_L  = 3   # left module temp × 3
RESP_TEMP_R  = 4   # right module temp × 3
RESP_FAULT_L = 5   # left fault  (0 or 1)
RESP_FAULT_R = 6   # right fault (0 or 1)

# Shared motor state, protected by a lock
_lock = _thread.allocate_lock()
_left_speed = 0.0
_right_speed = 0.0
_running = True

def _ack():
    print(chr(ACK))   # trailing \n triggers MicroPython stdout flush; host ignores it


def _nak():
    print(chr(NAK))


def _send_data(resp_id, value):
    """Send one sensor data packet (device→host)."""
    value   = max(0, min(255, int(value)))
    rtype   = resp_id + 0x30
    v_high  = (value >> 4) + 0x40
    v_low   = (value & 0x0F) + 0x50
    chk     = rtype ^ v_high ^ v_low
    sys.stdout.write(chr(SYNC) + chr(rtype) + chr(v_high) + chr(v_low) + chr(chk))


def _decode_speed(byte_val):
    if byte_val > 100:
        return -((byte_val - 100) / 100)
    return byte_val / 100


def motor_core(module_left, module_right):
    """Core 1: continuously apply latest motor speeds."""
    global _running
    while _running:
        _lock.acquire()
        left = _left_speed
        right = _right_speed
        _lock.release()

        for motor in module_left.motors:
            motor.speed(left)
        for motor in module_right.motors:
            motor.speed(-right)

        sleep(0.02)


# Hardware setup
yukon = Yukon()
module2 = DualMotorModule()   # left motors
module5 = DualMotorModule()   # right motors

yukon.set_led(LED_A, False)
yukon.set_led(LED_B, False)

try:
    yukon.register_with_slot(module2, SLOT2)
    yukon.register_with_slot(module5, SLOT5)
    yukon.verify_and_initialise()
    yukon.enable_main_output()

    module2.set_current_limit(CURRENT_LIMIT)
    module2.enable()
    for motor in module2.motors:
        motor.enable()
    module5.set_current_limit(CURRENT_LIMIT)
    module5.enable()
    for motor in module5.motors:
        motor.enable()

    sleep(0.1)  # Let motor drivers settle before monitoring starts

    # Launch motor control loop on core 1
    _thread.start_new_thread(motor_core, (module2, module5))

    # Core 0: serial comms + Yukon monitoring
    current_time  = ticks_ms()
    last_sensor   = ticks_ms()
    state = 'SYNC'
    pkt_cmd   = 0
    pkt_vhigh = 0
    pkt_vlow  = 0

    while not yukon.is_boot_pressed():
        ready = select.select([sys.stdin], [], [], 0.01)
        if ready[0]:
            ch = sys.stdin.read(1)
            b = ord(ch)

            # SYNC byte (0x7E) always restarts framing, in any state
            if b == SYNC:
                state = 'CMD'
                continue

            if state == 'CMD':
                if 0x21 <= b <= 0x25:
                    pkt_cmd = b
                    state = 'V_HIGH'
                else:
                    _nak()
                    state = 'SYNC'

            elif state == 'V_HIGH':
                if 0x40 <= b <= 0x4F:
                    pkt_vhigh = b
                    state = 'V_LOW'
                else:
                    _nak()
                    state = 'SYNC'

            elif state == 'V_LOW':
                if 0x50 <= b <= 0x5F:
                    pkt_vlow = b
                    state = 'CHK'
                else:
                    _nak()
                    state = 'SYNC'

            elif state == 'CHK':
                expected_chk = pkt_cmd ^ pkt_vhigh ^ pkt_vlow
                if b != expected_chk:
                    _nak()
                else:
                    cmd_code = pkt_cmd - 0x20
                    value    = ((pkt_vhigh - 0x40) << 4) | (pkt_vlow - 0x50)

                    if cmd_code == CMD_LED:
                        if value == 0:
                            yukon.set_led(LED_A, False)
                        elif value == 1:
                            yukon.set_led(LED_A, True)
                        elif value == 2:
                            yukon.set_led(LED_B, False)
                        elif value == 3:
                            yukon.set_led(LED_B, True)

                    elif cmd_code == CMD_LEFT:
                        speed = _decode_speed(value)
                        _lock.acquire()
                        _left_speed = speed
                        _lock.release()

                    elif cmd_code == CMD_RIGHT:
                        speed = _decode_speed(value)
                        _lock.acquire()
                        _right_speed = speed
                        _lock.release()

                    elif cmd_code == CMD_KILL:
                        _lock.acquire()
                        _left_speed = 0.0
                        _right_speed = 0.0
                        _lock.release()

                    elif cmd_code == CMD_SENSOR:
                        try:
                            _send_data(RESP_VOLTAGE, yukon.read_input_voltage() * 10)
                            _send_data(RESP_CURRENT, yukon.read_current() * 100)
                            _send_data(RESP_TEMP,    yukon.read_temperature() * 3)
                            _send_data(RESP_TEMP_L,  module2.read_temperature() * 3)
                            _send_data(RESP_TEMP_R,  module5.read_temperature() * 3)
                            _send_data(RESP_FAULT_L, int(module2.read_fault()))
                            _send_data(RESP_FAULT_R, int(module5.read_fault()))
                        except Exception as se:
                            print(f"Sensor error: {se}")
                            _nak()
                            state = 'SYNC'
                            continue

                    _ack()

                state = 'SYNC'

        current_time = ticks_add(current_time, int(1000 / UPDATES))
        try:
            yukon.monitor_until_ms(current_time)
        except FaultError as e:
            print(f"Fault: {e} — recovering")
            _lock.acquire()
            _left_speed = 0.0
            _right_speed = 0.0
            _lock.release()
            try:
                sleep(0.05)
                yukon.enable_main_output()
                module2.enable()
                module5.enable()
            except Exception as e2:
                print(f"Recovery error: {e2}")
            current_time = ticks_ms()
        except Exception as e:
            print(f"Unexpected error: {e}")
            current_time = ticks_ms()

        now = ticks_ms()
        if now - last_sensor >= SENSOR_PERIOD:
            last_sensor = now
            try:
                v  = yukon.read_input_voltage()
                i  = yukon.read_current()
                t  = yukon.read_temperature()
                tL = module2.read_temperature()
                tR = module5.read_temperature()
                fL = module2.read_fault()
                fR = module5.read_fault()
                print(f"SENS v={v:.2f} i={i:.3f} t={t:.1f} tL={tL:.1f} tR={tR:.1f} fL={int(fL)} fR={int(fR)}")
            except Exception as se:
                print(f"Sensor error: {se}")

finally:
    _running = False
    print("Shutting down")
    yukon.reset()
