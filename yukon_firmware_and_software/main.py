import _thread
from utime import sleep, sleep_ms
import sys
import select

from pimoroni_yukon import Yukon, SLOT2, SLOT5
from pimoroni_yukon.modules import DualMotorModule
from pimoroni_yukon.timing import ticks_ms, ticks_add
from pimoroni_yukon.errors import FaultError

# Hardware constants
LED_A = 'A'
LED_B = 'B'
UPDATES      = 50
CURRENT_LIMIT = 2
SENSOR_PERIOD = 1000   # ms between periodic sensor log lines

# Bearing-hold proportional gain.
# Correction = BEARING_KP * (error_degrees / 180).  Max correction = BEARING_KP.
# Tune upward if the robot drifts; tune downward if it oscillates.
BEARING_KP = 0.4

# ── Serial protocol ───────────────────────────────────────────────────────────
#
# Host → Device  [SYNC, CMD, V_HIGH, V_LOW, CHK]
#   SYNC   = 0x7E  ('~')       — unique framing byte, never in other fields
#   CMD    = cmd_code + 0x20   → 0x21–0x26  ('!' to '&')
#   V_HIGH = (value >> 4) + 0x40 → 0x40–0x4F  ('@' to 'O')
#   V_LOW  = (value & 0xF) + 0x50 → 0x50–0x5F  ('P' to '_')
#   CHK    = CMD ^ V_HIGH ^ V_LOW  (never equals SYNC)
#
# Device → Host  ACK (0x06) on success, NAK (0x15) on error.
#
# CMD_SENSOR response (Device → Host): N data packets then ACK
#   Data packet: [SYNC, RESP_TYPE, V_HIGH, V_LOW, CHK]
#   RESP_TYPE = sensor_id + 0x30  → 0x30–0x37  ('0' to '7')
#   CHK = RESP_TYPE ^ V_HIGH ^ V_LOW
#
# ─────────────────────────────────────────────────────────────────────────────

SYNC = 0x7E
ACK  = 0x06
NAK  = 0x15

CMD_LED     = 1   # value: 0=LED_A off, 1=LED_A on, 2=LED_B off, 3=LED_B on
CMD_LEFT    = 2   # value: motor speed byte (see _decode_speed)
CMD_RIGHT   = 3
CMD_KILL    = 4   # value: ignored — zeros both motors and clears bearing hold
CMD_SENSOR  = 5   # value: ignored — replies with sensor data packets then ACK
CMD_BEARING = 6   # value: 0–254 = target bearing (encoded 0–359°), 255 = disable

# CMD_SENSOR response IDs
RESP_VOLTAGE = 0   # input voltage × 10          (e.g. 11.2 V → 112)
RESP_CURRENT = 1   # current × 100               (e.g. 1.21 A → 121)
RESP_TEMP    = 2   # board temp × 3              (e.g. 22.5 °C → 67)
RESP_TEMP_L  = 3   # left module temp × 3
RESP_TEMP_R  = 4   # right module temp × 3
RESP_FAULT_L = 5   # left fault  (0 or 1)
RESP_FAULT_R = 6   # right fault (0 or 1)
RESP_HEADING = 7   # IMU heading, same encoding as CMD_BEARING (value * 254/359)
                   # Value 255 means IMU not available / heading unknown.

# ── Shared state (protected by _lock) ────────────────────────────────────────

_lock            = _thread.allocate_lock()
_left_speed      = 0.0
_right_speed     = 0.0
_bearing_target  = None    # None = disabled; float 0–360 = active target
_current_heading = 0.0     # updated each loop by imu.update() on core 0
_imu_ok          = False   # set True once IMU initialises successfully
_running         = True


# ── Protocol helpers ──────────────────────────────────────────────────────────

def _ack():
    sys.stdout.write(chr(ACK))

def _nak():
    sys.stdout.write(chr(NAK))

def _send_data(resp_id, value):
    """Send one sensor data packet (Device → Host)."""
    value  = max(0, min(255, int(value)))
    rtype  = resp_id + 0x30
    v_high = (value >> 4)   + 0x40
    v_low  = (value & 0x0F) + 0x50
    chk    = rtype ^ v_high ^ v_low
    sys.stdout.write(chr(SYNC) + chr(rtype) + chr(v_high) + chr(v_low) + chr(chk))

def _decode_speed(byte_val):
    """Decode motor speed byte → -1.0 .. +1.0."""
    if byte_val > 100:
        return -((byte_val - 100) / 100.0)
    return byte_val / 100.0

def _bearing_encode(degrees):
    """Encode degrees 0–359 → byte 0–254 (255 reserved for 'disable')."""
    return min(254, int(degrees * 254.0 / 359.0))

def _bearing_decode(value):
    """Decode protocol byte 0–254 → degrees 0–359."""
    return value * 359.0 / 254.0

def _angle_diff(target, current):
    """Signed shortest-arc difference (target − current), range −180..+180."""
    return (target - current + 180.0) % 360.0 - 180.0


# ── Core 1: motor drive loop ──────────────────────────────────────────────────

def motor_core(module_left, module_right):
    """Continuously apply the latest motor speeds with optional bearing hold."""
    global _running
    while _running:
        _lock.acquire()
        left    = _left_speed
        right   = _right_speed
        target  = _bearing_target
        heading = _current_heading
        _lock.release()

        if target is not None:
            error      = _angle_diff(target, heading)
            correction = max(-BEARING_KP, min(BEARING_KP,
                             BEARING_KP * error / 180.0))
            left  = max(-1.0, min(1.0, left  + correction))
            right = max(-1.0, min(1.0, right - correction))

        for motor in module_left.motors:
            motor.speed(left)
        for motor in module_right.motors:
            motor.speed(-right)

        sleep_ms(20)


# ── Hardware setup ────────────────────────────────────────────────────────────

yukon   = Yukon()
module2 = DualMotorModule()   # left motors  — SLOT2
module5 = DualMotorModule()   # right motors — SLOT5

yukon.set_led(LED_A, False)
yukon.set_led(LED_B, False)

# IMU setup (optional — bearing hold and heading reporting disabled if absent)
imu = None
try:
    from bno085 import BNO085
    imu    = BNO085(yukon.i2c)
    _imu_ok = True
    print("IMU OK")
except Exception as e:
    print("IMU not available:", e)

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

    sleep(0.1)   # let motor drivers settle before monitoring starts

    # Launch motor control on core 1
    _thread.start_new_thread(motor_core, (module2, module5))

    # ── Core 0: IMU + serial comms + Yukon monitoring ─────────────────────────

    current_time = ticks_ms()
    last_sensor  = ticks_ms()

    # Serial framing state machine
    state    = 'SYNC'
    pkt_cmd  = 0
    pkt_vhigh = 0
    pkt_vlow  = 0

    while not yukon.is_boot_pressed():

        # Update IMU heading (I²C must stay on core 0)
        if imu is not None:
            try:
                imu.update()
                _lock.acquire()
                _current_heading = imu.heading()
                _lock.release()
            except Exception:
                pass

        ready = select.select([sys.stdin], [], [], 0.01)
        if ready[0]:
            ch = sys.stdin.read(1)
            b  = ord(ch)

            # SYNC byte always resets framing regardless of current state
            if b == SYNC:
                state = 'CMD'
                continue

            if state == 'CMD':
                if 0x21 <= b <= 0x27:
                    pkt_cmd = b
                    state   = 'V_HIGH'
                else:
                    _nak()
                    state = 'SYNC'

            elif state == 'V_HIGH':
                if 0x40 <= b <= 0x4F:
                    pkt_vhigh = b
                    state     = 'V_LOW'
                else:
                    _nak()
                    state = 'SYNC'

            elif state == 'V_LOW':
                if 0x50 <= b <= 0x5F:
                    pkt_vlow = b
                    state    = 'CHK'
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
                        if   value == 0: yukon.set_led(LED_A, False)
                        elif value == 1: yukon.set_led(LED_A, True)
                        elif value == 2: yukon.set_led(LED_B, False)
                        elif value == 3: yukon.set_led(LED_B, True)

                    elif cmd_code == CMD_LEFT:
                        _lock.acquire()
                        _left_speed = _decode_speed(value)
                        _lock.release()

                    elif cmd_code == CMD_RIGHT:
                        _lock.acquire()
                        _right_speed = _decode_speed(value)
                        _lock.release()

                    elif cmd_code == CMD_KILL:
                        _lock.acquire()
                        _left_speed     = 0.0
                        _right_speed    = 0.0
                        _bearing_target = None
                        _lock.release()

                    elif cmd_code == CMD_SENSOR:
                        # ── Send all sensor data then ACK ──────────────────
                        try:
                            _send_data(RESP_VOLTAGE, yukon.read_input_voltage() * 10)
                            _send_data(RESP_CURRENT, yukon.read_current()       * 100)
                            _send_data(RESP_TEMP,    yukon.read_temperature()   * 3)
                            _send_data(RESP_TEMP_L,  module2.read_temperature() * 3)
                            _send_data(RESP_TEMP_R,  module5.read_temperature() * 3)
                            _send_data(RESP_FAULT_L, int(module2.read_fault()))
                            _send_data(RESP_FAULT_R, int(module5.read_fault()))
                            # RESP_HEADING: encode current heading, or 255 if IMU absent
                            _lock.acquire()
                            hdg = _current_heading
                            _lock.release()
                            if _imu_ok:
                                _send_data(RESP_HEADING, _bearing_encode(hdg))
                            else:
                                _send_data(RESP_HEADING, 255)
                        except Exception as se:
                            print("Sensor error:", se)
                            _nak()
                            state = 'SYNC'
                            continue

                    elif cmd_code == CMD_BEARING:
                        if value == 255:
                            # Disable bearing hold
                            _lock.acquire()
                            _bearing_target = None
                            _lock.release()
                        elif _imu_ok:
                            _lock.acquire()
                            _bearing_target = _bearing_decode(value)
                            _lock.release()
                        else:
                            _nak()   # NAK: no IMU fitted
                            state = 'SYNC'
                            continue

                    _ack()
                state = 'SYNC'

        # ── Yukon health monitoring ───────────────────────────────────────────

        current_time = ticks_add(current_time, int(1000 / UPDATES))
        try:
            yukon.monitor_until_ms(current_time)
        except FaultError as e:
            print("Fault:", e, "— recovering")
            _lock.acquire()
            _left_speed  = 0.0
            _right_speed = 0.0
            _lock.release()
            try:
                sleep(0.05)
                yukon.enable_main_output()
                module2.enable()
                module5.enable()
            except Exception as e2:
                print("Recovery error:", e2)
            current_time = ticks_ms()
        except Exception as e:
            print("Unexpected error:", e)
            current_time = ticks_ms()

        # ── Periodic sensor log line (human-readable, for debugging) ──────────

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
                _lock.acquire()
                hdg = _current_heading
                tgt = _bearing_target
                _lock.release()
                tgt_str = 'off' if tgt is None else '%.1f' % tgt
                print("SENS v=%.2f i=%.3f t=%.1f tL=%.1f tR=%.1f fL=%d fR=%d hdg=%.1f tgt=%s"
                      % (v, i, t, tL, tR, int(fL), int(fR), hdg, tgt_str))
            except Exception as se:
                print("Sensor error:", se)

finally:
    _running = False
    print("Shutting down")
    yukon.reset()
