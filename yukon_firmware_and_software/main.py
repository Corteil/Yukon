import _thread
from utime import sleep, sleep_ms, ticks_ms, ticks_diff, ticks_add
import sys
import select
import random

from pimoroni_yukon import Yukon, SLOT2, SLOT3, SLOT4, SLOT5
from pimoroni_yukon.modules import BenchPowerModule, DualMotorModule, LEDStripModule
from pimoroni_yukon.errors import (FaultError, OverVoltageError,
                                   OverCurrentError, OverTemperatureError)
import rp2
from machine import Pin

# ── PIO UART RX (GP26, 115200 baud) ──────────────────────────────────────────
@rp2.asm_pio(in_shiftdir=rp2.PIO.SHIFT_RIGHT)
def _pio_uart_rx():
    wait(1, pin, 0)          # wait for idle line (high)
    wait(0, pin, 0)          # detect start bit (falling edge)
    set(x, 7)        [10]    # 8 bits; delay 11 cycles → centre of first data bit
    label("bitloop")
    in_(pins, 1)
    jmp(x_dec, "bitloop") [6]  # sample bit; delay 6 = 8 cycles/bit total
    push(noblock)            # push received byte to RX FIFO

# Hardware constants
LED_A = 'A'
LED_B = 'B'
UPDATES      = 50
CURRENT_LIMIT = 2
SENSOR_PERIOD = 1000   # ms between periodic sensor log lines
MAX_CONSECUTIVE_FAULTS = 5     # give up recovery after this many in a row
FAULT_COOLDOWN_MS      = 500   # minimum wait between recovery attempts
NUM_LEDS               = 8     # number of NeoPixels on the LED strip module
BENCH_VOLTAGE          = 5.0  # Target output voltage in volts (set_voltage() is 1:1).

# Bearing-hold proportional gain.
# Correction = BEARING_KP * (error_degrees / 180).  Max correction = BEARING_KP.
# Tune upward if the robot drifts; tune downward if it oscillates.
BEARING_KP = 0.4

# ── Serial protocol ───────────────────────────────────────────────────────────
#
# Host → Device  [SYNC, CMD, V_HIGH, V_LOW, CHK]
#   SYNC   = 0x7E  ('~')       — unique framing byte, never in other fields
#   CMD    = cmd_code + 0x20   → 0x21–0x2A  ('!' to '*')
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
CMD_STRIP      = 7   # value: colour preset index (see _STRIP_COLOURS); 0 = off
CMD_PIXEL_SET  = 8   # value: high nibble = LED index, low nibble = colour index
CMD_PIXEL_SHOW = 9   # value: ignored; pushes all staged pixel data to hardware
CMD_PATTERN    = 10  # value: high nibble = colour index (0=keep current), low nibble = pattern
                     #        patterns: 0=off, 1=larson, 2=random, 3=rainbow, 4=retro_computer, 5=converge
CMD_MODE     = 11  # value: 0=MANUAL, 1=AUTO, 2=ESTOP
CMD_RC_QUERY = 12  # value: ignored — replies with 14 channel packets + validity byte then ACK
CMD_BENCH    = 13  # value: 0=disable bench power output, 1=enable bench power output
RESP_RC_BASE = 8   # response IDs 8-21 = channels 0-13; ID 22 = RC validity flag

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
RESP_PITCH       = 8   # IMU pitch  (nose up/down), encoded (pitch+90)*254/180, 255=absent
                       # Decode: pitch = value * 180/254 - 90  (-90deg..+90deg, ~0.7deg res)
RESP_ROLL        = 9   # IMU roll   (side tilt),   encoded (roll+180)*254/360, 255=absent
                       # Decode: roll  = value * 360/254 - 180 (-180deg..+180deg, ~1.4deg res)
RESP_BENCH_TEMP  = 10  # PowerBench module temp x 3 (e.g. 22.5 degC -> 67)
RESP_BENCH_FAULT = 11  # PowerBench fault (0 or 1)

# ── Shared state (protected by _lock) ────────────────────────────────────────

_lock            = _thread.allocate_lock()
_motor_sp        = [0.0, 0.0]  # [left, right] — mutable list avoids 'global' from core 1
_bearing_target  = None    # None = disabled; float 0–360 = active target
_current_heading = 0.0     # updated each loop by imu.update() on core 0
_current_pitch   = 0.0     # degrees, positive = nose up
_current_roll    = 0.0     # degrees, positive = right side down
_imu_ok          = False   # set True once IMU initialises successfully
_running         = True
_running_ref     = [True]  # single-element list so Core 1 can observe shutdown without global lookup

# ── RC / drive mode state ─────────────────────────────────────────────────────
RC_MANUAL             = 0
RC_AUTO               = 1
RC_ESTOP              = 2

_rc_mode              = RC_MANUAL   # 0=MANUAL, 1=AUTO, 2=ESTOP
_rc_channels          = [1500] * 14 # last valid iBUS channel values (µs, 1000-2000)
_rc_ts                = [0]         # [last_packet_ms] — mutable list avoids 'global' from core 1
_pi_last_cmd_ms       = 0           # ticks_ms() of last CMD_MODE from Pi (0=never)
_pi_last_rc_query_ms  = 0           # ticks_ms() of last CMD_RC_QUERY from Pi (0=never)

IBUS_FAILSAFE_MS      = 500   # ms without iBUS packet → zero motors in MANUAL
PI_FAILSAFE_MS        = 500   # ms without CMD_MODE from Pi → ESTOP
IBUS_DEADZONE         = 30    # µs deadzone either side of 1500
IBUS_SPEED_MIN        = 0.25  # minimum speed scale at CH6=1000
IBUS_CH_THROTTLE      = 2     # 0-based: CH3 (left stick vertical)
IBUS_CH_STEER         = 0     # 0-based: CH1 (right stick horizontal)
IBUS_CH_SPEED         = 5     # 0-based: CH6 speed limit

# ── Pattern state (Core 0 only — no lock needed) ──────────────────────────────
_pattern         = 0       # active pattern: 0=off 1=larson 2=random 3=rainbow 4=sparkle
_pat_pos         = 0       # current position / hue offset
_pat_dir         = 1       # direction (larson)
_pat_counter     = 0       # tick counter for rate-limiting pattern updates
_pat_colour_idx  = 8       # palette index used by sparkle (default 8 = white)


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
        return max(-1.0, -((byte_val - 100) / 100.0))
    return min(1.0, byte_val / 100.0)

def _bearing_encode(degrees):
    """Encode degrees 0–359 → byte 0–254 (255 reserved for 'disable')."""
    return min(254, int(degrees * 254.0 / 359.0))

def _bearing_decode(value):
    """Decode protocol byte 0–254 → degrees 0–359."""
    return value * 359.0 / 254.0

def _angle_diff(target, current):
    """Signed shortest-arc difference (target − current), range −180..+180."""
    return (target - current + 180.0) % 360.0 - 180.0


# ── iBUS RC receiver (PIO UART on GP26) ───────────────────────────────────────

_ibus_sm         = None           # rp2.StateMachine — assigned during hardware setup
_ibus_buf        = bytearray(32)
# _ibus_st: [idx, hunting, byte_count, pkt_count, bad_cksum]
# Stored in a mutable list so core 1 can update without 'global' declaration.
_ibus_st         = [0, True, 0, 0, 0]


def _ibus_tank_mix_raw(ch):
    """Pure computation: iBUS channel list → (left, right) motor speeds -1.0..+1.0."""
    def norm(v):
        raw = v - 1500
        if abs(raw) < IBUS_DEADZONE:
            return 0.0
        return max(-1.0, min(1.0, raw / 500.0))
    thr   = norm(ch[IBUS_CH_THROTTLE])
    ste   = norm(ch[IBUS_CH_STEER])
    t     = max(0.0, min(1.0, (ch[IBUS_CH_SPEED] - 1000) / 1000.0))
    scale = IBUS_SPEED_MIN + t * (1.0 - IBUS_SPEED_MIN)
    return (max(-1.0, min(1.0, thr - ste)) * scale,
            max(-1.0, min(1.0, thr + ste)) * scale)


def _decode_ibus(buf, st, lock, motor_sp, rc_channels, rc_ts):
    """Validate and decode the completed 32-byte iBUS buffer. Updates shared state.

    All mutable shared objects are passed as arguments — same Core 1
    global-visibility workaround as _ibus_poll().
    """
    expected = (0xFFFF - sum(buf[:30])) & 0xFFFF
    stored   = buf[30] | (buf[31] << 8)
    if expected != stored:
        st[4] += 1
        return
    st[3] += 1
    ch = []
    for i in range(14):
        off = 2 + i * 2
        v   = buf[off] | (buf[off + 1] << 8)
        ch.append(max(1000, min(2000, v)))
    # Compute motor speeds before locking (pure maths, no shared state read)
    # No-motors mode: Pi is sending CMD_RC_QUERY but not CMD_MODE → suppress iBUS motor control
    _pi_no_motors = (_pi_last_rc_query_ms != 0 and _pi_last_cmd_ms == 0)
    left = right = None
    if _rc_mode == RC_MANUAL and not _pi_no_motors:
        left, right = _ibus_tank_mix_raw(ch)
    lock.acquire()
    rc_channels[:] = ch
    rc_ts[0]       = ticks_ms()
    if left is not None:
        motor_sp[0] = left
        motor_sp[1] = right
    lock.release()


def _ibus_poll(sm, buf, st):
    """Non-blocking drain of PIO RX FIFO.  Returns True if a complete packet arrived.

    All state is passed as parameters — avoids module-global lookups from
    sub-functions called on core 1, which are unreliable on this MicroPython build.
    Caller (motor_core) is responsible for calling _decode_ibus(buf, st) when True is returned.
    """
    if sm is None:
        return False
    pkt = False
    while sm.rx_fifo():
        byte = (sm.get() >> 24) & 0xFF
        st[2] += 1
        if st[1]:                      # hunting for header
            if st[0] == 0:
                if byte == 0x20:
                    buf[0] = byte
                    st[0]  = 1
            else:                      # waiting for second header byte 0x40
                if byte == 0x40:
                    buf[1] = byte
                    st[0]  = 2
                    st[1]  = False
                elif byte == 0x20:
                    buf[0] = byte      # restart on new 0x20
                else:
                    st[0]  = 0
        else:
            buf[st[0]] = byte
            st[0] += 1
            if st[0] == 32:
                st[0] = 0
                st[1] = True
                pkt   = True
    return pkt


# ── LED strip ─────────────────────────────────────────────────────────────────

# Preset colours (R, G, B).  Index matches the CMD_STRIP value byte.
_STRIP_COLOURS = [
    (0,   0,   0  ),  # 0  off
    (255, 0,   0  ),  # 1  red     — ESTOP / fault
    (0,   255, 0  ),  # 2  green   — AUTO
    (0,   0,   255),  # 3  blue    — MANUAL
    (255, 165, 0  ),  # 4  orange
    (255, 255, 0  ),  # 5  yellow
    (0,   255, 255),  # 6  cyan
    (255, 0,   255),  # 7  magenta
    (255, 255, 255),  # 8  white
]

def _set_strip(r, g, b):
    """Set all NeoPixels to (r, g, b) and push to hardware."""
    for i in range(NUM_LEDS):
        module1.strip.set_rgb(i, r, g, b)
    module1.strip.update()


def _hsv_to_rgb(h):
    """Convert hue (0-359) to (r, g, b) at full saturation and half brightness."""
    h = h % 360
    s = h // 60
    f = (h % 60) * 255 // 60
    q = 255 - f
    if s == 0: return 127, f // 2,   0
    if s == 1: return q // 2,  127,  0
    if s == 2: return 0,   127,      f // 2
    if s == 3: return 0,   q // 2,   127
    if s == 4: return f // 2,   0,   127
    return 127, 0, q // 2


# Ticks between pattern updates (1 tick = 20 ms at 50 Hz)
_PAT_RATE = {1: 3, 2: 10, 3: 2, 4: 10, 5: 4, 6: 15}  # larson ~60 ms, random/retro ~200 ms, rainbow ~40 ms, converge ~80 ms, estop_flash ~300 ms


def _update_pattern():
    """Advance the active pattern by one step and push to the strip."""
    global _pattern, _pat_pos, _pat_dir, _pat_counter

    if _pattern == 0:
        return

    _pat_counter += 1
    if _pat_counter < _PAT_RATE.get(_pattern, 5):
        return
    _pat_counter = 0

    if _pattern == 1:                               # ── Larson scanner ──
        for i in range(NUM_LEDS):
            d = abs(i - _pat_pos)
            if d == 0:
                module1.strip.set_rgb(i, 255, 0, 0)
            elif d == 1:
                module1.strip.set_rgb(i, 48, 0, 0)
            elif d == 2:
                module1.strip.set_rgb(i, 8, 0, 0)
            else:
                module1.strip.set_rgb(i, 0, 0, 0)
        module1.strip.update()
        _pat_pos += _pat_dir
        if _pat_pos >= NUM_LEDS - 1:
            _pat_dir = -1
        elif _pat_pos <= 0:
            _pat_dir = 1

    elif _pattern == 2:                             # ── Random on/off ──
        for i in range(NUM_LEDS):
            if random.getrandbits(1):
                idx = random.randint(1, len(_STRIP_COLOURS) - 1)
                r, g, b = _STRIP_COLOURS[idx]
            else:
                r, g, b = 0, 0, 0
            module1.strip.set_rgb(i, r, g, b)
        module1.strip.update()

    elif _pattern == 3:                             # ── Rainbow ──
        for i in range(NUM_LEDS):
            hue = (_pat_pos + i * (360 // NUM_LEDS)) % 360
            r, g, b = _hsv_to_rgb(hue)
            module1.strip.set_rgb(i, r, g, b)
        module1.strip.update()
        _pat_pos = (_pat_pos + 8) % 360

    elif _pattern == 4:                             # ── Retro Computer ──
        r, g, b = _STRIP_COLOURS[_pat_colour_idx]
        for i in range(NUM_LEDS):
            if random.getrandbits(1):
                module1.strip.set_rgb(i, r, g, b)
            else:
                module1.strip.set_rgb(i, 0, 0, 0)
        module1.strip.update()

    elif _pattern == 5:                             # ── Converge ──
        r, g, b = _STRIP_COLOURS[_pat_colour_idx]
        fill = _pat_pos
        for i in range(NUM_LEDS):
            if i < fill or i >= NUM_LEDS - fill:
                module1.strip.set_rgb(i, r, g, b)
            else:
                module1.strip.set_rgb(i, 0, 0, 0)
        module1.strip.update()
        _pat_pos += _pat_dir
        if _pat_pos > NUM_LEDS // 2:
            _pat_dir = -1
            _pat_pos = NUM_LEDS // 2
        elif _pat_pos < 0:
            _pat_dir = 1
            _pat_pos = 0

    elif _pattern == 6:                             # ── ESTOP flash ──
        # LEDs 0-2 and 5-7 flash red; LEDs 3-4 show fault colour (_pat_colour_idx)
        fr, fg, fb = _STRIP_COLOURS[_pat_colour_idx]
        red = (255, 0, 0) if _pat_pos == 0 else (0, 0, 0)
        for i in range(3):
            module1.strip.set_rgb(i, red[0], red[1], red[2])
        module1.strip.set_rgb(3, fr, fg, fb)
        module1.strip.set_rgb(4, fr, fg, fb)
        for i in range(5, NUM_LEDS):
            module1.strip.set_rgb(i, red[0], red[1], red[2])
        module1.strip.update()
        _pat_pos = 1 - _pat_pos                     # toggle 0/1


# ── Core 1: motor drive loop ──────────────────────────────────────────────────

def motor_core(module_left, module_right,
               ibus_sm, ibus_buf, ibus_st,
               lock, motor_sp, rc_channels, rc_ts,
               fn_poll, fn_decode, fn_angle_diff,
               bearing_kp, running_ref):
    """Continuously apply the latest motor speeds with optional bearing hold.

    Runs on core 1.  All required state is passed as arguments — globals assigned
    only at module load are not reliably accessible from Core 1 on this
    MicroPython build, so nothing is looked up via the global dict.
    running_ref is a single-element list [True] so Core 1 can observe shutdown.
    """
    from utime import ticks_ms, ticks_diff, ticks_add
    print("motor_core: started on core 1")
    try:
        while running_ref[0]:
            lock.acquire()
            left    = motor_sp[0]
            right   = motor_sp[1]
            target  = _bearing_target
            heading = _current_heading
            lock.release()

            if target is not None:
                error      = fn_angle_diff(target, heading)
                correction = max(-bearing_kp, min(bearing_kp, bearing_kp * error / 180.0))
                left  = max(-1.0, min(1.0, left  + correction))
                right = max(-1.0, min(1.0, right - correction))

            for motor in module_left.motors:
                motor.speed(left)
            for motor in module_right.motors:
                motor.speed(-right)

            t_end = ticks_add(ticks_ms(), 20)
            while ticks_diff(t_end, ticks_ms()) > 0:
                if fn_poll(ibus_sm, ibus_buf, ibus_st):
                    fn_decode(ibus_buf, ibus_st, lock, motor_sp, rc_channels, rc_ts)
    except Exception as e:
        print("motor_core CRASHED:", type(e).__name__, e)
        lock.acquire()
        motor_sp[0] = 0.0
        motor_sp[1] = 0.0
        lock.release()
        running_ref[0] = False



# ── Hardware setup ────────────────────────────────────────────────────────────

yukon        = Yukon()
module_bench = BenchPowerModule()             # regulated power output — SLOT4
module1      = LEDStripModule(LEDStripModule.NEOPIXEL, pio=0, sm=0, num_leds=NUM_LEDS)
module2      = DualMotorModule()              # left motors  — SLOT2
module5      = DualMotorModule()              # right motors — SLOT5

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
    yukon.register_with_slot(module_bench, SLOT4)
    yukon.register_with_slot(module1, SLOT3)
    yukon.register_with_slot(module2, SLOT2)
    yukon.register_with_slot(module5, SLOT5)
    yukon.verify_and_initialise()
    yukon.enable_main_output()
    sleep_ms(500)                # allow main supply to stabilise before enabling modules

    module1.enable()
    _set_strip(0, 0, 0)   # strip off at startup

    module2.set_current_limit(CURRENT_LIMIT)
    module2.enable()
    for motor in module2.motors:
        motor.enable()

    module5.set_current_limit(CURRENT_LIMIT)
    module5.enable()
    for motor in module5.motors:
        motor.enable()

    module_bench.set_voltage(BENCH_VOLTAGE)  # pre-program voltage; output stays off until CMD_BENCH enable

    sleep(0.1)   # let motor drivers settle before monitoring starts

    # ── iBUS PIO state machine (GP26, 115200 baud) ────────────────────────────
    # Must be initialised BEFORE motor_core starts on core 1 — core 1 calls
    # _ibus_poll() which references _ibus_sm immediately.
    _ibus_sm = rp2.StateMachine(
        1,                                       # SM1 = PIO0_SM1 — LED strip uses SM0, SM1 is free
        _pio_uart_rx,
        freq=921600,                             # 115200 baud × 8 cycles/bit
        in_base=Pin(26, Pin.IN, Pin.PULL_UP),    # GP26, pull-up for idle-high line
    )
    _ibus_sm.active(1)
    print("iBUS SM1 active on GP26")

    # Launch motor control on core 1
    _thread.start_new_thread(motor_core, (
        module2, module5,
        _ibus_sm, _ibus_buf, _ibus_st,
        _lock, _motor_sp, _rc_channels, _rc_ts,
        _ibus_poll, _decode_ibus, _angle_diff,
        BEARING_KP, _running_ref,
    ))

    # ── Core 0: IMU + serial comms + Yukon monitoring ─────────────────────────

    current_time = ticks_ms()
    last_sensor  = ticks_ms()
    consecutive_faults = 0

    # Serial framing state machine
    state    = 'SYNC'
    pkt_cmd  = 0
    pkt_vhigh = 0
    pkt_vlow  = 0

    while not yukon.is_boot_pressed():

        # Update IMU orientation (I²C must stay on core 0)
        if imu is not None:
            try:
                imu.update()
                heading = imu.heading()
            except Exception:
                heading = None
            try:
                pitch = imu.pitch()
                roll  = imu.roll()
            except Exception:
                pitch = roll = None
            if heading is not None:
                _lock.acquire()
                _current_heading = heading
                if pitch is not None:
                    _current_pitch = pitch
                if roll  is not None:
                    _current_roll  = roll
                _lock.release()

        # ── watchdog ─────────────────────────────────────────────────────────
        # iBUS is polled continuously on core 1 inside motor_core().
        _now_ms = ticks_ms()

        # Pi watchdog: ESTOP if CMD_MODE stops arriving (Pi crash / USB drop)
        if _pi_last_cmd_ms != 0 and ticks_diff(_now_ms, _pi_last_cmd_ms) > PI_FAILSAFE_MS:
            if _rc_mode != RC_ESTOP:
                print("WATCHDOG: Pi timeout → ESTOP")
                _rc_mode = RC_ESTOP
                _lock.acquire()
                _motor_sp[0]    = 0.0
                _motor_sp[1]    = 0.0
                _bearing_target = None
                _lock.release()

        # RC failsafe: zero motors if iBUS signal lost in MANUAL mode
        if (_rc_mode == RC_MANUAL and _rc_ts[0] != 0 and
                ticks_diff(_now_ms, _rc_ts[0]) > IBUS_FAILSAFE_MS):
            _lock.acquire()
            _motor_sp[0] = 0.0
            _motor_sp[1] = 0.0
            _lock.release()

        ready = select.select([sys.stdin], [], [], 0.01)
        if ready[0]:
            ch = sys.stdin.read(1)
            b  = ord(ch)

            # SYNC byte always resets framing regardless of current state
            if b == SYNC:
                state = 'CMD'
                continue

            if state == 'CMD':
                if 0x21 <= b <= 0x2D:
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
                        else:
                            _nak()
                            state = 'SYNC'
                            continue

                    elif cmd_code == CMD_LEFT:
                        if _rc_mode == RC_AUTO:
                            _lock.acquire()
                            _motor_sp[0] = _decode_speed(value)
                            _lock.release()

                    elif cmd_code == CMD_RIGHT:
                        if _rc_mode == RC_AUTO:
                            _lock.acquire()
                            _motor_sp[1] = _decode_speed(value)
                            _lock.release()

                    elif cmd_code == CMD_KILL:
                        _lock.acquire()
                        _motor_sp[0]    = 0.0
                        _motor_sp[1]    = 0.0
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
                            _send_data(RESP_BENCH_TEMP,  module_bench.read_temperature() * 3)
                            _send_data(RESP_BENCH_FAULT, int(not module_bench.read_power_good()))
                            # RESP_HEADING / RESP_PITCH / RESP_ROLL: 255 if IMU absent
                            _lock.acquire()
                            hdg = _current_heading
                            pit = _current_pitch
                            rol = _current_roll
                            _lock.release()
                            if _imu_ok:
                                _send_data(RESP_HEADING, _bearing_encode(hdg))
                                _send_data(RESP_PITCH,
                                           int((pit + 90.0)  * 254.0 / 180.0 + 0.5))
                                _send_data(RESP_ROLL,
                                           int((rol + 180.0) * 254.0 / 360.0 + 0.5))
                            else:
                                _send_data(RESP_HEADING, 255)
                                _send_data(RESP_PITCH,   255)
                                _send_data(RESP_ROLL,    255)
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

                    elif cmd_code == CMD_STRIP:
                        _pattern = 0
                        idx = min(value, len(_STRIP_COLOURS) - 1)
                        r, g, b = _STRIP_COLOURS[idx]
                        _set_strip(r, g, b)

                    elif cmd_code == CMD_PIXEL_SET:
                        led_idx    = (value >> 4) & 0x0F
                        colour_idx = value & 0x0F
                        if led_idx < NUM_LEDS and colour_idx < len(_STRIP_COLOURS):
                            r, g, b = _STRIP_COLOURS[colour_idx]
                            module1.strip.set_rgb(led_idx, r, g, b)
                        else:
                            _nak()
                            state = 'SYNC'
                            continue

                    elif cmd_code == CMD_PIXEL_SHOW:
                        module1.strip.update()

                    elif cmd_code == CMD_PATTERN:
                        colour_nibble = (value >> 4) & 0x0F
                        pat           = value & 0x0F
                        if colour_nibble > 0 and colour_nibble < len(_STRIP_COLOURS):
                            _pat_colour_idx = colour_nibble
                        _pattern     = pat if pat <= 6 else 0
                        _pat_pos     = 0
                        _pat_dir     = 1
                        _pat_counter = 0
                        if _pattern == 0:
                            _set_strip(0, 0, 0)

                    elif cmd_code == CMD_MODE:
                        new_mode = value
                        if new_mode in (RC_MANUAL, RC_AUTO, RC_ESTOP):
                            _pi_last_cmd_ms = ticks_ms()
                            if new_mode == RC_ESTOP and _rc_mode != RC_ESTOP:
                                _lock.acquire()
                                _motor_sp[0]    = 0.0
                                _motor_sp[1]    = 0.0
                                _bearing_target = None
                                _lock.release()
                            _rc_mode = new_mode
                        else:
                            _nak()
                            state = 'SYNC'
                            continue

                    elif cmd_code == CMD_RC_QUERY:
                        _pi_last_rc_query_ms = ticks_ms()
                        _lock.acquire()
                        ch    = list(_rc_channels)
                        rc_ts = _rc_ts[0]
                        _lock.release()
                        age   = ticks_diff(ticks_ms(), rc_ts) if rc_ts != 0 else -1
                        rc_ok = 1 if (rc_ts != 0 and age < IBUS_FAILSAFE_MS) else 0
                        try:
                            for i in range(14):
                                _send_data(RESP_RC_BASE + i,
                                           (max(1000, min(2000, ch[i])) - 1000) // 5)
                            _send_data(RESP_RC_BASE + 14, rc_ok)  # validity flag (ID 22)
                        except Exception as e:
                            print("RC_QUERY error:", e)
                            _nak()
                            state = 'SYNC'
                            continue

                    elif cmd_code == CMD_BENCH:
                        if value == 0:
                            module_bench.disable()
                        else:
                            module_bench.enable()
                            module_bench.set_voltage(BENCH_VOLTAGE)

                    _ack()
                state = 'SYNC'

        # ── Pattern animation ─────────────────────────────────────────────────

        _update_pattern()

        # ── Yukon health monitoring ───────────────────────────────────────────

        current_time = ticks_add(current_time, int(1000 / UPDATES))
        try:
            yukon.monitor_until_ms(current_time)
            consecutive_faults = 0
        except (OverVoltageError, OverTemperatureError) as e:
            # Non-recoverable: SDK already disabled main output.
            # Stop motors and exit — let finally block call yukon.reset().
            print("CRITICAL:", type(e).__name__, e, "— shutting down")
            try:
                print("Last readings: v=%.2f i=%.3f t=%.1f tL=%.1f tR=%.1f"
                      % (yukon.read_input_voltage(), yukon.read_current(),
                         yukon.read_temperature(), module2.read_temperature(),
                         module5.read_temperature()))
            except Exception:
                pass
            _lock.acquire()
            _motor_sp[0]    = 0.0
            _motor_sp[1]    = 0.0
            _bearing_target = None
            _lock.release()
            _pat_colour_idx = 1   # red = Yukon hardware fault (reserved)
            _pat_pos        = 0
            _pattern        = 6   # estop_flash pattern
            break
        except (FaultError, OverCurrentError, Exception) as e:
            # Recoverable: transient faults, current spikes, undervoltage.
            # SDK already disabled main output — try to re-enable.
            consecutive_faults += 1
            print("Fault:", type(e).__name__, e,
                  "— recovery attempt %d/%d" % (consecutive_faults,
                                                MAX_CONSECUTIVE_FAULTS))
            _lock.acquire()
            _motor_sp[0]    = 0.0
            _motor_sp[1]    = 0.0
            _bearing_target = None
            _lock.release()
            _pat_colour_idx = 1   # red = Yukon hardware fault (reserved)
            _pat_pos        = 0
            _pattern        = 6   # estop_flash pattern
            if consecutive_faults >= MAX_CONSECUTIVE_FAULTS:
                print("Too many consecutive faults — shutting down")
                break
            sleep_ms(FAULT_COOLDOWN_MS)
            try:
                yukon.enable_main_output()
                sleep_ms(500)
                module2.enable()
                for motor in module2.motors:
                    motor.enable()
                module5.enable()
                for motor in module5.motors:
                    motor.enable()
                module_bench.enable()
                sleep_ms(500)
                module_bench.set_voltage(BENCH_VOLTAGE)
                _pattern = 0
                _set_strip(0, 0, 0)     # clear fault pattern after successful recovery
            except Exception as e2:
                print("Recovery failed:", e2, "— shutting down")
                break
            current_time = ticks_ms()

        # ── Periodic sensor log line (human-readable, for debugging) ──────────

        now = ticks_ms()
        if ticks_diff(now, last_sensor) >= SENSOR_PERIOD:
            last_sensor = now
            try:
                v  = yukon.read_input_voltage()
                i  = yukon.read_current()
                t  = yukon.read_temperature()
                tL  = module2.read_temperature()
                tR  = module5.read_temperature()
                tBP = module_bench.read_temperature()
                fL  = module2.read_fault()
                fR  = module5.read_fault()
                fBP = not module_bench.read_power_good()
                _lock.acquire()
                hdg = _current_heading
                pit = _current_pitch
                rol = _current_roll
                tgt = _bearing_target
                _lock.release()
                tgt_str = 'off' if tgt is None else '%.1f' % tgt
                print("SENS v=%.2f i=%.3f t=%.1f tL=%.1f tR=%.1f tBP=%.1f fL=%d fR=%d fBP=%d "
                      "hdg=%.1f pit=%.1f rol=%.1f tgt=%s"
                      % (v, i, t, tL, tR, tBP, int(fL), int(fR), int(fBP), hdg, pit, rol, tgt_str))
            except Exception as se:
                print("Sensor error:", se)

finally:
    _running = False
    _running_ref[0] = False
    print("Shutting down")
    try:
        _set_strip(0, 0, 0)
    except Exception:
        pass
    yukon.reset()
