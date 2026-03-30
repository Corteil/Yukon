"""
test_rc.py — iBUS RC receiver diagnostic for Yukon RP2040.

Stage 1: raw byte dump from GP26 PIO — confirms the wire is alive.
Stage 2: iBUS packet decode — confirms framing and checksum.

Run:  mpremote run test_rc.py
"""

import rp2
from machine import Pin
from utime import ticks_ms, ticks_diff, sleep_ms


# ── PIO UART RX (GP26, 115200 baud) ──────────────────────────────────────────

@rp2.asm_pio(in_shiftdir=rp2.PIO.SHIFT_RIGHT)
def _pio_uart_rx():
    wait(1, pin, 0)
    wait(0, pin, 0)
    set(x, 7)        [10]
    label("bitloop")
    in_(pins, 1)
    jmp(x_dec, "bitloop") [6]
    push(noblock)


print("Starting PIO state machine on GP26...")
try:
    sm = rp2.StateMachine(
        0,
        _pio_uart_rx,
        freq=921600,
        in_base=Pin(26, Pin.IN, Pin.PULL_UP),
    )
    sm.active(1)
    print("PIO SM0 running OK")
except Exception as e:
    print("PIO error:", e)
    raise


# ── Stage 1: raw byte dump (5 seconds) ───────────────────────────────────────

print()
print("=== Stage 1: raw bytes from GP26 (5 s) ===")
print("You should see a stream of hex bytes if the RX wire is connected and TX is on.")
print()

raw_bytes = []
t_end = ticks_ms() + 5000
while ticks_diff(t_end, ticks_ms()) > 0:
    while sm.rx_fifo():
        b = (sm.get() >> 24) & 0xFF
        raw_bytes.append(b)

if not raw_bytes:
    print("NO BYTES RECEIVED — check GP26 wire and transmitter power")
else:
    print("Received {} bytes. First 64:".format(len(raw_bytes)))
    hex_str = " ".join("{:02X}".format(b) for b in raw_bytes[:64])
    print(hex_str)
    # Look for iBUS header
    found = any(raw_bytes[i] == 0x20 and raw_bytes[i+1] == 0x40
                for i in range(len(raw_bytes) - 1))
    print("iBUS header (20 40) found:", "YES" if found else "NO")

print()


# ── Stage 2: iBUS decode loop ────────────────────────────────────────────────

print("=== Stage 2: iBUS channel display (Ctrl-C to stop) ===")
print()

buf      = bytearray(32)
idx      = 0
hunting  = True
channels = [1500] * 14
last_rx  = 0
packets  = 0
bad_cksum = 0

FAILSAFE_MS = 500

def poll():
    global idx, hunting, channels, last_rx, packets, bad_cksum
    while sm.rx_fifo():
        byte = (sm.get() >> 24) & 0xFF
        if hunting:
            if idx == 0:
                if byte == 0x20:
                    buf[0] = byte
                    idx = 1
            else:
                if byte == 0x40:
                    buf[1] = byte
                    idx = 2
                    hunting = False
                elif byte == 0x20:
                    buf[0] = byte
                else:
                    idx = 0
        else:
            buf[idx] = byte
            idx += 1
            if idx == 32:
                expected = (0xFFFF - sum(buf[:30])) & 0xFFFF
                stored   = buf[30] | (buf[31] << 8)
                if expected == stored:
                    ch = []
                    for i in range(14):
                        off = 2 + i * 2
                        ch.append(buf[off] | (buf[off + 1] << 8))
                    channels[:] = ch
                    last_rx = ticks_ms()
                    packets += 1
                else:
                    bad_cksum += 1
                idx = 0
                hunting = True

PRINT_INTERVAL_MS = 200   # display rate — poll runs continuously between prints

try:
    last_print = ticks_ms()
    while True:
        poll()   # drain FIFO immediately — must not sleep here

        if ticks_diff(ticks_ms(), last_print) >= PRINT_INTERVAL_MS:
            last_print = ticks_ms()
            age    = ticks_diff(ticks_ms(), last_rx) if last_rx else 99999
            active = age < FAILSAFE_MS

            if active:
                s = "  ".join("{:4d}".format(v) for v in channels[:8])
                print("OK  [{}] ch1-8: {}".format(packets, s))
            else:
                print("NO SIGNAL  packets={} bad_cksum={} age={}ms".format(
                    packets, bad_cksum, age if last_rx else "never"))

except KeyboardInterrupt:
    print("\nStopped. Total packets: {}  Bad checksum: {}".format(packets, bad_cksum))
