#!/usr/bin/env python3
"""Upload a file to MicroPython device via raw REPL.
Handles yukon.reset() USB reconnect after Ctrl+C interrupts main.py."""
import sys
import time
import base64
import serial
import serial.tools.list_ports

PORT = '/dev/ttyACM0'
BAUD = 115200
SRC  = sys.argv[1]
DEST = sys.argv[2] if len(sys.argv) > 2 else SRC.split('/')[-1]

with open(SRC, 'rb') as f:
    data = f.read()


def open_port():
    return serial.Serial(PORT, BAUD, timeout=3, dsrdtr=False, rtscts=False)


def wait_reconnect(timeout=10.0):
    """Wait for /dev/ttyACM0 to reappear after a reset."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        ports = [p.device for p in serial.tools.list_ports.comports()]
        if PORT in ports:
            return True
        time.sleep(0.2)
    return False


def send(ser, b):
    ser.write(b); ser.flush()


def wait_for(ser, pat, timeout=5.0):
    buf = b''
    deadline = time.time() + timeout
    while time.time() < deadline:
        c = ser.read(1)
        if c:
            buf += c
            if pat in buf:
                return buf
    raise TimeoutError(f"expected {pat!r}, got {buf!r}")


def raw_exec(ser, cmd, timeout=15.0):
    send(ser, cmd.encode() + b'\x04')
    # raw REPL response: OK + stdout + \x04 + stderr + \x04
    wait_for(ser, b'OK', timeout)
    out = wait_for(ser, b'\x04\x04', timeout)
    return out


# ── Step 1: interrupt running script (this causes USB disconnect via yukon.reset()) ──
print("Interrupting running script...")
try:
    ser = open_port()
    send(ser, b'\x03\x03')
    time.sleep(0.5)
    ser.close()
except Exception as e:
    print(f"  (interrupt: {e})")

# ── Step 2: wait for USB to reconnect after yukon.reset() ────────────────────
print("Waiting for device to reconnect...", end='', flush=True)
time.sleep(2.0)   # yukon.reset() drops then re-enumerates; main.py restarts
if not wait_reconnect(timeout=15.0):
    print("\nERROR: device did not reconnect")
    sys.exit(1)
print(" OK")
time.sleep(1.5)   # wait for main.py to fully start before interrupting again

# ── Step 3: interrupt main.py that restarted after reset, enter raw REPL ─────
print("Entering raw REPL...")
ser = open_port()
time.sleep(0.5)
send(ser, b'\x03\x03')  # Ctrl+C — interrupt restarted main.py
time.sleep(0.5)
# main.py finally runs yukon.reset() again; wait for second reconnect
ser.close()
print("Waiting for second reconnect...", end='', flush=True)
time.sleep(2.0)
if not wait_reconnect(timeout=15.0):
    print("\nERROR: device did not reconnect (2nd)")
    sys.exit(1)
print(" OK")
time.sleep(1.0)

ser = open_port()
time.sleep(0.3)
send(ser, b'\x01')   # Ctrl+A → raw REPL
wait_for(ser, b'raw REPL')
send(ser, b'\x04')   # flush
time.sleep(0.1)
# drain any leftover bytes
ser.timeout = 0.2
while ser.read(64): pass
ser.timeout = 3

# ── Step 4: upload in base64 chunks ──────────────────────────────────────────
CHUNK = 128
chunks = [data[i:i+CHUNK] for i in range(0, len(data), CHUNK)]
print(f"Uploading {SRC} → {DEST}  ({len(data)} bytes, {len(chunks)} chunks)")

raw_exec(ser, f"import ubinascii; f=open('{DEST}','wb')")

for n, chunk in enumerate(chunks):
    b64 = base64.b64encode(chunk).decode()
    raw_exec(ser, f"f.write(ubinascii.a2b_base64('{b64}'))")
    sys.stdout.write(f"\r  chunk {n+1}/{len(chunks)}")
    sys.stdout.flush()

raw_exec(ser, "f.close()")
print(f"\n  upload complete.")

# ── Step 5: soft reset to run new main.py ────────────────────────────────────
print("Resetting device...")
send(ser, b'\x02')   # Ctrl+B — friendly REPL
time.sleep(0.3)
send(ser, b'\x04')   # Ctrl+D — soft reset
time.sleep(1.0)
ser.close()
print("Done — new main.py is running.")
