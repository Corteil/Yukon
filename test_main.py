#!/usr/bin/env python3
"""
test_main.py - Host-side tests for Yukon main.py

Runs on a PC connected to the Yukon via USB serial.
Use --dry-run to verify the protocol encoder without hardware.

Protocol: 5-byte packets, all bytes are printable ASCII — no REPL interference
  [SYNC=0x7E, CMD, V_HIGH, V_LOW, CHK]
  SYNC   = 0x7E (126, '~') — unique, never appears in other fields
  CMD    = cmd_code + 0x20  → 0x21-0x24 (33-36, '!' to '$')
  V_HIGH = (value >> 4) + 0x40 → 0x40-0x4F (64-79, '@' to 'O')
  V_LOW  = (value & 0xF) + 0x50 → 0x50-0x5F (80-95, 'P' to '_')
  CHK    = CMD ^ V_HIGH ^ V_LOW  → 49-62 (bounded, never equals SYNC)
Response: ACK=0x06 on success, NAK=0x15 on any framing or checksum error

Usage:
    python3 test_main.py                    # auto-detect port
    python3 test_main.py --port /dev/ttyACM0
    python3 test_main.py --dry-run          # encoder-only, no hardware needed
"""

import sys
import time
import queue
import threading
import argparse

# ---------------------------------------------------------------------------
# Protocol constants
# ---------------------------------------------------------------------------

SYNC = 0x7E
ACK  = 0x06
NAK  = 0x15

CMD_LED    = 1   # value: 0=LED_A off, 1=LED_A on, 2=LED_B off, 3=LED_B on
CMD_LEFT   = 2   # value: motor speed byte, see speed_to_byte()
CMD_RIGHT  = 3
CMD_KILL   = 4   # value: ignored (use 0)
CMD_SENSOR = 5   # value: ignored; device replies with sensor data packets then ACK

# Sensor data packet RESP_TYPE IDs (device→host, resp_id + 0x30)
RESP_VOLTAGE = 0   # input voltage × 10
RESP_CURRENT = 1   # current × 100
RESP_TEMP    = 2   # board temp × 3
RESP_TEMP_L  = 3   # left module temp × 3
RESP_TEMP_R  = 4   # right module temp × 3
RESP_FAULT_L = 5   # left fault (0 or 1)
RESP_FAULT_R = 6   # right fault (0 or 1)

SENSOR_SCALE = {
    RESP_VOLTAGE: 10.0,
    RESP_CURRENT: 100.0,
    RESP_TEMP:    3.0,
    RESP_TEMP_L:  3.0,
    RESP_TEMP_R:  3.0,
    RESP_FAULT_L: 1.0,
    RESP_FAULT_R: 1.0,
}
SENSOR_NAMES = ['v', 'i', 't', 'tL', 'tR', 'fL', 'fR']


# ---------------------------------------------------------------------------
# Encoder / decoder
# ---------------------------------------------------------------------------

def encode(cmd_code, byte_value):
    """Return the 5-byte wire packet for (cmd_code, byte_value 0-255)."""
    if not (1 <= cmd_code <= 5):
        raise ValueError(f"cmd_code {cmd_code} out of 1-5 range")
    if not (0 <= byte_value <= 255):
        raise ValueError(f"byte_value {byte_value} out of 0-255 range")
    cmd    = cmd_code + 0x20
    v_high = (byte_value >> 4)   + 0x40
    v_low  = (byte_value & 0x0F) + 0x50
    chk    = cmd ^ v_high ^ v_low
    return bytes([SYNC, cmd, v_high, v_low, chk])


def decode(data):
    """
    Decode a 5-byte wire packet to (cmd_code, byte_value).
    Raises ValueError on bad sync, out-of-range fields, or checksum mismatch.
    """
    if len(data) != 5:
        raise ValueError(f"expected 5 bytes, got {len(data)}")
    sync, cmd, v_high, v_low, chk = data
    if sync != SYNC:
        raise ValueError(f"bad sync byte: {sync:#04x}")
    if not (0x21 <= cmd <= 0x25):
        raise ValueError(f"CMD {cmd:#04x} out of range 0x21-0x25")
    if not (0x40 <= v_high <= 0x4F):
        raise ValueError(f"V_HIGH {v_high:#04x} out of range 0x40-0x4F")
    if not (0x50 <= v_low <= 0x5F):
        raise ValueError(f"V_LOW {v_low:#04x} out of range 0x50-0x5F")
    if chk != cmd ^ v_high ^ v_low:
        raise ValueError(f"checksum mismatch: expected {cmd ^ v_high ^ v_low:#04x}, got {chk:#04x}")
    cmd_code   = cmd - 0x20
    byte_value = ((v_high - 0x40) << 4) | (v_low - 0x50)
    return cmd_code, byte_value


def speed_to_byte(speed):
    """
    Encode a motor speed (-1.0 .. 1.0) as a byte value.
    Matches the decode logic in main.py:
        byte <= 100  -> speed =  byte / 100        (forward)
        byte >  100  -> speed = -(byte - 100) / 100  (reverse)
    """
    speed = max(-1.0, min(1.0, speed))
    if speed >= 0.0:
        return round(speed * 100)
    return 100 + round(abs(speed) * 100)


def byte_to_speed(bval):
    """Inverse of speed_to_byte (for verification)."""
    if bval > 100:
        return -((bval - 100) / 100)
    return bval / 100


# ---------------------------------------------------------------------------
# Encoder unit tests (no hardware needed)
# ---------------------------------------------------------------------------

class EncoderTests:
    def __init__(self):
        self.passed = 0
        self.failed = 0

    def _ok(self, label):
        print(f"  PASS  {label}")
        self.passed += 1

    def _fail(self, label, detail):
        print(f"  FAIL  {label}")
        print(f"         {detail}")
        self.failed += 1

    def check(self, label, got, expected):
        if got == expected:
            self._ok(label)
        else:
            self._fail(label, f"expected {expected!r}, got {got!r}")

    def run(self):
        print("\n=== Encoder round-trip tests ===")

        cases = [
            (CMD_LED,   0),
            (CMD_LED,   1),
            (CMD_LED,   2),
            (CMD_LED,   3),
            (CMD_LEFT,  0),
            (CMD_LEFT,  50),
            (CMD_LEFT,  100),
            (CMD_LEFT,  150),
            (CMD_LEFT,  200),
            (CMD_LEFT,  255),
            (CMD_RIGHT, 0),
            (CMD_RIGHT, 75),
            (CMD_RIGHT, 175),
            (CMD_KILL,   0),
            (CMD_SENSOR, 0),
        ]

        for cmd, bval in cases:
            wire = encode(cmd, bval)
            got_cmd, got_bval = decode(wire)
            self.check(
                f"encode/decode cmd={cmd} bval={bval}",
                (got_cmd, got_bval),
                (cmd, bval),
            )

        print("\n=== Printable ASCII (all bytes in 33-126 range) ===")

        for cmd in (CMD_LED, CMD_LEFT, CMD_RIGHT, CMD_KILL):
            for bval in (0, 1, 50, 100, 150, 200, 255):
                wire = encode(cmd, bval)
                ok = all(33 <= b <= 126 for b in wire)
                self.check(
                    f"printable ASCII cmd={cmd} bval={bval}: {[hex(b) for b in wire]}",
                    ok, True,
                )

        print("\n=== Field range checks ===")

        for cmd in (CMD_LED, CMD_LEFT, CMD_RIGHT, CMD_KILL):
            for bval in (0, 127, 255):
                wire = encode(cmd, bval)
                _, c, vh, vl, chk = wire
                self.check(f"CMD in 0x21-0x24 (cmd={cmd})",      0x21 <= c  <= 0x24, True)
                self.check(f"V_HIGH in 0x40-0x4F (bval={bval})", 0x40 <= vh <= 0x4F, True)
                self.check(f"V_LOW in 0x50-0x5F (bval={bval})",  0x50 <= vl <= 0x5F, True)
                self.check(f"CHK in 49-62 (cmd={cmd} bval={bval})", 49 <= chk <= 62, True)
                self.check(f"CHK != SYNC (cmd={cmd} bval={bval})",  chk != SYNC,     True)

        print("\n=== Checksum verification ===")

        for cmd in (CMD_LED, CMD_LEFT, CMD_RIGHT, CMD_KILL):
            for bval in (0, 1, 127, 255):
                wire = encode(cmd, bval)
                _, c, vh, vl, chk = wire
                self.check(
                    f"checksum cmd={cmd} bval={bval}",
                    chk, c ^ vh ^ vl,
                )

        print("\n=== Speed encoding tests ===")

        speed_cases = [
            ( 0.0,   0),
            ( 0.5,  50),
            ( 1.0, 100),
            (-0.5, 150),
            (-1.0, 200),
        ]
        for speed, expected_byte in speed_cases:
            bval = speed_to_byte(speed)
            self.check(f"speed_to_byte({speed})", bval, expected_byte)
            recovered = byte_to_speed(bval)
            self.check(f"byte_to_speed({bval})", recovered, speed)

        return self.failed == 0


# ---------------------------------------------------------------------------
# Hardware tests (require live Yukon over USB serial)
# ---------------------------------------------------------------------------

class YukonTester:
    RETRIES = 2

    def __init__(self, port, baud=115200):
        import serial
        # dsrdtr=False prevents DTR toggling, which can trigger RP2040 reset
        self.ser = serial.Serial(port, baud, timeout=0.1, dsrdtr=False)
        time.sleep(1.0)
        self.ser.reset_input_buffer()
        self.passed = 0
        self.failed = 0
        # Background reader: routes ACK/NAK to _ack_queue, data packets to _sensor_queue
        self._ack_queue    = queue.Queue()
        self._sensor_queue = queue.Queue()
        self._stop_reader  = threading.Event()
        self._reader = threading.Thread(target=self._reader_thread, daemon=True)
        self._reader.start()

    def _reader_thread(self):
        """Owns all serial RX.
        - ACK/NAK              → _ack_queue
        - Data packets (0x30-0x36) → _sensor_queue
        - Text lines           → print Fault/error lines only
        """
        import serial as _serial
        line_buf = b""
        pkt_buf  = b""
        in_pkt   = False

        while not self._stop_reader.is_set():
            try:
                data = self.ser.read(1)
            except _serial.SerialException:
                break
            if not data:
                continue
            b = data[0]

            if b == ACK:
                self._ack_queue.put(True)
                in_pkt = False; line_buf = b""
                continue
            if b == NAK:
                self._ack_queue.put(False)
                in_pkt = False; line_buf = b""
                continue
            if b == SYNC:
                in_pkt = True; pkt_buf = data
                continue
            if in_pkt:
                pkt_buf += data
                if len(pkt_buf) == 5:
                    _, rtype, v_high, v_low, chk = pkt_buf
                    if (0x30 <= rtype <= 0x36
                            and 0x40 <= v_high <= 0x4F
                            and 0x50 <= v_low  <= 0x5F
                            and chk == rtype ^ v_high ^ v_low):
                        resp_id = rtype - 0x30
                        value   = ((v_high - 0x40) << 4) | (v_low - 0x50)
                        self._sensor_queue.put((resp_id, value))
                    in_pkt = False; pkt_buf = b""
                continue

            # Plain text byte
            line_buf += data
            if b == ord('\n'):
                line = line_buf.strip()
                if line.startswith(b"Fault") or line.startswith(b"Sensor error"):
                    print(f"  [device] {line.decode('ascii', errors='replace')}")
                line_buf = b""

    def close(self):
        self._stop_reader.set()
        self.ser.close()

    def _send(self, cmd_code, byte_value):
        self.ser.write(encode(cmd_code, byte_value))

    def _read_ack(self, timeout=2.0):
        """
        Wait for ACK or NAK from the background reader thread.
        Returns True=ACK, False=NAK, None=timeout.
        """
        try:
            return self._ack_queue.get(timeout=timeout)
        except queue.Empty:
            return None

    def request_sensors(self, timeout=2.0):
        """
        Send CMD_SENSOR and return a decoded sensor dict, or None on failure.
        Keys: v (V), i (A), t (°C), tL (°C), tR (°C), fL (0/1), fR (0/1)
        """
        while not self._ack_queue.empty():
            self._ack_queue.get_nowait()
        while not self._sensor_queue.empty():
            self._sensor_queue.get_nowait()
        self._send(CMD_SENSOR, 0)
        if self._read_ack(timeout=timeout) is not True:
            return None
        sensors = {}
        while not self._sensor_queue.empty():
            resp_id, raw = self._sensor_queue.get_nowait()
            if resp_id in SENSOR_SCALE:
                sensors[SENSOR_NAMES[resp_id]] = raw / SENSOR_SCALE[resp_id]
        return sensors if len(sensors) == 7 else None

    def test_sensor(self):
        print("\n--- Sensor tests ---")
        sensors = self.request_sensors()
        if sensors is None:
            print("  FAIL  sensor request (no response)")
            self.failed += 1
            return
        print(f"  PASS  sensor request")
        self.passed += 1
        for key, val in sensors.items():
            print(f"         {key} = {val}")

    def _check(self, label, cmd_code, byte_value):
        """Send command, expect ACK. Retries on NAK or timeout."""
        result = None
        for attempt in range(self.RETRIES + 1):
            self._send(cmd_code, byte_value)
            result = self._read_ack()
            if result is True:
                print(f"  PASS  {label}")
                self.passed += 1
                return
            if attempt < self.RETRIES:
                status = 'NAK' if result is False else 'timeout'
                print(f"  RETRY {label} (attempt {attempt + 1}, {status})")
                time.sleep(0.1)
        status = 'NAK' if result is False else 'timeout'
        print(f"  FAIL  {label} ({status} after {self.RETRIES} retries)")
        self.failed += 1

    def _check_nak(self, label, raw_bytes):
        """Send raw bytes and expect NAK (error detection test)."""
        # drain any pending ACKs before sending malformed packet
        while not self._ack_queue.empty():
            self._ack_queue.get_nowait()
        self.ser.write(raw_bytes)
        result = self._read_ack(timeout=2.0)
        if result is False:
            print(f"  PASS  {label}")
            self.passed += 1
        else:
            status = 'ACK (unexpected)' if result is True else 'timeout'
            print(f"  FAIL  {label} (expected NAK, got {status})")
            self.failed += 1

    def test_led(self):
        print("\n--- LED tests ---")
        self._check("LED A on",  CMD_LED, 1)
        self._check("LED A off", CMD_LED, 0)
        self._check("LED B on",  CMD_LED, 3)
        self._check("LED B off", CMD_LED, 2)

    def test_left_motors(self):
        print("\n--- Left motor tests ---")
        self._check("Left fwd 50%",  CMD_LEFT, speed_to_byte( 0.5))
        self._check("Left fwd 100%", CMD_LEFT, speed_to_byte( 1.0))
        self._check("Left rev 50%",  CMD_LEFT, speed_to_byte(-0.5))
        self._check("Left stop",     CMD_LEFT, speed_to_byte( 0.0))

    def test_right_motors(self):
        print("\n--- Right motor tests ---")
        self._check("Right fwd 50%",  CMD_RIGHT, speed_to_byte( 0.5))
        self._check("Right fwd 100%", CMD_RIGHT, speed_to_byte( 1.0))
        self._check("Right rev 50%",  CMD_RIGHT, speed_to_byte(-0.5))
        self._check("Right stop",     CMD_RIGHT, speed_to_byte( 0.0))

    def test_kill(self):
        print("\n--- Kill motors test ---")
        self._send(CMD_LEFT,  speed_to_byte(0.8))
        self._send(CMD_RIGHT, speed_to_byte(0.8))
        time.sleep(0.3)
        # Drain ACKs from the un-awaited LEFT/RIGHT sends above
        while not self._ack_queue.empty():
            self._ack_queue.get_nowait()
        self._check("Kill all motors", CMD_KILL, 0)

    def test_error_detection(self):
        print("\n--- Error detection tests ---")

        # Corrupt the checksum byte — device should NAK
        pkt = bytearray(encode(CMD_LEFT, 50))
        pkt[4] ^= 0x01
        self._check_nak("Bad checksum rejected", bytes(pkt))

        # V_HIGH out of range — device should NAK
        bad_vhigh = bytes([SYNC, 0x22, 0x30, 0x50, 0x22 ^ 0x30 ^ 0x50])
        self._check_nak("Bad V_HIGH rejected", bad_vhigh)

        # CMD out of range — device should NAK
        bad_cmd = bytes([SYNC, 0x10, 0x40, 0x50, 0x10 ^ 0x40 ^ 0x50])
        self._check_nak("Bad CMD rejected", bad_cmd)

    def test_ramp(self, duration=30.0):
        """
        Ramp motors through a 30-second profile:
          0- 5s  ramp forward  0% → 100%
          5-10s  hold forward  100%
         10-13s  ramp down     100% → 50%
         13-16s  hold forward  50%
         16-20s  ramp reverse  50% → -50%
         20-25s  hold reverse  -50%
         25-28s  ramp to stop  -50% → 0%
         28-30s  stopped
        Prints speed updates every second and kills motors on exit.
        """
        print(f"\n--- Motor ramp test ({duration:.0f}s) ---")

        # (start_t, end_t, start_speed, end_speed, label)
        segments = [
            ( 0,  5, 0.0,  1.0, "ramp fwd 0→100%"),
            ( 5, 10, 1.0,  1.0, "hold fwd 100%"),
            (10, 13, 1.0,  0.5, "ramp down 100→50%"),
            (13, 16, 0.5,  0.5, "hold fwd 50%"),
            (16, 20, 0.5, -0.5, "ramp reverse 50→-50%"),
            (20, 25,-0.5, -0.5, "hold rev -50%"),
            (25, 28,-0.5,  0.0, "ramp to stop"),
            (28, 30, 0.0,  0.0, "stopped"),
        ]

        UPDATE_HZ = 10          # speed updates per second
        interval  = 1.0 / UPDATE_HZ
        start     = time.time()
        last_left = last_right = None
        last_report = -1        # last whole second reported

        try:
            while True:
                elapsed = time.time() - start
                if elapsed >= duration:
                    break

                # find current segment
                speed = 0.0
                seg_label = "stopped"
                for s0, s1, v0, v1, label in segments:
                    if s0 <= elapsed < s1:
                        t = (elapsed - s0) / (s1 - s0) if s1 > s0 else 1.0
                        speed = v0 + t * (v1 - v0)
                        seg_label = label
                        break

                left_b  = speed_to_byte(speed)
                right_b = speed_to_byte(speed)

                if left_b != last_left:
                    self._send(CMD_LEFT, left_b)
                    self._read_ack(timeout=0.5)
                    last_left = left_b

                if right_b != last_right:
                    self._send(CMD_RIGHT, right_b)
                    self._read_ack(timeout=0.5)
                    last_right = right_b

                whole_sec = int(elapsed)
                if whole_sec != last_report:
                    sensors = self.request_sensors(timeout=0.5)
                    if sensors:
                        print(f"  t={elapsed:5.1f}s  speed={speed:+.2f}  [{seg_label}]"
                              f"  v={sensors['v']:.2f}V i={sensors['i']:.3f}A"
                              f"  t={sensors['t']:.1f} tL={sensors['tL']:.1f} tR={sensors['tR']:.1f}°C"
                              f"  fL={int(sensors['fL'])} fR={int(sensors['fR'])}")
                    else:
                        print(f"  t={elapsed:5.1f}s  speed={speed:+.2f}  [{seg_label}]")
                    last_report = whole_sec

                time.sleep(interval)

        finally:
            # Always kill motors when the test ends
            self._send(CMD_KILL, 0)
            self._read_ack(timeout=1.0)
            print("  Motors killed — ramp test complete")

    def summary(self):
        total = self.passed + self.failed
        print(f"\n{'='*44}")
        print(f"Hardware results: {self.passed}/{total} passed", end="")
        print(f"  ({self.failed} failed)" if self.failed else "  (all passed)")
        print("=" * 44)
        return self.failed == 0


def find_yukon_port():
    import serial.tools.list_ports
    for p in serial.tools.list_ports.comports():
        if p.vid == 0x2E8A:         # Raspberry Pi / RP2040 vendor ID
            return p.device
    ports = serial.tools.list_ports.comports()
    return ports[0].device if ports else None


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Test Yukon main.py")
    parser.add_argument("--port",    help="Serial port, e.g. /dev/ttyACM0")
    parser.add_argument("--baud",    type=int, default=115200)
    parser.add_argument("--dry-run", action="store_true",
                        help="Run encoder tests only (no hardware required)")
    parser.add_argument("--ramp", action="store_true",
                        help="Run 30-second motor ramp test instead of the standard suite")
    args = parser.parse_args()

    enc = EncoderTests()
    enc_ok = enc.run()

    enc_total = enc.passed + enc.failed
    print(f"\n{'='*44}")
    print(f"Encoder results: {enc.passed}/{enc_total} passed", end="")
    print(f"  ({enc.failed} failed)" if enc.failed else "  (all passed)")
    print("=" * 44)

    if args.dry_run:
        sys.exit(0 if enc_ok else 1)

    try:
        import serial
    except ImportError:
        print("\npyserial not installed. Run:  pip install pyserial")
        sys.exit(1)

    port = args.port or find_yukon_port()
    if not port:
        print("\nNo serial port found. Use --port or --dry-run.")
        sys.exit(1)

    print(f"\nConnecting to Yukon on {port} @ {args.baud} baud ...")
    try:
        tester = YukonTester(port, args.baud)
    except serial.SerialException as e:
        print(f"Error: {e}")
        sys.exit(1)

    try:
        if args.ramp:
            tester.test_ramp()
        else:
            tester.test_led()
            tester.test_left_motors()
            tester.test_right_motors()
            tester.test_kill()
            tester.test_sensor()
            tester.test_error_detection()
    finally:
        tester.close()

    if not args.ramp:
        hw_ok = tester.summary()
        sys.exit(0 if (enc_ok and hw_ok) else 1)


if __name__ == "__main__":
    main()
