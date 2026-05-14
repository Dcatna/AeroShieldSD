import json
import time
import sys
import serial
import threading
import zmq

UART_PORT = "/dev/serial0"
UART_BAUD = 57600

BS_ZMQ_ADDR = "tcp://10.155.35.103:5555"   # change to basestation IP/port

SYNC1 = 0xAA
SYNC2 = 0x55
MAX_PAYLOAD = 256

running = True
is_leader = False
leader_lock = threading.Lock()


def checksum8(data: bytes) -> int:
    return sum(data) & 0xFF


try:
    ser = serial.Serial(UART_PORT, UART_BAUD, timeout=0, write_timeout=1)
    print(f"UART opened: {UART_PORT} @ {UART_BAUD}")
except Exception as e:
    print(f"Failed to open UART ({UART_PORT}): {e}")
    sys.exit(1)


# ZMQ setup
ctx = zmq.Context.instance()
push_sock = ctx.socket(zmq.PUSH)
push_sock.connect(BS_ZMQ_ADDR)
print(f"ZMQ PUSH connected to {BS_ZMQ_ADDR}")


def set_leader_state(new_state: bool, reason: str = ""):
    global is_leader
    with leader_lock:
        old = is_leader
        is_leader = new_state

    if old != new_state:
        print(f"Leader forwarding state changed: {old} -> {new_state} ({reason})")


def get_leader_state() -> bool:
    with leader_lock:
        return is_leader


def handle_status_line(line: bytes):
    """
    Handles ESP status messages like:
    @@{"type":"leader_status","event":"became_leader",...,"isLeader":true}
    """
    if not line.startswith(b"@@"):
        return

    try:
        text = line[2:].decode("utf-8", errors="replace").strip()
        msg = json.loads(text)
    except Exception as e:
        print(f"Failed to parse status line: {line!r} err={e}")
        return

    msg_type = msg.get("type")
    if msg_type == "leader_status":
        new_state = bool(msg.get("isLeader", False))
        event = msg.get("event", "unknown")
        set_leader_state(new_state, event)
        print(f"leader_status: {json.dumps(msg, indent=2)}")
    else:
        print(f"status/meta: {json.dumps(msg, indent=2)}")


def forward_raw_frame_to_bs(raw_frame: bytes, seq: int, payload: bytes):
    """
    Sends raw, undecoded UART frame to the basestation over ZMQ.
    We wrap it in JSON with hex so it's easy to inspect at the BS.
    """
    message = {
        "type": "uart_frame",
        "ts": time.time(),
        "seq": seq,
        "raw_frame_hex": raw_frame.hex(),
        "payload_hex": payload.hex(),
        "frame_len": len(raw_frame),
        "payload_len": len(payload),
    }

    try:
        push_sock.send_json(message, flags=zmq.NOBLOCK)
        print(f"Forwarded raw frame to BS: seq={seq} frame_len={len(raw_frame)}")
    except zmq.Again:
        print(f"ZMQ send would block, dropping frame seq={seq}")
    except Exception as e:
        print(f"Failed to send frame to BS: {e}")


def on_frame_received(payload: bytes, seq: int, raw_frame: bytes):
    """
    Called when a valid UART frame is received.
    raw_frame is the full undecoded UART frame including sync/len/seq/crc.
    """
    if get_leader_state():
        forward_raw_frame_to_bs(raw_frame, seq, payload)

    if len(payload) < 4:
        print(f"[seq={seq}] Frame too short to contain node ID")
        return

    # First 4 bytes = sender node ID (little-endian)
    sender_id = int.from_bytes(payload[:4], byteorder="little")
    payload_wo_id = payload[4:]

    try:
        text = payload_wo_id.decode("utf-8")
        data = json.loads(text)
    except Exception:
        print(f"[node={sender_id} seq={seq}] Raw payload ({len(payload_wo_id)}B): {payload_wo_id.hex()}")
        return

    print(f"\n[node={sender_id} seq={seq}] Decoded JSON:")
    print(json.dumps(data, indent=2))


def serial_reader():
    WAIT_SYNC1, WAIT_SYNC2, READ_LEN1, READ_LEN2, \
    READ_SEQ1, READ_SEQ2, READ_PAYLOAD, READ_CRC = range(8)

    state = WAIT_SYNC1
    payload_len = 0
    seq = 0
    payload = bytearray()
    frame_buf = bytearray()

    FRAME_TIMEOUT = 0.2
    last_byte_time = time.time()

    line_buf = bytearray()
    in_status_line = False
    at_prefix_candidate = False

    while running:
        if state != WAIT_SYNC1 and (time.time() - last_byte_time > FRAME_TIMEOUT):
            print("UART frame timeout, resetting parser")
            state = WAIT_SYNC1
            payload.clear()
            frame_buf.clear()

        raw = ser.read(256)
        if not raw:
            time.sleep(0.001)
            continue

        for b in raw:
            last_byte_time = time.time()

            # Handle status lines that start with @@
            if state == WAIT_SYNC1:
                if in_status_line:
                    line_buf.append(b)
                    if b == 0x0A:  # newline
                        handle_status_line(bytes(line_buf))
                        line_buf.clear()
                        in_status_line = False
                        at_prefix_candidate = False
                    continue

                if at_prefix_candidate:
                    if b == ord("@"):
                        in_status_line = True
                        line_buf = bytearray(b"@@")
                        at_prefix_candidate = False
                        continue
                    else:
                        at_prefix_candidate = False
                        # fall through and let normal frame parsing inspect this byte

                if b == ord("@"):
                    at_prefix_candidate = True
                    continue

            # Normal binary frame parsing
            if state == WAIT_SYNC1:
                if b == SYNC1:
                    frame_buf.clear()
                    frame_buf.append(b)
                    state = WAIT_SYNC2

            elif state == WAIT_SYNC2:
                if b == SYNC2:
                    frame_buf.append(b)
                    state = READ_LEN1
                else:
                    state = WAIT_SYNC1
                    frame_buf.clear()

            elif state == READ_LEN1:
                payload_len = b
                frame_buf.append(b)
                state = READ_LEN2

            elif state == READ_LEN2:
                payload_len |= (b << 8)
                frame_buf.append(b)

                if payload_len == 0 or payload_len > MAX_PAYLOAD:
                    print(f"Invalid payload len={payload_len}, resetting")
                    state = WAIT_SYNC1
                    frame_buf.clear()
                else:
                    state = READ_SEQ1

            elif state == READ_SEQ1:
                seq = b
                frame_buf.append(b)
                state = READ_SEQ2

            elif state == READ_SEQ2:
                seq |= (b << 8)
                frame_buf.append(b)
                payload.clear()
                state = READ_PAYLOAD

            elif state == READ_PAYLOAD:
                payload.append(b)
                frame_buf.append(b)
                if len(payload) >= payload_len:
                    state = READ_CRC

            elif state == READ_CRC:
                crc_rx = b
                frame_buf.append(b)

                hdr = bytes([
                    payload_len & 0xFF,
                    (payload_len >> 8) & 0xFF,
                    seq & 0xFF,
                    (seq >> 8) & 0xFF,
                ])
                crc_calc = checksum8(hdr + bytes(payload))

                if crc_calc == crc_rx:
                    raw_frame = bytes(frame_buf)
                    on_frame_received(bytes(payload), seq, raw_frame)
                else:
                    print(
                        f"CRC mismatch: calc={crc_calc} rx={crc_rx} "
                        f"len={payload_len} seq={seq}"
                    )

                state = WAIT_SYNC1
                payload.clear()
                frame_buf.clear()


reader_thread = threading.Thread(target=serial_reader, daemon=True)
reader_thread.start()

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("\nStopping receiver...")
finally:
    running = False
    reader_thread.join(timeout=1)

    try:
        push_sock.close(0)
        ctx.term()
    except Exception:
        pass

    ser.close()
    print("Receiver shut down cleanly")