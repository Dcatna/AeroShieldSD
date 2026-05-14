import json
import time
import zmq

BIND_ADDR = f"tcp://10.155.35.103:5555"

ctx = zmq.Context.instance()
sock = ctx.socket(zmq.PULL)
sock.bind(BIND_ADDR)

print(f"Base station PULL receiver bound to {BIND_ADDR}")


def try_decode_payload(payload: bytes):
    # First try: payload is directly JSON
    try:
        text = payload.decode("utf-8")
        data = json.loads(text)
        return None, data, "direct_json"
    except Exception:
        pass

    # Second try: first 4 bytes are sender_id, rest is JSON
    if len(payload) >= 4:
        sender_id = int.from_bytes(payload[:4], byteorder="little")
        payload_wo_id = payload[4:]

        try:
            text = payload_wo_id.decode("utf-8")
            data = json.loads(text)
            return sender_id, data, "sender_id_plus_json"
        except Exception:
            return sender_id, None, "unknown_with_sender_prefix"

    return None, None, "unknown"


try:
    while True:
        msg = sock.recv_json()

        if msg.get("type") != "uart_frame":
            print(f"Unknown message type: {msg}")
            continue

        ts = msg.get("ts", time.time())
        seq = msg.get("seq")
        frame_len = msg.get("frame_len")
        payload_len = msg.get("payload_len")
        raw_frame_hex = msg.get("raw_frame_hex", "")
        payload_hex = msg.get("payload_hex", "")

        try:
            raw_frame = bytes.fromhex(raw_frame_hex)
            payload = bytes.fromhex(payload_hex)
        except Exception as e:
            print(f"Failed to decode hex from message: {e}")
            continue

        print("\n" + "=" * 60)
        print("Received frame from Pi")
        print(f"  ts         : {ts}")
        print(f"  seq        : {seq}")
        print(f"  frame_len  : {frame_len}")
        print(f"  payload_len: {payload_len}")

        sender_id, decoded_json, decode_mode = try_decode_payload(payload)
        print(f"  decode_mode: {decode_mode}")

        if sender_id is not None:
            print(f"  sender_id  : {sender_id}")

        if decoded_json is not None:
            print("  decoded JSON:")
            print(json.dumps(decoded_json, indent=2))
        else:
            print(f"  raw payload hex: {payload.hex()}")

        # Optional:
        # print(f"  raw frame hex: {raw_frame.hex()}")

except KeyboardInterrupt:
    print("\nStopping base station receiver...")
finally:
    sock.close(0)
    ctx.term()
    print("Receiver shut down cleanly")