import time
import argparse
import os
import serial

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--port", required=True)
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--payload_bytes", type=int, default=100)
    p.add_argument("--interval_ms", type=float, default=10)
    p.add_argument("--pattern", choices=["random", "increment"], default="increment")
    args = p.parse_args()
    ser = serial.Serial(args.port, args.baud, timeout=0)

    seq = 0
    dt = args.interval_ms / 1000.0
    payload_len = max(1, args.payload_bytes)

    print(f"writing to {args.port} @ {args.baud}, {payload_len}, bytes every {args.interval_ms} ms")

    last = time.time()
    sent = 0

    while True:
        if args.pattern == "random":
            payload = os.random(payload_len)

        else:
            payload = bytes([seq % 256]) * payload_len
            seq = (seq + 1) % 256

        n = ser.write(payload)
        sent += n

        now = time.time()
        if now - last >= 1.0:
            rate = sent / (now-last)
            print(f"TX UART: {rate:.1f} B/s ({rate*8/1000:.2f} kbps)")
            print("Payload (hex):", payload.hex())
            last = now
            sent = 0
        if dt > 0:
            time.sleep(dt)

if __name__ == "__main__":
    main()