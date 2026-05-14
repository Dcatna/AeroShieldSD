import time
import argparse
import serial

SYNC1 = 0xAA
SYNC2 = 0x55

def checksum8(data: bytes) -> int:
    return sum(data) & 0xFF

def make_packet(seq: int, payload: bytes) -> bytes:
    length = len(payload)
    header = bytes([
        SYNC1,
        SYNC2,
        length & 0xFF,
        (length >> 8) & 0xFF,
        seq & 0xFF,
        (seq >> 8) & 0xFF,
    ])
    crc = checksum8(header[2:] + payload)
    return header + payload + bytes([crc])

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--port", required=True)
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--interval_ms", type=float, default=4000)
    args = p.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0)

    dt = args.interval_ms / 1000.0
    seq = 0
    last = time.time()

    fixed_payload = b"HELLO_MESH_TEST_1234"[:20].ljust(20, b'_')

    print(f"writing fixed framed packets to {args.port} @ {args.baud}")

    while True:
        payload = fixed_payload
        pkt = make_packet(seq, payload)
        ser.write(pkt)
        ser.flush()
        print("TX packet hex:", pkt.hex())

        now = time.time()
        if now - last >= 1.0:
            print(f"TX seq={seq} len={len(payload)} crc=0x{pkt[-1]:02X}")
            print("TX payload:", payload)
            print("TX payload hex:", payload.hex())
            last = now

        if dt > 0:
            time.sleep(dt)

        # seq = (seq + 1) & 0xFFFF

if __name__ == "__main__":
    main()