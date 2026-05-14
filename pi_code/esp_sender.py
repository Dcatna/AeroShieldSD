import cv2
from ultralytics import YOLO
import json
import time
import signal
import sys
import serial
from picamera2 import Picamera2

# ========================= CONFIG =========================
CAMERA_ID = "PI_4"
CONF_THRESHOLD = 0.5
MODEL = "yolov8n.pt"

UART_PORT = "/dev/serial0"
UART_BAUD = 57600

SYNC1 = 0xAA
SYNC2 = 0x55

MAX_DETECTIONS_PER_FRAME = 10  # set to None for unlimited
SLEEP_S = 0.05                 # 50ms => ~20 pkt/s max
PRINT_HEX_EVERY = 10           # print hex every N frames
# ==========================================================

def checksum8(data: bytes) -> int:
    return sum(data) & 0xFF

def make_packet(seq: int, payload: bytes) -> bytes:
    length = len(payload)
    header = bytes([
        SYNC1, SYNC2,
        length & 0xFF, (length >> 8) & 0xFF,
        seq & 0xFF, (seq >> 8) & 0xFF,
    ])
    crc = checksum8(header[2:] + payload)
    return header + payload + bytes([crc])

def main():
    try:
        ser = serial.Serial(UART_PORT, UART_BAUD, timeout=0, write_timeout=1)
        print(f"UART opened: {UART_PORT} @ {UART_BAUD}")
    except Exception as e:
        print(f"Failed to open UART ({UART_PORT}): {e}")
        sys.exit(1)

    print(f"Loading model: {MODEL}")
    model = YOLO(MODEL)
    print("Model loaded")

    cam = Picamera2()
    cfg = cam.create_preview_configuration(
        main={"format": "BGR888", "size": (320, 320)},
        sensor={"output_size": (3280, 2464)},  # change to match your camera module
        controls={
            "FrameDurationLimits": (1, 33333),
            "ExposureTime": 5000,   # 5ms max exposure
            "AnalogueGain": 4.0
        },
    )
    cam.configure(cfg)
    # cam.set_controls({"FrameRate": 30})
    # cam.video_configuration.controls.FrameRate = 25.0
    cam.start()

    running = True
    def signal_handler(sig, frame):
        nonlocal running
        print("\nShutting down sender...")
        running = False
    signal.signal(signal.SIGINT, signal_handler)

    seq = 0
    frame_i = 0

    last_stat = time.time()
    frames = 0
    bytes_sent = 0

    print("Sender started – Ctrl+C to stop (UART streaming enabled)")

    try:
        while running:
            frame = cam.capture_array()

            # cv2.imshow("Picamera2", frame) # testing zoom
            # if cv2.waitKey(1) & 0xFF == ord('q'): break # testing zoom

            frame_h, frame_w = frame.shape[:2]

            results = model(frame, imgsz=320, verbose=False)

            detections = []
            for result in results:
                for box in result.boxes:
                    conf = float(box.conf[0])
                    if conf < CONF_THRESHOLD:
                        continue

                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    cx = (x1 + x2) / 2.0
                    cy = (y1 + y2) / 2.0
                    label = result.names[int(box.cls[0])]

                    detections.append({
                        "label": label,
                        "confidence": round(conf, 3),
                        "cx": round(cx, 1),
                        "cy": round(cy, 1),
                    })

            if MAX_DETECTIONS_PER_FRAME is not None and len(detections) > MAX_DETECTIONS_PER_FRAME:
                detections.sort(key=lambda d: d["confidence"], reverse=True)
                detections = detections[:MAX_DETECTIONS_PER_FRAME]

            frame_msg = {
                "t": time.time(),
                "camera_id": CAMERA_ID,
                "w": frame_w,
                "h": frame_h,
                "detections": detections,
            }

            payload_bytes = json.dumps(frame_msg, separators=(",", ":")).encode("utf-8")
            packet = make_packet(seq, payload_bytes)

            frame_i += 1
            print(f"\n--- UART TX seq={seq} payload_len={len(payload_bytes)} packet_len={len(packet)} dets={len(detections)} ---")
            print("PAYLOAD(JSON):", payload_bytes.decode("utf-8", errors="replace"))

            if PRINT_HEX_EVERY and (frame_i % PRINT_HEX_EVERY == 0):
                print("PACKET(HEX first 200):", packet[:200].hex(), "...")

            ser.write(packet)
            seq = (seq + 1) & 0xFFFF

            # time.sleep(SLEEP_S)

            frames += 1
            bytes_sent += len(packet)
            now = time.time()
            # print(f"NOW: {now}\nLAST_STAT: {last_stat}")
            if now - last_stat >= 1.0:
                fps = frames / (now - last_stat)
                kbps = (bytes_sent * 8) / (now - last_stat) / 1000.0
                print(f"TX rate: {fps:.1f} pkt/s, {kbps:.1f} kbps, last payload={len(payload_bytes)} bytes, dets={len(detections)}")
                frames = 0
                bytes_sent = 0
                last_stat = now

    finally:
        try:
            cam.close()
        except Exception:
            pass
        try:
            ser.close()
        except Exception:
            pass
        print("Sender shut down cleanly")

if __name__ == "__main__":
    main()
