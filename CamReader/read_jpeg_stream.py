"""
Read JPEG frames from ESP32-P4 over serial.

Expected serial format:

    *FRAME <byte_count>\n
    <exactly byte_count bytes of JPEG data>
    **DONE\n

Writes each frame to:
    output.jpg  (latest frame)
    output_stream.mjpg (continuous stream)

Optional: live preview via OpenCV.
"""

import argparse
import queue
import threading
import time

import serial
import numpy as np

try:
    import cv2

    HAS_DISPLAY = True
except ImportError:
    HAS_DISPLAY = False
PORT = "COM4"

BAUD = 115200

OUTPUT_SINGLE = "output.jpg"
OUTPUT_STREAM = "output_stream.mjpg"

PREVIEW_WINDOW = "JPEG Preview"


# ------------------------------------------------------------
# JPEG helpers
# ------------------------------------------------------------


def extract_jpeg(buffer: bytes):
    """
    Extract valid JPEG from buffer using SOI/EOI markers.
    Returns bytes or None.
    """
    start = buffer.find(b"\xff\xd8")  # SOI
    end = buffer.find(b"\xff\xd9")  # EOI

    if start != -1 and end != -1 and end > start:
        return buffer[start : end + 2]

    return None


def decode_jpeg(jpeg_bytes: bytes):
    """
    Decode JPEG to BGR numpy array.
    """
    arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
    return cv2.imdecode(arr, cv2.IMREAD_COLOR)


# ------------------------------------------------------------
# Serial worker
# ------------------------------------------------------------


def serial_worker(ser, frame_queue, stop_event):
    frame_count = 0

    stream_file = open(OUTPUT_STREAM, "wb")

    try:
        while not stop_event.is_set():

            line = ser.readline().decode("utf-8", errors="ignore").strip()
            print(f"< {line}")

            if line.startswith("*FRAME"):
                parts = line.split()
                if len(parts) < 2:
                    print("Got *FRAME with no size, skipping")
                    continue

                frame_size = int(parts[1])
                print(f"Got *FRAME, expecting {frame_size} bytes")

                # Read exactly frame_size bytes
                buffer = bytearray()
                remaining = frame_size
                while remaining > 0:
                    chunk = ser.read(min(remaining, 4096))
                    if not chunk:
                        continue
                    buffer.extend(chunk)
                    remaining -= len(chunk)

                print(f"Received {len(buffer)} / {frame_size} bytes")

                # Read until **DONE
                while True:
                    done_line = ser.readline().decode("utf-8", errors="ignore").strip()
                    if done_line == "**DONE":
                        break

                # Diagnostic: print first 16 bytes and scan for SOI
                print(f"First 16 bytes: {buffer[:16].hex(' ')}")
                soi_pos = bytes(buffer).find(b"\xff\xd8")
                if soi_pos >= 0:
                    print(f"SOI (FF D8) found at offset {soi_pos}")
                else:
                    print("SOI (FF D8) NOT found anywhere in buffer!")

                # Find JPEG data — use SOI position if not at start
                if soi_pos == 0:
                    jpeg = bytes(buffer)
                elif soi_pos > 0:
                    jpeg = bytes(buffer[soi_pos:])
                    print(f"Trimmed {soi_pos} leading bytes")
                else:
                    jpeg = None

                if jpeg:
                    frame_count += 1
                    print(f"Frame {frame_count}: {len(jpeg)} bytes")

                    # Save latest frame
                    with open(OUTPUT_SINGLE, "wb") as f:
                        f.write(jpeg)

                    # Append to MJPEG stream
                    stream_file.write(jpeg)
                    stream_file.flush()

                    if not frame_queue.full():
                        try:
                            frame_queue.put_nowait(jpeg)
                        except queue.Full:
                            pass
                else:
                    print(f"Invalid JPEG frame, no SOI found")

    except Exception as e:
        print("Serial error:", e)

    finally:
        stream_file.close()
        print("Stopped.")


# ------------------------------------------------------------
# Main
# ------------------------------------------------------------


def main():
    parser = argparse.ArgumentParser(description="Read JPEG stream from ESP32")
    args = parser.parse_args()

    print(f"Opening {PORT} @ {BAUD}")
    ser = serial.Serial(PORT, BAUD)

    frame_queue = queue.Queue(maxsize=2)
    stop_event = threading.Event()

    worker = threading.Thread(
        target=serial_worker,
        args=(ser, frame_queue, stop_event),
        daemon=True,
    )
    worker.start()

    if HAS_DISPLAY:
        cv2.namedWindow(PREVIEW_WINDOW, cv2.WINDOW_NORMAL)
        print("Press 'q' in preview window to quit.")
    else:
        print("Install opencv-python for live preview.")

    try:
        last_img = None

        while not stop_event.is_set():

            if HAS_DISPLAY:
                chunk = None

                while True:
                    try:
                        chunk = frame_queue.get_nowait()
                    except queue.Empty:
                        break

                if chunk is not None:
                    img = decode_jpeg(chunk)
                    if img is not None:
                        last_img = img

                if last_img is not None:
                    cv2.imshow(PREVIEW_WINDOW, last_img)

                if cv2.waitKey(1) & 0xFF == ord("q"):
                    stop_event.set()
            else:
                time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nStopping...")

    finally:
        stop_event.set()
        worker.join(timeout=2.0)
        ser.close()
        if HAS_DISPLAY:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
