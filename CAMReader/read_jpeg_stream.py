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

import io
import serial
import numpy as np
from PIL import Image, ImageFile

ImageFile.LOAD_TRUNCATED_IMAGES = True

try:
    import cv2

    HAS_DISPLAY = True
except ImportError:
    HAS_DISPLAY = False
PORT = "/dev/cu.usbmodem01"

BAUD = 115200

OUTPUT_SINGLE = "output.jpg"
OUTPUT_STREAM = "output_stream.mjpg"

PREVIEW_WINDOW = "JPEG Preview"


# ------------------------------------------------------------
# JPEG helpers
# ------------------------------------------------------------


def extract_jpegs(buffer: bytes):
    """
    Extract all valid JPEGs from buffer using SOI/EOI markers.
    Returns a list of bytes objects (may be empty).
    """
    results = []
    search_from = 0

    while search_from < len(buffer):
        start = buffer.find(b"\xff\xd8", search_from)  # SOI
        if start == -1:
            break

        end = buffer.find(b"\xff\xd9", start + 2)  # EOI
        if end == -1:
            break

        results.append(buffer[start : end + 2])
        search_from = end + 2

    return results


def decode_jpeg(jpeg_bytes: bytes):
    """
    Decode JPEG to BGR numpy array. Tries Pillow first (lenient),
    falls back to OpenCV.
    """
    # Pillow is much more lenient with corrupt JPEGs
    try:
        pil_img = Image.open(io.BytesIO(jpeg_bytes))
        pil_img.load()  # force decode
        img = np.array(pil_img)
        if len(img.shape) == 3 and img.shape[2] == 3:
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        return img
    except Exception as e:
        print(f"  Pillow decode failed: {e}")

    # Fallback to OpenCV
    arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
    return cv2.imdecode(arr, cv2.IMREAD_COLOR)


# ------------------------------------------------------------
# Serial worker
# ------------------------------------------------------------


def serial_worker(ser, frame_queue, stop_event):
    frame_count = 0

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

                jpeg = bytes(buffer)

                # Diagnostic: raw buffer start
                print(f"Raw first 16 bytes: {jpeg[:16].hex(' ')}")

                # If JPEG should start at byte 0 but doesn't, the data is corrupted from the start
                if jpeg[:2] != b"\xff\xd8":
                    print(
                        f"WARNING: buffer does NOT start with FF D8 — data is corrupt or mis-framed"
                    )

                frame_count += 1
                print(f"Frame {frame_count}: {len(jpeg)} bytes")

                # Save raw buffer as-is (don't trim — let us inspect the actual data)
                # with open(OUTPUT_SINGLE, "wb") as f:
                #     f.write(jpeg)
                # Also save numbered copy for comparison
                # with open(f"frame_{frame_count:04d}.bin", "wb") as f:
                #     f.write(jpeg)

                if not frame_queue.full():
                    try:
                        frame_queue.put_nowait(jpeg)

                    except queue.Full:
                        pass

    except Exception as e:
        print("Serial error:", e)

    finally:
        print("Stopped.")


# ------------------------------------------------------------
# Main
# ------------------------------------------------------------


def main():
    stream_file = open(OUTPUT_STREAM, "wb")

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
                        # Append to MJPEG stream
                        stream_file.write(chunk)
                        stream_file.flush()
                    except queue.Empty:
                        break

                if chunk is not None:
                    img = decode_jpeg(chunk)
                    if img is not None:
                        last_img = img
                    else:
                        print(
                            f"cv2.imdecode FAILED for {len(chunk)} byte frame (first 4: {chunk[:4].hex(' ')})"
                        )

                if last_img is not None:
                    cv2.imshow(PREVIEW_WINDOW, last_img)

                if cv2.waitKey(1) & 0xFF == ord("q"):
                    stop_event.set()
            else:
                time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nStopping...")

    finally:
        stream_file.close()
        stop_event.set()
        worker.join(timeout=2.0)
        ser.close()
        if HAS_DISPLAY:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
