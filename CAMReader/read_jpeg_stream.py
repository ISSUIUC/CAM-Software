"""
Read JPEG frames from ESP32-P4 over serial.

Expected serial format:

*FRAME \n

**DONE\n

Writes each frame to:
output.jpg (latest frame)
output_stream.mjpg (continuous stream)

Optional: live preview via OpenCV.
Optional: web MJPEG stream via Flask at http://0.0.0.0:5002/video_feed
"""

import argparse
import queue
import threading
import time

import io
import serial
import numpy as np
from PIL import Image, ImageFile
from combine_images import re_combine

from flask import Flask, Response, render_template_string
from flask_cors import CORS
from gevent.pywsgi import WSGIServer

ImageFile.LOAD_TRUNCATED_IMAGES = True

try:
    import cv2
    HAS_DISPLAY = True
except ImportError:
    HAS_DISPLAY = False

# PORT = "/dev/cu.usbmodem01"
PORT = "COM4"

BAUD = 115200

OUTPUT_SINGLE = "output.jpg"
OUTPUT_STREAM = "output_stream.mjpg"

PREVIEW_WINDOW = "JPEG Preview"
PREVIEW_COMBINED = "JPEG COMBINED"

WEB_PORT = 5002

# ------------------------------------------------------------
# Shared latest frame (thread-safe)
# ------------------------------------------------------------

latest_frame_lock = threading.Lock()
latest_frame: bytes = b""

INDEX_HTML = """
<!DOCTYPE html>
<html>
<head>
  <title>ESP32-P4 Live Stream</title>
  <style>
    body { margin: 0; background: #111; display: flex; flex-direction: column;
           align-items: center; justify-content: center; height: 100vh; color: #eee;
           font-family: monospace; }
    img  { max-width: 100%; max-height: 90vh; border: 2px solid #444; }
    h2   { margin-bottom: 12px; letter-spacing: 2px; }
  </style>
</head>
<body>
  <h2>ESP32-P4 LIVE</h2>
  <img src="/video_feed" alt="MJPEG Stream" />
</body>
</html>
"""

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
    try:
        pil_img = Image.open(io.BytesIO(jpeg_bytes))
        pil_img.load()
        img = np.array(pil_img)
        if len(img.shape) == 3 and img.shape[2] == 3:
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        re_combined_img = re_combine(img)
        return img, re_combined_img
    except Exception as e:
        print(f"  Pillow decode failed: {e}")

    arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
    return cv2.imdecode(arr, cv2.IMREAD_COLOR), 69


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

                buffer = bytearray()
                remaining = frame_size
                while remaining > 0:
                    chunk = ser.read(min(remaining, 4096))
                    if not chunk:
                        continue
                    buffer.extend(chunk)
                    remaining -= len(chunk)

                print(f"Received {len(buffer)} / {frame_size} bytes")

                while True:
                    done_line = ser.readline().decode("utf-8", errors="ignore").strip()
                    if done_line == "**DONE":
                        break

                jpeg = bytes(buffer)

                print(f"Raw first 16 bytes: {jpeg[:16].hex(' ')}")

                if jpeg[:2] != b"\xff\xd8":
                    print(
                        f"WARNING: buffer does NOT start with FF D8 — data is corrupt or mis-framed"
                    )

                frame_count += 1
                print(f"Frame {frame_count}: {len(jpeg)} bytes")

                # Push to local display queue
                if not frame_queue.full():
                    try:
                        frame_queue.put_nowait(jpeg)
                    except queue.Full:
                        pass

                # Push to web streaming slot (drop oldest, keep latest)
                global latest_frame
                with latest_frame_lock:
                    latest_frame = jpeg

    except Exception as e:
        print("Serial error:", e)

    finally:
        print("Stopped.")


# ------------------------------------------------------------
# Flask web server
# ------------------------------------------------------------

app = Flask(__name__)
CORS(app)


@app.route("/")
def index():
    return render_template_string(INDEX_HTML)


def generate_mjpeg():
    """Generator that yields the latest JPEG as an MJPEG boundary stream."""
    while True:
        with latest_frame_lock:
            frame = latest_frame

        if frame:
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
            )
        else:
            # No frame yet — yield a small delay to avoid busy-spin
            time.sleep(0.05)
            continue

        time.sleep(0.033)  # ~30 fps cap for web clients


@app.route("/video_feed")
def video_feed():
    return Response(
        generate_mjpeg(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


@app.route("/ping")
def ping():
    return "OK"


def web_server_worker():
    """Runs the Flask/gevent WSGI server in a background daemon thread."""
    print(f"[web] Starting MJPEG server at http://0.0.0.0:{WEB_PORT}")
    http_server = WSGIServer(("0.0.0.0", WEB_PORT), app)
    http_server.serve_forever()


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

    # --- Serial worker thread ---
    worker = threading.Thread(
        target=serial_worker,
        args=(ser, frame_queue, stop_event),
        daemon=True,
    )
    worker.start()

    # --- Web server thread ---
    web_thread = threading.Thread(target=web_server_worker, daemon=True)
    web_thread.start()

    if HAS_DISPLAY:
        cv2.namedWindow(PREVIEW_WINDOW, cv2.WINDOW_NORMAL)
        cv2.namedWindow(PREVIEW_COMBINED, cv2.WINDOW_NORMAL)
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
                        stream_file.write(chunk)
                        stream_file.flush()
                    except queue.Empty:
                        break

                if chunk is not None:
                    img, recombined_image = decode_jpeg(chunk)
                    if img is not None:
                        last_img = img
                    else:
                        print(
                            f"cv2.imdecode FAILED for {len(chunk)} byte frame "
                            f"(first 4: {chunk[:4].hex(' ')})"
                        )

                if last_img is not None:
                    cv2.imshow(PREVIEW_WINDOW, last_img)
                    if recombined_image is not None:
                        cv2.imshow(PREVIEW_COMBINED, recombined_image)
                    else:
                        cv2.imshow(PREVIEW_COMBINED, last_img)

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