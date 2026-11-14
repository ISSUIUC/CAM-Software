"""
Fixes up partial frames from EAGLE.

When EAGLE can't get every packet of a frame (and Reed-Solomon couldn't fix it), it still sends
the frame, with this line in front:

*FRAME <len> frag=<data bytes per packet> missing=<i>,<j>,...

The bytes where the missing packets should be are garbage. The CAM's JPEGs have no restart
markers, so once one byte of the image data is missing, everything after it is useless.

How we fix it:
1. If a missing packet is in the JPEG header, we copy the header from the last complete frame.
   The CAM's encoder settings never change, so every frame has the same header.
2. We cut the image data off at the first missing packet and add an end marker (EOI). Pillow
   decodes the part before the cut and paints every block after it flat gray.
3. We fill those gray blocks in with the previous frame.
"""

import io
import math

import cv2
import numpy as np
from PIL import Image, ImageFile

ImageFile.LOAD_TRUNCATED_IMAGES = True

SOI = b"\xff\xd8"  # start of image marker
EOI = b"\xff\xd9"  # end of image marker
SOS = 0xDA  # start of scan marker (the image data comes right after this segment)
FILL_GRAY = 128  # the color Pillow paints blocks it got no data for
SEAM_MCUS = 3  # blocks right before the gray ones that also decode wrong (worst case seen over ~1600 cuts of real frames)
REENCODE_QUALITY = 90


def parse_frame_line(line):
    # '*FRAME <len> [frag=<n> missing=<i,j,...>]' -> (frame size, packet size, list of missing packets)
    parts = line.split()
    frame_size = int(parts[1])

    fields = {}
    for part in parts[2:]:
        if "=" in part:
            name, value = part.split("=", 1)
            fields[name] = value

    packet_size = int(fields.get("frag", 0))

    missing_packets = []
    for number in fields.get("missing", "").split(","):
        if number != "":
            missing_packets.append(int(number))

    return frame_size, packet_size, missing_packets


def header_length(jpeg):
    # Number of bytes from the start of the file through the end of the SOS segment.
    # Returns None if the header doesn't make sense.
    if jpeg[:2] != SOI:
        return None

    pos = 2
    while pos + 4 <= len(jpeg):
        # every segment starts with 0xFF, then the marker byte, then a 2-byte big-endian length
        if jpeg[pos] != 0xFF:
            return None
        marker = jpeg[pos + 1]
        segment_length = jpeg[pos + 2] * 256 + jpeg[pos + 3]
        pos = pos + 2 + segment_length
        if marker == SOS:
            if pos <= len(jpeg):
                return pos
            return None
    return None


def decode_image(jpeg):
    # Returns (BGR image, MCU width, MCU height).
    # An MCU is the block of pixels the JPEG encoder works on at a time (8x8, 16x8 or 16x16).
    pil_image = Image.open(io.BytesIO(jpeg))
    pil_image.load()

    # pil_image.layer has one entry per color channel: (id, horizontal sampling, vertical sampling, quant table)
    h_samplings = []
    v_samplings = []
    for channel in pil_image.layer:
        h_samplings.append(channel[1])
        v_samplings.append(channel[2])
    mcu_width = 8 * max(h_samplings)
    mcu_height = 8 * max(v_samplings)

    image = cv2.cvtColor(np.array(pil_image.convert("RGB")), cv2.COLOR_RGB2BGR)
    return image, mcu_width, mcu_height


def fill_from_previous(image, previous_image, mcu_width, mcu_height):
    # Copies the previous frame into the gray blocks Pillow left after the cut.
    # Returns False if there weren't any gray blocks.
    is_gray = np.all(image == FILL_GRAY, axis=2)
    height, width = is_gray.shape
    blocks_per_row = math.ceil(width / mcu_width)
    blocks_per_column = math.ceil(height / mcu_height)

    # Blocks are decoded left to right, top to bottom, so the first all-gray one is where the data ran out
    first_gray_block = None
    for block in range(blocks_per_row * blocks_per_column):
        row = block // blocks_per_row
        col = block % blocks_per_row
        top = row * mcu_height
        left = col * mcu_width
        if is_gray[top:top + mcu_height, left:left + mcu_width].all():
            first_gray_block = block
            break

    if first_gray_block is None:
        return False

    # The last few blocks before it were decoded from cut-off data, so they're wrong too
    start_block = max(first_gray_block - SEAM_MCUS, 0)
    row = start_block // blocks_per_row
    col = start_block % blocks_per_row
    top = row * mcu_height
    bottom = top + mcu_height
    left = col * mcu_width

    # rest of the row the start block is in
    image[top:bottom, left:] = previous_image[top:bottom, left:]
    # every row below that
    image[bottom:] = previous_image[bottom:]
    return True


class FrameRepairer:
    def __init__(self):
        self.header = None  # header of the last complete frame
        self.last_jpeg = None  # last frame we handed out, complete or repaired

    def process(self, jpeg, packet_size, missing_packets):
        # Returns the JPEG to show, or None if nothing in this frame is usable.
        if not missing_packets:
            header_size = header_length(jpeg)
            if header_size:
                self.header = jpeg[:header_size]
            self.last_jpeg = jpeg
            return jpeg

        try:
            result = self.repair(jpeg, packet_size, missing_packets)
        except Exception as error:  # don't let one bad frame crash the serial reader
            print(f"  repair failed: {error}")
            return None
        if result is not None:
            self.last_jpeg = result
        return result

    def repair(self, jpeg, packet_size, missing_packets):
        if self.header is None or packet_size <= 0:
            return None
        header_size = len(self.header)

        # Find the first missing packet that has image data in it. Everything from there on is lost.
        cut_at = len(jpeg)
        for packet in sorted(missing_packets):
            packet_start = packet * packet_size
            packet_end = packet_start + packet_size
            if packet_end > header_size:
                cut_at = max(packet_start, header_size)
                break
        if cut_at <= header_size:
            return None

        if cut_at == len(jpeg):  # only header packets were lost
            return self.header + jpeg[header_size:]

        partial_jpeg = self.header + jpeg[header_size:cut_at] + EOI
        if self.last_jpeg is None:
            return partial_jpeg

        image, mcu_width, mcu_height = decode_image(partial_jpeg)
        previous_image, _, _ = decode_image(self.last_jpeg)
        if previous_image.shape != image.shape:
            return partial_jpeg
        if not fill_from_previous(image, previous_image, mcu_width, mcu_height):
            return partial_jpeg

        success, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, REENCODE_QUALITY])
        if success:
            return encoded.tobytes()
        return partial_jpeg
