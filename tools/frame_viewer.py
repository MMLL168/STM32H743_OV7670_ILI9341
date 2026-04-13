"""
Frame viewer for raw RGB565 frames received over STM32 UART.

Usage:
    python frame_viewer.py [COM_PORT]

Examples:
    python frame_viewer.py COM13
    python frame_viewer.py

Protocol:
    Header: "FRAME" + width_be16 + height_be16 + bpp(1) = 10 bytes
    Data:   width * height * bpp bytes
    Tail:   "END\\n"

Requirements:
    pip install pyserial pillow
    Optional: pip install numpy
"""

import struct
import sys

import serial
import serial.tools.list_ports
from PIL import Image

NUMPY_IMPORT_ERROR = None
try:
    import numpy as np
except Exception as exc:
    np = None
    NUMPY_IMPORT_ERROR = exc

BAUD = 921600


def find_port():
    """Auto-detect the ST-Link virtual COM port."""
    for port in serial.tools.list_ports.comports():
        desc = (port.description or "").lower()
        if "stlink" in desc or "st-link" in desc or "virtual com" in desc:
            return port.device

    ports = list(serial.tools.list_ports.comports())
    return ports[0].device if ports else None


def rgb565_to_image(data, width, height):
    """Convert raw RGB565 bytes to a Pillow RGB image."""
    if np is not None:
        pixels = np.frombuffer(data, dtype=np.uint16).reshape((height, width))
        r = ((pixels >> 11) & 0x1F).astype(np.uint8)
        g = ((pixels >> 5) & 0x3F).astype(np.uint8)
        b = (pixels & 0x1F).astype(np.uint8)
        rgb = np.stack([r * 255 // 31, g * 255 // 63, b * 255 // 31], axis=-1)
        return Image.fromarray(rgb.astype(np.uint8), "RGB")

    rgb = bytearray(width * height * 3)
    for i in range(width * height):
        px = data[i * 2] | (data[i * 2 + 1] << 8)
        rgb[i * 3 + 0] = ((px >> 11) & 0x1F) * 255 // 31
        rgb[i * 3 + 1] = ((px >> 5) & 0x3F) * 255 // 63
        rgb[i * 3 + 2] = (px & 0x1F) * 255 // 31

    return Image.frombytes("RGB", (width, height), bytes(rgb))


def byteswap_16(data):
    swapped = bytearray(len(data))
    swapped[0::2] = data[1::2]
    swapped[1::2] = data[0::2]
    return bytes(swapped)


def wordswap_32(data):
    swapped = bytearray(data)
    limit = len(data) - (len(data) % 4)
    for i in range(0, limit, 4):
        swapped[i:i + 4] = data[i + 2:i + 4] + data[i:i + 2]
    return bytes(swapped)


def main():
    if NUMPY_IMPORT_ERROR is not None:
        print(f"NumPy unavailable, using pure-Python conversion: {NUMPY_IMPORT_ERROR}")

    port = sys.argv[1] if len(sys.argv) > 1 else find_port()
    if not port:
        print("ERROR: No serial port found. Specify COM port as argument.")
        sys.exit(1)

    print(f"Opening {port} @ {BAUD} baud...")
    ser = serial.Serial(port, BAUD, timeout=10)
    ser.reset_input_buffer()

    print("Waiting for frame header 'FRAME'...")

    buf = b""
    while True:
        byte = ser.read(1)
        if not byte:
            print("Timeout waiting for data.")
            continue

        buf += byte
        if len(buf) > 1024:
            buf = buf[-256:]

        idx = buf.find(b"FRAME")
        if idx < 0:
            continue

        rest = buf[idx + 5:]
        while len(rest) < 5:
            rest += ser.read(5 - len(rest))

        width = struct.unpack(">H", rest[0:2])[0]
        height = struct.unpack(">H", rest[2:4])[0]
        bpp = rest[4]
        print(f"Frame: {width}x{height}, {bpp} bytes/pixel")

        total = width * height * bpp
        print(f"Receiving {total} bytes...")
        data = rest[5:]
        while len(data) < total:
            chunk = ser.read(total - len(data))
            if not chunk:
                print(f"Timeout! Got {len(data)}/{total} bytes")
                break
            data += chunk

        if len(data) >= total:
            pixel_data = data[:total]
            variants = {
                "frame_normal.png": pixel_data,
                "frame_swapped.png": byteswap_16(pixel_data),
                "frame_wordswapped.png": wordswap_32(pixel_data),
                "frame_word_byteswapped.png": byteswap_16(wordswap_32(pixel_data)),
            }

            for filename, raw in variants.items():
                img = rgb565_to_image(raw, width, height)
                img.save(filename)
                img.resize((width * 3, height * 3), Image.NEAREST).show()

            print(
                "Saved: frame_normal.png, frame_swapped.png, "
                "frame_wordswapped.png, frame_word_byteswapped.png"
            )

        break

    ser.close()
    print("Done.")


if __name__ == "__main__":
    main()
