#!/usr/bin/env python3
"""Open the CAN channel on a µCAN adapter and print what it receives.

Usage: serialdump.py <serial-device> [bitrate-index]
bitrate-index is the SLCAN S argument: 0..8 = 10k 20k 50k 100k 125k 250k 500k 800k 1M
(default 8, 1 Mbit/s).
"""

import sys

import serial


def main():
    if len(sys.argv) not in (2, 3):
        print(__doc__.strip(), file=sys.stderr)
        return 2

    bitrate = sys.argv[2] if len(sys.argv) == 3 else "8"
    port = serial.Serial(sys.argv[1], 115200, timeout=0.1)
    port.write(b"\rC\rS" + bitrate.encode("ascii") + b"\rO\r")

    try:
        while True:
            data = port.read(256)
            if data:
                sys.stdout.write(data.replace(b"\r", b"\n").replace(b"\a", b"<BELL>\n").decode("ascii", "replace"))
                sys.stdout.flush()
    except KeyboardInterrupt:
        port.write(b"C\r")
    return 0


if __name__ == "__main__":
    sys.exit(main())
