#!/usr/bin/env python3
"""Ask a running µCAN adapter to restart into its USB DFU bootloader.

Usage: startdfu.py <serial-device>
Exit status 1 if the adapter rejected the request.
"""

import sys
import time

import serial

TIMEOUT_S = 2.0


def main():
    if len(sys.argv) != 2:
        print(__doc__.strip(), file=sys.stderr)
        return 2

    port = serial.Serial(sys.argv[1], 115200, timeout=0.05)

    # Finish any partial command the adapter is holding, and discard the reply
    port.write(b"\r")
    time.sleep(0.1)
    port.reset_input_buffer()

    port.write(b"__dfu\r")
    line = b""
    deadline = time.monotonic() + TIMEOUT_S
    try:
        while time.monotonic() < deadline:
            byte = port.read(1)
            if not byte:
                continue
            if byte in (b"\r", b"\a"):
                # Received CAN frames are non-empty lines; the reply is empty
                if not line:
                    if byte == b"\a":
                        print("Device rejected DFU entry command", file=sys.stderr)
                        return 1
                    print("Device entering DFU")
                    return 0
                line = b""
            else:
                line += byte
    except serial.SerialException:
        # The adapter detached before we read the reply
        print("Device entering DFU")
        return 0

    # Older firmware resets without replying; the caller waits for the
    # bootloader to appear and reports if it doesn't
    print("No response from device; assuming it is entering DFU")
    return 0


if __name__ == "__main__":
    sys.exit(main())
