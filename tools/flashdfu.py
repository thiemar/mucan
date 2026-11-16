#!/usr/bin/env python3
"""Stage, read back, and start µCAN firmware; only the app changes boot options.

Requires native libusb and pyserial. BOOT0 must be held high on this board.
See AN3156 for DfuSe upload, leave and Read Unprotect, and RM0091 for the
five STM32F042 option pairs. Never download to the option-byte address here.
"""

import argparse
from pathlib import Path
import re
import struct
import sys
import time

from stm32_dfu import DfuTool, FlashError

FLASH_BASE = 0x08000000
FLASH_SIZE = 32 * 1024
OPTION_BASE = 0x1FFFF800
OPTION_BLOCK_SIZE = 16  # ROM upload block size; only the first 10 bytes are implemented.
UID_BASE = 0x1FFFF7AC
APP_VID, APP_PID = 0x0483, 0x5740
DFU_ID = "0483:df11"


class CommitUnverified(FlashError):
    pass


class CommandRejected(FlashError):
    """The application answered a serial command with BELL."""


def validate_firmware(image):
    if not 192 <= len(image) <= FLASH_SIZE or len(image) % 4:
        raise FlashError("Firmware must be a word-aligned raw image of 192..32768 bytes")
    vectors = struct.unpack_from("<48I", image)
    if not 0x20000000 < vectors[0] <= 0x20001800 or vectors[0] % 8:
        raise FlashError("Firmware initial stack is outside/aligned incorrectly for 6 KiB SRAM")
    for index, entry in enumerate(vectors[1:], 1):
        if entry == 0 and index != 1:
            continue
        if not entry & 1 or not FLASH_BASE + 192 <= (entry & ~1) < FLASH_BASE + len(image):
            raise FlashError(f"Firmware vector {index} does not point to Thumb code in the image")


def effective_options(raw):
    # RM0091: a failed complement check loads FF, except RDP (level 1).
    # Keep raw bytes for identity comparisons; only normalize the expected commit.
    normalized = bytearray(raw)
    for index in range(2, 10, 2):
        if raw[index] ^ raw[index + 1] != 0xFF:
            normalized[index:index + 2] = b"\xff\xff"
    return bytes(normalized)


def validate_options(raw, require_rom=True):
    if len(raw) != 10:
        raise FlashError("Expected all five STM32F042 option pairs")
    if raw[:2] != b"\xaa\x55":
        raise FlashError("RDP is not level 0 (AA/55); --recover explicitly authorizes Read Unprotect")
    effective = effective_options(raw)
    if effective[8] != 0xFF:
        raise FlashError("Flash write protection is enabled; refusing to erase or alter it")
    # BOOT_SEL, nBOOT1, nBOOT0, and software watchdog. BOOT0 is physically high.
    if require_rom and effective[2] & 0x99 != 0x99:
        raise FlashError("Persistent ROM boot is not selected; run the app's __dfu command first")
    return raw


def production_options(raw):
    if effective_options(raw)[2] != 0xFF:
        raise FlashError("Production boot requires the supported USER=FF ROM profile")
    updated = bytearray(effective_options(raw))
    updated[2] = (updated[2] & ~0x80) | 0x18
    updated[3] = updated[2] ^ 0xFF
    return bytes(updated)


class SerialPorts:
    def __init__(self):
        try:
            import serial
            from serial.tools import list_ports
        except ImportError as exc:
            raise FlashError("Install pyserial: python3 -m pip install pyserial") from exc
        self.serial = serial
        self.list_ports = list_ports

    def find(self, uid):
        return [p.device for p in self.list_ports.comports()
                if p.vid == APP_VID and p.pid == APP_PID and
                (p.serial_number or "").upper() == uid]

    def uid_for_path(self, path):
        def canonical(value):
            value = str(Path(value).resolve())
            # macOS exposes tty and cu names for the same USB serial interface.
            return value.replace("/dev/tty.", "/dev/cu.", 1) if sys.platform == "darwin" else value
        matches = [p for p in self.list_ports.comports()
                   if canonical(p.device) == canonical(path) and p.vid == APP_VID and p.pid == APP_PID]
        if len(matches) != 1 or not re.fullmatch(r"[0-9A-Fa-f]{24}", matches[0].serial_number or ""):
            raise FlashError("Requested CDC port does not identify one adapter with a full STM32 UID")
        return matches[0].serial_number.upper()

    @staticmethod
    def _drain(port, quiet=0.1, limit=1.0):
        """Discard input until the adapter has been quiet for `quiet` seconds
        (or `limit` seconds pass, if received CAN frames keep arriving)."""
        start = last = time.monotonic()
        while True:
            now = time.monotonic()
            if port.read(256):
                last = now
            elif now - last >= quiet or now - start >= limit:
                return

    def command(self, path, command):
        """Send one SLCAN command and return its reply line (without the CR).

        Leaves an open CAN channel alone: received frames (t/T/r/R lines) that
        arrive alongside the reply are skipped.
        """
        try:
            with self.serial.Serial(path, 115200, timeout=0.05, write_timeout=2) as port:
                # End any partial command the adapter holds, then wait until its
                # reply (and anything else already queued) has been discarded, so
                # a late BELL can't be mistaken for a rejection of `command`.
                port.write(b"\r")
                self._drain(port)
                port.write(command + b"\r")
                port.flush()
                line = bytearray()
                deadline = time.monotonic() + 2
                while time.monotonic() < deadline:
                    byte = port.read(1)
                    if not byte:
                        continue
                    if byte == b"\a":
                        raise CommandRejected(f"Application rejected {command.decode()}")
                    if byte == b"\r":
                        if line[:1] in (b"t", b"T", b"r", b"R"):
                            line.clear()  # a received CAN frame, not our reply
                            continue
                        return bytes(line)
                    line += byte
                    if len(line) > 96:
                        raise FlashError("Oversized serial reply")
        except (self.serial.SerialException, OSError) as exc:
            raise FlashError(f"Serial command {command.decode()} failed: {exc}") from exc
        raise FlashError(f"No serial reply to {command.decode()}")


def wait_dfu(tool, timeout, serial=None, path=None, exclude=(), sleep=time.sleep):
    deadline = time.monotonic() + timeout
    while True:
        devices = [d for d in tool.devices() if d.key not in exclude and
                   (serial is None or d.serial == serial) and (path is None or d.path == path)]
        if len(devices) > 1:
            choices = ", ".join(f"{d.path} ({d.serial})" for d in devices)
            raise FlashError(f"Multiple DFU devices: {choices}; select --dfu-path or --dfu-serial")
        if devices:
            devices[0].validate_layout()
            return devices[0]
        if time.monotonic() >= deadline:
            raise FlashError("Matching ROM DFU device did not enumerate")
        sleep(0.1)


def wait_app(ports, uid, timeout, require_disconnect=False, sleep=time.sleep):
    deadline = time.monotonic() + timeout
    disconnected = not require_disconnect
    while True:
        matches = ports.find(uid)
        if len(matches) > 1:
            raise FlashError("More than one CDC port advertises the target UID")
        if not matches:
            disconnected = True
        elif disconnected:
            return matches[0]
        if time.monotonic() >= deadline:
            raise FlashError("The same-UID application did not reconnect")
        sleep(0.02)


def require_app_identity(ports, uid, path):
    if ports.find(uid) != [path]:
        raise FlashError("Application USB identity changed before serial command")


def app_options(ports, uid, path):
    require_app_identity(ports, uid, path)
    version = ports.command(path, b"v")
    if not re.fullmatch(rb"v[0-9A-F]{4}", version) or int(version[1:], 16) < 3:
        raise FlashError("Application handshake requires firmware v0003 or newer")
    require_app_identity(ports, uid, path)
    reply = ports.command(path, b"__options")
    if not re.fullmatch(rb"OB[0-9A-F]{20}", reply):
        raise FlashError("Application did not return all five option pairs")
    return bytes.fromhex(reply[2:].decode("ascii"))


def update(args, image, tool, ports, sleep=time.sleep):
    validate_firmware(image)  # No host operation is allowed before local validation.
    if args.recover and args.device:
        raise FlashError("--recover requires an adapter already in ROM DFU; omit the CDC device")
    before = set()
    requested_uid = None
    if args.device:
        requested_uid = ports.uid_for_path(args.device)
        before = {d.key for d in tool.devices()}
        try:
            ack = ports.command(args.device, b"__dfu")
        except CommandRejected as exc:
            raise FlashError("The adapter refused to select ROM boot (__dfu); it is still running "
                             "the application. ROM selection needs BOOT0 to read high and "
                             "option bytes the application can rewrite: check the board's "
                             "BOOT0 connection and read __options.") from exc
        if ack != b"":
            raise FlashError("Invalid acknowledgement of __dfu")
    device = wait_dfu(tool, args.timeout, args.dfu_serial, args.dfu_path, before, sleep)
    if args.recover:
        print("Read Unprotect requested: protected flash will be mass-erased and options reset.")
        tool.recover(device)
        # ROM serial can change after unprotect; the physical path must not.
        sleep(0.5)
        device = wait_dfu(tool, args.timeout, path=device.path, sleep=sleep)
    try:
        options = validate_options(tool.options(device))
    except FlashError as exc:
        hint = ("If read protection is enabled, --recover explicitly authorizes destructive "
                "Read Unprotect.")
        if args.device:
            hint += (" Losing power while the adapter rewrote its option bytes for __dfu "
                     "leaves RDP erased, which reads as level 1; --recover then restores "
                     "level 0 by erasing the application and writes this image.")
        raise FlashError(f"ROM preflight failed before application write: {exc}. {hint}") from exc
    uid = tool.uid(device)
    if requested_uid is not None and uid != requested_uid:
        raise FlashError("ROM DFU UID differs from the requested CDC adapter; no firmware write")
    if args.production:
        production_options(options)  # Reject unsupported production profiles before writing.
    if ports.find(uid):
        raise FlashError("Target UID is present as CDC and DFU; refusing ambiguous identity")
    if tool.uid(device) != uid or tool.options(device) != options:
        raise FlashError("Device identity/options changed during preflight")
    if not args.start:
        tool.write_firmware(device, image)
    readback = tool.read(device, FLASH_BASE, len(image))
    if readback != image:
        raise FlashError("Firmware readback mismatch; ROM remains selected, no application start/commit")
    if tool.uid(device) != uid or tool.options(device) != options:
        raise FlashError("Device identity/options changed after verification; not starting application")
    print(f"Verified all {len(image)} firmware bytes for {uid}.")
    tool.leave(device)
    path = wait_app(ports, uid, args.timeout, sleep=sleep)
    running_options = app_options(ports, uid, path)
    if running_options != options:
        raise FlashError("Running app option readback differs from ROM preflight; no production commit")
    validate_options(running_options)
    if not args.production:
        print(f"Firmware verified and running on {path}; persistent ROM boot remains selected.")
        return
    expected = production_options(running_options)
    require_app_identity(ports, uid, path)
    try:
        if ports.command(path, b"__bootflash") != b"":
            raise FlashError("Invalid acknowledgement of __bootflash")
        path = wait_app(ports, uid, args.timeout, require_disconnect=True, sleep=sleep)
        actual = app_options(ports, uid, path)
        if actual != expected:
            raise FlashError("Production option readback mismatch")
        validate_options(actual, require_rom=False)
    except (FlashError, OSError, KeyboardInterrupt) as exc:
        raise CommitUnverified("Production commit is unverified/indeterminate: " + str(exc) +
                               ". Reconnect and query __options; if ROM DFU appears, repeat the verified update. "
                               "Use --recover only when accepting Read Unprotect data loss.") from exc
    print(f"Production boot committed and all five option pairs verified on {path}.")


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("device", nargs="?", default="", help="running CDC port (empty if already in ROM DFU)")
    parser.add_argument("firmware", nargs="?", default=str(Path(__file__).resolve().parents[1] / "firmware/app.bin"))
    parser.add_argument("--production", "--boot-app", action="store_true", help="commit flash boot after verified app reconnect")
    parser.add_argument("--start", action="store_true", help="verify stored image against firmware, then start without rewriting")
    parser.add_argument("--recover", action="store_true", help="authorize destructive ROM Read Unprotect before updating")
    parser.add_argument("--dfu-serial", help="exact ROM DFU serial to select")
    parser.add_argument("--dfu-path", help="exact physical USB bus-port path to select")
    parser.add_argument("--timeout", type=float, default=15, help="enumeration timeout in seconds (default: 15)")
    args = parser.parse_args(argv)
    if not 0 < args.timeout <= 120:
        parser.error("--timeout must be in (0, 120] seconds")
    if args.recover and args.start:
        parser.error("--recover erases the stored image and cannot be combined with --start")
    if args.recover and args.device:
        parser.error("--recover requires an adapter already in ROM DFU; omit the CDC device")
    return args


def main(argv=None):
    args = parse_args(argv)
    try:
        image = Path(args.firmware).read_bytes()
        validate_firmware(image)
        with DfuTool() as tool:
            update(args, image, tool, SerialPorts())
        return 0
    except CommitUnverified as exc:
        print(exc, file=sys.stderr)
        return 3
    except (FlashError, OSError) as exc:
        print(exc, file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("Interrupted; production boot was not committed. Repeat the verified update from ROM DFU.", file=sys.stderr)
        return 130


if __name__ == "__main__":
    sys.exit(main())
