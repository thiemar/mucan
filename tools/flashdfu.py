#!/usr/bin/env python3
"""Select ROM boot, then stage, read back, and start µCAN firmware.

Requires native libusb and pyserial. BOOT0 is tied low on these boards, so once
flash holds an image only the option bits (BOOT_SEL=0, nBOOT0=0: USER=0x77)
can bring the adapter back to ROM DFU. That selection is written and verified
through ROM DFU before any flash write. See AN3156 for DfuSe upload, option
write, leave and Read Unprotect, and RM0091 for the five STM32F042 option pairs.
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

# USER option byte (RM0091 4.1): boot source selection bits
USER_BOOT_SEL = 0x80  # 1: BOOT0 pin decides; 0: the nBOOT0 bit decides
USER_NBOOT1 = 0x10
USER_NBOOT0 = 0x08
ROM_USER = 0x77  # BOOT_SEL=0, nBOOT0=0: ROM bootloader whatever BOOT0 does
APP_USER = 0x7F  # BOOT_SEL=0, nBOOT0=1: application flash


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


def validate_options(raw):
    if len(raw) != 10:
        raise FlashError("Expected all five STM32F042 option pairs")
    if raw[:2] != b"\xaa\x55":
        raise FlashError("RDP is not level 0 (AA/55); --recover explicitly authorizes Read Unprotect")
    effective = effective_options(raw)
    if effective[8] != 0xFF:
        raise FlashError("Flash write protection is enabled; refusing to erase or alter it")
    return raw


def rom_selected(raw):
    """True if reset starts the ROM bootloader through the option bits alone."""
    user = effective_options(raw)[2]
    return user & (USER_BOOT_SEL | USER_NBOOT1 | USER_NBOOT0) == USER_NBOOT1


def rom_option_block(block):
    """The 16-byte option block with USER selecting ROM through the option
    bits. Other USER bits are kept; invalid non-RDP pairs are written as their
    effective FF/00 value, and bytes past the five implemented pairs are kept."""
    updated = bytearray(block)
    for index in range(2, 10, 2):
        if updated[index] ^ updated[index + 1] != 0xFF:
            updated[index:index + 2] = b"\xff\x00"
    user = (updated[2] & ~(USER_BOOT_SEL | USER_NBOOT0) | USER_NBOOT1) & 0xFF
    updated[2], updated[3] = user, user ^ 0xFF
    return bytes(updated)


def production_options(raw):
    if effective_options(raw)[2] != ROM_USER:
        raise FlashError(f"Production boot requires the supported USER=0x{ROM_USER:02X} ROM profile")
    updated = bytearray(effective_options(raw))
    updated[2] = APP_USER
    updated[3] = APP_USER ^ 0xFF
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


def select_rom(tool, device, block, uid, timeout, sleep=time.sleep):
    """Write USER=0x77 through ROM DFU and wait for the adapter to come back
    on the same USB path with the new options. Returns (device, block)."""
    target = rom_option_block(block)
    print("Selecting persistent ROM boot (USER=0x77) before writing flash.")
    tool.write_options(device, target)
    deadline = time.monotonic() + timeout
    problem = "it did not reappear"
    while time.monotonic() < deadline:
        sleep(0.2)  # the ROM resets to load the new options
        try:
            current = wait_dfu(tool, max(0.1, deadline - time.monotonic()), path=device.path,
                               sleep=sleep)
            readback = tool.option_block(current)
            if readback[:10] == target[:10] and tool.uid(current) == uid:
                return current, readback
            problem = "its option bytes read back as " + readback[:10].hex(" ").upper()
        except FlashError as exc:
            problem = str(exc)
    raise FlashError(f"ROM boot selection did not verify ({problem}); no flash was written")


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
    # v0004 is the first firmware whose __dfu selects ROM through the option
    # bits; with BOOT0 tied low an older image could never be replaced again.
    if not re.fullmatch(rb"v[0-9A-F]{4}", version) or int(version[1:], 16) < 4:
        raise FlashError("Application handshake requires firmware v0004 or newer")
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
                             "the application. Its option bytes could not be rewritten "
                             "(read protection level 2 or inconsistent protection prevent it); "
                             "read __options for details.") from exc
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
        block = tool.option_block(device)
        options = validate_options(block[:10])
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
        # Reject unsupported production profiles before changing anything.
        production_options(rom_option_block(block)[:10])
    if ports.find(uid):
        raise FlashError("Target UID is present as CDC and DFU; refusing ambiguous identity")
    if not rom_selected(options):
        # BOOT0 is tied low: once flash holds an image, a reset only reaches
        # ROM DFU if the option bits select it. Never write flash without it.
        device, block = select_rom(tool, device, block, uid, args.timeout, sleep)
        options = validate_options(block[:10])
    if not rom_selected(options):
        raise FlashError("Persistent ROM boot is not selected; no flash was written")
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
    if not rom_selected(running_options):
        raise FlashError("Running app reports ROM boot is not selected; no production commit")
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
        validate_options(actual)
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
