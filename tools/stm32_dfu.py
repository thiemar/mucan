"""Bounded STM32F042 ROM DFU operations over the native libusb transport.

Protocol: ST AN3156, sections 2--5. Each transfer resets the address pointer
and uses block 2, including the final short block: the ROM computes addresses
using the current request length, not a fixed transfer size. Option bytes are
read-only here. Only the explicit recover() operation sends Read Unprotect.
"""

from contextlib import contextmanager
from dataclasses import dataclass, field
import re
import struct
import time

FLASH_BASE = 0x08000000
FLASH_SIZE = 32 * 1024
PAGE_SIZE = 1024
OPTION_BASE = 0x1FFFF800
UID_BASE = 0x1FFFF7AC
DFU_IDLE, DNLOAD_SYNC, DNBUSY, DNLOAD_IDLE = 2, 3, 4, 5
MANIFEST, UPLOAD_IDLE, DFU_ERROR = 7, 9, 10
DNLOAD, UPLOAD, GETSTATUS, CLRSTATUS, ABORT = 1, 2, 3, 4, 6


class FlashError(Exception):
    pass


@dataclass
class DfuDevice:
    path: str
    serial: str
    alts: dict = field(default_factory=dict)
    configuration: int = 1
    interface: int = 0
    transfer_size: int = 2048
    _usb: object = field(default=None, repr=False, compare=False)

    @property
    def key(self):
        return self.path, self.serial

    def validate_layout(self):
        flash = re.sub(r"\s+", "", self.alts.get(0, "")).lower()
        options = re.sub(r"\s+", "", self.alts.get(1, "")).lower()
        if not re.fullmatch(r"@internalflash/0x08000000/0*32\*0*1kg", flash):
            raise FlashError("DFU memory layout is not the STM32F042 32 KiB flash layout")
        if not re.fullmatch(r"@optionbytes/0x1ffff800/0*1\*0*16e", options):
            raise FlashError("DFU memory layout does not expose the expected 16-byte option block")


def _transfer_size(interfaces):
    sizes = set()
    for interface in interfaces:
        for extra in (interface.extra, interface.configuration_extra):
            offset = 0
            while offset < len(extra):
                length = extra[offset]
                if length < 2 or offset + length > len(extra):
                    raise FlashError("Malformed USB extra descriptor")
                descriptor = extra[offset:offset + length]
                if descriptor[1] == 0x21:
                    if length != 9 or descriptor[2] & 3 != 3:
                        raise FlashError("ROM DFU descriptor does not permit upload and download")
                    size, version = struct.unpack_from("<HH", descriptor, 5)
                    if version != 0x011A or not 16 <= size <= 2048:
                        raise FlashError("Unsupported STM32 DfuSe transfer descriptor")
                    sizes.add(size)
                offset += length
    if len(sizes) != 1:
        raise FlashError("Missing or inconsistent ROM DFU functional descriptor")
    # Keep firmware writes word-aligned even if a descriptor advertises an odd size.
    return sizes.pop() & ~3


class _Session:
    def __init__(self, tool, device, handle):
        self.tool, self.device, self.handle = tool, device, handle
        self.deadline = tool.monotonic() + 45

    def transfer(self, request, value=0, data=b"", incoming=False):
        remaining = self.deadline - self.tool.monotonic()
        if remaining <= 0:
            raise FlashError("ROM DFU operation timed out")
        result = self.handle.ctrl_transfer(0xA1 if incoming else 0x21,
                                          request, value, self.device.interface, data,
                                          timeout_ms=max(1, min(2000, int(remaining * 1000))))
        if not incoming and result != len(data):
            raise FlashError("Short DFU control download")
        return bytes(result) if incoming else result

    def status(self, allow_error=False):
        raw = self.transfer(GETSTATUS, data=6, incoming=True)
        if len(raw) != 6:
            raise FlashError("Malformed DFU GETSTATUS reply")
        status, delay, state = raw[0], int.from_bytes(raw[1:4], "little") / 1000, raw[4]
        if not allow_error and (status or state == DFU_ERROR):
            raise FlashError(f"ROM DFU error: status 0x{status:02x}, state {state}")
        return status, delay, state

    def delay(self, seconds):
        # Zero-timeout busy replies still yield and cannot create a tight spin.
        seconds = max(seconds, 0.001)
        if seconds >= self.deadline - self.tool.monotonic():
            raise FlashError("ROM DFU poll timeout exceeds operation deadline")
        self.tool.sleep(seconds)

    def prepare(self):
        status, delay, state = self.status(allow_error=True)
        if state == DFU_ERROR:
            # Clear an error left by a previous session, never a current transfer.
            self.transfer(CLRSTATUS)
            status, delay, state = self.status()
        elif status:
            raise FlashError(f"Invalid initial DFU status 0x{status:02x}")
        while state in (DNLOAD_SYNC, DNBUSY):
            self.delay(delay)
            _, delay, state = self.status()
        if state in (DNLOAD_IDLE, UPLOAD_IDLE):
            self.abort()
        elif state != DFU_IDLE:
            raise FlashError(f"Cannot begin a ROM DFU operation in state {state}")

    def abort(self):
        self.transfer(ABORT)
        _, _, state = self.status()
        if state != DFU_IDLE:
            raise FlashError(f"DFU ABORT did not reach idle (state {state})")

    def download(self, block, data):
        self.transfer(DNLOAD, block, data)
        _, delay, state = self.status()
        if state != DNBUSY:
            raise FlashError(f"DFU download did not start execution (state {state})")
        while state == DNBUSY:
            self.delay(delay)
            _, delay, state = self.status()
        if state != DNLOAD_IDLE:
            raise FlashError(f"DFU download did not complete (state {state})")

    def address(self, address):
        self.download(0, b"\x21" + struct.pack("<I", address))

    def reset_command(self, data, expected_state):
        self.transfer(DNLOAD, 0, data)
        try:
            _, delay, state = self.status()
        except self.tool.usb_error as exc:
            if exc.code not in (-4, -7, -9):
                raise
            # A lost response at this exact boundary is indeterminate. The caller
            # must verify same-device enumeration and options before reporting success.
            return
        if state != expected_state:
            raise FlashError(f"ROM did not accept reset/leave command (state {state})")
        if delay:
            self.delay(delay)


class DfuTool:
    def __init__(self, transport=None, sleep=time.sleep, monotonic=time.monotonic):
        if transport is None:
            # Importing the workflow/tests never loads a native library or opens USB.
            from libusb_transport import LibusbTransport, UsbError
            try:
                transport = LibusbTransport()
            except (UsbError, OSError) as exc:
                raise FlashError(f"Cannot initialize libusb: {exc}") from exc
            self.usb_error = UsbError
        else:
            self.usb_error = transport.error_type
        self.transport, self.sleep, self.monotonic = transport, sleep, monotonic

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    def close(self):
        self.transport.close()

    def devices(self):
        devices = []
        try:
            for usb in self.transport.devices(0x0483, 0xDF11):
                interfaces = [i for i in usb.interfaces if
                              (i.interface_class, i.subclass, i.protocol) == (0xFE, 1, 2)]
                if not interfaces:
                    continue
                groups = {(i.configuration_value, i.number) for i in interfaces}
                if len(groups) != 1:
                    raise FlashError("Ambiguous ROM DFU interface/configuration")
                if not usb.port_numbers or usb.bus is None:
                    raise FlashError("libusb cannot identify a stable physical USB path")
                with usb.open() as handle:
                    serial = handle.string(usb.serial_index) if usb.serial_index else ""
                    alts = {i.alternate: handle.string(i.name_index) for i in interfaces}
                configuration, interface = groups.pop()
                devices.append(DfuDevice(usb.path, serial, alts, configuration, interface,
                                         _transfer_size(interfaces), usb))
        except self.usb_error as exc:
            raise FlashError(f"Cannot enumerate ROM DFU devices: {exc}") from exc
        return devices

    @contextmanager
    def session(self, device, alt=0):
        device.validate_layout()
        matches = [d for d in self.devices() if d.key == device.key]
        if len(matches) != 1:
            raise FlashError("The selected ROM DFU device disappeared or became ambiguous")
        current = matches[0]
        current.validate_layout()
        # The native transport retains/caches each libusb device reference for
        # this connection. A replacement on the same port may have the same
        # non-unique ROM serial, but cannot share the retained reference.
        if current._usb is not device._usb:
            raise FlashError("The selected physical USB connection was replaced")
        if (current.alts, current.configuration, current.interface, current.transfer_size) != (
                device.alts, device.configuration, device.interface, device.transfer_size):
            raise FlashError("ROM DFU descriptors changed after selection")
        try:
            with current._usb.open() as handle:
                handle.set_configuration(current.configuration)
                handle.claim_interface(current.interface)
                handle.set_interface_altsetting(current.interface, alt)
                session = _Session(self, current, handle)
                session.prepare()
                yield session
        except self.usb_error as exc:
            raise FlashError(f"ROM DFU USB transfer failed: {exc}") from exc

    def read(self, device, address, size, alt=0, force=False):
        del force  # UID is explicitly whitelisted; no unrestricted memory access.
        flash = alt == 0 and FLASH_BASE <= address and address + size <= FLASH_BASE + FLASH_SIZE
        options = alt == 1 and (address, size) == (OPTION_BASE, 16)
        uid = alt == 0 and (address, size) == (UID_BASE, 12)
        if size < 2 or size % 2 or address % 2 or not (flash or options or uid):
            raise FlashError("Unsupported ROM read range or alignment")
        result = bytearray()
        with self.session(device, alt) as session:
            while len(result) < size:
                count = min(session.device.transfer_size, size - len(result))
                session.address(address + len(result))
                session.abort()  # DNLOAD-IDLE -> IDLE, retaining the address pointer.
                data = session.transfer(UPLOAD, 2, count, incoming=True)
                _, _, state = session.status()
                if len(data) != count or state not in (DFU_IDLE, UPLOAD_IDLE):
                    raise FlashError(f"Short/invalid DFU upload: expected {count}, received {len(data)} bytes, state {state}")
                result.extend(data)
                session.abort()  # A subsequent SET_ADDRESS requires download/idle state.
        return bytes(result)

    def options(self, device):
        return self.read(device, OPTION_BASE, 16, alt=1)[:10]

    def uid(self, device):
        raw = self.read(device, UID_BASE, 12)
        if raw in (bytes(12), b"\xff" * 12):
            raise FlashError("Invalid STM32 unique ID; refusing ambiguous application matching")
        return raw.hex().upper()

    def write_firmware(self, device, image):
        if not image or len(image) > FLASH_SIZE or len(image) % 4:
            raise FlashError("Application download must be word-aligned and fit 32 KiB flash")
        with self.session(device) as session:
            for offset in range(0, len(image), PAGE_SIZE):
                session.download(0, b"\x41" + struct.pack("<I", FLASH_BASE + offset))
            for offset in range(0, len(image), session.device.transfer_size):
                session.address(FLASH_BASE + offset)
                session.download(2, image[offset:offset + session.device.transfer_size])

    def leave(self, device):
        with self.session(device) as session:
            session.address(FLASH_BASE)
            session.reset_command(b"", MANIFEST)

    def recover(self, device):
        with self.session(device) as session:
            session.reset_command(b"\x92", DNBUSY)
