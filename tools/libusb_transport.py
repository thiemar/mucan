"""Small synchronous libusb-1.0 binding using only Python's standard library.

Descriptors are copied out of libusb-owned buffers. A transport owns device
references and open handles until close(); use it as a context manager. No
library is loaded and no USB operation happens merely by importing this file.
ABI/API: https://libusb.sourceforge.io/api-1.0/ (libusb.h).
"""

import ctypes as C
from ctypes.util import find_library
from dataclasses import dataclass, field
import os
import sys

LIBUSB_ERROR_NO_DEVICE = NO_DEVICE = -4
LIBUSB_ERROR_TIMEOUT = TIMEOUT = -7
LIBUSB_ERROR_NOT_SUPPORTED = -12
LIBUSB_ERROR_OTHER = -99


class UsbError(Exception):
    def __init__(self, message, code=LIBUSB_ERROR_OTHER):
        super().__init__(message)
        self.code = code

    @property
    def disconnected(self):
        return self.code == LIBUSB_ERROR_NO_DEVICE


_U8P = C.POINTER(C.c_uint8)
_DeviceP = C.c_void_p
_DeviceList = C.POINTER(_DeviceP)


class _DeviceDescriptor(C.Structure):
    _fields_ = [("bLength", C.c_uint8), ("bDescriptorType", C.c_uint8),
                ("bcdUSB", C.c_uint16), ("bDeviceClass", C.c_uint8),
                ("bDeviceSubClass", C.c_uint8), ("bDeviceProtocol", C.c_uint8),
                ("bMaxPacketSize0", C.c_uint8), ("idVendor", C.c_uint16),
                ("idProduct", C.c_uint16), ("bcdDevice", C.c_uint16),
                ("iManufacturer", C.c_uint8), ("iProduct", C.c_uint8),
                ("iSerialNumber", C.c_uint8), ("bNumConfigurations", C.c_uint8)]


class _EndpointDescriptor(C.Structure):
    _fields_ = [("bLength", C.c_uint8), ("bDescriptorType", C.c_uint8),
                ("bEndpointAddress", C.c_uint8), ("bmAttributes", C.c_uint8),
                ("wMaxPacketSize", C.c_uint16), ("bInterval", C.c_uint8),
                ("bRefresh", C.c_uint8), ("bSynchAddress", C.c_uint8),
                ("extra", _U8P), ("extra_length", C.c_int)]


class _InterfaceDescriptor(C.Structure):
    _fields_ = [("bLength", C.c_uint8), ("bDescriptorType", C.c_uint8),
                ("bInterfaceNumber", C.c_uint8), ("bAlternateSetting", C.c_uint8),
                ("bNumEndpoints", C.c_uint8), ("bInterfaceClass", C.c_uint8),
                ("bInterfaceSubClass", C.c_uint8), ("bInterfaceProtocol", C.c_uint8),
                ("iInterface", C.c_uint8),
                ("endpoint", C.POINTER(_EndpointDescriptor)),
                ("extra", _U8P), ("extra_length", C.c_int)]


class _Interface(C.Structure):
    _fields_ = [("altsetting", C.POINTER(_InterfaceDescriptor)),
                ("num_altsetting", C.c_int)]


class _ConfigDescriptor(C.Structure):
    _fields_ = [("bLength", C.c_uint8), ("bDescriptorType", C.c_uint8),
                ("wTotalLength", C.c_uint16), ("bNumInterfaces", C.c_uint8),
                ("bConfigurationValue", C.c_uint8), ("iConfiguration", C.c_uint8),
                ("bmAttributes", C.c_uint8), ("MaxPower", C.c_uint8),
                ("interface", C.POINTER(_Interface)),
                ("extra", _U8P), ("extra_length", C.c_int)]


def _load_library():
    override = os.environ.get("LIBUSB_LIBRARY")
    if override:
        candidates = [override]
    else:
        candidates = [find_library("usb-1.0")]
        if sys.platform == "darwin":
            candidates += ["/opt/homebrew/lib/libusb-1.0.dylib",
                           "/usr/local/lib/libusb-1.0.dylib", "libusb-1.0.dylib"]
        elif sys.platform == "win32":
            candidates += ["libusb-1.0.dll"]
        else:
            candidates += ["libusb-1.0.so.0", "libusb-1.0.so"]
    # libusb uses WINAPI/stdcall on Windows, including 32-bit Python.
    loader = C.WinDLL if sys.platform == "win32" else C.CDLL
    for name in dict.fromkeys(filter(None, candidates)):
        try:
            return loader(name)
        except OSError:
            pass
    raise UsbError("Cannot load libusb-1.0. Install the native library "
                   "(macOS: brew install libusb), or set LIBUSB_LIBRARY to its file.")


def _bind(lib):
    def function(name, result, *args, optional=False):
        fn = getattr(lib, name, None)
        if fn is None:
            if optional:
                return
            raise UsbError(f"libusb library is missing {name}")
        fn.restype, fn.argtypes = result, list(args)

    ptr, integer = C.c_void_p, C.c_int
    function("libusb_init", integer, C.POINTER(ptr))
    function("libusb_exit", None, ptr)
    function("libusb_get_device_list", C.c_ssize_t, ptr, C.POINTER(_DeviceList))
    function("libusb_free_device_list", None, _DeviceList, integer)
    function("libusb_ref_device", ptr, ptr)
    function("libusb_unref_device", None, ptr)
    function("libusb_get_device_descriptor", integer, ptr, C.POINTER(_DeviceDescriptor))
    function("libusb_get_config_descriptor", integer, ptr, C.c_uint8,
             C.POINTER(C.POINTER(_ConfigDescriptor)))
    function("libusb_free_config_descriptor", None, C.POINTER(_ConfigDescriptor))
    function("libusb_get_bus_number", C.c_uint8, ptr)
    function("libusb_get_device_address", C.c_uint8, ptr)
    function("libusb_get_port_numbers", integer, ptr, _U8P, integer, optional=True)
    function("libusb_get_port_number", C.c_uint8, ptr, optional=True)
    function("libusb_get_parent", ptr, ptr, optional=True)
    function("libusb_open", integer, ptr, C.POINTER(ptr))
    function("libusb_close", None, ptr)
    function("libusb_get_string_descriptor_ascii", integer, ptr, C.c_uint8, _U8P, integer)
    function("libusb_get_configuration", integer, ptr, C.POINTER(integer))
    function("libusb_set_configuration", integer, ptr, integer)
    function("libusb_claim_interface", integer, ptr, integer)
    function("libusb_release_interface", integer, ptr, integer)
    function("libusb_set_interface_alt_setting", integer, ptr, integer, integer)
    function("libusb_set_auto_detach_kernel_driver", integer, ptr, integer, optional=True)
    function("libusb_kernel_driver_active", integer, ptr, integer, optional=True)
    function("libusb_detach_kernel_driver", integer, ptr, integer, optional=True)
    function("libusb_attach_kernel_driver", integer, ptr, integer, optional=True)
    function("libusb_control_transfer", integer, ptr, C.c_uint8, C.c_uint8,
             C.c_uint16, C.c_uint16, _U8P, C.c_uint16, C.c_uint)
    function("libusb_error_name", C.c_char_p, integer, optional=True)


def _extra(pointer, length):
    if length < 0 or length > 65535 or (length and not pointer):
        raise UsbError("Invalid extra USB descriptor length")
    return C.string_at(pointer, length) if length else b""


@dataclass(frozen=True)
class EndpointInfo:
    address: int
    attributes: int
    max_packet_size: int
    interval: int
    extra: bytes


@dataclass(frozen=True)
class InterfaceInfo:
    configuration_value: int
    number: int
    alternate: int
    name_index: int
    interface_class: int
    subclass: int
    protocol: int
    extra: bytes
    configuration_extra: bytes
    endpoints: tuple = ()


@dataclass(frozen=True)
class UsbDevice:
    bus: int
    address: int
    port_numbers: tuple
    serial_index: int
    interfaces: tuple
    _transport: object = field(repr=False, compare=False)
    _pointer: int = field(repr=False, compare=False)

    @property
    def path(self):
        return f"{self.bus}-" + ".".join(str(port) for port in self.port_numbers)

    def open(self):
        return DeviceHandle(self)


class LibusbTransport:
    error_type = UsbError

    def __init__(self, library=None):
        self._lib = _load_library() if library is None else library
        _bind(self._lib)
        self._context = C.c_void_p()
        self._devices = {}
        self._handles = set()
        self._closed = True
        self._check(self._lib.libusb_init(C.byref(self._context)), "Initialize libusb")
        self._closed = False

    def _check(self, code, operation):
        if code < 0:
            name_fn = getattr(self._lib, "libusb_error_name", None)
            name = name_fn(code) if name_fn else None
            detail = name.decode("ascii", "replace") if name else str(code)
            raise UsbError(f"{operation}: {detail}", code)
        return code

    def _ensure_open(self):
        if self._closed:
            raise UsbError("libusb transport is closed")

    def _ports(self, pointer):
        fn = getattr(self._lib, "libusb_get_port_numbers", None)
        if fn:
            ports = (C.c_uint8 * 8)()
            count = fn(pointer, ports, len(ports))
            if count > 0:
                if count > len(ports):
                    raise UsbError("Invalid USB port depth")
                return tuple(ports[:count])
            if count not in (0, LIBUSB_ERROR_NOT_SUPPORTED):
                self._check(count, "Read USB port path")
        port_fn = getattr(self._lib, "libusb_get_port_number", None)
        parent_fn = getattr(self._lib, "libusb_get_parent", None)
        if port_fn and parent_fn:
            ports, seen = [], set()
            while pointer and pointer not in seen and len(ports) < 8:
                seen.add(pointer)
                port = port_fn(pointer)
                if not port:
                    return tuple(reversed(ports)) if ports else self._no_ports()
                ports.append(port)
                pointer = parent_fn(pointer)
            if not pointer and ports:
                return tuple(reversed(ports))
        return self._no_ports()

    @staticmethod
    def _no_ports():
        # A USB address changes after reset; never present it as a stable path.
        raise UsbError("USB port topology is unavailable; cannot identify the "
                       "same adapter across resets", LIBUSB_ERROR_NOT_SUPPORTED)

    def _interfaces(self, pointer, configurations):
        result = []
        for index in range(configurations):
            config = C.POINTER(_ConfigDescriptor)()
            self._check(self._lib.libusb_get_config_descriptor(pointer, index, C.byref(config)),
                        "Read USB configuration descriptor")
            if not config:
                raise UsbError("libusb returned an empty configuration descriptor")
            try:
                value = config.contents
                extra = _extra(value.extra, value.extra_length)
                if value.bNumInterfaces and not value.interface:
                    raise UsbError("USB configuration has no interface descriptors")
                for i in range(value.bNumInterfaces):
                    interface = value.interface[i]
                    if not 0 <= interface.num_altsetting <= 256 or (
                            interface.num_altsetting and not interface.altsetting):
                        raise UsbError("Invalid USB alternate-setting descriptors")
                    for a in range(interface.num_altsetting):
                        alt = interface.altsetting[a]
                        if alt.bNumEndpoints and not alt.endpoint:
                            raise UsbError("USB interface has no endpoint descriptors")
                        endpoints = tuple(EndpointInfo(
                            ep.bEndpointAddress, ep.bmAttributes, ep.wMaxPacketSize,
                            ep.bInterval, _extra(ep.extra, ep.extra_length))
                            for ep in (alt.endpoint[e] for e in range(alt.bNumEndpoints)))
                        result.append(InterfaceInfo(
                            value.bConfigurationValue, alt.bInterfaceNumber,
                            alt.bAlternateSetting, alt.iInterface, alt.bInterfaceClass,
                            alt.bInterfaceSubClass, alt.bInterfaceProtocol,
                            _extra(alt.extra, alt.extra_length), extra, endpoints))
            finally:
                self._lib.libusb_free_config_descriptor(config)
        return tuple(result)

    def devices(self, vid=0x0483, pid=0xDF11):
        self._ensure_open()
        listing = _DeviceList()
        count = self._check(self._lib.libusb_get_device_list(self._context, C.byref(listing)),
                            "Enumerate USB devices")
        result = []
        try:
            for i in range(count):
                pointer = listing[i]
                descriptor = _DeviceDescriptor()
                try:
                    self._check(self._lib.libusb_get_device_descriptor(pointer, C.byref(descriptor)),
                                "Read USB device descriptor")
                    if descriptor.idVendor != vid or descriptor.idProduct != pid:
                        continue
                    if pointer not in self._devices:
                        device = UsbDevice(
                            self._lib.libusb_get_bus_number(pointer),
                            self._lib.libusb_get_device_address(pointer), self._ports(pointer),
                            descriptor.iSerialNumber,
                            self._interfaces(pointer, descriptor.bNumConfigurations), self, pointer)
                        self._lib.libusb_ref_device(pointer)
                        self._devices[pointer] = device
                    result.append(self._devices[pointer])
                except UsbError as exc:
                    if exc.code != LIBUSB_ERROR_NO_DEVICE:
                        raise
                    # A device can vanish between enumeration and descriptors.
        finally:
            self._lib.libusb_free_device_list(listing, 1)
        return result

    def close(self):
        if self._closed:
            return
        for handle in tuple(self._handles):
            handle.close()
        for pointer in self._devices:
            self._lib.libusb_unref_device(pointer)
        self._devices.clear()
        self._lib.libusb_exit(self._context)
        self._closed = True

    def __enter__(self):
        self._ensure_open()
        return self

    def __exit__(self, *_):
        self.close()


class DeviceHandle:
    def __init__(self, device):
        self._transport = device._transport
        self._transport._ensure_open()
        self._lib = self._transport._lib
        self._pointer = C.c_void_p()
        self._claimed = []
        self._detached = set()
        self._closed = True
        self._transport._check(self._lib.libusb_open(device._pointer, C.byref(self._pointer)),
                               f"Open USB device {device.path}")
        self._closed = False
        self._transport._handles.add(self)
        self._auto_detach = False
        fn = getattr(self._lib, "libusb_set_auto_detach_kernel_driver", None)
        if fn:
            status = fn(self._pointer, 1)
            if status == 0:
                self._auto_detach = True
            elif status != LIBUSB_ERROR_NOT_SUPPORTED:
                self.close()
                self._transport._check(status, "Enable USB kernel-driver detach")

    def _call(self, name, *args):
        if self._closed:
            raise UsbError("USB device handle is closed")
        return self._transport._check(getattr(self._lib, name)(self._pointer, *args), name)

    def string(self, index):
        if not 0 <= index <= 255:
            raise ValueError("USB string index must fit in one byte")
        if not index:
            return ""
        data = (C.c_uint8 * 255)()
        size = self._call("libusb_get_string_descriptor_ascii", index, data, len(data))
        if size > len(data):
            raise UsbError("Invalid USB string length")
        return bytes(data[:size]).decode("ascii", "replace")

    def get_configuration(self):
        value = C.c_int()
        self._call("libusb_get_configuration", C.byref(value))
        return value.value

    def set_configuration(self, value):
        if not 0 <= value <= 255:
            raise ValueError("USB configuration must fit in one byte")
        if self.get_configuration() != value:
            self._call("libusb_set_configuration", value)

    def claim_interface(self, number):
        if self._closed:
            raise UsbError("USB device handle is closed")
        if not 0 <= number <= 255:
            raise ValueError("USB interface must fit in one byte")
        if number in self._claimed:
            return
        if not self._auto_detach:
            active = getattr(self._lib, "libusb_kernel_driver_active", None)
            if active:
                status = active(self._pointer, number)
                if status == 1:
                    detach = getattr(self._lib, "libusb_detach_kernel_driver", None)
                    attach = getattr(self._lib, "libusb_attach_kernel_driver", None)
                    if detach is None or attach is None:
                        raise UsbError("Cannot detach and restore the USB kernel driver",
                                       LIBUSB_ERROR_NOT_SUPPORTED)
                    self._call("libusb_detach_kernel_driver", number)
                    self._detached.add(number)
                elif status != LIBUSB_ERROR_NOT_SUPPORTED:
                    self._transport._check(status, "Check USB kernel driver")
        try:
            self._call("libusb_claim_interface", number)
        except Exception:
            if number in self._detached:
                self._reattach(number)
            raise
        self._claimed.append(number)

    def set_interface_altsetting(self, number, alternate):
        if number not in self._claimed:
            raise UsbError("Claim the USB interface before selecting its alternate setting")
        if not 0 <= alternate <= 255:
            raise ValueError("USB alternate setting must fit in one byte")
        self._call("libusb_set_interface_alt_setting", number, alternate)

    def ctrl_transfer(self, request_type, request, value=0, index=0,
                      data_or_length=b"", timeout_ms=5000):
        for label, number, maximum in (("request type", request_type, 255),
                                        ("request", request, 255),
                                        ("value", value, 65535), ("index", index, 65535)):
            if not isinstance(number, int) or not 0 <= number <= maximum:
                raise ValueError(f"USB {label} is outside its field range")
        if not isinstance(timeout_ms, int) or not 1 <= timeout_ms <= 60000:
            raise ValueError("USB timeout must be 1..60000 milliseconds")
        incoming = bool(request_type & 0x80)
        if incoming:
            if not isinstance(data_or_length, int) or not 0 <= data_or_length <= 65535:
                raise ValueError("USB IN request needs a length of 0..65535")
            length = data_or_length
            data = (C.c_uint8 * max(1, length))()
        else:
            if not isinstance(data_or_length, (bytes, bytearray, memoryview)):
                raise ValueError("USB OUT request needs bytes")
            payload = bytes(data_or_length)
            length = len(payload)
            if length > 65535:
                raise ValueError("USB OUT payload exceeds 65535 bytes")
            data = (C.c_uint8 * max(1, length))()
            if length:
                C.memmove(data, payload, length)
        transferred = self._call("libusb_control_transfer", request_type, request,
                                 value, index, data, length, timeout_ms)
        if transferred > length:
            raise UsbError("libusb reported an oversized control transfer")
        return bytes(data[:transferred]) if incoming else transferred

    def _reattach(self, number):
        fn = getattr(self._lib, "libusb_attach_kernel_driver", None)
        if fn:
            fn(self._pointer, number)  # Best effort: reset/disconnect may have removed it.
        self._detached.discard(number)

    def close(self):
        if self._closed:
            return
        for number in reversed(self._claimed):
            self._lib.libusb_release_interface(self._pointer, number)
        self._claimed.clear()
        for number in tuple(self._detached):
            self._reattach(number)
        self._lib.libusb_close(self._pointer)
        self._closed = True
        self._transport._handles.discard(self)

    def __enter__(self):
        if self._closed:
            raise UsbError("USB device handle is closed")
        return self

    def __exit__(self, *_):
        self.close()
