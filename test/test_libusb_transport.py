#!/usr/bin/env python3
"""Exercise the native ABI boundary with a fake C API; never enumerate hardware."""
import ctypes as C
from dataclasses import FrozenInstanceError
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import libusb_transport as usb


class Function:
    def __init__(self, owner, name, implementation):
        self.owner, self.name, self.implementation = owner, name, implementation
        self.restype = self.argtypes = None

    def __call__(self, *args):
        self.owner.calls.append((self.name, args))
        if self.name in self.owner.fail:
            return self.owner.fail[self.name]
        return self.implementation(*args)


def output(pointer, ctype, value):
    C.cast(pointer, C.POINTER(ctype))[0] = value


class FakeLib:
    def __init__(self, auto=False):
        self.calls, self.fail = [], {}
        self.auto = auto
        self.active = 1
        self.configuration = 1
        self.transfer_reply = b"\x01\x02\x03"
        self.transfer_count = None
        self.out_payload = None
        self.extra_data = b"\x09\x21\x0b\xff\x00\x00\x08\x1a\x01"
        self.extra = (C.c_uint8 * len(self.extra_data)).from_buffer_copy(self.extra_data)
        self.config_extra = (C.c_uint8 * 3)(3, 0x24, 0x77)
        self.endpoint_extra = (C.c_uint8 * 2)(2, 0x25)
        self.endpoints = (usb._EndpointDescriptor * 1)()
        self.endpoints[0].bEndpointAddress = 0x81
        self.endpoints[0].wMaxPacketSize = 64
        self.endpoints[0].extra = self.endpoint_extra
        self.endpoints[0].extra_length = 2
        self.alts = (usb._InterfaceDescriptor * 2)()
        for index, alt in enumerate(self.alts):
            alt.bInterfaceNumber = 0
            alt.bAlternateSetting = index
            alt.bInterfaceClass = 0xFE
            alt.bInterfaceSubClass = 1
            alt.bInterfaceProtocol = 2
            alt.iInterface = index + 2
            alt.extra = self.extra
            alt.extra_length = len(self.extra)
        self.alts[0].bNumEndpoints = 1
        self.alts[0].endpoint = self.endpoints
        self.interfaces = (usb._Interface * 1)()
        self.interfaces[0].altsetting = self.alts
        self.interfaces[0].num_altsetting = 2
        self.config = usb._ConfigDescriptor()
        self.config.bConfigurationValue = 1
        self.config.bNumInterfaces = 1
        self.config.interface = self.interfaces
        self.config.extra = self.config_extra
        self.config.extra_length = len(self.config_extra)
        self.listing = (C.c_void_p * 3)(101, 102, None)
        methods = {
            "init": lambda p: output(p, C.c_void_p, 11) or 0,
            "exit": lambda ctx: None,
            "get_device_list": self.device_list,
            "free_device_list": lambda listing, unref: None,
            "ref_device": lambda device: device,
            "unref_device": lambda device: None,
            "get_device_descriptor": self.descriptor,
            "get_config_descriptor": self.config_descriptor,
            "free_config_descriptor": self.free_config,
            "get_bus_number": lambda device: 1,
            "get_device_address": lambda device: 9,
            "get_port_numbers": self.ports,
            "get_port_number": lambda device: {101: 3, 90: 2, 80: 0}[device],
            "get_parent": lambda device: {101: 90, 90: 80, 80: None}[device],
            "open": lambda dev, out: output(out, C.c_void_p, 201) or 0,
            "close": lambda handle: None,
            "get_string_descriptor_ascii": self.string,
            "get_configuration": lambda h, out: output(out, C.c_int, self.configuration) or 0,
            "set_configuration": self.set_configuration,
            "claim_interface": lambda h, interface: 0,
            "release_interface": lambda h, interface: 0,
            "set_interface_alt_setting": lambda h, interface, alt: 0,
            "set_auto_detach_kernel_driver": lambda h, enabled: 0 if self.auto else -12,
            "kernel_driver_active": lambda h, interface: self.active,
            "detach_kernel_driver": lambda h, interface: 0,
            "attach_kernel_driver": lambda h, interface: 0,
            "control_transfer": self.control_transfer,
            "error_name": lambda code: f"ERROR_{code}".encode(),
        }
        for name, method in methods.items():
            setattr(self, "libusb_" + name, Function(self, name, method))

    def names(self):
        return [name for name, _ in self.calls]

    def device_list(self, ctx, out):
        output(out, usb._DeviceList, C.cast(self.listing, usb._DeviceList))
        return 2

    def descriptor(self, device, out):
        descriptor = usb._DeviceDescriptor()
        descriptor.idVendor = 0x0483 if device == 101 else 0x1234
        descriptor.idProduct = 0xDF11
        descriptor.iSerialNumber = 1
        descriptor.bNumConfigurations = 1
        output(out, usb._DeviceDescriptor, descriptor)
        return 0

    def config_descriptor(self, device, index, out):
        output(out, C.POINTER(usb._ConfigDescriptor), C.pointer(self.config))
        return 0

    def free_config(self, config):
        # Poison buffers to prove returned Python objects own descriptor data.
        for i in range(len(self.extra)):
            self.extra[i] = 0
        self.config_extra[0] = 0
        self.endpoint_extra[0] = 0

    def ports(self, device, data, length):
        data[0], data[1] = 2, 3
        return 2

    def string(self, handle, index, data, size):
        text = {1: b"STM32SERIAL", 2: b"@Internal Flash", 3: b"@Option Bytes"}[index]
        C.memmove(data, text, len(text))
        return len(text)

    def set_configuration(self, handle, value):
        self.configuration = value
        return 0

    def control_transfer(self, handle, request_type, request, value, index, data, length, timeout):
        if request_type & 0x80:
            payload = self.transfer_reply[:length]
            C.memmove(data, payload, len(payload))
            return len(payload) if self.transfer_count is None else self.transfer_count
        self.out_payload = bytes(data[:length])
        return length if self.transfer_count is None else self.transfer_count


class TransportTests(unittest.TestCase):
    def test_signatures_are_native_width_and_signed(self):
        lib = FakeLib()
        with usb.LibusbTransport(lib):
            self.assertIs(lib.libusb_get_device_list.restype, C.c_ssize_t)
            self.assertIs(lib.libusb_control_transfer.restype, C.c_int)
            self.assertIs(lib.libusb_ref_device.restype, C.c_void_p)
            self.assertEqual(lib.libusb_get_device_list.argtypes[1], C.POINTER(usb._DeviceList))
            self.assertEqual(lib.libusb_control_transfer.argtypes[-2:], [C.c_uint16, C.c_uint])
        self.assertEqual(lib.names().count("exit"), 1)

    def test_enumeration_copies_and_keeps_one_reference(self):
        lib = FakeLib()
        with usb.LibusbTransport(lib) as transport:
            device, = transport.devices()
            self.assertEqual((device.bus, device.address, device.path), (1, 9, "1-2.3"))
            self.assertEqual(device.interfaces[0].extra, lib.extra_data)
            self.assertEqual(device.interfaces[0].configuration_extra, b"\x03\x24\x77")
            self.assertEqual(device.interfaces[0].endpoints[0].extra, b"\x02\x25")
            self.assertEqual(device.interfaces[1].alternate, 1)
            self.assertEqual(device.interfaces[1].name_index, 3)
            self.assertIs(transport.devices()[0], device)
            self.assertEqual(lib.names().count("ref_device"), 1)
            self.assertEqual(lib.names().count("free_device_list"), 2)
            self.assertEqual(lib.names().count("free_config_descriptor"), 1)
            with self.assertRaises(FrozenInstanceError):
                device.interfaces[0].extra = b"broken"
            self.assertNotIn("open", lib.names())
        self.assertEqual(lib.names().count("unref_device"), 1)
        self.assertLess(lib.names().index("unref_device"), lib.names().index("exit"))
        with self.assertRaises(usb.UsbError):
            device.open()

    def test_error_cleanup_of_enumeration(self):
        lib = FakeLib()
        lib.alts[0].extra_length = -1
        with usb.LibusbTransport(lib) as transport:
            with self.assertRaises(usb.UsbError):
                transport.devices()
        self.assertIn("free_config_descriptor", lib.names())
        self.assertIn("free_device_list", lib.names())
        self.assertNotIn("ref_device", lib.names())

    def test_negative_enumeration_result_does_not_become_unsigned_count(self):
        lib = FakeLib()
        lib.fail["get_device_list"] = -7
        with usb.LibusbTransport(lib) as transport:
            with self.assertRaises(usb.UsbError) as error:
                transport.devices()
            self.assertEqual(error.exception.code, -7)
        self.assertNotIn("free_device_list", lib.names())

    def test_disconnected_during_descriptor_read_is_skipped(self):
        lib = FakeLib()
        lib.fail["get_config_descriptor"] = -4
        with usb.LibusbTransport(lib) as transport:
            self.assertEqual(transport.devices(), [])
        self.assertIn("free_device_list", lib.names())
        self.assertNotIn("ref_device", lib.names())

    def test_topology_parent_fallback_and_refusal_of_unstable_address(self):
        lib = FakeLib()
        lib.fail["get_port_numbers"] = -12
        with usb.LibusbTransport(lib) as transport:
            self.assertEqual(transport.devices()[0].path, "1-2.3")
        lib = FakeLib()
        del lib.libusb_get_port_numbers
        del lib.libusb_get_parent
        with usb.LibusbTransport(lib) as transport:
            with self.assertRaises(usb.UsbError) as error:
                transport.devices()
            self.assertEqual(error.exception.code, -12)
        self.assertIn("free_device_list", lib.names())

    def test_handle_claim_string_transfer_and_cleanup(self):
        lib = FakeLib()
        with usb.LibusbTransport(lib) as transport:
            device, = transport.devices()
            with device.open() as handle:
                self.assertEqual(handle.string(0), "")
                self.assertEqual(handle.string(device.serial_index), "STM32SERIAL")
                handle.set_configuration(1)
                self.assertNotIn("set_configuration", lib.names())
                handle.set_configuration(2)
                handle.claim_interface(0)
                handle.claim_interface(0)
                handle.set_interface_altsetting(0, 1)
                self.assertEqual(handle.ctrl_transfer(0xA1, 3, 0, 0, 6, 100), b"\x01\x02\x03")
                self.assertEqual(handle.ctrl_transfer(0x21, 1, 2, 0, b"\x00\xFF", 100), 2)
                self.assertEqual(lib.out_payload, b"\x00\xFF")
                self.assertEqual(handle.ctrl_transfer(0x21, 1), 0)
            self.assertEqual(lib.names().count("claim_interface"), 1)
            self.assertEqual(lib.names().count("release_interface"), 1)
            self.assertEqual(lib.names().count("attach_kernel_driver"), 1)
            self.assertLess(lib.names().index("release_interface"), lib.names().index("attach_kernel_driver"))
            self.assertLess(lib.names().index("attach_kernel_driver"), lib.names().index("close"))
            handle.close()
            with self.assertRaises(usb.UsbError):
                handle.get_configuration()
            calls_after_close = len(lib.calls)
            with self.assertRaises(usb.UsbError):
                handle.claim_interface(0)
            self.assertEqual(len(lib.calls), calls_after_close)
        self.assertEqual(lib.names().count("close"), 1)

    def test_transport_closes_live_handles_before_unref(self):
        lib = FakeLib(auto=True)
        transport = usb.LibusbTransport(lib)
        device, = transport.devices()
        handle = device.open()
        handle.claim_interface(0)
        transport.close()
        transport.close()
        self.assertNotIn("kernel_driver_active", lib.names())
        self.assertNotIn("attach_kernel_driver", lib.names())
        self.assertLess(lib.names().index("close"), lib.names().index("unref_device"))
        self.assertEqual(lib.names().count("exit"), 1)

    def test_failed_claim_reattaches_and_preserves_error(self):
        lib = FakeLib()
        lib.fail["claim_interface"] = -6
        with usb.LibusbTransport(lib) as transport:
            device, = transport.devices()
            with device.open() as handle:
                with self.assertRaises(usb.UsbError) as error:
                    handle.claim_interface(0)
                self.assertEqual(error.exception.code, -6)
                self.assertIn("attach_kernel_driver", lib.names())
        self.assertEqual(lib.names().count("attach_kernel_driver"), 1)
        self.assertNotIn("release_interface", lib.names())

    def test_disconnect_errors_are_not_masked_by_cleanup(self):
        lib = FakeLib()
        lib.fail.update(control_transfer=-4, release_interface=-4, attach_kernel_driver=-4)
        with usb.LibusbTransport(lib) as transport:
            device, = transport.devices()
            with self.assertRaises(usb.UsbError) as error:
                with device.open() as handle:
                    handle.claim_interface(0)
                    handle.ctrl_transfer(0xA1, 3, 0, 0, 6)
            self.assertEqual(error.exception.code, -4)
            self.assertTrue(error.exception.disconnected)
        self.assertIn("close", lib.names())

    def test_open_and_auto_detach_errors_cleanup_correctly(self):
        for operation in ("open", "set_auto_detach_kernel_driver"):
            lib = FakeLib()
            lib.fail[operation] = -3
            with usb.LibusbTransport(lib) as transport:
                device, = transport.devices()
                with self.assertRaises(usb.UsbError):
                    device.open()
            self.assertEqual(lib.names().count("close"), int(operation != "open"))

    def test_transfer_errors_short_count_and_arguments(self):
        lib = FakeLib()
        with usb.LibusbTransport(lib) as transport:
            with transport.devices()[0].open() as handle:
                lib.fail["control_transfer"] = -7
                with self.assertRaises(usb.UsbError) as error:
                    handle.ctrl_transfer(0xA1, 3, 0, 0, 6)
                self.assertEqual(error.exception.code, -7)
                del lib.fail["control_transfer"]
                lib.transfer_count = 1
                self.assertEqual(handle.ctrl_transfer(0x21, 1, 0, 0, b"abc"), 1)
                lib.transfer_count = 7
                with self.assertRaises(usb.UsbError):
                    handle.ctrl_transfer(0xA1, 3, 0, 0, 6)
                before = lib.names().count("control_transfer")
                for args in ((0xA1, 3, 0, 0, 65536), (0x21, 1, 0, 0, 5),
                             (0x21, 1, 0, 0, b"", 0), (0x21, 1, 0, 0, b"", 60001),
                             (0x100, 1), (0x21, 1, -1)):
                    with self.assertRaises(ValueError):
                        handle.ctrl_transfer(*args)
                self.assertEqual(lib.names().count("control_transfer"), before)

    def test_loader_homebrew_fallback_and_explicit_override(self):
        sentinel = object()
        def loader(name):
            if name == "/opt/homebrew/lib/libusb-1.0.dylib":
                return sentinel
            raise OSError("not present")
        with patch.object(usb.sys, "platform", "darwin"), patch.object(usb, "find_library", return_value=None), \
                patch.object(usb.C, "CDLL", side_effect=loader), patch.dict(usb.os.environ, {}, clear=True):
            self.assertIs(usb._load_library(), sentinel)
        with patch.object(usb.C, "CDLL", side_effect=OSError("not present")) as load, \
                patch.dict(usb.os.environ, {"LIBUSB_LIBRARY": "/chosen/library"}, clear=True):
            with self.assertRaises(usb.UsbError):
                usb._load_library()
            load.assert_called_once_with("/chosen/library")

    def test_windows_loader_uses_stdcall(self):
        sentinel = object()
        with patch.object(usb.sys, "platform", "win32"), \
                patch.object(usb, "find_library", return_value=None), \
                patch.object(usb.C, "WinDLL", return_value=sentinel, create=True) as load, \
                patch.object(usb.C, "CDLL", side_effect=AssertionError("wrong calling convention")), \
                patch.dict(usb.os.environ, {}, clear=True):
            self.assertIs(usb._load_library(), sentinel)
            load.assert_called_once_with("libusb-1.0.dll")


if __name__ == "__main__":
    unittest.main()
