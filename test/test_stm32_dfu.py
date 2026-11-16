"""Protocol traces against a stateful ROM model; never load libusb or access USB."""

from pathlib import Path
import struct
import sys
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import stm32_dfu as dfu


class UsbError(Exception):
    def __init__(self, code=-1):
        self.code = code
        super().__init__(f"USB error {code}")


class Clock:
    def __init__(self):
        self.now = 0
        self.sleeps = []

    def sleep(self, delay):
        self.sleeps.append(delay)
        self.now += delay


class Rom:
    """Implements AN3156 address arithmetic, not the host's chunking strategy."""
    def __init__(self):
        self.bus, self.address, self.port_numbers = 3, 17, (2, 1)
        self.path, self.serial_index = "3-2.1", 3
        functional = bytes.fromhex("09210BFF0080001A01")  # 128-byte maximum
        self.interfaces = [SimpleNamespace(configuration_value=2, number=3, alternate=alt,
                                          name_index=alt + 4, interface_class=0xFE,
                                          subclass=1, protocol=2, extra=functional,
                                          configuration_extra=b"") for alt in (0, 1)]
        self.strings = {3: "ROM123", 4: "@Internal Flash /0x08000000/032*0001Kg",
                        5: "@Option Bytes /0x1FFFF800/01*016 e"}
        self.flash = bytearray(b"\x55" * dfu.FLASH_SIZE)
        self.options = bytes.fromhex("AA55FF0012ED34CBFF00") + b"\xff" * 6
        self.uid = bytes.fromhex("00112233445566778899AABB")
        self.state, self.status = dfu.DFU_IDLE, 0
        self.pointer, self.pending = dfu.FLASH_BASE, None
        self.alt = 0
        self.trace = []
        self.open_count = self.close_count = 0
        self.claimed = False
        self.fail_command = None
        self.error_after_upload = False
        self.stuck_busy = False
        self.poll_ms = 7
        self.short_upload = False
        self.malformed_status = False
        self.reset_error = None
        self.download_error = None

    def open(self):
        self.open_count += 1
        return self

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.claimed = False
        self.close_count += 1

    def string(self, index):
        return self.strings[index]

    def set_configuration(self, value):
        assert value == 2
        self.trace.append(("configuration", value))

    def claim_interface(self, number):
        assert number == 3
        self.claimed = True
        self.trace.append(("claim", number))

    def set_interface_altsetting(self, number, alt):
        assert self.claimed and number == 3
        self.alt = alt
        self.trace.append(("alt", alt))

    def ctrl_transfer(self, direction, request, value, index, data, timeout_ms):
        assert self.claimed and index == 3 and 0 < timeout_ms <= 2000
        self.trace.append((direction, request, value, index, data))
        if request == dfu.GETSTATUS:
            assert (direction, value, data) == (0xA1, 0, 6)
            if self.malformed_status:
                return bytes(5)
            if self.state == dfu.DNLOAD_SYNC:
                block, payload = self.pending
                self.state = dfu.DNBUSY
                if payload == b"" or payload == b"\x92":
                    if self.reset_error:
                        raise UsbError(self.reset_error)
                    self.state = dfu.MANIFEST if payload == b"" else dfu.DNBUSY
                return self.reply()
            if self.state == dfu.DNBUSY and not self.stuck_busy:
                self.execute()
            return self.reply()
        if request == dfu.CLRSTATUS:
            assert self.state == dfu.DFU_ERROR
            self.state, self.status = dfu.DFU_IDLE, 0
            return 0
        if request == dfu.ABORT:
            assert self.state in (dfu.DFU_IDLE, dfu.DNLOAD_IDLE, dfu.UPLOAD_IDLE)
            self.state = dfu.DFU_IDLE
            return 0
        if request == dfu.DNLOAD:
            assert direction == 0x21 and self.state in (dfu.DFU_IDLE, dfu.DNLOAD_IDLE)
            if self.download_error:
                raise UsbError(self.download_error)
            assert len(data) <= 128
            self.pending = (value, bytes(data))
            self.state = dfu.DNLOAD_SYNC
            return len(data)
        if request == dfu.UPLOAD:
            assert direction == 0xA1 and self.state in (dfu.DFU_IDLE, dfu.UPLOAD_IDLE)
            assert value >= 2 and 2 <= data <= 128
            address = self.pointer + (value - 2) * data
            if self.alt == 1:
                assert (address, data) == (dfu.OPTION_BASE, 16)
                result = self.options
            elif address == dfu.UID_BASE:
                assert data == 12
                result = self.uid
            else:
                offset = address - dfu.FLASH_BASE
                assert 0 <= offset < len(self.flash)
                result = bytes(self.flash[offset:offset + data])
            self.state = dfu.UPLOAD_IDLE
            if self.error_after_upload:
                self.state, self.status = dfu.DFU_ERROR, 11
            return result[:-1] if self.short_upload else result
        raise AssertionError(f"Unexpected DFU request {request}")

    def reply(self):
        return bytes([self.status]) + self.poll_ms.to_bytes(3, "little") + bytes([self.state, 0])

    def execute(self):
        block, payload = self.pending
        if block == 0:
            command = payload[0]
            if command == self.fail_command:
                self.state, self.status = dfu.DFU_ERROR, 3
                return
            assert len(payload) == 5  # No mass erase, option writes or unexpected commands.
            address = struct.unpack_from("<I", payload, 1)[0]
            if command == 0x21:
                self.pointer = address
            elif command == 0x41:
                offset = address - dfu.FLASH_BASE
                assert self.alt == 0 and offset % 1024 == 0 and 0 <= offset < dfu.FLASH_SIZE
                self.flash[offset:offset + 1024] = b"\xff" * 1024
            else:
                raise AssertionError(f"Unexpected command {command}")
        else:
            address = self.pointer + (block - 2) * len(payload)
            offset = address - dfu.FLASH_BASE
            assert self.alt == 0 and 0 <= offset < dfu.FLASH_SIZE and len(payload) % 4 == 0
            assert self.flash[offset:offset + len(payload)] == b"\xff" * len(payload)
            self.flash[offset:offset + len(payload)] = payload
        self.state = dfu.DNLOAD_IDLE


class Transport:
    error_type = UsbError

    def __init__(self, rom):
        self.rom = rom
        self.closed = False

    def devices(self, vid, pid):
        assert (vid, pid) == (0x0483, 0xDF11)
        return [self.rom]

    def close(self):
        self.closed = True


class ProtocolTests(unittest.TestCase):
    def setUp(self):
        self.rom, self.clock = Rom(), Clock()
        self.transport = Transport(self.rom)
        self.tool = dfu.DfuTool(self.transport, self.clock.sleep, lambda: self.clock.now)
        self.device = self.tool.devices()[0]

    def tearDown(self):
        self.assertEqual(self.rom.open_count, self.rom.close_count)
        self.assertFalse(self.rom.claimed)

    def transfers(self, request):
        return [event for event in self.rom.trace if len(event) == 5 and event[1] == request]

    def test_enumerates_real_interface_and_descriptor(self):
        self.assertEqual(self.device.key, ("3-2.1", "ROM123"))
        self.assertEqual((self.device.configuration, self.device.interface, self.device.transfer_size), (2, 3, 128))
        self.device.validate_layout()
        with self.tool:
            self.assertEqual(self.tool.uid(self.device), self.rom.uid.hex().upper())
        self.assertTrue(self.transport.closed)

    def test_page_erases_and_short_final_chunk_round_trip(self):
        image = bytes(i % 251 for i in range(2052))
        self.tool.write_firmware(self.device, image)
        self.assertEqual(self.tool.read(self.device, dfu.FLASH_BASE, len(image)), image)
        self.assertEqual(self.rom.flash[3072:], b"\x55" * (dfu.FLASH_SIZE - 3072))
        downloads = self.transfers(dfu.DNLOAD)
        erase_addresses = [struct.unpack_from("<I", e[4], 1)[0] for e in downloads
                           if e[2] == 0 and e[4][:1] == b"\x41"]
        self.assertEqual(erase_addresses, [dfu.FLASH_BASE + n * 1024 for n in range(3)])
        writes = [e for e in downloads if e[2] >= 2]
        self.assertEqual([e[2] for e in writes], [2] * 17)
        self.assertEqual(len(writes[-1][4]), 4)
        self.assertEqual([e[2] for e in self.transfers(dfu.UPLOAD)], [2] * 17)
        self.assertTrue(self.clock.sleeps)
        self.assertTrue(all(delay == 0.007 for delay in self.clock.sleeps))

    def test_options_read_is_one_complete_block_on_alt_one(self):
        self.assertEqual(self.tool.options(self.device), self.rom.options[:10])
        self.assertEqual([e[4] for e in self.transfers(dfu.UPLOAD)], [16])
        self.assertIn(("alt", 1), self.rom.trace)
        self.assertTrue(all(e[2] == 0 and e[4][0] == 0x21 for e in self.transfers(dfu.DNLOAD)))

    def test_previous_upload_and_error_states_are_recovered(self):
        for state, status in [(dfu.UPLOAD_IDLE, 0), (dfu.DNLOAD_IDLE, 0), (dfu.DFU_ERROR, 11)]:
            with self.subTest(state=state):
                self.rom.state, self.rom.status = state, status
                self.assertEqual(self.tool.uid(self.device), self.rom.uid.hex().upper())
        self.assertEqual(len(self.transfers(dfu.CLRSTATUS)), 1)

    def test_error_after_erase_is_not_cleared_or_followed_by_write(self):
        self.rom.fail_command = 0x41
        with self.assertRaisesRegex(dfu.FlashError, "status 0x03"):
            self.tool.write_firmware(self.device, bytes(128))
        self.assertFalse(self.transfers(dfu.CLRSTATUS))
        self.assertFalse(any(e[2] >= 2 for e in self.transfers(dfu.DNLOAD)))

    def test_read_checks_device_status_and_exact_length(self):
        for failure in ("error_after_upload", "short_upload"):
            with self.subTest(failure=failure):
                setattr(self.rom, failure, True)
                with self.assertRaises(dfu.FlashError):
                    self.tool.uid(self.device)
                setattr(self.rom, failure, False)
                self.assertFalse(self.transfers(dfu.CLRSTATUS))
                self.rom.state, self.rom.status = dfu.DFU_IDLE, 0

    def test_busy_and_excessive_poll_timeout_are_bounded(self):
        for delay in (0, 0xFFFFFF):
            with self.subTest(delay=delay):
                self.rom.state, self.rom.status = dfu.DFU_IDLE, 0
                self.rom.stuck_busy, self.rom.poll_ms = True, delay
                start = self.clock.now
                with self.assertRaisesRegex(dfu.FlashError, "timeout|timed out"):
                    self.tool.uid(self.device)
                self.assertLessEqual(self.clock.now - start, 45)

    def test_malformed_status_and_unexpected_states_fail(self):
        self.rom.malformed_status = True
        with self.assertRaisesRegex(dfu.FlashError, "Malformed"):
            self.tool.uid(self.device)
        self.rom.malformed_status = False
        self.rom.state = dfu.MANIFEST
        with self.assertRaisesRegex(dfu.FlashError, "state 7"):
            self.tool.uid(self.device)

    def test_leave_sets_flash_address_then_only_zero_length_download(self):
        self.tool.leave(self.device)
        downloads = self.transfers(dfu.DNLOAD)
        self.assertEqual([(e[2], e[4]) for e in downloads],
                         [(0, b"\x21" + struct.pack("<I", dfu.FLASH_BASE)), (0, b"")])
        self.assertEqual(self.rom.state, dfu.MANIFEST)

    def test_recover_is_the_only_unprotect_command(self):
        self.tool.recover(self.device)
        self.assertEqual([(e[2], e[4]) for e in self.transfers(dfu.DNLOAD)], [(0, b"\x92")])

    def test_disconnect_is_tolerated_only_at_reset_status_boundary(self):
        for code in (-4, -7, -9):
            for method in (self.tool.leave, self.tool.recover):
                with self.subTest(code=code, method=method):
                    self.rom.state, self.rom.status = dfu.DFU_IDLE, 0
                    self.rom.reset_error = code
                    method(self.device)
        self.rom.state, self.rom.status = dfu.DFU_IDLE, 0
        self.rom.reset_error = -1
        with self.assertRaisesRegex(dfu.FlashError, "USB transfer failed"):
            self.tool.recover(self.device)
        self.rom.state, self.rom.status = dfu.DFU_IDLE, 0
        for code in (-4, -7, -9):
            self.rom.download_error = code
            with self.assertRaisesRegex(dfu.FlashError, "USB transfer failed"):
                self.tool.write_firmware(self.device, bytes(4))

    def test_invalid_ranges_and_images_do_not_issue_requests(self):
        for address, size, alt in [(dfu.OPTION_BASE, 10, 1), (dfu.OPTION_BASE, 16, 0),
                                   (dfu.FLASH_BASE, 32770, 0), (dfu.UID_BASE, 16, 0),
                                   (dfu.FLASH_BASE + 1, 4, 0)]:
            with self.assertRaises(dfu.FlashError):
                self.tool.read(self.device, address, size, alt)
        for image in (b"", bytes(3), bytes(32772)):
            with self.assertRaises(dfu.FlashError):
                self.tool.write_firmware(self.device, image)
        self.assertEqual(self.rom.trace, [])

    def test_descriptor_or_identity_changes_prevent_transfers(self):
        self.rom.strings[3] = "OTHER"
        with self.assertRaisesRegex(dfu.FlashError, "disappeared"):
            self.tool.uid(self.device)
        self.rom.strings[3] = "ROM123"
        self.rom.strings[4] = "@Internal Flash /0x08000000/064*0001Kg"
        with self.assertRaisesRegex(dfu.FlashError, "32 KiB"):
            self.tool.uid(self.device)
        self.assertEqual(self.rom.trace, [])

    def test_missing_or_malformed_functional_descriptor_rejected(self):
        for extra in (b"", b"\x00\x21", bytes.fromhex("092101FF0080001A01")):
            for interface in self.rom.interfaces:
                interface.extra = extra
            with self.assertRaises(dfu.FlashError):
                self.tool.devices()

    def test_same_port_and_serial_replacement_cannot_be_erased(self):
        replacement = Rom()
        self.transport.rom = replacement
        with self.assertRaisesRegex(dfu.FlashError, "connection was replaced"):
            self.tool.write_firmware(self.device, bytes(128))
        self.assertEqual(replacement.trace, [])
        self.assertEqual(replacement.open_count, replacement.close_count)

    def test_changed_interface_configuration_or_transfer_size_is_rejected(self):
        for field, value in (("number", 4), ("configuration_value", 3),
                             ("extra", bytes.fromhex("09210BFF0040001A01"))):
            old = [getattr(i, field) for i in self.rom.interfaces]
            for interface in self.rom.interfaces:
                setattr(interface, field, value)
            with self.assertRaisesRegex(dfu.FlashError, "descriptors changed"):
                self.tool.uid(self.device)
            for interface, original in zip(self.rom.interfaces, old):
                setattr(interface, field, original)
        self.assertEqual(self.rom.trace, [])


if __name__ == "__main__":
    unittest.main()
