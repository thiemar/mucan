"""Exercise the actual host workflow with fake DFU and CDC I/O; no USB access."""

import contextlib
import importlib.util
import io
import itertools
import os
from pathlib import Path
import shlex
import struct
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

MODULE_PATH = Path(__file__).resolve().parents[1] / "tools/flashdfu.py"
sys.path.insert(0, str(MODULE_PATH.parent))
spec = importlib.util.spec_from_file_location("flashdfu", MODULE_PATH)
flashdfu = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = flashdfu
spec.loader.exec_module(flashdfu)


def firmware(size=512):
    image = bytearray((i * 7) & 255 for i in range(size))
    struct.pack_into("<48I", image, 0, 0x20001800, *([0x080000C1] * 47))
    return bytes(image)


class FakeBoard:
    def __init__(self):
        self.image = firmware()
        self.options_raw = bytes.fromhex("AA55FF0012ED34CBFF00")
        self.uid_raw = bytes.fromhex("30002B0012514B31363930A7")
        self.path = "20-1.2"
        self.serial = "FFFFFFFEFFFF"
        self.port = "/dev/tty.test"
        self.events = []
        self.dfu = True
        self.app = False
        self.cycle = 0
        self.version = b"v0004"
        self.other_devices = []
        self.on_event = lambda event: None
        self.readback_corrupt = False
        self.upload_short = False
        self.download_status = 0
        self.leave_failure = False
        self.no_app = False
        self.no_reset = False
        self.commit_corrupt = False
        self.commit_failure = False
        self.handshake_failure = False
        self.dfu_rejected = False
        self.options_write_failure = False
        self.options_write_ignored = False
        self.option_reply = None
        self.layout = "032*0001Kg"
        self.targets = []

    def event(self, *event):
        self.events.append(event)
        self.on_event(event)

    def device(self, path=None, serial=None):
        from stm32_dfu import DfuDevice
        return DfuDevice(path or self.path, serial or self.serial, {
            0: "@Internal Flash  /0x08000000/" + self.layout,
            1: "@Option Bytes  /0x1FFFF800/01*016 e",
        })

    def devices(self):
        self.event("list")
        return ([self.device()] if self.dfu else []) + [self.device(*d) for d in self.other_devices]

    def selected(self, device, operation):
        if device.path != self.path or device.serial != self.serial:
            raise AssertionError("Operation targeted the wrong DFU path/serial")
        self.targets.append((operation, device.path, device.serial))

    def recover(self, device):
        self.selected(device, "recover")
        self.event("recover")
        self.options_raw = bytes.fromhex("AA55FF00FF00FF00FF00")
        self.image = b"\xff" * len(self.image)
        self.serial = "RECOVERED"

    def leave(self, device):
        self.selected(device, "leave")
        self.event("leave")
        if self.leave_failure:
            raise flashdfu.FlashError("DFU leave failed")
        self.dfu = False
        self.app = not self.no_app

    def write_firmware(self, device, image):
        self.selected(device, "write")
        self.event("write", flashdfu.FLASH_BASE, len(image))
        self.image = image
        if self.download_status:
            raise flashdfu.FlashError("DFU download failed")

    def read(self, device, address, size, alt=0, force=False):
        self.selected(device, "read")
        self.event("read", address, size)
        if address == flashdfu.OPTION_BASE:
            if size != 16 or alt != 1:
                raise AssertionError("Option upload must request the complete ROM block")
            payload = self.options_raw + b"\xff" * 6
        elif address == flashdfu.UID_BASE:
            payload = self.uid_raw
        elif address == flashdfu.FLASH_BASE:
            payload = self.image[:size]
            if self.readback_corrupt:
                payload = payload[:-1] + bytes([payload[-1] ^ 1])
        else:
            raise AssertionError(f"Unexpected memory read: {address:x}")
        if self.upload_short:
            raise flashdfu.FlashError("Short/oversized DFU upload")
        return payload

    def option_block(self, device):
        return self.read(device, flashdfu.OPTION_BASE, 16, alt=1)

    def options(self, device):
        return self.option_block(device)[:10]

    def write_options(self, device, block):
        self.selected(device, "options")
        self.event("options", bytes(block[:10]).hex().upper())
        if self.options_write_failure:
            raise flashdfu.FlashError("DFU option write failed")
        if not self.options_write_ignored:
            self.options_raw = bytes(block[:10])

    def uid(self, device):
        return self.read(device, flashdfu.UID_BASE, 12).hex().upper()

    def find(self, uid):
        self.event("find", uid)
        if self.cycle:
            self.cycle -= 1
            return []
        return [self.port] if self.app and self.uid_raw.hex().upper() == uid else []

    def uid_for_path(self, path):
        if path != self.port or not self.app:
            raise flashdfu.FlashError("Unknown source CDC device")
        return self.uid_raw.hex().upper()

    def command(self, path, command):
        if path != self.port:
            raise AssertionError("Operation targeted the wrong CDC port")
        self.event("command", command)
        if command == b"__dfu" and self.dfu_rejected:
            raise flashdfu.CommandRejected("Application rejected __dfu")
        if command == b"__dfu":
            self.dfu, self.app = True, False
            return b""
        if self.handshake_failure:
            raise flashdfu.FlashError("No serial connection")
        if command == b"v":
            return self.version
        if command == b"__options":
            return self.option_reply or b"OB" + self.options_raw.hex().upper().encode()
        if command == b"__bootflash":
            if self.commit_failure:
                raise flashdfu.FlashError("Disconnected before reply")
            self.options_raw = flashdfu.production_options(self.options_raw)
            if self.commit_corrupt:
                self.options_raw = self.options_raw[:4] + b"\xff\x00" + self.options_raw[6:]
            self.cycle = 0 if self.no_reset else 1
            return b""
        raise AssertionError(f"Unexpected command: {command}")


class WorkflowTests(unittest.TestCase):
    def setUp(self):
        self.board = FakeBoard()
        self.tool = self.board
        self.args = SimpleNamespace(device="", firmware="unused", production=False, start=False,
                                    recover=False, dfu_serial=None, dfu_path=None, timeout=0.001)

    def run_update(self, image=None):
        ticks = itertools.count(0, 0.00001)
        with contextlib.redirect_stdout(io.StringIO()), \
                mock.patch.object(flashdfu.time, "monotonic", side_effect=lambda: next(ticks)):
            flashdfu.update(self.args, firmware() if image is None else image,
                            self.tool, self.board, sleep=lambda seconds: None)

    def committed(self):
        return ("command", b"__bootflash") in self.board.events

    def assert_no_write_or_commit(self):
        self.assertFalse(any(event[0] == "write" for event in self.board.events))
        self.assertFalse(self.committed())

    def test_development_verifies_all_bytes_and_leaves_rom_selected(self):
        self.run_update()
        # Factory USER=FF (BOOT0 pin decides; tied low) becomes USER=77
        self.assertEqual(self.board.options_raw, bytes.fromhex("AA55778812ED34CBFF00"))
        options = self.board.events.index(("options", "AA55778812ED34CBFF00"))
        first_write = next(i for i, e in enumerate(self.board.events) if e[0] == "write")
        self.assertLess(options, first_write)
        self.assertFalse(self.committed())
        self.assertIn(("read", flashdfu.FLASH_BASE, len(firmware())), self.board.events)
        self.assertLess(self.board.events.index(("read", flashdfu.FLASH_BASE, len(firmware()))),
                        self.board.events.index(("leave",)))
        self.assertIn(("command", b"__options"), self.board.events)

    def test_production_commits_after_readback_and_app_handshake(self):
        self.args.production = True
        original = self.board.options_raw
        self.run_update()
        commit = self.board.events.index(("command", b"__bootflash"))
        self.assertLess(self.board.events.index(("read", flashdfu.FLASH_BASE, len(firmware()))), commit)
        self.assertLess(self.board.events.index(("command", b"v")), commit)
        self.assertLess(self.board.events.index(("command", b"__options")), commit)
        rom = flashdfu.rom_option_block(original + b"\xff" * 6)[:10]
        self.assertEqual(self.board.options_raw, flashdfu.production_options(rom))
        self.assertEqual(self.board.options_raw[2:4], bytes([flashdfu.APP_USER, 0x80]))
        self.assertEqual(self.board.events.count(("command", b"__options")), 2)
        self.assertEqual(self.board.options_raw[4:], original[4:])

    def test_start_verifies_stored_image_without_download(self):
        self.args.start = True
        self.run_update()
        self.assertFalse(any(event[0] == "write" for event in self.board.events))
        self.assertIn(("leave",), self.board.events)

    def test_image_mismatch_never_leaves_or_commits(self):
        self.args.production = True
        self.board.readback_corrupt = True
        with self.assertRaisesRegex(flashdfu.FlashError, "readback mismatch"):
            self.run_update()
        self.assertNotIn(("leave",), self.board.events)
        self.assertFalse(self.committed())

    def test_start_mismatch_does_not_reflash(self):
        self.args.start = True
        self.board.image = firmware()[:-4] + b"bad!"
        with self.assertRaisesRegex(flashdfu.FlashError, "readback mismatch"):
            self.run_update()
        self.assert_no_write_or_commit()

    def test_failed_download_never_starts_or_commits(self):
        self.args.production = True
        self.board.download_status = 74
        with self.assertRaisesRegex(flashdfu.FlashError, "DFU download failed"):
            self.run_update()
        self.assertNotIn(("leave",), self.board.events)
        self.assertFalse(self.committed())

    def test_leave_error_never_commits(self):
        self.args.production = True
        self.board.leave_failure = True
        with self.assertRaisesRegex(flashdfu.FlashError, "DFU leave failed"):
            self.run_update()
        self.assertFalse(self.committed())

    def test_missing_app_never_commits(self):
        self.args.production = True
        self.board.no_app = True
        with self.assertRaisesRegex(flashdfu.FlashError, "did not reconnect"):
            self.run_update()
        self.assertFalse(self.committed())

    def test_missing_serial_handshake_never_commits(self):
        self.args.production = True
        self.board.handshake_failure = True
        with self.assertRaisesRegex(flashdfu.FlashError, "No serial connection"):
            self.run_update()
        self.assertFalse(self.committed())

    def test_old_version_or_bad_option_reply_never_commits(self):
        self.args.production = True
        for version, reply in [(b"v0003", None), (b"garbage", None), (b"v0004", b"OBAA55")]:
            with self.subTest(version=version, reply=reply):
                self.board = FakeBoard()
                self.tool = self.board
                self.board.version, self.board.option_reply = version, reply
                with self.assertRaises(flashdfu.FlashError):
                    self.run_update()
                self.assertFalse(self.committed())

    def test_app_option_mismatch_never_commits(self):
        self.args.production = True
        self.board.option_reply = b"OB" + bytes.fromhex("AA55FF00FF00FF00FF00").hex().upper().encode()
        with self.assertRaisesRegex(flashdfu.FlashError, "differs from ROM"):
            self.run_update()
        self.assertFalse(self.committed())

    def test_commit_failure_or_missing_reset_is_indeterminate(self):
        self.args.production = True
        for field in ("commit_failure", "no_reset", "commit_corrupt"):
            with self.subTest(field=field):
                self.board = FakeBoard()
                self.tool = self.board
                setattr(self.board, field, True)
                with self.assertRaisesRegex(flashdfu.CommitUnverified, "indeterminate"):
                    self.run_update()
                self.assertTrue(self.committed())

    def test_interruption_before_commit_never_commits(self):
        self.args.production = True
        for phase in (("write",), ("read", flashdfu.FLASH_BASE), ("leave",),
                      ("command", b"v"), ("command", b"__options")):
            with self.subTest(phase=phase):
                self.board = FakeBoard()
                self.tool = self.board
                def interrupt(event):
                    if event[:len(phase)] == phase:
                        raise KeyboardInterrupt()
                self.board.on_event = interrupt
                with self.assertRaises(KeyboardInterrupt):
                    self.run_update()
                self.assertFalse(self.committed())

    def test_interruption_during_commit_is_indeterminate(self):
        self.args.production = True
        def interrupt(event):
            if event == ("command", b"__bootflash"):
                raise KeyboardInterrupt()
        self.board.on_event = interrupt
        with self.assertRaises(flashdfu.CommitUnverified):
            self.run_update()

    def test_invalid_firmware_has_no_host_operations(self):
        bad_images = [b"", b"x" * 32772, firmware()[:-1]]
        for offset, value in [(0, 0x20002000), (0, 0x200017FC), (4, 0x080000C0),
                              (4, 0x08008001), (8, 0xFFFFFFFF)]:
            image = bytearray(firmware())
            struct.pack_into("<I", image, offset, value)
            bad_images.append(bytes(image))
        for image in bad_images:
            with self.subTest(length=len(image)):
                with self.assertRaises(flashdfu.FlashError):
                    self.run_update(image)
                self.assertEqual(self.board.events, [])

    def test_bad_options_stop_before_application_write(self):
        for raw in ["FFFF" * 5, "CC33FF0012ED34CBFF00",
                    "AA55FF0012ED34CBFE01", "AA00FF0012ED34CBFF00"]:
            with self.subTest(raw=raw):
                self.board = FakeBoard()
                self.tool = self.board
                self.board.options_raw = bytes.fromhex(raw)
                with self.assertRaises(flashdfu.FlashError):
                    self.run_update()
                self.assert_no_write_or_commit()
                self.assertFalse(any(e[0] == "options" for e in self.board.events))
                self.assertNotIn(("recover",), self.board.events)

    def test_interrupted_pairs_are_normalized_when_rom_is_selected(self):
        self.board.options_raw = bytes.fromhex("AA55FF807FEDFFCBFF00")
        self.run_update()
        self.assertEqual(self.board.options_raw, bytes.fromhex("AA557788FF00FF00FF00"))
        self.assertNotIn(("recover",), self.board.events)
        self.assertFalse(self.committed())

    def test_production_normalizes_interrupted_pairs_and_preserves_valid_pairs(self):
        self.args.production = True
        self.board.options_raw = bytes.fromhex("AA55FF807FED34CBFF01")
        self.run_update()
        self.assertEqual(self.board.options_raw, bytes.fromhex("AA557F80FF0034CBFF00"))
        self.assertNotIn(("recover",), self.board.events)

    def test_rom_already_selected_is_not_rewritten(self):
        self.board.options_raw = bytes.fromhex("AA55778812ED34CBFF00")
        self.run_update()
        self.assertFalse(any(e[0] == "options" for e in self.board.events))
        self.assertTrue(any(e[0] == "write" for e in self.board.events))

    def test_unconfirmed_rom_selection_never_writes_flash(self):
        for flag, message in (("options_write_failure", "option write failed"),
                              ("options_write_ignored", "did not verify")):
            with self.subTest(flag=flag):
                self.board = FakeBoard()
                self.tool = self.board
                setattr(self.board, flag, True)
                with self.assertRaisesRegex(flashdfu.FlashError, message):
                    self.run_update()
                self.assert_no_write_or_commit()
                self.assertNotIn(("leave",), self.board.events)

    def test_production_profile_checked_before_any_option_write(self):
        self.args.production = True
        self.board.options_raw = bytes.fromhex("AA55FD0212ED34CBFF00")
        with self.assertRaisesRegex(flashdfu.FlashError, "supported USER"):
            self.run_update()
        self.assertFalse(any(e[0] == "options" for e in self.board.events))
        self.assert_no_write_or_commit()

    def test_explicit_recovery_reidentifies_path_and_revalidates(self):
        self.args.recover = True
        self.board.options_raw = b"\xff" * 10
        self.run_update()
        self.assertIn(("recover",), self.board.events)
        write = next(i for i, event in enumerate(self.board.events) if event[0] == "write")
        self.assertLess(self.board.events.index(("recover",)), write)
        self.assertIn(("write", self.board.path, "RECOVERED"), self.board.targets)

    def test_recovery_with_source_cdc_is_rejected_before_any_host_operation(self):
        self.args.recover = True
        self.args.device = self.board.port
        self.board.dfu, self.board.app = False, True
        with self.assertRaisesRegex(flashdfu.FlashError, "already in ROM DFU"):
            self.run_update()
        self.assertEqual(self.board.events, [])

    def test_partial_option_upload_is_rejected(self):
        self.board.upload_short = True
        with self.assertRaisesRegex(flashdfu.FlashError, "Short/oversized"):
            self.run_update()
        self.assert_no_write_or_commit()

    def test_multiple_devices_require_exact_selection(self):
        self.board.other_devices = [("20-2", "OTHER")]
        with self.assertRaisesRegex(flashdfu.FlashError, "Multiple DFU"):
            self.run_update()
        self.assert_no_write_or_commit()
        self.args.dfu_path = self.board.path
        self.run_update()
        self.assertTrue(any(event[0] == "write" for event in self.board.events))

    def test_serial_entry_does_not_choose_preexisting_other_dfu(self):
        self.board.dfu, self.board.app = False, True
        self.board.other_devices = [("20-2", "OTHER")]
        self.args.device = self.board.port
        self.run_update()
        self.assertIn(("command", b"__dfu"), self.board.events)

    def test_rejected_rom_selection_explains_boot0_and_stops(self):
        self.board.dfu, self.board.app = False, True
        self.board.dfu_rejected = True
        self.args.device = self.board.port
        with self.assertRaisesRegex(flashdfu.FlashError, "refused to select ROM boot"):
            self.run_update()
        self.assertFalse(any(event[0] == "list" for event in self.board.events[1:]))
        self.assert_no_write_or_commit()

    def test_rdp_after_serial_entry_explains_interrupted_option_rewrite(self):
        self.board.dfu, self.board.app = False, True
        self.args.device = self.board.port
        self.board.options_raw = bytes.fromhex("FFFFFFFF12ED34CBFF00")
        with self.assertRaisesRegex(flashdfu.FlashError, "rewrote its option bytes for __dfu"):
            self.run_update()
        self.assert_no_write_or_commit()

    def test_different_device_after_serial_entry_is_rejected_before_write(self):
        self.board.dfu, self.board.app = False, True
        self.args.device = self.board.port
        def swap(event):
            if event == ("command", b"__dfu"):
                self.board.uid_raw = bytes.fromhex("0102030405060708090A0B0C")
        self.board.on_event = swap
        with self.assertRaisesRegex(flashdfu.FlashError, "differs from the requested CDC"):
            self.run_update()
        self.assert_no_write_or_commit()

    def test_cdc_identity_change_after_options_never_commits(self):
        self.args.production = True
        def remove(event):
            if event == ("command", b"__options"):
                self.board.app = False
        self.board.on_event = remove
        with self.assertRaisesRegex(flashdfu.FlashError, "USB identity changed"):
            self.run_update()
        self.assertFalse(self.committed())

    def test_wrong_flash_layout_is_rejected(self):
        self.board.layout = "064*0002Kg"
        with self.assertRaisesRegex(flashdfu.FlashError, "32 KiB"):
            self.run_update()
        self.assert_no_write_or_commit()

    def test_uid_changes_before_download_are_rejected(self):
        # Second UID read: the ROM-selection check, or (ROM already selected)
        # the final preflight check
        for options, message in (("AA55FF0012ED34CBFF00", "did not verify"),
                                 ("AA55778812ED34CBFF00", "identity/options changed")):
            with self.subTest(options=options):
                self.board = FakeBoard()
                self.tool = self.board
                self.board.options_raw = bytes.fromhex(options)
                seen = 0
                def swap(event):
                    nonlocal seen
                    if event == ("read", flashdfu.UID_BASE, 12):
                        seen += 1
                        if seen == 2:
                            self.board.uid_raw = bytes.fromhex("0102030405060708090A0B0C")
                self.board.on_event = swap
                with self.assertRaisesRegex(flashdfu.FlashError, message):
                    self.run_update()
                self.assert_no_write_or_commit()


class CliTests(unittest.TestCase):
    def test_relative_path_is_read_from_caller_directory(self):
        with tempfile.TemporaryDirectory(prefix="mucan caller ") as directory:
            image = Path(directory) / "custom image.bin"
            image.write_bytes(firmware())
            harness = Path(directory) / "interpreter.py"
            harness.write_text('''import importlib.util, sys
from contextlib import nullcontext
from pathlib import Path
sys.path.insert(0, str(Path(sys.argv[1]).parent))
spec = importlib.util.spec_from_file_location("flashdfu", sys.argv[1])
module = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = module
spec.loader.exec_module(module)
module.DfuTool = lambda: nullcontext(None)
module.SerialPorts = lambda: None
def update(args, image, tool, ports):
    assert args.firmware == "custom image.bin" and args.production
    print(image.hex())
module.update = update
sys.exit(module.main(sys.argv[2:]))
''')
            interpreter = Path(directory) / "python3"
            interpreter.write_text("#!/bin/sh\nexec " + shlex.quote(sys.executable) + " " +
                                   shlex.quote(str(harness)) + ' "$@"\n')
            interpreter.chmod(0o755)
            result = subprocess.run([str(MODULE_PATH.with_suffix(".sh")), "", image.name, "--production"],
                                    cwd=directory, capture_output=True, text=True, check=True,
                                    env={**os.environ, "PATH": directory + os.pathsep + os.environ["PATH"]})
            self.assertEqual(bytes.fromhex(result.stdout.strip()), firmware())

    def test_default_path_is_relative_to_checkout(self):
        self.assertEqual(Path(flashdfu.parse_args([]).firmware), MODULE_PATH.parents[1] / "firmware/app.bin")

    def test_invalid_file_never_constructs_host_tools(self):
        with mock.patch.object(flashdfu, "DfuTool") as tool, mock.patch.object(flashdfu, "SerialPorts") as ports, \
                contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(flashdfu.main(["/dev/example", "/does-not-exist/app.bin"]), 1)
            tool.assert_not_called()
            ports.assert_not_called()

    def test_native_context_is_closed_on_success_failure_and_interrupt(self):
        with tempfile.TemporaryDirectory() as directory:
            image = Path(directory) / "app.bin"
            image.write_bytes(firmware())
            for error, status in [(None, 0), (flashdfu.FlashError("failed"), 1),
                                  (KeyboardInterrupt(), 130), (flashdfu.CommitUnverified("unknown"), 3)]:
                with self.subTest(status=status), contextlib.redirect_stderr(io.StringIO()), \
                        mock.patch.object(flashdfu, "DfuTool") as tool, \
                        mock.patch.object(flashdfu, "SerialPorts"), \
                        mock.patch.object(flashdfu, "update", side_effect=error):
                    self.assertEqual(flashdfu.main(["", str(image)]), status)
                    tool.return_value.__enter__.assert_called_once()
                    tool.return_value.__exit__.assert_called_once()

    def test_native_context_is_closed_when_serial_dependency_is_missing(self):
        with tempfile.TemporaryDirectory() as directory:
            image = Path(directory) / "app.bin"
            image.write_bytes(firmware())
            with contextlib.redirect_stderr(io.StringIO()), mock.patch.object(flashdfu, "DfuTool") as tool, \
                    mock.patch.object(flashdfu, "SerialPorts", side_effect=flashdfu.FlashError("missing")):
                self.assertEqual(flashdfu.main(["", str(image)]), 1)
                tool.return_value.__exit__.assert_called_once()

    def test_conflicting_recovery_start_rejected(self):
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            flashdfu.parse_args(["--recover", "--start"])

    def test_recovery_with_cdc_device_rejected(self):
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            flashdfu.parse_args(["--recover", "/dev/example"])

    def test_invalid_timeouts_are_rejected(self):
        for value in ("nan", "inf", "-inf", "0", "121"):
            with self.subTest(value=value), contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                flashdfu.parse_args(["--timeout=" + value])


class OptionTests(unittest.TestCase):
    def test_rom_selection_uses_option_bits_only(self):
        self.assertTrue(flashdfu.rom_selected(bytes.fromhex("AA55778812ED34CBFF00")))
        for user in ("FF00", "7F80", "F708", "FFFF"):  # pin-decided, flash, BOOT_SEL=1, erased
            self.assertFalse(flashdfu.rom_selected(bytes.fromhex("AA55" + user + "12ED34CBFF00")))

    def test_rom_block_keeps_other_bits_and_trailing_bytes(self):
        block = bytes.fromhex("AA55FB0412ED34CBFF00") + bytes.fromhex("A1B2C3D4E5F6")
        rom = flashdfu.rom_option_block(block)
        self.assertEqual(rom[2:4], bytes.fromhex("738C"))  # FB -> BOOT_SEL, nBOOT0 cleared
        self.assertEqual(rom[:2] + rom[4:], block[:2] + block[4:])
        self.assertTrue(flashdfu.rom_selected(rom[:10]))

    def test_production_commit_is_rom_to_app(self):
        self.assertEqual(flashdfu.production_options(bytes.fromhex("AA55778812ED34CBFF00")),
                         bytes.fromhex("AA557F8012ED34CBFF00"))
        with self.assertRaises(flashdfu.FlashError):
            flashdfu.production_options(bytes.fromhex("AA55FF0012ED34CBFF00"))


class SerialTransportTests(unittest.TestCase):
    def setUp(self):
        self.reply = b"v0003\r"
        self.pending = bytearray(b"\a\rstale CAN frame\r")
        self.events = []
        self.ports = flashdfu.SerialPorts.__new__(flashdfu.SerialPorts)
        class SerialException(Exception):
            pass
        self.ports.serial = SimpleNamespace(Serial=lambda *args, **kwargs: self,
                                            SerialException=SerialException)
        self.serial_error = None
        self.ports.list_ports = SimpleNamespace(comports=lambda: [])

    def __enter__(self):
        return self

    def __exit__(self, *args):
        pass

    def write(self, data):
        self.events.append(("write", data))
        if self.serial_error:
            raise self.serial_error
        # The flush (an empty line) is answered with BELL, like the firmware
        self.pending.extend(b"\a" if data == b"\r" else self.reply)
        return len(data)

    def reset_input_buffer(self):
        self.events.append(("reset",))
        self.pending.clear()

    def flush(self):
        self.events.append(("flush",))

    def read(self, size):
        data = bytes(self.pending[:size])
        del self.pending[:size]
        return data

    def command(self, command=b"v"):
        ticks = itertools.count(0, 0.01)
        with mock.patch.object(flashdfu.time, "sleep"), \
                mock.patch.object(flashdfu.time, "monotonic", side_effect=lambda: next(ticks)):
            return self.ports.command("/dev/fake", command)

    def test_flushes_without_closing_can_and_drains_stale_data(self):
        self.assertEqual(self.command(), b"v0003")
        self.assertEqual(self.events, [("write", b"\r"), ("write", b"v\r"), ("flush",)])
        self.assertFalse(any(b"C" in event[1] for event in self.events if event[0] == "write"))

    def test_received_frames_around_the_reply_are_skipped(self):
        self.reply = b"t1230\rR123456780\rT0000000112\rv0003\rt7FF0\r"
        self.assertEqual(self.command(), b"v0003")

    def test_rejection_is_distinguishable(self):
        self.reply = b"\a"
        with self.assertRaises(flashdfu.CommandRejected):
            self.command(b"__dfu")

    def test_empty_success_ack(self):
        self.reply = b"\r"
        self.assertEqual(self.command(b"__bootflash"), b"")

    def test_bell_timeout_and_oversized_replies_fail(self):
        for reply, message in [(b"\a", "rejected"), (b"", "No serial reply"),
                               (b"x" * 97, "Oversized")]:
            with self.subTest(reply=reply):
                self.reply = reply
                with self.assertRaisesRegex(flashdfu.FlashError, message):
                    self.command()

    def test_disconnection_becomes_workflow_error(self):
        self.serial_error = self.ports.serial.SerialException("disconnected")
        with self.assertRaisesRegex(flashdfu.FlashError, "disconnected"):
            self.command()

    def test_source_port_requires_full_uid_and_product_match(self):
        port = SimpleNamespace(device="/dev/fake", vid=flashdfu.APP_VID, pid=flashdfu.APP_PID,
                               serial_number="30002b0012514b31363930a7")
        self.ports.list_ports.comports = lambda: [port]
        self.assertEqual(self.ports.uid_for_path("/dev/fake"), port.serial_number.upper())
        self.assertEqual(self.ports.find(port.serial_number.upper()), ["/dev/fake"])
        port.serial_number = "UNKNOWN"
        with self.assertRaises(flashdfu.FlashError):
            self.ports.uid_for_path("/dev/fake")
        port.serial_number = "30002b0012514b31363930a7"
        port.pid = 0
        with self.assertRaises(flashdfu.FlashError):
            self.ports.uid_for_path("/dev/fake")


if __name__ == "__main__":
    unittest.main()
