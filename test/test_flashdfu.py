"""Exercise the updater with fake host tools; no USB device is accessed."""

import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


class FlashDfuTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="mucan updater ")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.tools = self.root / "checkout" / "tools"
        self.tools.mkdir(parents=True)
        self.script = self.tools / "flashdfu.sh"
        shutil.copyfile(Path(__file__).resolve().parents[1] / "tools/flashdfu.sh", self.script)
        (self.tools / "startdfu.py").touch()
        self.default = self.tools.parent / "firmware" / "app.bin"
        self.default.parent.mkdir()
        self.default.write_bytes(b"firmware")
        self.caller = self.root / "caller directory"
        self.caller.mkdir()
        self.custom = self.caller / "release image.bin"
        self.custom.write_bytes(b"custom firmware")
        self.bin = self.root / "bin"
        self.bin.mkdir()
        self.log = self.root / "dfu args"
        self.python_log = self.root / "python args"
        self.env = dict(os.environ, PATH=str(self.bin) + os.pathsep + os.environ["PATH"],
                        TEST_DFU_LOG=str(self.log), TEST_PYTHON_LOG=str(self.python_log))
        self.stub("dfu-util", """#!/bin/sh
if [ "$1" = -l ]; then
    echo 'Found DFU: [0483:df11]'
    exit 0
fi
printf '%s\\n' "$@" > "$TEST_DFU_LOG"
for arg do firmware="$arg"; done
[ -f "$firmware" ] || exit 9
echo 'File downloaded successfully'
""")
        self.stub("python3", """#!/bin/sh
printf '%s\\n' "$@" > "$TEST_PYTHON_LOG"
[ -f "$1" ]
""")

    def stub(self, name, contents):
        path = self.bin / name
        path.write_text(contents)
        path.chmod(0o755)

    def run_updater(self, *args):
        return subprocess.run(["sh", str(self.script), *args], cwd=self.caller,
                              env=self.env, capture_output=True, text=True, timeout=5)

    def check_download(self, result, expected):
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("Firmware updated", result.stdout)
        actual = Path(self.log.read_text().splitlines()[-1])
        if not actual.is_absolute():
            actual = self.caller / actual
        self.assertEqual(actual.resolve(), expected.resolve())

    def test_relative_firmware_uses_callers_directory(self):
        self.check_download(self.run_updater("", self.custom.name), self.custom)

    def test_absolute_firmware_path(self):
        self.check_download(self.run_updater("", str(self.custom)), self.custom)

    def test_default_firmware_from_unrelated_directory(self):
        self.check_download(self.run_updater(), self.default)

    def test_serial_helper_is_found_without_changing_directory(self):
        self.check_download(self.run_updater("/dev/example", self.custom.name), self.custom)
        self.assertEqual(self.python_log.read_text().splitlines(),
                         [str(self.tools / "startdfu.py"), "/dev/example"])

    def test_missing_firmware_never_calls_host_tools(self):
        result = self.run_updater("/dev/example", "missing.bin")
        self.assertEqual(result.returncode, 1)
        self.assertFalse(self.log.exists())
        self.assertFalse(self.python_log.exists())


if __name__ == "__main__":
    unittest.main()
