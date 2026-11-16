"""Check startup's word-copy contract against actual ARM ELF load addresses."""

import argparse
from pathlib import Path
import struct
import subprocess
import tempfile
import unittest


class Elf:
    def __init__(self, path):
        self.data = Path(path).read_bytes()
        if self.data[:6] != b"\x7fELF\x01\x01":
            raise ValueError("expected a little-endian ELF32 image")
        header = struct.unpack_from("<16sHHIIIIIHHHHHH", self.data)
        raw = [struct.unpack_from("<10I", self.data, header[6] + i * header[11])
               for i in range(header[12])]

        def contents(section):
            return self.data[section[4]:section[4] + section[5]]

        def string(table, offset):
            return table[offset:table.index(b"\0", offset)].decode("ascii")

        names = contents(raw[header[13]])
        self.sections = {string(names, s[0]): s for s in raw}
        self.symbols = {}
        for section in raw:
            if section[1] != 2:  # SHT_SYMTAB
                continue
            strings = contents(raw[section[6]])
            for offset in range(section[4], section[4] + section[5], section[9]):
                name, value, _, _, _, _ = struct.unpack_from("<IIIBBH", self.data, offset)
                self.symbols[string(strings, name)] = value
        self.segments = [struct.unpack_from("<8I", self.data, header[5] + i * header[9])
                         for i in range(header[10])]

    def load_address(self, name):
        section = self.sections[name]
        for segment in self.segments:
            kind, offset, _, physical, file_size, _, _, _ = segment
            if kind == 1 and offset <= section[4] and section[4] + section[5] <= offset + file_size:
                return physical + section[4] - offset
        raise ValueError("section has no file-backed load segment: " + name)


class LinkerTests(unittest.TestCase):
    def check_layout(self, path):
        elf = Elf(path)
        symbols = elf.symbols
        for symbol in ("_eronly", "_sdata", "_edata", "_sbss", "_ebss"):
            self.assertEqual(symbols[symbol] % 4, 0, symbol)
        self.assertEqual(symbols["_eronly"], elf.load_address(".data"))
        self.assertEqual(symbols["_edata"] - symbols["_sdata"], elf.sections[".data"][5])
        self.assertEqual(symbols["_ebss"] - symbols["_sbss"], elf.sections[".bss"][5])
        self.assertLessEqual(symbols["_edata"], symbols["_sbss"])
        if ".noinit" in elf.sections:
            self.assertLessEqual(symbols["_ebss"], elf.sections[".noinit"][3])
            self.assertEqual(elf.sections[".noinit"][1], 8)  # SHT_NOBITS: never copied
        self.assertEqual(symbols["__stack"] % 8, 0)
        text = elf.sections[".text"]
        self.assertEqual(text[3], 0x08000000)
        self.assertEqual(struct.unpack_from("<I", elf.data, text[4])[0], symbols["__stack"])
        ram_end = symbols["_ebss"]
        if ".noinit" in elf.sections:
            ram_end = elf.sections[".noinit"][3] + elf.sections[".noinit"][5]
        self.assertLessEqual(ram_end, symbols["__stack"])
        return elf

    def test_firmware_layout(self):
        self.check_layout(OPTIONS.firmware)

    def test_odd_section_sizes_and_flash_padding(self):
        linker = Path(__file__).resolve().parents[1] / "app.ld"
        with tempfile.TemporaryDirectory(prefix="mucan linker ") as temp:
            source = Path(temp) / "fixture.S"
            output = Path(temp) / "fixture.elf"
            for padding in range(4):
                with self.subTest(flash_padding=padding):
                    source.write_text(""".syntax unified
.cpu cortex-m0
.thumb
.section .app_vectors,"a",%progbits
.global app_vectors
app_vectors:
.word __stack, __start
.section .text,"ax",%progbits
.global __start, abort
.thumb_func
__start: b .
.thumb_func
abort: b .
.section .rodata,"a",%progbits
.byte 0x55
""" + ".space " + str(padding) + "\n" + """.section .data,"aw",%progbits
.byte 0x11, 0x22, 0x33
.section .bss,"aw",%nobits
.space 5
.section .noinit,"aw",%nobits
.space 8
""")
                    subprocess.run([OPTIONS.cross + "gcc", "-mcpu=cortex-m0", "-mthumb",
                                    "-nostdlib", "-Wl,--build-id=none", "-T", str(linker),
                                    str(source), "-o", str(output)], check=True, capture_output=True)
                    elf = self.check_layout(output)
                    data = elf.sections[".data"]
                    self.assertEqual(elf.data[data[4]:data[4] + 3], b"\x11\x22\x33")
                    self.assertEqual(data[5], 4)
                    self.assertEqual(elf.sections[".bss"][5], 8)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--cross", default="arm-none-eabi-")
    parser.add_argument("--firmware", default="../firmware/app.elf")
    OPTIONS, remaining = parser.parse_known_args()
    unittest.main(argv=[__file__, *remaining])
