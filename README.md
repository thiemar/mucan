## µCAN

USB to CAN adapter based on the STM32F042. It appears as a USB serial port
(CDC-ACM) and speaks the SLCAN (Lawicel) protocol, so it works with Linux
`slcand`/SocketCAN, python-can's `slcan` interface and similar tools.

### SLCAN commands

| Command | Meaning | Reply |
|---|---|---|
| `Sn` | Bitrate, n = 0..8: 10k 20k 50k 100k 125k 250k 500k 800k 1M (channel closed) | `\r` |
| `O` / `L` / `C` | Open / open listen-only / close the channel | `\r` |
| `tiiildd…` | Send a standard frame (ID 000–7FF, length 0–8, data in hex) | `z\r` |
| `Tiiiiiiiildd…` | Send an extended frame (ID 00000000–1FFFFFFF) | `Z\r` |
| `riiil` / `Riiiiiiiil` | Send a remote frame | `z\r` / `Z\r` |
| `F` | Status flags (channel open), as two hex digits | `Fxx\r` |
| `V` / `v` / `N` | Hardware version, firmware version, serial number | `V0001\r`, `v0002\r`, `Nxxxx\r` |
| `__dfu` | Restart into the USB DFU bootloader | `\r` |

Failures are answered with BELL (`\a`). Received frames are sent in the same
`t`/`T`/`r`/`R` format, ending in `\r`. Frames are only sent while the channel
is open; transmits wait up to 100 ms for room in the 16-frame transmit queue
(the host is held off over USB meanwhile) and then fail with BELL.

The `F` bits are: 0 receive buffer overflowed, 1 transmit queue full, 2 error
warning, 3 receive overrun, 5 error passive, 6 arbitration lost, 7 bus error
or bus off. Bits 0, 3, 6 and 7 are cleared when read.

The channel closes when the host drops DTR (closes the port) or the USB
connection is reset.

### Building

You need the `arm-none-eabi` GCC toolchain on your PATH (any recent release
of the Arm GNU Toolchain works, as does the `gcc-arm-none-eabi` package on
Debian/Ubuntu):

    git clone https://github.com/thiemar/mucan.git
    cd mucan
    make -j8

This produces `firmware/app.bin`. To use a toolchain that isn't on your PATH:

    make CROSS=/path/to/bin/arm-none-eabi-

`make test` runs the host-side unit tests for USB, CAN, boot-option migration,
SLCAN and the firmware updater. It needs Python 3 and a host C compiler with
AddressSanitizer support (clang or gcc). The updater tests use fake host tools
and do not access a USB device.

`make test-firmware` builds the firmware and checks the actual ARM image's
startup copy/clear boundaries and load addresses. It also links fixtures with
odd section sizes to check alignment independently of the current code size.
Use the same `CROSS` setting as for `make`.

### Upgrading firmware

Install [`dfu-util`](http://dfu-util.sourceforge.net), Python 3 and
`pyserial` (`pip install pyserial`). Attach the adapter, find its serial
device (`/dev/ttyACM<x>` on Linux, `/dev/tty.usbmodem<x>` on macOS), then:

    tools/flashdfu.sh /path/to/device

An optional second argument selects another firmware image. Relative paths
are resolved from your current directory:

    tools/flashdfu.sh /path/to/device firmware/custom.bin

This asks the adapter to restart into the STM32's built-in DFU bootloader
(USB ID `0483:df11`), writes the firmware and starts it. If the adapter is
already in DFU mode, leave out the device:

    tools/flashdfu.sh

or run `dfu-util` directly:

    dfu-util -d 0483:df11 -a 0 -s 0x08000000:leave -D firmware/app.bin

Replugging the adapter while it is in DFU mode returns it to its current
firmware. If an update fails partway through, re-run the `dfu-util` command
while the adapter is still in DFU mode. If it can no longer start or reach
DFU, recover it through an SWD programmer such as an ST-Link.

On its first boot, this firmware migrates the STM32F042 boot options to
software-selected flash boot (`BOOT_SEL=0`, `nBOOT0=1`). This permits the
RAM-flag DFU jump with populated application flash without asserting BOOT0.
Other option bytes are preserved, and subsequent DFU requests do not rewrite
them. The physical BOOT0 pin no longer overrides boot selection after this
migration; use SWD for recovery. Keep power connected during the first boot's
option migration. If migration fails, `__dfu` returns BELL.

#### Upgrading from older firmware

Older firmware entered DFU mode by changing the option bytes, which made the
adapter boot into the bootloader until they were changed back (the old
instructions used an `ob.bin` file for that). The `:leave` step above starts
the new firmware, which migrates the option bytes on its first boot. If an
adapter stays in DFU mode after being replugged, run the `dfu-util` command
above.
