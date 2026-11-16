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
| `V` / `v` / `N` | Hardware version, firmware version, serial number | `V0001\r`, `v0003\r`, `Nxxxx\r` |
| `__dfu` / `__bootrom` | Select persistent ROM DFU boot, then restart (BELL if BOOT0 reads low) | `\r` |
| `__bootflash` | Select persistent application boot, then restart | `\r` |
| `__options` | Read the five raw option/complement pairs in address order | `OB` + 20 hex digits + `\r` |

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

`make test` runs the host-side unit tests for USB, CAN, explicit boot-option
changes, clock start-up (including the SysTick retune after the switch to
48 MHz), SLCAN, the firmware updater, the ROM DFU protocol
and the libusb binding. It needs Python 3 and a host C compiler with
AddressSanitizer support (clang or gcc). The updater tests use fake host tools
and do not access a USB device.

`make test-firmware` builds the firmware and checks the actual ARM image's
startup copy/clear boundaries and load addresses. It also links fixtures with
odd section sizes to check alignment independently of the current code size.
Use the same `CROSS` setting as for `make`.

### USB provisioning and firmware updates

This workflow is for the STM32F042C6U6 production board with **PF11/BOOT0
connected directly to VDD**. Factory options select the ROM USB DFU
bootloader on power-up. No SWD connection is required. The application never
programs option bytes during normal startup.

The updater implements the required STM32 ROM DFU commands in Python and
calls **libusb directly through `ctypes`**. It does not require `dfu-util`
or PyUSB. Install Python 3, [libusb 1.0](https://libusb.info/), and pyserial:

    python3 -m pip install -r tools/requirements.txt

On macOS, libusb is available through `brew install libusb`. On Linux, install
your distribution's libusb 1.0 runtime and grant your account access to the
DFU USB device. Windows needs a libusb-compatible driver, such as WinUSB,
for the STM32 DFU interface.
If library discovery fails, set `LIBUSB_LIBRARY` to the native library's full
path. Unit tests use simulated USB devices and do not require libusb or pyserial.

`tools/flashdfu.sh` is a compatibility wrapper for `python3 tools/flashdfu.py`.
The utility checks the image before
accessing the board, confirms persistent ROM boot, writes application flash,
and reads the complete image back for a byte-for-byte comparison. It then
starts the verified image and reconnects to the same adapter's USB serial
interface, checking its firmware version and option bytes.

#### Development

With a new board, or one already in ROM DFU:

    tools/flashdfu.sh

The application starts for this session, but ROM boot remains selected.
Unplugging and reconnecting the board returns to DFU, including when a
development image fails to start. A serial handshake failure is reported
without changing the boot options.

To start an already-written image, verifying it against the local binary
without programming it again:

    tools/flashdfu.sh --start

When the application is running, supply its serial device to request
persistent ROM boot before updating:

    tools/flashdfu.sh /path/to/device

An optional second positional argument selects the image. Relative paths
are resolved from the caller's working directory:

    tools/flashdfu.sh /path/to/device firmware/custom.bin

#### Production

To provision a new board for automatic application startup:

    tools/flashdfu.sh --production

To update a running production board:

    tools/flashdfu.sh --production /path/to/device

Production mode performs the same write, readback and serial reconnection
as development mode. **Only after those checks pass** does it send
`__bootflash` to the running application. It then reconnects again and checks
the committed options. The host utility does not write option-byte images
through DFU.

For the standard production configuration, the USER byte is `0xFF` for ROM
boot and `0x7F` for application boot. When the application selects ROM it
stores USER as an erased pair (`FF FF`), which loads as `0xFF` exactly like
the factory's programmed `FF 00`. The application preserves valid implemented
option pairs and normalizes invalid pairs to their effective erased `0xFF`
setting. It verifies each write and programs USER last. When the stored
options already match the requested mode, it skips programming.

Selecting ROM on a production board erases and rewrites the option bank (RDP
first). Selecting the application afterwards only programs the erased USER
pair in place, so the production commit never erases the bank. A first
`--production` on a factory board does erase it once. `__dfu` is refused with
BELL if BOOT0 reads low, because the reset would only restart the
application.

#### Interrupted updates and recovery

Before application flash is changed, ROM boot is selected persistently.
An interrupted transfer therefore leaves the board accessible through ROM
DFU after reconnecting USB. Retry the appropriate flashing command. The
updater refuses to write application flash if it cannot establish this
boot configuration.

An erased or invalid USER option pair is interpreted as `0xFF`, which selects
ROM with this board's high BOOT0. Losing power while the option bank is
erased and rewritten (during `__dfu` on a production board, about 40 ms) can
also leave RDP erased, which reads as level 1. Explicit recovery removes that protection and
**erases the application**, then reflashes and verifies the supplied image:

    tools/flashdfu.sh --recover

Add `--production` to commit application boot after successful recovery and
serial validation. Never enable RDP level 2; that setting is irreversible
and disables ROM recovery.

When several DFU devices are connected, select the intended adapter with
`--dfu-serial SERIAL` or `--dfu-path BUS-PORT`; the utility refuses ambiguous
selection. Run `tools/flashdfu.sh --help` for all options.

Production firmware must support the boot commands and successfully start
its USB serial interface. This workflow checks transfer integrity and that
basic startup path; it does not provide a separate recovery loader for an
application that later stops accepting commands. Keep experimental firmware
in development mode.

The option-byte fallback and read-protection behavior are specified in
[RM0091, sections 2.5, 3.3 and 4](https://www.st.com/resource/en/reference_manual/DM00031936.pdf).
ROM readback, option programming and Read Unprotect are specified in
[AN3156](https://www.st.com/resource/en/application_note/an3156-how-to-use-usb-dfu-protocol-in-bootloader-on-stm32-mcus-stmicroelectronics.pdf).
