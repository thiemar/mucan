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
| `V` / `v` / `N` | Hardware version, firmware version, serial number | `V0001\r`, `v0004\r`, `Nxxxx\r` |
| `__dfu` / `__bootrom` | Select persistent ROM DFU boot (USER `0x77`), then restart | `\r` |
| `__bootflash` | Select persistent application boot (USER `0x7F`), then restart | `\r` |
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

#### Hardware: BOOT0 is tied low

On the STM32F042C6U6 production board every pin the design does not use,
**PF11/BOOT0 and the SWD pins included, is tied directly to a PCB plane.
BOOT0 is tied low.** There is no jumper, button or debug connector, and the
board is too dense to rework, so USB DFU is the only way to load firmware
and no hardware signal can force the bootloader.

With BOOT0 low, the STM32 starts its ROM USB DFU bootloader only:

- while application flash is **blank** (the empty-flash rule), or
- when the **option bits** select it: BOOT_SEL=0 with nBOOT0=0.

Once flash holds an image and the option bits do not select ROM, the only
way back to DFU is the running application's `__dfu` command. **An image
that cannot run `__dfu` can never be replaced on that board.**

| USER option byte | Boot source | Used for |
|---|---|---|
| `0xFF` (factory) | BOOT0 pin: flash, or ROM while flash is blank | new boards |
| `0x77` (BOOT_SEL=0, nBOOT0=0) | ROM DFU bootloader on every reset | development, and during updates |
| `0x7F` (BOOT_SEL=0, nBOOT0=1) | application flash | production |

#### The updater

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
In every mode the utility:

1. checks the image before accessing the board;
2. reads the option bytes through ROM DFU and, unless they already select
   ROM, **writes USER `0x77` before anything else**, waits for the adapter to
   restart into ROM on the same USB port, and verifies the options and the
   chip's unique ID. If that cannot be confirmed, no flash is written;
3. writes application flash and reads the complete image back for a
   byte-for-byte comparison;
4. starts the verified image and reconnects to the same adapter's USB serial
   interface, checking its firmware version (`v0004` or newer) and option
   bytes.

Read protection is never changed by an option write. Only the explicit
`--recover` (below) sends Read Unprotect.

#### Development

With a new (blank) board, or one already in ROM DFU:

    tools/flashdfu.sh

The application starts for this session, but ROM boot (`0x77`) remains
selected. Unplugging and reconnecting the board returns to DFU, including
when a development image fails to start.

To start an already-written image, verifying it against the local binary
without programming it again:

    tools/flashdfu.sh --start

When the application is running, supply its serial device; its `__dfu`
selects ROM (`0x77`) and restarts into DFU before the update:

    tools/flashdfu.sh /path/to/device

An optional second positional argument selects the image. Relative paths
are resolved from the caller's working directory:

    tools/flashdfu.sh /path/to/device firmware/custom.bin

#### Production

To provision a board for automatic application startup:

    tools/flashdfu.sh --production

To update a running production board:

    tools/flashdfu.sh --production /path/to/device

Production mode performs the same ROM selection, write, readback and serial
reconnection as development mode. **Only after those checks pass** does it
send `__bootflash` to the running application, which selects `0x7F`. It then
reconnects again and checks the committed options.

From then on the board can only be updated through its application's
`__dfu`. **Keep experimental firmware in development mode**: a production
image that stops accepting commands cannot be recovered.

The application preserves valid implemented option pairs (RDP, DATA, WRP)
and normalizes invalid pairs to their effective `0xFF` setting. It verifies
each write and programs USER last. When the stored options already match
the requested mode, it skips programming.

#### Interrupted updates and recovery

Flash is only written while `0x77` is selected, so an interrupted transfer
leaves the board in ROM DFU after reconnecting USB. Retry the same command.

Switching between `0x77` and `0x7F` erases and rewrites the option bank
(about 40 ms). If power is lost during that erase, USER reads erased (`0xFF`:
the pin, so the existing application starts) and RDP reads erased, which is
read-protection level 1. Run the updater with the application's serial
device again so `__dfu` selects ROM, then clear level 1 with explicit
recovery, which **erases the application** and then reflashes and verifies
the supplied image:

    tools/flashdfu.sh --recover

Add `--production` to commit application boot after successful recovery and
serial validation. Never enable RDP level 2; that setting is irreversible
and disables ROM recovery.

When several DFU devices are connected, select the intended adapter with
`--dfu-serial SERIAL` or `--dfu-path BUS-PORT`; the utility refuses ambiguous
selection. Run `tools/flashdfu.sh --help` for all options.

The option-byte fallback and read-protection behavior are specified in
[RM0091, sections 2.5, 3.3 and 4](https://www.st.com/resource/en/reference_manual/DM00031936.pdf).
ROM readback, option programming and Read Unprotect are specified in
[AN3156](https://www.st.com/resource/en/application_note/an3156-how-to-use-usb-dfu-protocol-in-bootloader-on-stm32-mcus-stmicroelectronics.pdf).
