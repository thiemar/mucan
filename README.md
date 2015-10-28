## µCAN

### Upgrading firmware

Install [`dfu-util`](http://dfu-util.sourceforge.net), and ensure you have
version 4.8 or 4.9 of `gcc-arm-none-eabi`.

You will also need a functioning Python 2.7+ or 3.4+ environment, with
`pyserial` installed:

    pip install pyserial

Download the µCAN source code:

    git clone https://github.com/thiemar/mucan.git
    cd mucan
    make -j8

Attach the µCAN adapter via USB, identify the device name (typically
`/dev/ttyACM<x>`, `/dev/ttyUSB<x>`, or `/dev/tty.usbmodem<x>`), then run:

    cd tools
    ./flashdfu.sh /path/to/device

If the command fails, and the device is in DFU mode already, you can run the
`dfu-util` commands directly:

    dfu-util --device 0483:df11 -a 0 --dfuse-address 0x08000000 --download ../firmware/app.bin
    dfu-util --device 0483:df11 -a 1 --dfuse-address 0x1FFFF800:leave --download ../firmware/ob.bin

The second command will display a warning; this can be ignored. The µCAN
adapter should restart and be re-enumerated at this point.
