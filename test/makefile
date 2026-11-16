# Host-side unit tests.
#   test_usb:   usb/usb_cdc.c against a model of the STM32F0 USB peripheral
#   test_slcan: app/slcan.c against fake CAN and serial hooks
#   test_can:   app/can.c against a CAN mode-transition model
#   test_boot:  app/boot.c against flash option-byte and boot models
#   test_clock: inherited ROM and cold-boot clock configurations
# Usage: make -C test   (or `make test` from the top level)

HOSTCC ?= cc
PYTHON ?= python3
CROSS ?= arm-none-eabi-
CFLAGS = -std=c99 -g -O1 -Wall -Wextra -Werror -I../app -I../usb -I.
SANITIZE ?= -fsanitize=address,undefined -fno-omit-frame-pointer

all: run

build/test_usb: ../usb/usb_cdc.c ../usb/usb_cdc.h usb_sim.c usb_sim.h test_usb.c
	@mkdir -p build
	$(HOSTCC) $(CFLAGS) $(SANITIZE) -DUSB_SIM -o $@ ../usb/usb_cdc.c usb_sim.c test_usb.c

build/test_slcan: ../app/slcan.c ../app/slcan.h ../app/can_frame.h test_slcan.c
	@mkdir -p build
	$(HOSTCC) $(CFLAGS) $(SANITIZE) -o $@ ../app/slcan.c test_slcan.c

build/test_can: ../app/can.c ../app/can.h ../app/can_frame.h can_sim.c can_sim.h test_can.c ../sys/stm32f042x6.h
	@mkdir -p build
	$(HOSTCC) $(CFLAGS) $(SANITIZE) -DCAN_SIM -o $@ ../app/can.c can_sim.c test_can.c

build/test_boot: ../app/boot.c ../app/boot.h boot_sim.c boot_sim.h test_boot.c ../sys/stm32f042x6.h
	@mkdir -p build
	$(HOSTCC) $(CFLAGS) $(SANITIZE) -DBOOT_SIM -o $@ ../app/boot.c boot_sim.c test_boot.c

build/test_clock: ../sys/system_stm32f0xx.c ../sys/system_stm32f0xx.h ../sys/stm32f042x6.h ../app/stm32f0xx_hal_conf.h clock_sim.h test_clock.c
	@mkdir -p $(@D)
	$(HOSTCC) $(CFLAGS) $(SANITIZE) -I../sys -DSTM32F042x6 -DCLOCK_SIM -o $@ ../sys/system_stm32f0xx.c test_clock.c

run: build/test_usb build/test_slcan build/test_can build/test_boot build/test_clock
	./build/test_usb
	./build/test_slcan
	./build/test_can
	./build/test_boot
	./build/test_clock
	$(PYTHON) test_flashdfu.py
	$(PYTHON) test_stm32_dfu.py
	$(PYTHON) test_libusb_transport.py

test-firmware:
	$(PYTHON) test_linker.py --cross "$(CROSS)" --firmware ../firmware/app.elf

clean:
	rm -rf build

.PHONY: all run test-firmware clean
