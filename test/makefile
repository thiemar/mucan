# Host-side unit tests.
#   test_usb:   usb/usb_cdc.c against a model of the STM32F0 USB peripheral
#   test_slcan: app/slcan.c against fake CAN and serial hooks
# Usage: make -C test   (or `make test` from the top level)

HOSTCC ?= cc
CFLAGS = -std=c99 -g -O1 -Wall -Wextra -Werror -I../app -I../usb -I.
SANITIZE ?= -fsanitize=address,undefined -fno-omit-frame-pointer

all: run

build/test_usb: ../usb/usb_cdc.c ../usb/usb_cdc.h usb_sim.c usb_sim.h test_usb.c
	@mkdir -p build
	$(HOSTCC) $(CFLAGS) $(SANITIZE) -DUSB_SIM -o $@ ../usb/usb_cdc.c usb_sim.c test_usb.c

build/test_slcan: ../app/slcan.c ../app/slcan.h ../app/can_frame.h test_slcan.c
	@mkdir -p build
	$(HOSTCC) $(CFLAGS) $(SANITIZE) -o $@ ../app/slcan.c test_slcan.c

run: build/test_usb build/test_slcan
	./build/test_usb
	./build/test_slcan

clean:
	rm -rf build

.PHONY: all run clean
