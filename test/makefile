# Host-side tests for the USB driver, run against a model of the peripheral.
# Usage: make -C test   (or `make test` from the top level)

HOSTCC ?= cc
CFLAGS = -std=c99 -g -O1 -Wall -Wextra -Werror -DUSB_SIM -I../usb -I.
SANITIZE ?= -fsanitize=address,undefined -fno-omit-frame-pointer

SOURCES = ../usb/usb_cdc.c usb_sim.c test_usb.c
HEADERS = ../usb/usb_cdc.h usb_sim.h

all: run

build/test_usb: $(SOURCES) $(HEADERS)
	@mkdir -p build
	$(HOSTCC) $(CFLAGS) $(SANITIZE) -o $@ $(SOURCES)

run: build/test_usb
	./build/test_usb

clean:
	rm -rf build

.PHONY: all run clean
