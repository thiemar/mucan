# µCAN firmware for the STM32F042 (Cortex-M0, 32 KiB flash, 6 KiB RAM).
#
#   make                 build firmware/app.bin
#   make test            run the host-side unit tests (needs a host C compiler)
#   make test-firmware   build firmware and check its startup memory layout
#   make clean
#
# The arm-none-eabi toolchain must be on PATH, or set CROSS to its prefix,
# e.g. make CROSS=/opt/arm-gnu-toolchain/bin/arm-none-eabi-

CROSS   ?= arm-none-eabi-
CC      := $(CROSS)gcc
OBJCOPY := $(CROSS)objcopy
SIZE    := $(CROSS)size

PROJECT := app
BUILD   := build/

OBJECTS := $(addprefix $(BUILD), \
	app/main.o app/boot.o app/can.o app/clock.o app/slcan.o \
	app/stm32f0xx_it.o app/vectors.o app/libc.o \
	sys/stm32f0xx_hal.o sys/stm32f0xx_hal_cortex.o \
	sys/stm32f0xx_hal_dma.o sys/stm32f0xx_hal_flash.o \
	sys/stm32f0xx_hal_flash_ex.o sys/stm32f0xx_hal_gpio.o \
	sys/stm32f0xx_hal_pwr.o sys/stm32f0xx_hal_pwr_ex.o \
	sys/stm32f0xx_hal_rcc.o sys/stm32f0xx_hal_rcc_ex.o \
	sys/system_stm32f0xx.o \
	usb/usb_cdc.o)

ARCH    := -mthumb -mcpu=cortex-m0
CFLAGS  := $(ARCH) -Os -g -std=gnu99 -ffreestanding -fno-common \
	-ffunction-sections -fdata-sections -funsigned-char \
	-Wall -DSTM32F042x6 -Iapp -Isys -Iusb -MMD -MP
LDFLAGS := $(ARCH) -Os -nostartfiles -Wl,--gc-sections \
	-Wl,-Map=$(BUILD)$(PROJECT).map -T$(PROJECT).ld

all: firmware/$(PROJECT).bin

test:
	$(MAKE) -C test

test-firmware: firmware/$(PROJECT).bin
	$(MAKE) -C test test-firmware CROSS="$(CROSS)"

clean:
	rm -rf $(BUILD) firmware
	$(MAKE) -C test clean

.PHONY: all test test-firmware clean

$(BUILD)%.o: %.c
	@mkdir -p $(@D)
	$(CC) $(CFLAGS) -c -o $@ $<

firmware/$(PROJECT).elf: $(OBJECTS) $(PROJECT).ld
	@mkdir -p $(@D)
	$(CC) $(LDFLAGS) -o $@ $(OBJECTS)

firmware/$(PROJECT).bin: firmware/$(PROJECT).elf
	$(SIZE) $<
	$(OBJCOPY) -O binary $< $@

-include $(OBJECTS:.o=.d)
