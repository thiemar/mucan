##############################################################################
# GNU GCC ARM Embeded Toolchain base directories and binaries
##############################################################################
# GCC_BASE = /usr/local/opt/gcc-arm-none-eabi/

CC_BIN := $(shell dirname `which arm-none-eabi-gcc`)/
CC_LIB  := $(CC_BIN)../arm-none-eabi/lib/
CC_INC  := $(CC_BIN)../arm-none-eabi/include/
AS       := $(CC_BIN)arm-none-eabi-as
CC       := $(CC_BIN)arm-none-eabi-gcc
CPP      := $(CC_BIN)arm-none-eabi-g++
LD       := $(CC_BIN)arm-none-eabi-g++
OBJCOPY  := $(CC_BIN)arm-none-eabi-objcopy
SIZE     := $(CC_BIN)arm-none-eabi-size

PYTHON   := python


##############################################################################
# Custom options for cortex-m and cortex-r processors
##############################################################################
CORTEX_M0PLUS_CC_FLAGS  = -mthumb -mcpu=cortex-m0plus
CORTEX_M0PLUS_LIB_PATH  = $(CC_LIB)armv6-m
CORTEX_M0_CC_FLAGS      = -mthumb -mcpu=cortex-m0
CORTEX_M0_LIB_PATH      = $(CC_LIB)armv6-m
CORTEX_M1_CC_FLAGS      = -mthumb -mcpu=cortex-m1
CORTEX_M1_LIB_PATH      = $(CC_LIB)armv6-m
CORTEX_M3_CC_FLAGS      = -mthumb -mcpu=cortex-m3
CORTEX_M3_LIB_PATH      = $(CC_LIB)armv7-m
CORTEX_M4_NOFP_CC_FLAGS = -mthumb -mcpu=cortex-m4
CORTEX_M4_NOFP_LIB_PATH = $(CC_LIB)armv7e-m
CORTEX_M4_SWFP_CC_FLAGS = -mthumb -mcpu=cortex-m4 -mfloat-abi=softfp -mfpu=fpv4-sp-d16
CORTEX_M4_SWFP_LIB_PATH = $(CC_LIB)armv7e-m/softfp
CORTEX_M4_HWFP_CC_FLAGS = -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16
CORTEX_M4_HWFP_LIB_PATH = $(CC_LIB)armv7e-m/fpu
CORTEX_R4_NOFP_CC_FLAGS = -mthumb -march=armv7-r
CORTEX_R4_NOFP_LIB_PATH = $(CC_LIB)armv7-r/thumb
CORTEX_R4_SWFP_CC_FLAGS = -mthumb -march=armv7-r -mfloat-abi=softfp -mfloat-abi=softfp -mfpu=vfpv3-d16
CORTEX_R4_SWFP_LIB_PATH = $(CC_LIB)armv7-r/thumb/softfp
CORTEX_R4_HWFP_CC_FLAGS = -mthumb -march=armv7-r -mfloat-abi=softfp -mfloat-abi=hard -mfpu=vfpv3-d16
CORTEX_R4_HWFP_LIB_PATH = $(CC_LIB)armv7-r/thumb/fpu
CORTEX_R5_NOFP_CC_FLAGS = -mthumb -march=armv7-r
CORTEX_R5_NOFP_LIB_PATH = $(CC_LIB)armv7-r/thumb
CORTEX_R5_SWFP_CC_FLAGS = -mthumb -march=armv7-r -mfloat-abi=softfp -mfloat-abi=softfp -mfpu=vfpv3-d16
CORTEX_R5_SWFP_LIB_PATH = $(CC_LIB)armv7-r/thumb/softfp
CORTEX_R5_HWFP_CC_FLAGS = -mthumb -march=armv7-r -mfloat-abi=softfp -mfloat-abi=hard -mfpu=vfpv3-d16
CORTEX_R5_HWFP_LIB_PATH = $(CC_LIB)armv7-r/thumb/fpu


##############################################################################
# Main makefile project configuration
#    PROJECT      = <name of the target to be built>
#    MCU_CC_FLAGS = <one of the CC_FLAGS above>
#    MCU_LIB_PATH = <one of the LIB_PATH above>
#    OPTIMIZE_FOR = < SIZE or nothing >
#    DEBUG_LEVEL  = < -g compiler option or nothing >
#    OPTIM_LEVEL  = < -O compiler option or nothing >
##############################################################################
PROJECT           = app
BUILD             = build/
MCU_CC_FLAGS      = $(CORTEX_M0_CC_FLAGS)
MCU_LIB_PATH      = $(CORTEX_M0_LIB_PATH)
DEBUG_LEVEL       = 0
OPTIM_FLAGS       = -Os
LINKER_SCRIPT     = $(PROJECT).ld
PROJECT_OBJECTS   = $(addprefix $(BUILD), \
					  app/main.o app/stm32f0xx_it.o app/usb_device.o \
					  app/usbd_cdc_if.o app/usbd_conf.o app/usbd_desc.o \
					  app/vectors.o app/libc.o sys/stm32f0xx_hal.o \
					  sys/stm32f0xx_hal_can.o sys/stm32f0xx_hal_cortex.o \
					  sys/stm32f0xx_hal_dma.o sys/stm32f0xx_hal_flash.o \
					  sys/stm32f0xx_hal_flash.o sys/stm32f0xx_hal_flash_ex.o \
					  sys/stm32f0xx_hal_gpio.o sys/stm32f0xx_hal_pcd.o \
					  sys/stm32f0xx_hal_pcd_ex.o sys/stm32f0xx_hal_pwr.o \
					  sys/stm32f0xx_hal_pwr_ex.o sys/stm32f0xx_hal_rcc.o \
					  sys/stm32f0xx_hal_rcc_ex.o sys/system_stm32f0xx.o \
					  usb/usbd_cdc.o usb/usbd_core.o usb/usbd_ctlreq.o \
					  usb/usbd_ioreq.o)
PROJECT_INC_PATHS = -Iapp/ -Isys/ -Iusb/
PROJECT_LIB_PATHS = -L.
PROJECT_LIBRARIES =
PROJECT_SYMBOLS   = -DTOOLCHAIN_GCC_ARM -DNO_RELOC='0' -DSTM32F042x6
PROJECT_LD_FLAGS  = --specs=nosys.specs


###############################################################################
# Command line building
###############################################################################
ifdef DEBUG_LEVEL
CC_DEBUG_FLAGS = -g$(DEBUG_LEVEL)
CC_SYMBOLS = -DDEBUG $(PROJECT_SYMBOLS) $(UAVCAN_CONFIG)
else
CC_DEBUG_FLAGS =
CC_SYMBOLS = -DNODEBUG $(PROJECT_SYMBOLS) $(UAVCAN_CONFIG)
endif

INCLUDE_PATHS = $(PROJECT_INC_PATHS)
LIBRARY_PATHS = $(PROJECT_LIB_PATHS)
ARCH_CC_FLAGS = -ffreestanding -fno-common -fmessage-length=0 -Wall -fno-exceptions -ffunction-sections -fdata-sections -funsigned-char
CFLAGS = $(MCU_CC_FLAGS) -c $(OPTIM_FLAGS) $(CC_DEBUG_FLAGS) $(ARCH_CC_FLAGS)
CXXFLAGS = $(CFLAGS) -fno-rtti -fno-threadsafe-statics
LD_FLAGS = $(MCU_CC_FLAGS) $(OPTIM_FLAGS) -Wl,--gc-sections $(SYS_LD_FLAGS) -Wl,-Map=$(BUILD)$(PROJECT).map -ffreestanding -nostartfiles
LD_SYS_LIBS = $(SYS_LIBRARIES)


###############################################################################
# Makefile execution
###############################################################################

all: firmware/$(PROJECT).bin

clean:
	rm -f $(PROJECT_OBJECTS)
	rm -f "firmware/*.bin" "firmware/*.elf"

$(BUILD)%.o: %.S
	@mkdir -p $(@D)
	$(CC) $(CFLAGS) $(CC_SYMBOLS) -D__ASSEMBLY__ $(INCLUDE_PATHS) -o $@ $<

$(BUILD)%.o: %.c
	@mkdir -p $(@D)
	$(CC)  $(CFLAGS) $(CC_SYMBOLS) -std=gnu99 $(INCLUDE_PATHS) -o $@ $<

$(BUILD)%.o: %.cpp
	@mkdir -p $(@D)
	$(CPP) $(CXXFLAGS) $(CC_SYMBOLS) -std=c++11 $(INCLUDE_PATHS) -o $@ $<

firmware/$(PROJECT).elf: $(PROJECT_OBJECTS)
	@mkdir -p $(@D)
	$(LD) $(LD_FLAGS) -T$(LINKER_SCRIPT) $(LIBRARY_PATHS) -o $@ $^ $(PROJECT_LIBRARIES)

firmware/$(PROJECT).bin: firmware/$(PROJECT).elf
	@mkdir -p $(@D)
	$(SIZE) $<
	$(OBJCOPY) -O binary $< $@
