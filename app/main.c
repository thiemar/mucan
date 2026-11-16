/**
  ******************************************************************************
  * File Name          : main.c
  * Date               : 04/05/2015 18:45:31
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/*
 * µCAN: USB CDC-ACM to CAN adapter speaking SLCAN.
 *
 * Everything runs from one main loop. The only interrupts are SysTick (time
 * base) and CAN receive, which only fills a frame ring; USB is polled.
 *
 *   USB OUT -> slcan_service() -> can_transmit() -> software queue -> mailboxes
 *   CAN RX ISR -> frame ring -> can_receive() -> slcan_format_frame() -> USB IN
 */

#include <stdbool.h>

#include "stm32f0xx_hal.h"

#include "boot.h"
#include "can.h"
#include "clock.h"
#include "slcan.h"
#include "usb_cdc.h"

/* Section boundaries from app.ld */
extern volatile uint32_t _eronly; /* .data initialisers in flash */
extern volatile uint32_t _sdata;
extern volatile uint32_t _edata;
extern volatile uint32_t _sbss;
extern volatile uint32_t _ebss;

/* 96-bit unique device ID (RM0091, "Device electronic signature") */
#define DEVICE_UID ((const volatile uint32_t *)0x1FFFF7ACu)

#define BOOTLOADER_FLUSH_MS 100u  /* time for the host to collect the reply */
#define BOOTLOADER_DETACH_MS 20u  /* off the bus before resetting */

static slcan_t slcan;
static bool clock_synced;
static bool boot_mode_requested;

void __start(void) __attribute__((noreturn));

void HAL_MspInit(void) {
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ---- SLCAN device hooks ------------------------------------------------ */

bool slcan_port_set_bitrate(uint32_t bitrate) {
    return can_set_bitrate(bitrate);
}

bool slcan_port_open(bool listen_only) {
    /* HSI48 is only accurate enough for CAN once CRS has trimmed it against
     * the USB start-of-frame packets */
    return clock_synced && can_open(listen_only);
}

void slcan_port_close(void) {
    can_close();
}

bool slcan_port_transmit(const can_frame_t *frame) {
    return can_transmit(frame);
}

uint8_t slcan_port_status(void) {
    return can_status();
}

uint16_t slcan_port_serial(void) {
    uint32_t folded = DEVICE_UID[0] ^ DEVICE_UID[1] ^ DEVICE_UID[2];
    return (uint16_t)(folded ^ (folded >> 16));
}

bool slcan_port_boot_mode(bool flash_boot) {
    if (boot_mode_requested || !boot_prepare_mode(flash_boot)) {
        return false;
    }
    boot_mode_requested = true;
    return true;
}

void slcan_port_options(uint8_t bytes[SLCAN_OPTION_BYTES]) {
    uint16_t pairs[SLCAN_OPTION_BYTES / 2u];
    boot_read_options(pairs);
    for (unsigned i = 0; i < SLCAN_OPTION_BYTES / 2u; ++i) {
        bytes[2u * i] = (uint8_t)pairs[i];
        bytes[2u * i + 1u] = (uint8_t)(pairs[i] >> 8);
    }
}

int slcan_port_getc(void) {
    return usb_cdc_getc();
}

uint16_t slcan_port_write_space(void) {
    return usb_cdc_write_space();
}

bool slcan_port_write(const uint8_t *data, uint16_t len) {
    return usb_cdc_write(data, len);
}

/* ------------------------------------------------------------------------ */

/* Close the channel and start the command stream afresh, discarding any
 * input the host sent before it went away */
static void session_reset(void) {
    slcan_reset(&slcan);
    while (usb_cdc_getc() >= 0) {
    }
}

static void crs_start(void) {
    RCC_CRSInitTypeDef crs;

    /* Trim HSI48 continuously against the 1 kHz USB start-of-frame */
    __CRS_CLK_ENABLE();
    crs.Prescaler = RCC_CRS_SYNC_DIV1;
    crs.Source = RCC_CRS_SYNC_SOURCE_USB;
    crs.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
    crs.ReloadValue = RCC_CRS_RELOADVALUE_DEFAULT;
    crs.ErrorLimitValue = RCC_CRS_ERRORLIMIT_DEFAULT;
    crs.HSI48CalibrationValue = RCC_CRS_HSI48CALIBRATION_DEFAULT;
    HAL_RCCEx_CRSConfig(&crs);
}

static void __attribute__((noreturn)) apply_boot_mode(void) {
    uint32_t start = HAL_GetTick();

    /* Let the host collect the boot-command reply, then detach cleanly. */
    while (!usb_cdc_tx_empty() && (uint32_t)(HAL_GetTick() - start) < BOOTLOADER_FLUSH_MS) {
        watchdog_kick();
        usb_poll();
    }

    slcan_reset(&slcan);
    usb_disconnect();

    start = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - start) < BOOTLOADER_DETACH_MS) {
        watchdog_kick();
    }

    boot_apply_mode();
}

void __start(void) {
    const volatile uint32_t *src;
    volatile uint32_t *dest;
    bool session = false;
    bool dtr = false;

    boot_early();

    /* Set up .data and .bss */
    for (src = &_eronly, dest = &_sdata; dest < &_edata;) {
        *dest++ = *src++;
    }
    for (dest = &_sbss; dest < &_ebss;) {
        *dest++ = 0;
    }

    /* HAL's tick starts from whatever clock is running (HSI after reset, or
     * HSI48 if the ROM bootloader started us); clock_init() then switches to
     * HSI48 and retunes the tick for it. */
    SystemCoreClockUpdate();
    HAL_Init();
    __enable_irq();
    clock_init();

    watchdog_start();

    /* boot_early() detached ROM USB. Give the host time to observe that
     * disconnect before announcing the application, without resetting back
     * to ROM or modifying the persistent boot selection. */
    HAL_Delay(BOOTLOADER_DETACH_MS);

    __GPIOA_CLK_ENABLE();
    usb_init();
    can_init();
    crs_start();
    slcan_init(&slcan);

    for (;;) {
        can_frame_t frame;
        bool dtr_now;

        watchdog_kick();
        usb_poll();
        can_poll();

        if (!clock_synced && (CRS->ISR & CRS_ISR_SYNCOKF)) {
            clock_synced = true;
        }

        if (!usb_cdc_configured()) {
            /* Unplugged, reset or deconfigured by the host: leave the bus */
            if (session) {
                slcan_reset(&slcan);
                session = false;
            }
            continue;
        }

        if (!session) {
            session = true;
            dtr = usb_cdc_dtr();
        }

        /* Host closed the port (DTR dropped): leave the bus and start over.
         * Hosts that never raise DTR are unaffected. */
        dtr_now = usb_cdc_dtr();
        if (dtr && !dtr_now) {
            session_reset();
        }
        dtr = dtr_now;

        slcan_service(&slcan, HAL_GetTick());

        if (boot_mode_requested) {
            apply_boot_mode();
        }

        /* Received frames go to the host while there's room for a whole line
         * plus a command reply; otherwise they wait in the CAN ring */
        while (usb_cdc_write_space() >= SLCAN_MAX_FRAME_LINE + SLCAN_MAX_REPLY &&
               can_receive(&frame)) {
            uint8_t line[SLCAN_MAX_FRAME_LINE];
            (void)usb_cdc_write(line, slcan_format_frame(&frame, line));
        }
    }
}



#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
