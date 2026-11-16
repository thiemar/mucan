/*
 * Start-up, DFU bootloader entry and watchdog.
 *
 * After the one-time boot-option migration, DFU entry does not rewrite them.
 * The application leaves a magic value in RAM that survives a reset and
 * resets; the very first thing the
 * next boot does is spot it and jump to the ROM bootloader. A power cycle or
 * dfu-util's :leave therefore always returns to the application.
 */

#ifndef BOOT_H
#define BOOT_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Call first in __start, before .data/.bss are set up. Jumps to the ROM
 * bootloader if one was requested, maps our vector table at address 0 and
 * returns the reset flags (0 means we were entered by a jump, not a reset).
 */
uint32_t boot_early(void);

/*
 * One-time STM32F042 migration to BOOT_SEL=0, nBOOT0=1. This is required
 * for software entry to the ROM even when system memory has been remapped
 * (AN2606, STM32F04xxx). Preserve the other option bytes and verify writes.
 * A successful migration reloads the options and resets. Returns true only
 * if software DFU is already enabled; false means DFU must be rejected.
 *
 * This setting disables the physical BOOT0 pin as a boot selector. Recovery
 * of a damaged application requires SWD; BOOT0 alone cannot recover it.
 * Run before the application starts its watchdog. Do not cut power during
 * the one-time option-bank erase/program operation.
 */
bool boot_repair_option_bytes(void);

/* Reset into the ROM DFU bootloader; does not return */
void boot_enter_bootloader(void) __attribute__((noreturn));

/* Independent watchdog, about 1 s; must be kicked from the main loop */
void watchdog_start(void);
void watchdog_kick(void);

#endif /* BOOT_H */
