/*
 * Start-up, DFU bootloader entry and watchdog.
 *
 * DFU entry no longer touches option bytes. The application leaves a magic
 * value in RAM that survives a reset and resets; the very first thing the
 * next boot does is spot it and jump to the ROM bootloader. A power cycle or
 * dfu-util's :leave therefore always returns to the application.
 */

#ifndef BOOT_H
#define BOOT_H

#include <stdint.h>

/*
 * Call first in __start, before .data/.bss are set up. Jumps to the ROM
 * bootloader if one was requested, maps our vector table at address 0 and
 * returns the reset flags (0 means we were entered by a jump, not a reset).
 */
uint32_t boot_early(void);

/*
 * Firmware before this change entered DFU by programming the option bytes to
 * boot the ROM bootloader. If a device still has that setting, restore normal
 * boot (resets the device if anything was changed).
 */
void boot_repair_option_bytes(void);

/* Reset into the ROM DFU bootloader; does not return */
void boot_enter_bootloader(void) __attribute__((noreturn));

/* Independent watchdog, about 1 s; must be kicked from the main loop */
void watchdog_start(void);
void watchdog_kick(void);

#endif /* BOOT_H */
