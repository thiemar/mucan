/* Start-up, explicit boot-mode selection and watchdog for STM32F042.
 *
 * These boards tie PF11/BOOT0 low (to the ground plane, like every unused
 * pin), so the pin can never select the ROM bootloader. Boot mode is chosen
 * by the option bits alone (BOOT_SEL=0): USER=0x77 (nBOOT0=0) starts the ROM
 * DFU bootloader on every reset; USER=0x7F (nBOOT0=1) starts application
 * flash. Factory USER=0xFF lets the pin decide, which here means flash (or
 * ROM while flash is still blank). Startup NEVER changes option bytes.
 * The host selects 0x77 through ROM DFU before writing flash, and may run a
 * development image with DFU :leave while ROM stays selected. Production
 * selects flash only after readback and a successful application
 * connection. Only explicit commands change the boot mode.
 */
#ifndef BOOT_H
#define BOOT_H

#include <stdbool.h>
#include <stdint.h>

#define BOOT_OPTION_COUNT 5u /* RDP, USER, DATA0, DATA1, WRP0; no reserved slots. */

/* First in __start, before .data/.bss. Quiesce ROM peripherals/interrupts
 * and map flash vectors. Leaves CPU interrupts masked; the caller enables
 * them after initializing RAM and the HAL tick. */
void boot_early(void);

/* Read-only checks/reporting: safe on every application startup.
 * boot_dfu_available(): the option bank can be rewritten safely. */
bool boot_dfu_available(void);
void boot_read_options(uint16_t options[BOOT_OPTION_COUNT]);

/* Explicit command only. Prepare and verify persistent ROM (false) or flash
 * (true) selection, leaving active options unchanged until boot_apply_mode.
 * Returns false on failure; the caller must reject the command and continue
 * running the current application. Preserves valid option pairs, including
 * noncanonical RDP1 values, DATA and WRP. Invalid complements are recovered
 * as their effective erased/0xFF settings; their lost original values cannot
 * be reconstructed. Never downgrades active RDP1 to RDP0 (that erases flash).
 * Flash selection requires the effective 0x77/0x7F/0xFF profile and valid
 * RDP0; an invalid USER pair defaults to 0xFF and can be repaired here.
 *
 * Switching between 0x77 and 0x7F erases the option bank and rewrites it,
 * RDP first and USER last. Power loss during the erase leaves RDP erased
 * (level 1) and USER erased (0xFF: flash, since BOOT0 is low), so the
 * existing application must still be able to run __dfu; level 1 is then
 * cleared from ROM with Read Unprotect, at the cost of the application. An
 * erased USER pair is programmed in place without another erase.
 *
 * Keep application flash intact until ROM selection is confirmed, and
 * verify an image before selecting flash. With BOOT0 tied low there is no
 * hardware fallback: an application that cannot run __dfu cannot be
 * replaced once flash is selected.
 */
bool boot_prepare_mode(bool flash_boot);

/* After a successful prepare and after ACK flush/USB disconnect: reload
 * options and reset. An unexpected reload failure falls back to a normal
 * reset; the verified stored mode then takes effect on a power cycle. */
void boot_apply_mode(void) __attribute__((noreturn));

/* Independent watchdog, about 1 s; must be kicked from the main loop. */
void watchdog_start(void);
void watchdog_kick(void);

#endif
