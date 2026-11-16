/* Start-up, explicit boot-mode selection and watchdog for STM32F042.
 *
 * These boards have PF11/BOOT0 tied high. USER=0xFF selects ROM persistently;
 * USER=0x7F selects application flash. Startup NEVER changes option bytes.
 * The host may run a development image with DFU :leave while keeping ROM
 * selected. Production selects flash only after readback and a successful
 * application connection. Only explicit commands change the boot mode.
 */
#ifndef BOOT_H
#define BOOT_H

#include <stdbool.h>
#include <stdint.h>

#define BOOT_OPTION_COUNT 5u /* RDP, USER, DATA0, DATA1, WRP0; no reserved slots. */

/* First in __start, before .data/.bss. Quiesce ROM peripherals/interrupts,
 * map flash vectors and return reset flags. Leaves CPU interrupts masked;
 * the caller enables them after initializing RAM and the HAL tick. */
uint32_t boot_early(void);

/* Read-only checks/reporting: safe on every application startup. */
bool boot_dfu_available(void);
void boot_read_options(uint16_t options[BOOT_OPTION_COUNT]);

/* Explicit command only. Prepare and verify persistent ROM (false) or flash
 * (true) selection, leaving active options unchanged until boot_apply_mode.
 * Returns false on failure; the caller must reject the command and continue
 * running the current application. Preserves valid option pairs, including
 * noncanonical RDP1 values, DATA and WRP. Invalid complements are recovered
 * as their effective erased/0xFF settings; their lost original values cannot
 * be reconstructed. Never downgrades active RDP1 to RDP0 (that erases flash).
 * Flash selection requires the effective 0xFF/0x7F profile and valid RDP0;
 * an invalid USER pair defaults to 0xFF and can be repaired at this point.
 * ROM recovery can retain RDP1 for host Read Unprotect/reflash after
 * hardware ROM boot.
 *
 * With BOOT0 high, erased/invalid USER defaults to ROM. Keep application
 * flash intact until ROM selection is confirmed, and verify an image before
 * selecting flash. This does not recover a completely nonfunctional app.
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
