/* System clock bring-up: 48 MHz HSI48 for the core, peripherals and USB. */

#ifndef CLOCK_H
#define CLOCK_H

/*
 * Call after HAL_Init() with interrupts enabled (the RCC calls use HAL_GetTick
 * timeouts). Switches SYSCLK, HCLK and PCLK to HSI48, selects HSI48 for USB,
 * and re-derives the 1 ms SysTick from the new HCLK.
 */
void clock_init(void);

#endif /* CLOCK_H */
