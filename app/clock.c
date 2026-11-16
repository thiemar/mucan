/*
 * System clock bring-up. See clock.h.
 * Based on the STM32CubeMX SystemClock_Config() template (STMicroelectronics).
 */

#ifdef CLOCK_SIM
#include "clock_sim.h"
#else
#include "stm32f0xx_hal.h"
#endif

#include "clock.h"

void clock_init(void) {
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};
    RCC_PeriphCLKInitTypeDef periph = {0};

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
    osc.HSI48State = RCC_HSI48_ON;
    osc.PLL.PLLState = RCC_PLL_NONE;
    HAL_RCC_OscConfig(&osc);

    clk.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_1);

    periph.PeriphClockSelection = RCC_PERIPHCLK_USB;
    periph.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
    HAL_RCCEx_PeriphCLKConfig(&periph);

    __SYSCFG_CLK_ENABLE();

    /*
     * HAL_Init() set SysTick from whatever clock was running then: the 8 MHz
     * HSI after a power-on reset, or HSI48 when the ROM bootloader started
     * us. HAL_RCC_ClockConfig() does not retune it, so without this every
     * HAL millisecond -- flash, CAN and protocol timeouts included -- would
     * last 1/6 ms after a cold boot.
     */
    HAL_InitTick(TICK_INT_PRIORITY);
}
