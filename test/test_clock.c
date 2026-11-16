#include "clock_sim.h"

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

#include "clock.h"

RCC_TypeDef clock_sim_rcc;

/* ---- Fake HAL --------------------------------------------------------------
 * The RCC calls switch the modelled clock the way the hardware does, and
 * HAL_InitTick records the HCLK it derives the 1 ms SysTick from, exactly as
 * the real HAL_InitTick does via HAL_RCC_GetHCLKFreq(). */

static unsigned osc_calls, clock_calls, periph_calls, tick_calls;
static uint32_t tick_hclk; /* HCLK the current SysTick reload was computed for */

HAL_StatusTypeDef HAL_RCC_OscConfig(RCC_OscInitTypeDef *config) {
    assert(config->OscillatorType == RCC_OSCILLATORTYPE_HSI48);
    assert(config->HSI48State == RCC_HSI48_ON);
    assert(config->PLL.PLLState == RCC_PLL_NONE);
    osc_calls++;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_RCC_ClockConfig(RCC_ClkInitTypeDef *config, uint32_t latency) {
    assert(osc_calls > clock_calls); /* oscillator first */
    assert(config->SYSCLKSource == RCC_SYSCLKSOURCE_HSI48);
    assert(config->AHBCLKDivider == RCC_SYSCLK_DIV1);
    assert(config->APB1CLKDivider == RCC_HCLK_DIV1);
    assert(latency == FLASH_LATENCY_1);
    RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_SWS | RCC_CFGR_HPRE | RCC_CFGR_PPRE)) |
                RCC_CFGR_SWS_HSI48;
    clock_calls++;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef *config) {
    assert(config->PeriphClockSelection == RCC_PERIPHCLK_USB);
    assert(config->UsbClockSelection == RCC_USBCLKSOURCE_HSI48);
    periph_calls++;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_InitTick(uint32_t priority) {
    assert(priority == TICK_INT_PRIORITY);
    SystemCoreClockUpdate();
    tick_hclk = SystemCoreClock;
    tick_calls++;
    return HAL_OK;
}

/* ---- SystemCoreClockUpdate ---------------------------------------------- */

static void expect_clock(uint32_t cfgr, uint32_t cfgr2, uint32_t expected) {
    RCC->CFGR = cfgr;
    RCC->CFGR2 = cfgr2;
    SystemCoreClock = 123u; /* Ensure a previous/default value is not reused. */
    SystemCoreClockUpdate();
    assert(SystemCoreClock == expected);
    assert(RCC->CFGR == cfgr);
    assert(RCC->CFGR2 == cfgr2);
}

static void test_inherited_clocks(void) {
    static const uint16_t divisors[16] = {
        1, 1, 1, 1, 1, 1, 1, 1, 2, 4, 8, 16, 64, 128, 256, 512
    };
    unsigned encoding;

    /* ROM DFU can leave HSI48 selected. The 1 ms HAL tick must be derived
     * from its real HCLK, including any inherited AHB prescaler. */
    for (encoding = 0; encoding < 16; ++encoding) {
        expect_clock(RCC_CFGR_SWS_HSI48 | (encoding << 4), 0,
                     48000000u / divisors[encoding]);
    }
    expect_clock(RCC_CFGR_SWS_HSI, 0, 8000000u);
    expect_clock(RCC_CFGR_SWS_HSE | RCC_CFGR_HPRE_DIV2, 0, 4000000u);
    expect_clock(RCC_CFGR_SWS_PLL | RCC_CFGR_PLLSRC_HSI48_PREDIV | RCC_CFGR_PLLMUL2,
                 RCC_CFGR2_PREDIV_DIV2, 48000000u);

    /* Reading the clock never reconfigures it */
    assert(!osc_calls && !clock_calls && !periph_calls && !tick_calls);
}

/* ---- Start-up sequence -------------------------------------------------- */

/* HAL_Init() followed by clock_init(), from the clock state left by `cfgr` */
static void start_up(uint32_t cfgr) {
    osc_calls = clock_calls = periph_calls = tick_calls = 0;
    RCC->CFGR = cfgr;
    RCC->CFGR2 = 0;
    RCC->APB2ENR = 0;

    HAL_InitTick(TICK_INT_PRIORITY); /* as HAL_Init() does */
    clock_init();

    assert(osc_calls == 1 && clock_calls == 1 && periph_calls == 1);
    assert(tick_calls == 2);
    assert(tick_hclk == 48000000u); /* 1 ms is 1 ms at the running clock */
    assert(RCC->APB2ENR & RCC_APB2ENR_SYSCFGCOMPEN);
}

static void test_tick_follows_clock_switch(void) {
    /* Power-on reset: HAL_Init() sees the 8 MHz HSI */
    start_up(RCC_CFGR_SWS_HSI);
    /* Entered from the ROM bootloader with HSI48 already running */
    start_up(RCC_CFGR_SWS_HSI48);
    /* ...or with an inherited AHB prescaler */
    start_up(RCC_CFGR_SWS_HSI48 | RCC_CFGR_HPRE_DIV2);
}

int main(void) {
    test_inherited_clocks();
    test_tick_follows_clock_switch();
    puts("clock: 19 inherited-clock cases and 3 start-up sequences passed");
    return 0;
}
