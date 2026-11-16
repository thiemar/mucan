#include "clock_sim.h"

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

RCC_TypeDef clock_sim_rcc;

/* This test reads an inherited RCC configuration. It must never configure
 * oscillators or call SystemInit's hardware initialization path. */
HAL_StatusTypeDef HAL_RCC_OscConfig(RCC_OscInitTypeDef *config) {
    (void)config;
    abort();
}
HAL_StatusTypeDef HAL_RCC_ClockConfig(RCC_ClkInitTypeDef *config, uint32_t latency) {
    (void)config;
    (void)latency;
    abort();
}

static void expect_clock(uint32_t cfgr, uint32_t cfgr2, uint32_t expected) {
    RCC->CFGR = cfgr;
    RCC->CFGR2 = cfgr2;
    SystemCoreClock = 123u; /* Ensure a previous/default value is not reused. */
    SystemCoreClockUpdate();
    assert(SystemCoreClock == expected);
    assert(RCC->CFGR == cfgr);
    assert(RCC->CFGR2 == cfgr2);
}

int main(void) {
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
    puts("clock: 19 inherited-clock cases passed");
    return 0;
}
