/* Start-up, DFU bootloader entry and watchdog. See boot.h. */

#include "stm32f0xx_hal.h"

#include "boot.h"

/* STM32F04x system memory, holding the ROM USB DFU bootloader (AN2606) */
#define SYSTEM_MEMORY_BASE 0x1FFFC400u

#define BOOT_MAGIC 0xDF00B007u

#define RESET_FLAGS                                                                   \
    (RCC_CSR_LPWRRSTF | RCC_CSR_WWDGRSTF | RCC_CSR_IWDGRSTF | RCC_CSR_SFTRSTF |       \
     RCC_CSR_PORRSTF | RCC_CSR_PINRSTF | RCC_CSR_OBLRSTF | RCC_CSR_V18PWRRSTF)

#define IWDG_KEY_RELOAD 0xAAAAu
#define IWDG_KEY_UNLOCK 0x5555u
#define IWDG_KEY_START 0xCCCCu
#define IWDG_PRESCALER_32 3u
#define IWDG_RELOAD 1249u /* LSI ~40 kHz / 32 / 1250 = ~1 s (0.8-1.3 s over LSI range) */

/* Survives a system reset: placed outside .data/.bss so start-up leaves it alone */
static volatile uint32_t boot_request[2] __attribute__((section(".noinit")));

static void __attribute__((noreturn)) jump_to_system_memory(void) {
    uint32_t stack = *(const volatile uint32_t *)SYSTEM_MEMORY_BASE;
    uint32_t entry = *(const volatile uint32_t *)(SYSTEM_MEMORY_BASE + 4u);

    __asm volatile("msr msp, %0\n\tbx %1" : : "r"(stack), "r"(entry) : "memory");
    __builtin_unreachable();
}

uint32_t boot_early(void) {
    uint32_t reset_flags = RCC->CSR & RESET_FLAGS;

    RCC->CSR |= RCC_CSR_RMVF;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;

    if (boot_request[0] == BOOT_MAGIC && boot_request[1] == ~BOOT_MAGIC) {
        boot_request[0] = 0;
        boot_request[1] = 0;

        /* Cortex-M0 has no VTOR: map system memory at 0 so the bootloader's
         * own vector table is the one in use */
        SYSCFG->CFGR1 = (SYSCFG->CFGR1 & ~SYSCFG_CFGR1_MEM_MODE) | SYSCFG_CFGR1_MEM_MODE_0;
        jump_to_system_memory();
    }

    /* Main flash at 0, whatever started us -- after dfu-util's :leave the
     * ROM's mapping (system memory at 0) may still be in place */
    SYSCFG->CFGR1 &= ~SYSCFG_CFGR1_MEM_MODE;

    return reset_flags;
}

void boot_repair_option_bytes(void) {
    FLASH_OBProgramInitTypeDef ob;

    HAL_FLASHEx_OBGetConfig(&ob);

    /* Boot from the ROM regardless of the BOOT0 pin: BOOT_SEL = 0 (use the
     * nBOOT0 bit) with nBOOT0 = 0. Nothing else is touched. */
    if ((ob.USERConfig & (OB_BOOT_SEL_SET | OB_BOOT0_SET)) != 0u) {
        return;
    }

    ob.OptionType = OPTIONBYTE_USER;
    ob.USERConfig |= OB_BOOT_SEL_SET | OB_BOOT0_SET;

    HAL_FLASH_Unlock();
    HAL_FLASH_OB_Unlock();
    if (HAL_FLASHEx_OBErase() == HAL_OK && HAL_FLASHEx_OBProgram(&ob) == HAL_OK) {
        HAL_FLASH_OB_Launch(); /* reloads the option bytes with a reset */
    }
    HAL_FLASH_OB_Lock();
    HAL_FLASH_Lock();
}

void boot_enter_bootloader(void) {
    boot_request[0] = BOOT_MAGIC;
    boot_request[1] = ~BOOT_MAGIC;
    NVIC_SystemReset();
    for (;;) {
    }
}

void watchdog_start(void) {
    uint32_t start;

    /* Stop the watchdog while a debugger has the core halted */
    __DBGMCU_CLK_ENABLE();
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;

    IWDG->KR = IWDG_KEY_START; /* also starts the LSI oscillator */
    IWDG->KR = IWDG_KEY_UNLOCK;
    IWDG->PR = IWDG_PRESCALER_32;
    IWDG->RLR = IWDG_RELOAD;

    start = HAL_GetTick();
    while (IWDG->SR && (uint32_t)(HAL_GetTick() - start) < 10u) {
        /* Registers update in the LSI domain */
    }
    IWDG->KR = IWDG_KEY_RELOAD;
}

void watchdog_kick(void) {
    IWDG->KR = IWDG_KEY_RELOAD;
}
