/* Start-up, DFU bootloader entry and watchdog. See boot.h. */

#ifdef BOOT_SIM
#include "boot_sim.h"
#else
#include "stm32f0xx_hal.h"
#endif

#include "boot.h"

/* STM32F04x system memory, holding the ROM USB DFU bootloader (AN2606) */
#define SYSTEM_MEMORY_BASE 0x1FFFC400u

#define BOOT_MAGIC 0xDF00B007u
#define OPTION_COUNT 5u /* F042 implements RDP, USER, DATA0, DATA1, WRP0 only. */
#define OPTION_USER 1u
#define OPTION_TIMEOUT_MS 1000u

/* Also used internally by the bundled HAL flash extension. */
extern HAL_StatusTypeDef FLASH_WaitForLastOperation(uint32_t timeout);

static bool software_boot_enabled(void) {
    uint32_t options = FLASH->OBR;
    uint8_t user = (uint8_t)(options >> 8);
    return !(options & (FLASH_OBR_OPTERR | FLASH_OBR_RDPRT2)) &&
           (user & (OB_BOOT_SEL_SET | OB_BOOT0_SET)) == OB_BOOT0_SET;
}

#define RESET_FLAGS                                                                   \
    (RCC_CSR_LPWRRSTF | RCC_CSR_WWDGRSTF | RCC_CSR_IWDGRSTF | RCC_CSR_SFTRSTF |       \
     RCC_CSR_PORRSTF | RCC_CSR_PINRSTF | RCC_CSR_OBLRSTF | RCC_CSR_V18PWRRSTF)

#define IWDG_KEY_RELOAD 0xAAAAu
#define IWDG_KEY_UNLOCK 0x5555u
#define IWDG_KEY_START 0xCCCCu
#define IWDG_PRESCALER_32 3u
#define IWDG_RELOAD 1249u /* LSI ~40 kHz / 32 / 1250 = ~1 s (0.8-1.3 s over LSI range) */

/* Survives a system reset: placed outside .data/.bss so start-up leaves it alone */
static volatile uint32_t boot_request[2]
#ifndef BOOT_SIM
    __attribute__((section(".noinit")))
#endif
    ;

static void __attribute__((noreturn)) jump_to_system_memory(void) {
#ifdef BOOT_SIM
    boot_sim_jump_to_rom();
#else
    uint32_t stack = *(const volatile uint32_t *)SYSTEM_MEMORY_BASE;
    uint32_t entry = *(const volatile uint32_t *)(SYSTEM_MEMORY_BASE + 4u);

    __asm volatile("msr msp, %0\n\tbx %1" : : "r"(stack), "r"(entry) : "memory");
    __builtin_unreachable();
#endif
}

uint32_t boot_early(void) {
    uint32_t reset_flags = RCC->CSR & RESET_FLAGS;

    RCC->CSR |= RCC_CSR_RMVF;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;

    bool requested = boot_request[0] == BOOT_MAGIC && boot_request[1] == ~BOOT_MAGIC;
    boot_request[0] = 0;
    boot_request[1] = 0;

    /* A stale SRAM flag after power-on must not select DFU. The F042 ROM
     * also requires software boot selection, independently of MEM_MODE. */
    if (requested && (reset_flags & RCC_CSR_SFTRSTF) &&
        !(reset_flags & RCC_CSR_PORRSTF) && software_boot_enabled()) {
        /* Cortex-M0 has no VTOR: map system memory at 0 so the bootloader's
         * own vector table is the one in use */
        SYSCFG->CFGR1 = (SYSCFG->CFGR1 & ~SYSCFG_CFGR1_MEM_MODE) | SYSCFG_CFGR1_MEM_MODE_0;
        __DSB();
        __ISB();
        jump_to_system_memory();
    }

    /* Main flash at 0, whatever started us -- after dfu-util's :leave the
     * ROM's mapping (system memory at 0) may still be in place */
    SYSCFG->CFGR1 &= ~SYSCFG_CFGR1_MEM_MODE;

    return reset_flags;
}

/* Option bytes contain one data byte and its hardware-generated complement
 * per halfword. Keep all five implemented pairs: an option erase clears
 * the entire bank, not just USER. RM0091 section 4.1.3 specifies that F04x
 * implements only WRP0; the WRP1/2/3 slots must not be accessed. */
static uint16_t option_read(unsigned index) {
#ifdef BOOT_SIM
    return boot_sim_get_option(index);
#else
    return ((const volatile uint16_t *)OB)[index];
#endif
}

static bool options_write(const uint16_t image[OPTION_COUNT]) {
    HAL_StatusTypeDef status;
    unsigned i;

    if (FLASH_WaitForLastOperation(OPTION_TIMEOUT_MS) != HAL_OK) {
        return false;
    }
    FLASH->CR |= FLASH_CR_OPTER;
    FLASH->CR |= FLASH_CR_STRT;
    status = FLASH_WaitForLastOperation(OPTION_TIMEOUT_MS);
    FLASH->CR &= ~FLASH_CR_OPTER;
    if (status != HAL_OK) {
        return false;
    }

    /* RDP is restored first, with exactly its previous byte value. In
     * particular, never downgrade protection, which can mass-erase flash. */
    for (i = 0; i < OPTION_COUNT; ++i) {
        if (image[i] == 0xFFFFu) {
            continue; /* Preserve an erased option pair as erased. */
        }
        FLASH->CR |= FLASH_CR_OPTPG;
#ifdef BOOT_SIM
        boot_sim_program_option(i, (uint8_t)image[i]);
#else
        ((volatile uint16_t *)OB)[i] = (uint8_t)image[i];
#endif
        status = FLASH_WaitForLastOperation(OPTION_TIMEOUT_MS);
        FLASH->CR &= ~FLASH_CR_OPTPG;
        if (status != HAL_OK || option_read(i) != image[i]) {
            return false;
        }
    }
    for (i = 0; i < OPTION_COUNT; ++i) {
        if (option_read(i) != image[i]) {
            return false;
        }
    }
    return true;
}

bool boot_repair_option_bytes(void) {
    uint16_t original[OPTION_COUNT];
    uint16_t updated[OPTION_COUNT];
    uint8_t user;
    unsigned i;

    /* RDP level 2 is irreversible and forbids option erasure. Invalid option
     * complements also need external repair, not an automatic rewrite. */
    if (FLASH->OBR & (FLASH_OBR_OPTERR | FLASH_OBR_RDPRT2)) {
        return false;
    }
    if (software_boot_enabled()) {
        return true;
    }
    for (i = 0; i < OPTION_COUNT; ++i) {
        uint16_t pair = option_read(i);
        if (pair != 0xFFFFu && (uint8_t)(pair >> 8) != (uint8_t)~pair) {
            return false;
        }
        original[i] = updated[i] = pair;
    }
    /* AN2606, STM32F04xxx: software ROM entry checks BOOT_SEL even after
     * remapping. BOOT_SEL=0, nBOOT0=1 boots flash normally and permits the
     * RAM-flag jump. It disables physical BOOT0 recovery; see boot.h. */
    user = ((uint8_t)original[OPTION_USER] & ~OB_BOOT_SEL_SET) | OB_BOOT0_SET;
    updated[OPTION_USER] = user | ((uint16_t)(uint8_t)~user << 8);

    if ((FLASH->CR & FLASH_CR_LOCK) && HAL_FLASH_Unlock() != HAL_OK) {
        return false;
    }
    if (!(FLASH->CR & FLASH_CR_OPTWRE) && HAL_FLASH_OB_Unlock() != HAL_OK) {
        HAL_FLASH_Lock();
        return false;
    }
    if (options_write(updated)) {
        /* Reload only a completely verified image. Normally does not return.
         * If reset/reload fails, report unavailable rather than claiming the
         * new boot configuration is already active. */
        (void)HAL_FLASH_OB_Launch();
    } else {
        /* A failed write may have erased unrelated bytes. Make one bounded
         * attempt to restore the complete original image; never reload a
         * partially programmed bank or advertise DFU after any failure. */
        (void)options_write(original);
    }
    HAL_FLASH_OB_Lock();
    HAL_FLASH_Lock();
    return false;
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
