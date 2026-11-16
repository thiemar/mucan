/* Start-up, explicit boot-mode selection and watchdog. See boot.h. */
#ifdef BOOT_SIM
#include "boot_sim.h"
#else
#include "stm32f0xx_hal.h"
#endif
#include "boot.h"

#define OPTION_RDP 0u
#define OPTION_USER 1u
#define OPTION_TIMEOUT_MS 100u
#define USER_BOOT_SEL 0x80u
#define USER_NBOOT1 0x10u
#define USER_NBOOT0 0x08u
#define RESET_FLAGS \
    (RCC_CSR_LPWRRSTF | RCC_CSR_WWDGRSTF | RCC_CSR_IWDGRSTF | RCC_CSR_SFTRSTF | \
     RCC_CSR_PORRSTF | RCC_CSR_PINRSTF | RCC_CSR_OBLRSTF | RCC_CSR_V18PWRRSTF)

#define IWDG_KEY_RELOAD 0xAAAAu
#define IWDG_KEY_UNLOCK 0x5555u
#define IWDG_KEY_START 0xCCCCu
#define IWDG_PRESCALER_32 3u
#define IWDG_RELOAD 1249u /* LSI ~40 kHz / 32 / 1250 = ~1 s (0.8-1.3 s over LSI range) */

/* Also used internally by the bundled HAL flash extension. */
extern HAL_StatusTypeDef FLASH_WaitForLastOperation(uint32_t timeout);

uint32_t boot_early(void) {
    uint32_t reset_flags = RCC->CSR & RESET_FLAGS;
    const uint32_t apb1 = RCC_APB1RSTR_USBRST | RCC_APB1RSTR_CRSRST |
                          RCC_APB1RSTR_I2C1RST | RCC_APB1RSTR_USART2RST;
    const uint32_t gpio = RCC_AHBRSTR_GPIOARST | RCC_AHBRSTR_GPIOBRST |
                          RCC_AHBRSTR_GPIOFRST;

    /* DFU :leave can jump here with ROM vectors, SysTick and peripheral
     * interrupts still active. Stop them before replacing vectors or RAM. */
    __disable_irq();
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;
    NVIC->ICER[0] = 0xFFFFFFFFu;
    RCC->APB1RSTR |= apb1;
    RCC->APB1RSTR &= ~apb1;
    RCC->APB2RSTR |= RCC_APB2RSTR_USART1RST;
    RCC->APB2RSTR &= ~RCC_APB2RSTR_USART1RST;
    RCC->AHBRSTR |= gpio;
    RCC->AHBRSTR &= ~gpio;
    NVIC->ICPR[0] = 0xFFFFFFFFu;
    SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk | SCB_ICSR_PENDSVCLR_Msk;
    RCC->CSR |= RCC_CSR_RMVF;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
    SYSCFG->CFGR1 &= ~SYSCFG_CFGR1_MEM_MODE;
    __DSB();
    __ISB();
    return reset_flags;
}

/* RM0091 4.1.3: F042 implements only these five halfwords, not WRP1/2/3. */
static uint16_t option_read(unsigned index) {
#ifdef BOOT_SIM
    return boot_sim_get_option(index);
#else
    return ((const volatile uint16_t *)OB)[index];
#endif
}

void boot_read_options(uint16_t options[BOOT_OPTION_COUNT]) {
    for (unsigned i = 0; i < BOOT_OPTION_COUNT; ++i) options[i] = option_read(i);
}

static bool option_valid(uint16_t pair) {
    return pair == 0xFFFFu || (uint8_t)(pair >> 8) == (uint8_t)~pair;
}

static uint16_t option_pair(uint8_t value) {
    return value | ((uint16_t)(uint8_t)~value << 8);
}

static bool option_snapshot(uint16_t image[BOOT_OPTION_COUNT]) {
    if (FLASH->OBR & FLASH_OBR_RDPRT2) return false;
    for (unsigned i = 0; i < BOOT_OPTION_COUNT; ++i) {
        uint16_t pair = option_read(i);
        /* Invalid complements load as FF; preserve that effective setting.
         * In particular an invalid AA/nRDP pair is RDP1, NEVER RDP0. */
        image[i] = option_valid(pair) ? pair : 0xFFFFu;
    }
    if (image[OPTION_RDP] == option_pair(0xCC)) return false;
    /* Do not program AA when active protection is level 1: that mass-erases
     * the running application, even if raw bytes already contain AA. */
    if ((FLASH->OBR & FLASH_OBR_RDPRT1) && image[OPTION_RDP] == option_pair(0xAA)) {
        return false;
    }
    return true;
}

bool boot_dfu_available(void) {
    uint16_t image[BOOT_OPTION_COUNT];
    return option_snapshot(image);
}

static bool options_unlock(void) {
    if ((FLASH->CR & FLASH_CR_LOCK) && HAL_FLASH_Unlock() != HAL_OK) return false;
    if (!(FLASH->CR & FLASH_CR_OPTWRE) && HAL_FLASH_OB_Unlock() != HAL_OK) {
        HAL_FLASH_Lock();
        return false;
    }
    return true;
}

static bool option_program(unsigned index, uint16_t pair) {
    if (pair == 0xFFFFu) return option_read(index) == pair;
    watchdog_kick();
    FLASH->CR |= FLASH_CR_OPTPG;
#ifdef BOOT_SIM
    boot_sim_program_option(index, (uint8_t)pair);
#else
    ((volatile uint16_t *)OB)[index] = (uint8_t)pair;
#endif
    HAL_StatusTypeDef status = FLASH_WaitForLastOperation(OPTION_TIMEOUT_MS);
    FLASH->CR &= ~FLASH_CR_OPTPG;
    return status == HAL_OK && option_read(index) == pair;
}

static bool options_write(const uint16_t image[BOOT_OPTION_COUNT]) {
    watchdog_kick();
    if (FLASH_WaitForLastOperation(OPTION_TIMEOUT_MS) != HAL_OK) return false;
    FLASH->CR |= FLASH_CR_OPTER;
    FLASH->CR |= FLASH_CR_STRT;
    HAL_StatusTypeDef status = FLASH_WaitForLastOperation(OPTION_TIMEOUT_MS);
    FLASH->CR &= ~FLASH_CR_OPTER;
    if (status != HAL_OK) return false;
    for (unsigned i = 0; i < BOOT_OPTION_COUNT; ++i) {
        if (option_read(i) != 0xFFFFu) return false;
    }
    /* USER is the commit: restore and verify RDP/DATA/WRP first. Never select
     * flash while any preceding setting is still being programmed. */
    for (unsigned i = 0; i < BOOT_OPTION_COUNT; ++i) {
        if (i != OPTION_USER && !option_program(i, image[i])) return false;
    }
    if (!option_program(OPTION_USER, image[OPTION_USER])) return false;
    for (unsigned i = 0; i < BOOT_OPTION_COUNT; ++i) {
        if (option_read(i) != image[i]) return false;
    }
    return true;
}

bool boot_prepare_mode(bool flash_boot) {
    uint16_t original[BOOT_OPTION_COUNT], updated[BOOT_OPTION_COUNT];
    if (!option_snapshot(original)) return false;
    if (flash_boot && ((FLASH->OBR & FLASH_OBR_RDPRT1) ||
                      original[OPTION_RDP] != option_pair(0xAA) ||
                      (original[OPTION_USER] != option_pair(0xFF) &&
                       original[OPTION_USER] != 0xFFFFu &&
                       original[OPTION_USER] != option_pair(0x7F)))) {
        return false;
    }
    /* OPTERR alone is recoverable: invalid USER/DATA/WRP pairs load as FF.
     * A verified application may commit flash after an interrupted option
     * write when RDP is still valid AA; snapshot preserves those effective
     * FF settings. Invalid/protected RDP is excluded by the checks above. */
    bool unchanged = true;
    for (unsigned i = 0; i < BOOT_OPTION_COUNT; ++i) updated[i] = original[i];
    uint8_t user = (uint8_t)original[OPTION_USER] | USER_NBOOT1;
    user = flash_boot ? (user & ~USER_BOOT_SEL) | USER_NBOOT0 : user | USER_BOOT_SEL;
    updated[OPTION_USER] = option_pair(user);
    for (unsigned i = 0; i < BOOT_OPTION_COUNT; ++i) {
        if (option_read(i) != updated[i]) unchanged = false;
    }
    if (unchanged) return true;
    if (!options_unlock()) return false;
    bool success = options_write(updated);
    if (!success) {
        /* Application flash is still intact. One bounded attempt restores
         * its original effective options; no reload or ACK on failure. */
        (void)options_write(original);
    }
    HAL_FLASH_OB_Lock();
    HAL_FLASH_Lock();
    return success;
}

void boot_apply_mode(void) {
    watchdog_kick();
    if (options_unlock()) (void)HAL_FLASH_OB_Launch();
    /* OBL_LAUNCH normally resets immediately. If it unexpectedly returns,
     * reboot the still-valid application; stored options survive for POR. */
    HAL_FLASH_OB_Lock();
    HAL_FLASH_Lock();
    NVIC_SystemReset();
    for (;;) {}
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
