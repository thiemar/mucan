/* Five implemented byte/complement pairs. Whole-bank erase, USER-last writes,
 * latched OBR, invalid-pair fallback and interruptible flash operations. */
#include <assert.h>
#include <stdbool.h>
#include <string.h>
#include "boot_sim.h"

RCC_TypeDef boot_sim_rcc;
SYSCFG_TypeDef boot_sim_syscfg;
FLASH_TypeDef boot_sim_flash;
boot_sim_options_t boot_sim_ob;
DBGMCU_TypeDef boot_sim_dbgmcu;
IWDG_TypeDef boot_sim_iwdg;
GPIO_TypeDef boot_sim_gpiof;
boot_sim_systick_t boot_sim_systick;
boot_sim_nvic_t boot_sim_nvic;
boot_sim_scb_t boot_sim_scb;
bool boot_sim_irq_disabled;
jmp_buf boot_sim_escape;
unsigned boot_sim_erases, boot_sim_programs, boot_sim_launches;
unsigned boot_sim_program_order[16], boot_sim_operations;
unsigned boot_sim_cut_operation;
uint16_t boot_sim_cut_mask, boot_sim_erase_masks[5];
bool boot_sim_main_flash_erased;
unsigned boot_sim_fail_program, boot_sim_corrupt_program;
bool boot_sim_fail_unlock, boot_sim_fail_option_unlock;
bool boot_sim_fail_erase, boot_sim_fail_launch, boot_sim_fail_persistent;
static bool pending_error;

void boot_sim_set_raw_option(unsigned index, uint16_t pair) {
    assert(index < 5); /* Reserved WRP1/2/3 reads and writes are forbidden. */
    ((volatile uint16_t *)OB)[index] = pair;
}
void boot_sim_set_option(unsigned index, uint8_t value) {
    boot_sim_set_raw_option(index, value | ((uint16_t)(uint8_t)~value << 8));
}
uint16_t boot_sim_get_option(unsigned index) {
    assert(index < 5);
    return ((volatile uint16_t *)OB)[index];
}
void boot_sim_load_options(void) {
    uint8_t effective[5];
    FLASH->OBR = 0;
    for (unsigned i = 0; i < 5; ++i) {
        uint16_t pair = boot_sim_get_option(i);
        bool valid = (uint8_t)(pair >> 8) == (uint8_t)~pair;
        effective[i] = valid ? (uint8_t)pair : 0xFF;
        if (!valid && pair != 0xFFFFu) FLASH->OBR |= FLASH_OBR_OPTERR;
    }
    FLASH->OBR |= ((uint32_t)effective[1] << 8) |
                  ((uint32_t)effective[2] << 16) |
                  ((uint32_t)effective[3] << 24);
    if (effective[0] != 0xAA) FLASH->OBR |= FLASH_OBR_RDPRT1;
    if (effective[0] == 0xCC) FLASH->OBR |= FLASH_OBR_RDPRT2;
    FLASH->WRPR = 0xFFFFFF00u | effective[4];
}
void boot_sim_init(uint8_t user, uint8_t rdp) {
    memset(&boot_sim_rcc, 0, sizeof boot_sim_rcc);
    memset(&boot_sim_syscfg, 0, sizeof boot_sim_syscfg);
    memset(&boot_sim_flash, 0, sizeof boot_sim_flash);
    memset(&boot_sim_ob, 0xFF, sizeof boot_sim_ob);
    memset(&boot_sim_iwdg, 0, sizeof boot_sim_iwdg);
    memset(&boot_sim_gpiof, 0, sizeof boot_sim_gpiof);
    boot_sim_gpiof.IDR = 1u << 11; /* production boards tie BOOT0 high */
    memset(&boot_sim_systick, 0, sizeof boot_sim_systick);
    memset(&boot_sim_nvic, 0, sizeof boot_sim_nvic);
    memset(&boot_sim_scb, 0, sizeof boot_sim_scb);
    memset(boot_sim_program_order, 0, sizeof boot_sim_program_order);
    FLASH->CR = FLASH_CR_LOCK;
    boot_sim_set_option(0, rdp);
    boot_sim_set_option(1, user);
    boot_sim_load_options();
    boot_sim_erases = boot_sim_programs = boot_sim_launches = boot_sim_operations = 0;
    boot_sim_fail_program = boot_sim_corrupt_program = boot_sim_cut_operation = 0;
    boot_sim_cut_mask = 0;
    for (unsigned i = 0; i < 5; ++i) boot_sim_erase_masks[i] = 0xFFFFu;
    boot_sim_fail_unlock = boot_sim_fail_option_unlock = false;
    boot_sim_fail_erase = boot_sim_fail_launch = boot_sim_fail_persistent = false;
    boot_sim_main_flash_erased = boot_sim_irq_disabled = pending_error = false;
}
static void power_cut(void) {
    FLASH->CR = FLASH_CR_LOCK;
    boot_sim_load_options();
    RCC->CSR = RCC_CSR_PORRSTF;
    longjmp(boot_sim_escape, BOOT_SIM_POWER_CUT);
}
void boot_sim_program_option(unsigned index, uint8_t value) {
    assert(index < 5);
    assert(!(FLASH->CR & FLASH_CR_LOCK));
    assert(FLASH->CR & FLASH_CR_OPTWRE);
    assert(FLASH->CR & FLASH_CR_OPTPG);
    assert(boot_sim_programs < 16);
    boot_sim_program_order[boot_sim_programs++] = index;
    ++boot_sim_operations;
    if (boot_sim_fail_persistent || boot_sim_programs == boot_sim_fail_program) {
        pending_error = true;
        return;
    }
    assert(boot_sim_get_option(index) == 0xFFFFu);
    if (index == 0 && value == 0xAA && (FLASH->OBR & FLASH_OBR_RDPRT1)) {
        boot_sim_main_flash_erased = true;
    }
    uint16_t pair = value | ((uint16_t)(uint8_t)~value << 8);
    if (boot_sim_operations == boot_sim_cut_operation) {
        /* Only a subset of the requested zero bits finished programming. */
        boot_sim_set_raw_option(index, pair | boot_sim_cut_mask);
        power_cut();
    }
    boot_sim_set_raw_option(index, pair);
    if (boot_sim_programs == boot_sim_corrupt_program) {
        boot_sim_set_raw_option(index, pair ^ 1u);
    }
}
HAL_StatusTypeDef FLASH_WaitForLastOperation(uint32_t timeout) {
    assert(timeout <= 100); /* Bound each wait below the running watchdog. */
    if ((FLASH->CR & (FLASH_CR_OPTER | FLASH_CR_STRT)) ==
        (FLASH_CR_OPTER | FLASH_CR_STRT)) {
        assert(!(FLASH->CR & FLASH_CR_LOCK));
        assert(FLASH->CR & FLASH_CR_OPTWRE);
        ++boot_sim_erases;
        ++boot_sim_operations;
        if (boot_sim_operations == boot_sim_cut_operation) {
            for (unsigned i = 0; i < 5; ++i) {
                boot_sim_set_raw_option(i, boot_sim_get_option(i) | boot_sim_erase_masks[i]);
            }
            power_cut();
        }
        memset(&boot_sim_ob, 0xFF, sizeof boot_sim_ob);
        FLASH->CR &= ~FLASH_CR_STRT;
        if (boot_sim_fail_erase || boot_sim_fail_persistent) {
            boot_sim_fail_erase = false;
            return HAL_ERROR;
        }
    }
    if (pending_error) {
        pending_error = false;
        return HAL_ERROR;
    }
    return HAL_OK;
}
HAL_StatusTypeDef HAL_FLASH_Unlock(void) {
    if (boot_sim_fail_unlock) return HAL_ERROR;
    FLASH->CR &= ~FLASH_CR_LOCK;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_FLASH_Lock(void) {
    FLASH->CR |= FLASH_CR_LOCK;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_FLASH_OB_Unlock(void) {
    if (boot_sim_fail_option_unlock) return HAL_ERROR;
    FLASH->CR |= FLASH_CR_OPTWRE;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_FLASH_OB_Lock(void) {
    FLASH->CR &= ~FLASH_CR_OPTWRE;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_FLASH_OB_Launch(void) {
    ++boot_sim_launches;
    if (boot_sim_fail_launch) return HAL_ERROR;
    assert(!(FLASH->CR & FLASH_CR_LOCK));
    assert(FLASH->CR & FLASH_CR_OPTWRE);
    boot_sim_load_options();
    RCC->CSR = RCC_CSR_OBLRSTF;
    longjmp(boot_sim_escape, BOOT_SIM_RELOAD);
}
uint32_t HAL_GetTick(void) {
    static uint32_t tick;
    return tick++;
}
void boot_sim_reset(void) {
    longjmp(boot_sim_escape, BOOT_SIM_RESET);
}
