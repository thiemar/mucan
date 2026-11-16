/* The F042 flash bank holds five byte/complement pairs. Erasing clears ALL of
 * them; programming generates complements; OBR changes only on reload. */
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
jmp_buf boot_sim_escape;
unsigned boot_sim_erases, boot_sim_programs, boot_sim_launches;
unsigned boot_sim_fail_program, boot_sim_corrupt_program;
bool boot_sim_fail_unlock, boot_sim_fail_option_unlock;
bool boot_sim_fail_erase, boot_sim_fail_launch, boot_sim_fail_persistent;
static bool pending_error;

void boot_sim_set_option(unsigned index, uint8_t value) {
    assert(index < 5); /* Reserved WRP1/2/3 reads and writes are forbidden. */
    ((volatile uint16_t *)OB)[index] = value | ((uint16_t)(uint8_t)~value << 8);
}

uint16_t boot_sim_get_option(unsigned index) {
    assert(index < 5); /* Reserved WRP1/2/3 reads and writes are forbidden. */
    return ((volatile uint16_t *)OB)[index];
}

void boot_sim_load_options(void) {
    uint8_t rdp = OB->RDP;
    FLASH->OBR = ((uint32_t)(uint8_t)OB->USER << 8) |
                 ((uint32_t)(uint8_t)OB->DATA0 << 16) |
                 ((uint32_t)(uint8_t)OB->DATA1 << 24);
    if (rdp != 0xAA) FLASH->OBR |= FLASH_OBR_RDPRT1;
    if (rdp == 0xCC) FLASH->OBR |= FLASH_OBR_RDPRT2;
    FLASH->WRPR = 0xFFFFFF00u | (uint8_t)OB->WRP0;
}

void boot_sim_init(uint8_t user, uint8_t rdp) {
    memset(&boot_sim_rcc, 0, sizeof boot_sim_rcc);
    memset(&boot_sim_syscfg, 0, sizeof boot_sim_syscfg);
    memset(&boot_sim_flash, 0, sizeof boot_sim_flash);
    memset(&boot_sim_ob, 0xFF, sizeof boot_sim_ob);
    FLASH->CR = FLASH_CR_LOCK;
    boot_sim_set_option(0, rdp);
    boot_sim_set_option(1, user);
    boot_sim_load_options();
    boot_sim_erases = boot_sim_programs = boot_sim_launches = 0;
    boot_sim_fail_program = boot_sim_corrupt_program = 0;
    boot_sim_fail_unlock = boot_sim_fail_option_unlock = false;
    boot_sim_fail_erase = boot_sim_fail_launch = boot_sim_fail_persistent = false;
    pending_error = false;
}

void boot_sim_program_option(unsigned index, uint8_t value) {
    assert(!(FLASH->CR & FLASH_CR_LOCK));
    assert(FLASH->CR & FLASH_CR_OPTWRE);
    assert(FLASH->CR & FLASH_CR_OPTPG);
    ++boot_sim_programs;
    if (boot_sim_fail_persistent || boot_sim_programs == boot_sim_fail_program) {
        pending_error = true;
        return;
    }
    assert(boot_sim_get_option(index) == 0xFFFFu);
    boot_sim_set_option(index, value);
    if (boot_sim_programs == boot_sim_corrupt_program) {
        ((volatile uint16_t *)OB)[index] ^= 1u;
    }
}

HAL_StatusTypeDef FLASH_WaitForLastOperation(uint32_t timeout) {
    (void)timeout;
    if ((FLASH->CR & (FLASH_CR_OPTER | FLASH_CR_STRT)) ==
        (FLASH_CR_OPTER | FLASH_CR_STRT)) {
        assert(!(FLASH->CR & FLASH_CR_LOCK));
        assert(FLASH->CR & FLASH_CR_OPTWRE);
        ++boot_sim_erases;
        memset(&boot_sim_ob, 0xFF, sizeof boot_sim_ob);
        FLASH->CR &= ~FLASH_CR_STRT;
        /* Failure may be reported after an erase took effect. */
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
    boot_sim_load_options();
    longjmp(boot_sim_escape, BOOT_SIM_RELOAD);
}
uint32_t HAL_GetTick(void) {
    static uint32_t tick;
    return tick++;
}
void boot_sim_reset(void) {
    longjmp(boot_sim_escape, BOOT_SIM_RESET);
}
void boot_sim_jump_to_rom(void) {
    assert((SYSCFG->CFGR1 & SYSCFG_CFGR1_MEM_MODE) == SYSCFG_CFGR1_MEM_MODE_0);
    longjmp(boot_sim_escape, BOOT_SIM_ROM);
}
