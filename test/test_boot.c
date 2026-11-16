#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include "boot.h"
#include "boot_sim.h"

static void request_reset(void) {
    int event = setjmp(boot_sim_escape);
    if (!event) boot_enter_bootloader();
    assert(event == BOOT_SIM_RESET);
}

static void migrate(void) {
    int event = setjmp(boot_sim_escape);
    if (!event) {
        (void)boot_repair_option_bytes();
        assert(!"migration should reload options and reset");
    }
    assert(event == BOOT_SIM_RELOAD);
}

static void test_all_user_options(void) {
    for (unsigned user = 0; user <= 255; ++user) {
        boot_sim_init(user, 0xAA);
        if ((user & 0x88u) == 0x08u) {
            assert(boot_repair_option_bytes());
            assert(boot_sim_erases == 0);
        } else {
            migrate();
            assert((uint8_t)OB->USER == ((user & ~0x80u) | 0x08u));
            assert(boot_sim_erases == 1);
            assert(boot_sim_launches == 1);
            assert(boot_repair_option_bytes());
            assert(boot_sim_erases == 1); /* No wear on later boots. */
        }
    }
}

static void test_preserve_bank(void) {
    /* Noncanonical level-1 RDP byte, custom data and the implemented WRP0 byte. */
    boot_sim_init(0xF7, 0x37);
    for (unsigned i = 2; i < 5; ++i) boot_sim_set_option(i, 0x51u + i);
    uint16_t original[5];
    for (unsigned i = 0; i < 5; ++i) original[i] = boot_sim_get_option(i);
    migrate();
    for (unsigned i = 0; i < 5; ++i) {
        if (i != 1) assert(boot_sim_get_option(i) == original[i]);
    }
    assert(FLASH->OBR & FLASH_OBR_RDPRT1);
    assert(!(FLASH->OBR & FLASH_OBR_RDPRT2));
    assert(boot_repair_option_bytes());

    /* ROM callers can leave flash unlocked; HAL unlock treats that as an
     * error, so migration must accept the already-unlocked state. */
    boot_sim_init(0xF7, 0xAA);
    FLASH->CR = FLASH_CR_OPTWRE;
    migrate();
    assert(boot_repair_option_bytes());
}

static void fill_bank(void) {
    boot_sim_init(0xFF, 0xAA);
    for (unsigned i = 2; i < 5; ++i) boot_sim_set_option(i, 0xA0 + i);
}

static void test_failed_writes_restore_bank(void) {
    /* Fail each write, including RDP, and verify recovery restores EVERY
     * option instead of leaving data/protection erased or launching reset. */
    for (unsigned failure = 1; failure <= 5; ++failure) {
        for (unsigned corruption = 0; corruption < 2; ++corruption) {
            fill_bank();
            uint16_t original[5];
            for (unsigned i = 0; i < 5; ++i) original[i] = boot_sim_get_option(i);
            if (corruption) boot_sim_corrupt_program = failure;
            else boot_sim_fail_program = failure;
            assert(!boot_repair_option_bytes());
            for (unsigned i = 0; i < 5; ++i) {
                assert(boot_sim_get_option(i) == original[i]);
            }
            assert(boot_sim_erases == 2);
            assert(boot_sim_launches == 0);
            assert(FLASH->CR & FLASH_CR_LOCK);
            assert(!(FLASH->CR & FLASH_CR_OPTWRE));
        }
    }
    fill_bank();
    boot_sim_fail_erase = true;
    assert(!boot_repair_option_bytes());
    assert(OB->USER == 0x00FFu);
    assert((uint8_t)OB->DATA0 == 0xA2);
    assert(boot_sim_launches == 0);

    fill_bank();
    boot_sim_fail_persistent = true;
    assert(!boot_repair_option_bytes());
    assert(boot_sim_erases == 2); /* Recovery is bounded. */
    assert(boot_sim_launches == 0);
    assert(FLASH->CR & FLASH_CR_LOCK);
}

static void test_refuse_unsafe_migration(void) {
    boot_sim_init(0xFF, 0xCC); /* Irreversible RDP level 2. */
    assert(!boot_repair_option_bytes());
    assert(boot_sim_erases == 0);
    boot_sim_init(0xFF, 0xAA);
    OB->DATA0 = 0x1234; /* Invalid byte/complement pair. */
    assert(!boot_repair_option_bytes());
    assert(boot_sim_erases == 0);
    boot_sim_init(0xFF, 0xAA);
    FLASH->OBR |= FLASH_OBR_OPTERR;
    assert(!boot_repair_option_bytes());
    assert(boot_sim_erases == 0);

    boot_sim_init(0xFF, 0xAA);
    boot_sim_fail_unlock = true;
    assert(!boot_repair_option_bytes());
    assert(boot_sim_erases == 0);
    boot_sim_init(0xFF, 0xAA);
    boot_sim_fail_option_unlock = true;
    assert(!boot_repair_option_bytes());
    assert(boot_sim_erases == 0);
    assert(FLASH->CR & FLASH_CR_LOCK);

    boot_sim_init(0xFF, 0xAA);
    boot_sim_fail_launch = true;
    assert(!boot_repair_option_bytes());
    assert(boot_sim_launches == 1);
    assert((uint8_t)(FLASH->OBR >> 8) == 0xFF); /* Not active yet. */
    assert(FLASH->CR & FLASH_CR_LOCK);
}

static void test_entry_eligibility(void) {
    boot_sim_init(0x7F, 0xAA);
    request_reset();
    RCC->CSR = RCC_CSR_SFTRSTF;
    int event = setjmp(boot_sim_escape);
    if (!event) {
        (void)boot_early();
        assert(!"valid software request should enter ROM");
    }
    assert(event == BOOT_SIM_ROM);
    RCC->CSR = RCC_CSR_SFTRSTF;
    (void)boot_early(); /* Request consumed before ROM jump. */
    assert(!(SYSCFG->CFGR1 & SYSCFG_CFGR1_MEM_MODE));

    /* Same RAM contents must not request DFU after power-on. */
    request_reset();
    RCC->CSR = RCC_CSR_PORRSTF | RCC_CSR_SFTRSTF;
    (void)boot_early();
    assert(!(SYSCFG->CFGR1 & SYSCFG_CFGR1_MEM_MODE));

    /* A failed/absent migration must never attempt an unsupported jump. */
    boot_sim_init(0xFF, 0xAA);
    request_reset();
    RCC->CSR = RCC_CSR_SFTRSTF;
    (void)boot_early();
    assert(!(SYSCFG->CFGR1 & SYSCFG_CFGR1_MEM_MODE));
}

int main(void) {
    test_all_user_options();
    test_preserve_bank();
    test_failed_writes_restore_bank();
    test_refuse_unsafe_migration();
    test_entry_eligibility();
    puts("boot: all tests passed");
    return 0;
}
