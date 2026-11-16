#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include "boot.h"
#include "boot_sim.h"

static uint16_t pair(uint8_t value) {
    return value | ((uint16_t)(uint8_t)~value << 8);
}
static uint8_t active_user(void) { return (uint8_t)(FLASH->OBR >> 8); }
static void fill_bank(uint8_t user) {
    boot_sim_init(user, 0xAA);
    for (unsigned i = 2; i < BOOT_OPTION_COUNT; ++i) boot_sim_set_option(i, 0xA0 + i);
    boot_sim_load_options();
}
static void apply(int expected) {
    int event = setjmp(boot_sim_escape);
    if (!event) boot_apply_mode();
    assert(event == expected);
}
static void cut_prepare(bool flash_boot) {
    int event = setjmp(boot_sim_escape);
    if (!event) {
        (void)boot_prepare_mode(flash_boot);
        assert(!"configured interruption should cut power");
    }
    assert(event == BOOT_SIM_POWER_CUT);
    assert(!boot_sim_main_flash_erased);
    assert(!(FLASH->OBR & FLASH_OBR_RDPRT2));
}

static void test_startup_is_read_only_and_quiesces_rom(void) {
    for (unsigned user = 0; user <= 255; ++user) {
        {
            boot_sim_init(user, 0xAA);
            uint16_t before[BOOT_OPTION_COUNT], after[BOOT_OPTION_COUNT];
            boot_read_options(before);
            SYSCFG->CFGR1 = SYSCFG_CFGR1_MEM_MODE_0 | (1u << 8);
            SysTick->CTRL = 7;
            SysTick->LOAD = 47999;
            SysTick->VAL = 1234;
            NVIC->ICER[0] = NVIC->ICPR[0] = 0;
            SCB->ICSR = 0;
            boot_early();
            assert(boot_sim_irq_disabled);
            assert(!SysTick->CTRL && !SysTick->LOAD && !SysTick->VAL);
            assert(NVIC->ICER[0] == 0xFFFFFFFFu && NVIC->ICPR[0] == 0xFFFFFFFFu);
            assert(SCB->ICSR == (SCB_ICSR_PENDSTCLR_Msk | SCB_ICSR_PENDSVCLR_Msk));
            assert(SYSCFG->CFGR1 == (1u << 8));
            assert(boot_dfu_available());
            boot_read_options(after);
            for (unsigned i = 0; i < BOOT_OPTION_COUNT; ++i) assert(before[i] == after[i]);
            assert(!boot_sim_erases && !boot_sim_programs && !boot_sim_launches);
        }
    }
}

static void test_modes_and_commit_order(void) {
    fill_bank(0xFF);
    assert(boot_prepare_mode(false)); /* Development stays in ROM on reset. */
    assert(!boot_sim_erases && !boot_sim_programs);
    apply(BOOT_SIM_RELOAD);
    assert(active_user() == 0xFF);
    assert(boot_prepare_mode(true));
    assert(active_user() == 0xFF); /* Not committed until ACK drained/reload. */
    assert(OB->USER == pair(0x7F));
    assert(boot_sim_programs == 5);
    const unsigned expected[] = { 0, 2, 3, 4, 1 };
    for (unsigned i = 0; i < 5; ++i) assert(boot_sim_program_order[i] == expected[i]);
    assert(FLASH->CR & FLASH_CR_LOCK);
    assert(!(FLASH->CR & FLASH_CR_OPTWRE));
    assert(boot_prepare_mode(true));
    assert(boot_sim_erases == 1); /* Repeated prepare adds no wear. */
    apply(BOOT_SIM_RELOAD);
    assert(active_user() == 0x7F);
    assert(boot_dfu_available());
    assert(boot_sim_erases == 1); /* Normal startup does not rewrite modes. */
    assert(boot_prepare_mode(false));
    assert(active_user() == 0x7F);
    apply(BOOT_SIM_RELOAD);
    assert(active_user() == 0xFF);
    assert(!boot_sim_main_flash_erased);
}

static void test_preserve_all_user_bits_and_bank(void) {
    for (unsigned user = 0; user <= 255; ++user) {
        fill_bank(user);
        /* Preserve a noncanonical level-1 RDP, not a HAL canonical value. */
        boot_sim_set_option(0, 0x37);
        boot_sim_load_options();
        uint16_t original[BOOT_OPTION_COUNT];
        boot_read_options(original);
        assert(boot_dfu_available());
        assert(boot_prepare_mode(false));
        /* Standard ROM selection is stored erased; an already-equivalent
         * programmed 0xFF is left alone; other USER bits are kept exactly. */
        uint16_t rom_user = (user | 0x90u) != 0xFFu ? pair(user | 0x90u)
                            : user == 0xFFu ? pair(0xFF) : 0xFFFFu;
        for (unsigned i = 0; i < BOOT_OPTION_COUNT; ++i) {
            assert(boot_sim_get_option(i) == (i == 1 ? rom_user : original[i]));
        }
        assert(!boot_sim_main_flash_erased);
        apply(BOOT_SIM_RELOAD);
        assert(active_user() == (user | 0x90u));
        assert(FLASH->OBR & FLASH_OBR_RDPRT1);
        assert(!boot_prepare_mode(true)); /* Production commit requires RDP0. */
    }
    fill_bank(0x7F);
    FLASH->CR = FLASH_CR_OPTWRE; /* ROM may leave flash already unlocked. */
    assert(boot_prepare_mode(false));
    assert(OB->USER == 0xFFFFu);
}

static void test_failed_writes_restore_bank(void) {
    for (unsigned failure = 1; failure <= 4; ++failure) { /* ROM leaves USER erased */
        for (unsigned corruption = 0; corruption < 2; ++corruption) {
            fill_bank(0x7F);
            uint16_t original[BOOT_OPTION_COUNT];
            boot_read_options(original);
            if (corruption) boot_sim_corrupt_program = failure;
            else boot_sim_fail_program = failure;
            assert(!boot_prepare_mode(false));
            for (unsigned i = 0; i < BOOT_OPTION_COUNT; ++i) {
                assert(boot_sim_get_option(i) == original[i]);
            }
            assert(boot_sim_erases == 2 && !boot_sim_launches);
            assert(FLASH->CR & FLASH_CR_LOCK);
            assert(!(FLASH->CR & FLASH_CR_OPTWRE));
            assert(active_user() == 0x7F);
        }
    }
    fill_bank(0x7F);
    boot_sim_fail_erase = true;
    assert(!boot_prepare_mode(false));
    assert(OB->USER == pair(0x7F) && OB->DATA0 == pair(0xA2));
    assert(!boot_sim_launches);
    fill_bank(0x7F);
    boot_sim_fail_persistent = true;
    assert(!boot_prepare_mode(false));
    assert(boot_sim_erases == 2 && !boot_sim_launches);
    assert(FLASH->CR & FLASH_CR_LOCK);
}

static void test_invalid_options_and_rdp_safety(void) {
    /* Exercise all halfword states in the OBL model, not merely valid pairs. */
    boot_sim_init(0x7F, 0xAA);
    for (unsigned raw = 0; raw <= 0xFFFFu; ++raw) {
        boot_sim_set_raw_option(1, raw);
        boot_sim_load_options();
        bool valid = (uint8_t)(raw >> 8) == (uint8_t)~raw;
        assert(active_user() == (valid ? (uint8_t)raw : 0xFF));
        assert(!!(FLASH->OBR & FLASH_OBR_OPTERR) == (!valid && raw != 0xFFFFu));
    }
    fill_bank(0x7F);
    boot_sim_set_raw_option(0, 0x12AA); /* Invalid AA must stay protected. */
    boot_sim_set_raw_option(2, 0x1234);
    boot_sim_load_options();
    assert(FLASH->OBR & FLASH_OBR_RDPRT1);
    assert(boot_dfu_available());
    assert(!boot_prepare_mode(true));
    assert(boot_prepare_mode(false));
    assert(OB->RDP == 0xFFFFu && OB->DATA0 == 0xFFFFu);
    assert(OB->WRP0 == pair(0xA4));
    assert(!boot_sim_main_flash_erased);
    apply(BOOT_SIM_RELOAD);
    assert(active_user() == 0xFF && (FLASH->OBR & FLASH_OBR_RDPRT1));

    boot_sim_init(0x7F, 0xCC);
    assert(!boot_dfu_available() && !boot_prepare_mode(false));
    assert(!boot_sim_erases);
    boot_sim_init(0x7F, 0x37);
    boot_sim_set_option(0, 0xAA); /* Stored/active protection disagreement. */
    assert(!boot_dfu_available() && !boot_prepare_mode(false));
    assert(!boot_sim_erases && !boot_sim_main_flash_erased);
    boot_sim_init(0x7F, 0xAA);
    boot_sim_set_option(0, 0xCC); /* Never activate a stored level-2 byte. */
    assert(!boot_dfu_available() && !boot_prepare_mode(false));
    assert(!boot_sim_erases);
    boot_sim_init(0x77, 0xAA); /* Unexpected profile cannot commit production. */
    assert(!boot_prepare_mode(true));
    boot_sim_init(0xFF, 0xAA);
    boot_sim_set_raw_option(2, 0x1234); /* Bad raw data despite stale good OBR. */
    assert(boot_prepare_mode(true));
    assert(OB->DATA0 == 0xFFFFu && OB->USER == pair(0x7F));
    fill_bank(0xFF);
    boot_sim_set_raw_option(1, 0xFFFFu & ~0x80u); /* Interrupted USER write. */
    boot_sim_set_raw_option(2, 0x1234);
    boot_sim_set_raw_option(4, 0x5678);
    boot_sim_load_options();
    assert(active_user() == 0xFF && (FLASH->OBR & FLASH_OBR_OPTERR));
    assert(!(FLASH->OBR & FLASH_OBR_RDPRT1));
    assert(boot_prepare_mode(true));
    assert(OB->USER == pair(0x7F) && OB->RDP == pair(0xAA));
    assert(OB->DATA0 == 0xFFFFu && OB->WRP0 == 0xFFFFu);
    assert(OB->DATA1 == pair(0xA3)); /* Valid unrelated pairs stay exact. */
    apply(BOOT_SIM_RELOAD);
    assert(active_user() == 0x7F && !(FLASH->OBR & FLASH_OBR_OPTERR));
    assert(!boot_sim_main_flash_erased);
    boot_sim_init(0xFF, 0xAA);
    boot_sim_set_raw_option(1, 0xFFFFu); /* Erased USER loads FF without OPTERR. */
    boot_sim_load_options();
    assert(active_user() == 0xFF && !(FLASH->OBR & FLASH_OBR_OPTERR));
    assert(boot_prepare_mode(true));
    assert(OB->USER == pair(0x7F) && OB->RDP == pair(0xAA));
}

static void test_unlock_and_reload_failures(void) {
    fill_bank(0x7F);
    boot_sim_fail_unlock = true;
    assert(!boot_prepare_mode(false));
    assert(!boot_sim_erases);
    fill_bank(0x7F);
    boot_sim_fail_option_unlock = true;
    assert(!boot_prepare_mode(false));
    assert(!boot_sim_erases && (FLASH->CR & FLASH_CR_LOCK));
    fill_bank(0x7F);
    assert(boot_prepare_mode(false));
    boot_sim_fail_launch = true;
    apply(BOOT_SIM_RESET); /* Fallback reset; raw verified target survives. */
    assert(active_user() == 0x7F && OB->USER == 0xFFFFu);
    assert(FLASH->CR & FLASH_CR_LOCK);
}

/* Every valid byte/complement pair has eight zero bits. Enumerate every
 * subset that could have erased/programmed before an interrupted operation. */
static uint16_t zero_subset(uint16_t target, unsigned subset) {
    uint16_t mask = 0;
    unsigned n = 0;
    for (unsigned bit = 0; bit < 16; ++bit) {
        if (!(target & (1u << bit))) {
            if (subset & (1u << n)) mask |= 1u << bit;
            ++n;
        }
    }
    assert(n == 8);
    return mask;
}
static void test_power_cuts_during_program(void) {
    const unsigned order[] = { 0, 2, 3, 4, 1 };
    for (unsigned flash_boot = 0; flash_boot < 2; ++flash_boot) {
        for (unsigned position = 0; position < (flash_boot ? 5u : 4u); ++position) {
            for (unsigned subset = 0; subset < 256; ++subset) {
                fill_bank(flash_boot ? 0xFF : 0x7F);
                unsigned index = order[position];
                uint16_t target = index == 1 ? pair(flash_boot ? 0x7F : 0xFF)
                                            : boot_sim_get_option(index);
                boot_sim_cut_operation = position + 2; /* Erase is operation 1. */
                boot_sim_cut_mask = zero_subset(target, subset);
                cut_prepare(flash_boot);
                if (active_user() == 0x7F) {
                    assert(flash_boot && position == 4 && subset == 0);
                    /* Flash becomes selectable only after every other option
                     * is restored, and the USER pair is completely written. */
                    assert(OB->RDP == pair(0xAA));
                    for (unsigned i = 2; i < 5; ++i) {
                        assert(boot_sim_get_option(i) == pair(0xA0 + i));
                    }
                } else assert(active_user() == 0xFF);
            }
        }
    }
}
static void test_power_cuts_during_erase(void) {
    for (unsigned flash_boot = 0; flash_boot < 2; ++flash_boot) {
        for (unsigned partial = 0; partial < 5; ++partial) {
            for (unsigned erased = 0; erased < 32; ++erased) {
                for (unsigned subset = 0; subset < 256; ++subset) {
                    fill_bank(flash_boot ? 0xFF : 0x7F);
                    for (unsigned i = 0; i < 5; ++i) {
                        boot_sim_erase_masks[i] = (erased & (1u << i)) ? 0xFFFFu : 0;
                    }
                    boot_sim_erase_masks[partial] = zero_subset(boot_sim_get_option(partial), subset);
                    boot_sim_cut_operation = 1;
                    cut_prepare(flash_boot);
                    /* Starting from ROM, partial erase never selects flash.
                     * Starting from flash, the old app remains intact until
                     * ROM is confirmed, so either original or ROM is safe. */
                    assert(active_user() == 0xFF || (!flash_boot && active_user() == 0x7F));
                }
            }
        }
    }
}
static void test_rom_selection_requires_boot0_high(void) {
    fill_bank(0x7F);
    GPIOF->IDR = 0; /* BOOT0 low: a ROM selection would just reboot the app */
    assert(!boot_dfu_available());
    assert(!boot_prepare_mode(false));
    assert(!boot_sim_erases && !boot_sim_programs && OB->USER == pair(0x7F));
    assert(FLASH->CR & FLASH_CR_LOCK);
    assert(boot_prepare_mode(true)); /* flash selection ignores the pin */
    GPIOF->IDR = 1u << 11;
    assert(boot_dfu_available());
    assert(boot_prepare_mode(false));
    assert(OB->USER == 0xFFFFu);
}

/* Production -> ROM -> production, as an update does: only the first step
 * erases the bank; the commit programs the erased USER pair in place. */
static void test_flash_commit_programs_in_place(void) {
    fill_bank(0x7F);
    assert(boot_prepare_mode(false));
    assert(boot_sim_erases == 1 && OB->USER == 0xFFFFu);
    apply(BOOT_SIM_RELOAD);
    assert(active_user() == 0xFF);

    unsigned programs = boot_sim_programs;
    assert(boot_prepare_mode(true));
    assert(boot_sim_erases == 1);
    assert(boot_sim_programs == programs + 1 && boot_sim_program_order[programs] == 1);
    assert(OB->RDP == pair(0xAA) && OB->USER == pair(0x7F));
    for (unsigned i = 2; i < 5; ++i) assert(boot_sim_get_option(i) == pair(0xA0 + i));
    assert(FLASH->CR & FLASH_CR_LOCK);
    assert(!(FLASH->CR & FLASH_CR_OPTWRE));
    apply(BOOT_SIM_RELOAD);
    assert(active_user() == 0x7F);

    /* A failed in-place write needs no restore: USER still loads as ROM */
    fill_bank(0x7F);
    assert(boot_prepare_mode(false));
    apply(BOOT_SIM_RELOAD);
    boot_sim_fail_program = boot_sim_programs + 1;
    assert(!boot_prepare_mode(true));
    assert(boot_sim_erases == 1 && OB->USER == 0xFFFFu && OB->RDP == pair(0xAA));
    assert(FLASH->CR & FLASH_CR_LOCK);
    boot_sim_load_options();
    assert(active_user() == 0xFF);

    /* Power lost part-way through the USER write: RDP is never erased, and
     * anything short of the complete pair still loads as ROM */
    for (unsigned subset = 0; subset < 256; ++subset) {
        fill_bank(0x7F);
        assert(boot_prepare_mode(false));
        apply(BOOT_SIM_RELOAD);
        boot_sim_cut_operation = boot_sim_operations + 1;
        boot_sim_cut_mask = zero_subset(pair(0x7F), subset);
        cut_prepare(true);
        assert(OB->RDP == pair(0xAA) && !(FLASH->OBR & FLASH_OBR_RDPRT1));
        assert(active_user() == (subset == 0 ? 0x7F : 0xFF));
    }
}

int main(void) {
    test_startup_is_read_only_and_quiesces_rom();
    test_modes_and_commit_order();
    test_preserve_all_user_bits_and_bank();
    test_failed_writes_restore_bank();
    test_invalid_options_and_rdp_safety();
    test_unlock_and_reload_failures();
    test_power_cuts_during_program();
    test_power_cuts_during_erase();
    test_rom_selection_requires_boot0_high();
    test_flash_commit_programs_in_place();
    puts("boot: all tests passed");
    return 0;
}
