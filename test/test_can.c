/* Exercises the production driver against documented CAN init/BTR behavior.
 * This is a mode-transition model, not a CAN bus or arbitration simulator. */
#include "can.h"
#include "can_sim.h"

#include <assert.h>
#include <stdio.h>

#define BTR_10K 0x00050257u
#define BTR_1M 0x00050005u

static void setup_open_10k(void) {
    can_sim_init();
    can_init();
    assert(can_set_bitrate(10000));
    assert(CAN->BTR == BTR_10K);
    assert(can_open(false));
    can_sim_advance(1);
    assert(!(CAN->MSR & CAN_MSR_INAK));
}

static void test_close_waits_for_slow_frame(void) {
    can_frame_t frame;
    uint32_t start;

    setup_open_10k();
    can_sim_begin_receive(16);
    start = can_sim_time();
    can_close();
    assert(can_sim_time() - start >= 16u);
    assert(can_sim_reset_count() == 1u); /* only the initial reset */
    assert(CAN->MSR & CAN_MSR_INAK);
    assert(can_sim_received_count() == 1u);
    assert(!can_receive(&frame)); /* the frame finishing during close was discarded */

    /* C / S8 / L in one host packet must change both rate and mode. */
    assert(can_set_bitrate(1000000));
    assert(can_open(true));
    assert(CAN->BTR == (BTR_1M | CAN_BTR_SILM));
    assert(can_sim_rejected_btr_writes() == 0u);
}

static void test_timeout_resets_and_preserves_bitrate(void) {
    can_frame_t frame = {0};
    uint32_t start;

    setup_open_10k();
    /* Pending hardware and software transmits must not survive recovery. */
    CAN->sTxMailBox[0].TIR = CAN_TI0R_TXRQ;
    CAN->TSR = 0;
    assert(can_transmit(&frame));
    can_sim_stall_initialization(true);
    start = can_sim_time();
    can_close();
    assert(can_sim_time() - start > 20u);
    assert(can_sim_time() - start < 30u);
    assert(can_sim_reset_count() == 2u);
    assert(CAN->MSR & CAN_MSR_INAK);
    assert(CAN->BTR == BTR_10K);
    assert(CAN->sTxMailBox[0].TIR == 0u);
    assert(!can_transmit(&frame));
    assert(can_open(true));
    assert(CAN->BTR == (BTR_10K | CAN_BTR_SILM));
    CAN->TSR = CAN_TSR_TME0;
    can_poll();
    assert(CAN->sTxMailBox[0].TIR == 0u);
    assert(can_sim_rejected_btr_writes() == 0u);
}

static void test_failed_recovery_stays_closed(void) {
    can_frame_t frame = {0};
    uint32_t start;

    setup_open_10k();
    can_sim_begin_receive(100);
    can_sim_stall_initialization(false);
    start = can_sim_time();
    can_close();
    assert(can_sim_time() - start < 50u); /* two bounded init attempts */
    assert(can_sim_reset_count() == 3u);
    assert(CAN->MCR & CAN_MCR_SLEEP);
    assert(CAN->IER == 0u);
    assert(!can_sim_irq_enabled());
    assert(!can_set_bitrate(1000000));
    assert(!can_open(false));
    assert(!can_open(true));
    assert(!can_transmit(&frame));
    can_sim_advance(100);
    assert(can_sim_received_count() == 0u);
    assert(!can_receive(&frame));
    assert(can_sim_rejected_btr_writes() == 0u);
}

static void test_startup_init_failure_rejects_commands(void) {
    can_sim_init();
    can_sim_stall_initialization(false);
    can_init();
    assert(!can_set_bitrate(10000));
    assert(!can_open(false));
    assert(!can_sim_irq_enabled());
    assert(can_sim_rejected_btr_writes() == 0u);
}

static void test_closed_without_ack_rejects_commands(void) {
    can_sim_init();
    can_init();
    can_sim_stall_initialization(false);
    assert(!can_set_bitrate(10000));
    assert(!can_open(false));
    assert(!can_open(true));
    assert(can_sim_rejected_btr_writes() == 0u);
}

int main(void) {
    test_close_waits_for_slow_frame();
    test_timeout_resets_and_preserves_bitrate();
    test_failed_recovery_stays_closed();
    test_startup_init_failure_rejects_commands();
    test_closed_without_ack_rejects_commands();
    puts("CAN: 5 mode-transition tests passed");
    return 0;
}
