#include "can_sim.h"

#include <assert.h>
#include <string.h>

CAN_TypeDef can_sim_regs;
RCC_TypeDef can_sim_rcc;
GPIO_TypeDef can_sim_gpio;
uint32_t SystemCoreClock = 48000000u;

static uint32_t now_ms;
static uint32_t receive_left;
static uint32_t resets;
static uint32_t rejected_btr;
static uint32_t received;
static bool stuck_init;
static bool reset_recovers;
static bool irq_enabled;
static bool global_irq_enabled;

void stm32_cec_can(void);

void can_sim_init(void) {
    memset(&can_sim_regs, 0, sizeof(can_sim_regs));
    memset(&can_sim_rcc, 0, sizeof(can_sim_rcc));
    memset(&can_sim_gpio, 0, sizeof(can_sim_gpio));
    now_ms = receive_left = resets = rejected_btr = received = 0;
    stuck_init = irq_enabled = false;
    reset_recovers = global_irq_enabled = true;
}

void can_sim_advance(uint32_t milliseconds) {
    while (milliseconds--) {
        ++now_ms;
        if (receive_left && --receive_left == 0u) {
            /* Complete one extended eight-byte frame. The ISR runs while
             * can_close() waits, just as it does on the target. */
            CAN->sFIFOMailBox[0].RIR = (0x12345u << 3) | CAN_RI0R_IDE;
            CAN->sFIFOMailBox[0].RDTR = 8;
            CAN->sFIFOMailBox[0].RDLR = 0x44332211u;
            CAN->sFIFOMailBox[0].RDHR = 0x88776655u;
            CAN->RF0R = 1;
            ++received;
        }
        /* RM0091: entry to init waits for the current frame to finish. */
        if ((CAN->MCR & CAN_MCR_INRQ) && !receive_left && !stuck_init) {
            CAN->MSR |= CAN_MSR_INAK;
        } else if (!(CAN->MCR & CAN_MCR_INRQ)) {
            CAN->MSR &= ~CAN_MSR_INAK;
        }
        if (irq_enabled && global_irq_enabled && (CAN->IER & CAN_IER_FMPIE0) &&
            (CAN->RF0R & CAN_RF0R_FMP0)) {
            stm32_cec_can();
        }
    }
}

void can_sim_begin_receive(uint32_t milliseconds) {
    assert(milliseconds > 0u);
    assert(!(CAN->MSR & CAN_MSR_INAK));
    receive_left = milliseconds;
}

void can_sim_stall_initialization(bool recover) {
    stuck_init = true;
    reset_recovers = recover;
    CAN->MSR &= ~CAN_MSR_INAK;
}

void can_sim_write_bit_timing(uint32_t btr) {
    /* RM0091 29.9.1: BTR is writable only in initialization mode. */
    if (CAN->MSR & CAN_MSR_INAK) {
        CAN->BTR = btr;
    } else {
        ++rejected_btr;
    }
}

void can_sim_release_fifo0(void) {
    assert(CAN->RF0R & CAN_RF0R_FMP0);
    CAN->RF0R &= ~(CAN_RF0R_FMP0 | CAN_RF0R_RFOM0);
}

void can_sim_reset_peripheral(void) {
    memset(&can_sim_regs, 0, sizeof(can_sim_regs));
    CAN->MCR = CAN_MCR_SLEEP;
    CAN->MSR = CAN_MSR_SLAK;
    CAN->TSR = CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2;
    receive_left = 0;
    ++resets;
    if (reset_recovers) {
        stuck_init = false;
    }
}

uint32_t can_sim_time(void) { return now_ms; }
uint32_t can_sim_reset_count(void) { return resets; }
uint32_t can_sim_rejected_btr_writes(void) { return rejected_btr; }
uint32_t can_sim_received_count(void) { return received; }
bool can_sim_irq_enabled(void) { return irq_enabled; }

uint32_t HAL_GetTick(void) {
    can_sim_advance(1);
    return now_ms;
}

void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init) {
    (void)port;
    (void)init;
}
void NVIC_DisableIRQ(IRQn_Type irq) { (void)irq; irq_enabled = false; }
void NVIC_EnableIRQ(IRQn_Type irq) { (void)irq; irq_enabled = true; }
void NVIC_ClearPendingIRQ(IRQn_Type irq) { (void)irq; }
void NVIC_SetPriority(IRQn_Type irq, uint32_t priority) { (void)irq; (void)priority; }
void __disable_irq(void) { global_irq_enabled = false; }
void __enable_irq(void) { global_irq_enabled = true; }
