/* bxCAN driver, register level. See can.h. */

#include "stm32f0xx_hal.h"

#include "can.h"

#define RX_RING_SIZE 32u   /* frames; power of two */
#define TX_QUEUE_SIZE 16u  /* frames; power of two */
#define INIT_TIMEOUT_MS 10u

#if (RX_RING_SIZE & (RX_RING_SIZE - 1u)) || RX_RING_SIZE > 128u
#error "RX_RING_SIZE must be a power of two no larger than 128"
#endif
#if (TX_QUEUE_SIZE & (TX_QUEUE_SIZE - 1u)) || TX_QUEUE_SIZE > 128u
#error "TX_QUEUE_SIZE must be a power of two no larger than 128"
#endif

#define TSR_TME (CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2)
#define TSR_RQCP (CAN_TSR_RQCP0 | CAN_TSR_RQCP1 | CAN_TSR_RQCP2)
#define TSR_ALST (CAN_TSR_ALST0 | CAN_TSR_ALST1 | CAN_TSR_ALST2)
#define TSR_TERR (CAN_TSR_TERR0 | CAN_TSR_TERR1 | CAN_TSR_TERR2)
#define TSR_ABRQ (CAN_TSR_ABRQ0 | CAN_TSR_ABRQ1 | CAN_TSR_ABRQ2)
#define ESR_LEC_SHIFT 4u
#define LEC_NONE 0u
#define LEC_SOFTWARE 7u /* value software writes so the next error is visible */

/* Keeps the compiler from moving ring accesses across index updates */
#define COMPILER_BARRIER() __asm volatile("" ::: "memory")

/* Receive ring: written by the interrupt, read by the main loop */
static can_frame_t rx_ring[RX_RING_SIZE];
static volatile uint8_t rx_head;
static volatile uint8_t rx_tail;
static volatile uint8_t rx_status; /* CAN_STATUS_RX_FULL / OVERRUN, from the ISR */

/* Transmit queue: main loop only */
static can_frame_t tx_queue[TX_QUEUE_SIZE];
static uint8_t tx_head;
static uint8_t tx_tail;

static uint8_t latched_status; /* events seen by can_poll() since the last read */
static uint32_t bit_timing;    /* BTR without the mode bits */
static bool is_open;

/* 48 MHz kernel clock. 8 time quanta with the sample point at 87.5%,
 * except 800 kbit/s which needs 15 quanta (86.7%). SJW is 1 quantum. */
static bool bit_timing_for(uint32_t bitrate, uint32_t *btr) {
    uint32_t quanta, seg1, seg2, prescaler;

    if (bitrate == 0u) {
        return false;
    }
    if (bitrate == 800000u) {
        quanta = 15u;
        seg1 = 12u;
        seg2 = 2u;
    } else {
        quanta = 8u;
        seg1 = 6u;
        seg2 = 1u;
    }

    prescaler = SystemCoreClock / (bitrate * quanta);
    if (prescaler == 0u || prescaler > 1024u || prescaler * bitrate * quanta != SystemCoreClock) {
        return false;
    }

    *btr = ((seg2 - 1u) << 20) | ((seg1 - 1u) << 16) | (prescaler - 1u);
    return true;
}

static bool wait_for_init_mode(bool in_init) {
    uint32_t start = HAL_GetTick();

    while (((CAN->MSR & CAN_MSR_INAK) != 0u) != in_init) {
        if ((uint32_t)(HAL_GetTick() - start) > INIT_TIMEOUT_MS) {
            return false;
        }
    }
    return true;
}

static void release_fifo0(void) {
    CAN->RF0R = CAN_RF0R_RFOM0;
    while (CAN->RF0R & CAN_RF0R_RFOM0) {
        /* Cleared by hardware once the output mailbox is released */
    }
}

static void read_fifo0(can_frame_t *frame) {
    uint32_t rir = CAN->sFIFOMailBox[0].RIR;
    uint32_t rdtr = CAN->sFIFOMailBox[0].RDTR;
    uint32_t low = CAN->sFIFOMailBox[0].RDLR;
    uint32_t high = CAN->sFIFOMailBox[0].RDHR;
    uint8_t i;

    if (rir & CAN_RI0R_IDE) {
        frame->id = (rir >> 3) & CAN_EXT_ID_MAX;
        frame->flags = CAN_FRAME_EXT;
    } else {
        frame->id = (rir >> 21) & CAN_STD_ID_MAX;
        frame->flags = 0;
    }
    if (rir & CAN_RI0R_RTR) {
        frame->flags |= CAN_FRAME_RTR;
    }

    /* DLC 9..15 means 8 data bytes in classic CAN */
    frame->dlc = (uint8_t)(rdtr & 0xFu);
    if (frame->dlc > 8u) {
        frame->dlc = 8u;
    }

    for (i = 0; i < 4u; i++) {
        frame->data[i] = (uint8_t)(low >> (8u * i));
        frame->data[4u + i] = (uint8_t)(high >> (8u * i));
    }
}

/* CAN interrupt (vector name from vectors.c): drain FIFO 0 into the ring */
void stm32_cec_can(void) {
    while (CAN->RF0R & CAN_RF0R_FMP0) {
        uint8_t head = rx_head;

        if ((uint8_t)(head - rx_tail) < RX_RING_SIZE) {
            read_fifo0(&rx_ring[head & (RX_RING_SIZE - 1u)]);
            COMPILER_BARRIER();
            rx_head = (uint8_t)(head + 1u);
        } else {
            rx_status |= CAN_STATUS_RX_FULL | CAN_STATUS_OVERRUN;
        }
        release_fifo0();
    }

    if (CAN->RF0R & CAN_RF0R_FOVR0) {
        /* The hardware FIFO itself overflowed before we got to it */
        CAN->RF0R = CAN_RF0R_FOVR0 | CAN_RF0R_FULL0;
        rx_status |= CAN_STATUS_OVERRUN;
    }
}

static void flush_rx(void) {
    NVIC_DisableIRQ(CEC_CAN_IRQn);
    while (CAN->RF0R & CAN_RF0R_FMP0) {
        release_fifo0();
    }
    CAN->RF0R = CAN_RF0R_FOVR0 | CAN_RF0R_FULL0;
    rx_tail = rx_head;
    rx_status = 0;
    NVIC_EnableIRQ(CEC_CAN_IRQn);
}

static void load_mailbox(uint32_t mailbox, const can_frame_t *frame) {
    CAN_TxMailBox_TypeDef *box = &CAN->sTxMailBox[mailbox];
    uint32_t tir;

    if (frame->flags & CAN_FRAME_EXT) {
        tir = ((frame->id & CAN_EXT_ID_MAX) << 3) | CAN_TI0R_IDE;
    } else {
        tir = (frame->id & CAN_STD_ID_MAX) << 21;
    }
    if (frame->flags & CAN_FRAME_RTR) {
        tir |= CAN_TI0R_RTR;
    }

    box->TIR = tir;
    box->TDTR = frame->dlc > 8u ? 8u : frame->dlc;
    box->TDLR = (uint32_t)frame->data[0] | ((uint32_t)frame->data[1] << 8) |
                ((uint32_t)frame->data[2] << 16) | ((uint32_t)frame->data[3] << 24);
    box->TDHR = (uint32_t)frame->data[4] | ((uint32_t)frame->data[5] << 8) |
                ((uint32_t)frame->data[6] << 16) | ((uint32_t)frame->data[7] << 24);
    box->TIR = tir | CAN_TI0R_TXRQ;
}

void can_init(void) {
    GPIO_InitTypeDef gpio;

    __GPIOB_CLK_ENABLE();
    RCC->APB1ENR |= RCC_APB1ENR_CANEN;

    /* PB7: transceiver silent-mode input, held low (normal) by the pull-down */
    gpio.Pin = GPIO_PIN_7;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLDOWN;
    gpio.Speed = GPIO_SPEED_LOW;
    gpio.Alternate = 0;
    HAL_GPIO_Init(GPIOB, &gpio);

    /* PB8: CAN_RX, PB9: CAN_TX */
    gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_HIGH;
    gpio.Alternate = GPIO_AF4_CAN;
    HAL_GPIO_Init(GPIOB, &gpio);

    /* Leave sleep and enter initialisation mode */
    CAN->MCR = (CAN->MCR & ~CAN_MCR_SLEEP) | CAN_MCR_INRQ;
    (void)wait_for_init_mode(true);

    /* Automatic bus-off recovery; transmit in request order; automatic
     * retransmission on; overwrite on FIFO overrun (the ISR keeps up) */
    CAN->MCR = (CAN->MCR & ~(CAN_MCR_TTCM | CAN_MCR_AWUM | CAN_MCR_NART | CAN_MCR_RFLM)) |
               CAN_MCR_ABOM | CAN_MCR_TXFP;

    (void)bit_timing_for(1000000u, &bit_timing);
    CAN->BTR = bit_timing;

    /* Filter 0: 32-bit mask mode with an all-zero mask accepts everything
     * into FIFO 0 */
    CAN->FMR |= CAN_FMR_FINIT;
    CAN->FA1R &= ~1u;
    CAN->FM1R &= ~1u;
    CAN->FS1R |= 1u;
    CAN->FFA1R &= ~1u;
    CAN->sFilterRegister[0].FR1 = 0;
    CAN->sFilterRegister[0].FR2 = 0;
    CAN->FA1R |= 1u;
    CAN->FMR &= ~CAN_FMR_FINIT;

    CAN->IER = CAN_IER_FMPIE0 | CAN_IER_FOVIE0;
    NVIC_SetPriority(CEC_CAN_IRQn, 1);
    NVIC_EnableIRQ(CEC_CAN_IRQn);

    is_open = false;
}

bool can_set_bitrate(uint32_t bitrate) {
    uint32_t btr;

    if (is_open || !bit_timing_for(bitrate, &btr)) {
        return false;
    }
    bit_timing = btr;
    CAN->BTR = bit_timing; /* only writable in initialisation mode */
    return true;
}

bool can_open(bool listen_only) {
    if (is_open) {
        return false;
    }

    flush_rx();
    tx_head = tx_tail = 0;
    latched_status = 0;
    CAN->TSR = TSR_RQCP; /* forget completions from aborts in can_close() */

    CAN->BTR = bit_timing | (listen_only ? CAN_BTR_SILM : 0u);
    CAN->ESR = CAN_ESR_LEC; /* LEC = 7 until the first new error */

    /* The controller joins once it sees 11 recessive bits; no need to wait */
    CAN->MCR &= ~CAN_MCR_INRQ;
    is_open = true;
    return true;
}

void can_close(void) {
    if (!is_open) {
        return;
    }

    CAN->MCR |= CAN_MCR_INRQ;
    (void)wait_for_init_mode(true);

    /* Pending mailboxes would otherwise go out when the channel reopens */
    CAN->TSR = TSR_ABRQ;
    CAN->TSR = TSR_RQCP;

    tx_head = tx_tail = 0;
    flush_rx();
    is_open = false;
}

bool can_transmit(const can_frame_t *frame) {
    if (!is_open || (uint8_t)(tx_head - tx_tail) >= TX_QUEUE_SIZE) {
        return false;
    }
    tx_queue[tx_head & (TX_QUEUE_SIZE - 1u)] = *frame;
    tx_head = (uint8_t)(tx_head + 1u);
    can_poll();
    return true;
}

bool can_receive(can_frame_t *frame) {
    uint8_t tail = rx_tail;

    if (tail == rx_head) {
        return false;
    }
    COMPILER_BARRIER();
    *frame = rx_ring[tail & (RX_RING_SIZE - 1u)];
    COMPILER_BARRIER();
    rx_tail = (uint8_t)(tail + 1u);
    return true;
}

void can_poll(void) {
    uint32_t tsr, lec;

    if (!is_open) {
        return;
    }

    /* Completed mailbox requests: note arbitration losses and errors, then
     * acknowledge (writing RQCPx also clears TXOKx, ALSTx and TERRx) */
    tsr = CAN->TSR;
    if (tsr & TSR_ALST) {
        latched_status |= CAN_STATUS_ARBITRATION_LOST;
    }
    if (tsr & TSR_TERR) {
        latched_status |= CAN_STATUS_BUS_ERROR;
    }
    if (tsr & TSR_RQCP) {
        CAN->TSR = tsr & TSR_RQCP;
    }

    /* Feed queued frames into free mailboxes */
    while (tx_tail != tx_head && (CAN->TSR & TSR_TME)) {
        uint32_t mailbox = (CAN->TSR & CAN_TSR_CODE) >> 24;
        load_mailbox(mailbox, &tx_queue[tx_tail & (TX_QUEUE_SIZE - 1u)]);
        tx_tail = (uint8_t)(tx_tail + 1u);
    }

    /* Any protocol error since the last look */
    lec = (CAN->ESR & CAN_ESR_LEC) >> ESR_LEC_SHIFT;
    if (lec != LEC_NONE && lec != LEC_SOFTWARE) {
        latched_status |= CAN_STATUS_BUS_ERROR;
        CAN->ESR = CAN_ESR_LEC;
    }
}

uint8_t can_status(void) {
    uint32_t esr = CAN->ESR;
    uint8_t status;

    __disable_irq();
    status = rx_status;
    rx_status = 0;
    __enable_irq();

    status |= latched_status;
    latched_status = 0;

    if (esr & CAN_ESR_EWGF) {
        status |= CAN_STATUS_ERROR_WARNING;
    }
    if (esr & CAN_ESR_EPVF) {
        status |= CAN_STATUS_ERROR_PASSIVE;
    }
    if (esr & CAN_ESR_BOFF) {
        status |= CAN_STATUS_BUS_ERROR;
    }
    if ((uint8_t)(tx_head - tx_tail) >= TX_QUEUE_SIZE) {
        status |= CAN_STATUS_TX_FULL;
    }
    return status;
}
