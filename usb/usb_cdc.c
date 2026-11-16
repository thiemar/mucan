/*
 * Minimal polled USB CDC-ACM device for the STM32F0 USB full-speed peripheral.
 *
 * Replaces the ST USB device library and the HAL PCD driver. Design rules:
 *
 *  - Single context. usb_poll() runs from the main loop and is the only code
 *    that touches the peripheral; the USB interrupt stays disabled. There is
 *    no lock to fail and no state shared with an ISR.
 *  - One fixed configuration: EP0 control, EP1 bulk IN (0x81), EP2 bulk OUT
 *    (0x02) and EP3 interrupt IN (0x83, CDC notifications, never sent). The
 *    data IN and OUT directions use separate endpoint registers.
 *  - Endpoint register writes never clear a CTR flag they did not mean to
 *    clear (the other direction's CTR bit is always written as 1).
 *  - Bulk IN is fed from a byte ring. A zero-length packet follows any
 *    full-size packet that empties the ring, so host reads always complete.
 *  - Bulk OUT is flow-controlled: the endpoint is re-armed only after the
 *    application has consumed the previous packet.
 */

#include <stddef.h>
#include "usb_cdc.h"

/* ------------------------------------------------------------------------ */
/* Endpoints and packet memory layout                                        */
/* ------------------------------------------------------------------------ */

#define USB_VID 0x0483u /* Unchanged from the original firmware */
#define USB_PID 0x5740u

#define EP0_SIZE 64u
#define DATA_SIZE 64u
#define NOTIFY_SIZE 8u

#define EP_CTRL 0u
#define EP_IN 1u
#define EP_OUT 2u
#define EP_NOTIFY 3u
#define EP_COUNT 4u

#define EP_IN_ADDR (0x80u | EP_IN)
#define EP_OUT_ADDR EP_OUT
#define EP_NOTIFY_ADDR (0x80u | EP_NOTIFY)

/* Byte offsets into the 1 KiB packet memory area */
#define PMA_BTABLE 0x000u /* 8 bytes per endpoint */
#define PMA_EP0_TX 0x040u
#define PMA_EP0_RX 0x080u
#define PMA_IN 0x0C0u
#define PMA_OUT 0x100u
#define PMA_NOTIFY 0x140u

/* Buffer descriptor table fields, in 16-bit words */
#define BT_ADDR_TX 0u
#define BT_COUNT_TX 1u
#define BT_ADDR_RX 2u
#define BT_COUNT_RX 3u
#define BT_COUNT_MASK 0x03FFu

/* ------------------------------------------------------------------------ */
/* Register bits (RM0091, USB section)                                      */
/* ------------------------------------------------------------------------ */

#define EPR_CTR_RX 0x8000u  /* rc_w0 */
#define EPR_DTOG_RX 0x4000u /* toggle */
#define EPR_STAT_RX 0x3000u /* toggle */
#define EPR_SETUP 0x0800u   /* read-only */
#define EPR_TYPE 0x0600u    /* rw */
#define EPR_KIND 0x0100u    /* rw */
#define EPR_CTR_TX 0x0080u  /* rc_w0 */
#define EPR_DTOG_TX 0x0040u /* toggle */
#define EPR_STAT_TX 0x0030u /* toggle */
#define EPR_EA 0x000Fu      /* rw */

#define EPR_RW (EPR_TYPE | EPR_KIND | EPR_EA)
#define EPR_CTR (EPR_CTR_RX | EPR_CTR_TX)

#define EPR_TYPE_BULK 0x0000u
#define EPR_TYPE_CONTROL 0x0200u
#define EPR_TYPE_INTERRUPT 0x0600u

#define RX_DIS 0x0000u
#define RX_STALL 0x1000u
#define RX_NAK 0x2000u
#define RX_VALID 0x3000u
#define TX_DIS 0x0000u
#define TX_STALL 0x0010u
#define TX_NAK 0x0020u
#define TX_VALID 0x0030u

#define ISTR_CTR 0x8000u
#define ISTR_RESET 0x0400u
#define ISTR_EP_ID 0x000Fu

/* ------------------------------------------------------------------------ */
/* Hardware access. USB_SIM swaps in a host-side model for the unit tests.   */
/* ------------------------------------------------------------------------ */

#ifdef USB_SIM
#include "usb_sim.h"
#else
#include "stm32f0xx.h"

/* 96-bit unique device ID (RM0091, "Device electronic signature") */
#define HW_UID ((const uint8_t *)0x1FFFF7ACu)

static inline uint16_t hw_epr_get(uint8_t ep) {
    return (&USB->EP0R)[2u * ep];
}

static inline void hw_epr_set(uint8_t ep, uint16_t value) {
    (&USB->EP0R)[2u * ep] = value;
}

static inline uint16_t hw_istr_get(void) {
    return USB->ISTR;
}

static inline void hw_istr_clear(uint16_t bits) {
    /* ISTR flags are rc_w0: write 0 to clear, 1 to leave unchanged */
    USB->ISTR = (uint16_t)~bits;
}

static inline volatile uint16_t *hw_pma(uint16_t offset) {
    /* STM32F0 packet memory: 16-bit words at consecutive 16-bit addresses */
    return (volatile uint16_t *)(USB_PMAADDR + offset);
}

static inline void hw_set_address(uint8_t address) {
    USB->DADDR = (uint16_t)(USB_DADDR_EF | address);
}

static inline void hw_start(void) {
    volatile uint32_t i;

    /* USB clock source (HSI48) is selected in SystemClock_Config() */
    RCC->APB1ENR |= RCC_APB1ENR_USBEN;

    /* Power up the transceiver (PDWN=0) while holding the core in reset */
    USB->CNTR = USB_CNTR_FRES;
    for (i = 0; i < 48u; i++) {
        /* t_STARTUP is at most 1 us */
    }

    /* Release reset with every interrupt masked: the driver polls ISTR */
    USB->CNTR = 0;
    USB->ISTR = 0;
    USB->BTABLE = PMA_BTABLE;
}

static inline void hw_connect(void) {
    USB->BCDR |= USB_BCDR_DPPU;
}
#endif

/* ------------------------------------------------------------------------ */
/* Descriptors                                                               */
/* ------------------------------------------------------------------------ */

#define LO(x) ((uint8_t)((x) & 0xFFu))
#define HI(x) ((uint8_t)(((x) >> 8) & 0xFFu))

static const uint8_t device_desc[18] = {
    18, 1,            /* bLength, DEVICE */
    0x00, 0x02,       /* bcdUSB 2.00 */
    0x02, 0x00, 0x00, /* CDC at device level, so hosts bind without an IAD */
    EP0_SIZE,
    LO(USB_VID), HI(USB_VID),
    LO(USB_PID), HI(USB_PID),
    0x00, 0x02, /* bcdDevice 2.00 */
    1, 2, 3,    /* iManufacturer, iProduct, iSerialNumber */
    1           /* bNumConfigurations */
};

#define CONFIG_DESC_LEN 67u

static const uint8_t config_desc[CONFIG_DESC_LEN] = {
    /* Configuration */
    9, 2, LO(CONFIG_DESC_LEN), HI(CONFIG_DESC_LEN),
    2,    /* bNumInterfaces */
    1,    /* bConfigurationValue */
    0,    /* iConfiguration */
    0x80, /* bmAttributes: bus powered */
    50,   /* bMaxPower: 100 mA */

    /* Interface 0: communications class, abstract control model */
    9, 4, 0, 0, 1, 0x02, 0x02, 0x01, 0,
    /* Header functional descriptor, CDC 1.10 */
    5, 0x24, 0x00, 0x10, 0x01,
    /* Call management: none, data interface 1 */
    5, 0x24, 0x01, 0x00, 1,
    /* ACM: line coding and control line state requests */
    4, 0x24, 0x02, 0x02,
    /* Union: control interface 0, data interface 1 */
    5, 0x24, 0x06, 0, 1,
    /* Notification endpoint */
    7, 5, EP_NOTIFY_ADDR, 0x03, LO(NOTIFY_SIZE), HI(NOTIFY_SIZE), 16,

    /* Interface 1: CDC data */
    9, 4, 1, 0, 2, 0x0A, 0x00, 0x00, 0,
    7, 5, EP_OUT_ADDR, 0x02, LO(DATA_SIZE), HI(DATA_SIZE), 0,
    7, 5, EP_IN_ADDR, 0x02, LO(DATA_SIZE), HI(DATA_SIZE), 0
};

/* ------------------------------------------------------------------------ */
/* Requests                                                                  */
/* ------------------------------------------------------------------------ */

#define REQ_DIR_IN 0x80u
#define REQ_TYPE_MASK 0x60u
#define REQ_TYPE_STANDARD 0x00u
#define REQ_TYPE_CLASS 0x20u
#define REQ_RECIP_MASK 0x1Fu
#define REQ_RECIP_DEVICE 0u
#define REQ_RECIP_INTERFACE 1u
#define REQ_RECIP_ENDPOINT 2u

#define REQ_GET_STATUS 0u
#define REQ_CLEAR_FEATURE 1u
#define REQ_SET_FEATURE 3u
#define REQ_SET_ADDRESS 5u
#define REQ_GET_DESCRIPTOR 6u
#define REQ_GET_CONFIGURATION 8u
#define REQ_SET_CONFIGURATION 9u
#define REQ_GET_INTERFACE 10u
#define REQ_SET_INTERFACE 11u

#define CDC_SET_LINE_CODING 0x20u
#define CDC_GET_LINE_CODING 0x21u
#define CDC_SET_CONTROL_LINE_STATE 0x22u
#define CDC_SEND_BREAK 0x23u

#define FEATURE_ENDPOINT_HALT 0u

#define DESC_DEVICE 1u
#define DESC_CONFIGURATION 2u
#define DESC_STRING 3u

typedef struct {
    uint8_t bmRequestType;
    uint8_t bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
} setup_t;

enum {
    CTRL_IDLE,
    CTRL_DATA_IN,    /* sending the data stage */
    CTRL_DATA_OUT,   /* receiving the data stage */
    CTRL_STATUS_IN,  /* zero-length status packet armed */
    CTRL_STATUS_OUT  /* waiting for the host's zero-length status packet */
};

static struct {
    setup_t req;
    uint8_t state;
    uint8_t set_address; /* apply `address` once the status stage completes */
    uint8_t address;
    const uint8_t *in_data;
    uint16_t in_left;
    uint8_t in_zlp; /* data stage still needs a terminating short packet */
    uint16_t out_len;
    uint16_t out_got;
    uint8_t buf[EP0_SIZE]; /* generated replies and OUT data stages */
} ctrl;

static uint8_t configuration;
static uint8_t line_coding[7] = {0x00, 0xC2, 0x01, 0x00, 0, 0, 8}; /* 115200 8N1 */
static uint8_t line_state;                                          /* bit 0: DTR, bit 1: RTS */
static uint8_t halted;                                              /* bit n: endpoint n halted */

/* ------------------------------------------------------------------------ */
/* CDC data buffers                                                          */
/* ------------------------------------------------------------------------ */

#define TX_RING_SIZE 1024u
#if (TX_RING_SIZE & (TX_RING_SIZE - 1u)) != 0 || TX_RING_SIZE > 32768u
#error "TX_RING_SIZE must be a power of two no larger than 32768"
#endif

static uint8_t tx_ring[TX_RING_SIZE];
static uint16_t tx_head; /* free-running; used = tx_head - tx_tail */
static uint16_t tx_tail;
static uint8_t tx_busy;        /* a packet is armed on EP1 */
static uint8_t tx_zlp_pending; /* last packet was full-size */

static uint8_t rx_buf[DATA_SIZE];
static uint8_t rx_len;
static uint8_t rx_pos;

/* ------------------------------------------------------------------------ */
/* Endpoint and packet memory helpers                                        */
/* ------------------------------------------------------------------------ */

static volatile uint16_t *btable(uint8_t ep, uint8_t field) {
    return hw_pma((uint16_t)(PMA_BTABLE + 8u * ep + 2u * field));
}

/* COUNTn_RX block encoding for a receive buffer of `size` bytes */
static uint16_t rx_count_field(uint16_t size) {
    if (size > 62u) {
        return (uint16_t)(0x8000u | (((size / 32u) - 1u) << 10));
    }
    return (uint16_t)((size / 2u) << 10);
}

static uint16_t rx_count(uint8_t ep) {
    return (uint16_t)(*btable(ep, BT_COUNT_RX) & BT_COUNT_MASK);
}

static void pma_write(uint16_t offset, const uint8_t *src, uint16_t len) {
    volatile uint16_t *dst = hw_pma(offset);
    uint16_t i;

    for (i = 0; i < len; i += 2u) {
        uint16_t word = src[i];
        if (i + 1u < len) {
            word |= (uint16_t)(src[i + 1u] << 8);
        }
        *dst++ = word;
    }
}

static void pma_read(uint16_t offset, uint8_t *dst, uint16_t len) {
    volatile uint16_t *src = hw_pma(offset);
    uint16_t i;

    for (i = 0; i < len; i += 2u) {
        uint16_t word = *src++;
        dst[i] = (uint8_t)word;
        if (i + 1u < len) {
            dst[i + 1u] = (uint8_t)(word >> 8);
        }
    }
}

/*
 * EPnR has three kinds of bits, so every write is built explicitly:
 * rw bits are written back as read, toggle bits get 1 only where a change is
 * wanted, and CTR flags get 1 (no effect) unless being cleared on purpose.
 */
static void ep_set_status(uint8_t ep, uint16_t rx, uint16_t tx) {
    uint16_t r = hw_epr_get(ep);
    hw_epr_set(ep, (uint16_t)((r & EPR_RW) | EPR_CTR | ((r ^ rx) & EPR_STAT_RX) |
                              ((r ^ tx) & EPR_STAT_TX)));
}

static void ep_set_rx_status(uint8_t ep, uint16_t rx) {
    uint16_t r = hw_epr_get(ep);
    hw_epr_set(ep, (uint16_t)((r & EPR_RW) | EPR_CTR | ((r ^ rx) & EPR_STAT_RX)));
}

static void ep_set_tx_status(uint8_t ep, uint16_t tx) {
    uint16_t r = hw_epr_get(ep);
    hw_epr_set(ep, (uint16_t)((r & EPR_RW) | EPR_CTR | ((r ^ tx) & EPR_STAT_TX)));
}

static void ep_clear_ctr_rx(uint8_t ep) {
    hw_epr_set(ep, (uint16_t)((hw_epr_get(ep) & EPR_RW) | EPR_CTR_TX));
}

static void ep_clear_ctr_tx(uint8_t ep) {
    hw_epr_set(ep, (uint16_t)((hw_epr_get(ep) & EPR_RW) | EPR_CTR_RX));
}

/* Set type and address, clear both CTR flags, reset both data toggles to
 * DATA0 and set both statuses. */
static void ep_configure(uint8_t ep, uint16_t type, uint16_t rx, uint16_t tx) {
    uint16_t r = hw_epr_get(ep);
    hw_epr_set(ep, (uint16_t)(type | ep | (r & (EPR_DTOG_RX | EPR_DTOG_TX)) |
                              ((r ^ rx) & EPR_STAT_RX) | ((r ^ tx) & EPR_STAT_TX)));
}

static void ep_transmit(uint8_t ep, uint16_t pma_offset, const uint8_t *data, uint16_t len) {
    pma_write(pma_offset, data, len);
    *btable(ep, BT_COUNT_TX) = len;
    ep_set_tx_status(ep, TX_VALID);
}

/* ------------------------------------------------------------------------ */
/* CDC data path                                                             */
/* ------------------------------------------------------------------------ */

static void cdc_reset(void) {
    tx_head = tx_tail = 0;
    tx_busy = tx_zlp_pending = 0;
    rx_len = rx_pos = 0;
    halted = 0;
    line_state = 0;
}

static void cdc_tx_pump(void) {
    uint8_t packet[DATA_SIZE];
    uint16_t used, n, i;

    if (!configuration || tx_busy || (halted & (1u << EP_IN))) {
        return;
    }

    used = (uint16_t)(tx_head - tx_tail);
    if (!used && !tx_zlp_pending) {
        return;
    }

    n = used < DATA_SIZE ? used : DATA_SIZE;
    for (i = 0; i < n; i++) {
        packet[i] = tx_ring[(uint16_t)(tx_tail + i) & (TX_RING_SIZE - 1u)];
    }
    tx_tail = (uint16_t)(tx_tail + n);

    /* A full packet that drains the ring is followed by a ZLP next time */
    tx_zlp_pending = (n == DATA_SIZE);
    tx_busy = 1;
    ep_transmit(EP_IN, PMA_IN, packet, n);
}

static void cdc_out_received(void) {
    uint16_t n = rx_count(EP_OUT);

    if (n > DATA_SIZE) {
        n = DATA_SIZE;
    }
    pma_read(PMA_OUT, rx_buf, n);
    ep_clear_ctr_rx(EP_OUT);

    rx_len = (uint8_t)n;
    rx_pos = 0;

    /* Hardware leaves the endpoint NAKing; re-arm only once rx_buf drains */
    if (n == 0 && !(halted & (1u << EP_OUT))) {
        ep_set_rx_status(EP_OUT, RX_VALID);
    }
}

static void set_configuration(uint8_t value) {
    configuration = value;
    cdc_reset();

    if (value) {
        ep_configure(EP_IN, EPR_TYPE_BULK, RX_DIS, TX_NAK);
        ep_configure(EP_OUT, EPR_TYPE_BULK, RX_VALID, TX_DIS);
        ep_configure(EP_NOTIFY, EPR_TYPE_INTERRUPT, RX_DIS, TX_NAK);
    } else {
        ep_configure(EP_IN, EPR_TYPE_BULK, RX_DIS, TX_DIS);
        ep_configure(EP_OUT, EPR_TYPE_BULK, RX_DIS, TX_DIS);
        ep_configure(EP_NOTIFY, EPR_TYPE_INTERRUPT, RX_DIS, TX_DIS);
    }
}

/* Returns 1 if halted, 0 if not, -1 if the endpoint doesn't exist */
static int ep_halt_status(uint16_t address) {
    uint8_t ep;

    switch (address) {
        case 0x00u:
        case 0x80u:
            return 0;
        case EP_IN_ADDR:
            ep = EP_IN;
            break;
        case EP_OUT_ADDR:
            ep = EP_OUT;
            break;
        case EP_NOTIFY_ADDR:
            ep = EP_NOTIFY;
            break;
        default:
            return -1;
    }

    if (!configuration) {
        return -1;
    }
    return (halted >> ep) & 1;
}

/* SET_FEATURE / CLEAR_FEATURE (ENDPOINT_HALT). Clearing always resets the
 * data toggle, as USB 2.0 section 9.4.5 requires. */
static bool ep_set_halt(uint16_t address, bool halt) {
    if (address == 0x00u || address == 0x80u) {
        return !halt; /* EP0 cannot be halted; clearing is a no-op */
    }
    if (!configuration) {
        return false;
    }

    switch (address) {
        case EP_IN_ADDR:
            if (halt) {
                halted |= (uint8_t)(1u << EP_IN);
                ep_set_tx_status(EP_IN, TX_STALL);
            } else {
                halted &= (uint8_t)~(1u << EP_IN);
                ep_configure(EP_IN, EPR_TYPE_BULK, RX_DIS, TX_NAK);
                tx_busy = 0;
            }
            return true;
        case EP_OUT_ADDR:
            if (halt) {
                halted |= (uint8_t)(1u << EP_OUT);
                ep_set_rx_status(EP_OUT, RX_STALL);
            } else {
                halted &= (uint8_t)~(1u << EP_OUT);
                ep_configure(EP_OUT, EPR_TYPE_BULK, rx_len ? RX_NAK : RX_VALID, TX_DIS);
            }
            return true;
        case EP_NOTIFY_ADDR:
            if (halt) {
                halted |= (uint8_t)(1u << EP_NOTIFY);
                ep_set_tx_status(EP_NOTIFY, TX_STALL);
            } else {
                halted &= (uint8_t)~(1u << EP_NOTIFY);
                ep_configure(EP_NOTIFY, EPR_TYPE_INTERRUPT, RX_DIS, TX_NAK);
            }
            return true;
        default:
            return false;
    }
}

/* ------------------------------------------------------------------------ */
/* Control transfers                                                         */
/* ------------------------------------------------------------------------ */

static void ctrl_stall(void) {
    /* Protocol stall: the next SETUP is still accepted by the hardware */
    ctrl.state = CTRL_IDLE;
    ep_set_status(EP_CTRL, RX_STALL, TX_STALL);
}

/* Zero-length IN packet for the status stage */
static void ctrl_ack(void) {
    ctrl.state = CTRL_STATUS_IN;
    *btable(EP_CTRL, BT_COUNT_TX) = 0;
    ep_set_status(EP_CTRL, RX_VALID, TX_VALID);
}

static void ctrl_in_packet(void) {
    uint16_t n = ctrl.in_left < EP0_SIZE ? ctrl.in_left : EP0_SIZE;

    pma_write(PMA_EP0_TX, ctrl.in_data, n);
    *btable(EP_CTRL, BT_COUNT_TX) = n;
    ctrl.in_data += n;
    ctrl.in_left = (uint16_t)(ctrl.in_left - n);
    if (n < EP0_SIZE) {
        ctrl.in_zlp = 0; /* a short packet ends the data stage */
    }

    /* RX stays valid so the host can move to the status stage at any time */
    ep_set_status(EP_CTRL, RX_VALID, TX_VALID);
}

static void ctrl_reply(const uint8_t *data, uint16_t len) {
    if (ctrl.req.wLength == 0) {
        ctrl_ack();
        return;
    }
    if (len > ctrl.req.wLength) {
        len = ctrl.req.wLength;
    }
    ctrl.in_data = data;
    ctrl.in_left = len;
    ctrl.in_zlp = (len < ctrl.req.wLength) && ((len % EP0_SIZE) == 0u);
    ctrl.state = CTRL_DATA_IN;
    ctrl_in_packet();
}

static bool ctrl_receive(uint16_t len) {
    if (len > sizeof(ctrl.buf)) {
        return false;
    }
    ctrl.out_len = len;
    ctrl.out_got = 0;
    ctrl.state = CTRL_DATA_OUT;
    ep_set_status(EP_CTRL, RX_VALID, TX_NAK);
    return true;
}

static uint16_t string_descriptor(uint8_t index, uint8_t *out) {
    static const char hex[] = "0123456789ABCDEF";
    char serial[25];
    const char *text;
    uint16_t len = 2;
    uint8_t i;

    switch (index) {
        case 0:
            out[0] = 4;
            out[1] = DESC_STRING;
            out[2] = 0x09; /* English (United States) */
            out[3] = 0x04;
            return 4;
        case 1:
            text = "Thiemar";
            break;
        case 2:
            text = "muCAN";
            break;
        case 3:
            /* Unique per device, so hosts give each adapter a stable name */
            for (i = 0; i < 12u; i++) {
                serial[2u * i] = hex[HW_UID[i] >> 4];
                serial[2u * i + 1u] = hex[HW_UID[i] & 0xFu];
            }
            serial[24] = '\0';
            text = serial;
            break;
        default:
            return 0;
    }

    while (*text && len + 2u <= EP0_SIZE) {
        out[len++] = (uint8_t)*text++;
        out[len++] = 0;
    }
    out[0] = (uint8_t)len;
    out[1] = DESC_STRING;
    return len;
}

static bool get_descriptor(void) {
    uint8_t type = HI(ctrl.req.wValue);
    uint8_t index = LO(ctrl.req.wValue);
    uint16_t len;

    switch (type) {
        case DESC_DEVICE:
            if (index) {
                return false;
            }
            ctrl_reply(device_desc, sizeof(device_desc));
            return true;
        case DESC_CONFIGURATION:
            if (index) {
                return false;
            }
            ctrl_reply(config_desc, sizeof(config_desc));
            return true;
        case DESC_STRING:
            len = string_descriptor(index, ctrl.buf);
            if (!len) {
                return false;
            }
            ctrl_reply(ctrl.buf, len);
            return true;
        default:
            /* Including DEVICE_QUALIFIER, which a full-speed-only device
             * must answer with a STALL */
            return false;
    }
}

static bool standard_request(void) {
    const setup_t *req = &ctrl.req;
    uint8_t recipient = req->bmRequestType & REQ_RECIP_MASK;
    bool in = (req->bmRequestType & REQ_DIR_IN) != 0;
    int status;

    switch (req->bRequest) {
        case REQ_GET_STATUS:
            if (!in) {
                return false;
            }
            ctrl.buf[0] = 0; /* bus powered, no remote wakeup, not halted */
            ctrl.buf[1] = 0;
            if (recipient == REQ_RECIP_INTERFACE) {
                if (!configuration || req->wIndex > 1u) {
                    return false;
                }
            } else if (recipient == REQ_RECIP_ENDPOINT) {
                status = ep_halt_status(req->wIndex);
                if (status < 0) {
                    return false;
                }
                ctrl.buf[0] = (uint8_t)status;
            } else if (recipient != REQ_RECIP_DEVICE) {
                return false;
            }
            ctrl_reply(ctrl.buf, 2);
            return true;

        case REQ_CLEAR_FEATURE:
        case REQ_SET_FEATURE:
            /* Only ENDPOINT_HALT; remote wakeup and test modes aren't supported */
            if (in || recipient != REQ_RECIP_ENDPOINT || req->wValue != FEATURE_ENDPOINT_HALT ||
                !ep_set_halt(req->wIndex, req->bRequest == REQ_SET_FEATURE)) {
                return false;
            }
            ctrl_ack();
            return true;

        case REQ_SET_ADDRESS:
            if (in || recipient != REQ_RECIP_DEVICE || req->wValue > 127u || configuration) {
                return false;
            }
            /* The new address takes effect after the status stage */
            ctrl.address = (uint8_t)req->wValue;
            ctrl.set_address = 1;
            ctrl_ack();
            return true;

        case REQ_GET_DESCRIPTOR:
            return in && get_descriptor();

        case REQ_GET_CONFIGURATION:
            if (!in || recipient != REQ_RECIP_DEVICE) {
                return false;
            }
            ctrl.buf[0] = configuration;
            ctrl_reply(ctrl.buf, 1);
            return true;

        case REQ_SET_CONFIGURATION:
            if (in || recipient != REQ_RECIP_DEVICE || req->wValue > 1u) {
                return false;
            }
            set_configuration((uint8_t)req->wValue);
            ctrl_ack();
            return true;

        case REQ_GET_INTERFACE:
            if (!in || recipient != REQ_RECIP_INTERFACE || !configuration || req->wIndex > 1u) {
                return false;
            }
            ctrl.buf[0] = 0;
            ctrl_reply(ctrl.buf, 1);
            return true;

        case REQ_SET_INTERFACE:
            if (in || recipient != REQ_RECIP_INTERFACE || !configuration || req->wIndex > 1u ||
                req->wValue != 0u) {
                return false;
            }
            ctrl_ack();
            return true;

        default:
            return false;
    }
}

static bool cdc_request(void) {
    const setup_t *req = &ctrl.req;
    bool in = (req->bmRequestType & REQ_DIR_IN) != 0;

    if ((req->bmRequestType & REQ_RECIP_MASK) != REQ_RECIP_INTERFACE || req->wIndex != 0u ||
        !configuration) {
        return false;
    }

    switch (req->bRequest) {
        case CDC_SET_LINE_CODING:
            return !in && req->wLength == sizeof(line_coding) && ctrl_receive(sizeof(line_coding));
        case CDC_GET_LINE_CODING:
            if (!in) {
                return false;
            }
            ctrl_reply(line_coding, sizeof(line_coding));
            return true;
        case CDC_SET_CONTROL_LINE_STATE:
            if (in) {
                return false;
            }
            line_state = (uint8_t)(req->wValue & 0x3u);
            ctrl_ack();
            return true;
        case CDC_SEND_BREAK:
            if (in) {
                return false;
            }
            ctrl_ack();
            return true;
        default:
            return false;
    }
}

/* Data stage of a host-to-device request has fully arrived */
static bool ctrl_out_complete(void) {
    uint8_t i;

    if ((ctrl.req.bmRequestType & REQ_TYPE_MASK) == REQ_TYPE_CLASS &&
        ctrl.req.bRequest == CDC_SET_LINE_CODING && ctrl.out_got == sizeof(line_coding)) {
        for (i = 0; i < sizeof(line_coding); i++) {
            line_coding[i] = ctrl.buf[i];
        }
        return true;
    }
    return false;
}

static void ctrl_setup_received(void) {
    uint8_t raw[8];
    uint16_t n = rx_count(EP_CTRL);
    bool ok = false;

    /* Read before clearing CTR_RX: a new SETUP may overwrite the buffer */
    pma_read(PMA_EP0_RX, raw, sizeof(raw));
    ep_clear_ctr_rx(EP_CTRL);

    /* Any transfer in progress is abandoned */
    ctrl.state = CTRL_IDLE;
    ctrl.set_address = 0;

    if (n != sizeof(raw)) {
        ctrl_stall();
        return;
    }

    ctrl.req.bmRequestType = raw[0];
    ctrl.req.bRequest = raw[1];
    ctrl.req.wValue = (uint16_t)(raw[2] | (raw[3] << 8));
    ctrl.req.wIndex = (uint16_t)(raw[4] | (raw[5] << 8));
    ctrl.req.wLength = (uint16_t)(raw[6] | (raw[7] << 8));

    switch (ctrl.req.bmRequestType & REQ_TYPE_MASK) {
        case REQ_TYPE_STANDARD:
            ok = standard_request();
            break;
        case REQ_TYPE_CLASS:
            ok = cdc_request();
            break;
        default:
            break;
    }

    if (!ok) {
        ctrl_stall();
    }
}

static void ctrl_out_received(void) {
    uint16_t n = rx_count(EP_CTRL);

    if (ctrl.state != CTRL_DATA_OUT) {
        /* Status stage of an IN transfer (possibly ending it early), or a
         * stray packet: either way the transfer is over */
        ep_clear_ctr_rx(EP_CTRL);
        ctrl.state = CTRL_IDLE;
        ep_set_status(EP_CTRL, RX_VALID, TX_NAK);
        return;
    }

    if (n > (uint16_t)(ctrl.out_len - ctrl.out_got)) {
        ep_clear_ctr_rx(EP_CTRL);
        ctrl_stall();
        return;
    }

    pma_read(PMA_EP0_RX, ctrl.buf + ctrl.out_got, n);
    ep_clear_ctr_rx(EP_CTRL);
    ctrl.out_got = (uint16_t)(ctrl.out_got + n);

    if (ctrl.out_got < ctrl.out_len && n == EP0_SIZE) {
        ep_set_rx_status(EP_CTRL, RX_VALID);
    } else if (ctrl_out_complete()) {
        ctrl_ack();
    } else {
        ctrl_stall();
    }
}

static void ctrl_in_sent(void) {
    ep_clear_ctr_tx(EP_CTRL);

    switch (ctrl.state) {
        case CTRL_DATA_IN:
            if (ctrl.in_left || ctrl.in_zlp) {
                ctrl_in_packet();
            } else {
                ctrl.state = CTRL_STATUS_OUT;
            }
            break;
        case CTRL_STATUS_IN:
            if (ctrl.set_address) {
                hw_set_address(ctrl.address);
                ctrl.set_address = 0;
            }
            ctrl.state = CTRL_IDLE;
            break;
        default:
            break;
    }
}

/* ------------------------------------------------------------------------ */
/* Bus events                                                                */
/* ------------------------------------------------------------------------ */

static void bus_reset(void) {
    *btable(EP_CTRL, BT_ADDR_TX) = PMA_EP0_TX;
    *btable(EP_CTRL, BT_COUNT_TX) = 0;
    *btable(EP_CTRL, BT_ADDR_RX) = PMA_EP0_RX;
    *btable(EP_CTRL, BT_COUNT_RX) = rx_count_field(EP0_SIZE);

    *btable(EP_IN, BT_ADDR_TX) = PMA_IN;
    *btable(EP_IN, BT_COUNT_TX) = 0;
    *btable(EP_IN, BT_ADDR_RX) = 0;
    *btable(EP_IN, BT_COUNT_RX) = 0;

    *btable(EP_OUT, BT_ADDR_TX) = 0;
    *btable(EP_OUT, BT_COUNT_TX) = 0;
    *btable(EP_OUT, BT_ADDR_RX) = PMA_OUT;
    *btable(EP_OUT, BT_COUNT_RX) = rx_count_field(DATA_SIZE);

    *btable(EP_NOTIFY, BT_ADDR_TX) = PMA_NOTIFY;
    *btable(EP_NOTIFY, BT_COUNT_TX) = 0;
    *btable(EP_NOTIFY, BT_ADDR_RX) = 0;
    *btable(EP_NOTIFY, BT_COUNT_RX) = 0;

    ctrl.state = CTRL_IDLE;
    ctrl.set_address = 0;
    ep_configure(EP_CTRL, EPR_TYPE_CONTROL, RX_VALID, TX_NAK);
    set_configuration(0);
    hw_set_address(0);
}

void usb_init(void) {
    hw_start();
    bus_reset();
    hw_connect();
}

void usb_poll(void) {
    uint8_t pass;

    if (hw_istr_get() & ISTR_RESET) {
        hw_istr_clear(ISTR_RESET);
        bus_reset();
    }

    /* Each pass clears at least one CTR flag; the bound keeps the main loop
     * moving if the host is very busy (leftovers are handled next call) */
    for (pass = 0; pass < 8u; pass++) {
        uint16_t istr = hw_istr_get();
        uint8_t ep;
        uint16_t epr;

        if (!(istr & ISTR_CTR)) {
            break;
        }

        ep = (uint8_t)(istr & ISTR_EP_ID);
        epr = hw_epr_get(ep);

        switch (ep) {
            case EP_CTRL:
                /* Finish the previous IN before a SETUP starts a new transfer */
                if (epr & EPR_CTR_TX) {
                    ctrl_in_sent();
                }
                if (epr & EPR_CTR_RX) {
                    if (epr & EPR_SETUP) {
                        ctrl_setup_received();
                    } else {
                        ctrl_out_received();
                    }
                }
                break;
            case EP_IN:
                if (epr & EPR_CTR_TX) {
                    ep_clear_ctr_tx(EP_IN);
                    tx_busy = 0;
                }
                if (epr & EPR_CTR_RX) {
                    ep_clear_ctr_rx(EP_IN);
                }
                break;
            case EP_OUT:
                if (epr & EPR_CTR_RX) {
                    cdc_out_received();
                }
                if (epr & EPR_CTR_TX) {
                    ep_clear_ctr_tx(EP_OUT);
                }
                break;
            default:
                if (epr & EPR_CTR_RX) {
                    ep_clear_ctr_rx(ep);
                }
                if (epr & EPR_CTR_TX) {
                    ep_clear_ctr_tx(ep);
                }
                break;
        }
    }

    cdc_tx_pump();
}

/* ------------------------------------------------------------------------ */
/* Application interface                                                     */
/* ------------------------------------------------------------------------ */

bool usb_cdc_configured(void) {
    return configuration != 0;
}

bool usb_cdc_dtr(void) {
    return (line_state & 0x1u) != 0;
}

int usb_cdc_getc(void) {
    uint8_t c;

    if (rx_pos >= rx_len) {
        return -1;
    }

    c = rx_buf[rx_pos++];
    if (rx_pos == rx_len) {
        rx_len = rx_pos = 0;
        if (configuration && !(halted & (1u << EP_OUT))) {
            ep_set_rx_status(EP_OUT, RX_VALID);
        }
    }
    return c;
}

uint16_t usb_cdc_write_space(void) {
    if (!configuration) {
        return 0;
    }
    return (uint16_t)(TX_RING_SIZE - (uint16_t)(tx_head - tx_tail));
}

bool usb_cdc_write(const uint8_t *data, uint16_t len) {
    uint16_t i;

    if (!configuration || len > usb_cdc_write_space()) {
        return false;
    }

    for (i = 0; i < len; i++) {
        tx_ring[(uint16_t)(tx_head + i) & (TX_RING_SIZE - 1u)] = data[i];
    }
    tx_head = (uint16_t)(tx_head + len);

    cdc_tx_pump();
    return true;
}
