/* Host-side model of the STM32F0 USB peripheral. See usb_sim.h. */

#include <stdio.h>
#include <string.h>

#include "usb_sim.h"

/* EPnR (RM0091) */
#define CTR_RX 0x8000u
#define DTOG_RX 0x4000u
#define STAT_RX 0x3000u
#define SETUP 0x0800u
#define EP_TYPE 0x0600u
#define EP_KIND 0x0100u
#define CTR_TX 0x0080u
#define DTOG_TX 0x0040u
#define STAT_TX 0x0030u
#define EA 0x000Fu

#define TYPE_CONTROL 0x0200u
#define RX_DIS 0x0000u
#define RX_STALL 0x1000u
#define RX_NAK 0x2000u
#define TX_DIS 0x0000u
#define TX_STALL 0x0010u
#define TX_NAK 0x0020u

/* ISTR */
#define ISTR_CTR 0x8000u
#define ISTR_RESET 0x0400u
#define ISTR_DIR 0x0010u

#define PMA_BYTES 1024u
#define MAX_PACKET 64u

static uint16_t epr[8];
static uint16_t last_read[8]; /* what the driver last saw in each EPnR */
static uint16_t pma[PMA_BYTES / 2u];
static uint16_t istr_events;
static uint8_t daddr;
static bool connected;
static uint8_t host_in_toggle[8];
static uint8_t host_out_toggle[8];
static int errors;
static bool quiet;

static uint8_t hook_ep;
static void (*hook_fn)(void);

const uint8_t sim_uid[12] = {0x30, 0x00, 0x2B, 0x00, 0x12, 0x51, 0x4B, 0x31, 0x36, 0x39, 0x30, 0xA7};

#define FAIL(...)                         \
    do {                                  \
        if (!quiet) {                     \
            fprintf(stderr, "  sim: ");   \
            fprintf(stderr, __VA_ARGS__); \
            fputc('\n', stderr);          \
        }                                 \
        errors++;                         \
    } while (0)

/* ---- Hardware interface ------------------------------------------------ */

uint16_t hw_epr_get(uint8_t ep) {
    if (ep >= 8u) {
        FAIL("read of EP%uR", ep);
        return 0;
    }
    last_read[ep] = epr[ep];
    return epr[ep];
}

void hw_epr_set(uint8_t ep, uint16_t v) {
    uint16_t old, lost;

    if (ep >= 8u) {
        FAIL("write to EP%uR", ep);
        return;
    }

    if (hook_fn && hook_ep == ep) {
        void (*fn)(void) = hook_fn;
        hook_fn = NULL;
        fn();
    }

    old = epr[ep];

    /* A CTR flag the driver never saw must not be cleared by this write */
    lost = (uint16_t)(old & ~v & (CTR_RX | CTR_TX) & ~last_read[ep]);
    if (lost) {
        FAIL("EP%uR write 0x%04X cleared unseen %s", ep, v, (lost & CTR_RX) ? "CTR_RX" : "CTR_TX");
    }

    epr[ep] = (uint16_t)((v & (EP_TYPE | EP_KIND | EA)) |                 /* rw */
                         ((old ^ v) & (DTOG_RX | STAT_RX | DTOG_TX | STAT_TX)) | /* toggle */
                         (old & v & (CTR_RX | CTR_TX)) |                        /* rc_w0 */
                         (old & SETUP));                                        /* read-only */
}

uint16_t hw_istr_get(void) {
    uint16_t v = istr_events;
    uint8_t ep;

    /* CTR, EP_ID and DIR reflect the highest-priority endpoint with a flag */
    for (ep = 0; ep < 8u; ep++) {
        if (epr[ep] & (CTR_RX | CTR_TX)) {
            v |= (uint16_t)(ISTR_CTR | ep | ((epr[ep] & CTR_RX) ? ISTR_DIR : 0u));
            break;
        }
    }
    return v;
}

void hw_istr_clear(uint16_t bits) {
    istr_events &= (uint16_t)~bits;
}

volatile uint16_t *hw_pma(uint16_t offset) {
    if ((offset & 1u) || offset >= PMA_BYTES) {
        FAIL("packet memory access at 0x%03X", offset);
        offset = 0;
    }
    return (volatile uint16_t *)&pma[offset / 2u];
}

void hw_set_address(uint8_t address) {
    daddr = (uint8_t)(0x80u | address);
}

void hw_start(void) {
    memset(epr, 0, sizeof(epr));
    memset(last_read, 0, sizeof(last_read));
    for (size_t i = 0; i < sizeof(pma) / sizeof(pma[0]); i++) {
        pma[i] = 0xA5A5u; /* packet memory isn't cleared by reset */
    }
    istr_events = 0;
    daddr = 0;
    connected = false;
}

void hw_connect(void) {
    connected = true;
}

void hw_disconnect(void) {
    connected = false;
}

/* ---- Host side ---------------------------------------------------------- */

static uint16_t bt(uint8_t ep, uint8_t word) {
    return pma[(8u * ep + 2u * word) / 2u]; /* BTABLE at 0 */
}

static void bt_set_rx_count(uint8_t ep, uint16_t n) {
    uint16_t *reg = &pma[(8u * ep + 6u) / 2u];
    *reg = (uint16_t)((*reg & ~0x03FFu) | n);
}

static uint16_t rx_capacity(uint8_t ep) {
    uint16_t reg = bt(ep, 3);
    uint16_t blocks = (uint16_t)((reg >> 10) & 0x1Fu);
    return (reg & 0x8000u) ? (uint16_t)((blocks + 1u) * 32u) : (uint16_t)(blocks * 2u);
}

static bool buffer_ok(uint16_t addr, uint16_t len, const char *what, uint8_t ep) {
    if (addr < 32u || (addr & 1u) || addr + len > PMA_BYTES) {
        FAIL("EP%u %s buffer at 0x%03X (+%u) is invalid", ep, what, addr, len);
        return false;
    }
    return true;
}

static void pma_put(uint16_t addr, const uint8_t *data, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
        uint16_t *w = &pma[(addr + i) / 2u];
        if ((addr + i) & 1u) {
            *w = (uint16_t)((*w & 0x00FFu) | (data[i] << 8));
        } else {
            *w = (uint16_t)((*w & 0xFF00u) | data[i]);
        }
    }
}

static void pma_get(uint16_t addr, uint8_t *data, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
        uint16_t w = pma[(addr + i) / 2u];
        data[i] = (uint8_t)(((addr + i) & 1u) ? (w >> 8) : w);
    }
}

static bool addressed(uint8_t address) {
    return connected && (daddr & 0x80u) && (daddr & 0x7Fu) == address;
}

static bool endpoint_ok(uint8_t ep) {
    if (ep >= 8u || (epr[ep] & EA) != ep) {
        return false;
    }
    return true;
}

void sim_bus_reset(void) {
    /* USB reset disables all endpoints and the device address */
    memset(epr, 0, sizeof(epr));
    memset(host_in_toggle, 0, sizeof(host_in_toggle));
    memset(host_out_toggle, 0, sizeof(host_out_toggle));
    daddr = 0;
    istr_events |= ISTR_RESET;
}

sim_handshake_t sim_setup(uint8_t address, const uint8_t packet[8]) {
    uint16_t e;

    if (!addressed(address) || !endpoint_ok(0)) {
        return SIM_TIMEOUT;
    }
    e = epr[0];
    if ((e & EP_TYPE) != TYPE_CONTROL || (e & STAT_RX) == RX_DIS) {
        return SIM_TIMEOUT;
    }
    if (e & CTR_RX) {
        /* Previous reception not yet serviced: SETUP is dropped, host retries */
        return SIM_TIMEOUT;
    }
    if (rx_capacity(0) < 8u || !buffer_ok(bt(0, 2), 8, "RX", 0)) {
        return SIM_TIMEOUT;
    }

    /* SETUP is accepted whatever STAT_RX says (NAK and STALL included) */
    pma_put(bt(0, 2), packet, 8);
    bt_set_rx_count(0, 8);

    /* Hardware: DTOG_TX=1, DTOG_RX=1 after the DATA0 SETUP, both NAK */
    e &= (uint16_t)~(STAT_RX | STAT_TX | DTOG_RX | DTOG_TX);
    e |= RX_NAK | TX_NAK | DTOG_RX | DTOG_TX | SETUP | CTR_RX;
    epr[0] = e;

    host_in_toggle[0] = 1;
    host_out_toggle[0] = 1;
    return SIM_ACK;
}

sim_handshake_t sim_out(uint8_t address, uint8_t ep, const uint8_t *data, uint16_t len) {
    uint16_t e;
    uint8_t device_toggle;

    if (!addressed(address) || !endpoint_ok(ep)) {
        return SIM_TIMEOUT;
    }
    e = epr[ep];
    switch (e & STAT_RX) {
        case RX_DIS:
            return SIM_TIMEOUT;
        case RX_STALL:
            return SIM_STALL;
        case RX_NAK:
            return SIM_NAK;
        default:
            break;
    }
    if (e & CTR_RX) {
        FAIL("EP%u armed for OUT while CTR_RX still set", ep);
    }
    if (len > MAX_PACKET || len > rx_capacity(ep)) {
        FAIL("EP%u OUT of %u bytes exceeds its %u-byte buffer", ep, len, rx_capacity(ep));
        return SIM_TIMEOUT;
    }
    if (!buffer_ok(bt(ep, 2), rx_capacity(ep), "RX", ep)) {
        return SIM_TIMEOUT;
    }

    device_toggle = (e & DTOG_RX) ? 1u : 0u;
    if (device_toggle != host_out_toggle[ep]) {
        /* Hardware ACKs a toggle mismatch but drops the data */
        FAIL("EP%u OUT data toggle: host sent DATA%u, device expects DATA%u", ep,
             host_out_toggle[ep], device_toggle);
        host_out_toggle[ep] ^= 1u;
        return SIM_ACK;
    }

    if (len) {
        pma_put(bt(ep, 2), data, len);
    }
    bt_set_rx_count(ep, len);

    e ^= DTOG_RX;
    e = (uint16_t)((e & ~STAT_RX) | RX_NAK | CTR_RX);
    e &= (uint16_t)~SETUP;
    epr[ep] = e;
    host_out_toggle[ep] ^= 1u;
    return SIM_ACK;
}

sim_handshake_t sim_in(uint8_t address, uint8_t ep, uint8_t *data, uint16_t *len) {
    uint16_t e, n;
    uint8_t device_toggle;

    *len = 0;
    if (!addressed(address) || !endpoint_ok(ep)) {
        return SIM_TIMEOUT;
    }
    e = epr[ep];
    switch (e & STAT_TX) {
        case TX_DIS:
            return SIM_TIMEOUT;
        case TX_STALL:
            return SIM_STALL;
        case TX_NAK:
            return SIM_NAK;
        default:
            break;
    }
    if (e & CTR_TX) {
        FAIL("EP%u armed for IN while CTR_TX still set", ep);
    }

    n = (uint16_t)(bt(ep, 1) & 0x03FFu);
    if (n > MAX_PACKET) {
        FAIL("EP%u IN packet of %u bytes", ep, n);
        return SIM_TIMEOUT;
    }
    if (!buffer_ok(bt(ep, 0), n, "TX", ep)) {
        return SIM_TIMEOUT;
    }

    device_toggle = (e & DTOG_TX) ? 1u : 0u;
    if (device_toggle != host_in_toggle[ep]) {
        FAIL("EP%u IN data toggle: device sent DATA%u, host expects DATA%u", ep, device_toggle,
             host_in_toggle[ep]);
    }

    pma_get(bt(ep, 0), data, n);
    *len = n;

    e ^= DTOG_TX;
    e = (uint16_t)((e & ~STAT_TX) | TX_NAK | CTR_TX);
    epr[ep] = e;
    host_in_toggle[ep] ^= 1u;
    return SIM_ACK;
}

void sim_reset_host_toggle(uint8_t ep_address) {
    if (ep_address & 0x80u) {
        host_in_toggle[ep_address & 0x7u] = 0;
    } else {
        host_out_toggle[ep_address & 0x7u] = 0;
    }
}

void sim_before_next_write(uint8_t ep, void (*fn)(void)) {
    hook_ep = ep;
    hook_fn = fn;
}

bool sim_connected(void) {
    return connected;
}

uint8_t sim_daddr(void) {
    return daddr;
}

int sim_take_errors(void) {
    int n = errors;
    errors = 0;
    return n;
}

void sim_set_quiet(bool q) {
    quiet = q;
}
