/*
 * Host-side tests for usb/usb_cdc.c, run against the peripheral model in
 * usb_sim.c. The host side behaves like a USB host stack: it issues one
 * transaction at a time and, like the firmware main loop, the device gets a
 * usb_poll() call between transactions.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "usb_cdc.h"
#include "usb_sim.h"

static int checks;
static int failures;

#define CHECK(cond)                                                              \
    do {                                                                         \
        checks++;                                                                \
        if (!(cond)) {                                                           \
            failures++;                                                          \
            fprintf(stderr, "%s:%d: CHECK(%s) failed\n", __FILE__, __LINE__, #cond); \
        }                                                                        \
    } while (0)

/* Fails the current test if the peripheral model saw a protocol violation */
#define CHECK_MODEL() CHECK(sim_take_errors() == 0)

#define LO(x) ((uint8_t)((x) & 0xFFu))
#define HI(x) ((uint8_t)(((x) >> 8) & 0xFFu))

enum { STALLED = -1, PROTOCOL_ERROR = -2 };

static uint8_t address;

static sim_handshake_t in_token(uint8_t ep, uint8_t *buf, uint16_t *len) {
    sim_handshake_t h = SIM_NAK;
    for (int i = 0; i < 4 && h == SIM_NAK; i++) {
        h = sim_in(address, ep, buf, len);
        if (h == SIM_NAK) {
            usb_poll();
        }
    }
    return h;
}

static sim_handshake_t out_token(uint8_t ep, const uint8_t *buf, uint16_t len) {
    sim_handshake_t h = SIM_NAK;
    for (int i = 0; i < 4 && h == SIM_NAK; i++) {
        h = sim_out(address, ep, buf, len);
        if (h == SIM_NAK) {
            usb_poll();
        }
    }
    return h;
}

/* One complete control transfer. Returns the data stage length, STALLED or
 * PROTOCOL_ERROR. */
static int control(uint8_t type, uint8_t request, uint16_t value, uint16_t index, uint16_t length,
                   uint8_t *data) {
    const uint8_t setup[8] = {type, request, LO(value), HI(value), LO(index), HI(index), LO(length),
                              HI(length)};
    uint8_t packet[64];
    uint16_t done = 0;
    uint16_t n;
    sim_handshake_t h;

    if (sim_setup(address, setup) != SIM_ACK) {
        return PROTOCOL_ERROR;
    }
    usb_poll();

    if (length && (type & 0x80u)) {
        for (;;) {
            h = in_token(0, packet, &n);
            if (h == SIM_STALL) {
                return STALLED;
            }
            if (h != SIM_ACK || done + n > length) {
                return PROTOCOL_ERROR;
            }
            memcpy(data + done, packet, n);
            done = (uint16_t)(done + n);
            usb_poll();
            if (n < 64u || done == length) {
                break;
            }
        }
        if (out_token(0, NULL, 0) != SIM_ACK) {
            return PROTOCOL_ERROR;
        }
        usb_poll();
        return done;
    }

    while (done < length) {
        n = (uint16_t)(length - done);
        if (n > 64u) {
            n = 64u;
        }
        h = out_token(0, data + done, n);
        if (h == SIM_STALL) {
            return STALLED;
        }
        if (h != SIM_ACK) {
            return PROTOCOL_ERROR;
        }
        done = (uint16_t)(done + n);
        usb_poll();
    }

    h = in_token(0, packet, &n);
    if (h == SIM_STALL) {
        return STALLED;
    }
    if (h != SIM_ACK || n != 0) {
        return PROTOCOL_ERROR;
    }
    usb_poll();
    return done;
}

static int get_descriptor(uint8_t type, uint8_t index, uint16_t length, uint8_t *buf) {
    return control(0x80, 6, (uint16_t)((type << 8) | index), 0, length, buf);
}

/* Read bulk IN packets until the device NAKs; returns total bytes, with the
 * sizes of the packets seen in `sizes` */
static int bulk_read(uint8_t *buf, int max, int *sizes, int *count) {
    int total = 0;
    *count = 0;
    for (;;) {
        uint8_t packet[64];
        uint16_t n;
        sim_handshake_t h = sim_in(address, 1, packet, &n);
        if (h == SIM_NAK) {
            usb_poll();
            h = sim_in(address, 1, packet, &n);
            if (h == SIM_NAK) {
                return total;
            }
        }
        if (h != SIM_ACK || total + n > max) {
            return PROTOCOL_ERROR;
        }
        memcpy(buf + total, packet, n);
        total += n;
        sizes[(*count)++] = n;
        usb_poll();
    }
}

static void enumerate(void) {
    uint8_t buf[256];

    usb_init();
    sim_bus_reset();
    usb_poll();
    address = 0;
    CHECK(get_descriptor(1, 0, 64, buf) == 18);
    CHECK(control(0x00, 5, 9, 0, 0, NULL) == 0); /* SET_ADDRESS 9 */
    address = 9;
    CHECK(control(0x00, 9, 1, 0, 0, NULL) == 0); /* SET_CONFIGURATION 1 */
    sim_reset_host_toggle(0x81);
    sim_reset_host_toggle(0x02);
    sim_reset_host_toggle(0x83);
    CHECK(usb_cdc_configured());
    CHECK_MODEL();
}

static void drain_rx(void) {
    while (usb_cdc_getc() >= 0) {
    }
}

/* ------------------------------------------------------------------------ */

static void test_enumeration(void) {
    uint8_t buf[256];
    uint8_t setup[8] = {0x80, 6, 0, 1, 0, 0, 18, 0};
    int n;

    usb_init();
    CHECK(sim_connected());
    CHECK(!usb_cdc_configured());

    sim_bus_reset();
    usb_poll();
    CHECK(sim_daddr() == 0x80);

    /* Windows-style first request: device descriptor with wLength 64 */
    address = 0;
    n = get_descriptor(1, 0, 64, buf);
    CHECK(n == 18);
    CHECK(buf[0] == 18 && buf[1] == 1);
    CHECK(buf[4] == 0x02);                    /* CDC device class */
    CHECK(buf[7] == 64);                      /* EP0 max packet */
    CHECK(buf[8] == 0x83 && buf[9] == 0x04);  /* VID 0483 */
    CHECK(buf[10] == 0x40 && buf[11] == 0x57); /* PID 5740 */
    CHECK(buf[17] == 1);

    /* Host resets again, then assigns an address */
    sim_bus_reset();
    usb_poll();
    CHECK(control(0x00, 5, 9, 0, 0, NULL) == 0);
    CHECK(sim_daddr() == (0x80 | 9));
    CHECK(sim_setup(0, setup) == SIM_TIMEOUT); /* no longer answers at 0 */
    address = 9;

    CHECK(get_descriptor(1, 0, 18, buf) == 18);

    /* Configuration: header first, then everything */
    CHECK(get_descriptor(2, 0, 9, buf) == 9);
    CHECK((buf[2] | (buf[3] << 8)) == 67);
    n = get_descriptor(2, 0, 255, buf);
    CHECK(n == 67);
    if (n == 67) {
        int pos = 0, interfaces = 0, endpoints = 0;
        uint8_t ep_addrs[3] = {0};
        while (pos < n) {
            CHECK(buf[pos] >= 2 && pos + buf[pos] <= n);
            if (buf[pos] < 2) {
                break;
            }
            if (buf[pos + 1] == 4) {
                interfaces++;
            }
            if (buf[pos + 1] == 5 && endpoints < 3) {
                ep_addrs[endpoints++] = buf[pos + 2];
            }
            pos += buf[pos];
        }
        CHECK(pos == n);
        CHECK(interfaces == 2 && endpoints == 3);
        CHECK(ep_addrs[0] == 0x83 && ep_addrs[1] == 0x02 && ep_addrs[2] == 0x81);
        CHECK(buf[7] == 0x80); /* bus powered */
    }

    /* Before configuration: no CDC requests, GET_CONFIGURATION is 0 */
    CHECK(control(0x80, 8, 0, 0, 1, buf) == 1 && buf[0] == 0);
    CHECK(control(0xA1, 0x21, 0, 0, 7, buf) == STALLED);
    CHECK(control(0x81, 0, 0, 0, 2, buf) == STALLED); /* interface status */

    CHECK(control(0x00, 9, 1, 0, 0, NULL) == 0);
    CHECK(usb_cdc_configured());
    CHECK(control(0x80, 8, 0, 0, 1, buf) == 1 && buf[0] == 1);
    CHECK(control(0x00, 5, 3, 0, 0, NULL) == STALLED); /* SET_ADDRESS once configured */
    CHECK_MODEL();
}

static void test_strings(void) {
    uint8_t buf[256];
    char expected[25];
    int n, i;

    enumerate();

    n = get_descriptor(3, 0, 255, buf);
    CHECK(n == 4 && buf[2] == 0x09 && buf[3] == 0x04);

    n = get_descriptor(3, 1, 255, buf);
    CHECK(n == 2 + 2 * 7 && buf[1] == 3 && buf[2] == 'T' && buf[3] == 0);

    n = get_descriptor(3, 2, 255, buf);
    CHECK(n == 2 + 2 * 5 && buf[2] == 'm' && buf[10] == 'N');

    n = get_descriptor(3, 3, 255, buf);
    CHECK(n == 2 + 2 * 24);
    for (i = 0; i < 12; i++) {
        snprintf(expected + 2 * i, 3, "%02X", sim_uid[i]);
    }
    for (i = 0; i < 24 && n == 50; i++) {
        CHECK(buf[2 + 2 * i] == (uint8_t)expected[i] && buf[3 + 2 * i] == 0);
    }

    /* Truncated to wLength, e.g. Windows asking for the header only */
    CHECK(get_descriptor(3, 3, 2, buf) == 2 && buf[0] == 50);

    CHECK(get_descriptor(3, 4, 255, buf) == STALLED);
    CHECK_MODEL();
}

static void test_stalls_recover(void) {
    uint8_t buf[256];

    enumerate();
    CHECK(get_descriptor(6, 0, 10, buf) == STALLED);     /* device qualifier: FS only */
    CHECK(get_descriptor(1, 0, 18, buf) == 18);          /* next request still works */
    CHECK(control(0xC0, 0x01, 0, 0, 4, buf) == STALLED); /* vendor request */
    CHECK(control(0x40, 0x01, 0, 0, 0, NULL) == STALLED);
    CHECK(control(0x00, 3, 1, 0, 0, NULL) == STALLED); /* SET_FEATURE remote wakeup */
    CHECK(control(0x80, 0, 0, 0, 2, buf) == 2 && buf[0] == 0 && buf[1] == 0);
    CHECK(control(0x81, 10, 0, 1, 1, buf) == 1 && buf[0] == 0); /* GET_INTERFACE */
    CHECK(control(0x01, 11, 1, 1, 0, NULL) == STALLED);         /* SET_INTERFACE alt 1 */
    CHECK(control(0x01, 11, 0, 1, 0, NULL) == 0);
    CHECK_MODEL();
}

static void test_cdc_requests(void) {
    uint8_t buf[16];
    uint8_t coding[7] = {0xC0, 0xC6, 0x2D, 0x00, 0, 0, 8}; /* 3,000,000 8N1 */

    enumerate();
    CHECK(control(0xA1, 0x21, 0, 0, 7, buf) == 7);
    CHECK(buf[0] == 0x00 && buf[1] == 0xC2 && buf[2] == 0x01 && buf[6] == 8); /* default 115200 */

    CHECK(control(0x21, 0x20, 0, 0, 7, coding) == 7);
    CHECK(control(0xA1, 0x21, 0, 0, 7, buf) == 7 && memcmp(buf, coding, 7) == 0);
    CHECK(control(0x21, 0x20, 0, 0, 6, coding) == STALLED); /* wrong length */

    CHECK(!usb_cdc_dtr());
    CHECK(control(0x21, 0x22, 3, 0, 0, NULL) == 0);
    CHECK(usb_cdc_dtr());
    CHECK(control(0x21, 0x22, 0, 0, 0, NULL) == 0);
    CHECK(!usb_cdc_dtr());
    CHECK(control(0x21, 0x23, 0xFFFF, 0, 0, NULL) == 0); /* SEND_BREAK */
    CHECK(control(0x21, 0x22, 1, 1, 0, NULL) == STALLED); /* data interface */
    CHECK_MODEL();
}

static void test_bulk_out(void) {
    uint8_t packet[64];
    int i;

    enumerate();
    CHECK(usb_cdc_getc() == -1);

    CHECK(sim_out(address, 2, (const uint8_t *)"O\r", 2) == SIM_ACK);
    usb_poll();
    CHECK(usb_cdc_getc() == 'O');
    CHECK(usb_cdc_getc() == '\r');
    CHECK(usb_cdc_getc() == -1);

    /* Flow control: the host is NAKed until the previous packet is consumed */
    for (i = 0; i < 64; i++) {
        packet[i] = (uint8_t)i;
    }
    CHECK(sim_out(address, 2, packet, 64) == SIM_ACK);
    usb_poll();
    CHECK(sim_out(address, 2, packet, 10) == SIM_NAK);
    for (i = 0; i < 63; i++) {
        CHECK(usb_cdc_getc() == i);
    }
    usb_poll();
    CHECK(sim_out(address, 2, packet, 10) == SIM_NAK); /* one byte still unread */
    CHECK(usb_cdc_getc() == 63);
    CHECK(sim_out(address, 2, packet, 10) == SIM_ACK);
    usb_poll();
    drain_rx();

    /* A zero-length packet is absorbed and doesn't block the endpoint */
    CHECK(sim_out(address, 2, NULL, 0) == SIM_ACK);
    usb_poll();
    CHECK(usb_cdc_getc() == -1);
    CHECK(sim_out(address, 2, (const uint8_t *)"x", 1) == SIM_ACK);
    usb_poll();
    CHECK(usb_cdc_getc() == 'x');
    CHECK_MODEL();
}

static void test_bulk_in_zlp(void) {
    uint8_t data[300], got[1100];
    int sizes[32], count, n, i;

    enumerate();
    for (i = 0; i < (int)sizeof(data); i++) {
        data[i] = (uint8_t)(i * 7);
    }

    CHECK(usb_cdc_write(data, 10));
    n = bulk_read(got, sizeof(got), sizes, &count);
    CHECK(n == 10 && count == 1 && memcmp(got, data, 10) == 0);

    /* Exactly one full packet must be followed by a zero-length packet */
    CHECK(usb_cdc_write(data, 64));
    n = bulk_read(got, sizeof(got), sizes, &count);
    CHECK(n == 64 && count == 2 && sizes[0] == 64 && sizes[1] == 0);

    CHECK(usb_cdc_write(data, 128));
    n = bulk_read(got, sizeof(got), sizes, &count);
    CHECK(n == 128 && count == 3 && sizes[2] == 0 && memcmp(got, data, 128) == 0);

    CHECK(usb_cdc_write(data, 150));
    n = bulk_read(got, sizeof(got), sizes, &count);
    CHECK(n == 150 && count == 3 && sizes[0] == 64 && sizes[1] == 64 && sizes[2] == 22);
    CHECK(memcmp(got, data, 150) == 0);

    /* Data queued while a full packet is in flight continues the transfer
     * instead of being preceded by a ZLP */
    CHECK(usb_cdc_write(data, 64));
    CHECK(usb_cdc_write(data + 64, 5));
    n = bulk_read(got, sizeof(got), sizes, &count);
    CHECK(n == 69 && count == 2 && sizes[0] == 64 && sizes[1] == 5);
    CHECK_MODEL();
}

static void test_bulk_in_ring(void) {
    static uint8_t sent[20000], got[20000];
    uint8_t line[40];
    int sizes[400], count, n;
    int sent_len = 0, got_len = 0, dropped = 0;
    unsigned seed = 1;

    enumerate();

    /* All-or-nothing writes, never more than the ring holds */
    CHECK(usb_cdc_write_space() == 1024);
    CHECK(!usb_cdc_write(sent, 1025));
    CHECK(usb_cdc_write_space() == 1024);

    /* Lines of random length while the host reads irregularly: the stream
     * that arrives is exactly the lines that were accepted, in order */
    while (sent_len < 15000) {
        int len = 1 + (int)((seed = seed * 1103515245u + 12345u) >> 16) % 27;
        for (int i = 0; i < len; i++) {
            line[i] = (uint8_t)(sent_len + i);
        }
        line[len - 1] = '\r';
        if (usb_cdc_write(line, (uint16_t)len)) {
            memcpy(sent + sent_len, line, (size_t)len);
            sent_len += len;
        } else {
            dropped++;
        }
        if (((seed >> 20) & 127u) == 0) {
            n = bulk_read(got + got_len, (int)sizeof(got) - got_len, sizes, &count);
            CHECK(n >= 0);
            if (n < 0) {
                return;
            }
            got_len += n;
        }
        usb_poll();
    }
    n = bulk_read(got + got_len, (int)sizeof(got) - got_len, sizes, &count);
    CHECK(n >= 0);
    got_len += n > 0 ? n : 0;

    CHECK(dropped > 0); /* the host fell behind at least once */
    CHECK(got_len == sent_len);
    CHECK(memcmp(got, sent, (size_t)sent_len) == 0);
    CHECK(usb_cdc_write_space() == 1024);
    CHECK_MODEL();
}

static void test_halt(void) {
    uint8_t buf[64];
    uint16_t n;
    int sizes[8], count;

    enumerate();

    /* IN endpoint */
    CHECK(control(0x02, 3, 0, 0x81, 0, NULL) == 0);
    CHECK(control(0x82, 0, 0, 0x81, 2, buf) == 2 && buf[0] == 1);
    CHECK(usb_cdc_write((const uint8_t *)"abc", 3));
    usb_poll();
    CHECK(sim_in(address, 1, buf, &n) == SIM_STALL);
    CHECK(control(0x02, 1, 0, 0x81, 0, NULL) == 0);
    sim_reset_host_toggle(0x81);
    CHECK(control(0x82, 0, 0, 0x81, 2, buf) == 2 && buf[0] == 0);
    usb_poll();
    CHECK(bulk_read(buf, sizeof(buf), sizes, &count) == 3 && memcmp(buf, "abc", 3) == 0);

    /* OUT endpoint, and the toggle resets on CLEAR_FEATURE */
    CHECK(sim_out(address, 2, (const uint8_t *)"1", 1) == SIM_ACK); /* DATA0 */
    usb_poll();
    drain_rx();
    CHECK(control(0x02, 3, 0, 0x02, 0, NULL) == 0);
    CHECK(sim_out(address, 2, (const uint8_t *)"2", 1) == SIM_STALL);
    CHECK(control(0x02, 1, 0, 0x02, 0, NULL) == 0);
    sim_reset_host_toggle(0x02);
    CHECK(sim_out(address, 2, (const uint8_t *)"3", 1) == SIM_ACK); /* DATA0 again */
    usb_poll();
    CHECK(usb_cdc_getc() == '3');

    CHECK(control(0x02, 3, 0, 0x85, 0, NULL) == STALLED); /* no such endpoint */
    CHECK(control(0x82, 0, 0, 0x85, 2, buf) == STALLED);
    CHECK_MODEL();
}

static void check_disable_retry_mask(void) {
    CHECK(sim_irq_masked());
}

static void bulk_in_during_disable(void) {
    uint8_t buf[64];
    uint16_t n;

    /* The status toggle/readback must not be interrupted by the CAN ISR. */
    CHECK(sim_irq_masked());
    CHECK(sim_in(address, 1, buf, &n) == SIM_ACK && n == 3);
    CHECK(memcmp(buf, "abc", 3) == 0);
    sim_before_next_write(1, check_disable_retry_mask);
}

static void test_halt_pending_packets(void) {
    uint8_t data[130], buf[160];
    uint16_t n;
    int sizes[8], count;
    const uint16_t lengths[] = {3, 64, 130};

    for (size_t i = 0; i < sizeof(data); i++) {
        data[i] = (uint8_t)i;
    }
    for (size_t i = 0; i < sizeof(lengths) / sizeof(lengths[0]); i++) {
        enumerate();
        CHECK(usb_cdc_write(data, lengths[i]));
        /* Halt after the first packet has entered PMA, before its host IN. */
        CHECK(control(0x02, 3, 0, 0x81, 0, NULL) == 0);
        CHECK(sim_in(address, 1, buf, &n) == SIM_STALL);
        CHECK(control(0x02, 1, 0, 0x81, 0, NULL) == 0);
        sim_reset_host_toggle(0x81);
        CHECK(bulk_read(buf, sizeof(buf), sizes, &count) == lengths[i]);
        CHECK(memcmp(buf, data, lengths[i]) == 0);
        CHECK(usb_cdc_tx_empty());
        CHECK(!sim_irq_masked());
        CHECK_MODEL();
    }

    enumerate();
    CHECK(usb_cdc_write((const uint8_t *)"abc", 3));
    CHECK(sim_in(address, 1, buf, &n) == SIM_ACK && n == 3);
    /* EP0 is serviced before this bulk completion; clearing halt must not
     * replay a packet the host already acknowledged. */
    CHECK(control(0x02, 1, 0, 0x81, 0, NULL) == 0);
    sim_reset_host_toggle(0x81);
    CHECK(sim_in(address, 1, buf, &n) == SIM_NAK);
    CHECK(usb_cdc_tx_empty());

    /* Completion can also arrive during the first endpoint status write. */
    CHECK(usb_cdc_write((const uint8_t *)"abc", 3));
    sim_before_next_write(1, bulk_in_during_disable);
    CHECK(control(0x02, 1, 0, 0x81, 0, NULL) == 0);
    sim_reset_host_toggle(0x81);
    CHECK(sim_in(address, 1, buf, &n) == SIM_NAK);
    CHECK(usb_cdc_tx_empty());

    /* Clearing a full packet's completion still owes its terminating ZLP. */
    CHECK(usb_cdc_write(data, 64));
    CHECK(sim_in(address, 1, buf, &n) == SIM_ACK && n == 64);
    CHECK(control(0x02, 1, 0, 0x81, 0, NULL) == 0);
    sim_reset_host_toggle(0x81);
    CHECK(sim_in(address, 1, buf, &n) == SIM_ACK && n == 0);
    usb_poll();
    CHECK(usb_cdc_tx_empty());

    /* The same control priority must not erase an acknowledged OUT packet. */
    CHECK(sim_out(address, 2, (const uint8_t *)"O\r", 2) == SIM_ACK);
    CHECK(control(0x02, 1, 0, 0x02, 0, NULL) == 0);
    sim_reset_host_toggle(0x02);
    CHECK(usb_cdc_getc() == 'O' && usb_cdc_getc() == '\r');
    CHECK(usb_cdc_getc() == -1);
    CHECK(!sim_irq_masked());
    CHECK_MODEL();
}

static void test_set_interface_resets_endpoints(void) {
    uint8_t buf[64];
    uint16_t n;

    enumerate();
    CHECK(sim_out(address, 2, (const uint8_t *)"x", 1) == SIM_ACK);
    usb_poll();
    CHECK(usb_cdc_getc() == 'x');
    CHECK(usb_cdc_write((const uint8_t *)"y", 1));
    CHECK(sim_in(address, 1, buf, &n) == SIM_ACK && n == 1);
    usb_poll();

    /* Both bulk toggles are now DATA1. Selecting alt 0 again resets them. */
    CHECK(control(0x01, 11, 0, 1, 0, NULL) == 0);
    sim_reset_host_toggle(0x02);
    sim_reset_host_toggle(0x81);
    CHECK(sim_out(address, 2, (const uint8_t *)"O\r", 2) == SIM_ACK);
    usb_poll();
    CHECK(usb_cdc_getc() == 'O' && usb_cdc_getc() == '\r');
    CHECK(usb_cdc_write((const uint8_t *)"z\r", 2));
    CHECK(sim_in(address, 1, buf, &n) == SIM_ACK && n == 2);
    CHECK(memcmp(buf, "z\r", 2) == 0);
    usb_poll();

    CHECK(control(0x02, 3, 0, 0x81, 0, NULL) == 0);
    CHECK(control(0x02, 3, 0, 0x02, 0, NULL) == 0);
    CHECK(control(0x02, 3, 0, 0x83, 0, NULL) == 0);
    CHECK(control(0x01, 11, 0, 0, 0, NULL) == 0);
    CHECK(control(0x82, 0, 0, 0x83, 2, buf) == 2 && buf[0] == 0);
    CHECK(control(0x82, 0, 0, 0x81, 2, buf) == 2 && buf[0] == 1);
    CHECK(control(0x82, 0, 0, 0x02, 2, buf) == 2 && buf[0] == 1);
    CHECK(control(0x01, 11, 0, 1, 0, NULL) == 0);
    sim_reset_host_toggle(0x02);
    sim_reset_host_toggle(0x81);
    CHECK(control(0x82, 0, 0, 0x81, 2, buf) == 2 && buf[0] == 0);
    CHECK(control(0x82, 0, 0, 0x02, 2, buf) == 2 && buf[0] == 0);
    CHECK(!sim_irq_masked());
    CHECK_MODEL();
}

static void test_reset_and_reconfigure(void) {
    uint8_t buf[64];

    enumerate();
    CHECK(control(0x21, 0x22, 1, 0, 0, NULL) == 0);
    CHECK(usb_cdc_write((const uint8_t *)"stale", 5));
    CHECK(sim_out(address, 2, (const uint8_t *)"half", 4) == SIM_ACK);
    usb_poll();

    /* Bus reset mid-session: everything is dropped, nothing sticks */
    sim_bus_reset();
    usb_poll();
    CHECK(!usb_cdc_configured());
    CHECK(!usb_cdc_dtr());
    CHECK(usb_cdc_getc() == -1);
    CHECK(!usb_cdc_write((const uint8_t *)"x", 1));
    CHECK(usb_cdc_write_space() == 0);

    address = 0;
    CHECK(control(0x00, 5, 4, 0, 0, NULL) == 0);
    address = 4;
    CHECK(control(0x00, 9, 1, 0, 0, NULL) == 0);
    CHECK(usb_cdc_configured());
    usb_poll();
    CHECK(usb_cdc_write_space() == 1024);

    /* SET_CONFIGURATION 0 deconfigures */
    CHECK(control(0x00, 9, 0, 0, 0, NULL) == 0);
    CHECK(!usb_cdc_configured());
    CHECK(sim_out(address, 2, (const uint8_t *)"x", 1) == SIM_TIMEOUT);
    CHECK(control(0x00, 9, 2, 0, 0, NULL) == STALLED);
    CHECK(get_descriptor(1, 0, 18, buf) == 18);
    CHECK_MODEL();
}

static void test_tx_empty_and_disconnect(void) {
    uint8_t buf[64], data[64] = {0};
    uint16_t n;
    int sizes[4], count;

    enumerate();
    CHECK(usb_cdc_tx_empty());
    CHECK(usb_cdc_write((const uint8_t *)"\r", 1));
    CHECK(!usb_cdc_tx_empty());
    CHECK(bulk_read(buf, sizeof(buf), sizes, &count) == 1);
    CHECK(usb_cdc_tx_empty());

    /* After a full packet, not empty until the zero-length packet has gone */
    CHECK(usb_cdc_write(data, 64));
    CHECK(sim_in(address, 1, buf, &n) == SIM_ACK && n == 64);
    usb_poll();
    CHECK(!usb_cdc_tx_empty());
    CHECK(sim_in(address, 1, buf, &n) == SIM_ACK && n == 0);
    usb_poll();
    CHECK(usb_cdc_tx_empty());

    usb_disconnect();
    CHECK(!sim_connected());
    CHECK(!usb_cdc_configured());
    CHECK(!usb_cdc_write((const uint8_t *)"x", 1));
    CHECK_MODEL();
}

/* The host's status-stage OUT completes while the driver is clearing the
 * IN completion flag on the same endpoint register. The original HAL macro
 * wrote the other CTR bit back as read and lost this event. */
static void status_out_now(void) {
    CHECK(sim_out(address, 0, NULL, 0) == SIM_ACK);
}

static void test_ctr_race(void) {
    const uint8_t setup[8] = {0x80, 6, 0, 1, 0, 0, 18, 0};
    uint8_t buf[64];
    uint16_t n;

    enumerate();
    CHECK(sim_setup(address, setup) == SIM_ACK);
    usb_poll();
    CHECK(sim_in(address, 0, buf, &n) == SIM_ACK && n == 18);
    sim_before_next_write(0, status_out_now);
    usb_poll();
    CHECK_MODEL();
    CHECK(get_descriptor(1, 0, 18, buf) == 18);

    /* Status OUT and the final IN completion both pending at one poll */
    CHECK(sim_setup(address, setup) == SIM_ACK);
    usb_poll();
    CHECK(sim_in(address, 0, buf, &n) == SIM_ACK && n == 18);
    CHECK(sim_out(address, 0, NULL, 0) == SIM_ACK);
    usb_poll();
    CHECK_MODEL();
    CHECK(get_descriptor(1, 0, 18, buf) == 18);
    CHECK_MODEL();

    /* Negative control: the old HAL clear (CTR_RX written back as read)
     * must be flagged by the model */
    {
        uint16_t r = hw_epr_get(0);
        CHECK(sim_setup(address, setup) == SIM_ACK); /* sets CTR_RX after the read */
        sim_set_quiet(true);
        hw_epr_set(0, (uint16_t)(r & 0xFF7Fu & 0x8F8Fu));
        sim_set_quiet(false);
        CHECK(sim_take_errors() == 1);
        usb_init();
    }
}

/* Regression for hardware: the driver used to set EP0 to DISABLED while it
 * handled a request. A disabled endpoint gives no handshake; hosts treat that
 * as a transaction error and abort the transfer after three, so requests
 * with slower handlers (string descriptors) failed on a real adapter. Here a
 * host token is sent at every EP0 register write and must get NAK or ACK. */
static char probe_kind; /* 'I' IN token, 'O' OUT token, 0 done */
static int probe_skip, probe_silent;

static void probe_host_token(void) {
    uint8_t buf[64];
    uint16_t n;
    sim_handshake_t h;

    if (probe_skip > 0) {
        probe_skip--;
        sim_before_next_write(0, probe_host_token);
        return;
    }
    h = probe_kind == 'I' ? sim_in(address, 0, buf, &n) : sim_out(address, 0, NULL, 0);
    if (h == SIM_TIMEOUT) {
        probe_silent++;
    }
    if (h == SIM_ACK) {
        probe_kind = 0; /* the host got its packet; stop probing */
        return;
    }
    sim_before_next_write(0, probe_host_token);
}

static void test_endpoint_zero_always_handshakes(void) {
    static const uint16_t requests[][3] = {
        {0x0100, 0, 18},        /* device descriptor (constant) */
        {0x0301, 0x0409, 255},  /* manufacturer string (built per request) */
        {0x0303, 0x0409, 255},  /* serial number string (formats the UID) */
        {0x0303, 0x0409, 2},    /* its 2-byte header, as hosts ask first */
    };
    uint8_t buf[64];
    uint16_t n;

    enumerate();
    for (size_t r = 0; r < sizeof(requests) / sizeof(requests[0]); r++) {
        for (int skip = 0; skip < 4; skip++) {
            const uint8_t setup[8] = {0x80, 6, LO(requests[r][0]), HI(requests[r][0]),
                                      LO(requests[r][1]), HI(requests[r][1]),
                                      LO(requests[r][2]), HI(requests[r][2])};
            uint16_t expected = requests[r][2] < 18 ? requests[r][2] : 0;

            /* Data stage: IN tokens while the SETUP is being handled */
            CHECK(sim_setup(address, setup) == SIM_ACK);
            probe_kind = 'I';
            probe_skip = skip;
            probe_silent = 0;
            sim_before_next_write(0, probe_host_token);
            usb_poll();
            sim_before_next_write(0, NULL);
            CHECK(probe_silent == 0);
            if (probe_kind) {
                CHECK(sim_in(address, 0, buf, &n) == SIM_ACK);
                CHECK(n > 0 && (!expected || n == expected) && buf[1] == (r ? 3 : 1));
            }

            /* Status stage: OUT tokens while the IN completion is handled */
            probe_kind = 'O';
            probe_skip = skip;
            probe_silent = 0;
            sim_before_next_write(0, probe_host_token);
            usb_poll();
            sim_before_next_write(0, NULL);
            CHECK(probe_silent == 0);
            if (probe_kind) {
                CHECK(sim_out(address, 0, NULL, 0) == SIM_ACK);
            }
            usb_poll();
            probe_kind = 0;
            CHECK_MODEL();
        }
    }
    CHECK(get_descriptor(1, 0, 18, buf) == 18);
    CHECK_MODEL();
}

static const uint8_t abort_setup[8] = {0x80, 6, 0, 1, 0, 0, 18, 0};
static int setup_write_count;
static bool setup_accepted, setup_reply_seen;

/* Try the new request's IN at each register write while SETUP is handled.
 * Any accepted packet must belong to the new request, never the old tail. */
static void probe_setup_reply(void) {
    uint8_t buf[64];
    uint16_t n;
    sim_handshake_t h = sim_in(address, 0, buf, &n);

    /* NAK while the driver works; never silence, which hosts count as an
     * error (three in a row abort the transfer) */
    CHECK(h == SIM_NAK || h == SIM_ACK);
    if (h == SIM_ACK) {
        CHECK(n == 18 && buf[0] == 18 && buf[1] == 1);
        setup_reply_seen = true;
    } else {
        sim_before_next_write(0, probe_setup_reply);
    }
}

static void abort_setup_at_write(void) {
    sim_handshake_t h;

    if (--setup_write_count) {
        sim_before_next_write(0, abort_setup_at_write);
        return;
    }
    /* EP0 only ever NAKs while the driver works, so SETUP is always taken.
     * A real host needs microseconds before following it with an IN, so the
     * reply is checked once usb_poll() returns, not mid-sequence. */
    h = sim_setup(address, abort_setup);
    CHECK(h == SIM_ACK);
    setup_accepted = h == SIM_ACK;
}

static void test_setup_aborts_pending_in(void) {
    const uint8_t config_setup[8] = {0x80, 6, 0, 2, 0, 0, 67, 0};
    uint8_t buf[64];
    uint16_t n;

    /* Already pending, or arriving at each register write between an IN
     * completion and arming the next descriptor packet (there are three;
     * the fourth case injects nothing and sends the SETUP afterwards). */
    for (int write = 0; write <= 4; write++) {
        enumerate();
        CHECK(sim_setup(address, config_setup) == SIM_ACK);
        usb_poll();
        CHECK(sim_in(address, 0, buf, &n) == SIM_ACK && n == 64);
        setup_accepted = setup_reply_seen = false;
        if (!write) {
            CHECK(sim_setup(address, abort_setup) == SIM_ACK);
            setup_accepted = true;
            sim_before_next_write(0, probe_setup_reply);
        } else {
            setup_write_count = write;
            sim_before_next_write(0, abort_setup_at_write);
        }
        usb_poll();
        sim_before_next_write(0, NULL);
        if (!setup_accepted) {
            CHECK(sim_setup(address, abort_setup) == SIM_ACK);
            usb_poll();
        }
        if (!setup_reply_seen) {
            CHECK(sim_in(address, 0, buf, &n) == SIM_ACK && n == 18);
            CHECK(buf[0] == 18 && buf[1] == 1);
        }
        CHECK(sim_out(address, 0, NULL, 0) == SIM_ACK);
        usb_poll();
        CHECK(!sim_irq_masked());
        CHECK_MODEL();
    }

    /* Preserve the caller's interrupt mask, including an already masked
     * caller, rather than enabling interrupts unconditionally. */
    (void)hw_irq_save();
    CHECK(get_descriptor(1, 0, 18, buf) == 18);
    CHECK(sim_irq_masked());
    hw_irq_restore(0);
    CHECK_MODEL();
}

static void test_address_status_before_setup(void) {
    const uint8_t set_address[8] = {0, 5, 9, 0, 0, 0, 0, 0};
    uint8_t buf[64];
    uint16_t n;

    usb_init();
    sim_bus_reset();
    usb_poll();
    address = 0;
    CHECK(sim_setup(address, set_address) == SIM_ACK);
    usb_poll();
    CHECK(sim_in(address, 0, buf, &n) == SIM_ACK && n == 0);
    CHECK(sim_setup(address, abort_setup) == SIM_ACK);
    usb_poll();
    CHECK(sim_daddr() == (0x80 | 9));
    address = 9;
    CHECK(sim_in(address, 0, buf, &n) == SIM_ACK && n == 18);
    CHECK(buf[0] == 18 && buf[1] == 1);
    CHECK(sim_out(address, 0, NULL, 0) == SIM_ACK);
    usb_poll();
    CHECK(!sim_irq_masked());
    CHECK_MODEL();
}

int main(void) {
    struct {
        const char *name;
        void (*fn)(void);
    } tests[] = {
        {"enumeration", test_enumeration},
        {"strings", test_strings},
        {"stalls_recover", test_stalls_recover},
        {"cdc_requests", test_cdc_requests},
        {"bulk_out", test_bulk_out},
        {"bulk_in_zlp", test_bulk_in_zlp},
        {"bulk_in_ring", test_bulk_in_ring},
        {"halt", test_halt},
        {"halt_pending_packets", test_halt_pending_packets},
        {"set_interface_resets", test_set_interface_resets_endpoints},
        {"reset_and_reconfigure", test_reset_and_reconfigure},
        {"tx_empty_and_disconnect", test_tx_empty_and_disconnect},
        {"endpoint_zero_always_handshakes", test_endpoint_zero_always_handshakes},
        {"ctr_race", test_ctr_race},
        {"setup_aborts_pending_in", test_setup_aborts_pending_in},
        {"address_status_setup", test_address_status_before_setup},
    };

    for (size_t i = 0; i < sizeof(tests) / sizeof(tests[0]); i++) {
        int before = failures;
        tests[i].fn();
        printf("%-24s %s\n", tests[i].name, failures == before ? "ok" : "FAILED");
    }

    printf("%d checks, %d failed\n", checks, failures);
    return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
