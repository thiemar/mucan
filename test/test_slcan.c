/*
 * Host-side tests for app/slcan.c. The slcan_port_* hooks are fakes: the
 * "host" writes bytes into an input buffer, replies collect in an output
 * buffer, and the CAN side records what it is asked to do.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "slcan.h"

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

/* ---- Fake device ------------------------------------------------------- */

static struct {
    /* host -> device */
    char in[4096];
    size_t in_len, in_pos;
    /* device -> host */
    char out[4096];
    size_t out_len;
    uint16_t write_space;
    /* CAN */
    bool open, listen_only, open_fails;
    uint32_t bitrate;
    int closes;
    can_frame_t sent[64];
    int sent_count;
    int queue_space;
    uint8_t status;
    bool bootloader;
    bool flash_boot;
    bool bootloader_unavailable;
    unsigned option_writes;
    unsigned option_reads;
} dev;

static slcan_t s;
static uint32_t now;

bool slcan_port_set_bitrate(uint32_t bitrate) {
    CHECK(!dev.open);
    dev.bitrate = bitrate;
    return true;
}

bool slcan_port_open(bool listen_only) {
    if (dev.open_fails) {
        return false;
    }
    CHECK(!dev.open);
    dev.open = true;
    dev.listen_only = listen_only;
    return true;
}

void slcan_port_close(void) {
    CHECK(dev.open);
    dev.open = false;
    dev.closes++;
}

bool slcan_port_transmit(const can_frame_t *frame) {
    CHECK(dev.open && !dev.listen_only);
    if (dev.queue_space == 0 || dev.sent_count == 64) {
        return false;
    }
    dev.queue_space--;
    dev.sent[dev.sent_count++] = *frame;
    return true;
}

uint8_t slcan_port_status(void) {
    return dev.status;
}

uint16_t slcan_port_serial(void) {
    return 0xBEEF;
}

bool slcan_port_boot_mode(bool flash_boot) {
    if (dev.bootloader_unavailable) {
        return false;
    }
    dev.bootloader = !flash_boot;
    dev.flash_boot = flash_boot;
    dev.option_writes++;
    return true;
}

void slcan_port_options(uint8_t bytes[SLCAN_OPTION_BYTES]) {
    static const uint8_t options[SLCAN_OPTION_BYTES] = {
        0xAA, 0x55, 0xFF, 0x00, 0x12, 0xED, 0x34, 0xCB, 0xFF, 0x00
    };
    memcpy(bytes, options, sizeof(options));
    dev.option_reads++;
}

int slcan_port_getc(void) {
    return dev.in_pos < dev.in_len ? (unsigned char)dev.in[dev.in_pos++] : -1;
}

uint16_t slcan_port_write_space(void) {
    return dev.write_space;
}

bool slcan_port_write(const uint8_t *data, uint16_t len) {
    CHECK(len <= dev.write_space); /* replies are never dropped for lack of room */
    if (len > dev.write_space || dev.out_len + len > sizeof(dev.out)) {
        return false;
    }
    memcpy(dev.out + dev.out_len, data, len);
    dev.out_len += len;
    dev.write_space = (uint16_t)(dev.write_space - len);
    return true;
}

static void reset_all(void) {
    memset(&dev, 0, sizeof(dev));
    dev.write_space = 1024;
    dev.queue_space = 1000;
    now = 1000;
    slcan_init(&s);
}

/* Send text from the host and run the protocol; returns everything the
 * device replied (and clears it) */
static const char *exchange(const char *text) {
    static char reply[4096];
    size_t n = strlen(text);

    memcpy(dev.in + dev.in_len, text, n);
    dev.in_len += n;
    slcan_service(&s, now);

    memcpy(reply, dev.out, dev.out_len);
    reply[dev.out_len] = '\0';
    dev.write_space = (uint16_t)(dev.write_space + dev.out_len);
    dev.out_len = 0;
    return reply;
}

static bool frame_is(const can_frame_t *f, uint32_t id, uint8_t flags, uint8_t dlc, const char *data) {
    if (f->id != id || f->flags != flags || f->dlc != dlc) {
        return false;
    }
    for (uint8_t i = 0; i < 8; i++) {
        uint8_t expected = i < dlc && data ? (uint8_t)data[i] : 0;
        if (f->data[i] != expected) {
            return false;
        }
    }
    return true;
}

/* ---- Tests ------------------------------------------------------------- */

static void test_info(void) {
    reset_all();
    CHECK(strcmp(exchange("V\r"), "V0001\r") == 0);
    CHECK(strcmp(exchange("v\r"), "v0004\r") == 0);
    CHECK(strcmp(exchange("N\r"), "NBEEF\r") == 0);
    CHECK(strcmp(exchange("\r"), "\a") == 0);        /* empty line */
    CHECK(strcmp(exchange("Z1\r"), "\a") == 0);      /* timestamps unsupported */
    CHECK(strcmp(exchange("M00000000\r"), "\a") == 0);
    CHECK(strcmp(exchange("x\r"), "\a") == 0);
}

static void test_channel(void) {
    reset_all();
    CHECK(strcmp(exchange("C\r"), "\a") == 0); /* already closed */
    CHECK(strcmp(exchange("S6\r"), "\r") == 0 && dev.bitrate == 500000);
    CHECK(strcmp(exchange("S7\r"), "\r") == 0 && dev.bitrate == 800000);
    CHECK(strcmp(exchange("S0\r"), "\r") == 0 && dev.bitrate == 10000);
    CHECK(strcmp(exchange("S9\r"), "\a") == 0);
    CHECK(strcmp(exchange("S\r"), "\a") == 0);
    CHECK(strcmp(exchange("S88\r"), "\a") == 0);
    CHECK(strcmp(exchange("S8\r"), "\r") == 0 && dev.bitrate == 1000000);

    CHECK(strcmp(exchange("O\r"), "\r") == 0 && dev.open && !dev.listen_only);
    CHECK(strcmp(exchange("O\r"), "\a") == 0);
    CHECK(strcmp(exchange("L\r"), "\a") == 0);
    CHECK(strcmp(exchange("S6\r"), "\a") == 0); /* bitrate only while closed */
    CHECK(dev.bitrate == 1000000);
    CHECK(strcmp(exchange("C\r"), "\r") == 0 && !dev.open);

    CHECK(strcmp(exchange("L\r"), "\r") == 0 && dev.open && dev.listen_only);
    CHECK(strcmp(exchange("t1230\r"), "\a") == 0); /* no transmit in listen-only */
    CHECK(dev.sent_count == 0);
    CHECK(strcmp(exchange("C\r"), "\r") == 0);

    CHECK(strcmp(exchange("O1\r"), "\a") == 0);
    dev.open_fails = true; /* e.g. clock not yet synchronised */
    CHECK(strcmp(exchange("O\r"), "\a") == 0);
    CHECK(strcmp(exchange("t1230\r"), "\a") == 0);
}

static void test_transmit(void) {
    reset_all();
    CHECK(strcmp(exchange("t1230\r"), "\a") == 0); /* channel closed */
    exchange("O\r");

    CHECK(strcmp(exchange("t12380011223344556677\r"), "z\r") == 0);
    CHECK(dev.sent_count == 1 &&
          frame_is(&dev.sent[0], 0x123, 0, 8, "\x00\x11\x22\x33\x44\x55\x66\x77"));

    CHECK(strcmp(exchange("T1ABCDEF92AA55\r"), "Z\r") == 0);
    CHECK(frame_is(&dev.sent[1], 0x1ABCDEF9, CAN_FRAME_EXT, 2, "\xAA\x55"));

    CHECK(strcmp(exchange("t7ff1ab\r"), "z\r") == 0); /* lower-case hex */
    CHECK(frame_is(&dev.sent[2], 0x7FF, 0, 1, "\xAB"));

    CHECK(strcmp(exchange("t0000\r"), "z\r") == 0);
    CHECK(frame_is(&dev.sent[3], 0, 0, 0, NULL));

    /* Remote frames carry a length; the legacy form without one means 0 */
    CHECK(strcmp(exchange("r1234\r"), "z\r") == 0);
    CHECK(frame_is(&dev.sent[4], 0x123, CAN_FRAME_RTR, 4, NULL));
    CHECK(strcmp(exchange("r123\r"), "z\r") == 0);
    CHECK(frame_is(&dev.sent[5], 0x123, CAN_FRAME_RTR, 0, NULL));
    CHECK(strcmp(exchange("R1FFFFFFF8\r"), "Z\r") == 0);
    CHECK(frame_is(&dev.sent[6], 0x1FFFFFFF, CAN_FRAME_EXT | CAN_FRAME_RTR, 8, NULL));
    CHECK(dev.sent_count == 7);

    /* Malformed */
    const char *bad[] = {
        "t8000\r",      /* standard ID above 7FF */
        "T200000000\r", /* extended ID above 1FFFFFFF */
        "t12G0\r",      /* not hex */
        "t1239\r",      /* length above 8 */
        "t1232AA\r",    /* too little data */
        "t1231AABB\r",  /* too much data */
        "t1231A\r",     /* odd digit count */
        "t1231ZZ\r",    /* data not hex */
        "t12\r",        /* short ID */
        "t\r",
        "T1230\r",      /* extended ID needs 8 digits */
        "r12345\r",     /* RTR with data */
        "r1239\r",
        "R123\r",
    };
    for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) {
        CHECK(strcmp(exchange(bad[i]), "\a") == 0);
    }
    CHECK(dev.sent_count == 7);
}

static void test_line_handling(void) {
    char longline[80];

    reset_all();
    /* CRLF and bare LF are tolerated */
    CHECK(strcmp(exchange("V\r\nv\r\n"), "V0001\rv0004\r") == 0);
    CHECK(strcmp(exchange("\nN\r"), "NBEEF\r") == 0);

    /* Several commands in one USB packet */
    CHECK(strcmp(exchange("S6\rO\rt1230\rC\r"), "\r\rz\r\r") == 0);

    /* Overlong line: rejected once (not executed truncated -- its first 31
     * characters would be a valid V command), and the next line still parses */
    longline[0] = 'V';
    memset(longline + 1, 'x', 59);
    strcpy(longline + 60, "\rV\r");
    CHECK(strcmp(exchange(longline), "\aV0001\r") == 0);

    /* A command split across packets */
    CHECK(strcmp(exchange("V"), "") == 0);
    CHECK(strcmp(exchange("\r"), "V0001\r") == 0);
}

static void test_flow_control(void) {
    reset_all();
    exchange("O\r");

    /* Queue full: the transmit waits and later input isn't read */
    dev.queue_space = 0;
    CHECK(strcmp(exchange("t1230\rV\r"), "") == 0);
    CHECK(dev.in_pos == strlen("O\rt1230\r"));
    now += SLCAN_TX_TIMEOUT_MS - 1;
    CHECK(strcmp(exchange(""), "") == 0);

    /* Space appears: the frame goes, then the held-back command runs */
    dev.queue_space = 1;
    CHECK(strcmp(exchange(""), "z\rV0001\r") == 0);
    CHECK(dev.sent_count == 1);

    /* The bus never drains: fail after the timeout and carry on */
    dev.queue_space = 0;
    CHECK(strcmp(exchange("T000000010\rV\r"), "") == 0);
    now += SLCAN_TX_TIMEOUT_MS;
    CHECK(strcmp(exchange(""), "\aV0001\r") == 0);
    CHECK(dev.sent_count == 1);

    /* No room for a reply: input isn't consumed until there is */
    dev.write_space = SLCAN_MAX_REPLY - 1;
    dev.in_len = dev.in_pos = 0;
    memcpy(dev.in, "V\r", 2);
    dev.in_len = 2;
    slcan_service(&s, now);
    CHECK(dev.in_pos == 0 && dev.out_len == 0);
    dev.write_space = SLCAN_MAX_REPLY;
    slcan_service(&s, now);
    CHECK(dev.out_len == 6 && memcmp(dev.out, "V0001\r", 6) == 0);
}

static void test_status_and_dfu(void) {
    reset_all();
    CHECK(strcmp(exchange("F\r"), "\a") == 0); /* closed */
    exchange("O\r");
    dev.status = 0xA5;
    CHECK(strcmp(exchange("F\r"), "FA5\r") == 0);
    dev.status = 0x08;
    CHECK(strcmp(exchange("F\r"), "F08\r") == 0);
    CHECK(strcmp(exchange("F1\r"), "\a") == 0);

    CHECK(strcmp(exchange("_dfu\r"), "\a") == 0);
    CHECK(strcmp(exchange("__xyz\r"), "\a") == 0);
    CHECK(!dev.bootloader);
    dev.bootloader_unavailable = true;
    CHECK(strcmp(exchange("__dfu\r"), "\a") == 0);
    CHECK(!dev.bootloader);
    dev.bootloader_unavailable = false;
    CHECK(strcmp(exchange("__dfu\r"), "\r") == 0);
    CHECK(dev.bootloader);
}

static void test_boot_options(void) {
    reset_all();
    CHECK(strcmp(exchange("__options\r"), "OBAA55FF0012ED34CBFF00\r") == 0);
    CHECK(dev.option_reads == 1 && dev.option_writes == 0);
    CHECK(strcmp(exchange("__optionsX\r__bootflashX\r__bootromX\r__dfuX\r"),
                 "\a\a\a\a") == 0);
    CHECK(dev.option_writes == 0 && dev.option_reads == 1);

    dev.bootloader_unavailable = true;
    CHECK(strcmp(exchange("__bootflash\r"), "\a") == 0);
    CHECK(!s.reboot_pending && !dev.flash_boot && dev.option_writes == 0);
    dev.bootloader_unavailable = false;
    CHECK(strcmp(exchange("__bootflash\r__bootrom\r"), "\r") == 0);
    CHECK(dev.flash_boot && !dev.bootloader && dev.option_writes == 1);
    CHECK(s.reboot_pending); /* Do not rewrite options twice in one USB packet. */

    reset_all();
    CHECK(strcmp(exchange("__bootrom\r"), "\r") == 0);
    CHECK(dev.bootloader && !dev.flash_boot && dev.option_writes == 1);

    reset_all();
    dev.write_space = SLCAN_MAX_REPLY - 1u;
    CHECK(strcmp(exchange("__options\r"), "") == 0);
    CHECK(dev.option_reads == 0); /* Preserve a whole diagnostic reply. */
    dev.write_space = SLCAN_MAX_REPLY;
    CHECK(strcmp(exchange(""), "OBAA55FF0012ED34CBFF00\r") == 0);
    CHECK(dev.option_reads == 1 && dev.option_writes == 0);
}

static void test_reset(void) {
    reset_all();
    exchange("O\r");

    /* A transmit waiting for queue space is dropped */
    dev.queue_space = 0;
    exchange("t1230\r");
    CHECK(s.tx_pending);
    slcan_reset(&s);
    CHECK(!dev.open && dev.closes == 1);
    CHECK(s.state == SLCAN_CLOSED && !s.tx_pending);
    dev.queue_space = 10;
    CHECK(strcmp(exchange(""), "") == 0);
    CHECK(dev.sent_count == 0);

    /* So is a partial command */
    CHECK(strcmp(exchange("t12"), "") == 0);
    slcan_reset(&s); /* closed already: no second close */
    CHECK(dev.closes == 1);
    CHECK(strcmp(exchange("V\r"), "V0001\r") == 0);
}

static void test_format(void) {
    uint8_t out[SLCAN_MAX_FRAME_LINE + 1];
    can_frame_t f;
    uint8_t n;

    f = (can_frame_t){0x123, 3, 0, {0xDE, 0xAD, 0x0F}};
    n = slcan_format_frame(&f, out);
    out[n] = 0;
    CHECK(strcmp((char *)out, "t1233DEAD0F\r") == 0);

    f = (can_frame_t){0x1FFFFFFF, 8, CAN_FRAME_EXT, {1, 2, 3, 4, 5, 6, 7, 8}};
    n = slcan_format_frame(&f, out);
    out[n] = 0;
    CHECK(n == SLCAN_MAX_FRAME_LINE);
    CHECK(strcmp((char *)out, "T1FFFFFFF80102030405060708\r") == 0);

    f = (can_frame_t){0x7FF, 2, CAN_FRAME_RTR, {0}};
    n = slcan_format_frame(&f, out);
    out[n] = 0;
    CHECK(strcmp((char *)out, "r7FF2\r") == 0); /* remote frames include the length */

    f = (can_frame_t){0x00000001, 0, CAN_FRAME_EXT | CAN_FRAME_RTR, {0}};
    n = slcan_format_frame(&f, out);
    out[n] = 0;
    CHECK(strcmp((char *)out, "R000000010\r") == 0);
}

/* Anything the adapter emits for a received frame must read back as the same
 * frame when sent as a command */
static void test_round_trip(void) {
    unsigned seed = 7;

    reset_all();
    exchange("O\r");
    for (int i = 0; i < 2000; i++) {
        can_frame_t f = {0};
        uint8_t line[SLCAN_MAX_FRAME_LINE + 1];
        uint8_t n;

        seed = seed * 1103515245u + 12345u;
        f.flags = (uint8_t)((seed >> 8) & 3u);
        f.id = (seed >> 3) & ((f.flags & CAN_FRAME_EXT) ? CAN_EXT_ID_MAX : CAN_STD_ID_MAX);
        f.dlc = (uint8_t)((seed >> 16) % 9u);
        if (!(f.flags & CAN_FRAME_RTR)) {
            for (uint8_t b = 0; b < f.dlc; b++) {
                f.data[b] = (uint8_t)(seed >> (b * 3));
            }
        }

        n = slcan_format_frame(&f, line);
        line[n] = 0;
        dev.sent_count = 0;
        dev.queue_space = 1;
        dev.in_len = dev.in_pos = 0;
        CHECK(strcmp(exchange((char *)line), (f.flags & CAN_FRAME_EXT) ? "Z\r" : "z\r") == 0);
        CHECK(dev.sent_count == 1 &&
              frame_is(&dev.sent[0], f.id, f.flags, f.dlc, (const char *)f.data));
    }
}

int main(void) {
    struct {
        const char *name;
        void (*fn)(void);
    } tests[] = {
        {"info", test_info},
        {"channel", test_channel},
        {"transmit", test_transmit},
        {"line_handling", test_line_handling},
        {"flow_control", test_flow_control},
        {"status_and_dfu", test_status_and_dfu},
        {"boot_options", test_boot_options},
        {"reset", test_reset},
        {"format", test_format},
        {"round_trip", test_round_trip},
    };

    for (size_t i = 0; i < sizeof(tests) / sizeof(tests[0]); i++) {
        int before = failures;
        tests[i].fn();
        printf("%-24s %s\n", tests[i].name, failures == before ? "ok" : "FAILED");
    }

    printf("%d checks, %d failed\n", checks, failures);
    return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
