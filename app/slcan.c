/*
 * SLCAN (Lawicel ASCII) protocol. See slcan.h.
 *
 * Commands (each terminated by CR; a LF is ignored so CRLF hosts work):
 *   Sn          bitrate n = 0..8 (10k 20k 50k 100k 125k 250k 500k 800k 1M), closed only
 *   O / L / C   open / open listen-only / close
 *   tiiil<dd>   standard frame        -> z
 *   Tiiiiiiiil<dd>  extended frame    -> Z
 *   riiil       standard RTR          -> z   ("riii" without a length is accepted)
 *   Riiiiiiiil  extended RTR          -> Z
 *   F           status flags, open only -> Fxx
 *   V / v / N   hardware version, software version, serial number
 *   __dfu       restart into the DFU bootloader
 * Success is CR (or z/Z for transmits), failure is BELL.
 */

#include "slcan.h"

#define CR 0x0Du
#define LF 0x0Au
#define BELL 0x07u

#define HW_VERSION "0001"
#define SW_VERSION "0002"

static const uint32_t bitrates[9] = {10000u,  20000u,  50000u,  100000u, 125000u,
                                     250000u, 500000u, 800000u, 1000000u};

static const char hex_digits[] = "0123456789ABCDEF";

static void reply(const uint8_t *data, uint8_t len) {
    /* slcan_service() only consumes input when SLCAN_MAX_REPLY bytes are free */
    (void)slcan_port_write(data, len);
}

static void reply_byte(uint8_t byte) {
    reply(&byte, 1);
}

static void reply_text(char command, const char *text) {
    uint8_t out[SLCAN_MAX_REPLY];
    uint8_t n = 0;

    out[n++] = (uint8_t)command;
    while (*text && n < SLCAN_MAX_REPLY - 1u) {
        out[n++] = (uint8_t)*text++;
    }
    out[n++] = CR;
    reply(out, n);
}

static int hex_value(uint8_t c) {
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    if (c >= 'A' && c <= 'F') {
        return c - 'A' + 10;
    }
    if (c >= 'a' && c <= 'f') {
        return c - 'a' + 10;
    }
    return -1;
}

static bool parse_hex(const uint8_t *text, uint8_t digits, uint32_t *value) {
    uint32_t v = 0;
    uint8_t i;

    for (i = 0; i < digits; i++) {
        int nibble = hex_value(text[i]);
        if (nibble < 0) {
            return false;
        }
        v = (v << 4) | (uint32_t)nibble;
    }
    *value = v;
    return true;
}

/* t/T/r/R arguments (everything after the command letter) */
static bool parse_frame(uint8_t command, const uint8_t *arg, uint8_t n, can_frame_t *frame) {
    bool extended = command == 'T' || command == 'R';
    bool remote = command == 'r' || command == 'R';
    uint8_t id_digits = extended ? 8u : 3u;
    uint32_t value;
    uint8_t i;

    if (n < id_digits || !parse_hex(arg, id_digits, &value) ||
        value > (extended ? CAN_EXT_ID_MAX : CAN_STD_ID_MAX)) {
        return false;
    }

    frame->id = value;
    frame->flags = (uint8_t)((extended ? CAN_FRAME_EXT : 0u) | (remote ? CAN_FRAME_RTR : 0u));
    frame->dlc = 0;
    for (i = 0; i < sizeof(frame->data); i++) {
        frame->data[i] = 0;
    }

    if (remote && n == id_digits) {
        return true; /* legacy RTR without a length digit */
    }
    if (n < id_digits + 1u || arg[id_digits] < '0' || arg[id_digits] > '8') {
        return false;
    }
    frame->dlc = (uint8_t)(arg[id_digits] - '0');

    if (remote) {
        return n == id_digits + 1u;
    }
    if (n != id_digits + 1u + 2u * frame->dlc) {
        return false;
    }
    for (i = 0; i < frame->dlc; i++) {
        if (!parse_hex(arg + id_digits + 1u + 2u * i, 2, &value)) {
            return false;
        }
        frame->data[i] = (uint8_t)value;
    }
    return true;
}

/* Returns true once the pending transmit has been queued or given up on */
static bool transmit_pending(slcan_t *s, uint32_t now_ms) {
    if (slcan_port_transmit(&s->tx_frame)) {
        uint8_t ack[2] = {(uint8_t)(s->tx_extended ? 'Z' : 'z'), CR};
        s->tx_pending = false;
        reply(ack, sizeof(ack));
        return true;
    }
    if ((uint32_t)(now_ms - s->tx_since) >= SLCAN_TX_TIMEOUT_MS) {
        /* The bus isn't draining (no ACK, bus off, ...): drop the frame
         * rather than holding up every other command */
        s->tx_pending = false;
        reply_byte(BELL);
        return true;
    }
    return false;
}

static bool is_dfu_request(const uint8_t *arg, uint8_t n) {
    return n == 4u && arg[0] == '_' && arg[1] == 'd' && arg[2] == 'f' && arg[3] == 'u';
}

static void execute(slcan_t *s, uint32_t now_ms) {
    uint8_t command;
    const uint8_t *arg = s->line + 1;
    uint8_t n;
    uint8_t out[SLCAN_MAX_REPLY];

    if (s->len == 0) {
        reply_byte(BELL);
        return;
    }
    command = s->line[0];
    n = (uint8_t)(s->len - 1u);

    switch (command) {
        case 'S':
            if (s->state != SLCAN_CLOSED || n != 1u || arg[0] < '0' || arg[0] > '8' ||
                !slcan_port_set_bitrate(bitrates[arg[0] - '0'])) {
                reply_byte(BELL);
            } else {
                reply_byte(CR);
            }
            return;

        case 'O':
        case 'L':
            if (s->state != SLCAN_CLOSED || n != 0u || !slcan_port_open(command == 'L')) {
                reply_byte(BELL);
                return;
            }
            s->state = command == 'L' ? SLCAN_LISTEN_ONLY : SLCAN_OPEN;
            reply_byte(CR);
            return;

        case 'C':
            if (s->state == SLCAN_CLOSED || n != 0u) {
                reply_byte(BELL);
                return;
            }
            slcan_port_close();
            s->state = SLCAN_CLOSED;
            reply_byte(CR);
            return;

        case 't':
        case 'T':
        case 'r':
        case 'R':
            if (s->state != SLCAN_OPEN || !parse_frame(command, arg, n, &s->tx_frame)) {
                reply_byte(BELL);
                return;
            }
            s->tx_pending = true;
            s->tx_extended = command == 'T' || command == 'R';
            s->tx_since = now_ms;
            (void)transmit_pending(s, now_ms);
            return;

        case 'F': {
            uint8_t flags;
            if (s->state == SLCAN_CLOSED || n != 0u) {
                reply_byte(BELL);
                return;
            }
            flags = slcan_port_status();
            out[0] = 'F';
            out[1] = (uint8_t)hex_digits[flags >> 4];
            out[2] = (uint8_t)hex_digits[flags & 0xFu];
            out[3] = CR;
            reply(out, 4);
            return;
        }

        case 'V':
            reply_text('V', HW_VERSION);
            return;

        case 'v':
            reply_text('v', SW_VERSION);
            return;

        case 'N': {
            uint16_t serial = slcan_port_serial();
            out[0] = 'N';
            out[1] = (uint8_t)hex_digits[(serial >> 12) & 0xFu];
            out[2] = (uint8_t)hex_digits[(serial >> 8) & 0xFu];
            out[3] = (uint8_t)hex_digits[(serial >> 4) & 0xFu];
            out[4] = (uint8_t)hex_digits[serial & 0xFu];
            out[5] = CR;
            reply(out, 6);
            return;
        }

        case '_':
            if (!is_dfu_request(arg, n)) {
                reply_byte(BELL);
                return;
            }
            reply_byte(CR);
            slcan_port_bootloader();
            return;

        default:
            /* Including Z (timestamps), M/m (acceptance filter), s (BTR) */
            reply_byte(BELL);
            return;
    }
}

static void input(slcan_t *s, uint8_t c, uint32_t now_ms) {
    if (c == LF) {
        return;
    }
    if (c == CR) {
        if (s->overflow) {
            reply_byte(BELL);
        } else {
            execute(s, now_ms);
        }
        s->len = 0;
        s->overflow = false;
        return;
    }
    if (s->overflow) {
        return;
    }
    if (s->len == sizeof(s->line)) {
        /* Too long to be valid: ignore the rest of the line, then fail it */
        s->overflow = true;
        return;
    }
    s->line[s->len++] = c;
}

void slcan_init(slcan_t *s) {
    s->state = SLCAN_CLOSED;
    s->len = 0;
    s->overflow = false;
    s->tx_pending = false;
    s->tx_extended = false;
    s->tx_since = 0;
}

void slcan_reset(slcan_t *s) {
    if (s->state != SLCAN_CLOSED) {
        slcan_port_close();
    }
    slcan_init(s);
}

void slcan_service(slcan_t *s, uint32_t now_ms) {
    for (;;) {
        int c;

        if (s->tx_pending && !transmit_pending(s, now_ms)) {
            return;
        }
        if (slcan_port_write_space() < SLCAN_MAX_REPLY) {
            return;
        }
        c = slcan_port_getc();
        if (c < 0) {
            return;
        }
        input(s, (uint8_t)c, now_ms);
    }
}

uint8_t slcan_format_frame(const can_frame_t *frame, uint8_t *out) {
    bool extended = (frame->flags & CAN_FRAME_EXT) != 0;
    bool remote = (frame->flags & CAN_FRAME_RTR) != 0;
    uint8_t digits = extended ? 8u : 3u;
    uint8_t dlc = frame->dlc > 8u ? 8u : frame->dlc;
    uint8_t n = 0;
    uint8_t i;

    out[n++] = (uint8_t)(remote ? (extended ? 'R' : 'r') : (extended ? 'T' : 't'));
    while (digits) {
        digits--;
        out[n++] = (uint8_t)hex_digits[(frame->id >> (4u * digits)) & 0xFu];
    }
    out[n++] = (uint8_t)('0' + dlc);
    if (!remote) {
        for (i = 0; i < dlc; i++) {
            out[n++] = (uint8_t)hex_digits[frame->data[i] >> 4];
            out[n++] = (uint8_t)hex_digits[frame->data[i] & 0xFu];
        }
    }
    out[n++] = CR;
    return n;
}
