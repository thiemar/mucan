/*
 * SLCAN (Lawicel ASCII) protocol: command parsing and execution, and frame
 * formatting. Pure C with no hardware access -- the device side is reached
 * through the slcan_port_* hooks below, which the firmware implements in
 * main.c and the host tests implement as fakes.
 */

#ifndef SLCAN_H
#define SLCAN_H

#include <stdbool.h>
#include <stdint.h>

#include "can_frame.h"

#define SLCAN_MAX_FRAME_LINE 27u  /* "T" + 8 id + 1 dlc + 16 data + CR */
#define SLCAN_MAX_REPLY 8u        /* longest command reply ("V0001\r") */
#define SLCAN_MAX_COMMAND 31u     /* longest accepted command, excluding CR */
#define SLCAN_TX_TIMEOUT_MS 100u  /* how long a transmit waits for queue space */

typedef enum {
    SLCAN_CLOSED,
    SLCAN_OPEN,
    SLCAN_LISTEN_ONLY
} slcan_state_t;

typedef struct {
    slcan_state_t state;
    uint8_t line[SLCAN_MAX_COMMAND];
    uint8_t len;
    bool overflow;     /* discarding an overlong line up to its CR */
    bool tx_pending;   /* a transmit is waiting for queue space */
    bool tx_extended;  /* reply Z rather than z */
    uint32_t tx_since;
    can_frame_t tx_frame;
} slcan_t;

/* ---- Device hooks ---------------------------------------------------- */

bool slcan_port_set_bitrate(uint32_t bitrate);    /* only called while closed */
bool slcan_port_open(bool listen_only);
void slcan_port_close(void);
bool slcan_port_transmit(const can_frame_t *frame); /* false: queue full */
uint8_t slcan_port_status(void);                  /* 'F' flags; clears latched bits */
uint16_t slcan_port_serial(void);
void slcan_port_bootloader(void);                 /* reset into DFU after replying */
int slcan_port_getc(void);                        /* -1 if no input */
uint16_t slcan_port_write_space(void);
bool slcan_port_write(const uint8_t *data, uint16_t len);

/* ---- Protocol -------------------------------------------------------- */

void slcan_init(slcan_t *s);

/* Close the channel if open and forget any partial command */
void slcan_reset(slcan_t *s);

/*
 * Consume and execute host commands. Returns when input runs out, when there
 * is no room for a reply, or while a transmit is waiting for queue space --
 * the unread input then holds the host off through USB flow control.
 */
void slcan_service(slcan_t *s, uint32_t now_ms);

/* Format a received frame; returns its length (at most SLCAN_MAX_FRAME_LINE) */
uint8_t slcan_format_frame(const can_frame_t *frame, uint8_t *out);

#endif /* SLCAN_H */
