/*
 * bxCAN driver, register level.
 *
 * Received frames are moved from the 3-deep hardware FIFO into a RAM ring by
 * the CAN interrupt, so the main loop can be slow without losing frames.
 * Transmit frames go through a software queue that can_poll() feeds into the
 * three mailboxes in request order (TXFP), so frames leave in the order the
 * host sent them.
 */

#ifndef CAN_H
#define CAN_H

#include <stdbool.h>
#include <stdint.h>

#include "can_frame.h"

/* Status bits, laid out as the Lawicel 'F' reply */
#define CAN_STATUS_RX_FULL 0x01u          /* receive ring overflowed */
#define CAN_STATUS_TX_FULL 0x02u          /* transmit queue full */
#define CAN_STATUS_ERROR_WARNING 0x04u    /* error counter >= 96 */
#define CAN_STATUS_OVERRUN 0x08u          /* frames lost on receive */
#define CAN_STATUS_ERROR_PASSIVE 0x20u    /* error counter >= 128 */
#define CAN_STATUS_ARBITRATION_LOST 0x40u
#define CAN_STATUS_BUS_ERROR 0x80u        /* protocol error seen, or bus off */

/* Configure pins and controller; the channel starts closed at 1 Mbit/s */
void can_init(void);

/* Supported: 10k, 20k, 50k, 100k, 125k, 250k, 500k, 800k, 1M. Closed only. */
bool can_set_bitrate(uint32_t bitrate);

/* Join the bus (listen-only never transmits, ACKs or signals errors) */
bool can_open(bool listen_only);

/* Leave the bus, abort pending transmits and discard queued frames */
void can_close(void);

/* Queue a frame; false if closed or the queue is full */
bool can_transmit(const can_frame_t *frame);

/* Next received frame, if any */
bool can_receive(can_frame_t *frame);

/* Move queued frames into free mailboxes and latch error events */
void can_poll(void);

/* CAN_STATUS_* bits; reading clears the latched ones */
uint8_t can_status(void);

#endif /* CAN_H */
