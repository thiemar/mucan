/* A classic CAN 2.0 frame, shared by the CAN driver and the SLCAN protocol. */

#ifndef CAN_FRAME_H
#define CAN_FRAME_H

#include <stdint.h>

#define CAN_FRAME_EXT 0x01u /* 29-bit identifier */
#define CAN_FRAME_RTR 0x02u /* remote transmission request */

#define CAN_STD_ID_MAX 0x7FFu
#define CAN_EXT_ID_MAX 0x1FFFFFFFu

typedef struct {
    uint32_t id;
    uint8_t dlc; /* 0..8 */
    uint8_t flags;
    uint8_t data[8];
} can_frame_t;

#endif /* CAN_FRAME_H */
