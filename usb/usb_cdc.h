/*
 * Minimal polled USB CDC-ACM device for the STM32F0 USB full-speed peripheral.
 *
 * All USB work happens inside usb_poll(), which must be called regularly from
 * the main loop. The USB interrupt is never enabled, so the driver has no
 * shared state with interrupt handlers and needs no locking.
 */

#ifndef USB_CDC_H
#define USB_CDC_H

#include <stdbool.h>
#include <stdint.h>

/* Enable the peripheral and connect to the bus (D+ pull-up on). */
void usb_init(void);

/* Service the peripheral: enumeration, control requests and data endpoints. */
void usb_poll(void);

/* Drop off the bus (D+ pull-up off), e.g. before resetting into DFU. */
void usb_disconnect(void);

/* True once the host has selected the configuration. */
bool usb_cdc_configured(void);

/* DTR as last set by the host (SET_CONTROL_LINE_STATE). */
bool usb_cdc_dtr(void);

/*
 * Next byte received from the host, or -1 if none is pending. The OUT
 * endpoint is re-armed only after the previous packet has been consumed, so
 * the host is held off (NAK) while the application is busy.
 */
int usb_cdc_getc(void);

/* Bytes that can currently be queued with usb_cdc_write(). */
uint16_t usb_cdc_write_space(void);

/*
 * Queue bytes for the host. All-or-nothing: returns false and queues nothing
 * if the device is not configured or there is not enough space, so callers
 * never emit a partial line.
 */
bool usb_cdc_write(const uint8_t *data, uint16_t len);

/* True once everything written has been collected by the host. */
bool usb_cdc_tx_empty(void);

#endif /* USB_CDC_H */
