/*
 * Host-side model of the STM32F0 USB full-speed peripheral, used to unit-test
 * usb/usb_cdc.c without hardware. The model is written from RM0091 and is
 * independent of the driver's own register definitions.
 *
 * It covers what the driver depends on: EPnR write semantics (rw, toggle and
 * rc_w0 bits), ISTR, the packet memory and buffer descriptor table, address
 * filtering, NAK/STALL handshakes and DATA0/DATA1 toggling. It does not model
 * timing, suspend/resume or electrical behaviour.
 */

#ifndef USB_SIM_H
#define USB_SIM_H

#include <stdbool.h>
#include <stdint.h>

/* Hardware interface consumed by usb_cdc.c when built with -DUSB_SIM */
uint16_t hw_epr_get(uint8_t ep);
void hw_epr_set(uint8_t ep, uint16_t value);
uint16_t hw_istr_get(void);
void hw_istr_clear(uint16_t bits);
volatile uint16_t *hw_pma(uint16_t offset);
void hw_set_address(uint8_t address);
void hw_start(void);
void hw_connect(void);
void hw_disconnect(void);
uint32_t hw_irq_save(void);
void hw_irq_restore(uint32_t mask);

extern const uint8_t sim_uid[12];
#define HW_UID sim_uid

/* Host side: one call is one transaction, returning the device's handshake */
typedef enum { SIM_ACK, SIM_NAK, SIM_STALL, SIM_TIMEOUT } sim_handshake_t;

void sim_bus_reset(void);
sim_handshake_t sim_setup(uint8_t address, const uint8_t packet[8]);
sim_handshake_t sim_out(uint8_t address, uint8_t ep, const uint8_t *data, uint16_t len);
sim_handshake_t sim_in(uint8_t address, uint8_t ep, uint8_t *data, uint16_t *len);

/* Host resets its expected data toggle (after SET_CONFIGURATION or
 * CLEAR_FEATURE(ENDPOINT_HALT)) */
void sim_reset_host_toggle(uint8_t ep_address);

/* Run `fn` the next time the driver writes EPnR `ep`, between its read and
 * its write -- i.e. a bus transaction completing mid read-modify-write */
void sim_before_next_write(uint8_t ep, void (*fn)(void));

bool sim_connected(void);
uint8_t sim_daddr(void);
bool sim_irq_masked(void);

/* Protocol violations detected by the model since the last call */
int sim_take_errors(void);

/* Count violations without printing them (for deliberate negative tests) */
void sim_set_quiet(bool quiet);

#endif /* USB_SIM_H */
