/* Host model for CAN mode transitions. Register layouts and bit definitions
 * come from the production device header; only CPU/HAL services are stubbed. */
#ifndef CAN_SIM_H
#define CAN_SIM_H

#include <stdbool.h>
#include <stdint.h>

#define __CORE_CM0_H_GENERIC
#define __CORE_CM0_H_DEPENDANT
#define __IO volatile
#define __I volatile const
#define __O volatile
#include "../sys/stm32f042x6.h"

extern CAN_TypeDef can_sim_regs;
extern RCC_TypeDef can_sim_rcc;
extern GPIO_TypeDef can_sim_gpio;
#undef CAN
#undef RCC
#undef GPIOB
#define CAN (&can_sim_regs)
#define RCC (&can_sim_rcc)
#define GPIOB (&can_sim_gpio)

typedef struct {
    uint32_t Pin, Mode, Pull, Speed, Alternate;
} GPIO_InitTypeDef;

#define GPIO_PIN_7 0x0080u
#define GPIO_PIN_8 0x0100u
#define GPIO_PIN_9 0x0200u
#define GPIO_MODE_INPUT 0u
#define GPIO_MODE_AF_PP 2u
#define GPIO_PULLDOWN 2u
#define GPIO_NOPULL 0u
#define GPIO_SPEED_LOW 0u
#define GPIO_SPEED_HIGH 3u
#define GPIO_AF4_CAN 4u
#define __GPIOB_CLK_ENABLE() ((void)0)

void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init);
uint32_t HAL_GetTick(void);
void NVIC_DisableIRQ(IRQn_Type irq);
void NVIC_EnableIRQ(IRQn_Type irq);
void NVIC_ClearPendingIRQ(IRQn_Type irq);
void NVIC_SetPriority(IRQn_Type irq, uint32_t priority);
void __disable_irq(void);
void __enable_irq(void);

void can_sim_write_bit_timing(uint32_t btr);
void can_sim_release_fifo0(void);
void can_sim_reset_peripheral(void);

void can_sim_init(void);
void can_sim_advance(uint32_t milliseconds);
void can_sim_begin_receive(uint32_t milliseconds);
void can_sim_stall_initialization(bool reset_recovers);
uint32_t can_sim_time(void);
uint32_t can_sim_reset_count(void);
uint32_t can_sim_rejected_btr_writes(void);
uint32_t can_sim_received_count(void);
bool can_sim_irq_enabled(void);

#endif
