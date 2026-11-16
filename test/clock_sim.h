/* Host-register substitution for the production CMSIS clock calculation. */
#ifndef CLOCK_SIM_H
#define CLOCK_SIM_H

#include <stdint.h>

#define __CORE_CM0_H_GENERIC
#define __CORE_CM0_H_DEPENDANT
#define __IO volatile
#define __I volatile const
#define __O volatile
#define __STATIC_INLINE static inline
#include "../sys/stm32f042x6.h"
#include "../app/stm32f0xx_hal_conf.h"

extern RCC_TypeDef clock_sim_rcc;
#undef RCC
#define RCC (&clock_sim_rcc)

#endif
