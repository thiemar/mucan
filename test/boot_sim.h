/* Minimal STM32F042 option-flash/reset model for boot.c host tests. */
#ifndef BOOT_SIM_H
#define BOOT_SIM_H

#include <setjmp.h>
#include <stdbool.h>
#include <stdint.h>

/* Match the device layouts/bit masks without compiling ARM CPU intrinsics. */
#define __CORE_CM0_H_GENERIC
#define __CORE_CM0_H_DEPENDANT
#define __IO volatile
#define __I volatile const
#define __O volatile
#include "../sys/stm32f042x6.h"

/* The vendor's generic OB_TypeDef also names WRP1/2/3. RM0091 section 4.1.3
 * says they do not exist on F042, so this model deliberately omits them. */
typedef struct {
    volatile uint16_t RDP, USER, DATA0, DATA1, WRP0;
} boot_sim_options_t;
typedef enum { HAL_OK, HAL_ERROR } HAL_StatusTypeDef;
#define __DBGMCU_CLK_ENABLE() ((void)0)

HAL_StatusTypeDef HAL_FLASH_Unlock(void);
HAL_StatusTypeDef HAL_FLASH_Lock(void);
HAL_StatusTypeDef HAL_FLASH_OB_Unlock(void);
HAL_StatusTypeDef HAL_FLASH_OB_Lock(void);
HAL_StatusTypeDef HAL_FLASH_OB_Launch(void);
uint32_t HAL_GetTick(void);

extern RCC_TypeDef boot_sim_rcc;
extern SYSCFG_TypeDef boot_sim_syscfg;
extern FLASH_TypeDef boot_sim_flash;
extern boot_sim_options_t boot_sim_ob;
extern DBGMCU_TypeDef boot_sim_dbgmcu;
extern IWDG_TypeDef boot_sim_iwdg;
/* CMSIS core types are excluded above with the ARM intrinsics. */
typedef struct { uint32_t CTRL, LOAD, VAL; } boot_sim_systick_t;
typedef struct { uint32_t ICER[1], ICPR[1]; } boot_sim_nvic_t;
typedef struct { uint32_t ICSR; } boot_sim_scb_t;
extern boot_sim_systick_t boot_sim_systick;
extern boot_sim_nvic_t boot_sim_nvic;
extern boot_sim_scb_t boot_sim_scb;
extern bool boot_sim_irq_disabled;
#define SysTick (&boot_sim_systick)
#define NVIC (&boot_sim_nvic)
#define SCB (&boot_sim_scb)
#define SCB_ICSR_PENDSTCLR_Msk (1u << 25)
#define SCB_ICSR_PENDSVCLR_Msk (1u << 27)
#define __disable_irq() (boot_sim_irq_disabled = true)

#undef RCC
#undef SYSCFG
#undef FLASH
#undef OB
#undef DBGMCU
#undef IWDG
#define RCC (&boot_sim_rcc)
#define SYSCFG (&boot_sim_syscfg)
#define FLASH (&boot_sim_flash)
#define OB (&boot_sim_ob)
#define DBGMCU (&boot_sim_dbgmcu)
#define IWDG (&boot_sim_iwdg)

#undef __DSB
#undef __ISB
#define __DSB() ((void)0)
#define __ISB() ((void)0)
#define NVIC_SystemReset boot_sim_reset

enum { BOOT_SIM_RESET = 1, BOOT_SIM_RELOAD, BOOT_SIM_POWER_CUT };
extern jmp_buf boot_sim_escape;
extern unsigned boot_sim_erases, boot_sim_programs, boot_sim_launches;
extern unsigned boot_sim_program_order[16], boot_sim_operations;
extern unsigned boot_sim_cut_operation;
extern uint16_t boot_sim_cut_mask, boot_sim_erase_masks[5];
extern bool boot_sim_main_flash_erased;
extern unsigned boot_sim_fail_program, boot_sim_corrupt_program;
extern bool boot_sim_fail_unlock, boot_sim_fail_option_unlock;
extern bool boot_sim_fail_erase, boot_sim_fail_launch, boot_sim_fail_persistent;

void boot_sim_init(uint8_t user, uint8_t rdp);
void boot_sim_load_options(void);
void boot_sim_set_option(unsigned index, uint8_t value);
void boot_sim_set_raw_option(unsigned index, uint16_t pair);
uint16_t boot_sim_get_option(unsigned index);
void boot_sim_program_option(unsigned index, uint8_t value);
void boot_sim_reset(void) __attribute__((noreturn));

#endif
