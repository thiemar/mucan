#include <stdint.h>

void __attribute__((weak)) default_handler(void) {
    while (1u);
}

void __attribute__ ((weak, alias ("default_handler")))
stm32_reserved(void);
void __start(void) __attribute__ ((no_instrument_function));

/* Function prototypes for each defined vector */
#define UNUSED(x)
#define VECTOR(x, y) void __attribute__ ((weak, alias ("default_handler"))) \
                     x(void);

/* Standard vectors */
VECTOR(stm32_nmi, 2)
VECTOR(stm32_hardfault, 3)
UNUSED(4)
UNUSED(5)
UNUSED(6)
UNUSED(7)
UNUSED(8)
UNUSED(9)
UNUSED(10)
VECTOR(stm32_svcall, 11)
UNUSED(12)
UNUSED(13)
VECTOR(stm32_pendsv, 14)
VECTOR(SysTick_Handler, 15)

VECTOR(stm32_wwdg, 16)
VECTOR(stm32_pvd_vvdio, 17)
VECTOR(stm32_rtc, 18)
VECTOR(stm32_flash, 19)
VECTOR(stm32_rcc_crs, 20)
VECTOR(stm32_exti0_1, 21)
VECTOR(stm32_exti2_3, 22)
VECTOR(stm32_exti4_15, 23)
VECTOR(stm32_tsc, 24)
VECTOR(stm32_dma1_channel1, 25)
VECTOR(stm32_dma1_channel2_3, 26)
VECTOR(stm32_dma1_channel4_5, 27)
VECTOR(stm32_adc, 28)
VECTOR(stm32_tim1_brk_up_trg_com, 29)
VECTOR(stm32_tim1_cc, 30)
VECTOR(stm32_tim2, 31)
VECTOR(stm32_tim3, 32)
UNUSED(33)
UNUSED(34)
VECTOR(stm32_tim14, 35)
UNUSED(36)
VECTOR(stm32_tim16, 37)
VECTOR(stm32_tim17, 38)
VECTOR(stm32_i2c1, 39)
UNUSED(40)
VECTOR(stm32_spi1, 41)
VECTOR(stm32_spi2, 42)
VECTOR(stm32_usart1, 43)
VECTOR(stm32_usart2, 44)
UNUSED(45)
VECTOR(stm32_cec_can, 46)
VECTOR(USB_IRQHandler, 47)

#undef VECTOR
#undef UNUSED


extern uint32_t __stack;

typedef void (* const vector_entry)(void);

__attribute__ ((section(".app_vectors"),used))
vector_entry app_vectors[] = {
    (vector_entry)&__stack,
    __start,

#define UNUSED(x) (vector_entry)(stm32_reserved),
#define VECTOR(x, y) (vector_entry)x,

/* Standard vectors */
VECTOR(stm32_nmi, 2)
VECTOR(stm32_hardfault, 3)
UNUSED(4)
UNUSED(5)
UNUSED(6)
UNUSED(7)
UNUSED(8)
UNUSED(9)
UNUSED(10)
VECTOR(stm32_svcall, 11)
UNUSED(12)
UNUSED(13)
VECTOR(stm32_pendsv, 14)
VECTOR(SysTick_Handler, 15)

VECTOR(stm32_wwdg, 16)
VECTOR(stm32_pvd_vvdio, 17)
VECTOR(stm32_rtc, 18)
VECTOR(stm32_flash, 19)
VECTOR(stm32_rcc_crs, 20)
VECTOR(stm32_exti0_1, 21)
VECTOR(stm32_exti2_3, 22)
VECTOR(stm32_exti4_15, 23)
VECTOR(stm32_tsc, 24)
VECTOR(stm32_dma1_channel1, 25)
VECTOR(stm32_dma1_channel2_3, 26)
VECTOR(stm32_dma1_channel4_5, 27)
VECTOR(stm32_adc, 28)
VECTOR(stm32_tim1_brk_up_trg_com, 29)
VECTOR(stm32_tim1_cc, 30)
VECTOR(stm32_tim2, 31)
VECTOR(stm32_tim3, 32)
UNUSED(33)
UNUSED(34)
VECTOR(stm32_tim14, 35)
UNUSED(36)
VECTOR(stm32_tim16, 37)
VECTOR(stm32_tim17, 38)
VECTOR(stm32_i2c1, 39)
UNUSED(40)
VECTOR(stm32_spi1, 41)
VECTOR(stm32_spi2, 42)
VECTOR(stm32_usart1, 43)
VECTOR(stm32_usart2, 44)
UNUSED(45)
VECTOR(stm32_cec_can, 46)
VECTOR(USB_IRQHandler, 47)

#undef VECTOR
#undef UNUSED
};
