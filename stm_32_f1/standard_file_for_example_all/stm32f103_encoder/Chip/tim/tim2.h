#ifndef __TIM2_H
#define __TIM2_H

#include "stm32f10x.h"

#define TIM2_CH1_GPIO_CLK    RCC_APB2Periph_GPIOA
#define TIM2_CH1_PORT        GPIOA
#define TIM2_CH1_PIN         GPIO_Pin_0

#define TIM2_CH2_GPIO_CLK    RCC_APB2Periph_GPIOA
#define TIM2_CH2_PORT        GPIOA
#define TIM2_CH2_PIN         GPIO_Pin_1

void tim2_encoderInit(uint16_t arr, uint16_t psc);

#endif  //__TIMER_H

