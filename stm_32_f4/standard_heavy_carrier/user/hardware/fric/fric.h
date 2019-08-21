#ifndef FRIC_H
#define FRIC_H
#include "main.h"

#define Fric_UP 1400
#define Fric_DOWN 1300
#define Fric_OFF 1000
#define PE4    GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_4)
#define PE5    GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_5)
extern void gpio2_Init(void);//pf1
extern void TIM8_PWM1_Init(uint32_t arr,uint32_t psc,uint32_t date);//第四个pwm输出PI5 tim8 ch1
extern void fric_PWM_configuration(uint32_t arr,uint32_t psc,uint32_t date);
extern void fric_off(void);
extern void fric1_on(uint16_t cmd);
extern void fric2_on(uint16_t cmd);
extern void TIM4_PWM_Init(uint32_t arr,uint32_t psc,uint32_t date);
extern void TIM5_PWM2_Init(uint32_t arr,uint32_t psc,uint32_t date);
extern void TIM2_PWM3_Init(uint32_t arr,uint32_t psc,uint32_t date);
extern void vcc_Init(void);//控制高低电平PE5,6,7
extern void Out_EXTI_Configuration1(void);
extern void Out_EXTI_Configuration2(void);
#endif
