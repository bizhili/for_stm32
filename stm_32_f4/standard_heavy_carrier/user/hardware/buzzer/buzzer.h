#ifndef BUZZER_H
#define BUZZER_H
#include "main.h"
extern void buzzer_init(uint16_t arr, uint16_t psc);
extern void buzzer_on(uint16_t psc, uint16_t pwm);
extern void buzzer_off(void);
 extern void IWDG_Init(uint8_t prer,uint16_t rlr);

#endif
