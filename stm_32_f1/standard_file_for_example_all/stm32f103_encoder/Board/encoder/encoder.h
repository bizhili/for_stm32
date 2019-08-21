#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f10x.h"

void encoder_init(void);
void encoder_setCounter(uint16_t counter);
int16_t encoder_getCounter(void);



#endif /* __ENCODER_H */

