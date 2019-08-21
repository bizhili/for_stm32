#include "../Board/encoder/encoder.h"
#include "../Chip/tim/tim2.h"
#include "../Chip/usart/usart1.h"
#include "../Chip/delay/delay.h"

void encoder_init(void)
{
  tim2_encoderInit(0xffff,0);
}

void encoder_setCounter(uint16_t counter)
{
  TIM_SetCounter(TIM2,counter);
}

int16_t encoder_getCounter(void)
{
  TIM_SetCounter(TIM2,0);
  delay_ms(10);
  return (int16_t)TIM_GetCounter(TIM2);   //这里利用了原码补码的特性可以输出正负
  //printf("counter:%d\n",(int16_t)TIM_GetCounter(TIM2));
}



