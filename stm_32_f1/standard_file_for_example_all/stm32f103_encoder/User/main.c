#include "stm32f10x.h"

#include "usart1.h"
#include "delay.h"
#include "../Board/encoder/encoder.h"


int main(void)
{
  int16_t counter;
  float speed;
  usart1_init(115200);
  delay_init();
  
  encoder_init();
  while(1)
  {
    counter = encoder_getCounter();
    printf("counter:%d\n",counter);
    speed = counter / 390.0 *100;
    printf("speed:%f\n",speed);
    printf("\n");
    delay_ms(1000);
  }
}
