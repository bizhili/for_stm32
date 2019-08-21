#include "../Board/servo/servo.h"
#include "../Chip/tim/tim1.h"
#include "../Chip/delay/delay.h"



void servo_init(void)
{
  tim1_pwmInit(SERVO_TIM_ARR,SERVO_TIM_PSC);
  TIM_SetCompare1(TIM1,150);  //使舵机恢复到中间位置
}

//0.5ms--0°  2.5ms--180°
void servo_angle(uint16_t angle)
{
  uint16_t pulse;
  
  //针对舵机可转角度限辐
  if(angle <= 5)
    angle = 5;
  if(angle >= 175)
    angle = 175;
  //将角度值转换为脉冲值  
  pulse = (uint16_t)(50 + angle * 100/90.0);  //此转换公式需根据pwm的arr及psc配置来做相应变化
  TIM_SetCompare1(TIM1, pulse);
  
}


void servo_debug(void)
{
  uint8_t i;
  for(i = 0; i < 10; ++i)
  {
    delay_ms(500);
    servo_angle(45);
    delay_ms(500);
    servo_angle(90);
    delay_ms(500);
    servo_angle(135);
    delay_ms(500);
    servo_angle(90);
  }
}



