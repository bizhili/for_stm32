#include "sensor1.h"

void sensor_Init(void)//pc2,4,0,5;pb1
{
GPIO_InitTypeDef GPIO_Struct1;	
RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOA,ENABLE);
GPIO_Struct1.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_2|GPIO_Pin_4|GPIO_Pin_5;
GPIO_Struct1.GPIO_Mode=GPIO_Mode_IN;
GPIO_Struct1.GPIO_Speed=GPIO_Speed_100MHz;
GPIO_Struct1.GPIO_PuPd=GPIO_PuPd_UP;	
GPIO_Init(GPIOC,&GPIO_Struct1);
GPIO_Struct1.GPIO_Mode = GPIO_Mode_OUT;        //复用功能
GPIO_Struct1.GPIO_Speed = GPIO_Speed_100MHz;    //速度100MHz
GPIO_Struct1.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
GPIO_Struct1.GPIO_PuPd = GPIO_PuPd_DOWN;        //上拉
GPIO_Struct1.GPIO_Pin=GPIO_Pin_4;
GPIO_Init(GPIOA,&GPIO_Struct1);
GPIO_SetBits(GPIOC,GPIO_Pin_0|GPIO_Pin_3|GPIO_Pin_4);
GPIO_ResetBits(GPIOA,GPIO_Pin_4);
}