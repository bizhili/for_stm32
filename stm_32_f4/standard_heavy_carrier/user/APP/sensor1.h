#ifndef SENSOR1_H
#define SENSOR1_H
#include "main.h"
#include "stm32f4xx.h"
#define PC5     GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_5)
#define PC0     GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_0)//for ctrl


#define PC2     GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_2)//for sensor
#define PC4     GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_4)//chassis right
#define PA4     GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)//chassis left
extern void sensor_Init(void);


#endif
