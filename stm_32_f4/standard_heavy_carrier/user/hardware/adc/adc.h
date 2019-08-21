#ifndef ADC_H
#define ADC_H
#include "main.h"

extern void temperature_ADC_init(void);
extern fp32 get_temprate(void);
void Adc_Init(void); 				
uint16_t  Get_Adc1(uint8_t ch); 				
uint16_t Get_Adc_Average(uint8_t ch,uint8_t times);  

#endif
