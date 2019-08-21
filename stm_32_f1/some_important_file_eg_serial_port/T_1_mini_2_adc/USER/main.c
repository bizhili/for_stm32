#include "led.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "lcd.h"
#include "adc.h"
u8 reverse=1;
float adc[6];
  u16 Res; 	
 int main(void)
 { 
	u8 sign=0;
	u8 len,t;
	u16 adcx,times=0;
	delay_init();	    	 //延时函数初始化	  
	uart_init(9600);	 	//串口初始化为9600
	LED_Init();		  		//初始化与LED连接的硬件接口
 	LCD_Init();
 	Adc_Init();		  		//ADC初始化	    
	POINT_COLOR=RED;//设置字体为红色 
	LCD_ShowString(60,50,200,16,16,"Mini STM32");	
	LCD_ShowString(60,70,200,16,16,"ADC TEST");	
	POINT_COLOR=BLUE;//设置字体为蓝色
	LCD_ShowString(60,90,200,16,16,"your messege:");	    
	while(1)
	{
		adcx=Get_Adc_Average(ADC_Channel_1,10);	
		LCD_ShowxNum(90,110,Res,3,16,0);
		if(Res>=250)
		{
		sign=Res-250;
			while(Res>=250);
			adc[sign]=3*Res;
		}
		
		
	}											    
}	
 
void USART1_IRQHandler(void)                	//串口1中断服务程序
	{
	
#if SYSTEM_SUPPORT_OS 		//如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
	OSIntEnter();    
#endif
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
		{
		Res =USART_ReceiveData(USART1);	//读取接收到的数据	
									
     } 
#if SYSTEM_SUPPORT_OS 	//如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
	OSIntExit();  											 
#endif
}
