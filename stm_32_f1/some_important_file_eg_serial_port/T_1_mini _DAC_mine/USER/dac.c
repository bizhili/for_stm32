#include "pin.h"
#include "led.h"
#include "delay.h"
#include "sys.h"
#include "pwm.h"
#include "motor.h"
#include "dac.h"

void init_dac()
	{	
		GPIO_InitTypeDef GPIO_InitStructure;//gpio初始化的结构体
		DAC_InitTypeDef DAC_InitStruct;//DAC初始化结构体		//打开GPIOA时钟，DAC接在PA4上
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);		//打开DAC时钟，是DAC不是ADC，刚开始我打开的是ADC的时钟，找了好久才找到原因
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC,ENABLE);		//DAC输出时IO要配置为模拟输入，《STM32中文参考手册 》12.2章节最后部分有解释	
		//打开DAC功能后，PA4会自动连接到DAC输出，《TM32中文参考手册 》12.2章节最后部分有解释	
		GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AIN;		
		GPIO_InitStructure.GPIO_Pin=GPIO_Pin_4;	
		GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;	
		GPIO_Init(GPIOA,&GPIO_InitStructure);		
		DAC_DeInit();	//这里的每个配置都可以查看具体配置的是寄存器的哪个位，注释在STM32F10X_DAC.H文件里的第130-135行，其实就是DAC_Init()这个函数定义的地方
		DAC_InitStruct.DAC_LFSRUnmask_TriangleAmplitude=DAC_LFSRUnmask_Bit0;//这个参数只是在波形发生器才会用到	
		DAC_InitStruct.DAC_OutputBuffer=DAC_OutputBuffer_Disable;//不输出缓存	
		DAC_InitStruct.DAC_Trigger=DAC_Trigger_None;//不需要触发事件	
		DAC_InitStruct.DAC_WaveGeneration=DAC_WaveGeneration_None;//不使用波形发生器	
		DAC_Init(DAC_Channel_1,&DAC_InitStruct);//这里使用的是DAC的通道1	
		DAC_Cmd(DAC_Channel_1,ENABLE);//使能DAC		
		DAC_SetChannel1Data(DAC_Align_8b_R,0);//初始化输出0v
		}
	



		