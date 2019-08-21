#include "tim2.h"


//高级定时器1pwm输出初始化
//arr：自动重装值（周期）  psc：时钟预分频数
void tim2_encoderInit(uint16_t arr, uint16_t psc)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef  TIM_ICInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); // 使能定时器1的外设时钟
 	RCC_APB2PeriphClockCmd(TIM2_CH1_GPIO_CLK | TIM2_CH2_GPIO_CLK, ENABLE);  //使能GPIO外设时钟使能                               	

   //设置该引脚为复用输出功能,输出TIM1 CH1的PWM脉冲波形
	GPIO_InitStructure.GPIO_Pin = TIM2_CH1_PIN | TIM2_CH2_PIN; //TIM_CH1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  //复用推挽输出
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(TIM2_CH1_PORT, &GPIO_InitStructure);
  GPIO_Init(TIM2_CH2_PORT, &GPIO_InitStructure);
	
	TIM_TimeBaseStructure.TIM_Period = arr; //(ENCODER_RESOLUTION * 4) - 1; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 80K
	TIM_TimeBaseStructure.TIM_Prescaler = psc; //设置用来作为TIMx时钟频率除数的预分频值  不分频
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
  
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1| TIM_Channel_2;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x06;
  TIM_ICInit(TIM2,&TIM_ICInitStructure);
  
  TIM_EncoderInterfaceConfig(TIM2,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);
  
  //TIM_ClearFlag(TIM2, TIM_FLAG_Update);
	//TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
  TIM_SetCounter(TIM2,0);
  
  TIM_Cmd(TIM2, ENABLE);  //使能TIM1
}




