/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       main.c/h
  * @brief      stm32初始化以及开始任务freeRTOS。h文件定义相关全局宏定义以及
  *             typedef 一些常用数据类型
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
	
		#include "stm32f4xx.h"

		#include "adc.h"
		#include "buzzer.h"
		#include "can.h"
		#include "delay.h"
		#include "flash.h"
		#include "fric.h"
		#include "laser.h"
		#include "led.h"
		#include "power_ctrl.h"
		#include "rc.h"
		#include "rng.h"
		#include "sys.h"
		#include "timer.h"
#include "CAN_Receive.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "calibrate_task.h"
#include "remote_control.h"
#include "start_task.h"
#include "main.h"
#include "Gimbal_Task.h"
#include "gimbal_behaviour.h"
#include "stm32f4xx.h"
#include "Gimbal_Task.h"
#include "adc.h"
#include "buzzer.h"
#include "can.h"
#include "delay.h"
#include "flash.h"
#include "fric.h"
#include "laser.h"
#include "led.h"
#include "power_ctrl.h"
#include "rc.h"
#include "rng.h"
#include "sys.h"
#include "timer.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "calibrate_task.h"
#include "remote_control.h"
#include "start_task.h"
#include "chassis_task.h"
#include "math.h"
void BSP_init(void);
void shoot_init(void);
void pid1(int x,int y,int *p);
const motor_measure_t *motor1,*motor2,*motor3;//云台电机1，2接受数据用，含pid
const  motor_measure_t *motora1,*motora2,*motora3,*motora4;//底盘电机1，2，3，4接受数据，含pid
const RC_ctrl_t *rc1;
int 	pid2(int x);	
u8 USART2_RX_BUF[32]; //接收缓冲,最大USART2_MAX_RECV_LEN个字节.
u16 USART2_RX_STA = 0;
u8 reverse=1;
u8 Res=0;
u16 anglex=0,angley=0,anglex2=0,angley2=0;
int main(void)
{ 
	u8 t,len;
	u16 times=0;//服务于串口
	
	BSP_init();
	motor1=get_Yaw_Gimbal_Motor_Measure_Point();//云台1返回
	motor2=get_Pitch_Gimbal_Motor_Measure_Point();//云台2返回
	motor3=get_Trigger_Motor_Measure_Point();//拨盘电机返回
	motora1=get_Chassis_Motor_Measure_Point(0x201);
	motora2=get_Chassis_Motor_Measure_Point(0x202);
	motora3=get_Chassis_Motor_Measure_Point(0x203);
	motora4=get_Chassis_Motor_Measure_Point(0x204);//地盘电机1，2，3，4返回
	rc1=get_remote_control_point();
	Adc_Init();    
	SPI_OLED_Init();
	SPI6Init();
	OLED_Init();
	int pid[8];//pid用8字节数组
	u8 adcx;//用于电流adc
	float Adc1;//用于电流adc
	oled_clear(Pen_Clear);//清除显示界面		
	delay_ms(200);
	fric1_on(1000),fric2_on(1000);
	int a=1000,b=-2500,e=0,count=0,vecd=a=motor3->ecd,h=0;//a，b用于摩擦轮控制
	usart2_init(115200);
	
		while (1)
				{	
						
					//if(1)
					//USART_SendData(USART2, 8);	
					led_red_off();
					int vpid=0,dv1=0,dv2=0,dv3=0,dv4=0,v1=0,v2=0,v3=0,v4=0;
					int x=rc1->rc.ch[0]-1024,y=rc1->rc.ch[1]-1024,z=rc1->rc.ch[2]-1024,w=rc1->rc.ch[3]-1024,c,d;
					e++;//用于显示屏防漂移
					oled_display_on();//oled显示开，每次开防漂移
					oled_shownum(1,0,         22, 1, 6);//oled显示
					oled_shownum(1,7, 				motor1->ecd, 1, 6);
					oled_shownum(2,1, 				motor2->ecd, 1, 6);
					
					oled_shownum(3,2,     motora2->speed_rpm, 1, 6);
					oled_shownum(3,9, 		Res, 1, 6);
					oled_shownum(4,4,  anglex, 1, 6);
					oled_shownum(4,11, angley, 1, 6);
					
					
					oled_refresh_gram();						
			if(rc1->rc.s[0]==2&&rc1->rc.s[1]==2)//云台普通控制模式
				{
					
				int vx,vy,vs,tx=x*3,ty=y*2;
					pid1(tx,ty,&pid[0]);
					vx=pid[0];
					vy=pid[1];
					if(w<-500)
					{
					a+=3;
					delay_ms(3);
					}
					else
						a=1000;
						if (a>=1400)
						a=1400;
					fric1_on(a),fric2_on(a);
						if(w<-300&a>=1400)//云台到位后允许开启射击
						{
							delay_ms(20);
							count++;
								if(count<=12)
								{
									h=-2000; 		
								}
								else
								{h=10000-motor3->speed_rpm*10;}
								if(count>80)
									count=0;
						}
						else
						{	h=0;		buzzer_off();}
						CAN_CMD_GIMBAL(vx,vy,h,1000);
				}
				
				
				
				if(rc1->rc.s[0]==3&&rc1->rc.s[1]==2 )//auto_mode1,云台数值自动模式
				{
					
					int vx,vy,vs,tx=22.3*(anglex-132),ty=-22.3*(angley-50);
					pid1(tx,ty,&pid[0]);
					vx=pid[0];
					vy=pid[1];
					if(w<-500)
					{
					a+=3;
					delay_ms(3);
					}
					else
						a=1000;
						if (a>=1400)
						a=1400;
					fric1_on(a),fric2_on(a);
						if(w<-300&a>=1400)//云台到位后允许开启射击
						{
							delay_ms(20);
							count++;
								if(count<=12)
								{
									h=-2000; 		
								}
								else
								{h=10000-motor3->speed_rpm*10;}
								if(count>80)
									count=0;
						}
						else
						{	h=0;		buzzer_off();}
						CAN_CMD_GIMBAL(vx,vy,h,1000);
					}
				
				
				
				
				
				if(rc1->rc.s[0]==1&&rc1->rc.s[1]==2 )//pid_chassis_mode,地盘pid模式
				{
					CAN_CMD_GIMBAL(0,0,0,0);
					if(x==0&&y==0&&z==0&&w==0)
					CAN_CMD_CHASSIS(0,0,0,0);
					vpid=(-motora1->speed_rpm-motora2->speed_rpm+motora3->speed_rpm+motora4->speed_rpm)/4;
					v1=-motora1->speed_rpm,v2=-motora2->speed_rpm,v3=motora3->speed_rpm,v4=motora4->speed_rpm;
					dv1=vpid-v1,dv2=vpid-v2,dv3=vpid-v3,dv4=vpid-v4;
					v1=y*3+dv4*4,v2=-y*3-dv1*4,v3=-y*3-dv2*4,v4=3*y+dv3*4;
				if(x!=0|y!=0)
						{
					if(((y-1.7*x>0)&(y+1.7*x>0))|((y-1.7*x<0)&(y+1.7*x<0)))
					{
					CAN_CMD_CHASSIS(v1,v2,v3,v4);}
				else if(((x-1.7*y>0)&(x+1.7*y>0))|((x-1.7*y<0)&(x+1.7*y<0)))
					CAN_CMD_CHASSIS(-x*4,-x*4,x*4,x*4);
			else
			{
			if(x*y<0)
				CAN_CMD_CHASSIS(-x*8,100,x*8,100);
			if(x*y>=0)
				CAN_CMD_CHASSIS(100,-y*8,100,y*8);
		}
				}
						if(z!=0)
			CAN_CMD_CHASSIS(z*2,z*2,z*2,z*2);
						
				}
				}						
				}
	
void	pid1(int x,int y,int *p)//云台pid,x为x坐标角度-800~4000，y为y坐标角度-500~1000（2000时垂直）
{
int xp,yp,dx,dy,vx,vy,px,py;
	if(y<=-200)
		y=-200;
	px=7400-x,py=6800-y;
	if(motor1->ecd<=3000)
		xp=motor1->ecd+8191;
	else
		xp=motor1->ecd;
	yp=motor2->ecd;
	dx=(px-xp),dy=-(py-yp);
	if(fabs(dx)>500)
	vx=dx*20;
	else if(fabs(dx)>300)
		vx=dx*40;
	else 
		vx=dx*70;
	if(fabs(dy)>=300)
	vy=-dy*40;
	else
	vy=-dy*70;
	*p=vx;
	*(p+1)=vy;
}

#define USART_RX_LEN 16
void USART2_IRQHandler(void)//串口2中断处理函数
{    
if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
Res=USART_ReceiveData(USART2);//(USART1->DR); //读取接收到的数
if(Res>=100&&Res<=200)
anglex=Res;
if(Res>30&&Res<100)
angley=Res;
}  


//四个24v 输出 依次开启 间隔 709us
#define POWER_CTRL_ONE_BY_ONE_TIME 709

void BSP_init(void)
{
    //中断组 4
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    //初始化滴答时钟
    delay_init(configTICK_RATE_HZ);
    //流水灯，红绿灯初始化
    led_configuration();
    //stm32 板载温度传感器初始化
    temperature_ADC_init();
#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE
    //stm32 随机数发生器初始化
    RNG_init();
#endif
    //24输出控制口 初始化
    power_ctrl_configuration();
    //摩擦轮电机PWM初始化
    fric_PWM_configuration();
    //蜂鸣器初始化
    buzzer_init(30000, 90);
    //激光IO初始化
    laser_configuration();
    //定时器6 初始化
    TIM6_Init(60000, 90);
    //CAN接口初始化
    CAN1_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);
    CAN2_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);

    //24v 输出 依次上电
    for (uint8_t i = POWER1_CTRL_SWITCH; i < POWER4_CTRL_SWITCH + 1; i++)
    {
        power_ctrl_on(i);
        delay_us(POWER_CTRL_ONE_BY_ONE_TIME);
    }
    //遥控器初始化
    remote_control_init();
    //flash读取函数，把校准值放回对应参数
    cali_param_init();
}
