/*
author 月华敝之
based on DJ
*/
#include "stm32f4xx.h"
#include "main.h"
#include "math.h"
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
#include "calibrate_task.h"
#include "remote_control.h"
#include "start_task.h"
#include "Gimbal_Task.h"
#include "gimbal_behaviour.h"
#include "chassis_task.h"

#include "..\user\APP\sensor1.h"

void TIM7_IRQHandler(void);
u8 return_zero();
u8 grab_open(void);
u8 grab_close(void);
void BSP_init(void);//初始化声明
u8 x_goto(u16 x);
u8 gimbal_goto(int x);
u8 roll_clockotherwise(void);
u8 roll_clockwise(void);
void shoot_init(void);//射击初始化申明
void pid1(int x,int y,int *p);
u8 auto_pilot(int x, int y,int r);
u8 y_goto(u16 y);
int time_count=80;
u8 justify=0,justify2=0,tim4_j=0,tim5_j=0,justifyx=0,flagx=0,flagy=0,flag_grab=0,flagw=0;
u16 grab_count=80;
u8 roll_count=0,flag_roll=0,flag_gimbal=0;
int vx=0,tx=0;
u8 flag_tx=0;
int pwm_count=1,pwm_count2=1;
const motor_measure_t *motor1,*motor2,*motor3;//云台电机1，2接受数据用，含pid
const  motor_measure_t *motora1,*motora2,*motora3,*motora4,*motora5,*motora6,*motora7,*motora8;//底盘电机1，2，3，4接受数据，含pid
const RC_ctrl_t *rc1;//遥控器指针
int pid2(int x);
int main(void)
{ 
	BSP_init();//总初始化
	IWDG_Init(4,500);//start watch-door dog	
	delay_ms(1);//延时启动
	SPI_OLED_Init();//以下三个初始化提供给显示屏
	SPI6Init();
	OLED_Init();
	int a,b,c;//用于步进电机
	u16 e;//提供给显示屏防漂移
	oled_clear(Pen_Clear);//清除显示界面		
	motor3=get_Trigger_Motor_Measure_Point();//云台电机返回
	motora1=get_Chassis_Motor_Measure_Point(0x201);
	motora2=get_Chassis_Motor_Measure_Point(0x202);
	motora3=get_Chassis_Motor_Measure_Point(0x203);
	motora4=get_Chassis_Motor_Measure_Point(0x204);//地盘电机1，2，3，4返回
	motora5=get_Yaw_Gimbal_Motor_Measure_Point();
	motora6=get_Pitch_Gimbal_Motor_Measure_Point();
	motora7=get_Chassis_Motor_Measure_Point(0x203);
	motora8=get_Chassis_Motor_Measure_Point(0x204);
	rc1=get_remote_control_point();//遥控器得到竖直
	int vpid=0,dv1=0,dv2=0,dv3=0,dv4=0,v1=0,v2=0,v3=0,v4=0,v5=0,v6=0;
	TIM2_PWM3_Init(500,100000,180);//motor1,2,3pwm初始化
	//TIM4_PWM_Init(5000-1,180-1,1800);//for pwm test
	TIM5_PWM2_Init(500-1,100000,180-1);
	TIM4_PWM_Init(500-1,100000,180-1);
	fric_PWM_configuration(350,10,180);
	u8 k1;
	u16 xcome=0,ycome=0;
		while (1)
				{
					IWDG_ReloadCounter();//feed dog
					rc1=get_remote_control_point();//遥控器得到竖直
					k1=GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_0);
					if (k1)
					{
						delay_ms(2);
						k1=GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_0);
					}
					//TIM4_PWM_Init(12000-1,180-1,605);//for pwm test//大于325顺时针小慢，小于210逆时针小块
					int x=rc1->rc.ch[0]-1024,y=rc1->rc.ch[1]-1024,z=rc1->rc.ch[2]-1024,w=rc1->rc.ch[3]-1024;//遥控器xyzw
					e++;//用于显示屏防漂移
					oled_display_on();//oled显示开，每次开防漂移
					
					
					
					oled_shownum(1,1,xcome, 1, 4);//第一，二值代表显示首位字坐标xy，第三值放置显示的数（不可为负数），四值默认1，五值表示数字长度
					oled_shownum(1,7,ycome, 1, 4);
					oled_shownum(2,1,flag_tx, 1, 4);
					oled_shownum(3,9,tx, 1, 4);
					oled_shownum(4,9,motor3->ecd, 1, 4);
					
					oled_refresh_gram();	//用于显示屏防漂移					
					if(rc1->rc.s[0]==1&&rc1->rc.s[1]==1)//pid_chassis_mode,地盘pid模式
					{
						pwm_count=1;
						pwm_count2=1;
						buzzer_off();
						justify=0;
						justify2=0;
						justifyx=0;
						flagx=0;
						flagy=0;
						flagw=0;
						flag_tx=0;
						TIM2_PWM3_Init(500,100000,180);//地盘操作时
						TIM5_PWM2_Init(500,100000,180);
						TIM4_PWM_Init(500,100000,180);
						if(x==0&&y==0&&z==0&&w==0)//遥控器无值时地盘无力，云台电机无力
							CAN_CMD_CHASSIS(0,0,0,0);
				if((x!=0)||(y!=0))//前后左右
				{
					if(((y-1.7*x>0)&&(y+1.7*x>0))|((y-1.7*x<0)&&(y+1.7*x<0)))//前后
					{
						//vpid=(-motora1->speed_rpm -motora2->speed_rpm +motora3->speed_rpm +motora4->speed_rpm)/4;//取四电机均值	
						vpid=y*15;
						v1=-motora1->speed_rpm,v2=-motora2->speed_rpm,v3=motora3->speed_rpm,v4=motora4->speed_rpm;//先用此四值取回返回值
						dv1=vpid-v1,dv2=vpid-v2,dv3=vpid-v3,dv4=vpid-v4;//求差
						v1=y*2+dv4*3,v2=-y*2-dv1*3,v3=-y*2-dv2*3,v4=2*y+dv3*3;//比例反馈
						CAN_CMD_CHASSIS(v1,v2,v3,v4);//输入
					}
				else if(((x-1.7*y>0)&&(x+1.7*y>0))|((x-1.7*y<0)&&(x+1.7*y<0)))//左右
					{
						//vpid=(motora1->speed_rpm -motora2->speed_rpm -motora3->speed_rpm +motora4->speed_rpm)/4;//取四电机均值	
						vpid=x*15;
						v1=motora1->speed_rpm,v2=-motora2->speed_rpm,v3=-motora3->speed_rpm,v4=motora4->speed_rpm;//先用此四值取回返回值
						dv1=vpid-v1,dv2=vpid-v2,dv3=vpid-v3,dv4=vpid-v4;//求差
						v1=x*2+dv4*3,v2=x*2+dv1*3,v3=-x*2-dv2*3,v4=-2*x-dv3*3;//比例反馈
						CAN_CMD_CHASSIS(v1,v2,v3,v4);//输入
					}
					
				else//斜走无pid
					{
						if(x*y<0)
							CAN_CMD_CHASSIS(100,x*7,100,-x*7);
						if(x*y>=0)
							CAN_CMD_CHASSIS(x*7,100,-x*7,100);
					}
				}
				if(z!=0)//原地转
				{
					//vpid=(motora1->speed_rpm +motora2->speed_rpm +motora3->speed_rpm +motora4->speed_rpm)/4;//取四电机均值	
					vpid=z*10;
					v1=motora1->speed_rpm,v2=motora2->speed_rpm,v3=motora3->speed_rpm,v4=motora4->speed_rpm;//先用此四值取回返回值
					dv1=vpid-v1,dv2=vpid-v2,dv3=vpid-v3,dv4=vpid-v4;//求差
					v1=z*2+dv4*3,v2=z*2+dv1*3,v3=z*2+dv2*3,v4=2*z+dv3*3;//比例反馈
					CAN_CMD_CHASSIS(v1,v2,v3,v4);//输入
				}
				if(w<-40)
				{
					TIM8_PWM1_Init(500,300,180);
				
				GPIO_SetBits(GPIOF, GPIO_Pin_10);
				}
				else if(w>40)
				{
					
			TIM8_PWM1_Init(500,300,180);
				GPIO_ResetBits(GPIOF, GPIO_Pin_10);
				}
				if(w==0)
				{
				TIM8_PWM1_Init(500,100000,180);
					
				}			
				
				//if w=0,the gimbal motor will be locked,or controled
				if(w!=0)
				{
					CAN_CMD_GIMBAL(0,0,-w*10,0);
					tx=motor3->ecd;
				}
				else
				{
				delay_ms(1);
				vx=(tx-motor3->ecd)*30;
				CAN_CMD_GIMBAL(0,0,vx,0);
				}
				
			}						

			
				else if(rc1->rc.s[0]==1&&rc1->rc.s[1]==3 )//操作手控制模式
				{
					pwm_count=1;
					pwm_count2=1;
					buzzer_off();
					justify=0;
					justify2=0;
					justifyx=0;
					flagx=0;
					flagy=0;
					flagw=0;
					flag_tx=0;
					a=b=c=100000;
					if(x==0&&y==0&&z==0&&w==0)
					{
					CAN_CMD_GIMBAL(0,0,0,0);
					v5=0;v6=0;
					}						
					if(y>=150)//motor1
					{
						v5=motora5->speed_rpm,v6=motora6->speed_rpm;//先用此四值取回返回值					
						v5=-y*30,v6=-y*30;
						a=3;
						GPIO_ResetBits(GPIOE,GPIO_Pin_4);					
					}	
					
					else if(rc1->rc.ch[1]-1024<=-150)
					{
						a=3;
						GPIO_SetBits(GPIOE,GPIO_Pin_4);						
					}
				
					CAN_CMD_GIMBAL(v5,v6,0,0);
					if(rc1->rc.ch[0]-1024>150)//motor2
					{
						b=3+22-(rc1->rc.ch[0]-1024)/30;
						GPIO_ResetBits(GPIOE,GPIO_Pin_5);
					}
					else if(rc1->rc.ch[0]-1024<=-150)
					{					
						b=3+22+(rc1->rc.ch[0]-1024)/30;
						GPIO_SetBits(GPIOE,GPIO_Pin_5);
					}											
					if(rc1->rc.ch[2]-1024>150)//motor3
					{
						c=7;
						GPIO_SetBits(GPIOE,GPIO_Pin_6);											
					}
					else if(rc1->rc.ch[2]-1024<=-150)
					{
						GPIO_ResetBits(GPIOE,GPIO_Pin_6);									
						c=7;			
					}															
					if(rc1->rc.ch[2]-1024<=-1023&&rc1->rc.ch[0]-1024<=-1023&&rc1->rc.ch[1]-1024<=-1023)
					{
						TIM2_PWM3_Init(500,100000,180);//motor1,2,3
						TIM5_PWM2_Init(500,100000,180);
						TIM4_PWM_Init(500,100000,180);
					}
					else
					{
						if(PC5==0&&PE4==0)
						a=100000;
						if(PC0==0&&PE5==0)
						b=100000;
						TIM2_PWM3_Init(350,c,180);//motor1,2,3
						TIM5_PWM2_Init(350,b,180);
						TIM4_PWM_Init(430,a,330);
					}
					
					/*
					if(w!=0)
					{
					CAN_CMD_GIMBAL(dv5,dv6,-10*w,10);
					tx=motor3->ecd;
					}
					else
					{
					delay_ms(20);
					vx=(tx-motor3->ecd);
					CAN_CMD_GIMBAL(dv5,dv6,vx*10,0);
					}
						*/
					if(w<-40)
				{
					TIM8_PWM1_Init(500,300,180);
				
				GPIO_SetBits(GPIOF, GPIO_Pin_10);
				}
				else if(w>40)
				{
					
			TIM8_PWM1_Init(500,300,180);
				GPIO_ResetBits(GPIOF, GPIO_Pin_10);
				}
				if(w==0)
				{
				TIM8_PWM1_Init(500,100000,180);
					
				}

					
					
				}
				
					else if(rc1->rc.s[0]==1&&rc1->rc.s[1]==2)//pid_chassis_mode,地盘模式
					{
						GPIO_SetBits(GPIOA, GPIO_Pin_4);//Gimbale disable
						pwm_count=1;
						pwm_count2=1;
						buzzer_off();
						justify=0;
						justify2=0;
						justifyx=0;
						flagx=0;
						flagy=0;
						flagw=0;
						flag_tx=0;
						GPIO_ResetBits(GPIOE,GPIO_Pin_4);
						GPIO_ResetBits(GPIOE,GPIO_Pin_5); 
						TIM2_PWM3_Init(500,100000,180);//地盘操作时
						TIM5_PWM2_Init(500,100000,180);
						if(x==0&&y==0&&z==0&&w==0)//遥控器无值时地盘无力，云台电机无力
							CAN_CMD_CHASSIS(0,0,0,0);
						if(PC5==0&&PE4==0)
					a=100000;
						else a=3;
						TIM4_PWM_Init(500,a,330);
						
						if(PC0==0&&PE5==0)
						b=100000;
						else b=5;
						TIM5_PWM2_Init(500,b,330);
						
						GPIO_ResetBits(GPIOE,GPIO_Pin_4);
						v1=v2=v3=v4=0;
				if((x!=0)||(y!=0))//前后左右
				{
					if(((y-1.7*x>0)&&(y+1.7*x>0))|((y-1.7*x<0)&&(y+1.7*x<0)))//前后
					{
						//vpid=(-motora1->speed_rpm -motora2->speed_rpm +motora3->speed_rpm +motora4->speed_rpm)/4;//取四电机均值	
						vpid=y*15;
						v1=-motora1->speed_rpm,v2=-motora2->speed_rpm,v3=motora3->speed_rpm,v4=motora4->speed_rpm;//先用此四值取回返回值
						dv1=vpid-v1,dv2=vpid-v2,dv3=vpid-v3,dv4=vpid-v4;//求差
						v1=y*2+dv4*3,v2=-y*2-dv1*3,v3=-y*2-dv2*3,v4=2*y+dv3*3;//比例反馈
						CAN_CMD_CHASSIS(v1,v2,v3,v4);//输入
					}
				else if(((x-1.7*y>0)&&(x+1.7*y>0))|((x-1.7*y<0)&&(x+1.7*y<0)))//左右
					{
						//vpid=(motora1->speed_rpm -motora2->speed_rpm -motora3->speed_rpm +motora4->speed_rpm)/4;//取四电机均值	
						vpid=x*15;
						v1=motora1->speed_rpm,v2=-motora2->speed_rpm,v3=-motora3->speed_rpm,v4=motora4->speed_rpm;//先用此四值取回返回值
						dv1=vpid-v1,dv2=vpid-v2,dv3=vpid-v3,dv4=vpid-v4;//求差
						v1=x*2+dv4*3,v2=x*2+dv1*3,v3=-x*2-dv2*3,v4=-2*x-dv3*3;//比例反馈
						CAN_CMD_CHASSIS(v1,v2,v3,v4);//输入
					}
					
				else//斜走无pid
					{
						if(x*y<0)
							CAN_CMD_CHASSIS(100,x*7,100,-x*7);
						if(x*y>=0)
							CAN_CMD_CHASSIS(x*7,100,-x*7,100);
					}
				}
				if(z!=0)//原地转
				{
					//vpid=(motora1->speed_rpm +motora2->speed_rpm +motora3->speed_rpm +motora4->speed_rpm)/4;//取四电机均值	
					vpid=z*10;
					v1=motora1->speed_rpm,v2=motora2->speed_rpm,v3=motora3->speed_rpm,v4=motora4->speed_rpm;//先用此四值取回返回值
					dv1=vpid-v1,dv2=vpid-v2,dv3=vpid-v3,dv4=vpid-v4;//求差
					v1=z*2+dv4*3,v2=z*2+dv1*3,v3=z*2+dv2*3,v4=2*z+dv3*3;//比例反馈
					CAN_CMD_CHASSIS(v1,v2,v3,v4);//输入
				}
				/*
				if(w!=0)
				{
					CAN_CMD_GIMBAL(-w*10,-w*10,-w*10,-w*10);
					tx=motor3->ecd;
				}
				else
				{
				delay_ms(20);
				vx=(tx-motor3->ecd)*30;
				CAN_CMD_GIMBAL(0,0,vx,0);
					
				}
				*/
				
				
				TIM8_PWM1_Init(500,100000,180);
				
			}
					
			
			
			else if(rc1->rc.s[0]==3&&rc1->rc.s[1]==2)//test mode
					{
						if(flag_tx==0)
						tx=380;
						else if(flag_tx==1)
							tx=6490;
						else if(flag_tx==2)
							tx=380;
						else if(flag_tx==3)
							tx=2420;
						else if(flag_tx==4)
							tx=380;
						else if(flag_tx==5)
							tx=2420;
						else if(flag_tx==6)
							tx=2383;
						else if(flag_tx==7)
							tx=2383;
						TIM8_PWM1_Init(500,100000,180);
						flag_gimbal=0;
						if(PC4==0)
						led_red_on();
						else
							led_red_off();
						if(PA4==0)
							led_green_on();
						else
							led_green_off();
						
						
						if(PC5==1&&PC0==1&&justify==0&&justify2==0&&justifyx==0&&flagx==0&&flagy==0)
							return_zero();	
						else
						{
								justifyx=1;				
						}
						
						 if((PC0==0||justify==1)&&flagx==0)
						{
							delay_us(500);
							if(PC0==0||justify==1)
								if(x_goto(1))
								{
									delay_us(500);
										flagx++;	
								}									
						}										
						if((PC5==0||justify2==1)&&flagy==0)//step motor2
						{
							delay_us(500);
							if(PC5==0||justify2==1)
							if(y_goto(1))
							{
								delay_us(500);
								flagy++;	
							}								
						}
				//****************************************************************//
					if(x==0&&y==0&&z==0)//遥控器无值时地盘无力，云台电机无力
							CAN_CMD_CHASSIS(0,0,0,0);
						
				if((x!=0)||(y!=0))//前后左右
				{
					if(((y-1.7*x>0)&&(y+1.7*x>0))|((y-1.7*x<0)&&(y+1.7*x<0)))//前后
					{
						//vpid=(-motora1->speed_rpm -motora2->speed_rpm +motora3->speed_rpm +motora4->speed_rpm)/4;//取四电机均值	
						vpid=y*15;
						v1=-motora1->speed_rpm,v2=-motora2->speed_rpm,v3=motora3->speed_rpm,v4=motora4->speed_rpm;//先用此四值取回返回值
						dv1=vpid-v1,dv2=vpid-v2,dv3=vpid-v3,dv4=vpid-v4;//求差
						v1=y*2+dv4*3,v2=-y*2-dv1*3,v3=-y*2-dv2*3,v4=2*y+dv3*3;//比例反馈
						CAN_CMD_CHASSIS(v1,v2,v3,v4);//输入
					}
				else if(((x-1.7*y>0)&&(x+1.7*y>0))|((x-1.7*y<0)&&(x+1.7*y<0)))//左右
					{
						//vpid=(motora1->speed_rpm -motora2->speed_rpm -motora3->speed_rpm +motora4->speed_rpm)/4;//取四电机均值	
						
						vpid=x*15;
						v1=motora1->speed_rpm,v2=-motora2->speed_rpm,v3=-motora3->speed_rpm,v4=motora4->speed_rpm;//先用此四值取回返回值
						dv1=vpid-v1,dv2=vpid-v2,dv3=vpid-v3,dv4=vpid-v4;//求差
						v1=x*2+dv4*3,v2=x*2+dv1*3,v3=-x*2-dv2*3,v4=-2*x-dv3*3;//比例反馈
						CAN_CMD_CHASSIS(v1,v2,v3,v4);//输入
					}
					
				else//斜走无pid
					{
						if(x*y<0)
							CAN_CMD_CHASSIS(100,x*7,100,-x*7);
						if(x*y>=0)
							CAN_CMD_CHASSIS(x*7,100,-x*7,100);
					}
				}
				if((z<=-200||z>=200))//原地转
				{
					if(z<=-150)
						z=(z+150)/2;
					if(z>=150)
						z=(z-150)/2;
					
					vpid=z*10;
					v1=motora1->speed_rpm,v2=motora2->speed_rpm,v3=motora3->speed_rpm,v4=motora4->speed_rpm;//先用此四值取回返回值
					dv1=vpid-v1,dv2=vpid-v2,dv3=vpid-v3,dv4=vpid-v4;//求差
					v1=z*2+dv4*3,v2=z*2+dv1*3,v3=z*2+dv2*3,v4=2*z+dv3*3;//比例反馈
					
					CAN_CMD_CHASSIS(v1,v2,v3,v4);//输入
				}	
				//***************************************************************//
						buzzer_off();
						if(flagw==0)
							flagw=1;
							xcome=pwm_count;
							ycome=pwm_count2;
						if(w<-658&&flagx==1&&flagw==1)
							if(x_goto(102))
							flagx++;
						if(w<-658&&flagy==1&&flagw==1)
							if(y_goto(840))
							flagy++;
            if(w>=-400&&(flagx==2||flagy==2)&&flagw==1)
						flagw++;
						
						
	   				if(w<-658&&flagx==2&&flagw==2)
							if(x_goto(153))
							flagx++;
						if(w<-658&&flagy==2&&flagw==2)
							if(y_goto(660))
							flagy++;
						if(w>=-400&&(flagx==3||flagy==3)&&flagw==2)
						flagw++;
					
							
						if(w<-658&&flagx==3&&flagw==3)
							if(grab_close())
							if(x_goto(40))
							flagx++;
						if(w<-658&&flagy==3&&flagw==3)
							if(grab_close())
							if(y_goto(1))
							flagy++;
						if(w>=-400&&(flagx==4||flagy==4)&&flagw==3)
						flagw++;
						
						if(w<-658&&flagx==4&&flagw==4)
							if(x_goto(240))
							flagx++;
						if(w<-658&&flagy==4&&flagw==4)
							if(y_goto(1))
							flagy++;
						if(w>=-400&&(flagx==5||flagy==5)&&flagw==4)
						flagw++;
						
						if(w<-658&&flagx==5&&flagw==5)
							if(x_goto(110))
							{
							flagx++;
								flag_tx=1;
							}
						if(w<-658&&flagy==5&&flagw==5)
						{
							if(y_goto(285))
							flagy++;//flag_tx++;//1
						}
						if(w>=-400&&(flagx==6||flagy==6)&&flagw==5)
						flagw++;
						
						
						
						if(w<-658&&flagx==6&&flagw==6)
							if(x_goto(212))
							flagx++;
						if(w<-658&&flagy==6&&flagw==6&&flagx==7)						
							if(y_goto(610))
							flagy++;
						if(w>=-400&&(flagx==7||flagy==7)&&flagw==6)
						flagw++;
						
						
						
						if(w<-658&&flagx==7&&flagw==7)
							if(x_goto(212))
							flagx++;
						if(w<-658&&flagy==7&&flagw==7&&flagx==8)						
							if(y_goto(645))
							flagy++;
						if(w<-658&&(flagx==8&&flagy==8)&&flagw==7)//
						flagw++;
						
						
						if(w<-658&&flagx==8&&flagw==8)
							if(grab_open())
							if(x_goto(214))
							flagx++;
						if(w<-658&&flagy==8&&flagw==8)	
               if(grab_open())							
							if(y_goto(300))
							flagy++;
						if(w<=-658&&(flagx==9&&flagy==9)&&flagw==8)//
						flagw++;
						
						
						//TWO
						if(w<-658&&flagx==9&&flagw==9)/////////////////////////////////////////////////////////////////			
							if(x_goto(30))
							{
							flagx++;
							flag_tx=6;
							}
						if(w<-658&&flagy==9&&flagw==9)
							if(y_goto(660))
						
							flagy++;
						if(w>=-400&&(flagx==10||flagy==10)&&flagw==9)
						flagw++;
						
						if(w<-658&&flagx==10&&flagw==10)
						{
							if(grab_close())
							if(x_goto(40)&&roll_clockwise())
							{
							flagx++;
							flag_tx=2;
							//flag_tx++;//2
							}
						}
						if(w<-658&&flagy==10&&flagw==10)
							if(grab_close())
							if(y_goto(1))
							flagy++;
						if(w>=-400&&(flagx==11||flagy==11)&&flagw==10)
						flagw++;
						
						if(w<-658&&flagx==11&&flagw==11)
							if(x_goto(240))
							flagx++;
						if(w<-658&&flagy==11&&flagw==11)
							if(y_goto(1))
							{
							flagy++;
							roll_count=140;
							}
						if(w>=-400&&(flagx==12||flagy==12)&&flagw==11)
						flagw++;
						
						if(w<-658&&flagx==12&&flagw==12)
						{
							if(x_goto(147)&&roll_clockwise())//162->150
							{
								//flag_tx++;//3
							flagx++;
							}
						}
						if(w<-658&&flagy==12&&flagw==12)
						{
							flag_tx=3;
							if(y_goto(285))
							flagy++;
						}
						if(w>=-400&&(flagx==13||flagy==13)&&flagw==12)
						flagw++;
						
						
						if(w<-658&&flagx==13&&flagw==13)
							if(x_goto(49))
							flagx++;
						if(w<-658&&flagy==13&&flagw==13&&flagx==14)							
						if(y_goto(610))
							flagy++;
						if(w>=-400&&(flagx==14||flagy==14)&&flagw==13)
						flagw++;
						
						
						if(w<-658&&flagx==14&&flagw==14)
							if(x_goto(49))
							flagx++;
						if(w<-658&&flagy==14&&flagw==14&&flagx==15)							
						if(y_goto(645))
							flagy++;
						if(w<=658&&(flagx==15&&flagy==15)&&flagw==14)//
						flagw++;
						
			
						if(w<-658&&flagx==15&&flagw==15)
							if(grab_open())
							if(x_goto(47))
							flagx++;
						if(w<-658&&flagy==15&&flagw==15)
							if(grab_open())
							if(y_goto(270))
							flagy++;
						if(w<=658&&(flagx==16&&flagy==16)&&flagw==15)//
						flagw++;
						
						
						//THREE
						if(w<-658&&flagx==16&&flagw==16)					
							if(x_goto(194))
							{
							flagx++;
							flag_tx=7;
							}
						if(w<-658&&flagy==16&&flagw==16)
							if(y_goto(660))
							flagy++;
						if(w>=-400&&(flagx==17||flagy==17)&&flagw==16)
						flagw++;
						
						if(w<-658&&flagx==17&&flagw==17)
							if(grab_close())
							if(x_goto(40))
							{
							flagx++;
							flag_tx=4;
							}
						if(w<-658&&flagy==17&&flagw==17)
						{
							if(grab_close())
							if(y_goto(1))
							if(roll_clockotherwise())
							{
							//flag_tx++;//4
							flagy++;
							}
						}
						if(w>=-400&&(flagx==18||flagy==18)&&flagw==17)
						flagw++;
						
						
						if(w<-658&&flagx==18&&flagw==18)
							if(x_goto(240))
							flagx++;
						if(w<-658&&flagy==18&&flagw==18)
							if(y_goto(1))
							flagy++;
						if(w>=-400&&(flagx==19||flagy==19)&&flagw==18)
						flagw++;
						 
						if(w<-658&&flagx==19&&flagw==19)
							if(x_goto(147))
							flagx++;
						if(w<-658&&flagy==19&&flagw==19)
						{
							flag_tx=5;
							if(y_goto(285))
								if(roll_clockwise())
								{
							//flag_tx++;//5
							flagy++;
								}
							}
						if(w>=-400&&(flagx==20||flagy==20)&&flagw==19)
						flagw++;
						
						
						if(w<-658&&flagx==20&&flagw==20)
							if(x_goto(147))//162->150
							flagx++;
						if(w<-658&&flagy==20&&flagw==20)							
							if(y_goto(610))
							flagy++;
						if(w>=-400&&(flagx==21||flagy==21)&&flagw==20)
						flagw++;
						
						
						if(w<-658&&flagx==21&&flagw==21)
							if(x_goto(147))
							flagx++;
						if(w<-658&&flagy==21&&flagw==21)							
							if(y_goto(645))
							flagy++;
						if(w>=-400&&(flagx==22||flagy==22)&&flagw==21)
						flagw++;
						
						
						if(w<-658&&flagx==22&&flagw==22)
							if(grab_open())
							if(x_goto(147))
							flagx++;
						if(w<-658&&flagy==22&&flagw==22)
							if(grab_open())
							if(y_goto(600))
							flagy++;
						if(w>=-400&&(flagx==23||flagy==23)&&flagw==22)
						flagw++;
						
						/*
						if(z<-150&&(w>-650&&w<650))
				{
					if(z>-300)
					TIM8_PWM1_Init(500,400,180);
					else if(z>-500)
					TIM8_PWM1_Init(500,300,180);
					else
					TIM8_PWM1_Init(500,100,180);	
				GPIO_ResetBits(GPIOF, GPIO_Pin_10);
					
				}
				else if(z>150&&(w>-650&&w<650))
				{
					if(z<300)
					TIM8_PWM1_Init(500,400,180);
					else if(z<500)
					TIM8_PWM1_Init(500,300,180);
					else
					TIM8_PWM1_Init(500,100,180);
				GPIO_SetBits(GPIOF, GPIO_Pin_10);
				}
				if((w<=-650||w>650)&&flag_roll==0)
				{
				TIM8_PWM1_Init(500,100000,180);
				}
				*/
				
				
				if(0)
				{
					if(tx<380)
					vx=(380-tx)*100;
					else if(tx>6600)
					vx=(6600-tx)*20;
					CAN_CMD_GIMBAL(0,0,vx,0);
				} 

				if(abs(tx-motor3->ecd)<8000)
				vx=(tx-motor3->ecd)*80;
				else
				vx=(tx-motor3->ecd);
				if(vx<-8000)
					vx=-8000;
				if(vx>8000)
					vx=8000;
				CAN_CMD_GIMBAL(0,0,vx,0);
				
					

				
		
						
				}
					
						
						
					
				
				else if(rc1->rc.s[0]==2&&rc1->rc.s[1]==2)//auto mode
					{
						GPIO_ResetBits(GPIOA, GPIO_Pin_4);//gimbal enable
						buzzer_off();
						if(z>200)
						grab_open();
						else if(z<-200)
						grab_close();	//grab control
											
						if(x>20)
						{
						xcome--;
							delay_ms(3);
						}
						else if(x<-20)
						{
						xcome++;
							delay_ms(3);
						
						}
						
						
							if (xcome<=1)
								xcome=1;
						if(xcome>=250)
							xcome=250;
						if(flagx>=0)
								x_goto(xcome);	
						if(y>20)
						{
						ycome--;
							delay_ms(2);
						}
						else if(y<-20)
						{
						ycome++;
							delay_ms(2);
						}
						
						if (ycome<=1)
								ycome=1;
						if(ycome>=840)
							ycome=840;
						if(flagy>=0)
							y_goto(ycome);
						
						/*
						if(w<-200&&w>-658)
				{
					TIM8_PWM1_Init(500,300,180);
				
				GPIO_SetBits(GPIOF, GPIO_Pin_10);
				}
				else if(w>200)
				{
					
			TIM8_PWM1_Init(500,300,180);
				GPIO_ResetBits(GPIOF, GPIO_Pin_10);
				}
				if(w==0||w<=-658)
				{
				TIM8_PWM1_Init(500,100000,180);
				}
				*/
					if(w!=0)
				{
					CAN_CMD_GIMBAL(0,0,-w*10,0);
					tx=motor3->ecd;
				}
				else
				{
				delay_ms(1);
				vx=(tx-motor3->ecd)*30;
				CAN_CMD_GIMBAL(0,0,vx,0);
				}	
				
				
				}
	
				else
				{
					TIM4_PWM_Init(500,100000,330);	
					TIM5_PWM2_Init(500,100000,330);
					justify=0;
					justify2=0;
					flagw=0;
					flagx=0;
					flagy=0;
				buzzer_on(11,15000);
						delay_ms(300);
				buzzer_on(3,15000);
						delay_ms(300);
				buzzer_on(5,15000);
						delay_ms(300);
					buzzer_on(5,15000);
						delay_ms(300);
				buzzer_on(3,15000);
						delay_ms(600);
				buzzer_on(4,15000);
						delay_ms(300);
					buzzer_on(3,15000);
						delay_ms(300);
				buzzer_on(2,15000);
						delay_ms(300);
				buzzer_on(4,15000);
						delay_ms(800);//warning
					
				CAN_CMD_CHASSIS(0,0,0,0);	
				}
				}						
				}
/*
void EXTI3_IRQHandler(void)//中断三处理程序
{
if(EXTI_GetITStatus(EXTI_Line3) != RESET)
{
led_green_on();
delay_ms(5);
EXTI_ClearITPendingBit(EXTI_Line3);
}
}
void EXTI1_IRQHandler(void)//中断一处理程序
{

if(EXTI_GetITStatus(EXTI_Line1) != RESET)
{
led_red_on();
delay_ms(10);
EXTI_ClearITPendingBit(EXTI_Line1);
}
}
*/			
//四个24v 输出 依次开启 间隔 709us
#define POWER_CTRL_ONE_BY_ONE_TIME 709
void TIM3_IRQHandler(void)//timer interrupt 
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
    {
        
        time_count++;
				if(tim5_j==1&&PE5==1)
					
				pwm_count++;
				else if(tim5_j==1&&PE5==0)
				pwm_count--;		
				if(tim4_j==1&&PE4==1)//step motor 2
					
				pwm_count2++;//step motor 1
				else if(tim4_j==1&&PE4==0)
				pwm_count2--;
				
				if (flag_grab==1)
				grab_count++;
				else if(flag_grab==2)
				grab_count--;
				
				if(grab_count>=80)
					grab_count=80;
				if(grab_count<=1)
					grab_count=1;
				
				if(flag_roll==1)
					roll_count++;
				else if(flag_roll==2)
					roll_count--;
				
				TIM_ClearFlag(TIM3, TIM_IT_Update);
				
    }
}

void TIM7_IRQHandler(void)
{
	//if(TIM_GetITStatus(TIM7,TIM_IT_Update)!=RESET)
		//time_count++;
	//if (time_count==60000000)
		//time_count=0;
	TIM_ClearFlag(TIM7,TIM_IT_Update);/*清中断标志*/
}




u8 roll_clockotherwise(void)
{
	return 1;
	if(0)
	{
flag_roll=1;
	if(roll_count<140)
	{
	
	TIM8_PWM1_Init(500,300,180);		
	GPIO_SetBits(GPIOF, GPIO_Pin_10);
	return 0;
	}
	else
	{
	TIM8_PWM1_Init(500,100000,180);		
	flag_roll=0;
	roll_count=140;
	return 1;
	}
}
	
	}

u8 roll_clockwise(void)
{
	return 1;
	if(0)
	{
flag_roll=2;
	if(roll_count>1)
	{
	TIM8_PWM1_Init(500,300,180);		
	GPIO_ResetBits(GPIOF, GPIO_Pin_10);
	return 0;
	}
	else
	{
	TIM8_PWM1_Init(500,100000,180);		
	flag_roll=0;
	roll_count=1;
	return 1;
	}
}
}



u8 grab_open(void)
{
	flag_grab=1;
if (grab_count<80)
{
	TIM2_PWM3_Init(350,4,180);
	GPIO_ResetBits(GPIOE,GPIO_Pin_6);
	return 0;
}
else
{
	TIM2_PWM3_Init(350,100000,180);
	grab_count=80;
	flag_grab=0;
	return 1;
}
}

u8 grab_close(void)//must first,remember open it max
{
	flag_grab=2;
if (grab_count>1)
{
	TIM2_PWM3_Init(350,4,180);
	GPIO_SetBits(GPIOE,GPIO_Pin_6);
	return 0;
}
else
{
	TIM2_PWM3_Init(350,100000,180);
	grab_count=1;
	flag_grab=0;
	return 1;
}
}


u8 return_zero(void)
	{
						u32 a,b;
						tim5_j=0;
						tim4_j=0;
						pwm_count=1;
						pwm_count2=1;
						GPIO_ResetBits(GPIOE,GPIO_Pin_4);
						GPIO_ResetBits(GPIOE,GPIO_Pin_5); 
						TIM2_PWM3_Init(500,100000,180);//地盘操作时
						TIM5_PWM2_Init(500,100000,180);
						CAN_CMD_CHASSIS(0,0,0,0);					
						if(PC5==0&&PE4==0)
						a=100000;
						else a=3;
						TIM4_PWM_Init(500,a,330);						
						if(PC0==0&&PE5==0)
						b=100000;
						else b=5;
						TIM5_PWM2_Init(500,b,330);
						if((PC5==1&&PC0==1)&&(justify==0&&justify2==0))
						return 1;
						else
						justifyx=1;
						return 0;
							
	}
	
u8 x_goto(u16 x)
	{
							u32 b;
							b=100000;
							justify=1;
							if(x>=250)
							x=250;
							if(pwm_count<=1)
								pwm_count=1;
							if(pwm_count<x)
							{
							tim5_j=1;
							b=5;
							GPIO_SetBits(GPIOE,GPIO_Pin_5);
							}							
							else if(pwm_count>x)
							{
							tim5_j=1;
							b=5;
							GPIO_ResetBits(GPIOE,GPIO_Pin_5);
							}			
							else
							{
							tim5_j=0;
							b=100000;
							TIM5_PWM2_Init(500,b,330);
							return 1;						
							}
							if(PC0==0&&PE5==0)
							{
							delay_us(50);
							if(PC0==0&&PE5==0)
								{
							b=100000;
							pwm_count=1;
								}
							}
							TIM5_PWM2_Init(500,b,330);
							return 0;
	}
	
u8 y_goto(u16 y)
	{
							u32 a;
							a=100000;
							justify2=1;
							if(y>=840)
							y=840;
							if(pwm_count2<=1)
								pwm_count2=1;
							if(pwm_count2<y)
							{
							tim4_j=1;
							a=3;
							GPIO_SetBits(GPIOE,GPIO_Pin_4);
							}	
							
							else if(pwm_count2>y)
							{
							tim4_j=1;
							a=3;
							GPIO_ResetBits(GPIOE,GPIO_Pin_4);
							}
							
							else
							{
							tim4_j=0;
							a=100000;
							TIM4_PWM_Init(500,a,330);
							return 1;						
							}
							if(PC5==0&&PE4==0)
							{
							delay_us(500);
							if(PC5==0&&PE4==0)
								{
							a=100000;
							pwm_count2=1;
								}
							}
							TIM4_PWM_Init(500,a,330);
							return 0;
	}
	
u8 auto_pilot(int x, int y,int r)
	{
						int vpid=y*15;
						int v1=-motora1->speed_rpm,v2=-motora2->speed_rpm,v3=motora3->speed_rpm,v4=motora4->speed_rpm;//先用此四值取回返回值
						int dv1=0,dv2=0,dv3=0,dv4=0;
						if(x==0&&y==0&&r==0)
						{
						y=300;
						vpid=y*15;
						v1=-motora1->speed_rpm,v2=-motora2->speed_rpm,v3=motora3->speed_rpm,v4=motora4->speed_rpm;//先用此四值取回返回值
						dv1=vpid-v1,dv2=vpid-v2,dv3=vpid-v3,dv4=vpid-v4;//求差
						if(PC2==1)
						{
						if(PA4==1&&PC4==1)
						v1=v2=v3=v4=0;
						else if(PA4==0&&PC4==1)
						v1=y+dv4,v2=-y*2-dv1*3,v3=-y*2-dv2*3,v4=y+dv3;
						else
						v1=y*2+dv4*3,v2=-y-dv1,v3=-y-dv2,v4=2*y+dv3*3;
						}
						else
						{
						if(PA4==1&&PC4==1)
						v1=y*2+dv4*3,v2=-y*2-dv1*3,v3=-y*2-dv2*3,v4=2*y+dv3*3;
						else if(PA4==0&&PC4==1)
						v1=y+dv4,v2=-y*2-dv1*3,v3=-y*2-dv2*3,v4=y+dv3;
						else
						v1=y*2+dv4*3,v2=-y-dv1,v3=-y-dv2,v4=2*y+dv3*3;		
						}
						CAN_CMD_CHASSIS(v1,v2,v3,v4);//输入
						}
	return 0;
	}

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
		vcc_Init();

    //蜂鸣器初始化
    buzzer_init(30000, 90);
    //激光IO初始化
    laser_configuration();
    //定时器6 初始化
    TIM6_Init(60000, 90);
    //CAN接口初始化
    CAN1_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);
    CAN2_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);
		TIM3_Init(180-1, 5000-1);
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
		Out_EXTI_Configuration1();
		Out_EXTI_Configuration2();
		gpio2_Init();//pf1
		sensor_Init();
		
}
