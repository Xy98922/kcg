/******************************************************************************************************
#2019-10-07
	1、添加ROS节点下发命令清零里程计数据功能
	
#2020-9-8
	1、更新了DBUS中的函数名称
	2、更新遥控器信号丢失造成的数据乱码引起“疯转”的问题
	3、统一4轮和2轮差速小车模型电机控制函数的单位为 m/s  和 rad/s      
	
	
* update 2019-10-02
* maker: crp
******************************************************************************************************/
#include "stm32f10x.h"
#include "stdio.h"//包含串口发送的FILE属性
#include "stdlib.h"

#include "stm32f10x_Delay.h"
 
// #include "speed_cntr.h"
#include "speed_control.h"
#include "bsp_uart.h"
#include "bsp_can.h"
#include "DBUS.h"
#include "DJI_Motor.h"
#include "ln3x.h"
#include "adc.h"
/*********************************************************/
/**********************宏定义****************************/
#define EnableInterrupt 	__set_PRIMASK(0) //总中断 写‘0‘ 开总中断   ’1‘关总中断
#define DisableInterrupt 	__set_PRIMASK(1) //总中断 写‘0‘ 开总中断   ’1‘关总中断
uint32_t TimingDelay; //用于delay控制的精确延时
uint16_t ADC_ConvertedValue; 
 
#define DEBUUG 1
#define DEBUUG_MATRIX_KEYSACN 0

#define LED1_FLIP  GPIO_Flip_level(GPIOE,GPIO_Pin_5) 
#define LED2_FLIP  GPIO_Flip_level(GPIOE,GPIO_Pin_6) 
 
/*********************************************************/
/******************全局变量声明***************************/

volatile uint8_t UART1_DMA_Flag=0x00;
volatile uint8_t UART2_Flag=0x00;
volatile uint8_t DC_flag=0x00;
volatile uint8_t CAN1_Flag=0x00;
volatile uint8_t MS_Flag=0x00; //主从控制标志位，=1从机模式，=0主机模式
volatile uint8_t MS=0x00;
volatile uint8_t LS_flag=0x00; //落锁标志位
volatile uint32_t Timer2_Counter1=0; //分别用来标记接收命令是否超过了限制范围
volatile uint32_t Timer2_Counter2=0;
volatile uint32_t Timer2_Counter3=0; 
volatile uint32_t Timer2_Counter4=0;
volatile uint32_t Timer2_Counter5=0; 
volatile uint32_t Timer2_Counter6=0;        //舵机限流保护
volatile int16_t  beforevalch7=0;
volatile int16_t  beforevalch8=0;
volatile int16_t  beforevalch13=0;
extern uint8_t loraRec_flag; //接收到lora数据

extern int set_v,set_spd[2];  //底盘四个电机目标速度
extern command_t recived_cmd; //底盘接收上位机命令结构体

extern CanRxMsg RxMessage;				               //接收缓冲区
extern uint8_t USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节
extern rc_info_t dbus_rc;               //DBUS 变量
extern moto_measure_t moto_chassis[2] ; //CAN读取电机状态数据变量 
extern moto_measure_t moto_info;

extern int set_spd[2]; 

extern sPkg Load_pkg;
extern u8 Recv_counter;
/*********************************************************/
/**********************函数声明***************************/
void IMU_Test_Upload_Message(void);
void S_Test_Upload_Message(void);
 
 
void Main_Delay(unsigned int delayvalue);
void Main_Delay_us(unsigned int delayvalue);
void recievePkg(sPkg pkg);
extern uint8_t USART1_RX_BUF[26];
extern int USART3Rec_speedx;
extern int USART3Rec_speedw;
extern uint8_t USART3Rec_flag;
/*********************************************************/
/*****************主函数**********************************/
int main(void)
{	
 
	   unsigned int i=0;
		 SysTick_Init();//初始化滴答定时器
	   My_GPIO_Init(GPIOE,GPIO_Pin_5,GPIO_Mode_Out_PP, GPIO_Speed_10MHz);//初始化LED端口
	   My_GPIO_Init(GPIOE,GPIO_Pin_6,GPIO_Mode_Out_PP, GPIO_Speed_10MHz);//初始化LED端口
	   My_GPIO_Init(GPIOD,GPIO_Pin_10,GPIO_Mode_Out_PP, GPIO_Speed_10MHz);//落锁引脚
	   My_GPIO_Init(GPIOD,GPIO_Pin_11,GPIO_Mode_Out_PP, GPIO_Speed_10MHz);//落锁引脚
	   USART1_Config(100000 ,1);                 //遥控器通信
	   My_Config_USART_Init(USART2,115200,1);	   //上位机or测试通信
	   My_Config_USART_Init(USART3,115200,1);		 //主从通信
	   My_Config_USART_Init(UART4,115200,0);     //485通信
	   UART_send_string(USART2,"MickX4 ROS Car \n");//字符串函数
		 DiffX2_Wheel_Speed_Model(0,0); //开机以后发送设置电机目标值为0 
		 Timer_2to7_counter_Generalfuncation(TIM2,1000);//定时中断时间为1ms
		 Timer_2to7_Generalfuncation_start(TIM2);
     UART_send_string(USART2,"Init Successful ....\n");//字符串函数
		 servoinit(dbus_rc.ch2-dbus_rc.ch2_offset);        //pwm初始化
//	   KEY_Init();
//	   EXTIX_Init();
	   Adc_Init();
	   GPIO_ResetBits(GPIOD,GPIO_Pin_10);
	   GPIO_SetBits(GPIOD,GPIO_Pin_11);
		 while(1)
			{		
				
						if(UART1_DMA_Flag) //遥控器介入控制命令逻辑  7ms 发送一次
						{					
              							
							Sbus_Data_Count(USART1_RX_BUF);
						  UART1_DMA_Flag=0x00;
							if(dbus_rc.ch10 == 66)  
								{
									GPIO_ResetBits(GPIOD,GPIO_Pin_10);
									GPIO_ResetBits(GPIOD,GPIO_Pin_11);									
								}
								
								if(dbus_rc.ch10 != 66)  //图传开关
								{								
									GPIO_ResetBits(GPIOD,GPIO_Pin_10);
									GPIO_SetBits(GPIOD,GPIO_Pin_11);								
								}	
							if((dbus_rc.sw1 !=1) && (dbus_rc.available)&&(MS_Flag==0)) //使能遥控器模式
							{				
								if(beforevalch13-dbus_rc.ch13!=0||beforevalch7-dbus_rc.ch7!=0||beforevalch8-dbus_rc.ch8!=0||(Get_Adc_Average(5)/4096*3300-2422)/60>=2||(Get_Adc_Average(5)/4096*3300-2422)/60<=-2)
							{
								TIM_Cmd(TIM3,ENABLE);    
								TIM_Cmd(TIM4,ENABLE); 
								Timer2_Counter6=0;							
							}
							if((Get_Adc_Average(5)/4096*3300-2422)/60>=2||(Get_Adc_Average(5)/4096*3300-2422)/60<=-2)
							{
							dbus_rc.ch7=54;
							dbus_rc.ch8=54;
							dbus_rc.ch13=54;
							}
                TIM_SetCompare1(TIM3,dbus_rc.ch7);	 
								TIM_SetCompare1(TIM4,dbus_rc.ch8);
                TIM_SetCompare2(TIM4,dbus_rc.ch13);	 								//新机对接	
								beforevalch7=dbus_rc.ch7;
								beforevalch8=dbus_rc.ch8;
								beforevalch13=dbus_rc.ch13;
							 if(Timer2_Counter6>=1000){								 							 
								TIM_Cmd(TIM3,DISABLE);    
								TIM_Cmd(TIM4,DISABLE);
							  Timer2_Counter6=1000;		            //数字舵机只发送一次数据就行					 
							 }    						
								Timer2_Counter1=0; //清空定时计数器
								if(dbus_rc.sw2 ==1) //1 档模式 最大1m/s
								{
									DiffX2_Wheel_Speed_Model((dbus_rc.ch2-dbus_rc.ch2_offset)*0.00152,(dbus_rc.ch1-dbus_rc.ch1_offset)*0.00152);
								  UART_send_string(USART2,"YIAKONGQISET:  ");UART_send_intdata(USART2,dbus_rc.ch2-dbus_rc.ch2_offset);UART_send_char(USART2,'\t');
                  UART_send_string(USART2,"ADC:  ");UART_send_intdata(USART2,Get_Adc());UART_send_char(USART2,'\t');
								}
								else if(dbus_rc.sw2 ==3) //2 档模式 最大2m/s
								{
									DiffX2_Wheel_Speed_Model((dbus_rc.ch2-dbus_rc.ch2_offset)*0.00304,(dbus_rc.ch1-dbus_rc.ch1_offset)*0.00304);
                  UART_send_string(USART2,"YIAKONGQISET:  ");UART_send_intdata(USART2,dbus_rc.ch2-dbus_rc.ch2_offset);UART_send_char(USART2,'\t');

								}
								else if(dbus_rc.sw2 ==2) //3 档模式 最大3.5m/s
								{
									DiffX2_Wheel_Speed_Model((dbus_rc.ch2-dbus_rc.ch2_offset)*0.0053,(dbus_rc.ch1-dbus_rc.ch1_offset)*0.0053);
                  UART_send_string(USART2,"YIAKONGQISET:  ");UART_send_intdata(USART2,dbus_rc.ch2-dbus_rc.ch2_offset);UART_send_char(USART2,'\t');

								}
								else
									DiffX2_Wheel_Speed_Model(0,0);
							}

         		LED1_FLIP;
						}
						
						if(UART2_Flag && recived_cmd.flag) //上位机指令下发接口
						{
								LED1_FLIP;
								if((dbus_rc.sw1 ==1) && (dbus_rc.available))//串口接收有数据过来
								{
										
									DiffX2_Wheel_Rpm_Model(0,0);
									Timer2_Counter1=0; //清空定时计数器
									if(recived_cmd.cmd == 0xF1)
									{	
										DiffX2_Wheel_Rpm_Model(recived_cmd.tag_rpm1,recived_cmd.tag_rpm2);

									}
									else if(recived_cmd.cmd == 0xF2)
									{
										DiffX2_Wheel_Speed_Model(recived_cmd.tag_speed_x,recived_cmd.tag_speed_w);

									}
									else if(recived_cmd.cmd == 0xF3)
									{
										DiffX2_Wheel_Speed_Model(recived_cmd.tag_speed_x,recived_cmd.tag_speed_w);

									}
									else if(recived_cmd.cmd == 0xE1) //里程计清零
									{
										 DJI_Motor_Clear_Odom();
									}
									else;
									//Delay_10us(50000);
									//DJI_Motor_Show_Message();
								}
								else
									DiffX2_Wheel_Rpm_Model(0,0);
//									Mecanum_Wheel_Rpm_Model(0,0,0,0);
					 
							recived_cmd.flag =0;//使用一次以后丢弃该数据
							UART2_Flag=0x00;
						}

            if(dbus_rc.ch9==1)
						{
							MS_Flag=0;
							MS=0;
						}
						else if(dbus_rc.ch9!=1)
            {
							MS_Flag=1;
							MS=1;
						}							
					
				    if(USART3Rec_flag==1 &&  MS_Flag==1)
						{
							DiffX2_Wheel_Speed_Model(USART3Rec_speedx*0.01,USART3Rec_speedw*0.01);
							USART3Rec_flag=0;
							Timer2_Counter1=0; 	
							UART_send_string(USART3,"USART3Rec_speedx:  ");UART_send_intdata(USART3,USART3Rec_speedx);UART_send_char(USART3,'\t');	
						  UART_send_string(USART3,"USART3Rec_speedw:  ");UART_send_intdata(USART3,USART3Rec_speedw);UART_send_char(USART3,'\t');
						}
						if((Timer2_Counter1>100*10)) // 如果定时计数器操作4s还没有被清零，说明通讯出现了中断
						{			
									if(dbus_rc.available == 0x00)
									{
										dbus_rc.sw1 = 5; //标记接收数据不可用
										DiffX2_Wheel_Rpm_Model(0,0);
//										Mecanum_Wheel_Rpm_Model(0,0,0,0);
									}
									else if(recived_cmd.flag ==0x00)
									{
										DiffX2_Wheel_Rpm_Model(0,0);
//										Mecanum_Wheel_Rpm_Model(0,0,0,0);
									}
									else if(loraRec_flag==0)
									{
										DiffX2_Wheel_Rpm_Model(0,0);
									}
									Timer2_Counter1=0;
						}
						
						if(CAN1_Flag) //打印CAN中断接收的数据
						{
							CAN1_Flag=0x00;  
							if(Timer2_Counter3 == 20) //1ms*20  50HZ 打印频率
							{
								LED2_FLIP;
								Timer2_Counter3=0;
								DJI_Motor_Upload_Message();
								//DJI_Motor_Show_Message();
							}
							else if(Timer2_Counter3 > 20)  
							{
								 Timer2_Counter3=0;
							}
						}
      }
     
 // exit from main() function
}
 
 //延时函数 6.3ms
//void Main_Delay(unsigned int delayvalue)
//{
//	unsigned int i;
//	while(delayvalue-->0)
//	{	
//		i=5000;
//		while(i-->0);
//	}
//}
void Main_Delay_us(unsigned int delayvalue)
{
//	unsigned int i;
	while(delayvalue-->0)
	{	
		;
	}
}

// 测试 ros接收IMU数据
void IMU_Test_Upload_Message(void)
{
		short IMU_test=1;
		static  uint32_t IMU_upload_counter=0;
		unsigned char senddat[70];
		unsigned char i=0,j=0;	
		unsigned int sum=0x00;	

		short ax=IMU_test+1;
		short ay=IMU_test+2;
		short az=IMU_test+3;
		short gx=IMU_test+4;
		short gy=IMU_test+5;
		short gz=IMU_test+6;
		short mx=IMU_test+7;
		short my=IMU_test+8;
		short mz=IMU_test+9;
		short pitch = IMU_test + 10;
		short roll=IMU_test+11;
		short yaw=IMU_test+12;

		pitch = pitch*100;
		roll=roll*100;
		yaw=yaw*100;
			
		senddat[i++]=0xAE;
		senddat[i++]=0xEA;
		senddat[i++]=0x01;//数据长度在后面赋值
		senddat[i++]=0x10;
	
	  //上传数据帧计数
		senddat[i++]=(IMU_upload_counter>>24);
		senddat[i++]=(IMU_upload_counter>>16);
		senddat[i++]=(IMU_upload_counter>>8);
		senddat[i++]=(IMU_upload_counter);
			
		senddat[i++] = ax>>8; //int16
		senddat[i++] = ax;
		senddat[i++] = ay>>8; 
		senddat[i++] = ay;
		senddat[i++] = az>>8; 
		senddat[i++] = az;
		senddat[i++] = gx>>8; 
		senddat[i++] = gx;
		senddat[i++] = gy>>8; 
		senddat[i++] = gy;
		senddat[i++] = gz>>8; 
		senddat[i++] = gz;
		senddat[i++] = mx>>8; 
		senddat[i++] = mx;
		senddat[i++] = my>>8; 
		senddat[i++] = my;
		senddat[i++] = mz>>8; 
		senddat[i++] = mz;
		senddat[i++] = pitch>>8; 
		senddat[i++] = pitch;
		senddat[i++] = roll>>8; 
		senddat[i++] = roll;
		senddat[i++] = yaw>>8; 
		senddat[i++] = yaw;

		senddat[2]=i-1; //数据长度
		for(j=2;j<i;j++)
			sum+=senddat[j];
    senddat[i++]=sum;
		
		senddat[i++]=0xEF;
		senddat[i++]=0xFE;
		 
		//UART_send_string(USART2,senddat);
		UART_send_buffer(USART2,senddat,i);
		IMU_upload_counter++;
}

// 测试 ros接收超声波数据
void S_Test_Upload_Message(void)
{
	  uint16_t S_test=1;
	  static  uint32_t S_upload_counter=0;
		unsigned char senddat[70];
		unsigned char i=0,j=0;	
		unsigned int sum=0x00;	
	  unsigned char z;
	
	   u16 S_data[12]={0};
		for(z=0;z<12;z++){
			S_data[z]=S_test+2;
		}
			
		senddat[i++]=0xAE;
		senddat[i++]=0xEA;
		senddat[i++]=0x01;//数据长度在后面赋值
		senddat[i++]=0x11;
	
	  //上传数据帧计数
		senddat[i++]=(S_upload_counter>>24);
		senddat[i++]=(S_upload_counter>>16);
		senddat[i++]=(S_upload_counter>>8);
		senddat[i++]=(S_upload_counter);
	
		for(j=0;j<12;j++){
			senddat[i++] = S_data[j]>>8; //int16
			senddat[i++] = S_data[j];
		}
		
		
		senddat[2]=i-1; //数据长度
		for(j=2;j<i;j++)
			sum+=senddat[j];
      senddat[i++]=sum;
		
		senddat[i++]=0xEF;
		senddat[i++]=0xFE;
		 
		//UART_send_string(USART2,senddat);
		UART_send_buffer(USART2,senddat,i);
		S_upload_counter++;
}



// --------------------------------------------------------------//
/***************************END OF FILE**********************/
