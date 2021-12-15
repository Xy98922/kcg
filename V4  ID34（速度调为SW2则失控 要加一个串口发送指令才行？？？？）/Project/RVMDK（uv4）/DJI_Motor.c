#include "stm32f10x.h"
#include "math.h"
#include "stm32f10x_it.h"
#include "bsp_uart.h"
#include "bsp_can.h"
#include "DJI_Motor.h"  
#include "stm32f10x_pwm.h" 
#include "ln3x.h"
#include "yn800.h"
#include "DBUS.h"
#include "td_2d.h"
// C620电调控制范围-16384~0~16384 对应电调输出 -20A-0-20A 发送频率1KHz
// C620电调反馈 机械转子角度 0-360°  对应0-8191  转子速度为RPM单位  温度单位为℃
// 3508电机最大空载转速482  最大额定转速469 额定电流10A 最大连续转矩3 N*M
// 由于M3508电机配备了1:19的减速器 因此转子的最大转速为 482*19(RPM)

//大车四轮车使用的是10英寸的轮子   25.4cm
// 麦克纳姆轮的直径为 152.5mm



#define ABS(x)		((x>0)? (x): (-x)) 
#define MAX_WHEEL_SPEED 1144800   //电机最大转速单位 RPM   482*19=9158
#define N 5   //滑窗深度5
#define LED2_FLIP  GPIO_Flip_level(GPIOE,GPIO_Pin_6) 
//编码器信息
int16_t encoderData[2];
int16_t last_encoderData[2];
int16_t pre_last_encoderData[2];
int16_t pre1_last_encoderData[2];
int16_t pre2_last_encoderData[2];
int16_t encoder_history[2][5];

//int set_v,set_spd[4]; 
int set_v,set_spd[4]; 
float expected_anguSpeed;
int16_t desired_anguSpeed;
int16_t real_anguSpeed;
float expected_yawangle;
float actual_anguSpeed;
float actual_yawangle;
uint32_t motor_upload_counter; //数据上传计数器
float  test,test1;
unsigned char DATA[10],DATAV2[10],DATAP1[18],DATAP2[18],DATA0[8];    //v1,v2,p1,p2,零点
float p1,vmax,p2,vmax2;
float setv1,setv2;
//moto_measure_t moto_chassis[4] = {0};//4 chassis moto
moto_measure_t moto_chassis[2] = {0};//4 chassis moto
moto_measure_t moto_info;

//pid_t pid_spd[4]; //四个电机对应的PID结构体
pid_t pid_spd[2]; //四个电机对应的PID结构体
pid_t gzpid;
pid_t yawpid;

int16_t get_inte[2];
int16_t get_spd[2];

command_t recived_cmd; 

extern imudata imudata1;
extern volatile uint8_t MS_Flag; 
extern rc_info_t dbus_rc;

/**
* @funtion		小车的双轮模型
* @Brief        根据设定的X方向的速度和航向角速度W 
*  							计算出两个轮子的速度
* @Param		speed_x 小车X方向的线速度(m/s)  speed_w小车旋转角速度(rad/s)
* @Retval		None
* @Date     2020/8/16
* @maker    crp
*/
void DiffX2_Wheel_Speed_Model(float speed_x,float speed_w)
{

	float v1=0,v2=0;
		float L=0.366;//左右轮子的间距
	float R=0.183;//左右轮子的间距的一半
//	float L=0.4;//左右轮子的间距
//	float R=0.2;//左右轮子的间距的一半
	int16_t a;
	float real_anguSpeed;
	float Kp=3.2;
	
    if((speed_x<-5) || (speed_x>5)) 
		speed_x=0;
    if((speed_w<-5) || (speed_w>5)) 
		speed_w=0;
 

		
	v1 = speed_x+speed_w*R;
	v2 = -(speed_x-speed_w*R);
			

	// 我们使用的是10英寸的轮子 直径为25.4cm 半径为12.7cm
  //60/(2*PI*R) = 75.191313 
	//	19是减速比
//	v1 =75.191313*v1*19;// from (m/s) to (RPM)
//	v2 =75.191313*v2*19;
//	
	//	6是减速比
	v1 =106.103297*6*6*100*v1;// from (M/s) to (°/s)
	v2 =106.103297*6*6*100*v2;

	DiffX2_Wheel_Rpm_Model(v1,v2);

}

/**
* @funtion	 设置每个轮子PID控制器的目标值
* @Brief     输入参数单位（RPM）   
* @Param		
* @Retval		None
* @Date     2020/8/16
* @maker    crp
*/

void DiffX2_Wheel_Rpm_Model(float v1,float v2)
{

	if(v1>MAX_WHEEL_SPEED) 		v1=MAX_WHEEL_SPEED;
	else if(v1< -MAX_WHEEL_SPEED)		v1=-MAX_WHEEL_SPEED;
	else ;
	
	if((v1>-4000) && (v1<4000))  v1=0;
	
	if(v2>MAX_WHEEL_SPEED) 		v2=MAX_WHEEL_SPEED;
	else if(v2< -MAX_WHEEL_SPEED)		v2=-MAX_WHEEL_SPEED;
	else ;
	
	if((v2>-4000) && (v2<4000))  v2=0;
	
	setv1 = v1;
	setv2 = v2;	
	
	if(MS_Flag==0){
		 send_loradata();

	}
			

}

/**
* @funtion			计算PID输出，并通过CAN总线发送出去
* @Brief        供定时中断函数调用,每调用一次函数，则计算一次PID输出
*								并将计算结果通过函数 CAN_DJI_C620_DataSend(iq1,iq2,iq3,iq4) 下发到电机
* @Param		
* @Retval		None
* @Date     2020/8/16
* @maker    crp
*/
unsigned char icount=0;
void DJI_Motor_Control(void)
{
	
	
	// 电机速度控制
			//V1发布     速度环   ID1
	if(icount==0)
	{  DATA[0]=0x3E;
		 DATA[1]=0xA2;
		 DATA[2]=0x01;
		 DATA[3]=0x04;
		 DATA[4]=0xE5;
     DATA[8] = ((int)setv1 >> 24) & 0xff;	 
     DATA[7] = ((int)setv1 >> 16) & 0xff; 	   
     DATA[6] = ((int)setv1>> 8)  & 0xff;
     DATA[5] = (int)setv1&0xff;
		//test=(DATA[8]<<24)|(DATA[7]<<16)|(DATA[6]<<8)|DATA[5];
	  //UART_send_string(UART4,"v1:");UART_send_intdata(UART4,test);UART_send_char(UART4,'\t');	
		DATA[9]=DATA[5]+DATA[6]+DATA[7]+DATA[8];
		UART_send_char(UART4, DATA[0]);
		UART_send_char(UART4, DATA[1]);
		UART_send_char(UART4, DATA[2]);
		UART_send_char(UART4, DATA[3]);
		UART_send_char(UART4, DATA[4]);
		UART_send_char(UART4, DATA[5]);
		UART_send_char(UART4, DATA[6]);
		UART_send_char(UART4, DATA[7]);
		UART_send_char(UART4, DATA[8]);
		UART_send_char(UART4, DATA[9]);		
	
	}
	//V2发布    速度环     ID2
	if(icount==30)
	{
	   DATAV2[0]=0x3E;
		 DATAV2[1]=0xA2;
		 DATAV2[2]=0x02;
		 DATAV2[3]=0x04;
		 DATAV2[4]=0xE6;
     DATAV2[8] = ((int)setv2 >> 24) & 0xff;	 
     DATAV2[7] = ((int)setv2  >> 16) & 0xff; 	   
     DATAV2[6] = ((int)setv2 >> 8)  & 0xff;
     DATAV2[5] = (int)setv2 &0xff;
		
		DATAV2[9]=DATAV2[5]+DATAV2[6]+DATAV2[7]+DATAV2[8];
		//test=(DATAV2[8]<<24)|(DATAV2[7]<<16)|(DATAV2[6]<<8)|DATAV2[5];
		//UART_send_string(UART4,"v2:");UART_send_intdata(UART4,test);UART_send_char(UART4,'\t');	
		UART_send_char(UART4, DATAV2[0]);
		UART_send_char(UART4, DATAV2[1]);
		UART_send_char(UART4, DATAV2[2]);
		UART_send_char(UART4, DATAV2[3]);
		UART_send_char(UART4, DATAV2[4]);
		UART_send_char(UART4, DATAV2[5]);
		UART_send_char(UART4, DATAV2[6]);
		UART_send_char(UART4, DATAV2[7]);
		UART_send_char(UART4, DATAV2[8]);
		UART_send_char(UART4, DATAV2[9]);		
		
	}
		
		
		//P1 发布 位置环  (选定通道7)  K-power
	if(icount==40)
	{	
//   	p1=(dbus_rc.ch7-1024)*54.544*6;
//		if(dbus_rc.ch7-1024<10)
//		  {p1=1034;}
//			
			p1=(dbus_rc.ch7-1024)*1.55151;
		if(dbus_rc.ch7-1024<10)
		  {p1=1;}
			
	  DATAP1[0]=0xFF;
	  DATAP1[1]=0xFF;
		DATAP1[2]=0xFE;
		DATAP1[3]=0x05;
		DATAP1[4]=0x03;      
		DATAP1[5]=0x1E;           
		DATAP1[6]=((int)p1 >> 0) & 0xff;    
    DATAP1[7]=((int)p1 >> 8) & 0xff;	   
    DATAP1[8]=~(DATAP1[2]+DATAP1[3]+DATAP1[4]+DATAP1[5]+DATAP1[6]+DATAP1[7]);
    UART_send_char(UART4, DATAP1[0]);
		UART_send_char(UART4, DATAP1[1]);
		UART_send_char(UART4, DATAP1[2]);
		UART_send_char(UART4, DATAP1[3]);
		UART_send_char(UART4, DATAP1[4]);
		UART_send_char(UART4, DATAP1[5]);
		UART_send_char(UART4, DATAP1[6]);
		UART_send_char(UART4, DATAP1[7]);
		UART_send_char(UART4, DATAP1[8]);

		UART_send_string(USART2,"P1   :  ");UART_send_intdata(USART2,p1);UART_send_string(USART2,"\r\n");
			
			
/*			
    光琉机电的位置环			
			
			
//		vmax=38000;
//		DATAP1[0]=0x3E;
//	  DATAP1[1]=0xA4;
//		DATAP1[2]=0x03;
//		DATAP1[3]=0x0C;
//		DATAP1[4]=0xF1;      //0~3字节校验和
//		DATAP1[5]=((int)p1 >> 0) & 0xff;           //位置控制低字节                   
//		DATAP1[6]=((int)p1 >> 8) & 0xff;    
//    DATAP1[7]=((int)p1 >> 16) & 0xff;	   
//    DATAP1[8]=((int)p1 >> 24) & 0xff;
////    DATAP1[5]=0x00;           //位置控制低字节                   
////		DATAP1[6]=0x00;    
////    DATAP1[7]=0x00;	   
////    DATAP1[8]=0x00;
//		DATAP1[9]=0x00;
//		DATAP1[10]=0x00;
//		DATAP1[11]=0x00;
//		DATAP1[12]= 0x00;//位置控制高字节
//		DATAP1[13]=(int)vmax & 0xff;                        //速度限制低字节
//		DATAP1[14]=((int)vmax >> 8) & 0xff; 
//		DATAP1[15]=((int)vmax >> 16) & 0xff; 
//		DATAP1[16]=((int)vmax >> 24) & 0xff;	    //速度限制高字节
//   // test=(DATAP1[16]<<24)|(DATAP1[15]<<16)|(DATAP1[14]<<8)|DATAP1[13];		
//		DATAP1[17]=DATAP1[5]+DATAP1[6]+DATAP1[7]+DATAP1[8]+DATAP1[9]+DATAP1[10]+DATAP1[11]+DATAP1[12]+DATAP1[13]+DATAP1[14]+DATAP1[15]+DATAP1[16];
//	  UART_send_char(UART4, DATAP1[0]);
//		UART_send_char(UART4, DATAP1[1]);
//		UART_send_char(UART4, DATAP1[2]);
//		UART_send_char(UART4, DATAP1[3]);
//		UART_send_char(UART4, DATAP1[4]);
//		UART_send_char(UART4, DATAP1[5]);
//		UART_send_char(UART4, DATAP1[6]);
//		UART_send_char(UART4, DATAP1[7]);
//		UART_send_char(UART4, DATAP1[8]);
//		UART_send_char(UART4, DATAP1[9]);
//		UART_send_char(UART4, DATAP1[10]);
//		UART_send_char(UART4, DATAP1[11]);
//		UART_send_char(UART4, DATAP1[12]);
//		UART_send_char(UART4, DATAP1[13]);
//		UART_send_char(UART4, DATAP1[14]);
//		UART_send_char(UART4, DATAP1[15]);
//		UART_send_char(UART4, DATAP1[16]);
//		UART_send_char(UART4, DATAP1[17]);		
	
*/	
	}


	
//	//P2 发布 位置环  (选定通道8)  ROBS-802	
//if(icount==30)
//	{
//   	p2=(dbus_rc.ch8-1024)*6.205;
//		if(dbus_rc.ch8-1024<10)
//		  { p2=1;}
//		DATAP2[0]=0xFF;
//	  DATAP2[1]=0xFF;
//		DATAP2[2]=0xFE;
//		DATAP2[3]=0x05;
//		DATAP2[4]=0x03;      
//		DATAP2[5]=0X2A;                             
//		DATAP2[6]=((int)p2 >> 0) & 0xff;    
//    DATAP2[7]=((int)p2 >> 8) & 0xff;	   
//		DATAP2[8]=~(DATAP2[2]+DATAP2[3]+DATAP2[4]+DATAP2[5]+DATAP2[6]+DATAP2[7]);
//	  UART_send_char(UART4, DATAP2[0]);
//		UART_send_char(UART4, DATAP2[1]);
//		UART_send_char(UART4, DATAP2[2]);
//		UART_send_char(UART4, DATAP2[3]);
//		UART_send_char(UART4, DATAP2[4]);
//		UART_send_char(UART4, DATAP2[5]);
//		UART_send_char(UART4, DATAP2[6]);
//		UART_send_char(UART4, DATAP2[7]);
//		UART_send_char(UART4, DATAP2[8]);
//	}
//	
	icount++;
	if(icount==40)
	{icount=0;}
   }

	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
/**
* @funtion			大疆C620 电调CAN总线数据发送函数
* @Brief        
* @Param		
* @Retval		None
* @Date     2020/8/16
* @maker    crp
*/
void CAN_DJI_C620_DataSend( int16_t iq1, int16_t iq2)
{
    CanTxMsg TxMessage;
	TxMessage.StdId = 0x200;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 0x08;
	TxMessage.Data[0] = iq1 >> 8;
	TxMessage.Data[1] = iq1;
	TxMessage.Data[2] = iq2 >> 8;
	TxMessage.Data[3] = iq2;
	CAN_Transmit(CAN1, &TxMessage); // 发送消息  
}


///*
//*@bref 电机上电角度=0， 之后用这个函数更新3510电机的相对开机后（为0）的相对角度。
//*/
//void get_total_angle(moto_measure_t *p)
//{
//	int res1, res2, delta;
//	if(p->angle < p->last_angle)
//	{			//可能的情况
//		res1 = p->angle + 8192 - p->last_angle;	//正转，delta=+
//		res2 = p->angle - p->last_angle;				//反转	delta=-
//	}
//	else
//	{	//angle > last
//		res1 = p->angle - 8192 - p->last_angle ;//反转	delta -
//		res2 = p->angle - p->last_angle;				//正转	delta +
//	}
//	//不管正反转，肯定是转的角度小的那个是真的
//	if(ABS(res1)<ABS(res2))
//		delta = res1;
//	else
//		delta = res2;
// 
//	p->total_angle += delta;
//	p->last_angle = p->angle;
//}
//电机上报数据频率为1KHZ
void CAN_RxCpltCallback(CanRxMsg* RxMessage)
{
	static uint8_t i,j;
	//ignore can1 or can2.
	int16_t tempa,tempb,tempc,tempd,tempe,tempx,max,min;
	switch(RxMessage->StdId)
		{
		case CAN_3510Moto1_ID:
		case CAN_3510Moto2_ID:
//		case CAN_3510Moto3_ID:
//		case CAN_3510Moto4_ID:
			{
				i = RxMessage->StdId - CAN_3510Moto1_ID;
				moto_chassis[i].msg_cnt++ <= 50	?	get_moto_offset(&moto_chassis[i], RxMessage) : get_moto_measure(&moto_chassis[i], RxMessage);
				get_moto_measure(&moto_info,RxMessage);
			}
			break;
	   }
		
		for(j=0;j<2;j++)
		 {
     encoderData[j]=moto_chassis[j].speed_rpm;
     tempa=pre_last_encoderData[j];
     tempb=last_encoderData[j];
     tempc=encoderData[j];
     max=tempa>tempb?tempa:tempb;
     max=max>tempc?max:tempc;
     min=tempa<tempb?tempa:tempb;
     min=min<tempc?min:tempc;
     if(tempa>min && tempa<max) encoderData[j]=tempa;
     if(tempb>min && tempb<max) encoderData[j]=tempb;
     if(tempc>min && tempc<max) encoderData[j]=tempc;
     
     pre_last_encoderData[j]=last_encoderData[j];
     last_encoderData[j]=encoderData[j];
    }
}

/*******************************************************************************************
  * @Func			void get_moto_measure(moto_measure_t *ptr, CAN_HandleTypeDef* hcan)
  * @Brief    分析,3510电机通过CAN发过来的信息
  * @Param		
  * @Retval		None
  * @Date     2015/11/24
 *******************************************************************************************/
void get_moto_measure(moto_measure_t *ptr,CanRxMsg* RxMessage)
{
//	u32  sum=0;
//	u8	 i = FILTER_BUF_LEN;	
	/*BUG!!! dont use this para code*/
//	ptr->angle_buf[ptr->buf_idx] = (uint16_t)(hcan->pRxMsg->Data[0]<<8 | hcan->pRxMsg->Data[1]) ;
//	ptr->buf_idx = ptr->buf_idx++ > FILTER_BUF_LEN ? 0 : ptr->buf_idx;
//	while(i){
//		sum += ptr->angle_buf[--i];
//	}
//	ptr->fited_angle = sum / FILTER_BUF_LEN;
	
	
	int delta=0;

	ptr->last_angle = ptr->angle;
	ptr->angle = (uint16_t)(RxMessage->Data[0]<<8 | RxMessage->Data[1]) ; //电机转角
	
	ptr->real_current  = (int16_t)(RxMessage->Data[2]<<8 | RxMessage->Data[3]);
	ptr->speed_rpm = ptr->real_current;	//这里是因为两种电调对应位不一样的信息
	
	ptr->given_current = (int16_t)(RxMessage->Data[4]<<8 | RxMessage->Data[5])/-5;
	ptr->Temp = RxMessage->Data[6];
	
	if(ptr->speed_rpm > 10 ) //电机正转
	{
		if((ptr->angle - ptr->last_angle) >= 50)
		{
		  delta = ptr->angle - ptr->last_angle;
		}
		else if ((ptr->angle - ptr->last_angle) <= -50)
		{
		  delta = ptr->angle + 8192 - ptr->last_angle;
			ptr->round_cnt++;
		}
		else;
	}
  else if(ptr->speed_rpm < -10) //电机反转
	{
		if ((ptr->angle - ptr->last_angle) <= -50)
		{
		  delta =ptr->angle - ptr->last_angle;
		}
		else if((ptr->angle - ptr->last_angle) >= 50)
		{
		  delta = ptr->angle - 8192 - ptr->last_angle;
			ptr->round_cnt--;
		}
		else;
	}
	else
		delta=0;
	//ptr->total_angle += delta;
 
	ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;
}

 
// 获取电机静止时候的值
void get_moto_offset(moto_measure_t *ptr,CanRxMsg* RxMessage)
{
	ptr->angle = (uint16_t)(RxMessage->Data[0]<<8 | RxMessage->Data[1]) ;
	ptr->offset_angle = ptr->angle;
}



 /*
 * 函数名： USB_LP_CAN1_RX0_IRQHandler
 * 描述 ： USB 中断和 CAN 接收中断服务程序， USB 跟 CAN 公用 I/O，这里只用到 CAN 的中断。
 * 输入 ：无
 * 输出 : 无
 * 调用 ：无
 
 void USB_LP_CAN1_RX0_IRQHandler(void)
 {
 CanRxMsg RxMessage;

 RxMessage.StdId=0x00;
 RxMessage.ExtId=0x00;
 RxMessage.IDE=0;
 RxMessage.DLC=0;
 RxMessage.FMI=0;
 RxMessage.Data[0]=0x00;
 RxMessage.Data[1]=0x00;

 CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);

 if((RxMessage.ExtId==0x1234) && (RxMessage.IDE==CAN_ID_EXT)
 && (RxMessage.DLC==2) && ((RxMessage.Data[1]|RxMessage.Data[0]<<8)==0xDECA))
 {
 ret = 1;
 }
else
{
ret = 0;
 }
}
*/


//------------------------------串口接收指令协议-------------------------------//
//将串口2的数据转存到数组 UART2_ReBuff中

volatile uint8_t UART2_ReBuff[100];  
volatile uint16_t UART2_ReCont;  
volatile unsigned char UART2_Reflag; 

void DJI_Motor_WriteData_In_Buff(uint8_t *DataBuff,uint16_t datalength)
{
	uint16_t i =0;
	uint16_t rpm_offset =10000; //转速偏移一万转
	uint16_t speed_offset =10; //速度偏移10m/s
	
	for(i=0;i<datalength;i++)
	{
			UART2_ReBuff[i] = *DataBuff;
			DataBuff++;
	}
 	 
	UART2_ReCont = datalength;
	UART2_Reflag =0x01;
	
 recived_cmd.cmd=UART2_ReBuff[3];
	
 if(recived_cmd.cmd ==0xF1)
 {
		recived_cmd.tag_rpm1 = UART2_ReBuff[4]*256+UART2_ReBuff[5] - rpm_offset;
		recived_cmd.tag_rpm2 = UART2_ReBuff[6]*256+UART2_ReBuff[7] - rpm_offset;
//		recived_cmd.tag_rpm3 = UART2_ReBuff[8]*256+UART2_ReBuff[9] - rpm_offset;
//		recived_cmd.tag_rpm4 = UART2_ReBuff[10]*256+UART2_ReBuff[11] - rpm_offset;
	    recived_cmd.rpm_available=0x01;
	    recived_cmd.speed_available=0x00;
 }
 else if(recived_cmd.cmd ==0xF2)
 {
		recived_cmd.tag_speed_x = (UART2_ReBuff[4]*256+UART2_ReBuff[5])/100.0-speed_offset;
		recived_cmd.tag_speed_w = (UART2_ReBuff[6]*256+UART2_ReBuff[7])/100.0-speed_offset;
//		recived_cmd.tag_speed_z = (UART2_ReBuff[8]*256+UART2_ReBuff[9])/100.0-speed_offset;
	 
	    recived_cmd.rpm_available=0x00;
	    recived_cmd.speed_available=0x01;
 }
 else
 {
	    //recived_cmd.rpm_available=0x00;
	    //recived_cmd.speed_available=0x00;
	 ;
 }
 recived_cmd.flag=0x01;

 
	#if DEBUUG_Motor_RECIVED
		UART_send_string(USART2,"\n  Uart2 recived  data length:  ");
		UART_send_data(USART2,UART2_ReCont);
	  UART_send_string(USART2,"  Byte");
	#endif	
}
/**
*@funtion   串口打印调试信息
*@Brief 以文本的方式打印电机的速度信息 调试使用
*@data 20200816  
*/
void DJI_Motor_Show_Message(void)
{
	unsigned char i=0;
	if(recived_cmd.flag)
	{
		if(recived_cmd.cmd == 0xF1)
		{
				UART_send_string(USART2,"DJI_Motor:"); 
				UART_send_string(USART2,"tag_rpm1:");UART_send_data(USART2,recived_cmd.tag_rpm1);UART_send_char(USART2,'\t');	
				UART_send_string(USART2,"tag_rpm2:");UART_send_data(USART2,recived_cmd.tag_rpm2);UART_send_char(USART2,'\t');	
//				UART_send_string(USART2,"tag_rpm3:");UART_send_data(USART2,recived_cmd.tag_rpm3);UART_send_char(USART2,'\t');	
//				UART_send_string(USART2,"tag_rpm4:");UART_send_data(USART2,recived_cmd.tag_rpm4);UART_send_char(USART2,'\n');	
		}
   	else if(recived_cmd.cmd == 0xF2)
		{
				UART_send_string(USART2,"DJI_Motor:"); 
				UART_send_string(USART2,"tag_speed_x:");UART_send_floatdat(USART2,recived_cmd.tag_speed_x);UART_send_char(USART2,'\t');	
				UART_send_string(USART2,"tag_speed_w:");UART_send_floatdat(USART2,recived_cmd.tag_speed_w);UART_send_char(USART2,'\t');	
//				UART_send_string(USART2,"tag_speed_z:");UART_send_floatdat(USART2,recived_cmd.tag_speed_z);UART_send_char(USART2,'\n');	
		}
    else;
	
	}
	for(i=0;i<2;i++)
	{
		// 打印四个电机的转速、转角、温度等信息
		UART_send_string(USART2,"M"); UART_send_data(USART2,i);UART_send_string(USART2,"： ");
		UART_send_string(USART2,"v:");UART_send_floatdat(USART2,moto_chassis[i].speed_rpm);UART_send_char(USART2,'\t');	
		UART_send_string(USART2,"t_a:");UART_send_floatdat(USART2,moto_chassis[i].total_angle);UART_send_char(USART2,'\t');	
		UART_send_string(USART2,"n:");UART_send_floatdat(USART2,moto_chassis[i].round_cnt);UART_send_char(USART2,'\t');	
		UART_send_string(USART2,"a:");UART_send_floatdat(USART2,moto_chassis[i].angle);UART_send_char(USART2,'\n');	
	}
	UART_send_char(USART2,'\n');	
	UART_send_char(USART2,'\n');	
 
}

/**
*@funtion   电机上报信息到PC上
*@Brief 按照预定的协议格式上传四个电机的转速、位置、温度到工控机 
*@data 20200816  
*/
void DJI_Motor_Upload_Message(void)
{
		unsigned char senddat[70];
		unsigned char i=0,j=0;	
		unsigned int sum=0x00;	
	
		senddat[i++]=0xAE;
		senddat[i++]=0xEA;
		senddat[i++]=0x01;//数据长度在后面赋值
		senddat[i++]=0x01;
	
	  //上传数据帧计数
		senddat[i++]=(motor_upload_counter>>24);
		senddat[i++]=(motor_upload_counter>>16);
		senddat[i++]=(motor_upload_counter>>8);
		senddat[i++]=(motor_upload_counter);
	
		for(j=0;j<2;j++) //4*11个字节
		{
			senddat[i++] = moto_chassis[j].speed_rpm>>8; //int16
			senddat[i++] = moto_chassis[j].speed_rpm;

			senddat[i++] = moto_chassis[j].total_angle>>24;
			senddat[i++] = moto_chassis[j].total_angle>>16;	
			senddat[i++] = moto_chassis[j].total_angle>>8;		
			senddat[i++] = moto_chassis[j].total_angle;	

			senddat[i++] = moto_chassis[j].round_cnt>>24;
			senddat[i++] = moto_chassis[j].round_cnt>>16;	
			senddat[i++] = moto_chassis[j].round_cnt>>8; //int16
			senddat[i++] = moto_chassis[j].round_cnt;
			
			senddat[i++] = moto_chassis[j].angle>>8; //int16
			senddat[i++] = moto_chassis[j].angle;

			senddat[i++] = moto_chassis[j].Temp;
			senddat[i++] = moto_chassis[j].Temp;
			//senddat[i++] = set_spd[j];
			//senddat[i++] = get_spd[j];
			
		}
		imudata1.gz=0;
		imudata1.yaw=0;
		
		senddat[i++]=imudata1.gz>>8;
		senddat[i++]=imudata1.gz;
		senddat[i++]=imudata1.yaw>>8;
		senddat[i++]=imudata1.yaw;
		senddat[i++]=dbus_rc.ch10;
		senddat[2]=i-1; //数据长度
		for(j=2;j<i;j++)
			sum+=senddat[j];
        senddat[i++]=sum;
		
		senddat[i++]=0xEF;
		senddat[i++]=0xFE;
		 
		//UART_send_string(USART2,senddat);
		UART_send_buffer(USART2,senddat,i);
		motor_upload_counter++;
}


/**
*@funtion   在ROS节点启动时候清除里程计，使之从零开始计数
*@Brief 当上位机按照固定指令下发时候，启动该函数 清楚里程计的累计值
*@data 20200816 
*/
void DJI_Motor_Clear_Odom(void)
{
	unsigned char j=0;
		for(j=0;j<2;j++)  
		{
			moto_chassis[j].msg_cnt=0;  

			moto_chassis[j].angle=0;
			moto_chassis[j].last_angle=0;
			moto_chassis[j].speed_rpm=0;  
			moto_chassis[j].real_current=0;  
			moto_chassis[j].given_current=0;  
						
			moto_chassis[j].Temp=0;
			moto_chassis[j].offset_angle=0;
			moto_chassis[j].round_cnt=0;
			moto_chassis[j].total_angle=0;	

		}
		
		motor_upload_counter=0;
		UART_send_string(USART2,"OK");
}



//------------------------------------------PID相关参数---------------------------//
/*
* 功能：PID初始化 
*
*20190916  CRP
*参考：大疆官网C620控制代码
*/
void PID_struct_init( pid_t *pid, uint32_t mode,uint32_t maxout,uint32_t intergral_limit,
    float 	kp,float 	ki,float 	kd)
{
    /*init function pointer*/
   // pid->f_param_init = pid_param_init;
   // pid->f_pid_reset = pid_reset;
//	pid->f_cal_pid = pid_calc;	
//	pid->f_cal_sp_pid = pid_sp_calc;	//addition
	
    pid->IntegralLimit = intergral_limit;
    pid->MaxOutput = maxout;
    pid->pid_mode = mode;
    
    pid->p = kp;
    pid->i = ki;
    pid->d = kd;
    
}
/**
    *@bref. calculate delta PID and position PID
    *@param[in] set： target
    *@param[in] real	measure
    */
float pid_calc(pid_t* pid, float get, float set)
{
    pid->get[NOW] = get;
    pid->set[NOW] = set;
    pid->err[NOW] = set - get;	//set - measure
	
		if (pid->max_err != 0 && ABS(pid->err[NOW]) >  pid->max_err  )
			return 0;
		if (pid->deadband != 0 && ABS(pid->err[NOW]) < pid->deadband)
			return 0;
    
    if(pid->pid_mode == POSITION_PID) //位置式p
    {
        pid->pout = pid->p * pid->err[NOW];
        pid->iout += pid->i * pid->err[NOW];
        pid->dout = pid->d * (pid->err[NOW] - pid->err[LAST] );
        abs_limit(&(pid->iout), pid->IntegralLimit);
        pid->pos_out = pid->pout + pid->iout + pid->dout;
        abs_limit(&(pid->pos_out), pid->MaxOutput);
        pid->last_pos_out = pid->pos_out;	//update last time 
    }
    else if(pid->pid_mode == DELTA_PID)//增量式P
    {
        pid->pout = pid->p * (pid->err[NOW] - pid->err[LAST]);
        pid->iout = pid->i * pid->err[NOW];
        pid->dout = pid->d * (pid->err[NOW] - 2*pid->err[LAST] + pid->err[LLAST]);
        
        abs_limit(&(pid->iout), pid->IntegralLimit);
        pid->delta_u = pid->pout + pid->iout + pid->dout;
        pid->delta_out = pid->last_delta_out + pid->delta_u;
        abs_limit(&(pid->delta_out), pid->MaxOutput);
        pid->last_delta_out = pid->delta_out;	//update last time
    }
    
    pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST] = pid->err[NOW];
    pid->get[LLAST] = pid->get[LAST];
    pid->get[LAST] = pid->get[NOW];
    pid->set[LLAST] = pid->set[LAST];
    pid->set[LAST] = pid->set[NOW];
		
    return pid->pid_mode==POSITION_PID ? pid->pos_out : pid->delta_out;
}

/**
    *@bref. special calculate position PID @attention @use @gyro data!!
    *@param[in] set： target
    *@param[in] real	measure
    */
float pid_sp_calc(pid_t* pid, float get, float set, float gyro)
{
    pid->get[NOW] = get;
    pid->set[NOW] = set;
    pid->err[NOW] = set - get;	//set - measure
    
    
    if(pid->pid_mode == POSITION_PID) //位置式p
    {
        pid->pout = pid->p * pid->err[NOW];
				if(fabs(pid->i) >= 0.001f)
					pid->iout += pid->i * pid->err[NOW];
				else
					pid->iout = 0;
        pid->dout = -pid->d * gyro/100.0f;	
        abs_limit(&(pid->iout), pid->IntegralLimit);
        pid->pos_out = pid->pout + pid->iout + pid->dout;
        abs_limit(&(pid->pos_out), pid->MaxOutput);
        pid->last_pos_out = pid->pos_out;	//update last time 
    }
    else if(pid->pid_mode == DELTA_PID)//增量式P
    {
//        pid->pout = pid->p * (pid->err[NOW] - pid->err[LAST]);
//        pid->iout = pid->i * pid->err[NOW];
//        pid->dout = pid->d * (pid->err[NOW] - 2*pid->err[LAST] + pid->err[LLAST]);
//        
//        abs_limit(&(pid->iout), pid->IntegralLimit);
//        pid->delta_u = pid->pout + pid->iout + pid->dout;
//        pid->delta_out = pid->last_delta_out + pid->delta_u;
//        abs_limit(&(pid->delta_out), pid->MaxOutput);
//        pid->last_delta_out = pid->delta_out;	//update last time
    }
    
    pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST] = pid->err[NOW];
    pid->get[LLAST] = pid->get[LAST];
    pid->get[LAST] = pid->get[NOW];
    pid->set[LLAST] = pid->set[LAST];
    pid->set[LAST] = pid->set[NOW];
    return pid->pid_mode==POSITION_PID ? pid->pos_out : pid->delta_out;
//	
}

void abs_limit(float *a, float ABS_MAX)
{
    if(*a > ABS_MAX)
        *a = ABS_MAX;
    if(*a < -ABS_MAX)
        *a = -ABS_MAX;
}

//中途更改参数设定
static void pid_reset(pid_t	*pid, float kp, float ki, float kd)
{
    pid->p = kp;
    pid->i = ki;
    pid->d = kd;
}
//PWM; 180度舵机：脉冲宽度:0.5ms-2.5ms,周期20ms,预装载值设为720，则比较值处于【18-90】分别对应为【0°-180°】，则其精度为2.5°,(预装载值为7200，精度为0.25)
//     360度舵机：脉冲宽度:0.5ms-2.5ms,周期20ms,预装载值设为720，则比较值处于【18-90】分别对应为【0°-360°】，则其精度为5°;
void servoinit(int16_t angle1)
{
  PWM_config_Init(PWM2, PWM_Channel_0 , 0, 0);
	TIM_CtrlPWMOutputs(TIM4,ENABLE);
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable); 
	TIM_ARRPreloadConfig(TIM4, ENABLE);
	TIM_SetCompare1(TIM4,30);      // PB6,28
	PWM_config_Init(PWM1, PWM_Channel_0 , 0, 0);
	TIM_CtrlPWMOutputs(TIM3,ENABLE);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable); 
	TIM_ARRPreloadConfig(TIM3, ENABLE);
	TIM_SetCompare1(TIM3,41);     //  PA6

}

void Watch_PID(void) //将各阶段数据按照协议经过数传发送至上位机可视化
{
	u8 senddata[20]={0};
	u8 i=0,j=0,sum = 0;
	int16_t Encode1_x1,Encode2_x1;
	real_anguSpeed=(int16_t)(-imudata1.gz*2000*3.14159*1000/32768/180);
	desired_anguSpeed=expected_anguSpeed*1000;
	
	//Encode1_x1=(int16_t)Encode[0].x1;
	//Encode2_x1=(int16_t)Encode[1].x1;
	
	senddata[i++]=0xAA;   //0xAA为帧头
  senddata[i++]=0x05;   //0x55为数据发送源，具体请参考匿名协议，本字节用户可以随意编写
	senddata[i++]=0xAF;   //0xAF为数据目的地，AF表示上位机，具体请参考匿名协议
	senddata[i++]=0xF1;   //0xF1表示本帧为F1，用户自定义帧，对应高级收码F1功能帧
	senddata[i++]=0;      //本字节表示数据长度，这里先等于0，函数最后再赋值
	senddata[i++]=set_spd[0]>>8;  //int_16拆分，期望速度，跟踪微分滤波后的
	senddata[i++]=set_spd[0];
//	senddata[i++]=real_anguSpeed>>8;
//	senddata[i++]=real_anguSpeed;
//	senddata[i++]=desired_anguSpeed>>8;
//	senddata[i++]=desired_anguSpeed;
//	senddata[i++]=get_spd[0]>>8;  //pid输出
//	senddata[i++]=get_spd[0];    
//	senddata[i++]=encoderData[0]>>8; //传感器输出
//	senddata[i++]=encoderData[0];
//	senddata[i++]=get_inte[0]>>8;
//	senddata[i++]=get_inte[0];
//  senddata[i++]=moto_chassis[0].speed_rpm>>8;
//	senddata[i++]=moto_chassis[0].speed_rpm;
//	senddata[i++]=moto_chassis[1].speed_rpm>>8;
//	senddata[i++]=moto_chassis[1].speed_rpm;
	senddata[i++]=set_spd[1]>>8;//int_16拆分，期望速度，跟踪微分滤波后的
	senddata[i++]=set_spd[1];
//	senddata[i++]=Encode1_x1>>8;
//	senddata[i++]=Encode2_x1;
//	senddata[i++]=Encode2_x1>>8;
//	senddata[i++]=Encode2_x1;
	
	senddata[i++]=set_spd[2]>>8;//跟踪微分滤波前的遥控器数据
	senddata[i++]=set_spd[2];
	senddata[i++]=set_spd[3]>>8;//跟踪微分滤波前的遥控器数据
	senddata[i++]=set_spd[3];
//  senddata[i++]=get_spd[1]>>8;
//  senddata[i++]=get_spd[1];
	senddata[i++]=encoderData[0]>>8;//转速反馈通道中的编码器经中值滤波后的数据
	senddata[i++]=encoderData[0];
	senddata[i++]=encoderData[1]>>8;//转速反馈通道中的编码器经中值滤波后的数据
	senddata[i++]=encoderData[1];
//	senddata[i++]=get_inte[1]>>8;
//	senddata[i++]=get_inte[1];
	senddata[4] = i-5;   //i用来计算数据长度，减5减去前面5个非数据字节
	for(j=0;j<i;j++)
		sum += senddata[j];
	senddata[i++]=sum;   //和校验
	UART_send_buffer(USART3,senddata,i);
}




