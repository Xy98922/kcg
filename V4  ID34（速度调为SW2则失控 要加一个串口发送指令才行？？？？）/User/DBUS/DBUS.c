 
#include "stm32f10x.h"
#include "DBUS.h" 
#include "string.h"
#include "stdlib.h"
#include "bsp_uart.h"


unsigned char DBUS_flag=0;
uint32_t ch1_offset_sum=0,ch2_offset_sum=0,ch3_offset_sum=0,ch4_offset_sum=0,ch7_offset_sum=0,ch8_offset_sum=0;
uint32_t rc_counter=0;
rc_info_t dbus_rc;
int16_t sbus_ch[25];

/**
  * @brief       DBUS串口接收回调函数 
  * @param[out]  rc:   转换为每个通道的数据
  * @param[in]   pData: 输入长度为18字节的数据
  * @retval 
  * @maker    crp
	* @data 2019-9-8
  */

void Sbus_Data_Count(uint8_t *buf)
{
	
	if(buf == NULL)
	{

		return;
	}
	if(buf[0]!=25)
	{return;}
	if(buf[1]!=15)
		{
			//UART_send_string(USART2,"buf[0]=:  ");
			//UART_send_intdata(USART2, buf[1]);UART_send_string(USART2,"\r\n");
			return;
	  }

	dbus_rc.ch1 = ((int16_t)buf[ 2] >> 0 | ((int16_t)buf[ 3] << 8 )) & 0x07FF;
	dbus_rc.ch2 = ((int16_t)buf[ 3] >> 3 | ((int16_t)buf[ 4] << 5 )) & 0x07FF;
	dbus_rc.ch3 = ((int16_t)buf[ 4] >> 6 | ((int16_t)buf[ 5] << 2 )  | (int16_t)buf[ 6] << 10 ) & 0x07FF;
	dbus_rc.ch4 = ((int16_t)buf[ 6] >> 1 | ((int16_t)buf[ 7] << 7 )) & 0x07FF;
	dbus_rc.sw2 = ((int16_t)buf[ 7] >> 4 | ((int16_t)buf[ 8] << 4 )) & 0x07FF;
	dbus_rc.sw1 = ((int16_t)buf[ 8] >> 7 | ((int16_t)buf[ 9] << 1 )  | (int16_t)buf[10] <<  9 ) & 0x07FF;
	dbus_rc.ch7 = ((int16_t)buf[10] >> 2 | ((int16_t)buf[11] << 6 )) & 0x07FF;
	dbus_rc.ch8 = ((int16_t)buf[11] >> 5 | ((int16_t)buf[12] << 3 )) & 0x07FF;	// ch7/ch8其一为履带变形信号通道，单机有且仅用一个
	dbus_rc.ch9 = ((int16_t)buf[13] << 0 | ((int16_t)buf[14] << 8 )) & 0x07FF;  //模式切换
	dbus_rc.ch10= ((int16_t)buf[14] >> 3 | ((int16_t)buf[15] << 5 )) & 0x07FF;  //前车落锁通道（被占用）
  dbus_rc.ch11=((int16_t)buf[15] >> 6 | ((int16_t)buf[16] << 2 )  | (int16_t)buf[17] << 10 ) & 0x07FF;//选此通道为俯仰舵机信号通道
 	dbus_rc.ch12=((int16_t)buf[17] >> 1 | ((int16_t)buf[18] << 7 )) & 0x07FF;    //模式切换，单独控制此车是否接收遥控器信号
	dbus_rc.ch13=((int16_t)buf[18] >> 4 | ((int16_t)buf[19] << 4 )) & 0x07FF;     //履带变形二（新机）
	dbus_rc.ch14=((int16_t)buf[19] >> 7 | ((int16_t)buf[20] << 1 )  | (int16_t)buf[21] <<  9 ) & 0x07FF;  

    //UART_send_string(USART2,"Transform Successful ....\n");
//	  UART_send_string(USART2,"dbus_rc.ch2:  ");UART_send_intdata(USART2,dbus_rc.ch2);UART_send_string(USART2,"\r\n");	
		//UART_send_string(USART2,"dbus_rc.ch3:  ");UART_send_intdata(USART2,dbus_rc.ch3);UART_send_char(USART2,'\t');	
	if(	dbus_rc.sw2<1200&&dbus_rc.sw2>1000)
			dbus_rc.sw2=3;
	else if (	dbus_rc.sw2>1200&&	dbus_rc.sw2<1800)
			dbus_rc.sw2=1;
	else if(	dbus_rc.sw2<1000)
			dbus_rc.sw2=2;
	
	
		if(	dbus_rc.sw1<1200&&	dbus_rc.sw1>1000)
			dbus_rc.sw1=3;
	else if (	dbus_rc.sw1>1200&&	dbus_rc.sw1<1800)
			dbus_rc.sw1=1;
	else if(	dbus_rc.sw1<1000)
			dbus_rc.sw1=2;
	 //UART_send_string(USART2,"dbus_rc.sw1:  ");UART_send_intdata(USART2,dbus_rc.sw1);UART_send_string(USART2,"\r\n");	

			if(	dbus_rc.ch7<1200&&dbus_rc.ch7>1000)
			dbus_rc.ch7=18;
	else if (	dbus_rc.ch7>1200&&	dbus_rc.ch7<1800)
			dbus_rc.ch7=54;
	else if(	dbus_rc.ch7<1000)
			dbus_rc.ch7=90;
	
		if(	dbus_rc.ch8<1200&&dbus_rc.ch8>1000)
			dbus_rc.ch8=18;
	else if (	dbus_rc.ch8>1200&&	dbus_rc.ch8<1800)
			dbus_rc.ch8=54;
	else if(	dbus_rc.ch8<1000)
			dbus_rc.ch8=90;
	
	if(	dbus_rc.ch9<1200&&dbus_rc.ch9>1000)
			dbus_rc.ch9=3;
	else if (	dbus_rc.ch9>1200&&	dbus_rc.ch9<1800)
			dbus_rc.ch9=1;
	else if(	dbus_rc.ch9<1000)
			dbus_rc.ch9=2;
	
		if(	dbus_rc.ch10<1200&&dbus_rc.ch10>1000)
			dbus_rc.ch10=66;
	else if (	dbus_rc.ch10>1200&&	dbus_rc.ch10<1800)
			dbus_rc.ch10=66;
	else if(	dbus_rc.ch10<1000)
			dbus_rc.ch10=2;
	
	if(	dbus_rc.ch12<1200&&dbus_rc.ch12>1000)
			dbus_rc.ch12=3;
	else if (	dbus_rc.ch12>1200&&	dbus_rc.ch12<1800)
			dbus_rc.ch12=1;
	else if(	dbus_rc.ch12<1000)
			dbus_rc.ch12=2;
				if(	dbus_rc.ch13<1200&&dbus_rc.ch13>1000)
			dbus_rc.ch13=18;
	else if (	dbus_rc.ch13>1200&&	dbus_rc.ch13<1800)
			dbus_rc.ch13=54;
	else if(	dbus_rc.ch13<1000)
			dbus_rc.ch13=90;
	
 
		if(DBUS_flag == DBUS_INIT) 
	{
		dbus_rc.available =0x00;
		if(RC_Offset_Init())
			DBUS_flag = DBUS_RUN;
		else
			DBUS_flag = DBUS_INIT;
	}
	else
	{
		if((dbus_rc.ch1 > 2000) || (dbus_rc.ch1<100) 
			|| (dbus_rc.ch3 > 2000) || (dbus_rc.ch3<100)
		  || (dbus_rc.sw1 > 3) || (dbus_rc.sw1<1)
		  || (dbus_rc.sw2 > 3) || (dbus_rc.sw2<1))
		{
			dbus_rc.available =0x00;
		}
		else
		{
			dbus_rc.available =0x01;
		}
		dbus_rc.cnt =dbus_rc.v +1;	
	}
}

/**
  * @brief       遥控器初始化函数 采集10次数据求取平均值
  * @param[out]  校准完成返回1 否则返回0
  * @param[in]    
  * @retval 
  * @maker    crp
	* @data 2019-9-8
  */
char RC_Offset_Init(void)
{
	if((dbus_rc.ch1>1000) && (dbus_rc.ch1<1052))
		if((dbus_rc.ch2>1000) && (dbus_rc.ch2<1052))
			if((dbus_rc.ch3>1000) && (dbus_rc.ch3<1052))
				if((dbus_rc.ch4>1000) && (dbus_rc.ch4<1052))
					//if((dbus_rc.ch7>1000) && (dbus_rc.ch7<1052))
							//if((dbus_rc.ch8>1000) && (dbus_rc.ch8<1052))
				{
						ch1_offset_sum+=dbus_rc.ch1;
						ch2_offset_sum+=dbus_rc.ch2;
						ch3_offset_sum+=dbus_rc.ch3;
						ch4_offset_sum+=dbus_rc.ch4;
					  ch7_offset_sum+=dbus_rc.ch7;
					  ch8_offset_sum+=dbus_rc.ch8;
						rc_counter++;
				}

	if(rc_counter>10)
	{
	  ch1_offset_sum = ch1_offset_sum/rc_counter;
		ch2_offset_sum = ch2_offset_sum/rc_counter;
		ch3_offset_sum = ch3_offset_sum/rc_counter;
		ch4_offset_sum = ch4_offset_sum/rc_counter;
		ch7_offset_sum = ch7_offset_sum/rc_counter;
		ch8_offset_sum = ch8_offset_sum/rc_counter;
		dbus_rc.ch1_offset =ch1_offset_sum;
		dbus_rc.ch2_offset =ch2_offset_sum;
		dbus_rc.ch3_offset =ch3_offset_sum;
		dbus_rc.ch4_offset =ch4_offset_sum;
		dbus_rc.ch7_offset =ch7_offset_sum;
		dbus_rc.ch8_offset =ch8_offset_sum;
		//calibration failed 
		if((dbus_rc.ch1_offset ==0) || (dbus_rc.ch2_offset ==0) || (dbus_rc.ch3_offset ==0) || (dbus_rc.ch4_offset ==0)|| (dbus_rc.ch7_offset ==0)|| (dbus_rc.ch8_offset ==0))
		{
			dbus_rc.available =0x00; 
		  rc_counter=0;
			ch1_offset_sum = 0;
			ch2_offset_sum = 0;
			ch3_offset_sum = 0;
			ch4_offset_sum = 0;
			ch7_offset_sum = 0;
			return 0;
		}
		else
		{
			dbus_rc.available =0x01; 
			dbus_rc.cnt =rc_counter;
			
			return 1;
		}
	}
	
	dbus_rc.available =0x00;
	dbus_rc.cnt =rc_counter;
	
	return 0;
}
/**
  * @brief       调试遥控器，打印数据
  * @retval 
  * @maker    crp
	* @data 2019-9-8
  */
void RC_Debug_Message(void)
{
	UART_send_string(USART2,"SBUS:  ch1:");UART_send_data(USART2,dbus_rc.ch1);UART_send_char(USART2,'\t');		
	UART_send_string(USART2,"ch2:");UART_send_data(USART2,dbus_rc.ch2);UART_send_char(USART2,'\t');	
	UART_send_string(USART2,"ch3:");UART_send_data(USART2,dbus_rc.ch3);UART_send_char(USART2,'\t');	
	UART_send_string(USART2,"ch4:");UART_send_data(USART2,dbus_rc.ch4);UART_send_char(USART2,'\t');	
	UART_send_string(USART2,"sw1:");UART_send_data(USART2,dbus_rc.sw1);UART_send_char(USART2,'\t');	
	UART_send_string(USART2,"sw2:");UART_send_data(USART2,dbus_rc.sw2);UART_send_char(USART2,'\n');	
	
	UART_send_string(USART2,"ch1_offset:");UART_send_data(USART2,dbus_rc.ch1_offset);UART_send_char(USART2,'\t');		
	UART_send_string(USART2,"ch2_offset:");UART_send_data(USART2,dbus_rc.ch2_offset);UART_send_char(USART2,'\t');	
	UART_send_string(USART2,"ch3_offset:");UART_send_data(USART2,dbus_rc.ch3_offset);UART_send_char(USART2,'\t');	
	UART_send_string(USART2,"ch4_offset:");UART_send_data(USART2,dbus_rc.ch4_offset);UART_send_char(USART2,'\n');	
	
	UART_send_char(USART2,'\n');	
}
/**
  * @brief DBUS上传信息到PC上
  * @retval 
  * @maker    crp
	* @data 2019-9-8
  */
void RC_Upload_Message(void)
{
		char senddata[50];
		unsigned char i=0,j=0;	
		unsigned char cmd=0x03;	
		unsigned int sum=0x00;	
		senddata[i++]=0xAE;
		senddata[i++]=0xEA;
		senddata[i++]=0x00;
		senddata[i++]=cmd;
		senddata[i++]=dbus_rc.ch1>>8;
		senddata[i++]=dbus_rc.ch1;
		senddata[i++]=dbus_rc.ch2>>8;
		senddata[i++]=dbus_rc.ch2;
		senddata[i++]=dbus_rc.ch3>>8;
		senddata[i++]=dbus_rc.ch3;
		senddata[i++]=dbus_rc.ch4>>8;
		senddata[i++]=dbus_rc.ch4;
		senddata[i++]=dbus_rc.sw1;
		senddata[i++]=dbus_rc.sw1;
		for(j=2;j<i;j++)
			sum+=senddata[j];
		senddata[i++]=sum;
		senddata[2]=i-2; //数据长度
		senddata[i++]=0xEF;
		senddata[i++]=0xFE;
		senddata[i++]='\0';
		UART_send_string(USART2,senddata);
}
