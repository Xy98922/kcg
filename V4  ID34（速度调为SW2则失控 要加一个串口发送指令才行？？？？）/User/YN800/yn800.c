#include "stm32f10x.h"
#include "yn800.h"
#include "DJI_Motor.h"
#include "bsp_uart.h"

uint8_t loraRec_flag;
uint8_t uart3Rec_flag;
uint8_t uart3_revcnt;
uint8_t uart3Buf[20];

//解析lora数据
uint8_t loradata;
uint8_t last_loradata;
uint8_t lora_revbuf[20];
uint8_t lordatCnt=0;

extern command_t recived_cmd; 
extern int set_spd[2]; 
extern int8_t MS_Flag;

//发送lora数据
void send_loradata(void)
{
		unsigned char senddat[70];
		unsigned char i=0,j=0;	
		unsigned int sum=0x00;
	  
	  senddat[i++]=0xAE;
		senddat[i++]=0xEA;
		senddat[i++]=0x01;//数据长度在后面赋值
		senddat[i++]=0xE3;
	
	  senddat[i++]=set_spd[0]>>8;
	  senddat[i++]=set_spd[0];
	
	  senddat[i++]=set_spd[1]>>8;
	  senddat[i++]=set_spd[1];
	
	  senddat[2]=i-1; //数据长度
		for(j=2;j<i;j++)
			sum+=senddat[j];
        senddat[i++]=sum;
	
	  senddat[i++]=0xEF;
		senddat[i++]=0xFE;
		 
		//UART_send_string(USART3,senddat);
		UART_send_buffer(USART3,senddat,i);
}


void recv_loradata(uint8_t ch)
{	
	 last_loradata=loradata;
	 loradata=ch;
//	UART_send_char(USART2,0x00);
//	 UART_send_char(USART2,loradata);
//	 UART_send_char(USART2,last_loradata);
//	 UART_send_char(USART2,0x11);
	
//   UART_send_char(USART2,last_loradata);	
	 if(last_loradata==0xAE && loradata==0xEA){
		  loraRec_flag=1;
		  uart3Rec_flag=1;
		  lordatCnt=0;
			lora_revbuf[lordatCnt++]=last_loradata;
		  lora_revbuf[lordatCnt++]=loradata;
//		 	UART_send_char(USART2,lordatCnt);
	 }
	 else if(lordatCnt>=2 && uart3Rec_flag==1){
		  if(lordatCnt<3){
//				 UART_send_char(USART2,lordatCnt);
				 lora_revbuf[lordatCnt++]=loradata;     //数据长度
//				 UART_send_char(USART2,lora_revbuf[2]);
			}
			else if(lordatCnt<lora_revbuf[2]+2){
				 lora_revbuf[lordatCnt++]=loradata;     //功能号（标识号）+轮速数据+和校验
//			   UART_send_char(USART2,lora_revbuf[4]);
			}
			else if(lordatCnt==lora_revbuf[2]+2 && loradata==0xEF)
				 lora_revbuf[lordatCnt++]=loradata;     //
			else if(lordatCnt==lora_revbuf[2]+3 && loradata==0xFE){
		     lora_revbuf[lordatCnt++]=loradata;						
				 loraRec_flag=1;
				 uart3_revcnt=lordatCnt;
				 deal_loradata(lora_revbuf,uart3_revcnt);
				 uart3Rec_flag=0; 
			}
			else{
				 lordatCnt=0;
				 uart3Rec_flag=0;
			}
		}
	 else{
			   lordatCnt=0;
				 uart3Rec_flag=0;
	 }	
//   UART_send_buffer(USART2, lora_revbuf,lordatCnt);		
}

void deal_loradata(uint8_t *buf,uint8_t uart3_len)
{
	 uint8_t i;
	
	 for(i=0;i<uart3_len;i++){
			uart3Buf[i]=*buf;
		  buf++;
//      UART_send_char(USART2, uart3Buf[4]);		
//			UART_send_char(USART2, uart3Buf[5]);		
	 }
	 
	 uart3_revcnt=uart3_len;
	 loraRec_flag=1;
	 
	 recived_cmd.loraRev_flag=uart3Buf[3];
//	 UART_send_char(USART2, uart3Buf[4]);	
//	 UART_send_char(USART2, uart3Buf[5]);
	 MS_Flag=1;
	 if(recived_cmd.loraRev_flag ==0xE3 && MS_Flag==0x01)
   {
		  recived_cmd.rev_rpm1 = (int16_t)(uart3Buf[4]<<8|uart3Buf[5]);
		  recived_cmd.rev_rpm2 = (int16_t)(uart3Buf[6]<<8|uart3Buf[7]);
//		recived_cmd.tag_rpm3 = UART2_ReBuff[8]*256+UART2_ReBuff[9] - rpm_offset;
//		recived_cmd.tag_rpm4 = UART2_ReBuff[10]*256+UART2_ReBuff[11] - rpm_offset;
	    recived_cmd.rpm_available=0x01;
		  
		  UART_send_string(USART2,"recived_cmd.rev_rpm1:");
      UART_send_intdata(USART2,recived_cmd.rev_rpm1);
		  UART_send_char(USART2,'\t');	
      UART_send_string(USART2,"recived_cmd.rev_rpm2:");
      UART_send_intdata(USART2,recived_cmd.rev_rpm2);
		  UART_send_char(USART2,'\t');	
      
		 
//		  UART_send_data(USART2, recived_cmd.rev_rpm2);
//测试：AE EA 07 E3 01 9D FE 5C E2 EF FE		 
   }
}