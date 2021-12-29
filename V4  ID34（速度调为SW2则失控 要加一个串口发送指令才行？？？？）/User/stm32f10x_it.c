/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "stm32f10x_gpio.h"
#include <stdio.h>
#include "stm32f10x_Delay.h"

#include "bsp_uart.h"
#include "bsp_can.h"
#include "DBUS.h"
#include "DJI_Motor.h"
#include "ln3x.h"

extern volatile uint32_t TimingDelay;
extern volatile uint32_t Timer2_Counter1;
extern volatile uint32_t Timer2_Counter2;
extern volatile uint32_t Timer2_Counter3;
extern volatile uint32_t Timer2_Counter4;
extern volatile uint32_t Timer2_Counter5;
extern volatile uint32_t Timer2_Counter6;//dc���� ���
extern volatile uint8_t UART1_DMA_Flag; //����1�жϱ�־λ
extern volatile uint8_t UART2_Flag; //����2�жϱ�־λ
extern volatile uint8_t CAN1_Flag;  //CAN���߽����жϱ�־
extern volatile uint8_t DC_flag;  //CAN���߽����жϱ�־
extern int set_spd[2];

extern uint8_t USART_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�



extern volatile uint8_t UART2_ReBuff[100];  
extern volatile uint16_t UART2_ReCont;  
extern volatile unsigned char UART2_Reflag; 

uint16_t ReCont_2=0;  
unsigned char Reflag_2=0;  
unsigned char U2dat_value=0;
unsigned char U2dat_value_last=0;
imudata imudata1;
uint8_t USART1_RX_BUF[26];
uint8_t USART3_RX_BUF[14];
uint8_t USART2_RX_BUF[88];
uint8_t USART3Rec_flag;
int USART3Rec_speedx;
int USART3Rec_speedw;
#define DC_OVER   GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_7)//��ȡPB7 
#define LED1_FLIP  GPIO_Flip_level(GPIOE,GPIO_Pin_5) 
#define LED2_FLIP  GPIO_Flip_level(GPIOE,GPIO_Pin_6) 
//extern void Matrix_Keyscan_Analy(unsigned int interupt_times);//���󰴼���ȡ
//extern void Keyscan_Analy(unsigned int interupt_times);//����������ȡ

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}
 
 
/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}
void TIM2_IRQHandler(void)//��ʱ��2 �жϷ�����
{
	if ( TIM_GetITStatus(TIM2 , TIM_IT_Update) != RESET ) 
	{	
	  Timer2_Counter1++;
		Timer2_Counter2++;
		Timer2_Counter3++;	
		Timer2_Counter4++;
		Timer2_Counter5++;
		Timer2_Counter6++;
	  //GPIO_Flip_level(GPIOE,GPIO_Pin_1);
		//Sbus_Data_Count(USART1_RX_BUF);
		DJI_Motor_Control(); 
		
//		Watch_PID();
		TIM_ClearITPendingBit(TIM2 , TIM_FLAG_Update);   		
	}
}
void TIM3_IRQHandler(void)//��ʱ��6 �жϷ�����
{
	newPkg(4) ctlPkg1={8,0x90,0x90,0x30,0x00,{0x00,0x00,0x00,0x00}};//ע�����ʽ
	if ( TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET ) 
	{	
	   
	  //GPIO_Flip_level(GPIOE,GPIO_Pin_1);
		TIM_ClearITPendingBit(TIM3, TIM_FLAG_Update);   		
	}
}




void USART1_IRQHandler(void)
{
	uint8_t res;
	uint8_t clear = 0;
	static uint8_t Rx_Sta = 1;
	
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		res =USART1->DR;
		USART1_RX_BUF[Rx_Sta++] = res;
		
	}
	else if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)  //�����ж����ڼ�⵽���ݽ��պ󣨼����ڵ�RXNEλ����λ����ʼ��⣬����������һ���ֽڶ�Ӧ��������δ�����µ����ݽ���ʱ�����������ж�IDLEλ��Ӳ����1.
	{
		clear = USART1->SR;
		clear = USART1->DR;                     //IDEL����
		USART1_RX_BUF[0] = Rx_Sta - 1;
		Rx_Sta = 1;
	}
	  UART1_DMA_Flag =0x01;
	 //Sbus_Data_Count(USART1_RX_BUF);
    GPIO_Flip_level(GPIOE,GPIO_Pin_6) ;

}

void USART3_IRQHandler(void)
{
  uint8_t res;
	uint8_t clear = 0;
	static uint8_t Rx_Sta = 0;
	
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{
		res =USART3->DR;
		USART3_RX_BUF[Rx_Sta++] = res;
		USART_ClearITPendingBit(USART3, USART_IT_RXNE); 
		if(USART3_RX_BUF[0]!=0x3e)
		{Rx_Sta=0;}
	}
		if(Rx_Sta==14) 
			
  {
			
	if(*USART3_RX_BUF== NULL)
	{
    Rx_Sta=0;
		return;
	}

	if(USART3_RX_BUF[0]!=0X3e)
		{
//			UART_send_string(USART3,"buf[0]=:  ");
//			UART_send_data(USART3, USART3_RX_BUF[0]);UART_send_string(USART3,"\r\n");
			Rx_Sta=0;
			return;
	  }
		if(USART3_RX_BUF[1]!=0Xa2)
		{
//			UART_send_string(USART3,"buf[2]=:  ");
//			UART_send_intdata(USART3, USART3_RX_BUF[1]);UART_send_string(USART3,"\r\n");
			Rx_Sta=0;
			return;
	  }
		if(USART3_RX_BUF[2]!=0X01&&USART3_RX_BUF[2]!=0xFE)  //ID��֤
		{
//			UART_send_string(USART3,"buf[3]=:  ");
//			UART_send_intdata(USART3, USART3_RX_BUF[2]);UART_send_string(USART3,"\r\n");
			Rx_Sta=0;
			return;
	  }
		if(USART3_RX_BUF[13]!=(u8)(USART3_RX_BUF[7]+USART3_RX_BUF[6]+USART3_RX_BUF[8]+USART3_RX_BUF[5]+USART3_RX_BUF[12]+USART3_RX_BUF[11]+USART3_RX_BUF[10]+USART3_RX_BUF[9]))
		{
//			UART_send_string(USART3,"֡β����:  ");
//			UART_send_intdata(USART3, USART3_RX_BUF[13]);UART_send_string(USART3,"\r\n");
//			UART_send_char(USART3, USART3_RX_BUF[13]);
//			UART_send_char(USART3,USART3_RX_BUF[7]+USART3_RX_BUF[6]+USART3_RX_BUF[8]+USART3_RX_BUF[5]+USART3_RX_BUF[12]+USART3_RX_BUF[11]+USART3_RX_BUF[10]+USART3_RX_BUF[9]);
			Rx_Sta=0;
			return;
	  }
		USART3Rec_flag=1;
    USART3Rec_speedx=((int)(USART3_RX_BUF[8]&0xff)<<24)|((int)(USART3_RX_BUF[7]&0xff)<<16)|((int)(USART3_RX_BUF[6]&0xff)<<8)|((int)(USART3_RX_BUF[5]&0xff));
    USART3Rec_speedw=((int)(USART3_RX_BUF[12]&0xff)<<24)|((int)(USART3_RX_BUF[11]&0xff)<<16)|((int)(USART3_RX_BUF[10]&0xff)<<8)|((int)(USART3_RX_BUF[9]&0xff));
		Rx_Sta=0;
		if(USART3_RX_BUF[3]==0X05){
		TIM_SetCompare1(TIM4,90);
		TIM_SetCompare1(TIM3,90);
		}
		else if(USART3_RX_BUF[3]==0X06){
		TIM_SetCompare1(TIM4,18);
		TIM_SetCompare1(TIM3,18);
		}
		else{
		TIM_SetCompare1(TIM4,54);
		TIM_SetCompare1(TIM3,54);
		}
	
	}
}
 

// USRT2->IMUDATA ���� USART4->IMUDATA
void USART2_IRQHandler(void)
{


	static unsigned char ucRxBuffer[250];
	static unsigned char ucRxCnt = 0;	

	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
	ucRxBuffer[ucRxCnt++]=USART_ReceiveData(USART2);	
	if (ucRxBuffer[0]!=0x55) 
	{
		ucRxCnt=0;
		return;
	}
	if (ucRxCnt<2) {	return;}
	if (ucRxBuffer[1]!=0x51) {	
	ucRxCnt=0;
	return;}
	if (ucRxCnt<44) {	return;}
	else{
		if(ucRxBuffer[1]==0x51&&ucRxBuffer[12]==0x52&&ucRxBuffer[23]==0x53&&ucRxBuffer[34]==0x54)
		UART_send_buffer(USART3,ucRxBuffer,44);
	 else ucRxCnt=0;
	} 
	
		
	}
	
	 //USART_ClearITPendingBit(USART2, USART_IT_RXNE); 
	 ucRxCnt=0;

}

 
void SysTick_Handler(void)//ϵͳ��ȷ��ʱ�жϷ�����
{
		if (TimingDelay != 0x00)
		{ 
			 TimingDelay--;
		}
	;
}

 /*�ⲿ�жϷ�������д��ʽ*/
//void EXTI15_10_IRQnHandler(void)//PA��15~9��~PG��15~9���жϷ�����
//void EXTI9_5_IRQnHandler(void)//PA��5~9��~PG��5~9���жϷ�����
//void EXTI1_IRQHandler(void)//PA1~PG1 �жϷ�����
//void EXTI2_IRQHandler(void)//PA2~PG2 �жϷ�����
//void EXTI3_IRQHandler(void)//PA3~PG3 �жϷ�����
//void EXTI4_IRQHandler(void)//PA4~PG4 �жϷ�����

void EXTI0_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line0) != RESET) //ȷ���Ƿ������EXTI Line�ж�
	{
	 	//Delay_10us(500);//�޷�����ϵͳ��ʱ����
		 
	
		EXTI_ClearITPendingBit(EXTI_Line0);     //����жϱ�־λ
	}  
	/*****
		if(EXTI_GetITStatus(EXTI_Linex) != RESET) //ȷ���Ƿ������EXTI Line�ж�
	{
	    ///------------------
	     �û�����
      ///-----------------
		EXTI_ClearITPendingBit(EXTI_Linex);     //����жϱ�־λ
	}  
	*****/
}
// EXTI Line --> PF9
void Main_Delay(unsigned int delayvalue)
{
	unsigned int i;
	while(delayvalue-->0)
	{	
		i=5000;
		while(i-->0);
	}
}
void EXTI9_5_IRQHandler(void)
{ 
  Main_Delay(500);
  if(DC_OVER==1)	 //����KEY0
	{
	 TIM_SetCompare1(TIM3,54);	 
								TIM_SetCompare1(TIM4,54);
                TIM_SetCompare2(TIM3,54);	
	 LED2_FLIP;
	 TIM_Cmd(TIM3,ENABLE);    
								TIM_Cmd(TIM4,ENABLE); 
								Timer2_Counter6=0;
		Main_Delay(1000);
		TIM_Cmd(TIM3,DISABLE);    
								TIM_Cmd(TIM4,DISABLE); 
		
	}	

	EXTI_ClearITPendingBit(EXTI_Line7);  //���LINE7�ϵ��жϱ�־λ  
}




void SDIO_IRQHandler(void) //��SDIO_ITConfig(���������������sdio�ж�	�� ���ݴ������ʱ�����ж�
{
		
	//SD_ProcessIRQSrc();// Process All SDIO Interrupt Sources 
}
	
	
void DMA1_Channel5_IRQHandler(void)
{
if(DMA_GetFlagStatus(DMA1_FLAG_TC5)==SET)
   {
        DMA_ClearFlag(DMA1_FLAG_TC5);//���жϱ�־�������һֱ�ж�
			  UART1_DMA_Flag =0x01;
				GPIO_Flip_level(GPIOF,GPIO_Pin_7) ;		 
   }
}
// CAN1 �жϺ���
void USB_LP_CAN1_RX0_IRQHandler(void)
{
  CanRxMsg RxMessage;				 //CAN���ջ�����
	CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
	CAN_RxCpltCallback(&RxMessage);
	GPIO_Flip_level(GPIOF,GPIO_Pin_8);
	CAN1_Flag=0x01;	
 //if((RxMessage.ExtId==0x1234) && (RxMessage.IDE==CAN_ID_EXT)
 //&& (RxMessage.DLC==2) && ((RxMessage.Data[1]|RxMessage.Data[0]<<8)==0xDECA))

 
}
 



//����5�жϣ���̬���ٶȴ���,ͷ�ļ�����can.h �У�









void CopeSerial2Data(unsigned char ucData)
{
	//Delay_10us(10000);
 static unsigned char ucRxBuffer[250];
	static unsigned char ucRxCnt = 0;	
	

	ucRxBuffer[ucRxCnt++]=ucData;	//���յ������ݴ��뻺������

	if (ucRxBuffer[0]!=0x55) //����ͷ���ԣ������¿�ʼѰ��0x55����ͷ
	{
		ucRxCnt=0;
		return;
	}
	if (ucRxCnt<11) {return;}//���ݲ���11�����򷵻�
	else
	{
		switch(ucRxBuffer[1])//�ж��������������ݣ�Ȼ���俽������Ӧ�Ľṹ���У���Щ���ݰ���Ҫͨ����λ���򿪶�Ӧ������󣬲��ܽ��յ�������ݰ�������
		{


		  case 0x51:	 imudata1.ax= (int16_t)(ucRxBuffer[3]<<8|ucRxBuffer[2]);
//				           UART_send_floatdat(UART5,(float)imudata1.ax*180/32768);
//		               UART_send_data(UART5,0x0d);  
//			             UART_send_char(UART5,0x0a); 
//		 	      
		               imudata1.ay= (int16_t)(ucRxBuffer[5]<<8|ucRxBuffer[4]);
//                   UART_send_floatdat(UART5,(float)imudata1.ay*180/32768);
//			             UART_send_char(UART5,0x0d);  
//			             UART_send_char(UART5,0x0a);
//			    
				           imudata1.az= (int16_t)(ucRxBuffer[7]<<8|ucRxBuffer[6]);
//                   UART_send_floatdat(UART5,(float)imudata1.az*180/32768);
//			             UART_send_char(UART5,0x0d);  
//			             UART_send_char(UART5,0x0a);
//			
			             break;
		
			case 0x52:  imudata1.gx= (int16_t)(ucRxBuffer[3]<<8|ucRxBuffer[2]);
//                   UART_send_floatdat(UART5,(float)imudata1.gx*180/32768);
//			             UART_send_char(UART5,0x0d); 
//		 	             UART_send_char(UART5,0x0a);
//									 
		               imudata1.gy= (int16_t)(ucRxBuffer[5]<<8|ucRxBuffer[4]);
//                   UART_send_floatdat(UART5,(float)imudata1.gy*180/32768);
//			             UART_send_char(UART5,0x0d);  
//			             UART_send_char(UART5,0x0a);
//			    
				           imudata1.gz= (int16_t)(ucRxBuffer[7]<<8|ucRxBuffer[6]);
//                   UART_send_floatdat(UART5,(float)imudata1.gz*180/32768);
//			             UART_send_char(UART5,0x0d);  
//			             UART_send_char(UART5,0x0a);
			
			             break;

									 
			case 0x53:	imudata1.roll= (int16_t)(ucRxBuffer[3]<<8|ucRxBuffer[2]);
//                   UART_send_floatdat(UART5,(float)imudata1.roll*180/32768);
//		               UART_send_char(UART5,0x0d);  
//			             UART_send_char(UART5,0x0a); 
//		 	      
		               imudata1.pitch= (int16_t)(ucRxBuffer[5]<<8|ucRxBuffer[4]);
//                   UART_send_floatdat(UART5,(float)imudata1.pitch*180/32768);
//			             UART_send_char(UART5,0x0d);  
//			             UART_send_char(UART5,0x0a);
			    
				           imudata1.yaw= (int16_t)(ucRxBuffer[7]<<8|ucRxBuffer[6]);
//                   UART_send_floatdat(UART5,(float)imudata1.yaw*180/32768);
//			             UART_send_char(UART5,0x0d);  
//		             UART_send_char(UART5,0x0a);
//			
			             break;			
		}
    ucRxCnt=0;//��ջ�����
		
	}
    ucRxCnt=0;

}



void UART5_IRQHandler(void)
{  
  if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)
	{ 	
		u8 ch;
	  USART_ClearITPendingBit(UART5,USART_IT_RXNE);
		ch=UART5->DR;

	  
		
    CopeSerial2Data(ch);	  	 
	   	
	} 
}
	
	
	
	
/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
