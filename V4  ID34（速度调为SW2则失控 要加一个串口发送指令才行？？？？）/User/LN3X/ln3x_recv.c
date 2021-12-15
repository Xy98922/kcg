#include "ln3x.h"
#include "bsp_uart.h"

//�����־����һ���յ���0xfeʱ��ֵΪyes,����Ϊ0
static u8 escape = no;
//���ܰ��õĻ���
sPkg Load_pkg;

u8 Recv_counter = RS_IDLE;

//�����յ�1byte���ݣ��жϵ��ô˺���
void uartRevieveByte(u8 data)
{
	switch(data){
	case 0xff:             //�յ������ַ�
		recvTerminal();    
		break;
	case 0xfe:             //�յ�ת���ַ�
		escape=yes;
	break;
	default:               //�յ�һ������
		if(escape==yes){   //�յ���ǰһ������Ϊת���ַ�
			escape=no;
			if(data<=63){
				recvHead(data);              //�����ֳ�
			}
			else{
				recvData(data+2);            //�յ�ת�����ݵĺ�һ���������2
			}
		}
		else{
			recvData(data);                  //ǰһ���ַ�����ת�����ݣ�ֱ�ӽ���
		}
		break;
	}
	
}

////���ռ�����,��ʾһ�����յ��˶��ٸ�����
//#define RS_IDLE 0XFF//��ʾ����û���յ���ͷ�ͳ���
//#define RS_DONE 0xA0//��ʾ���Ѿ�����˽���

void recvHead(u8 totle)
{
	if(Recv_counter == RS_IDLE)
	{
		Load_pkg.length = totle;
		Recv_counter = 0;
	}
//   UART_send_char(USART2,totle);
}

void recvData(u8 data)
{	
	if(Recv_counter < Load_pkg.length)
	{		
		Load_pkg.data[Recv_counter] = data;
//		UART_send_char(USART2,Load_pkg.data[Recv_counter]);
		Recv_counter++;
	}
	else
	{
		Recv_counter = RS_IDLE;
	}
}

void recvTerminal(void)
{
	if(Load_pkg.length==Recv_counter)

			Recv_counter=RS_DONE;
	else
		Recv_counter=RS_IDLE;
	
//	UART_send_char(USART2,0xaa);
}