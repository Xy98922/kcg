#include "ln3x.h"
#include "bsp_uart.h"

//这个标志在上一个收到了0xfe时赋值为yes,否则为0
static u8 escape = no;
//接受包用的缓冲
sPkg Load_pkg;

u8 Recv_counter = RS_IDLE;

//串口收到1byte数据，中断调用此函数
void uartRevieveByte(u8 data)
{
	switch(data){
	case 0xff:             //收到结束字符
		recvTerminal();    
		break;
	case 0xfe:             //收到转义字符
		escape=yes;
	break;
	default:               //收到一般数据
		if(escape==yes){   //收到的前一个数据为转义字符
			escape=no;
			if(data<=63){
				recvHead(data);              //接收字长
			}
			else{
				recvData(data+2);            //收到转义数据的后一个数据需加2
			}
		}
		else{
			recvData(data);                  //前一个字符不是转义数据，直接接收
		}
		break;
	}
	
}

////接收计数器,表示一个包收到了多少个数据
//#define RS_IDLE 0XFF//表示包还没有收到包头和长度
//#define RS_DONE 0xA0//表示包已经完成了接收

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