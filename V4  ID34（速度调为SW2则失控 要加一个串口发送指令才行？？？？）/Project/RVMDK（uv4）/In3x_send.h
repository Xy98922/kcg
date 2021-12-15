#ifndef __LN3X_H
#define __LN3X_H

#include "stm32f10x.h"

#define yes 1
#define no 0
#define NULL 0
#define fail 1
#define done 0
#define true 1
#define falth 0

//接收计数器,表示一个包收到了多少个数据
#define RS_IDLE 0XFF//表示包还没有收到包头和长度
#define RS_DONE 0xA0//表示包已经完成了接收

typedef struct sPkg__
{
	 u8 length;              //数据长度
	 u8 src_port;            //源端口号
	 u8 dis_port;            //目的端口号
	 u8 remote_addrH;        //目的地址
	 u8 remote_addrL;
	 u8 data[59];            //传输数据   
}sPkg;


//接收包使用的结构体,将端口和地址都融入了data中
typedef struct sPkgBase__
{
	u8 length;
	u8 data[63];
}sPkgBase;

#define newPkg(num)  \
struct               \
{                    \
	u8 length;       \
	u8 src_port;     \
	u8 dis_port;     \
	u8 remote_addrH; \
	u8 remote_addrL; \
	u8 data[num];    \
}


//收到包头函数
void recvHead(u8 totle);
//收到包尾0xff
void recvTerminal(void);
//收到数据函数
void recvData(u8 data);
//这个标志在上一个收到了0xfe时赋值为yes,否则为0

////调用这个函数说明上一个包已经处理完了,并尝试接收一个包,
////如果已经收到包,则返回一个指向包的指针,否则返回null
//sPkg* getNextPkg();

//串口中断调用该函数，用于数据包解析
void uartRevieveByte(u8 data);

u8 uartSendNextByte(void);
u8 sendPkg(sPkg* pkg);

#endif
