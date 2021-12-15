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

//���ռ�����,��ʾһ�����յ��˶��ٸ�����
#define RS_IDLE 0XFF//��ʾ����û���յ���ͷ�ͳ���
#define RS_DONE 0xA0//��ʾ���Ѿ�����˽���

typedef struct sPkg__
{
	 u8 length;              //���ݳ���
	 u8 src_port;            //Դ�˿ں�
	 u8 dis_port;            //Ŀ�Ķ˿ں�
	 u8 remote_addrH;        //Ŀ�ĵ�ַ
	 u8 remote_addrL;
	 u8 data[59];            //��������   
}sPkg;


//���հ�ʹ�õĽṹ��,���˿ں͵�ַ��������data��
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


//�յ���ͷ����
void recvHead(u8 totle);
//�յ���β0xff
void recvTerminal(void);
//�յ����ݺ���
void recvData(u8 data);
//�����־����һ���յ���0xfeʱ��ֵΪyes,����Ϊ0

////�����������˵����һ�����Ѿ���������,�����Խ���һ����,
////����Ѿ��յ���,�򷵻�һ��ָ�����ָ��,���򷵻�null
//sPkg* getNextPkg();

//�����жϵ��øú������������ݰ�����
void uartRevieveByte(u8 data);

u8 uartSendNextByte(void);
u8 sendPkg(sPkg* pkg);

#endif
