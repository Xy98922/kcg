#ifndef __TD_2D_H
#define	__TD_2D_H

#include "stm32f10x.h"

typedef struct{
	 /*****���Ź��ȹ���*******/
	  float x1;//����΢����״̬��
	  float x2;//����΢����״̬��΢����
	  float r;//ʱ��߶�
	  float h;//ADRCϵͳ����ʱ��
	  uint16_t N0;//����΢��������ٶȳ���h0=N*h

	  float h0;
	  float fh;//����΢�ּ��ٶȸ�����
} Fhan_Data;

void ADRC_Init(Fhan_Data *fhan_Input);
void ADRC1_Init(Fhan_Data *fhan_Input);
void Fhan_TD(Fhan_Data *fhan_Input,float expect_rcDat);

#endif
