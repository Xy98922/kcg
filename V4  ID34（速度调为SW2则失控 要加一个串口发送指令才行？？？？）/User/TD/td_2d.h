#ifndef __TD_2D_H
#define	__TD_2D_H

#include "stm32f10x.h"

typedef struct{
	 /*****安排过度过程*******/
	  float x1;//跟踪微分期状态量
	  float x2;//跟踪微分期状态量微分项
	  float r;//时间尺度
	  float h;//ADRC系统积分时间
	  uint16_t N0;//跟踪微分器解决速度超调h0=N*h

	  float h0;
	  float fh;//最速微分加速度跟踪量
} Fhan_Data;

void ADRC_Init(Fhan_Data *fhan_Input);
void ADRC1_Init(Fhan_Data *fhan_Input);
void Fhan_TD(Fhan_Data *fhan_Input,float expect_rcDat);

#endif
