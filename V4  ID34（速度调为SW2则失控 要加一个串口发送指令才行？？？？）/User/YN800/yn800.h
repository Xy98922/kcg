#ifndef __YN800_H
#define __YN800_H

#include "stm32f10x.h"




//数据发送函数
void send_loradata(void);

void recv_loradata(uint8_t ch);

void deal_loradata(uint8_t *buf,uint8_t uart3_len);
#endif