#ifndef __DBUS_H
#define	__DBUS_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f10x.h"
#include "stm32f10x_usart.h"

 
enum{
    DBUS_INIT	= 0,
    DBUS_RUN 	= 1,
 
};
/** 
  * @brief  remote control information
  */
typedef __packed struct
{
  /* rocker channel information */
  int16_t ch1;
  int16_t ch2;
  int16_t ch3;
  int16_t ch4;

  /* left and right lever information */
  int16_t sw1;
  int16_t sw2;
	
	uint8_t x;
	uint8_t y;
	uint8_t z;
	uint8_t press_l;
	uint8_t press_r;
	uint8_t v;

	int16_t ch1_offset;
  int16_t ch2_offset;
  int16_t ch3_offset;
  int16_t ch4_offset;
	int16_t ch7_offset;
	int16_t ch8_offset;
	int16_t ch9_offset;
	int16_t ch10_offset;
	uint32_t cnt; //标记接收帧的编号
	uint8_t available;
	int16_t ch7;
	int16_t ch8;
	int16_t ch9;
	int16_t ch10;
	int16_t ch11;
	int16_t ch12;
	int16_t ch13;
	int16_t ch14;
	int16_t ch15;
	int16_t ch16;
} rc_info_t;


void RC_Callback_Handler(uint8_t *buff);
char RC_Offset_Init(void);
void RC_Debug_Message(void);
void RC_Upload_Message(void);
 void Sbus_Data_Count(uint8_t *buf);



#ifdef __cplusplus
}
#endif

#endif
