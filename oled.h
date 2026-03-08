#ifndef __OLED_H
#define __OLED_H

#include "stc8h.h"

// ????
sbit OLED_SCL = P2^4;
sbit OLED_SDA = P2^5;

// ????
void OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowChar(unsigned char x, unsigned char y, unsigned char chr);
void OLED_ShowString(unsigned char x, unsigned char y, char *s);
void OLED_ShowNumber(unsigned char x, unsigned char y, unsigned long num, unsigned char len, unsigned char size);
#endif