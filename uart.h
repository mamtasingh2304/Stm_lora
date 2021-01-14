#ifndef __UART_h__
#define __UART_h__


#include "stm32f10x.h"





void USART1_PutChar(uint16_t Data);
int get_char_usart1(void);
void USART1_PutString(char *s);
void num(int x);


#endif



/********************************* END OF FILE ********************************/
/******************************************************************************/


