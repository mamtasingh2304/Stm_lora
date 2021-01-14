#ifndef __ADC_h__
#define __ADC_h__


#include "stm32f10x.h"




void ADC1_Init(void);
uint16_t ADC1_Read(void);
extern uint16_t adcValue;
extern char sAdcValue[5];


#endif



/********************************* END OF FILE ********************************/
/******************************************************************************/

