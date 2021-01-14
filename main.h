#ifndef _MAIN_H
#define _MAIN_H

#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_tim.h"

#include "string.h"
#include "stdio.h"

#include "ctype.h"
#include "misc.h"






#define HAL_GPIO_WritePin(PORT,PIN,STATE) GPIO_WriteBit(PORT, PIN, STATE)
#define RED_LED_H() GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_SET)
#define RED_LED_L() GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_RESET)
					
#define RED_LED_Pin GPIO_Pin_13
#define RED_LED_GPIO_Port GPIOC
#define Reset_Pin GPIO_Pin_1
#define Reset_GPIO_Port GPIOA
#define nIrq_Pin GPIO_Pin_2
#define nIrq_GPIO_Port GPIOA
#define DIO1_Pin GPIO_Pin_3
#define DIO1_GPIO_Port GPIOA
#define SPI_NSS_Pin GPIO_Pin_4
#define SPI_NSS_GPIO_Port GPIOA
#define SPI_SCK_Pin GPIO_Pin_5
#define SPI_SCK_GPIO_Port GPIOA
#define SPI_MISO_Pin GPIO_Pin_6
#define SPI_MISO_GPIO_Port GPIOA
#define SPI_MOSI_Pin GPIO_Pin_7
#define SPI_MOSI_GPIO_Port GPIOA

#define GPIO_PIN_RESET Bit_RESET
#define GPIO_PIN_SET Bit_SET

void TimingDelay_Decrement(void);
void HAL_Delay(__IO uint32_t nTime);

#endif
