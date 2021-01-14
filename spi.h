#ifndef __SPI_h__
#define __SPI_h__


#include "stm32f10x.h"

  #define SPI1_CLK               RCC_APB2Periph_SPI1
  #define SPI1_GPIO              GPIOA
  #define SPI1_GPIO_CLK          RCC_APB2Periph_GPIOA  
  #define SPI1_PIN_SCK           GPIO_Pin_5
  #define SPI1_PIN_MISO          GPIO_Pin_6
  #define SPI1_PIN_MOSI          GPIO_Pin_7

#define NOP()                _asm("nop")

//#define nSEL_H() 					NOP()//	PA_ODR |= 0x08//PA3
//#define nSEL_L() 					NOP()//	PA_ODR &= 0xf7
//#define SCK_H()						NOP()//	PC_ODR |= 0x20//PC5
//#define SCK_L()						NOP()//	PC_ODR &= 0xdf
//#define SDI_H()						NOP()//	PC_ODR |= 0x40//PC6
//#define SDI_L()						NOP()//	PC_ODR &= 0xbf
//#define SDN_H()						NOP()//	PC_ODR |= 0x08//PC3
//#define SDN_L()						NOP()//	PC_ODR &= 0xf7
//#define Get_SDO()					NOP()//	(PC_IDR & 0x80) == 0x80//PC7
#define NSS_H()						  GPIO_WriteBit(GPIOA, GPIO_Pin_4,Bit_SET);
#define NSS_L()						  GPIO_WriteBit(GPIOA, GPIO_Pin_4,Bit_RESET);
#define Get_NIRQ()				  GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2)


void SPI1_EnableSlave(void);
void SPI1_DisableSlave(void);
void SPICmd8bit(uint8_t outByte); // need to be change the return type to void
uint8_t SPIRead(uint8_t adr);
uint8_t SPIRead8bit(void);
void SPIWrite(uint8_t adr, uint8_t WrPara);
void SPIBurstRead(uint8_t adr, uint8_t *ptr, uint8_t length);
void BurstWrite(uint8_t adr, const uint8_t *ptr, uint8_t length);

#endif
/********************************* END OF FILE ********************************/
/******************************************************************************/

