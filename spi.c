#include "spi.h"
#include "main.h"

// ========================================================
/// @file       SPI.c
/// @brief      Software simulation SPI performance function
/// @version    V1.0
/// @date       2014/08/04
/// @company    .
/// @website    
/// @author     ISA
/// @mobile     
/// @tel        
// ========================================================
#include "spi.h"

void HAL_SPI_Transmit(uint8_t* buf)
{
	//SPI_I2S_SendData(SPI1,*buf);
	SPI1->DR = *buf;  
//	while (!(SPI1->SR & SPI_SR_TXE)) ;  
			//SPI_I2S_SendData(SPI1,*buf);
		//*(uint32_t *)0x4001300C = *buf;  
	  volatile uint8_t flag = *(uint8_t *)0x40013008;
		//while (!(flag & SPI_SR_TXE)) ;  
	HAL_Delay(1);
		*buf = *(uint8_t *)0x4001300C;  
}

/**********************************************************
**Name:     SPICmd8bit
**Function: SPI Write one byte
**Input:    WrPara
**Output:   none
**note:     use for burst mode
**********************************************************/
void SPICmd8bit(uint8_t WrPara)
{
 // uint8_t bitcnt;
 // //nSEL_L();
	NSS_L();
 // SCK_L();
  
  HAL_SPI_Transmit(&WrPara);
//  for(bitcnt=8; bitcnt!=0; bitcnt--)
//  {
//    SCK_L();
//    if(WrPara&0x80)
//      SDI_H();
//    else
//      SDI_L();
//    SCK_H();
//    WrPara <<= 1;
//  }
//  SCK_L();
//  SDI_H();
}

void HAL_SPI_Receive(uint8_t* buf)
{
		//SPI_I2S_SendData(SPI1,*buf);
		SPI1->DR = 0;  
	  volatile uint8_t flag = *(uint8_t *)0x40013008;
		//while (!(flag & SPI_SR_RXNE)) ;  
	   HAL_Delay(1);
		*buf = *(uint8_t *)0x4001300C;  
}

/**********************************************************
**Name:     SPIRead8bit
**Function: SPI Read one byte
**Input:    None
**Output:   result byte
**Note:     use for burst mode
**********************************************************/
uint8_t SPIRead8bit(void)
{
 uint8_t RdPara = 0;
// uint8_t bitcnt;
 
//  //nSEL_L();
 NSS_L();
//  SDI_H();                                                 //Read one byte data from FIFO, MOSI hold to High
 HAL_SPI_Receive(&RdPara);
//  for(bitcnt=8; bitcnt!=0; bitcnt--)
//  {
//    SCK_L();
//    RdPara <<= 1;
//    SCK_H();
//    if(Get_SDO())
//      RdPara |= 0x01;
//    else
//      RdPara |= 0x00;
//  }
//  SCK_L();
  return(RdPara);
}

/**********************************************************
**Name:     SPIRead
**Function: SPI Read CMD
**Input:    adr -> address for read
**Output:   None
**********************************************************/
uint8_t SPIRead(uint8_t adr)
{
  uint8_t tmp; 
  SPICmd8bit(adr);                                         //Send address first
  tmp = SPIRead8bit();  
  //nSEL_H();
  NSS_H();
  return(tmp);
}

/**********************************************************
**Name:     SPIWrite
**Function: SPI Write CMD
**Input:    uint8_t address & uint8_t data
**Output:   None
**********************************************************/
void SPIWrite(uint8_t adr, uint8_t WrPara)  
{
	
	//nSEL_L();
	NSS_L();
	SPICmd8bit(adr|0x80);
	SPICmd8bit(WrPara);
	
  //SCK_L();
  //SDI_H();
  //nSEL_H();
	NSS_H();
}
/**********************************************************
**Name:     SPIBurstRead
**Function: SPI burst read mode
**Input:    adr-----address for read
**          ptr-----data buffer point for read
**          length--how many bytes for read
**Output:   None
**********************************************************/
void SPIBurstRead(uint8_t adr, uint8_t *ptr, uint8_t length)
{
  uint8_t i;
  if(length<=1)                                            //length must more than one
    return;
  else
  {
    //SCK_L();
    //nSEL_L();
	  NSS_L();
    SPICmd8bit(adr); 
    for(i=0;i<length;i++)
    	ptr[i] = SPIRead8bit();
    //nSEL_H();
    NSS_H();
  }
}

/**********************************************************
**Name:     SPIBurstWrite
**Function: SPI burst write mode
**Input:    adr-----address for write
**          ptr-----data buffer point for write
**          length--how many bytes for write
**Output:   none
**********************************************************/
void BurstWrite(uint8_t adr, const uint8_t *ptr, uint8_t length)
{ 
  uint8_t i;

  if(length<=1)                                            //length must more than one
    return;
  else  
  {   
   // SCK_L();
    //nSEL_L();
	  NSS_L();
    SPICmd8bit(adr|0x80);
    for(i=0;i<length;i++)
		SPICmd8bit(ptr[i]);
    //nSEL_H();
    NSS_H();
  }
}