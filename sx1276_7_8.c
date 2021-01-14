/*
 * sx1276_7_8.c
 *
 *  
 *      Author: mamta
 */

#include "sx1276_7_8.h"
#include "main.h"

/************************Description************************

	STM32F103C8t6		LORA RA-01
		PA7			-	MOSI - 14
		PA6			-	MISO - 13
		PA5			-	SCK - 12
		PA4			- 	NSS - 15
		PA3			-	DIO1 - 6
		PA2			-	DIO0 - 5
		PA1			-	RESET - 4
		3.3v		-	3.3v - 3
		GND			- 	GND-2,9,16

************************************************/
/************************************************
//  RF module:           sx1276_7_8
//  FSK:
//  Carry Frequency:     434MHz
//  Bit Rate:            1.2Kbps/2.4Kbps/4.8Kbps/9.6Kbps
//  Tx Power Output:     20dbm/17dbm/14dbm/11dbm
//  Frequency Deviation: +/-35KHz
//  Receive Bandwidth:   83KHz
//  Coding:              NRZ
//  Packet Format:       0x5555555555+0xAA2DD4+"Mark1 Lora sx1276_7_8" (total: 29 bytes)
//  LoRa:
//  Carry Frequency:     434MHz
//  Spreading Factor:    6/7/8/9/10/11/12
//  Tx Power Output:     20dbm/17dbm/14dbm/11dbm
//  Receive Bandwidth:   7.8KHz/10.4KHz/15.6KHz/20.8KHz/31.2KHz/41.7KHz/62.5KHz/125KHz/250KHz/500KHz
//  Coding:              NRZ
//  Packet Format:       "Mark1 Lora sx1276_7_8" (total: 21 bytes)
//  Tx Current:          about 120mA  (RFOP=+20dBm,typ.)
//  Rx Current:          about 11.5mA  (typ.)
**********************************************************/

/**********************************************************
**Parameter table define
**********************************************************/
const uint8_t sx1276_7_8FreqTbl[1][3] =
{
  //{0x6C, 0x80, 0x00}, //434MHz
    //{0xD8,0x40,0X00},//865MHz
		{0xD8,0xC0,0X00},//867MHz
};

const uint8_t sx1276_7_8PowerTbl[4] =
{
  0xFF,                   //20dbm
  0xFC,                   //17dbm
  0xF9,                   //14dbm
  0xF6,                   //11dbm
};

const uint8_t sx1276_7_8SpreadFactorTbl[7] =
{
  6,7,8,9,10,11,12
};

const uint8_t sx1276_7_8LoRaBwTbl[10] =
{
//7.8KHz,10.4KHz,15.6KHz,20.8KHz,31.2KHz,41.7KHz,62.5KHz,125KHz,250KHz,500KHz
  0,1,2,3,4,5,6,7,8,9
};

uint8_t  sx1276_7_8Data[64] = {"000_000_00000000_0000_000"};
//uint8_t  sx1276_7_8Data[64];
//const uint8_t  sx1276_7_8Data[] = {"Mark1 Lora sx1276_7_8"};

uint8_t RxData[64];

/**********************************************************
**Variable define
**********************************************************/

void sx1276_7_8_Config(void);

/**********************************************************
**Name:     sx1276_7_8_Standby
**Function: Entry standby mode
**Input:    None
**Output:   None
**********************************************************/
void sx1276_7_8_Standby(void)
{
  //SPIWrite(LR_RegOpMode,0x09);                              		//Standby//Low Frequency Mode
	SPIWrite(LR_RegOpMode,0x01);                              	 //Standby//High Frequency Mode
}

/**********************************************************
**Name:     sx1276_7_8_Sleep
**Function: Entry sleep mode
**Input:    None
**Output:   None
**********************************************************/
void sx1276_7_8_Sleep(void)
{
  //SPIWrite(LR_RegOpMode,0x08);                              		//Sleep//Low Frequency Mode
	SPIWrite(LR_RegOpMode,0x00);                            		 //Sleep//High Frequency Mode
}

/*********************************************************/
//LoRa mode
/*********************************************************/
/**********************************************************
**Name:     sx1276_7_8_EntryLoRa
**Function: Set RFM69 entry LoRa(LongRange) mode
**Input:    None
**Output:   None
**********************************************************/
void sx1276_7_8_EntryLoRa(void)
{
  //SPIWrite(LR_RegOpMode,0x88);//Low Frequency Mode
	SPIWrite(LR_RegOpMode,0x80);//High Frequency Mode
}

/**********************************************************
**Name:     sx1276_7_8_LoRaClearIrq
**Function: Clear all irq
**Input:    None
**Output:   None
**********************************************************/
void sx1276_7_8_LoRaClearIrq(void)
{
  SPIWrite(LR_RegIrqFlags,0xFF);
}

/**********************************************************
**Name:     sx1276_7_8_LoRaEntryRx
**Function: Entry Rx mode
**Input:    None
**Output:   None
**********************************************************/
uint8_t sx1276_7_8_LoRaEntryRx(void)
{
  uint8_t addr;

  sx1276_7_8_Config();                                         //setting base parameter

  SPIWrite(REG_LR_PADAC,0x84);                              //Normal and Rx
  SPIWrite(LR_RegHopPeriod,0xFF);                          //RegHopPeriod NO FHSS
  SPIWrite(REG_LR_DIOMAPPING1,0x01);                       //DIO0=00, DIO1=00, DIO2=00, DIO3=01

  SPIWrite(LR_RegIrqFlagsMask,0x3F);                       //Open RxDone interrupt & Timeout
  sx1276_7_8_LoRaClearIrq();

  SPIWrite(LR_RegPayloadLength,strlen((char*)sx1276_7_8Data));                       //RegPayloadLength  21byte(this register must difine when the data long of one byte in SF is 6)

  addr = SPIRead(LR_RegFifoRxBaseAddr);           				//Read RxBaseAddr
  SPIWrite(LR_RegFifoAddrPtr,addr);                        //RxBaseAddr -> FiFoAddrPtr¡¡
  //SPIWrite(LR_RegOpMode,0x8d);                        		//Continuous Rx Mode//Low Frequency Mode
	SPIWrite(LR_RegOpMode,0x85);                        		//Continuous Rx Mode//High Frequency Mode
	SysTime = 0;
	while(1)
	{
		uint8_t ret = SPIRead(LR_RegModemStat);
		if(( ret &0x04) == 0x04)   //Rx-on going RegModemStat
			break;
		if(SysTime>=3)
			return 0;                                              //over time for error
	}
	return 0;
}

/**********************************************************
**Name:     sx1276_7_8_LoRaReadRSSI
**Function: Read the packet RSSI value
**Input:    none
**Output:   temp, RSSI value
**********************************************************/
int sx1276_7_8_LoRaPktReadRSSI(void)
{
	int8_t buf[10];
	int8_t RSSI= 0;
  int8_t temp1= 0,temp2=0;
  temp1=SPIRead(LR_RegPktRssiValue);                  //Read RegRssiValue£¬Rssi value
	temp2=SPIRead(LR_RegPktSnrValue);
	//sprintf((char*)buf,"snr %i",temp2);
	if (temp2>=0){
	RSSI=  -157+1.06*temp1;	
	}
	else{
		RSSI=-157+temp1+0.25*temp2;	
	}
                                        //127:Max RSSI, 137:RSSI offset
	return RSSI;
}

/**********************************************************
**Name:     sx1276_7_8_LoRaReadRSSI
**Function: Read the RSSI value
**Input:    none
**Output:   temp, RSSI value
**********************************************************/
uint8_t sx1276_7_8_LoRaReadRSSI(void)
{
	uint8_t temp1[10];
  uint16_t temp=10;
  temp=SPIRead(LR_RegRssiValue);                  //Read RegRssiValue£¬Rssi value
	sprintf((char*)temp1,"TEMP  : %i\n",temp);
	//USART1_PutString((char*)temp1);
	//USART1_PutString("\r\n");
  temp=temp+127-137;                                       //127:Max RSSI, 137:RSSI offset
  return (uint8_t)temp;
}

/**********************************************************
**Name:     sx1276_7_8_LoRaRxPacket
**Function: Receive data in LoRa mode
**Input:    None
**Output:   1- Success
            0- Fail
**********************************************************/
uint8_t sx1276_7_8_LoRaRxPacket(void)
{
	uint8_t i;
  uint8_t addr;
  uint8_t packet_size;
 
  if(Get_NIRQ())
  {
    for(i=0;i<52;i++)
      RxData[i] = 0x00;
		//memset(RxData,'\0',sizeof(RxData));

    addr = SPIRead(LR_RegFifoRxCurrentaddr);      //last packet addr
    SPIWrite(LR_RegFifoAddrPtr,addr);                      //RxBaseAddr -> FiFoAddrPtr
    if(sx1276_7_8SpreadFactorTbl[Lora_Rate_Sel]==6)           //When SpreadFactor is six£¬will used Implicit Header mode(Excluding internal packet length)
    packet_size=21;
    else
      packet_size = SPIRead(LR_RegRxNbBytes);     //Number for received bytes
		/*************debug*********************
		sprintf(rx,"packet size :%i",packet_size);
		USART1_PutString(rx);
		***************************************/
    SPIBurstRead(0x00, RxData, packet_size);

    sx1276_7_8_LoRaClearIrq();
//    for(i=0;i<17;i++)
//    {
//      if(RxData[i]!=sx1276_7_8Data[i])
//        break;
//    }
//    if(i>=17)                                              //Rx success
//      return(1);
//    else
//      return(0);
  }
  else
    return(0);
  return(1);
}

/**********************************************************
**Name:     sx1276_7_8_LoRaEntryTx
**Function: Entry Tx mode
**Input:    None
**Output:   None
**********************************************************/
uint8_t sx1276_7_8_LoRaEntryTx(void)
{
  uint8_t addr,temp;
  sx1276_7_8_Config();                                         //setting base parameter
  char len[10];
  SPIWrite(REG_LR_PADAC,0x87);                                   //Tx for 20dBm
  SPIWrite(LR_RegHopPeriod,0x00);                               //RegHopPeriod NO FHSS
  SPIWrite(REG_LR_DIOMAPPING1,0x41);                       //DIO0=01, DIO1=00, DIO2=00, DIO3=01

  sx1276_7_8_LoRaClearIrq();
  SPIWrite(LR_RegIrqFlagsMask,0xF7);                       //Open TxDone interrupt
	
  SPIWrite(LR_RegPayloadLength,strlen((char*)sx1276_7_8Data));                       //RegPayloadLength  21byte*/

  addr = SPIRead(LR_RegFifoTxBaseAddr);           //RegFiFoTxBaseAddr
  SPIWrite(LR_RegFifoAddrPtr,addr);                        //RegFifoAddrPtr
	SysTime = 0;
	while(1)
	{
		temp=SPIRead(LR_RegPayloadLength);
		/**************************************************************************
		sprintf(len,"payload len :%i\r\n",temp);
		USART1_PutString(len);
		*************************************************************************/
	  if (temp == strlen((char*)sx1276_7_8Data))	//if(temp==155)
		{
			break;
		}
		if(SysTime>=3)
			return 0;
	}
	return 0;
}
/**********************************************************
**Name:     sx1276_7_8_LoRaTxPacket
**Function: Send data in LoRa mode
**Input:    None
**Output:   1- Send over
**********************************************************/
uint8_t sx1276_7_8_LoRaTxPacket(void)
{
 // uint8_t TxFlag=0;
 // uint8_t addr;
size_t len= 0;
	BurstWrite(0x00, (uint8_t *)sx1276_7_8Data, strlen((char*)sx1276_7_8Data));
	/**********debug******************************
	//USART1_PutString((char*)sx1276_7_8Data);
	 len  = strlen((char*)sx1276_7_8Data);
	 USART1_PutString((char*)len);
	********************************************************/
	//SPIWrite(LR_RegOpMode,0x8b);                    //Tx Mode
	SPIWrite(LR_RegOpMode,0x83);                    //TX HIGH MODE
	while(1)
	{
		if(Get_NIRQ())                      //Packet send over
		{
			SPIRead(LR_RegIrqFlags);
			sx1276_7_8_LoRaClearIrq();                                //Clear irq

			sx1276_7_8_Standby();                                     //Entry Standby mode

			break;
		}
	}
	return 0;
}

/**********************************************************
**Name:     sx1276_7_8_ReadRSSI
**Function: Read the RSSI value
**Input:    none
**Output:   temp, RSSI value
**********************************************************/
uint8_t sx1276_7_8_ReadRSSI(void)
{
  uint8_t temp=0xff;

  temp=SPIRead(0x11);
  temp>>=1;
  temp=127-temp;                                           //127:Max RSSI
  return temp;
}
/**********************************************************
**Name:     sx1276_7_8_Config
**Function: sx1276_7_8 base config
**Input:    mode
**Output:   None
**********************************************************/
void sx1276_7_8_Config(void)
{
  //uint8_t i;

  sx1276_7_8_Sleep();                                      //Change modem mode Must in Sleep mode
    //NOP();
	HAL_Delay(10);

  //lora mode
	sx1276_7_8_EntryLoRa();
	//SPIWrite(0x5904);   //?? Change digital regulator form 1.6V to 1.47V: see errata note

	BurstWrite(LR_RegFrMsb,sx1276_7_8FreqTbl[Freq_Sel],3);  //setting frequency parameter

	//setting base parameter
	SPIWrite(LR_RegPaConfig,sx1276_7_8PowerTbl[Power_Sel]);             //Setting output power parameter

	SPIWrite(LR_RegOcp,0x0B);                              //RegOcp,Close Ocp
	SPIWrite(LR_RegLna,0x23);                              //RegLNA,High & LNA Enable

	if(sx1276_7_8SpreadFactorTbl[Lora_Rate_Sel]==6)           //SFactor=6
	{
		uint8_t tmp;
		SPIWrite(LR_RegModemConfig1,((sx1276_7_8LoRaBwTbl[BandWide_Sel]<<4)+(CR<<1)+0x01));//Implicit Enable CRC Enable(0x02) & Error Coding rate 4/5(0x01), 4/6(0x02), 4/7(0x03), 4/8(0x04)
		SPIWrite(LR_RegModemConfig2,((sx1276_7_8SpreadFactorTbl[Lora_Rate_Sel]<<4)+(SPI_CRC<<2)+0x03));

		tmp = SPIRead(0x31);
		tmp &= 0xF8;
		tmp |= 0x05;
		SPIWrite(0x31,tmp);
		SPIWrite(0x37,0x0C);
	}
	else
	{
		SPIWrite(LR_RegModemConfig1,((sx1276_7_8LoRaBwTbl[BandWide_Sel]<<4)+(CR<<1)+0x00));//Explicit Enable CRC Enable(0x02) & Error Coding rate 4/5(0x01), 4/6(0x02), 4/7(0x03), 4/8(0x04)
		SPIWrite(LR_RegModemConfig2,((sx1276_7_8SpreadFactorTbl[Lora_Rate_Sel]<<4)+(SPI_CRC<<2)+0x03));  //SFactor &  LNA gain set by the internal AGC loop
	}
	SPIWrite(LR_RegSymbTimeoutLsb,0xFF);                   //RegSymbTimeoutLsb Timeout = 0x3FF(Max)

	SPIWrite(LR_RegPreambleMsb,0x00);                       //RegPreambleMsb
	SPIWrite(LR_RegPreambleLsb,12);                      //RegPreambleLsb 8+4=12byte Preamble

	SPIWrite(REG_LR_DIOMAPPING2,0x01);                     //RegDioMapping2 DIO5=00, DIO4=01

  sx1276_7_8_Standby();                                         //Entry standby mode
}
