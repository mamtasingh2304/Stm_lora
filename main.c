#include "main.h"
#include "spi.h"
#include "sx1276_7_8.h"
#include "uart.h"



#define lora_receiver  0
#define mqtt_subscribe 1
#define mqtt_publish   2
volatile uint32_t time_up;
volatile uint32_t time_us;
volatile uint8_t t_sec = 0; 
volatile uint8_t timer = 0;
volatile uint16_t timsec = 0;
volatile uint8_t len = 0;

volatile uint8_t v1 = 0;
volatile uint8_t v2 = 0;
uint8_t cycle = 0;
//////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////MQTT CREDENTIAL //////////////////////////////////////
#define RX_BUF_SIZE 80
#define PUB_BUF_SIZE 80
#define res_high()    GPIO_WriteBit(GPIOB, GPIO_Pin_9, Bit_SET)
#define res_low()     GPIO_WriteBit(GPIOB, GPIO_Pin_9, Bit_RESET)

#define v1_h()     GPIO_WriteBit(GPIOB, GPIO_Pin_10, Bit_RESET)
#define v1_l()     GPIO_WriteBit(GPIOB, GPIO_Pin_10, Bit_SET)
#define v2_h()     GPIO_WriteBit(GPIOB, GPIO_Pin_12, Bit_RESET)
#define v2_l()     GPIO_WriteBit(GPIOB, GPIO_Pin_12, Bit_SET)


uint8_t read_response3(char *command,  char *expected, char *expected2, uint32_t Timeout);
uint8_t read_response(char *command, char *expected, char *expected2, uint32_t Timeout);
uint8_t read_response2(char *command, char *expected1, char *expected2, char *expected3, uint32_t Timeout);
/******************************************************************/
void task_handler(uint8_t task);
uint8_t TCP_init(uint8_t AT);
uint8_t connect(void);
uint8_t TCP_init2(uint8_t command);
uint8_t  MQTTpublish(char *MQTTTopic, char *data);
uint8_t MQTTsubscribe();
void send_motorstatus();

uint8_t idr;
char aux_str[27];
char *APN_3= "internet";
char *APN_2= "airtelgprs.com";
char *APN_1= "portalnmms";
char * usrnm    = "";
char * password = "";
uint8_t network = 0;


unsigned int Counter = 0;
unsigned long datalength, checksum, rLength;
unsigned short topiclength;
unsigned short topiclength2;
char topic[50];

char str[250];
char str2[250];
unsigned char encodedByte;
int16_t X;

unsigned short MQTTProtocolNameLength;
unsigned short MQTTClientIDLength;
unsigned short MQTTUsernameLength;
unsigned short MQTTPasswordLength;
unsigned short MQTTTopicLength;

char *MQTTHost = "mqtt.xyz.com";
char *MQTTPort = "1883";
char *MQTTClientID = "ABC";
char *MQTTTopic4= "Gateway/feedback/GA140001";
char *MQTTTopic3= "Gateway/sensor/GA140001";
char *MQTTTopic2= "Gateway/action/GA140001";
char *MQTTTopic1 = "Gateway/init/GA140001";

char *MQTTProtocolName = "MQTT";
char MQTTLVL = 0x04;
char MQTTFlags = 0xC2;
uint16_t MQTTKeepAlive =0x00C8;
char *MQTTUsername = "ubuntu";
char *MQTTPassword = "mamta@2019";
char MQTTQOS = 0x00;
char MQTTPacketID = 0x0001;

char relay_buf[10];
volatile uint8_t step = 0;
volatile char RX_FLAG_END_LINE = 0;
volatile char SUB = 0;
volatile uint8_t task = 0;
volatile uint8_t TCP = 0;
volatile char RXi;
volatile char PXi;
volatile char RXc;
char pub_data[PUB_BUF_SIZE];
char RX_BUF[RX_BUF_SIZE] = {'\0'};
volatile uint8_t t_on = 0;
void clear_RXBuffer(void) {
    for (RXi=0; RXi<RX_BUF_SIZE; RXi++)
        RX_BUF[RXi] = '\0';
    RXi = 0;
}
//////////////////////////////////////////////////////////////////////////////
void USART1_IRQHandler(void)
{
    if ((USART1->SR & USART_FLAG_RXNE) != (u16)RESET)
    {
        RXc = USART_ReceiveData(USART1);
        
        RX_BUF[RXi] = RXc;
        
        
        RXi++;
        
        
        
        //Echo
        //	USART_SendData(USART2, RXc);
    }   
}
//////////////////////////////////////////////////////////////////////////////
void usart1_init(void)
{
    /* Enable USART1 and GPIOA clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
    
    /* NVIC Configuration */
    NVIC_InitTypeDef NVIC_InitStructure;
    /* Enable the USARTx Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    /* Configure the GPIOs */
    GPIO_InitTypeDef GPIO_InitStructure;
    
    /* Configure USART1 Tx (PA.09) as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    /* Configure USART1 Rx (PA.10) as input floating */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    /* Configure the USART1 */
    USART_InitTypeDef USART_InitStructure;
    
    /* USART1 configuration ------------------------------------------------------*/
    /* USART1 configured as follow:
     - BaudRate = 115200 baud
     - Word Length = 8 Bits
     - One Stop Bit
     - No parity
     - Hardware flow control disabled (RTS and CTS signals)
     - Receive and transmit enabled
     - USART Clock disabled
     - USART CPOL: Clock is active low
     - USART CPHA: Data is captured on the middle
     - USART LastBit: The clock pulse of the last data bit is not output to
     the SCLK pin
     */
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    
    USART_Init(USART1, &USART_InitStructure);
    
    /* Enable USART1 */
    USART_Cmd(USART1, ENABLE);
    
    /* Enable the USART1 Receive interrupt: this interrupt is generated when the
     USART1 receive data register is not empty */
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}



void USARTSend2(char data)
{
    USART_SendData(USART1, (uint16_t)data);
    while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
    {
    }
}
uint8_t USARTSend(char *pucBuffer)
{
    while (*pucBuffer)
    {
        USART_SendData(USART1, (uint16_t) *pucBuffer++);
        while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
        {
        }
    }
    return 1;
}
/////////////////////////////////////////////////////////////////////
void TIM4_IRQHandler(void){
    if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
    {
	
       // timsec++;
			  time_up++;
        /*if(timsec == 10){
            
            task = mqtt_subscribe;
        }*/
				if(time_up == 129){
            task = mqtt_publish;
        }
			
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
				
}
		}
void tim_init(){
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); // TIM4
    // TIMER4
    TIM_TimeBaseInitTypeDef TIMER_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    
    /* NVIC Configuration */
    
    
    
    TIMER_InitStructure.TIM_Prescaler = 7200; // ???=????? ???????
    TIMER_InitStructure.TIM_Period = 10000; //  // F=72000000/7200/10000 = 1sec
    TIMER_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIMER_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &TIMER_InitStructure);
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE); //
    TIM_Cmd(TIM4, ENABLE);//
    
    
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
////////////////////////////////////////////////////////////////////////////////////
uint8_t strBuf1[140];
uint8_t strBuf[155];
uint16_t SysTime;
uint16_t time2_count;
uint16_t key1_time_count;
uint16_t key2_time_count;
uint8_t rf_rx_packet_length;

uint8_t mode;//lora--1/FSK--0
uint8_t Freq_Sel;//
uint8_t Power_Sel;//
uint8_t Lora_Rate_Sel;//
uint8_t BandWide_Sel;//

uint8_t Fsk_Rate_Sel;//

uint8_t key1_count;
/*key1_count = 0----------->lora master
 key1_count = 1----------->lora slaver
 key1_count = 2----------->FSK TX test
 key1_count = 3----------->FSK RX test
 */
uint8_t time_flag;
/*{
 bit0 time_1s;
 bit1 time_2s;
 bit2 time_50ms;
 bit3 ;
 bit4 ;
 bit5 ;
 bit6 ;
 bit7 ;
 }*/
uint8_t	operation_flag;
/*
 uchar	:;
 uchar	:;
 uchar	;
 }*/

SPI_InitTypeDef   SPI_InitStructure;

static __IO uint32_t TimingDelay;

/* Private function prototypes -----------------------------------------------*/
void Delay(__IO uint32_t nTime);



void gpio_initialize()
{
    
    GPIO_InitTypeDef Gpio_init;
    
    //Enable Clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOA, ENABLE);
    
    //Set Mode,pin,speed,nss
    Gpio_init.GPIO_Pin = GPIO_Pin_4;
    Gpio_init.GPIO_Speed = GPIO_Speed_2MHz;
    Gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &Gpio_init);
    NSS_H();
    
    
    // nIrq//DIO0//A2
    Gpio_init.GPIO_Pin = GPIO_Pin_2;
    Gpio_init.GPIO_Speed = GPIO_Speed_2MHz;
    Gpio_init.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &Gpio_init);
    
    //DIO1//A3
    Gpio_init.GPIO_Pin = GPIO_Pin_3;
    Gpio_init.GPIO_Speed = GPIO_Speed_2MHz;
    Gpio_init.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &Gpio_init);
    
    //Reset//A1
    Gpio_init.GPIO_Pin = GPIO_Pin_1;
    Gpio_init.GPIO_Speed = GPIO_Speed_2MHz;
    Gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &Gpio_init);
}

void spi_GPIO_Configuration(uint16_t SPI1_Mode)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    /* Configure SPI1 pins: SCK, MISO and MOSI ---------------------------------*/
    GPIO_InitStructure.GPIO_Pin = SPI1_PIN_SCK | SPI1_PIN_MOSI;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    
    if(SPI1_Mode == SPI_Mode_Master)
    {
        /* Configure SCK and MOSI pins as Alternate Function Push Pull */
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    }
    else
    {
        /* Configure SCK and MOSI pins as Input Floating */
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    }
    GPIO_Init(SPI1_GPIO, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = SPI1_PIN_MISO;
    
    if(SPI1_Mode == SPI_Mode_Master)
    {
        /* Configure MISO pin as Input Floating  */
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    }
    else
    {
        /* Configure MISO pin as Alternate Function Push Pull */
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    }
    GPIO_Init(SPI1_GPIO, &GPIO_InitStructure);
    
}

void spi_initialization()
{
    spi_GPIO_Configuration(SPI_Mode_Master);
    
    /* SPI1 Config -------------------------------------------------------------*/
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 10;
    SPI_Init(SPI1, &SPI_InitStructure);
    
    /* Enable SPI1 */
    SPI_Cmd(SPI1, ENABLE);
}

void RCC_Configuration(void)
{
    /* PCLK2 = HCLK/2 */
    RCC_PCLK2Config(RCC_HCLK_Div2);
    /* Enable SPI1 clock and GPIO clock for SPI1 and SPIz */
    RCC_APB2PeriphClockCmd(SPI1_GPIO_CLK | SPI1_GPIO_CLK | SPI1_CLK, ENABLE);
}

void HAL_Delay(__IO uint32_t nTime)
{ 
    TimingDelay = nTime;
    
    while(TimingDelay != 0);
}

void TimingDelay_Decrement(void)
{
    if (TimingDelay != 0x00)
    {
        TimingDelay--;
    }
}
///////////////////////////////////////////////////////////////////////////
void SetSysClockTo72(void)
{
    ErrorStatus HSEStartUpStatus;
    /* SYSCLK, HCLK, PCLK2 and PCLK1 configuration -----------------------------*/
    /* RCC system reset(for debug purpose) */
    RCC_DeInit();
    
    /* Enable HSE */
    RCC_HSEConfig( RCC_HSE_ON);
    
    /* Wait till HSE is ready */
    HSEStartUpStatus = RCC_WaitForHSEStartUp();
    
    if (HSEStartUpStatus == SUCCESS)
    {
        /* Enable Prefetch Buffer */
        //FLASH_PrefetchBufferCmd( FLASH_PrefetchBuffer_Enable);
        
        /* Flash 2 wait state */
        //FLASH_SetLatency( FLASH_Latency_2);
        
        /* HCLK = SYSCLK */
        RCC_HCLKConfig( RCC_SYSCLK_Div1);
        
        /* PCLK2 = HCLK */
        RCC_PCLK2Config( RCC_HCLK_Div1);
        
        /* PCLK1 = HCLK/2 */
        RCC_PCLK1Config( RCC_HCLK_Div2);
        
        /* PLLCLK = 8MHz * 9 = 72 MHz */
        //RCC_PLLConfig(0x00010000, RCC_PLLMul_9);
        RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
        
        /* Enable PLL */
        RCC_PLLCmd( ENABLE);
        
        /* Wait till PLL is ready */
        while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
        {
        }
        
        /* Select PLL as system clock source */
        RCC_SYSCLKConfig( RCC_SYSCLKSource_PLLCLK);
        
        /* Wait till PLL is used as system clock source */
        while (RCC_GetSYSCLKSource() != 0x08)
        {
        }
    }
    else
    { /* If HSE fails to start-up, the application will have wrong clock configuration.
       User can add here some code to deal with this error */
        
        /* Go to infinite loop */
        while (1)
        {
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////
int main()
{
    SetSysClockTo72();
    RCC_Configuration();
    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 1000); //delay init
    
    uint8_t count = 0;
    ///uint8_t str1[10];
    //uint8_t str2[10];
    //char SxRec[64];
    
    int rssi = 0;
    char prev_state[3];
    uint8_t str_data[75]={'\0'};
    volatile	uint16_t k=0;
    SysTime = 0;
    operation_flag = 0x00;
    key1_count = 0x00;
    mode = 0x01;//lora mode
    Freq_Sel = 0x00;//867M
    Power_Sel = 0x00;//
    Lora_Rate_Sel = 0x03;// SF = 12
    BandWide_Sel = 0x07;
    Fsk_Rate_Sel = 0x00;
    GPIO_InitTypeDef GPIOInitStruct;
    GPIO_InitTypeDef GPIOInitStruct1;
    // Initialize GPIOA as output for LED
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIOInitStruct.GPIO_Pin = GPIO_Pin_13;
    GPIOInitStruct.GPIO_Speed = GPIO_Speed_2MHz;
    GPIOInitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &GPIOInitStruct);
    
    
    
    
    //Intialize B12-v1,B13-v2,B14-v3,B15-v4
    GPIOInitStruct1.GPIO_Pin = GPIO_Pin_10|  GPIO_Pin_12;
    GPIOInitStruct1.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIOInitStruct1.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &GPIOInitStruct1);
    //GPIO pin for MISO
    v1_l();
    v2_l();
   
    gpio_initialize();
    spi_initialization();
    usart1_init();
    
    HAL_GPIO_WritePin(GPIOA, Reset_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(GPIOA, Reset_Pin, GPIO_PIN_SET);
    
    
    
    for(int i=0;i<128;i++)
    {
        uint8_t volatile tmp = SPIRead(i);
    }
    sx1276_7_8_Config();
    volatile int ver = SPIRead(REG_LR_VERSION);
    sx1276_7_8_LoRaEntryRx();
  //  task = lora_receiver;
    task = mqtt_subscribe;
    
    if(TCP_init(1)){
       if(connect())
         MQTTpublish(MQTTTopic1,"ON");   
        
			}
        tim_init();
   
    while (1)
    {
        switch(task){
            case lora_receiver :  if(sx1276_7_8_LoRaRxPacket()){
							                     
                                      sprintf((char*)strBuf,"%s",RxData);
							                       
							                         timsec = 0;		
                                      //rssi = sx1276_7_8_LoRaPktReadRSSI();
                                     //sprintf((char*)strBuf1,"Receive Data %i",rssi);
                                    //sprintf((char*)receiver,"Receive Data %s rssi %s \r\n",strBuf,strBuf1);
                                    //RED_LED_H();
                    
                                     //HAL_Delay(1000);
                                     //RED_LED_L();
                                    //sprintf((char*)str_data,"%s%i",(char*)strBuf,rssi);
                    
                                        clear_RXBuffer();
                
                                        if( MQTTpublish(MQTTTopic3, (char*)strBuf)){
                                            // memset(str_data,0,sizeof(str_data));
                                                  memset(strBuf,0,sizeof(strBuf));
                                             }
                                         else{
																					 HAL_Delay(500);
                                             TCP_init2(1);
                                                 if(connect()){
                                                    if( MQTTpublish(MQTTTopic3, (char*)strBuf)){
                                                        //memset(str_data,0,sizeof(str_data));
                                                        memset(strBuf,0,sizeof(strBuf));
                                                        }
																											}                                              
																								 }
                                        if(task == mqtt_publish)
																				break;
																	else
                                        task = mqtt_subscribe; 
                }

                break;
                
            case mqtt_subscribe :        
                       						  //timsec = 20;
						                       clear_RXBuffer();
						                        len = 0;
                                  if(MQTTsubscribe())
																				{
																					    SUB = 0;
                                               HAL_Delay(500);
                                               for(int j=43;j<46;j++)
                                               pub_data[len++]=RX_BUF[j];
																					    if (step == 0){
	                                          	         strcpy(prev_state,pub_data);
																								       step++;
																												send_motorstatus();
																												}
		                                         else{
		                                            if((strcmp(prev_state,pub_data)) != 0){
		                                                strcpy(prev_state,pub_data);
																								       send_motorstatus();
																							   }
																							 }
																						   

                                         }
																	else
																	       {
																					     HAL_Delay(500);
                                                TCP_init2(1);
                                                  if(connect()){
                                                     if(MQTTsubscribe()){
                                                      HAL_Delay(500);
                                                      for(int j=43;j<46;j++)
																											pub_data[len++]=RX_BUF[j];
																											 if (step == 0){
	                                          	         strcpy(prev_state,pub_data);
																													step++;
																												send_motorstatus();
																												}
		                                         else{
		                                            if((strcmp(prev_state,pub_data)) != 0){
		                                                strcpy(prev_state,pub_data);
																								       send_motorstatus();
																							   }
																							 }
																										  
                                                       }
                                                      }    
																						}
																		//timsec = 0;
																		if(task == mqtt_publish)
																				break;
																	
																		else 
                                       task =  mqtt_subscribe;
																	
                                    
                                    break;
                
						case mqtt_publish :       TIM_Cmd(TIM4, DISABLE); 
                                      time_up = 0;	
                                    	timsec = 0;		
                                      clear_RXBuffer();																		
							                        if(MQTTpublish(MQTTTopic1,"ON")){
							                        task =mqtt_subscribe;
					                             	}
																				else {
																					       HAL_Delay(500);
																					        TCP_init2(1);
																									   if(connect())
                                                        MQTTpublish(MQTTTopic1,"ON");
                                                        task = mqtt_subscribe;
                                                      }
																										
																				TIM_Cmd(TIM4, ENABLE); 
                break;
        }/* end of switch*/
        
        
        
    } /* end of while(1)*/
}

///////*******************************************************/////////////////////////////////////

void send_motorstatus(){
    uint8_t i = 0;
    uint8_t st_len = 0;
    uint8_t valves[2];
    
    	clear_RXBuffer();			
  
    //token = strtok(pub_data," ");
    valves[0] = pub_data[0];
	  valves[1] = pub_data[2];
    
 
    if(valves[0] == 0x31)
        v1_h();
    else
        v1_l();
    if(valves[1] == 0x31)
        v2_h();
    else
        v2_l();
    v1 = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10);
	  v2 = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12);
		if(v1==0)
			v1 = 1;
		else
			v1 = 0;
		
	  if(v2==0)
			v2 = 1;
		else
			v2 = 0;
		    sprintf(relay_buf,"%i_%i",v1,v2);
	     if( MQTTpublish(MQTTTopic4,relay_buf))
			 {
				cycle++;
				clear_RXBuffer();	
			 }
				 else {
					        HAL_Delay(500);
								  TCP_init2(1);
								  if(connect())
                  MQTTpublish(MQTTTopic4,relay_buf);
                  cycle++;
				          clear_RXBuffer();	
              }
						 
				memset(pub_data,'\0',sizeof(pub_data));	
        memset(relay_buf,0,sizeof(relay_buf));				 
}
uint8_t read_response3(char *command,  char *expected, char *expected2, uint32_t Timeout){
    uint8_t i=5;
    USARTSend(command);
    while(i>2){
			 
        HAL_Delay(Timeout);
        
        if (strstr(RX_BUF, expected) !=  NULL){
         
            return 1;
        }
        
        else{
            i--;
            HAL_Delay(2000);
            clear_RXBuffer();
            
        }
        // USARTSend(command);
    }
    return 0;
}
uint8_t read_response2(char *command, char *expected1, char *expected2, char *expected3, uint32_t Timeout){
     uint8_t i=5;
	 clear_RXBuffer();
    USARTSend(command);
    while(i>2){
        HAL_Delay(Timeout);
        if (strstr(RX_BUF, expected1) !=  NULL){
            memset(RX_BUF,0,sizeof(RX_BUF));
            clear_RXBuffer();
            return 1;
        }
        
       if (strstr(RX_BUF, expected2) !=  NULL){
            memset(RX_BUF,0,sizeof(RX_BUF));
            clear_RXBuffer();
            return 2;
        }
        // USARTSend(command);
	     if (strstr(RX_BUF, expected2) !=  NULL){
            memset(RX_BUF,0,sizeof(RX_BUF));
            clear_RXBuffer();
            return 3;
        }
			 else{
            i--;
            HAL_Delay(2000);
            clear_RXBuffer();
            
        }
    }
    return 0;
	} 
uint8_t read_response(char *command,  char *expected, char *expected2, uint32_t Timeout){
    uint8_t i=5;
    USARTSend(command);
    while(i){
        HAL_Delay(Timeout);
        if (strstr(RX_BUF, expected) !=  NULL){
            memset(RX_BUF,0,sizeof(RX_BUF));
            clear_RXBuffer();
            return 1;
        }
        
        else{
            i--;
            HAL_Delay(2000);
            clear_RXBuffer();
           
        }
        // USARTSend(command);
    }
    return 0;
}

uint8_t TCP_init2(uint8_t command){
uint8_t status = 0;
	while(command<3){
switch(command){	
	    
	   case 1:	if(read_response("AT+CIPSHUT\r\n", "SHUT OK\r\n", NULL, 2000))
            {
                command++;
            }
            else{
                command = 1;
            }
	           HAL_Delay(1000);
						break;
						
				    case 2:	status = read_response2("AT+CIPSTATUS\r\n","TCP CLOSED","IP INITIAL", NULL, 2000);
            if(status){
						if(status == 1){
                TCP = 0;
							  sprintf(aux_str,"AT+CIPSTART=\"TCP\",\"%s\",\"%s\"\r\n", MQTTHost, MQTTPort);
							  if(TCP_init(19))
							      command++;
								else
									  command--;
            }
						if(status == 2){
                TCP = 0;
							  if(TCP_init(11))
									 command++;
								else
									  command--;
            }
					}
					
           else{
							HAL_Delay(1000);
                command--;
            }
					  HAL_Delay(1000);
						break;
						
	
					}
}
	}
uint8_t TCP_init(uint8_t AT){
    
    int8_t COUNT=0;
    while (TCP<1){
    memset(RX_BUF,0,sizeof(RX_BUF));
        
        switch(AT){
                
            case 0: if(read_response("AT+CFUN=0\r\n", "OK\r\n","+CPIN: NOT READY\r\n", 1000))
            {
                
                if(read_response("AT+CFUN=1\r\n","+CPIN: READY\r\n",NULL ,2000)){
                    AT++;
                    HAL_Delay(7000);
                    clear_RXBuffer();
                }
                
            }
                break;
            case 1:  if(read_response("AT\r\n", "OK\r\n", NULL, 2000))
            {       AT++;
                
            }
            else
                AT =1;
                break;
                
            case 2:  if(read_response("AT+IPR=115200\r\n", "OK\r\n", NULL, 2000))
            {       AT++;
                
            }
            else
                AT = 1;
                
                break;
                
            case 3:	if(read_response("ATE0\r\n", "OK\r\n", NULL, 2000))
            {
                AT++;
            }
            else{
                AT--;
            }
                
                break;
                
            case 4:	if(read_response("AT+CIPSHUT\r\n", "SHUT OK\r\n", NULL, 2000))
            {
                AT++;
            }
            else{
                AT--;
            }
                
                break;
            case 5:	if(read_response2("AT+CREG?\r\n", "+CREG: 0,1","+CREG: 0,5",NULL, 2000))
            {
                AT=10;
                HAL_Delay(10);
            }
            else{
                AT=1;
            }
                break;
            case 6:	if(read_response("AT+CIPMUX=0\r\n","OK\r\n", NULL, 2000))
            {
                AT=8;
                HAL_Delay(1000);
            }
            else{
                AT--;
            }
                
                break;
                
            case 7:	if(read_response("AT+CIPRXGET=1\r\n", "OK\r\n", NULL, 2000))
                
            {
                AT++;
                HAL_Delay(10);
            }
            else{
                AT--;
            }
                break;
            case 8:	if(read_response("AT+CIPMODE=0\r\n","OK", NULL, 2000))
            {
                AT++;
                HAL_Delay(1000);
            }
            else{
                AT--;
            }
                break;
            case 9:	if(read_response("AT+CIPSRIP=0\r\n","OK", NULL, 2000))
            {
                AT++;
                memset(aux_str,0,sizeof(aux_str));
                HAL_Delay(1000);
            }
            else{
                AT--;
            }
                break;
            case 10:	if(read_response("AT+CGATT?\r\n","+CGATT: 1", NULL, 2000))
            {
                AT++;
                memset(aux_str,0,sizeof(aux_str));
                HAL_Delay(1000);
            }
            else{
                AT--;
            }
                break;
                
                
                
            case 11:	if(read_response("AT+CIPSTATUS\r\n","STATE: IP INITIAL", NULL, 2000))
            {
                AT++;
							
                sprintf(aux_str,"AT+CSTT=\"%s\",\"%s\",\"%s\"\r\n", APN_1,usrnm,password);
                HAL_Delay(1000);
            }
            else{
                AT = 6;
            }
                break;
						case 12:	network = read_response2("AT+CSPN?\r\n","Vodafone IN","airtel","IDEA", 2000);
            
              if(network){
							  if(network == 1)
                sprintf(aux_str,"AT+CSTT=\"%s\",\"%s\",\"%s\"\r\n", APN_1,usrnm,password);
								if(network == 2)
                sprintf(aux_str,"AT+CSTT=\"%s\",\"%s\",\"%s\"\r\n", APN_2,usrnm,password);
								if(network == 3)
                sprintf(aux_str,"AT+CSTT=\"%s\",\"%s\",\"%s\"\r\n", APN_3,usrnm,password);
                HAL_Delay(1000);
								   AT++;
							}
							else{
                  AT = 6;
                 }
                break;
                
         case 13:if(read_response(aux_str, "OK", NULL, 3000)){
                AT++;
                clear_RXBuffer();
                memset(aux_str,0,sizeof(aux_str));
                HAL_Delay(500);
            }
            else
            {
                AT=10;
            }
                break;
            case 14:	if(read_response("AT+CIPSTATUS\r\n","STATE: IP START", NULL, 2000))
            {
                AT++;
                HAL_Delay(1000);
            }
            else{
                AT = 10;
            }
                break;
            case 15:	 USARTSend("AT+CIICR\r\n");
                HAL_Delay(3000);
                if (strstr(RX_BUF,"OK") != NULL)
                {
                    AT++;
                    clear_RXBuffer();
                    HAL_Delay(500);
                }
                else{
                    if(COUNT!=3){
                        AT=13;
                        COUNT++;
                    }
                    else AT = 12;
                    
                    clear_RXBuffer();
                }
                break;
            case 16:	if(read_response("AT+CIPSTATUS\r\n","STATE: IP GPRSACT", NULL, 2000))
            {
                AT++;
                sprintf(aux_str,"AT+CSTT=\"%s\",\"%s\",\"%s\"\r\n", APN_1,usrnm,password);
                HAL_Delay(5000);
            }
            else{
                AT--;
            }
            case 17:	if(read_response("AT+CIFSR\r\n",".", NULL,3000))
            {
                AT++;
                HAL_Delay(1000);
            }
            else{
                AT--;
            }
                break;
            case 18:	if(read_response("AT+CIPSTATUS\r\n","STATE: IP STATUS", NULL, 2000))
            {
                AT++;
                sprintf(aux_str,"AT+CIPSTART=\"TCP\",\"%s\",\"%s\"\r\n", MQTTHost, MQTTPort);
                HAL_Delay(2000);
            }
            else{
                AT--;
            }
                
            case 19:	if(read_response2(aux_str, "OK\r\n\r\nCONNECT","CONNECT OK", NULL, 6000))
            {
                AT++;
               
                TCP = 1;
            }
            else
                AT = 1;
						
						   memset(aux_str,0,sizeof(aux_str));
						   HAL_Delay(1000);
                break;
        }
        
    }
 if(TCP == 1)
        return 1;
    return 0;
}
uint8_t connect(){
if (read_response("AT+CIPSEND\r\n", ">",NULL, 4000)){
    USARTSend2(0x10);
        
        MQTTProtocolNameLength = strlen(MQTTProtocolName);
        MQTTClientIDLength = strlen(MQTTClientID);
        MQTTUsernameLength = strlen(MQTTUsername);
        MQTTPasswordLength = strlen(MQTTPassword);
        datalength = MQTTProtocolNameLength + 2 + 4 + MQTTClientIDLength + 2 + MQTTUsernameLength + 2 + MQTTPasswordLength + 2;
        
			X = datalength;
        do
        {
            encodedByte = X % 128;
            X = X / 128;
            // if there are more data to encode, set the top bit of this byte
            if ( X > 0 ) {
                encodedByte |= 128;
            }
            
            USARTSend2(encodedByte);
        }
        while ( X > 0 );
        USARTSend2(MQTTProtocolNameLength >> 8);
        USARTSend2(MQTTProtocolNameLength & 0xFF);
        
        USARTSend(MQTTProtocolName);
        
        USARTSend2(MQTTLVL); // LVL
        USARTSend2(MQTTFlags); // Flags
        USARTSend2(MQTTKeepAlive>>8);
        USARTSend2(MQTTKeepAlive & 0xFF);
        
        
        USARTSend2(MQTTClientIDLength >> 8);
        USARTSend2(MQTTClientIDLength & 0xFF);
        USARTSend(MQTTClientID);
        
        
        USARTSend2(MQTTUsernameLength >> 8);
        USARTSend2(MQTTUsernameLength & 0xFF);
        USARTSend(MQTTUsername);
        
        
        
        USARTSend2(MQTTPasswordLength >> 8);
        USARTSend2(MQTTPasswordLength & 0xFF);
        USARTSend(MQTTPassword);
        
        
        USARTSend2(0x1A);
        if (read_response("\r\n", "SEND OK",NULL, 4000))
            
            
        return 1;
    }
else{
	      TCP = 0;
        TCP_init2(1);
            //if(read_response2("AT+CIPSTATUS\r\n",".STATE: CONNECT OK", "STATE: TCP CLOSED", 2000))
          connect();
    }
    return 0;
}
/**************************************************************************************************************/

uint8_t  MQTTpublish(char *MQTTTopic, char *data){
    
    if (read_response("AT+CIPSEND\r\n", ">",NULL, 3000)){
        
        memset(str, 0, sizeof(str));
        
        topiclength = sprintf(topic,"%s",MQTTTopic);
        datalength = sprintf(str,"%s%s", topic,data);
        //DelayUs(1000);
       // HAL_Delay(1000);
        
        USARTSend2(0x30);
        X = datalength + 2;
        do
        {
            encodedByte = X % 128;
            X = X / 128;
            // if there are more data to encode, set the top bit of this byte
            if ( X > 0 ) {
                encodedByte |= 128;
            }
            
            USARTSend2(encodedByte);
        }
        while ( X > 0 );
        
        USARTSend2(topiclength >> 8);
        USARTSend2(topiclength & 0xFF);
        USARTSend(str);
        USARTSend2(0x1A);
        if(read_response("\r\n", "SEND OK", NULL, 4000)) 
        return 1;
				
    }
    
    return 0;
    
    
}
/**************************************************************************************************************/
uint8_t MQTTsubscribe(){
    
    if (read_response("AT+CIPSEND\r\n", ">", NULL, 2000)) {
        
        memset(str2, 0, sizeof(str2));
        topiclength2 = strlen(MQTTTopic2);
        
        datalength = 2 + 2 + topiclength2 + 1;
        
        USARTSend2(0x82);
        X = datalength;
        do
        {
            encodedByte = X % 128;
            X = X / 128;
            // if there are more data to encode, set the top bit of this byte
            if ( X > 0 ) {
                encodedByte |= 128;
            }
            USARTSend2(encodedByte);
        }
        while ( X > 0 );
        
        USARTSend2(MQTTPacketID >> 8);  // Send MSB first
        USARTSend2(MQTTPacketID & 0xFF); // then send LSB
        USARTSend2(topiclength2 >> 8);
        USARTSend2(topiclength2 & 0xFF);
        USARTSend(MQTTTopic2);
        USARTSend2(MQTTQOS);
        USARTSend2(0x1A);
        if (read_response3("\r\n", "SEND OK", NULL, 4000)) {
            SUB = 1;
            return 1;
        }
    }
    
    return 0;
    
}
/*
void mqtt_publish_message(uint8_t * mqtt_message, char * topic, char * message) {

        uint8_t i = 0;
        uint8_t topic_length = strlen(topic);
        uint8_t message_length = strlen(message);

		mqtt_message[0] = 48;                                  // MQTT Message Type CONNECT
		mqtt_message[1] = 2 + topic_length + message_length;   // Remaining length
		mqtt_message[2] = 0;                                   // MQTT Message Type CONNECT
		mqtt_message[3] = topic_length;                        // MQTT Message Type CONNECT

        // Topic
        for(i = 0; i < topic_length; i++){
            mqtt_message[4 + i] = topic[i];
        }

        // Message
        for(i = 0; i < message_length; i++){
            mqtt_message[4 + topic_length + i] = message[i];
        }

	}
***/////////////////////////////////
