#include "uart.h" 


// Transmit a character over USART1 A9/TX
void USART1_PutChar(uint16_t Data) {
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
   // USART1->DR = (Data & 0xFF);
	  //USART1->DR = (Data & (uint16_t)0x01FF);
   
	  // while (!USART_GetFlagStatus(USART1, USART_FLAG_TXE)){}
	   // Send a char using USART2
	   USART_SendData(USART1, Data);
     
}
 
// Receive a character over USART1 A10/RX
int get_char_usart1(){
 
    // this while loop makes the call blocking
    // will wait until byte received
    // while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);
    // return USART1->DR & 0xFF;
 
    if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) != RESET){
        return USART1->DR & 0xFF;
    } else {
        return -1;
    }
}
 
// Transmit a string over USART1 A9/TX                  
void USART1_PutString(char *s)
{
	// Send a string
	while (*s)
	{
		USART1_PutChar(*s);
		s++;
	}
	
}
//

void num(int x){
static uint8_t n = 0;
static uint8_t m = 0;
if(x>9){
n=x/10;
USART1_PutChar((n+48));	
m=x%10;
USART1_PutChar((m+48));
	m=0;n=0;
}	
else{
	USART1_PutChar((x+48));
    }
	USART1_PutString("\r\n");	
}
