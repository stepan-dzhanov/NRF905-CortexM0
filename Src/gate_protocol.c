

#include "stm32f0xx_hal.h"
#include "gate_protocol.h"
#include "nrf905_driver.h"
#include <string.h>

extern UART_HandleTypeDef huart2;

static unsigned char RxData[128];
static   uint8_t point =0;
char str[32];
char data_ready = 0;


#define MAX_COMMAND  3

const char * strings[16] = {
"--",
"?",
"SET_CH",
"",
};

char CommandParcer (char *packet){
   char i=0;
   for( i=1; i<=(MAX_COMMAND);i++)  {
    if(!memcmp(packet,strings[i],strlen(strings[i]))) break;
      
    }
   if (i==MAX_COMMAND) return 0;
   else return i;
    
}


void CommandProcessor(char command) {
  char str[32];
  
  switch (command){
  case 1:
    sprintf(str,"OK\n\r");      // Test communication
    HAL_UART_Transmit(&huart2,str, strlen(str), 1000);
    break;
  case 2:
    Nrf905Init(RxData[6]);    // Channel number
    sprintf(str,"OK\n\r");
    HAL_UART_Transmit(&huart2,str, strlen(str), 1000);
    break;
  default:
    sprintf(str,"Invalid command\n\r");
    HAL_UART_Transmit(&huart2,str, strlen(str), 1000);
    break;
    
      
  }
}


char GetDataFromHost(char *buff)  {
  if (!data_ready) return 0;
  for (char i=0; i<32; i++) buff[i] = RxData[i];//memcpy(RxData, buff,32);
  return 1;
}
void ClearDataFromHost()  {
  data_ready = 0;
  point =0;
}

void Gate_Receive(){
  uint8_t b;
  
  if (data_ready) return;
  RxData[point++] = (uint8_t) (huart2.Instance->RDR & (uint16_t) 0x00ff); 
  if ( (point>=2)&&(RxData[point-2] == 0x0A)&&((RxData[point-1] == 0x0D)) ) {
    b = CommandParcer (RxData);
    if (!b){
      data_ready =1;
      point =0;
    }
      else {
        CommandProcessor(b); 
        data_ready = 0;
        point =0;
      }
    
    
  }
  if (point>=128) point =0;  
 

  
  
}