#include "stm32f0xx_hal.h"
#include <string.h>
#include "timer.h"
#include "host_protocol.h"
#define MAX_COMMAND_HOST  7

const char * hstrings[16] = {
"tst",
"wdl",
"ch1on",
"ch1off",
"ch2on",
"ch2off",
"pwm",
};

void HostCommandParcer (char *packet, char *str){
   char i=0;
   for( i=0; i<(MAX_COMMAND_HOST);i++)  {
    if(!memcmp(packet,hstrings[i],strlen(hstrings[i]))) break;
      
    }
   if (i==MAX_COMMAND_HOST) {
      sprintf(str,"Invalid command%c               \n", ADDR);
      return;
   }
   
   
  
    switch (i){
    case 0:
      sprintf(str,"OK%c                           \n", ADDR);      // Test communication
      break;
      
    case 1:                                // Water delivery
      sprintf(str,"OK%c                            \n", ADDR);
      HAL_GPIO_WritePin( CH1_GPIO_Port, CH1_Pin, GPIO_PIN_SET);
      SetTimer(1000*(packet[3]*10 +packet[4] ));
      HAL_GPIO_WritePin( CH1_GPIO_Port, CH1_Pin, GPIO_PIN_RESET); 
      break;
     case 2:                                // ch1 ON
      sprintf(str,"OK%c                            \n", ADDR);
      HAL_GPIO_WritePin( CH1_GPIO_Port, CH1_Pin, GPIO_PIN_SET);
      break;
     case 3:                                // ch1 OFF
      sprintf(str,"OK%c                            \n", ADDR);
      HAL_GPIO_WritePin( CH1_GPIO_Port, CH1_Pin, GPIO_PIN_RESET);
      break;
     case 4:                                // ch2 ON
      sprintf(str,"OK%c                            \n", ADDR);
      HAL_GPIO_WritePin( CH2_GPIO_Port, CH2_Pin, GPIO_PIN_SET);
      break;
     case 5:                                // ch2 OFF
      sprintf(str,"OK%c                            \n", ADDR);
      HAL_GPIO_WritePin( CH2_GPIO_Port, CH2_Pin, GPIO_PIN_RESET);
      break;
     case 6:                                // pwm
      sprintf(str,"OK%c                            \n", ADDR);
      ;
      break;
      
      
    
    default:
      sprintf(str,"Invalid command%c               \n", ADDR);
      ;
      break;
      
        
    }
}