#include "stm32f0xx_hal.h"
#include <string.h>
#include "timer.h"
#include "host_protocol.h"
#include "cooking.h"

//#define __DBG__
char db =0;
#define MAX_COMMAND_HOST  8

const char * hstrings[16] = {
"tst",
"wdl",
"ch1on",
"ch1off",
"ch2on",
"ch2off",
"pwm",
"scp",
};

unsigned int  StrToInt ( char *pData)  {
  unsigned int result;
  result = (pData[0]&0x0f)*100 +(pData[1]&0x0f)*10+ (pData[2]&0x0F);
  return result;
}



void HostCommandParcer (char *packet, char *str){
  unsigned int temperature[3];
  unsigned int time[3];

  char i=0;
   for( i=0; i<(MAX_COMMAND_HOST);i++)  {
    if(!memcmp(packet,hstrings[i],strlen(hstrings[i]))) break;
      
    }
   if (i==MAX_COMMAND_HOST) {
      sprintf(str,"Invalid command%c               \n", ADDR);
      return;
   }
   
#ifdef __DBG__

   if (db==0) i = 7;
   db =1;
#endif
  
    switch (i){
    case 0:
      sprintf(str,"OK%c\n", ADDR);      // Test communication
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
     case 7:                                // scp
      temperature[0] = StrToInt(packet+3);
      time[0] = (StrToInt(packet+6))*60;
      temperature[1] = StrToInt(packet+9);
      time[1] = (StrToInt(packet+12))*60;
#ifdef __DBG__
 temperature[0]= 100;
 temperature[1]= 100;
 time[0] = 60;
 time[1] = 60;
#endif 
      InitCoock(temperature, time,2);
      sprintf(str,"OK%c                            \n", ADDR);
      ;
      break;
      
      
    
    default:
      sprintf(str,"Invalid command%c               \n", ADDR);
      ;
      break;
      
        
    }
}