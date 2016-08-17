#include "stm32f0xx_hal.h"
#include "rf_protocol.h"
#include "nrf905_driver.h"

static char ptr=0;
static char rx_buff[64];
static char data_ready = 0;

extern UART_HandleTypeDef huart2;

char GetDataFromAir(char *rx_data){
  if (!data_ready) return 0;
  memcpy(rx_data,rx_buff,32);
  data_ready =0;
  return 1;
}

void RF_Receive()       {
  char i;
  data_ready = ReceiveMultiPacket(rx_buff,32);
 
  //rx_buff[ptr++] 
  
  
}