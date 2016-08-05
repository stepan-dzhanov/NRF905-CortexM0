#include "stm32f0xx_hal.h"
#include "rf_protocol.h"
#include "nrf905_driver.h"

static char ptr=0;
static char rx_buff[64];
static char tmp;

extern UART_HandleTypeDef huart2;

void RF_Receive()       {
  char i;
  tmp = ReceiveMultiPacket(rx_buff,32);
  rx_buff[32] = 0x0A;
  rx_buff[33] = 0x0D;
  HAL_UART_Transmit(&huart2,rx_buff, 34, 1000);
  
  //rx_buff[ptr++] 
  
  
}