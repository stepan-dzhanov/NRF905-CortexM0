
#include "stm32f0xx_hal.h"
#include "ext_irq.h"
#include "rf_protocol.h"
#include "nrf905_driver.h"

__weak void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

  switch (GPIO_Pin){
    case DATA_READY_Pin:
    if(!GetTxStatus()) RF_Receive();  
      else rTxStatus();
      
      break;
  }
}