
#include "stm32f0xx_hal.h"
#include "ext_irq.h"
#include "rf_protocol.h"
#include "nrf905_driver.h"
#include "main.h"
static char state =0;
static char bstate =0;

__weak void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

  switch (GPIO_Pin){
    case DATA_READY_Pin:
    if(!GetTxStatus()) RF_Receive();  
      else rTxStatus();
      
      break;
    case SWITCH_Pin:
      state =1;
      break;
    case Button_Pin:
      bstate =1;
      break;
  }
}


char GetDoorSensorState()       {
  char _state;
  _state = state;
  state =0;
  return _state;
}
char GetButtonState()        {
  char _state;
  _state = bstate;
  bstate = 0;
  return _state;
}
