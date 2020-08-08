#ifndef __EXT_IRQ__
#define __EXT_IRQ__

__weak void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
__weak void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp);

char GetDoorSensorState() ;
char GetButtonState() ;
int GetSensorCounter();
void ResetSensorCounter() ;

#endif
