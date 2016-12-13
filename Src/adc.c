#include "stm32f0xx_hal.h"
#include "adc.h"

extern ADC_HandleTypeDef hadc;

unsigned char GetBatteryStatus(){
  
  
  volatile unsigned int adcResult;
  unsigned char result;
  
  ADC_ChannelConfTypeDef sConfig;
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1 ;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  HAL_ADC_ConfigChannel(&hadc, &sConfig);
  
  
  HAL_ADC_Start(&hadc);
  HAL_ADC_PollForConversion(&hadc, 100);
  adcResult = HAL_ADC_GetValue(&hadc);
  HAL_ADC_Stop(&hadc);
  result = (char)(adcResult);
  return result;
}

unsigned int GetExtVoltage(){
  
  volatile unsigned int adcResult;
  unsigned char result;
  ADC_ChannelConfTypeDef sConfig;
  sConfig.Channel = ADC_CHANNEL_17;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  HAL_ADC_ConfigChannel(&hadc, &sConfig);
  
  
  HAL_ADC_Start(&hadc);
  HAL_ADC_PollForConversion(&hadc, 100);
  adcResult = HAL_ADC_GetValue(&hadc);
  HAL_ADC_Stop(&hadc);
  
  return adcResult;
}


  