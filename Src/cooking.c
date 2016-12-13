
#include "stm32f0xx_hal.h"
#include "cooking.h"
#include "timer.h"
#include "adc.h"

static int co_time [10];
static int co_temperature [10];
static int co_steps;
static int co_steps_set;
static int co_status = COOCK_SCRIPT_END;
static char ready_flg =0;
static char a = 0;
static char run = 0;
static char hstatus = 'F';
static char detailed_cook_status = 'S';


#define SET_HEATER HAL_GPIO_WritePin( CH1_GPIO_Port, CH1_Pin, GPIO_PIN_SET);
#define RESET_HEATER HAL_GPIO_WritePin( CH1_GPIO_Port, CH1_Pin, GPIO_PIN_RESET);

#define START_BEEP 300
#define STOP_BEEP  1500
#define RUN_BEEP   800

void Beep(int timeout)  {
;
}

char GetHeaterState(){
  return hstatus;
  
}

int GetCookStatus() {
  return detailed_cook_status;//co_status;

}
  

int  Termostat(int a){
  int temp;
  float tmpf;
  if(a == 0)  {
    RESET_HEATER
     hstatus='F';
    return 0;
  }
  temp = GetBatteryStatus();
  tmpf = ( ((float)temp)*0.847  );
  temp = (int)tmpf;
  if(temp<=0) temp = 0; // TO DO Error turn on device!!! 
  if ((a-temp)>HYSTERESYS)  {
    SET_HEATER
     hstatus='O';
    return 0;
  }
  if(a<=temp) {
    RESET_HEATER
     hstatus='F';
    return 1;
  }
  
  return 0;
  
}

void InitCoock(int *temperature_set, int *time_set, int steps_set)  {
  int i;
  co_steps = steps_set;
  co_steps_set = steps_set;
  for (i = 0; i < co_steps; i++)  {
    co_time [i] = time_set[i];
    co_temperature [i] = temperature_set[i];
    
  } 
  co_status = COOCK_SCRIPT_RUN;
  ready_flg = 0;
  a = 0;
  Beep(START_BEEP);
  detailed_cook_status = 'R';
}

int CoockScripts()  {
  int time;
  
  int current_temperature;
  
  if(co_status == COOCK_SCRIPT_END ) return COOCK_SCRIPT_END;
  
  if ( (run ==0)) {
    current_temperature = co_temperature [co_steps - 1];
    SetCoockTimer(co_time[co_steps - 1]);
    run = 1;
    return COOCK_SCRIPT_RUN;
  }
  if ( (run ==1)) {
     current_temperature = co_temperature [co_steps - 1];
     if((co_steps == co_steps_set)&&(ready_flg == 0)) {
       SetCoockTimer(co_time[co_steps - 1]);       
     }
    time = GetCoockTimer();
    if(time>0)  {
      if (Termostat(current_temperature)==1) {
        ready_flg =1;
        if(!a){
          Beep(RUN_BEEP);
          detailed_cook_status = 'G';
          a = 1;
        }
      }
    }
    if (time==0) {
      Termostat(0);
      if(co_steps>1) {
        co_steps--;
        run = 0;
        return COOCK_SCRIPT_RUN;
      }
      if(co_steps==1){
        co_status = COOCK_SCRIPT_END;
        Termostat(0);
        run = 0;
        Beep(STOP_BEEP);
        detailed_cook_status='S';
        return COOCK_SCRIPT_END;
        
      }
      
    }
    
  }
  return COOCK_SCRIPT_RUN;
  
  
  
}