

#include "timer.h"

static int time=0;
static int c_time=0;
static int counter =0;
#define COOCK_TIMER_TIMEOUT 1000


void SetCoockTimer (int sec)    {
   c_time = sec;
}

int GetCoockTimer()  {
  return c_time;
}



void SetTimer(int usec) {
time  =  usec;
}

void Timer(){
  if(time>0) time--;
  counter++;
  if (counter>=COOCK_TIMER_TIMEOUT){
    if(c_time>0) c_time--;
    counter =0;
  }
}

int GetTimer()  {
  return time;
}