

#include "timer.h"

static int time=0;
void SetTimer(int usec) {
time  =  usec;
}

void Timer(){
if(time>0) time--;
}

int GetTimer()  {
  return time;
}