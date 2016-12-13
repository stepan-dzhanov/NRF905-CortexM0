#ifndef __COOCK__
#define __COOCK__

#define COOCK_SCRIPT_RUN  1
#define COOCK_SCRIPT_END  0



#define HYSTERESYS        10

void InitCoock(int *temperature_set, int *time_set, int steps_set);
int CoockScripts();
int  Termostat(int a);
char GetHeaterState();
int GetCookStatus();
#endif