#include "contiki.h"
#include <stdio.h>
#include "dev/adc.h"
#include "dev/Arduino.h"

#define D4 4

/*---------------------------------------------------------------*/
PROCESS(motion_sensor,"Read motion sensor process");
AUTOSTART_PROCESSES(&motion_sensor);
/*---------------------------------------------------------------*/
PROCESS_THREAD(motion_sensor,ev,data)
{
	static struct etimer timer;
  static int counter=0;
  PROCESS_BEGIN();

  printf("Start loop\n");
  etimer_set(&timer, CLOCK_CONF_SECOND);
  while(1) {
    PROCESS_WAIT_EVENT();
    if(ev == PROCESS_EVENT_TIMER)
    {
      int sensor_value = digitalRead(D4);
      printf("counter:%d motion:%d\n", counter, sensor_value);
    	counter++;
      etimer_reset(&timer);
		}
  }
  PROCESS_END();
}
/*---------------------------------------------------------------*/
