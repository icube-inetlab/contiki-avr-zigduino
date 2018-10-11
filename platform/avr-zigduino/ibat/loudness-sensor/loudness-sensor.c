#include "contiki.h"
#include <stdio.h>
#include "dev/adc.h"

#define A2 16

/*---------------------------------------------------------------*/
PROCESS(loudness_sensor,"Read loudness sensor process");
AUTOSTART_PROCESSES(&loudness_sensor);
/*---------------------------------------------------------------*/
PROCESS_THREAD(loudness_sensor,ev,data)
{
	static struct etimer timer;
  static int counter=0;
  PROCESS_BEGIN();

  printf("Start loop\n");
  etimer_set(&timer, CLOCK_CONF_SECOND);
  while(1) {
    PROCESS_WAIT_EVENT();
    if(ev == PROCESS_EVENT_TIMER) {
	    int a2 = readADC(A2);
	    printf("counter:%d loudness:%d\n", counter, a2);
	    counter++;
			etimer_reset(&timer);
		}
  }
  PROCESS_END();
}
/*---------------------------------------------------------------*/
