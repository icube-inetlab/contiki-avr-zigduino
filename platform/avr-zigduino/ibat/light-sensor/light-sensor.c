#include "contiki.h"
#include <stdio.h>
#include "dev/adc.h"

#define A1 15

/*---------------------------------------------------------------*/
PROCESS(light_sensor,"Read light sensor process");
AUTOSTART_PROCESSES(&light_sensor);
/*---------------------------------------------------------------*/
PROCESS_THREAD(light_sensor,ev,data)
{
	static struct etimer timer;
    static int counter=0;
    PROCESS_BEGIN();

    printf("Start loop\n");
    etimer_set(&timer, CLOCK_CONF_SECOND);
    while(1) {
      PROCESS_WAIT_EVENT();
      if(ev == PROCESS_EVENT_TIMER) {
		    int sensor_value = readADC(A1);
	    	printf("counter:%d light:%d\n", counter, sensor_value);
	    	counter++;
        etimer_reset(&timer);
			}
    }
    PROCESS_END();
}
/*---------------------------------------------------------------*/
