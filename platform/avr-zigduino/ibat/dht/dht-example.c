#include "contiki.h"
#include <stdio.h>
#include "dev/dht.h"

#define A0 14

/*---------------------------------------------------------------*/
PROCESS(temp_hum_sensor,"Read Temperature and Humidity sensor process");
AUTOSTART_PROCESSES(&temp_hum_sensor);
/*---------------------------------------------------------------*/
PROCESS_THREAD(temp_hum_sensor,ev,data)
{
	static struct etimer timer;
  static int counter=0;
  PROCESS_BEGIN();
  printf("Start loop\n");
  etimer_set(&timer, CLOCK_CONF_SECOND);
  while(1) {
    PROCESS_WAIT_EVENT();
    if(ev == PROCESS_EVENT_TIMER) {
      dht_read(A0);
			printf("counter:%d ", counter);
      print_temp_hum(&DHT);
      printf("\n");
    	counter++;
      etimer_reset(&timer);
		}
  }
  PROCESS_END();
}
/*---------------------------------------------------------------*/
