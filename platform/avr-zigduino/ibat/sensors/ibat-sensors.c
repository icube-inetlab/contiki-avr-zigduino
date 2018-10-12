#include "contiki.h"
#include <stdio.h>
#include "dev/adc.h"
#include "dev/dht.h"
#include "dev/Arduino.h"

#define A0 14
#define A1 15
#define A2 16
#define D4 4

// [IBAT];hum=43.50;temp=27.60;light=1023;sound=7;pir=1

/*---------------------------------------------------------------*/
PROCESS(loudness_sensor,"Read loudness sensor process");
AUTOSTART_PROCESSES(&loudness_sensor);
/*---------------------------------------------------------------*/
PROCESS_THREAD(loudness_sensor,ev,data)
{
	static struct etimer timer;
  PROCESS_BEGIN();
	int light, sound, pir;
	struct dht SHT22;

  printf("Start loop\n");
  etimer_set(&timer, CLOCK_CONF_SECOND);
  while(1) {
    PROCESS_WAIT_EVENT();
    if(ev == PROCESS_EVENT_TIMER) {
			light = readADC(A1);
	    sound = readADC(A2);
			pir = digitalRead(D4);
			if (dht_read(&SHT22, A0) == 0){
					printf("[IBAT];hum=%u.%u;temp=%u.%u;light=%d;sound=%d;pir=%d\n",
							 SHT22.humidity_h, SHT22.humidity_l, SHT22.temp_h, SHT22.temp_l,
							 light, sound, pir);
			}
			etimer_reset(&timer);
		}
  }
  PROCESS_END();
}
/*---------------------------------------------------------------*/
