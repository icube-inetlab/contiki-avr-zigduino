/*
 * Example code for SHT1x or SHT7x sensors demonstrating blocking calls
 * for temperature and humidity measurement in the setup routine and
 * non-blocking calls in the main loop.  The pin 13 LED is flashed as a
 * background task while temperature and humidity measurements are made.
 * Note that the status register read/write demonstration code places the
 * sensor in low resolution mode.  Delete it to stay in high res mode.
 *
 * This example contains two versions of the code: one that checks library
 * function return codes for error indications and one that does not.
 * The version with error checking may be useful in debuging possible
 * connection issues with the sensor.  A #define selects between versions.
 */


#include "contiki.h"
#include <stdio.h> /* For printf() */
#include "Arduino.h"
#include <Sensirion.h>

const byte dataPin =  7;                 // SHTxx serial data
const byte sclkPin =  6;                 // SHTxx serial clock

/*---------------------------------------------------------------------------*/
PROCESS(shtSensirion_process, "shtSensirion");
AUTOSTART_PROCESSES(&shtSensirion_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(shtSensirion_process, ev, data)
{

    static struct etimer timer;
    static int counter=0;
    
    unsigned int rawData;
    float temperature;

    PROCESS_BEGIN();

    //PROCESS_PAUSE();
    unsigned long curMillis = millis();          // Get current time
    Sensirion sht = Sensirion(dataPin, sclkPin);

   
    printf("Start loop\n");    
   // etimer_set(&timer, CLOCK_CONF_SECOND*4);

     while(1) {
     //   PROCESS_WAIT_EVENT();
     //   if(ev == PROCESS_EVENT_TIMER)
     // Demonstrate blocking calls
       sht.measTemp(&rawData);                // sht.meas(TEMP, &rawData, BLOCK)
       temperature = sht.calcTemp(rawData);


        
    }
    PROCESS_END();
}
/*---------------------------------------------------------------------------*/
