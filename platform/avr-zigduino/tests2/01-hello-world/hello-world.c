/*
 * Copyright (c) 2011-2012 SmeshLink Technology Corporation.
 * All rights reserved.
 *
 * $Id: hello-world.c $
 */

/**
 * \file
 *         A very simple Contiki application showing how Contiki programs look.
 * \author
 *         SmeshLink
 */

#include "contiki.h"
#include "dev/leds.h"
#include <string.h>
#include <stdio.h>
#include <util/delay.h>

//#include "avr/clock.h"
#include "Arduino.h"

//#include "mx-shell.h"
#include "dev/rs232.h"

/*---------------------------------------------------------------------------*/
PROCESS(hello_world_process, "Hello world process");
PROCESS(leds_blink_process, "LEDs blink process");
AUTOSTART_PROCESSES(&hello_world_process,&leds_blink_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(hello_world_process, ev, data)
{
	/* Variables are declared static to ensure their values are kept between kernel calls. */
	static struct etimer timer;
	static int count = 0;
	//_delay_ms(50);
	
	/* Any process must start with this. */
	PROCESS_BEGIN();

	//mx_shell_init(RS232_PORT_0);

	/* Set the etimer to generate an event in one second. */
	etimer_set(&timer, CLOCK_CONF_SECOND);

	while(1) {
		/* Wait for an event. */
		PROCESS_WAIT_EVENT();

    /* Got the timer's event~ */
    if (ev == PROCESS_EVENT_TIMER) {
      printf("Hello, world #%i\n", count);
      count++;

			/* Reset the etimer so it will generate another event after the exact same time. */
			etimer_reset(&timer);
		}
	} // while (1)

	/* Any process must end with this, even if it is never reached. */
	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(leds_blink_process, ev, data)
{
	static struct etimer timer;
	static uint8_t leds_state = 0;

	/* Any process must start with this. */
	PROCESS_BEGIN();
        pinMode(6,OUTPUT);
        pinMode(7,OUTPUT);
        digitalWrite(7,LOW);
        digitalWrite(6,LOW);
	while(1) {
		/* Set the etimer every time. */
		etimer_set(&timer, CLOCK_CONF_SECOND/4);
		/* And wait until the specific event. */
		PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER);

		/* Change the state of leds. */
		//leds_off(LEDS_ALL);
  		digitalWrite(6,HIGH);
		_delay_us(10);
		//delayMicroseconds(1);
		//clock_delay_usec(1);
		//unsigned j;
		//for (j = 50; j > 0; j--) asm volatile("nop");
		digitalWrite(6,LOW);
                if(leds_state==0) {
		   digitalWrite(7,HIGH);
                   leds_state=1;
	        }
                else {
	           digitalWrite(7,LOW);
                   leds_state=0;
		}
#ifndef CONF_LOWPOWER
		//leds_on(leds_state);
               
		//leds_state += 1;
#endif
	}

	/* Any process must end with this, even if it is never reached. */
	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
