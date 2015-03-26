/*
 * Copyright (c) 2006, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 * $Id: hello-world.c,v 1.1 2006/10/02 21:46:46 adamdunkels Exp $
 */

/**
 * \file
 *         A very simple Contiki application showing how Contiki programs look
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

#include "contiki.h"

#include <stdio.h> /* For printf() */
#include "dev/adc.h"
#include <util/delay.h>
#include "Arduino.h"


//#include <EEPROM.h>
//#include "SHT1x.h"
//#include "Sensirion.h"
#include <Sensirion.h>
// Specify data and clock connections and instantiate SHT1x object

#define A0 14
#define A1 15
#define A2 16
#define A3 17

//#define dataPin  7
//#define clockPin 6

const uint8_t dataPin  =  7;
const uint8_t clockPin =  6;

float temperature;
float humidity;
float dewpoint;





/*---------------------------------------------------------------------------*/
PROCESS(sensors_process, "Read sensor process");
AUTOSTART_PROCESSES(&sensors_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(sensors_process, ev, data)
{

    static struct etimer timer;
    static int counter=0;
    

    PROCESS_BEGIN();
    //SHT1x sht1x(dataPin, clockPin);

    Sensirion tempSensor = Sensirion(dataPin, clockPin);

    int led1 = 5;
    int led2 = 2;
    int led3 = 3 ; 
    int bouton = 4;
    int etatBouton;

    pinMode(led1,OUTPUT);
    pinMode(led2,OUTPUT);
    pinMode(led3,OUTPUT);
    pinMode(bouton,INPUT);

    printf("Start loop\n");    
    etimer_set(&timer, CLOCK_CONF_SECOND);
    while(1) {
        PROCESS_WAIT_EVENT();
        if(ev == PROCESS_EVENT_TIMER)
        { 
	    int a0 = readADC(A0);
	    int a1 = readADC(A1);
            //  digitalWrite(led,HIGH);
	    printf("Read Sensor A0: %d\n", a0);
	    printf("Read Sensor A1: %d\n", a1);
	    _delay_ms(50);
	    tempSensor.measure(&temperature, &humidity, &dewpoint);

           etatBouton = digitalRead(bouton);
           if(etatBouton == 1){
	   	digitalWrite(led1,HIGH);
	   	digitalWrite(led2,HIGH);
           	digitalWrite(led3,HIGH);
	   }
	   else {
		digitalWrite(led1,LOW);
	   	digitalWrite(led2,LOW);
           	digitalWrite(led3,LOW);	

	   }

	   for (int i = 0; i < 512; i++)
               EEPROM.write(i, 0);

	   // delay(100);
	   // digitalWrite(led,LOW);
	   // delay(100);
  	   /*
	    if (a1 >= 1023)
		digitalWrite(led,LOW);
	    else
	    	digitalWrite(led,HIGH);
	   */

	    printf("Counter: %d\n", counter);
	    counter++;
            etimer_reset(&timer);
	}
    }

    PROCESS_END();
}
/*---------------------------------------------------------------------------*/
