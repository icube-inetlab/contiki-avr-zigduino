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
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND_rl
all: $(CONTIKI_PROJECT)
TARGET = avr-zigduino
CONTIKI = ../../../..
UIP_CONF_IPV6 = 1

include $(CONTIKI)/Makefile.include
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
#include "Arduino.h"
#include "rl_sht11.h"

//int temperatureCommand  = 0B00000011;  // command used to read temperature
//int humidityCommand = 0B00000101;  // command used to read humidity

//int clockPin = 6;  // pin used for clock
//int dataPin  = 7;  // pin used for data
//int ack;  // track acknowledgment for errors
// int val;


/*---------------------------------------------------------------------------*/
PROCESS(shtTest_process, "shtTest process");
AUTOSTART_PROCESSES(&shtTest_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(shtTest_process, ev, data)
{
    PROCESS_BEGIN();
    PROCESS_PAUSE();

   //Sensirion(8,9);
    pinMode(5,OUTPUT);
    digitalWrite(5, HIGH);
    printf("Hello, world... shtTest ! \n");
    sht11_temp();
  
    
  // turn the LED on when we're done
    digitalWrite(5, LOW); 

    PROCESS_END();
}
/*---------------------------------------------------------------------------*/
