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
#include "net/rime.h"
#include "random.h"
#include "lib/memb.h"
#include "dev/leds.h"
#include "net/netstack.h"
#include <stdlib.h>
#include <stdio.h> /* For printf() */
#include "radio/rf230bb/rf230bb.h"
#include "rf230bb.h"

#include "net/mac/frame802154.h"
#include "net/mac/framer-802154.h"

#include "dev/serial-line.h"

// nb packets per test
// MAX is NB_PACKETS * 100 < 2^16/2 = 32768
#define NB_PACKETS 25

// message payload with 20 bytes
static uint8_t msg[20];
static uint8_t packet_len = 21;
static uint8_t counter = 0;
static uint8_t results_loss[NB_PACKETS];
static int8_t results_rssi[NB_PACKETS];
static int sum_loss=0;
static int16_t sum_rssi=0;
static volatile int start=0;

static int8_t rssi = 0;
static uint8_t seq_nb = 0;

static rimeaddr_t addr_master;
static rimeaddr_t addr_slave;

uint8_t p = 0;
uint8_t tab_power[5] = {0, 6, 11, 14, 15};  //+3, 0, -5 , -12, -17

#define MASTER 1
static struct unicast_conn unicast;

uint8_t mode;

static void init_results_arrays()
{
    int i = 0;
    for (i=0; i <NB_PACKETS; i++) {
       results_loss[i]=0;
    }

    for (i=0; i <NB_PACKETS; i++) {
       results_rssi[i]=0;
    }

}


/*---------------------------------------------------------------------------*/
PROCESS(hello_world_process, "Hello world process");
AUTOSTART_PROCESSES(&hello_world_process);
/*---------------------------------------------------------------------------*/
static void
recv_uc(struct unicast_conn *c, const rimeaddr_t *from)
{
   ///printf("hello\n");

    uint8_t val = ((char *)packetbuf_dataptr())[0];
    msg[0] = val;
    seq_nb = val;
    rssi = -91 + packetbuf_attr(PACKETBUF_ATTR_RSSI);

    #if MASTER == 0
      printf("BRODCAST message received from (%02x:%02x): '%d' RSSI %d\n", from->u8[0], from->u8[1], seq_nb, rssi);

      leds_toggle(LEDS_RED);
      printf("SLAVE send a unicast with %d\n", seq_nb);
      packetbuf_clear();
      packetbuf_clear_hdr();
      packetbuf_copyfrom(msg, packet_len);
      leds_toggle(LEDS_YELLOW);
      unicast_send(&unicast, &addr_master);
    #endif

    #if MASTER == 1
      //printf("ACK message from (%02x:%02x): '%d RSSI %d'\n", from->u8[0], from->u8[1], seq_nb, rssi);
      leds_toggle(LEDS_RED);
      results_loss[val]=1;
      results_rssi[val]=rssi;
      if (val == (NB_PACKETS -1)) {
        int i=0;
        for (i=0; i<NB_PACKETS ; i++) {
            sum_loss = sum_loss + results_loss[i];
            if (results_loss[i] == 1) {
                sum_rssi = sum_rssi + results_rssi[i];
            }
        }
        
        //printf("PERCENT ACK RECEIVED (LOSS): %d \n", ((sum_loss / NB_PACKETS) * 100) );
        printf("%d;", (int)( (sum_loss / 25.0) * 100));
        //printf("RSSI AVERAGE: %d \n", sum_rssi/sum_loss) ;
        printf("%d;", sum_rssi/sum_loss) ;
        
        sum_loss=0;
        sum_rssi=0;
      }
    #endif

}
static const struct unicast_callbacks unicast_call = {recv_uc};


PROCESS_THREAD(hello_world_process, ev, data)
{
    static struct etimer et;
    
    PROCESS_BEGIN();

    /*****************************************/

    // Set master and slave rime address
    addr_master.u8[0] = 16;
    addr_slave.u8[0] = 15;

    // init results arrays
    init_results_arrays();

    // set radio channel (11 to 26)
    rf230_set_channel(11);

    #if MASTER == 1
        rf230_set_pan_addr(IEEE802154_PANID,0,(uint8_t *)&addr_master.u8);
        rimeaddr_set_node_addr(&addr_master);
    #else
        rf230_set_pan_addr(IEEE802154_PANID,0,(uint8_t *)&addr_slave.u8);
        rimeaddr_set_node_addr(&addr_slave);
    #endif


    /*****************************************/
    
    unicast_open(&unicast, 146, &unicast_call);
    
    printf("Rime Addr: %02x:%02x\n",rimeaddr_node_addr.u8[0],rimeaddr_node_addr.u8[1]);
    
    while(1) {
        
        PROCESS_WAIT_EVENT();
        
        //printf("ev: %d\n",ev);
        
        if(ev == serial_line_event_message) {
            char *tt = data;
            mode = tt[0];
            if(tt[0] == 'b') {
                printf("[INFO] exp begin\n");
                p = 0;
                start = 1;
                counter = 0;
                etimer_set(&et, CLOCK_SECOND/5);
            }
            else if(tt[0] == 'e') {
                printf("[INFO] exp stop\n");
                start = 0;
            }
            /*else if(tt[0] == 'h') {
                printf("[INFO] set tx power to high \n");
                rf230_set_txpower(TX_PWR_3DBM);
            }
            else if(tt[0] == 'l') {
                printf("[INFO] set tx power to low\n");
                rf230_set_txpower(TX_PWR_17_2DBM);
            }*/
            else {
                printf("[INFO] (b) begin exp \t (e) end exp \t (h) set tx power to high \t (l) set tx power to low\n");
            }
        }
        
         
        #if MASTER == 1
            if(ev == 136) { //timer expire
                    if(start == 1) {
                        
                        rf230_set_txpower(tab_power[p]);
                        
                        if(counter < NB_PACKETS) {
                            msg[0] = counter;
                            packetbuf_clear();
                            packetbuf_copyfrom(msg, packet_len);

                            unicast_send(&unicast, &addr_slave);
                            leds_toggle(LEDS_YELLOW);
                            //printf("MASTER send a unicast with %d\n", counter);
                            
                            counter = counter + 1;

                            etimer_reset(&et);
                        }
                        else {
                            
                            //printf("TX POWER: %d - %d \n", rf230_get_txpower(), tab_power[p]) ;
                            printf("%d\n", rf230_get_txpower()) ;
                            
                            p++;
                            counter = 0;
			    // init results arrays
       			    init_results_arrays();
                            
                            if(p<5) { //for 4 power values
                                etimer_reset(&et);
                            }
                        }
                    }
            }
        #endif
    }


    PROCESS_END();
}
/*---------------------------------------------------------------------------*/
