/*
 * Copyright (c) 2007, Swedish Institute of Computer Science.
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
 */

/**
 * \file
 *         Example of how the collect primitive works.
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

#include "contiki.h"
#include "lib/random.h"
#include "net/rime/rime.h"
#include "net/rime/collect.h"
#include "dev/leds.h"
#include "dev/button-sensor.h"
#include "dev/serial-line.h"

#include "dev/adc.h"
#include "dev/dht.h"
#include "dev/Arduino.h"


#include "net/netstack.h"

#include <stdio.h>

/* ADC sensors */
#define A0 14
#define A1 15
#define A2 16
#define D4 4
/* sampling sensors every 10 seconds */
//#define SAMPLE_INTERVAL_SEC 10
/* sampling average in seconds */
//#define AVERAGE_TIME_SEC 60

static struct collect_conn tc;

/*---------------------------------------------------------------------------*/
PROCESS(example_collect_process, "Test collect process");
AUTOSTART_PROCESSES(&example_collect_process);
/*---------------------------------------------------------------------------*/
static void
recv(const linkaddr_t *originator, uint8_t seqno, uint8_t hops)
{
  printf("sink_received;from=%d.%d;seqno=%d;hops=%d;len=%d;payload=%s;\n",
         originator->u8[6], originator->u8[7],
         seqno, hops,
         packetbuf_datalen(),
         (char *)packetbuf_dataptr());
}
/*---------------------------------------------------------------------------*/
static const struct collect_callbacks callbacks = { recv };
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(example_collect_process, ev, data)
{
  static struct etimer periodic;
  static struct etimer et;
  /* Networks metrics */
  const linkaddr_t *parent;
  uint8_t paddr[2];
  int depth;
  struct collect_neighbor *n;
  uint16_t parent_etx;
  uint16_t num_neighbors;
  /* sensors */
  int light, sound, pir;
	struct dht SHT22;
  /* controls */
  int ready=0;

  PROCESS_BEGIN();
  while (1) {
    //Wait here for an event to happen


    PROCESS_WAIT_EVENT();
    if(ev == serial_line_event_message) {
      printf("received line: %s\n", (char *)data);
      char *tt = data;
      if(tt[0] == 'm') {
        printf("[INIT] Node %d.%d must begin\n", linkaddr_node_addr.u8[6],
        linkaddr_node_addr.u8[7]);
        collect_open(&tc, 130, COLLECT_ROUTER, &callbacks);
        if(linkaddr_node_addr.u8[6] == 0 &&
           linkaddr_node_addr.u8[7] == 1) {
          printf("I am sink\n");
          collect_set_sink(&tc, 1);
        }
        /* Allow some time for the network to settle. */
        etimer_set(&et, 20 * CLOCK_SECOND);
        PROCESS_WAIT_UNTIL(etimer_expired(&et));
        ready = 1;
        if(ready) {
          while(1) {
            /* Send a packet every 10 seconds. */
            etimer_set(&periodic, CLOCK_SECOND * 10);
            etimer_set(&et, random_rand() % (CLOCK_SECOND * 10));

            PROCESS_WAIT_UNTIL(etimer_expired(&et));
            {
              light = readADC(A1);
              sound = readADC(A2);
              pir = digitalRead(D4);
              if (dht_read(&SHT22, A0) == 0){
                  printf("[IBAT];hum=%u.%u;temp=%u.%u;light=%d;sound=%d;pir=%d\n",
                       SHT22.humidity_h, SHT22.humidity_l, SHT22.temp_h, SHT22.temp_l,
                       light, sound, pir);
                   packetbuf_clear();
                   packetbuf_set_datalen(sprintf(packetbuf_dataptr(),
                       "hum=%u.%u;temp=%u.%u;light=%d;sound=%d;pir=%d",
                       SHT22.humidity_h, SHT22.humidity_l, SHT22.temp_h, SHT22.temp_l,
                       light, sound, pir)
                       + 1);
                   collect_send(&tc, 15);
              }

              depth = collect_depth(&tc);
              n = collect_neighbor_list_find(&tc.neighbor_list, &tc.parent);
              if(n != NULL)
                parent_etx = collect_neighbor_link_estimate(n);
              else
                parent_etx = 0;
              num_neighbors = collect_neighbor_list_num(&tc.neighbor_list);
              parent = collect_parent(&tc);
              paddr[0]=parent->u8[6];
              paddr[1]=parent->u8[7];

              printf("node_sending:%d.%d;payload=parent:%d.%d:depth:%d:etx:%d:rtmetric:%d:neighbors:%d\n",
                linkaddr_node_addr.u8[6],linkaddr_node_addr.u8[7],
                paddr[0],paddr[1],depth,parent_etx,tc.rtmetric,
                num_neighbors);
            }
            //PROCESS_WAIT_UNTIL(etimer_expired(&periodic));
          } //end while 1
        } // End ready
      } // end "m"
    } // end serial_line_event_message
  } // end while 1
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
