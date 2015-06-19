/*
 * Copyright (c) 2006, Technical University of Munich
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
#define ANNOUNCE_BOOT 1
#define DEBUG DEBUG_PRINT
#include "uip-debug.h" ////Does #define PRINTA(FORMAT,args...) printf_P(PSTR(FORMAT),##args) for AVR

#include <avr/pgmspace.h>
#include <avr/fuse.h>
#include <avr/eeprom.h>
#include <avr/io.h>
#include <stdio.h>
#include <string.h>
#include <dev/watchdog.h>

#include "loader/symbols-def.h"
#include "loader/symtab.h"
#include "radio/rf230bb/rf230bb.h"
#include "net/mac/frame802154.h"
#include "net/mac/framer-802154.h"
#include "net/sicslowpan.h"

#if !RF230BB        //radio driver using contiki core mac
#error Zigduino has only been tested with RF230BB
#endif /*RF230BB*/

//#if !UIP_CONF_IPV6
//#error Zigduino has only been tested with IPv6
//#endif

#include "contiki.h"
#include "contiki-net.h"
#include "contiki-lib.h"

#include "dev/rs232.h"
#include "dev/serial-line.h"
#include "dev/slip.h"

#include "lib/sensors.h"
#include "dev/button-sensor.h"
#include "dev/leds.h"

//#include "Arduino.h"

// This Zidguino implementation presumes a single button 'sensor'
SENSORS(&button_sensor);

// Zigduino has not been tested with Rime.
#if 1
#if UIP_CONF_ROUTER&&0
#include "net/routing/rimeroute.h"
#include "net/rime/rime-udp.h"
#endif
#include "net/rime.h"
#endif

/*----------------------Configuration of EEPROM---------------------------*/
/* Use existing EEPROM if it passes the integrity test, else reinitialize with build values */

#if RIMEADDR_SIZE == 8
uint8_t mac_address[8] EEMEM = {0x02, 0x11, 0x22, 0xff, 0xfe, 0x33, 0x44, 0x55};
#else
uint8_t short_mac_address[2] EEMEM = {0x01, 0xff};
#endif

/*
uint8_t rf_channel[2] EEMEM = {26, ~26};

static uint8_t get_channel_from_eeprom()
{
  uint8_t eeprom_channel;
  uint8_t eeprom_check;

  eeprom_channel = eeprom_read_byte(&rf_channel[0]);
  eeprom_check = eeprom_read_byte(&rf_channel[1]);

  if(eeprom_channel==~eeprom_check)
    return eeprom_channel;

  return 26;
}
*/

static bool get_mac_from_eeprom(uint8_t* macptr)
{
#if RIMEADDR_SIZE == 8
  eeprom_read_block ((void *)macptr,  &mac_address, 8);
#else
  printf("reading short_mac in eeprom\n");
  uint8_t small_addr[2];
  small_addr[0] = eeprom_read_byte( &short_mac_address[0]+8);
  small_addr[1] = eeprom_read_byte( &short_mac_address[1]+8);
  macptr[0]=small_addr[0];
  macptr[1]=small_addr[1];
  printf("read %02x:%02x\n", small_addr[0],small_addr[1]);
  
#endif
  return true;
}

void print_rime_addr()
{
#if RIMEADDR_SIZE == 8
    printf("Rime Addr: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\n",
            rimeaddr_node_addr.u8[0],
            rimeaddr_node_addr.u8[1],
            rimeaddr_node_addr.u8[2],
            rimeaddr_node_addr.u8[3],
            rimeaddr_node_addr.u8[4],
            rimeaddr_node_addr.u8[5],
            rimeaddr_node_addr.u8[6],
            rimeaddr_node_addr.u8[7]);
#else
    printf("Rime Addr: %02x:%02x\n",
            rimeaddr_node_addr.u8[0],
            rimeaddr_node_addr.u8[1]);
#endif
}

/*-------------------------Low level initialization------------------------*/
/*------Done in a subroutine to keep main routine stack usage small--------*/
void initialize(void)
{
  watchdog_init();
  watchdog_start();

#if WITH_SLIP
  //Slip border router on uart0
  rs232_init(RS232_PORT_0, USART_BAUD_38400,USART_PARITY_NONE | USART_STOP_BITS_1 | USART_DATA_BITS_8);
#else
  /* First rs232 port for debugging */
  rs232_init(RS232_PORT_0, USART_BAUD_57600,USART_PARITY_NONE | USART_STOP_BITS_1 | USART_DATA_BITS_8);

  /* Redirect stdout to first port */
  rs232_redirect_stdout(RS232_PORT_0);

  /* Get input from first port */
  rs232_set_input(RS232_PORT_0, serial_line_input_byte);
#endif

  clock_init();

  if(MCUSR & (1<<PORF )) PRINTA("Power-on reset.\n");
  if(MCUSR & (1<<EXTRF)) PRINTA("External reset!\n");
  if(MCUSR & (1<<BORF )) PRINTA("Brownout reset!\n");
  if(MCUSR & (1<<WDRF )) PRINTA("Watchdog reset!\n");
  if(MCUSR & (1<<JTRF )) PRINTA("JTAG reset!\n");

#if ANNOUNCE_BOOT
  PRINTA("\n*******Booting %s*******\n",CONTIKI_VERSION_STRING);
#endif

  /* rtimers needed for radio cycling */
  rtimer_init();

  /* Initialize process subsystem */
  process_init();

  /* etimers must be started before ctimer_init */
  process_start(&etimer_process, NULL);
  ctimer_init();

  /* Initilaize serial line for input */
  serial_line_init();

  /* Initialize board LEDs */
  leds_init();

  /* When used for testing, we should start with them off */
  leds_off(LEDS_GREEN|LEDS_YELLOW);

  /* Start radio and radio receive process */
  NETSTACK_RADIO.init();

  /* Set addresses BEFORE starting tcpip process */
  rimeaddr_t addr;
  memset(&addr, 0, sizeof(rimeaddr_t));
  get_mac_from_eeprom(addr.u8);
  rimeaddr_set_node_addr(&addr);
  
  /* Set channel */
  //uint8_t radio_channel = get_channel_from_eeprom();
  //rf230_set_channel(radio_channel);
  
  rf230_set_channel(26);
  rf230_set_txpower(TX_PWR_MAX);

  print_rime_addr();

  /* Initialize stack protocols */
  queuebuf_init();
  NETSTACK_RDC.init();
  NETSTACK_MAC.init();
  NETSTACK_NETWORK.init();

#if ANNOUNCE_BOOT
  PRINTA("Driver: %s + %s + %s\nChannel: %u , Check rate %u Hz, TX Power %u\n",NETSTACK_MAC.name, NETSTACK_RDC.name, NETSTACK_NETWORK.name, rf230_get_channel(),
    CLOCK_SECOND / (NETSTACK_RDC.channel_check_interval() == 0 ? 1:NETSTACK_RDC.channel_check_interval()),
    rf230_get_txpower());
#endif /* ANNOUNCE_BOOT */

  /* Sensors process means all processes will get an event posted whenever sensors change.
     Not always desired, so may want to put this on a compile switch. */
  //process_start(&sensors_process, NULL);
  //SENSORS_ACTIVATE(button_sensor);


#if UIP_CONF_IPV6
  process_start(&tcpip_process, NULL);
  //Give ourselves a prefix
  init_net();
#endif
  
  printf("Autostart other processes\n");
  /* Autostart other processes */
  autostart_start(autostart_processes);

  /*--------------------------Announce the configuration---------------------*/

}

/*---------------------------------------------------------------------------*/
void log_message(char *m1, char *m2)
{
  PRINTA("%s%s\n", m1, m2);
}

extern char rf230_interrupt_flag, rf230processflag;

/*-------------------------------------------------------------------------*/
/*------------------------- Main Scheduler loop----------------------------*/
/*-------------------------------------------------------------------------*/
int
main(void)
{
  initialize();

  while(1)
  {
    process_run();
    watchdog_periodic();
  }
  return 0;
}
// vim:ai:cin:sts=2 sw=2

