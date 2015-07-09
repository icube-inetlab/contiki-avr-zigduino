/*
 * Copyright (c) 2010 by Cristian Maglie <c.maglie@bug.st>
 * SPI Master library for arduino.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

#ifndef _SPI_H_INCLUDED
#define _SPI_H_INCLUDED

#include <avr/io.h>
//#include <avr/interrupt.h>


//#include <stdio.h>
//#include <Arduino.h>
//#include <avr/pgmspace.h>

#define SPI_CLOCK_DIV4 0x00
#define SPI_CLOCK_DIV16 0x01
#define SPI_CLOCK_DIV64 0x02
#define SPI_CLOCK_DIV128 0x03
#define SPI_CLOCK_DIV2 0x04
#define SPI_CLOCK_DIV8 0x05
#define SPI_CLOCK_DIV32 0x06
//#define SPI_CLOCK_DIV64 0x07

#define SPI_MODE0 0x00
#define SPI_MODE1 0x04
#define SPI_MODE2 0x08
#define SPI_MODE3 0x0C

#define SPI_MODE_MASK 0x0C  // CPOL = bit 3, CPHA = bit 2 on SPCR
#define SPI_CLOCK_MASK 0x03  // SPR1 = bit 1, SPR0 = bit 0 on SPCR
#define SPI_2XCLOCK_MASK 0x01  // SPI2X = bit 0 on SPSR
#define LSBFIRST 0
#define MSBFIRST 1

#define PORT_SPI    PORTB
#define PORT_MISO    PORTB3  //Digital 12
#define PORT_MOSI    PORTB2  // Digital 11
#define PORT_SCK    PORTB1  //Digital 13
#define PORT_SS    PORTB6   // Digital 

#define DDR_SPI     DDRB
#define DD_MISO     DDB3    //
#define DD_MOSI     DDB2    //
#define DD_SS       DDB6    //  
#define DD_SCK      DDB1    // 
#define DD_SSN      DDB0  // Pin dedicated in Atmega128rfa1 for SS

#define digital_Write_SS_Low()     (PORTB &= (0<<PORTB6))	/* CS Output=0 */
#define digital_Write_SS_HIGH()    (PORTB |=  (1<<PORTB6))	/* CSL Output=1 */
#define alarmPin_In()   	   (DDRE &= (0<<PORTE6))  	/* INT0 input */
#define alarmPin_High() 	   (PORTE |= (1<<PORTE6))

#define	pin_Mode_SS_OUTPUT()	   (DDR_SPI |= (1<<DD_SS))




  uint8_t SPI_transfer(uint8_t _data);

  // SPI Configuration methods

   void attachInterrupt();
  void detachInterrupt(); // Default

  void SPI_begin(); // Default
  void SPI_end();

  void SPI_setBitOrder(uint8_t);
  void SPI_setDataMode(uint8_t);
  void SPI_setClockDivider(uint8_t);



#endif

