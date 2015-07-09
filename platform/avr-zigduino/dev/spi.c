/*
 * Copyright (c) 2010 by Cristian Maglie <c.maglie@bug.st>
 * SPI Master library for arduino.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

//#include "pins_arduino.h"
#include "spi.h"
#include <avr/io.h>
//#include <avr/interrupt.h>


void SPI_begin() {

  // Set SS to high so a connected chip will be "deselected" by default
  digital_Write_SS_HIGH();

  // When the SS pin is set as OUTPUT, it can be used as
  // a general purpose output port (it doesn't influence
  // SPI operations).
  pin_Mode_SS_OUTPUT();

  // Warning: if the SS pin never becomes a LOW INPUT then SPI
  // automatically switches to Slave, so the data direction of
  // the SS pin MUST be kept as OUTPUT.
  SPCR |= _BV(MSTR);
  SPCR |= _BV(SPE);
  SPCR |= _BV(SPR0); // FCK/16    !!!! Important

  // Set direction register for SCK and MOSI pin.
  // MISO pin automatically overrides to INPUT.
  // By doing this AFTER enabling SPI, we avoid accidentally
  // clocking in a single bit since the lines go directly
  // from "input" to SPI control.  
  // http://code.google.com/p/arduino/issues/detail?id=888
  //pinMode(SCK, OUTPUT);
  //pinMode(MOSI, OUTPUT);
  
    // Define the following pins as output
    DDR_SPI |= ((1<<DD_MOSI)|(1<<DD_SCK));
    PORT_SPI  |= ((1<<PORT_MOSI)|(1<<PORT_SCK)); 

}


void SPI_end() {
  SPCR &= ~_BV(SPE);
}



void SPI_setBitOrder(uint8_t bitOrder)
{
  if(bitOrder == LSBFIRST) {
    SPCR |= _BV(DORD);
  } else {
    SPCR &= ~(_BV(DORD));
  }
}

void SPI_setDataMode(uint8_t mode)
{
  SPCR = (SPCR & ~SPI_MODE_MASK) | mode;
}

void SPI_setClockDivider(uint8_t rate)
{
  SPCR = (SPCR & ~SPI_CLOCK_MASK) | (rate & SPI_CLOCK_MASK);
  SPSR = (SPSR & ~SPI_2XCLOCK_MASK) | ((rate >> 2) & SPI_2XCLOCK_MASK);
}

extern uint8_t SPI_transfer(uint8_t _data) {
  SPDR = _data;
  while (!(SPSR & _BV(SPIF)))
    ;
  return SPDR;
}

void attachInterrupt() {
  SPCR |= _BV(SPIE);
}

void detachInterrupt() {
  SPCR &= ~_BV(SPIE);
}


