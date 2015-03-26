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
EEPROM_SIZE * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
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
//#include "dev/eeprom.h"
#include <stdio.h> /* For printf() */


#include "Arduino.h"

#define PULSE_LONG  delayMicroseconds(3)
#define PULSE_SHORT delayMicroseconds(1)

typedef unsigned char bool;


//uint16_t EEPROM_SIZE = 65535;

    uint8_t getResult(uint16_t *result);
    uint8_t putByte(uint8_t value);
    uint8_t getByte(bool ack);
    void startTransmission(void);
    void resetConnection(void);
    uint8_t meas(uint8_t cmd, uint16_t *result, bool block);
    void confSHT_pins(uint8_t dataPin, uint8_t clockPin);






int temperatureCommand  = B00000011;  // command used to read temperature
int humidityCommand = B00000101;  // command used to read humidity

//int clockPin = 6;  // pin used for clock
//int dataPin  = 7;  // pin used for data
int ack;  // track acknowledgment for errors
int val;
float val_f;
float temperature;
float humidity;

 uint8_t _pinData = 7;
 uint8_t _pinClock = 6;
 uint16_t *_presult;
 uint16_t data16;
 uint8_t error;
  

// User constants
const uint8_t TEMP     =     0;
const uint8_t HUMI     =     1;
const bool BLOCK       =  true;
const bool NONBLOCK    = false;

const bool noACK  = false;
const bool ACK    = true;

const uint8_t SOFT_RESET  = 0x1e;   // 000  1111   0
const uint8_t S_Err_NoACK  = 1;  // ACK expected but not received
const uint8_t S_Err_TO     = 3;  // Timeout
const uint8_t S_Meas_Rdy   = 4;  // Measurement ready
const uint8_t MEAS_TEMP   = 0x03;   // 000  0001   1
const uint8_t MEAS_HUMI   = 0x05;   // 000  0010   1



 uint8_t reset(void) {
 
// _stat_reg = 0x00;                 // Sensor status register default state

  resetConnection();                // Reset communication link with sensor

  return putByte(SOFT_RESET);       // Send soft reset command & return status

}

// Read byte from sensor and send acknowledge if "ack" is true


  uint8_t getByte(bool ack) {

  uint8_t i;

  uint8_t result = 0;

  for (i = 8; i > 0; i--) {

    result <<= 1;                   // Shift received bits towards MSB

    digitalWrite(_pinClock, HIGH);  // Generate clock pulse

    PULSE_SHORT;

    result |= digitalRead(_pinData);  // Merge next bit into LSB position

    digitalWrite(_pinClock, LOW);

    PULSE_SHORT;

  }

  pinMode(_pinData, OUTPUT);

  digitalWrite(_pinData, !ack);     // Assert ACK ('0') if ack == 1

  PULSE_SHORT;

  digitalWrite(_pinClock, HIGH);    // Clock #9 for ACK / noACK

  PULSE_LONG;

  digitalWrite(_pinClock, LOW);     // Finish with clock in low state

  PULSE_SHORT;

  pinMode(_pinData, INPUT);         // Return data line to input mode

#ifdef DATA_PU

  digitalWrite(_pinData, DATA_PU);  // Restore internal pullup state

#endif

  return result;

}




// Get measurement result from sensor (plus CRC, if enabled)

  uint8_t getResult(uint16_t *result) {

  *result = getByte(ACK);

  *result = (*result << 8) | getByte(noACK);

  return 0;

}

// Write byte to sensor and check for acknowledge


  uint8_t putByte(uint8_t value) {
 
  uint8_t mask, i;

  uint8_t error = 0;

  pinMode(_pinData, OUTPUT);        // Set data line to output mode
pinMode(_pinData, OUTPUT); 
  mask = 0x80;                      // Bit mask to transmit MSB first

  for (i = 8; i > 0; i--) {

    digitalWrite(_pinData, value & mask);

    PULSE_SHORT;

    digitalWrite(_pinClock, HIGH);  // Generate clock pulse

    PULSE_LONG;

    digitalWrite(_pinClock, LOW);

    PULSE_SHORT;

    mask >>= 1;                     // Shift mask for next data bit

  }

  pinMode(_pinData, INPUT);         // Return data line to input mode

#ifdef DATA_PU

  digitalWrite(_pinData, DATA_PU);  // Restore internal pullup state

#endif

  digitalWrite(_pinClock, HIGH);    // Clock #9 for ACK

  PULSE_LONG;

  if (digitalRead(_pinData))        // Verify ACK ('0') received from sensor

    error = S_Err_NoACK;

  PULSE_SHORT;

  digitalWrite(_pinClock, LOW);     // Finish with clock in low state

  return error;

}

void resetConnection(void) {
  uint8_t i;

  digitalWrite(_pinData, HIGH);  // Set data register high before turning on

  pinMode(_pinData, OUTPUT);     // output driver (avoid possible low pulse)

  PULSE_LONG;

  for (i = 0; i < 9; i++) {

    digitalWrite(_pinClock, HIGH);

    PULSE_LONG;

    digitalWrite(_pinClock, LOW);

    PULSE_LONG;

  }

  startTransmission();

}


/******************************************************************************

 * Sensirion signaling

 ******************************************************************************/



// Generate Sensirion-specific transmission start sequence

// This is where Sensirion does not conform to the I2C standard and is

// the main reason why the AVR TWI hardware support can not be used.

//       _____         ________

// DATA:      |_______|

//           ___     ___

// SCK : ___|   |___|   |______


void startTransmission(void) {
  digitalWrite(_pinData, HIGH);  // Set data register high before turning on
  pinMode(_pinData, OUTPUT);     // output driver (avoid possible low pulse)
  PULSE_SHORT;
  digitalWrite(_pinClock, HIGH);
  PULSE_SHORT;
  digitalWrite(_pinData, LOW);
  PULSE_SHORT;
  digitalWrite(_pinClock, LOW);
  PULSE_LONG;
  digitalWrite(_pinClock, HIGH);
  PULSE_SHORT;
  digitalWrite(_pinData, HIGH);
  PULSE_SHORT;
  digitalWrite(_pinClock, LOW);
  PULSE_SHORT;

  // Unnecessary here since putByte always follows startTransmission

//  pinMode(_pinData, INPUT);

}


// Initiate measurement.  If blocking, wait for result

  uint8_t meas(uint8_t cmd, uint16_t *result, bool block) {

  uint8_t error, i;

  startTransmission();

  if (cmd == TEMP)

    cmd = MEAS_TEMP;

  else

    cmd = MEAS_HUMI;

  if ((error = putByte(cmd)))

    return error;

  // If non-blocking, save pointer to result and return

  if (!block) {

    _presult = result;

    return 0;

  }

  // Otherwise, wait for measurement to complete with 720ms timeout

  i = 240;

  while (digitalRead(_pinData)) {

    i--;

    if (i == 0)

      return S_Err_TO;              // Error: Timeout

    delay(3);

  }

  error = getResult(result);

  return error;

}



// Check if non-blocking measurement has completed

// Non-zero return indicates complete (with or without error)

  uint8_t measRdy(void) {

  uint8_t error = 0;

  if (_presult == NULL)             // Already done?

    return S_Meas_Rdy;

  if (digitalRead(_pinData) != 0)   // Measurement ready yet?

    return 0;

  error = getResult(_presult);

  _presult = NULL;

  if (error)

    return error;                   // Only possible error is S_Err_CRC

  return S_Meas_Rdy;

}



void confSHT_pins(uint8_t dataPin, uint8_t clockPin) {

  _pinData = dataPin;
  _pinClock = clockPin;
  _presult = NULL;                  // No pending measurement

  //_stat_reg = 0x00;                 // Sensor status register default state

  // Initialize CLK signal direction

  // Note: All functions exit with CLK low and DAT in input mode

  pinMode(_pinClock, OUTPUT);



  // Return sensor to default state

  resetConnection();                // Reset communication link with sensor
  putByte(SOFT_RESET);              // Send soft reset command

}






/*---------------------------------------------------------------------------*/
PROCESS(hello_world_process, "Hello world process");
AUTOSTART_PROCESSES(&hello_world_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(hello_world_process, ev, data)
{
    
    static struct etimer timer;
    static int counter=0;
    PROCESS_BEGIN();
    PROCESS_PAUSE();

   // eeprom_write(0,0,1);
    printf("Hello, world\n");
    confSHT_pins(_pinClock,_pinData);
   // PULSE_SHORT;
    printf("Start loop\n");    
    etimer_set(&timer, CLOCK_CONF_SECOND*2);

    while(1){
       PROCESS_WAIT_EVENT();
       if(ev == PROCESS_EVENT_TIMER)
        { 
   	   error = meas(TEMP,&data16,NONBLOCK);
    	   printf(" data16 : %d\n", data16);
           //PULSE_SHORT;
    
           printf("Counter: %d\n", counter);
           counter++;
           etimer_reset(&timer);
         //delay(2000); 
         }
    }
    PROCESS_END();
}
/*---------------------------------------------------------------------------*/
