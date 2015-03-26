/*
 *
 * This file is part of the Contiki operating system.
 *
 */


#include "contiki.h"
#include <stdio.h> /* For printf() */
#include "Arduino.h"
//#include "Sensirion.h"
//#include <stdint.h>

/*********************** Includes Sensirion_c.h ******************************/

// Enable CRC checking
//#define CRC_ENA

// Enable ('1') or disable ('0') internal pullup on DATA line
// Commenting out this #define saves code space but leaves internal pullup
//   state undefined (ie, depends on last bit transmitted)
//#define DATA_PU 1

// Clock pulse timing macros
// Lengthening these may assist communication over long wires
//#define PULSE_LONG  delayMicroseconds(3)
//#define PULSE_SHORT delayMicroseconds(1)  

#define PULSE_LONG  delayMicroseconds(2)
#define PULSE_SHORT delayMicroseconds(1)  


// Useful macros
#define measTemp(result)  meas(TEMP, result, BLOCK)
#define measHumi(result)  meas(HUMI, result, BLOCK)

// User constants
const uint8_t TEMP     =     0;
const uint8_t HUMI     =     1;

const uint8_t BLOCK    =     0;	//true;

const uint8_t NONBLOCK =     1; //false;

// Function return code definitions
const uint8_t S_Err_NoACK  = 1;  // ACK expected but not received
const uint8_t S_Err_TO     = 3;  // Timeout
const uint8_t S_Meas_Rdy   = 4;  // Measurement ready


    uint8_t _pinData;
    uint8_t _pinClock;
    uint16_t *_presult;
//    uint8_t _stat_reg;

    uint8_t getResult(uint16_t *result);
    uint8_t putByte(uint8_t value);
    uint8_t getByte(uint8_t ack);
    void startTransmission(void);
    void resetConnection(void);
//#ifdef CRC_ENA
  //  void calcCRC(uint8_t value, uint8_t *crc);
//#endif

    void Sensirion_cfg(uint8_t dataPin, uint8_t clockPin);
    uint8_t meas(uint8_t cmd, uint16_t *result, uint8_t block);
    uint8_t measRdy(void);

    uint8_t reset(void);


/******************************************************************************

 * Definitions

 ******************************************************************************/



// Sensirion command definitions:      adr command r/w

const uint8_t MEAS_TEMP   = 0x03;   // 000  0001   1

const uint8_t MEAS_HUMI   = 0x05;   // 000  0010   1

//const uint8_t STAT_REG_W  = 0x06;   // 000  0011   0

//const uint8_t STAT_REG_R  = 0x07;   // 000  0011   1

const uint8_t SOFT_RESET  = 0x1e;   // 000  1111   0



// Status register writable bits

//const uint8_t SR_MASK     = 0x07;



// getByte flags

const uint8_t noACK  = 1; //false;

const uint8_t ACK    = 0; //true;




void Sensirion_cfg(uint8_t dataPin, uint8_t clockPin) {
  // Initialize private storage for library functions

  _pinData = dataPin;

  _pinClock = clockPin;

  _presult = NULL;                  // No pending measurement




  // Initialize CLK signal direction

  // Note: All functions exit with CLK low and DAT in input mode

  pinMode(_pinClock, OUTPUT);



  // Return sensor to default state

  resetConnection();                // Reset communication link with sensor

  putByte(SOFT_RESET);              // Send soft reset command

}





/******************************************************************************

 * User functions

 ******************************************************************************/


// Initiate measurement.  If blocking, wait for result

uint8_t meas(uint8_t cmd, uint16_t *result, uint8_t block) {
  uint8_t error, i;

  startTransmission();

  if (cmd == TEMP)

    cmd = MEAS_TEMP;

  else

    cmd = MEAS_HUMI;
    error = putByte(cmd);

  if (error)

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

//uint8_t Sensirion::measRdy(void) {
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


uint8_t getResult(uint16_t *result) {
  *result = getByte(ACK);
  *result = (*result << 8) | getByte(noACK);
  return 0;

}








// Note: Soft reset returns sensor status register to default values

//uint8_t Sensirion::reset(void) {
uint8_t reset(void) {
  resetConnection();                // Reset communication link with sensor

  return putByte(SOFT_RESET);       // Send soft reset command & return status

}





/******************************************************************************

 * Sensirion data communication

 ******************************************************************************/



// Write byte to sensor and check for acknowledge

//uint8_t Sensirion::putByte(uint8_t value) {
uint8_t putByte(uint8_t value) {
  uint8_t mask, i;

  uint8_t error = 0;

  pinMode(_pinData, OUTPUT);        // Set data line to output mode

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



// Read byte from sensor and send acknowledge if "ack" is true

uint8_t getByte(uint8_t ack) {
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

//void Sensirion::startTransmission(void) {
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



// Communication link reset

// At least 9 SCK cycles with DATA=1, followed by transmission start sequence

//      ______________________________________________________         ________

// DATA:                                                      |_______|

//          _    _    _    _    _    _    _    _    _        ___     ___

// SCK : __| |__| |__| |__| |__| |__| |__| |__| |__| |______|   |___|   |______

//void Sensirion::resetConnection(void) {
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


uint8_t retour;
uint16_t valeur_t;


/*---------------------------------------------------------------------------*/
PROCESS(gtestSHT75a_process, "gtesta SHT75");
AUTOSTART_PROCESSES(&gtestSHT75a_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(gtestSHT75a_process, ev, data)
{

   // static struct etimer timer;
    static int counter=0;

    PROCESS_BEGIN();

    PROCESS_PAUSE();
     pinMode(5, OUTPUT);
    Sensirion_cfg(7,6);

    printf("Start loop\n");    
   // etimer_set(&timer, CLOCK_CONF_SECOND*4);

     while(1) {
     //   PROCESS_WAIT_EVENT();
     //   if(ev == PROCESS_EVENT_TIMER)
        { 
	//resetConnection();
        retour = meas(TEMP,&valeur_t, BLOCK);
        printf("Hello, world ...retour= %d  temp = %d\n", retour,valeur_t);

        
    
  // turn the LED on when we're done
        digitalWrite(5, HIGH); 
	//clock_delay(1);  // 24 us
	//clock_wait(1);
	PULSE_SHORT;
	digitalWrite(5, LOW); 
	PULSE_LONG;
	digitalWrite(5, HIGH);
	PULSE_SHORT;
	digitalWrite(5, LOW); 
        printf("Counter: %d\n", counter);
        counter++;
       // etimer_reset(&timer);
	}
    }
    PROCESS_END();
}
/*---------------------------------------------------------------------------*/
