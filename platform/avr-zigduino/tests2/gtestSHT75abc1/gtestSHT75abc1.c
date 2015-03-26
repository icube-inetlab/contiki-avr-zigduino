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

#define PULSE_LONG  delay_t_uS(3);
#define PULSE_SHORT delay_1_uS(); 


// Useful macros
#define measTemp(result)  meas(TEMP, result)
#define measHumi(result)  meas(HUMI, result)


//RL Macros
#define sdaW_H  PORTE |= 0x80; DDRE |= 0x80;
#define sdaW_L  PORTE &= 0x7F; DDRE |= 0x80;

#define sclW_H  PORTE |= 0x10; DDRE |= 0x10;
#define sclW_L  PORTE &= 0xEF; DDRE |= 0x10;

#define delay_5_uS delay_1_uS();delay_1_uS();delay_1_uS();delay_1_uS();delay_1_uS();delay_1_uS();

// User constants
const uint8_t TEMP     =     0;
const uint8_t HUMI     =     1;


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
    uint8_t meas(uint8_t cmd, uint16_t *result);
    uint8_t measRdy(void);

    uint8_t reset(void);

    void delay_1_uS(void);
    void delay_t_uS(uint8_t t);

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

void delay_1_uS(void){
	uint8_t i;
	for(i=0;i<2;i++) {
	   asm volatile("nop");
	   asm volatile("nop");
	   asm volatile("nop");
	}
}


void delay_t_uS(uint8_t t){
	uint8_t i;
	for(i=0;i<2*t;i++) {
		asm volatile("nop");
		asm volatile("nop");
		//asm volatile("nop");
		//asm volatile("nop");
	}
}


void Sensirion_cfg(uint8_t dataPin, uint8_t clockPin) {
  // Initialize private storage for library functions

  _pinData = dataPin;

  _pinClock = clockPin;

  _presult = NULL;                  // No pending measurement




  // Initialize CLK signal direction

  // Note: All functions exit with CLK low and DAT in input mode

  pinMode(_pinClock, OUTPUT);
  digitalWrite(_pinClock, LOW);


  // Return sensor to default state

 // resetConnection();                // Reset communication link with sensor

 // putByte(SOFT_RESET);              // Send soft reset command

}





/******************************************************************************

 * User functions

 ******************************************************************************/


// Initiate measurement.  If blocking, wait for result

uint8_t meas(uint8_t cmd, uint16_t *result) {
  uint8_t error, i;

  startTransmission();

  if (cmd == TEMP)

    cmd = MEAS_TEMP;

  else

    cmd = MEAS_HUMI;
    error = putByte(cmd);

  if (error)

    return error;

 
  // Otherwise, wait for measurement to complete with 720ms timeout

  i = 240;

  while (digitalRead(_pinData)) {

    i--;

    if (i == 0)

      return S_Err_TO;              // Error: Timeout
    printf("Attente Rd\n");
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
  printf("GetByte result : %d\n",result);
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
// sclW_L;
// sdaW_H;
  pinMode(_pinData, OUTPUT);     // output driver (avoid possible low pulse)

  //PULSE_SHORT;
//  PULSE_LONG;
  digitalWrite(_pinClock, HIGH);
//  sclW_H;
 // PULSE_LONG;
  
  digitalWrite(_pinData, LOW);
 // sdaW_L;
  //PULSE_LONG;
  
  digitalWrite(_pinClock, LOW);
  //sclW_L;
 // PULSE_SHORT;
  //  PULSE_LONG;
  digitalWrite(_pinClock, HIGH);
  //sclW_H;
 // PULSE_LONG;
  //PULSE_SHORT;// 
  digitalWrite(_pinData, HIGH);
  //sdaW_H;
 // PULSE_LONG;

  digitalWrite(_pinClock, LOW);
  //sclW_L;
  //PULSE_LONG;

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
  sclW_L;
  //digitalWrite(_pinData, HIGH);  // Set data register high before turning on
  sdaW_H;
	
 // pinMode(_pinData, OUTPUT);     // output driver (avoid possible low pulse)    
   	
  PULSE_LONG;

  for (i = 0; i < 9; i++) {

    //digitalWrite(_pinClock, HIGH);
    sclW_H;
    PULSE_LONG;

    //digitalWrite(_pinClock, LOW);
    sclW_L;
    PULSE_LONG;

  }

 // startTransmission();

}


uint8_t retour;
uint16_t valeur_t;



/*
pinMode(6,OUTPUT); ==> SCL   PE4
pinMode(7, OUTPUT); ==> SDA  PE7

// PE7 à 1  SDA7_H
PORTE  |= 0x80;
DDRE   |= 0x80;

// PE7 à 0  SDA7_L
PORTE  &= 0x7F;
DDRE   |= 0x80;

// PE4 à 1  SCL6_H
PORTE  |= 0x10;
DDRE   |= 0x10;

// PE4 à 0  SCL6_L
PORTE  &= 0xEF;
DDRE   |= 0x10;

//PE3 <==> 5
// PE3 à 1
PORTE  |= 0x08;
DDRE   |= 0x08;

// PE3 à 0
PORTE  &= 0xF7;
DDRE   |= 0x08;


PORTE  |= 0x80;

PORTB = (1<<PB7)|(1<<PB6)|(1<<PB1)|(1<<PB0);
DDRB = (1<<DDB3)|(1<<DDB2)|(1<<DDB1)|(1<<DDB0);


 RELEASE_DATA; SCK=0;                   //Initial state
   DelayUs(2);
   SCK=1;
   DelayUs(2);
   LOW_DATA;CAPTURE_DATA;
   DelayUs(2);
   SCK=0;  
   DelayUs(5);
   SCK=1;
   DelayUs(2);
   RELEASE_DATA;		   
   DelayUs(2);
   SCK=0;
	DelayUs(2);	



*/



/*---------------------------------------------------------------------------*/
PROCESS(gtestSHT75a_process, "gtesta SHT75");
AUTOSTART_PROCESSES(&gtestSHT75a_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(gtestSHT75a_process, ev, data)
{

   // static struct etimer timer;
    static int counter=0;
//	int start, end;
//	uint16_t i,j;
 //      uint8_t s= 0xFF;

    PROCESS_BEGIN();

    //PROCESS_PAUSE();
     pinMode(5, OUTPUT);
     Sensirion_cfg(7,6);
   // startTransmission();

    printf("Start loop\n");    
   // etimer_set(&timer, CLOCK_CONF_SECOND*4);

     while(1) {
     //   PROCESS_WAIT_EVENT();
     //   if(ev == PROCESS_EVENT_TIMER)
        { 
	//resetConnection();
        //startTransmission();
	
//  digitalWrite(_pinData, HIGH);  // Set data register high before turning on
 sclW_L; sdaW_H;
 delay_t_uS(4);
 // pinMode(_pinData, OUTPUT);     // output driver (avoid possible low pulse)

  //PULSE_SHORT;
//  PULSE_LONG;
//  digitalWrite(_pinClock, HIGH);
  sclW_H; delay_t_uS(4);
  delay_t_uS(4);
 // PULSE_LONG;
//  PULSE_LONG;
 // digitalWrite(_pinData, LOW);
  sdaW_L; //delay_t_uS(1);
 // PULSE_LONG;
  
 //digitalWrite(_pinClock, LOW);
  sclW_L; delay_t_uS(4);
 // PULSE_SHORT;
 //  PULSE_LONG;
  //digitalWrite(_pinClock, HIGH);
  sclW_H; delay_t_uS(4);
 // PULSE_LONG;
  //PULSE_SHORT;// 
  //digitalWrite(_pinData, HIGH);
  sdaW_L; delay_t_uS(4);
  sdaW_H; delay_t_uS(1);
 // PULSE_LONG;

  //digitalWrite(_pinClock, LOW);
  sclW_L;
     //   retour = meas(TEMP,&valeur_t);
     //   printf("Hello, world ...retour= %d  temp = %d\n", retour,valeur_t);

       // Sensirion_cfg(7,6);
    
  // turn the LED on when we're done
    /*    digitalWrite(5, HIGH); 
	//clock_delay(1);  // 24 us
	//clock_wait(1);
	PULSE_SHORT;
	digitalWrite(5, LOW); 
	PULSE_LONG;
	digitalWrite(5, HIGH);
	PULSE_SHORT;
	digitalWrite(5, LOW); 
*/
//	start = micro();
/*
	sdaW_H;
	sclW_L;
	PULSE_LONG;
	sclW_H;
	PULSE_SHORT;
	sclW_L;
	PULSE_LONG;
	sclW_H;
	sdaW_L;
*/
	// PE3 à 1  (5)
	PORTE  |= 0x08;
	DDRE   |= 0x08;

	//__no_operation();

	//for(j=0;j<10000;j++) 
	//	for(i=0;i<10000;i++)  s&=0x1F ; 

	//delayMicroseconds(20);  // 1us => 1,69us  (-0,25us) 2us => 2,44us  10 => 10,44us
	//__asm__volatile("nop");  // n'est pas reconnu !!!

	delay_t_uS(7);

	// PE3 à 0   (5)
	PORTE  &= 0xF7;
//	DDRE   |= 0x08;
//	end = micro();

	delay_1_uS();

	// PE3 à 1  (5)
	PORTE  |= 0x08;
//	DDRE   |= 0x08;
	delay_5_uS;
	// PE3 à 0   (5)
	PORTE  &= 0xF7;
//	DDRE   |= 0x08;

        printf("Counter: %d\n", counter);
//	printf("j: %d\n", j);
        counter++;

	//delay(1);
       // etimer_reset(&timer);
	}
    }
    PROCESS_END();
}
/*---------------------------------------------------------------------------*/
