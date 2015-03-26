/**
 * \file
 *	Architecture-specific definitions for the SHT11 sensor on Zigduino r2
 * 
 */

#ifndef SHT11_ARCH_H
#define SHT11_ARCH_H

// CAUTION: I2C needs to be disabled to use the bitbang protocol of SHT11
// See /usr/lib/avr/include/avr/iom128rfa1.h
#define SHT11_ARCH_SDA  PORTE7  /* PE7 <=> Pin Zigduino 7 */
#define SHT11_ARCH_SCL  PORTE4  /* PE4 <=> Pin Zigduino 6 */
#define SHT11_ARCH_PWR  PORTE6  /* PE6 <=> Pin Zigduino 2 */

#define SHT11_PxDIR     DDRE    /* #define SHT11_PxDIRSDA       DDRE */ 
#define SHT11_PxOUT     PORTE
#define SHT11_PxIN      PINE
//#define SHT11_PxSEL		PINE7

#endif
