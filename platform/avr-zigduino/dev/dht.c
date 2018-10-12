#include <stdint.h>
#include <stdio.h>
#include "dht.h"
#include "Arduino.h"


int8_t readSensor(uint8_t pin, uint8_t wakeupDelay, uint8_t leadingZeroBits)
{
    // INIT BUFFERVAR TO RECEIVE DATA
    uint8_t mask = 128;
    uint8_t idx = 0;

    uint8_t data = 0;
    uint8_t state = LOW;
    uint8_t pstate = LOW;
    uint16_t zeroLoop = DHTLIB_TIMEOUT;
    uint16_t delta = 0;

    leadingZeroBits = 40 - leadingZeroBits; // reverse counting...

    // replace digitalRead() with Direct Port Reads.
    // reduces footprint ~100 bytes => portability issue?
    // direct port read is about 3x faster
    uint8_t bit = digitalPinToBitMask(pin);
    uint8_t port = digitalPinToPort(pin);
    volatile uint8_t *PIR = portInputRegister(port);

    // REQUEST SAMPLE
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW); // T-be
    delayMicroseconds(wakeupDelay * 1000UL);
    digitalWrite(pin, HIGH); // T-go
    pinMode(pin, INPUT);

    uint16_t loopCount = DHTLIB_TIMEOUT * 2;  // 200uSec max
    // while(digitalRead(pin) == HIGH)
    while ((*PIR & bit) != LOW )
    {
        if (--loopCount == 0) return DHTLIB_ERROR_CONNECT;
    }

    // GET ACKNOWLEDGE or TIMEOUT
    loopCount = DHTLIB_TIMEOUT;
    // while(digitalRead(pin) == LOW)
    while ((*PIR & bit) == LOW )  // T-rel
    {
        if (--loopCount == 0) return DHTLIB_ERROR_ACK_L;
    }

    loopCount = DHTLIB_TIMEOUT;
    // while(digitalRead(pin) == HIGH)
    while ((*PIR & bit) != LOW )  // T-reh
    {
        if (--loopCount == 0) return DHTLIB_ERROR_ACK_H;
    }

    loopCount = DHTLIB_TIMEOUT;

    // READ THE OUTPUT - 40 BITS => 5 BYTES
    uint8_t i;
    for (i = 40; i != 0; )
    {
        // WAIT FOR FALLING EDGE
        state = (*PIR & bit);
        if (state == LOW && pstate != LOW)
        {
            if (i > leadingZeroBits) // DHT22 first 6 bits are all zero !!   DHT11 only 1
            {
                zeroLoop = min(zeroLoop, loopCount);
                delta = (DHTLIB_TIMEOUT - zeroLoop)/4;
            }
            else if ( loopCount <= (zeroLoop - delta) ) // long -> one
            {
                data |= mask;
            }
            mask >>= 1;
            if (mask == 0)   // next byte
            {
                mask = 128;
                DHT.bits[idx] = data;
                idx++;
                data = 0;
            }
            // next bit
            --i;

            // reset timeout flag
            loopCount = DHTLIB_TIMEOUT;
        }
        pstate = state;
        // Check timeout
        if (--loopCount == 0)
        {
            return DHTLIB_ERROR_TIMEOUT;
        }

    }
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH);

    return DHTLIB_OK;
}

int8_t dht_read(uint8_t pin)
    {
        // READ VALUES
        int8_t result = readSensor(pin, DHTLIB_DHT_WAKEUP, DHTLIB_DHT_LEADING_ZEROS);

        // these bits are always zero, masking them reduces errors.
        DHT.bits[0] &= 0x03;
        DHT.bits[2] &= 0x83;

        // CONVERT AND STORE
        DHT.humidity = (DHT.bits[0]*256 + DHT.bits[1]) * 0.1;
        DHT.temperature = ((DHT.bits[2] & 0x7F)*256 + DHT.bits[3]) * 0.1;
        if (DHT.bits[2] & 0x80)  // negative temperature
        {
            DHT.temperature = -DHT.temperature;
        }

        DHT.temp_h = (int)DHT.temperature;
        DHT.temp_l = ((int)(DHT.temperature *10))%10;

        DHT.humidity_h = (int)DHT.humidity;
        DHT.humidity_l = ((int)(DHT.humidity*10))%10;

        // TEST CHECKSUM
        uint8_t sum = DHT.bits[0] + DHT.bits[1] + DHT.bits[2] + DHT.bits[3];
        if (DHT.bits[4] != sum)
        {
            return DHTLIB_ERROR_CHECKSUM;
        }
        return result;

    }

void print_temp_hum( struct dht *DHT ) {

   printf( "temperature : %u.%u *C ", DHT->temp_h, DHT->temp_l);
   printf( "humidity : %u.%u %% ", DHT->humidity_h, DHT->humidity_l);
}