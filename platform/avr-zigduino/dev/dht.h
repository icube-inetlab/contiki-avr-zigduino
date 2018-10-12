#ifndef dht_h
#define dht_h

#define DHT_LIB_VERSION "0.1.21"

#define DHTLIB_OK                   0
#define DHTLIB_ERROR_CHECKSUM       -1
#define DHTLIB_ERROR_TIMEOUT        -2
#define DHTLIB_ERROR_CONNECT        -3
#define DHTLIB_ERROR_ACK_L          -4
#define DHTLIB_ERROR_ACK_H          -5

#define DHTLIB_DHT11_WAKEUP         18
#define DHTLIB_DHT_WAKEUP           1

#define DHTLIB_DHT11_LEADING_ZEROS  1
#define DHTLIB_DHT_LEADING_ZEROS    6

// max timeout is 100 usec.
// For a 16 Mhz proc 100 usec is 1600 clock cycles
// loops using DHTLIB_TIMEOUT use at least 4 clock cycli
// so 100 us takes max 400 loops
// so by dividing F_CPU by 40000 we "fail" as fast as possible
#ifndef F_CPU
#define DHTLIB_TIMEOUT 1000  // ahould be approx. clock/40000
#else
#define DHTLIB_TIMEOUT (F_CPU/40000)
#endif

struct dht
{
	double humidity;
	double temperature;
	uint8_t bits[5];  // buffer to receive data
	uint8_t temp_h;
	uint8_t temp_l;
	uint8_t humidity_h;
	uint8_t humidity_l;
};

//struct dht DHT;

int8_t dht_read(struct dht *DHT, uint8_t pin);

int8_t readSensor(struct dht *DHT, uint8_t pin, uint8_t wakeupDelay, uint8_t leadingZeroBits);

void print_temp_hum(struct dht *DHT);

#endif /* dht_h */
