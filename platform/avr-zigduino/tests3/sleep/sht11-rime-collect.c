#include "contiki.h"
#include <stdio.h>
#include "dev/sht11.h"

#include "lib/random.h"
#include "net/rime.h"
#include "net/rime/collect.h"
#include "net/netstack.h"

#include <avr/sleep.h>
#include <dev/watchdog.h>

/* sink node id */
#define SINK 15
/* sampling sensors every 10 seconds */
#define SAMPLE_INTERVAL_SEC 10
/* average over 15 minutes */
#define AVERAGE_TIME_SEC 30

float temp=0;
float humidity=0;
static struct collect_conn tc;

float sht11_TemperatureC(int rawdata)
{
  int _val;                // Raw value returned from sensor
  float _temperature;      // Temperature derived from raw value

  // Conversion coefficients from SHT11 datasheet
  const float D1 = -39.6;
  const float D2 =   0.01;

  // Fetch raw value
  _val = rawdata;

  // Convert raw value to degrees Celsius
  _temperature = (_val * D2) + D1;

  return (_temperature);
}

float sht11_Humidity(int temprawdata,int humidityrawdata)
{
  int _val;                    // Raw humidity value returned from sensor
  float _linearHumidity;       // Humidity with linear correction applied
  float _correctedHumidity;    // Temperature-corrected humidity
  float _temperature;          // Raw temperature value

  // Conversion coefficients from SHT15 datasheet
  const float C1 = -4.0;       // for 12 Bit
  const float C2 =  0.0405;    // for 12 Bit
  const float C3 = -0.0000028; // for 12 Bit
  const float T1 =  0.01;      // for 14 Bit @ 5V
  const float T2 =  0.00008;   // for 14 Bit @ 5V

  _val = humidityrawdata;
   _linearHumidity = C1 + C2 * _val + C3 * _val * _val;

  // Get current temperature for humidity correction
  _temperature = sht11_TemperatureC(temprawdata);

  // Correct humidity value for current temperature
  _correctedHumidity = (_temperature - 25.0 ) * (T1 + T2 * _val) + _linearHumidity;

  return (_correctedHumidity);
}

char serial_char_received;

/*---------------------------------------------------------------------------*/
/* Sleep for howlong seconds, or until UART interrupt if howlong==0.
 *  Uses TIMER2 with external 32768 Hz crystal to sleep in 1 second multiples.
 *  TIMER2 may have already been set up for CLOCK_CONF_SECOND ticks/second in clock.c
 *
 *  
 *  Until someone figures out how to get UART to wake from powerdown,
 *  a three second powersave cycle is used with exit based on any character received.
 *
 * The system clock is adjusted to reflect the sleep time.
 * */
void micro_sleep(uint8_t howlong)
{
    uint8_t saved_sreg = SREG, saved_howlong = howlong;
#if AVR_CONF_USE32KCRYSTAL
/* Save TIMER2 configuration if clock.c is using it */
    uint8_t savedTCNT2=TCNT2, savedTCCR2A=TCCR2A, savedTCCR2B = TCCR2B, savedOCR2A = OCR2A;
#endif
       set_sleep_mode(SLEEP_MODE_PWR_SAVE);    // Sleep for howlong seconds
        if (howlong==0) howlong=3;              // 3*32/(32768/1024) = 3 second sleep cycle if not specified
        cli();                                  // Disable interrupts for the present
        ASSR |= (1 << AS2);                     // Set TIMER2 asyncronous from external crystal
        TCCR2A =(1<<WGM21);                     // CTC mode
        TCCR2B =((1<<CS22)|(1<<CS21)|(1<<CS20));// Prescale by 1024 = 32 ticks/sec
        OCR2A = howlong*32;                     // Set TIMER2 output compare register
        SREG = saved_sreg;                      // Restore interrupt state.

    TCNT2 = 0;                                  // Reset timer
    watchdog_stop();                            // Silence annoying distractions
    while(ASSR & (1 << TCN2UB));		// Wait for TCNT2 write to (which assures TCCR2x and OCR2A are finished!)
    TIMSK2 |= (1 << OCIE2A);                    // Enable TIMER2 output compare interrupt
    SMCR |= (1 <<  SE);                         // Enable sleep mode.
    while (1) {
       serial_char_received=0;                  // Set when chars received by UART
       sleep_mode();                            // Sleep

       /* Adjust clock.c for the time spent sleeping */
       //extern void clock_adjust_ticks(uint16_t howmany);
       clock_adjust_ticks(howlong * CLOCK_SECOND);
       printf(".");
       if (saved_howlong) break;                // Exit sleep if nonzero time specified
       if (serial_char_received) break;
    }
    SMCR  &= ~(1 << SE);                        //Disable sleep mode after wakeup
#if AVR_CONF_USE32KCRYSTAL
/* Restore clock.c configuration */
cli();
    TCCR2A = savedTCCR2A;
    TCCR2B = savedTCCR2B;
    OCR2A  = savedOCR2A;
    TCNT2  = savedTCNT2;
    sei();
#else
    TIMSK2 &= ~(1 << OCIE2A);                   //Disable TIMER2 interrupt
#endif

    watchdog_start();
}
#if !AVR_CONF_USE32KCRYSTAL
/*---------------------------------------------------------------------------*/
/* TIMER2 Interrupt service */

ISR(TIMER2_COMPA_vect)
{
//    TIMSK2 &= ~(1 << OCIE2A);       //Just one interrupt needed for waking
}
#endif /* !AVR_CONF_USE32KCRYSTAL */

#if DEBUGSERIAL
uint8_t serialcount;
char dbuf[30];
#endif




/*---------------------------------------------------------------------------*/
PROCESS(example_collect_process, "Test collect process");
AUTOSTART_PROCESSES(&example_collect_process);
/*---------------------------------------------------------------------------*/
static void
recv(const rimeaddr_t *originator, uint8_t seqno, uint8_t hops)
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
	
	static float sum_temp=0;
	static float sum_humidity=0;
	static int count=0;
	static int total_samples=AVERAGE_TIME_SEC/SAMPLE_INTERVAL_SEC;

	PROCESS_BEGIN();
	printf("[INIT] Node %d must begin\n", rimeaddr_node_addr.u8[7]);
	printf("[INIT] MAC address %x:%x:%x:%x:%x:%x:%x:%x\n",
			rimeaddr_node_addr.u8[0],
			rimeaddr_node_addr.u8[1],
			rimeaddr_node_addr.u8[2],
			rimeaddr_node_addr.u8[3],
			rimeaddr_node_addr.u8[4],
			rimeaddr_node_addr.u8[5],
			rimeaddr_node_addr.u8[6],
			rimeaddr_node_addr.u8[7]);
	
	collect_open(&tc, 130, COLLECT_ROUTER, &callbacks);

	if(rimeaddr_node_addr.u8[7] == SINK)
	{
		printf("[INIT] I am sink\n");
		collect_set_sink(&tc, 1);
	}

	/* Allow some time for the network to settle. */
	etimer_set(&periodic, 20 * CLOCK_SECOND);
	PROCESS_WAIT_UNTIL(etimer_expired(&periodic));

	while(1) {
		/* Sample sensors every X seconds. */
		etimer_set(&et, CLOCK_SECOND * SAMPLE_INTERVAL_SEC);
		printf("New cycle\n");

		PROCESS_WAIT_EVENT();
		if(etimer_expired(&et)) {
			sht11_init();
			/* Read temperature value. */
			clock_delay_msec(20);
			unsigned int raw_temp = sht11_temp();
			/* Read humidity value. */
			clock_delay_msec(20);
			unsigned int raw_humidity = sht11_humidity();
			  
			temp = sht11_TemperatureC(raw_temp);
			humidity = sht11_Humidity(raw_temp,raw_humidity);
			  
			printf("Acquire temp:%u.%u humidity:%u.%u\n",(int)temp,((int)(temp*10))%10 , (int)humidity,((int)(humidity*10))%10);
			
			sum_temp += temp;
			sum_humidity += humidity;
			count++;
			
			printf("count=%d/%d\n", count, total_samples);
			
			if (count == total_samples)
			{
				
				printf("node_sending;uid=%d.%d;payload=temp:%u.%u:humidity:%u.%u\n", 
						rimeaddr_node_addr.u8[6], 
						rimeaddr_node_addr.u8[7], 
						(int)(sum_temp / total_samples),
						((int)((sum_temp / total_samples)*10))%10, 
						(int) (sum_humidity / total_samples),
						((int)((sum_humidity / total_samples)*10))%10);
				packetbuf_clear();
					
				packetbuf_set_datalen(sprintf(packetbuf_dataptr(),
						"temp:%u.%u:humidity:%u.%u", 
						(int)(sum_temp / total_samples),
						((int)((sum_temp / total_samples)*10))%10, 
						(int) (sum_humidity / total_samples),
						((int)((sum_humidity / total_samples)*10))%10)
						+ 1);
				collect_send(&tc, 15);
				count=0;
				sum_temp=0;
				sum_humidity=0;
			}
		}
	}
	
	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
