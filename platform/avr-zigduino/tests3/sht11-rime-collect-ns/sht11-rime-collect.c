#include "contiki.h"
#include <stdio.h>

#include "dev/sht11.h"
#include "dev/i2c-drv.h"
#include "dev/spi.h"

#include "lib/random.h"
#include "net/rime.h"
#include "net/rime/collect.h"
#include "net/rime/broadcast-announcement.h"
#include "net/netstack.h"

/* sink node id */
#define SINK 12
/* sampling sensors every 10 seconds */
#define SAMPLE_INTERVAL_SEC 10
/* sampling average in seconds */
#define AVERAGE_TIME_SEC 60
/* true if SHT75 is plugged*/
#define SENSOR
#define NETWORK_STATS

char val_tempsDate[15];
uint8_t time_In_Eeprom[6];
uint8_t nodeNumber;
uint16_t nodeNumber_addr = 0x0;

uint8_t base_add_EEP1 = 0x50;  // EEprom1 device address	
uint16_t addr = 0x1FF; // start acquisition address 0x200 - 1


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


/******************************* spi ***************************/
void spi_init() {
    pin_Mode_SS_OUTPUT(); // chip select 
    // start the SPI library: 
    SPI_begin(); 
    SPI_setBitOrder(MSBFIRST); 
    SPI_setDataMode(SPI_MODE1); // both mode 1 & 3 should work 

}

//****************************************************************
void RTC_init(void){ 
                  
         uint8_t ad;
          
	  // start the SPI library:
	  //spi_init();   //SPI.begin();
	  //SPI.setBitOrder(MSBFIRST); 
	  //SPI.setDataMode(SPI_MODE1); // both mode 1 & 3 should work 
	  //set control register 
	  digital_Write_SS_Low(); // 
	  SPI_transfer(0x8E);// 
	  //0x25 = disable Osciallator, Battery and SQ wave , temp compensation, Alarms enabled 10S
	  SPI_transfer(0x25);//  
	  digital_Write_SS_HIGH();//
	  
          clock_delay_msec(10);//     
         
          digital_Write_SS_Low();//
	  ad=0x8F;
          SPI_transfer(0x8F);//A1F = 0;  Flag Alarm1
	  ad=0;
          SPI_transfer(0x0);//SPI.transfer(0x0); 
          digital_Write_SS_HIGH();//digitalWrite(cs, HIGH); 
         
          clock_delay_msec(10);//delay(10); 



         
        //Turn on internal pullup for INT/SQW pin 
/*
          alarmPin_In();    //pinMode(alarmPin, INPUT); 
          alarmPin_High();  //digitalWrite(alarmPin, HIGH);   
          SetAlarm();
*/
}


//=====================================
void SetTimeDate(int d, int mo, int y, int h, int mi, int s){ 
	int TimeDate [7]={s,mi,h,0,d,mo,y};
	uint8_t ad;
	int i, a, b;

	for(i=0; i<=6;i++){
		if(i==3)
			i++;
		b= TimeDate[i]/10;
		a= TimeDate[i]-b*10;
		if(i==2){
			if (b==2)
				b= 0x02;  //B00000010;
			else if (b==1)
				b= 0x01;//B00000001;
		}	
		TimeDate[i]= a+(b<<4);
		  
		digital_Write_SS_Low();//digitalWrite(cs, LOW);
		ad = i+0x80;
		SPI_transfer(i+0x80);//SPI.transfer(i+0x80); 
		ad =(uint8_t) TimeDate[i];
		SPI_transfer(ad);//SPI.transfer(TimeDate[i]);        
		digital_Write_SS_HIGH();//digitalWrite(cs, HIGH);
  }
}
//=====================================
char * ReadTimeDate(void){

	//uint8_t ad;
	int i, a, b;
 	uint8_t n;
	int TimeDate [7]; //second,minute,hour,null,day,month,year	
	
	for(i=0; i<=6;i++){
		if(i==3)
			i++;
		digital_Write_SS_Low();
		SPI_transfer(i+0x00);
		n = SPI_transfer(0x00);        
		digital_Write_SS_HIGH();
		a=n & 0x0F;//B00001111;    
		if(i==2){	
			b=(n & 0x30)>>4; //  B00110000  24 hour mode
			if(b== 0x02)     // B00000010
				b=20;        
			else if(b==0x01)     // B00000001 
				b=10;
			TimeDate[i]=a+b;
		}
		else if(i==4){
			b=(n & 0x30)>>4;  //B00110000
			TimeDate[i]=a+b*10;
		}
		else if(i==5){
			b=(n & 0x10)>>4;      //B00010000
			TimeDate[i]=a+b*10;
		}
		else if(i==6){
			b=(n &0x0F0 )>>4;          //B11110000
			TimeDate[i]=a+b*10;
		}
		else{	
			b=(n & 0x70)>>4;          //B01110000
			TimeDate[i]=a+b*10;	
			}
	}
	sprintf(val_tempsDate,"%d/%d/%d %d:%d:%d", TimeDate[4],TimeDate[5],TimeDate[6],TimeDate[2],TimeDate[1],TimeDate[0]);
	time_In_Eeprom[0]= TimeDate[4]; // day
	time_In_Eeprom[1]= TimeDate[5]; // month
	time_In_Eeprom[2]= TimeDate[6]; // year
	time_In_Eeprom[3]= TimeDate[2]; // hour
	time_In_Eeprom[4]= TimeDate[1]; // minute
	time_In_Eeprom[5]= TimeDate[0]; // second

  return(val_tempsDate);
}
//====================================================

/*---------------------------------------------------------------------------*/
PROCESS(example_collect_process, "Test collect process");
AUTOSTART_PROCESSES(&example_collect_process);
/*---------------------------------------------------------------------------*/
static void
recv(const rimeaddr_t *originator, uint8_t seqno, uint8_t hops)
{
  printf("sink_received;from=%d.%d;seqno=%d;hops=%d;len=%d;payload=%s;\n",
#if RIMEADDR_SIZE == 8	
         originator->u8[6], originator->u8[7],
#else
		 originator->u8[0], originator->u8[1],
#endif
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
	static int count=0;
	static int total_samples=AVERAGE_TIME_SEC/SAMPLE_INTERVAL_SEC;

	int i;

#ifdef SENSOR
	static float sum_temp=0;
	static float sum_humidity=0;
	
	static uint32_t sum_raw_t=0;
	static uint32_t sum_raw_h=0;
	
	uint8_t data_h=0;
	uint8_t data_l=0;

	uint16_t mask_h = 0xFF00;
	uint16_t mask_l = 0x00FF;
	
#endif

#ifdef NETWORK_STATS
	// test metrics
	const rimeaddr_t *parent;
	uint8_t paddr[2];
	int depth;
	struct collect_neighbor *n;
	uint16_t parent_etx;
	uint16_t num_neighbors;
	uint16_t beacon_interval;
#endif

	PROCESS_BEGIN();
	
#if RIMEADDR_SIZE == 8	
	nodeNumber = rimeaddr_node_addr.u8[7];
#else
	nodeNumber = rimeaddr_node_addr.u8[1];
#endif	
	printf("[INIT] Node %d must begin\n", nodeNumber);

	i2c_eeprom_write_byte(base_add_EEP1,nodeNumber_addr, nodeNumber); clock_delay_msec(5);

	collect_open(&tc, 130, COLLECT_ROUTER, &callbacks);

#if RIMEADDR_SIZE == 8	
	if(rimeaddr_node_addr.u8[7] == SINK)
#else
	if(rimeaddr_node_addr.u8[1] == SINK)
#endif
	{
		printf("[INIT] I am sink\n");
		collect_set_sink(&tc, 1);
	}

	/* Allow some time for the network to settle. */
	etimer_set(&periodic, 20 * CLOCK_SECOND);
	PROCESS_WAIT_UNTIL(etimer_expired(&periodic));

/**************************  Date and Time *********************/
	spi_init();
	RTC_init();
	SetTimeDate(29,7,15,13,20,0); 
	printf("UTC date and time ---start : %s\n",ReadTimeDate());
	

/***************************************************************/

//-------------------------- Read Node number in eeprom ----------------------

	//i2c_eeprom_write_byte(base_add_EEP1,nodeNumber_addr, nodeNumber); clock_delay_msec(5);
	printf(" EEprom node number = %u\n", i2c_eeprom_read_byte(base_add_EEP1, nodeNumber_addr)); // read eeprom 
	//clock_delay_msec(5); 
//--------------------------------------------------------------


	while(1) {
		/* Sample sensors every X seconds. */
		etimer_set(&et, CLOCK_SECOND * SAMPLE_INTERVAL_SEC);
		printf("New cycle\n");
		printf(" EEprom node number = %u\n", i2c_eeprom_read_byte(base_add_EEP1, nodeNumber_addr)); 			//clock_delay_msec(5);

		PROCESS_WAIT_EVENT();
		if(etimer_expired(&et)) {

#ifdef SENSOR
			// Read Time
			printf("UTC date and time : %s\n",ReadTimeDate());
			//clock_delay_msec(20);

			sht11_init();
			/* Read temperature value. */
			clock_delay_msec(20);
			unsigned int raw_temp = sht11_temp();
			/* Read humidity value. */
			clock_delay_msec(20);
			unsigned int raw_humidity = sht11_humidity();
			  
			temp = sht11_TemperatureC(raw_temp);
			humidity = sht11_Humidity(raw_temp,raw_humidity);
			
			printf("raw_temp %u\n",raw_temp);

			uint16_t data_h16 = mask_h & raw_temp;
			data_h = (uint8_t) (data_h16 >> 8);
			data_l = mask_l & raw_temp;
			
			raw_temp = (data_h << 8) + data_l;   
			printf("read i2c %u\n", raw_temp);
			
			printf("Acquire temp:%u.%u humidity:%u.%u\n",(int)temp,((int)(temp*10))%10 , (int)humidity,((int)(humidity*10))%10);
			
			sum_raw_t += raw_temp;
			sum_raw_h += raw_humidity;

			sum_temp += temp;
			sum_humidity += humidity;	
#endif
			count++;
			printf("count=%d/%d\n", count, total_samples);

#ifdef NETWORK_STATS
			// send network stats every 3*SAMPLE_INTERVAL_SEC sec.
			if (count%3 == 0 && count != total_samples)
			{
				parent = collect_parent(&tc);
#if RIMEADDR_SIZE == 8	
				paddr[0]=parent->u8[6];
				paddr[1]=parent->u8[7];
#else
				paddr[0]=parent->u8[0];
				paddr[1]=parent->u8[1];
#endif
			
				depth = collect_depth(&tc);
				n = collect_neighbor_list_find(&tc.neighbor_list,
									   &tc.parent);
				if(n != NULL) {
				   parent_etx = collect_neighbor_link_estimate(n);
				} else {
				   parent_etx = 0;
				}
				num_neighbors = collect_neighbor_list_num(&tc.neighbor_list);
				beacon_interval = broadcast_announcement_beacon_interval() / CLOCK_SECOND;
				
				printf("node_sending;payload=parent:%d.%d:depth:%d:etx:%d:rtmetric:%d:neighbors:%d:beacon:%d\n",
					paddr[0],paddr[1],depth,parent_etx,tc.rtmetric,
					num_neighbors,beacon_interval);

				packetbuf_clear();
				packetbuf_set_datalen(sprintf(packetbuf_dataptr(), "parent:%d.%d:depth:%d:etx:%d:rtmetric:%d:neighbors:%d:beacon:%d",
				  paddr[0],paddr[1],depth,parent_etx,tc.rtmetric,
				  num_neighbors,beacon_interval) + 1);
				collect_send(&tc, 15);
			}
#endif

			if (count == total_samples)
			{
						
				data_h16 = (uint16_t)(sum_raw_t / total_samples);
				data_l = mask_l & data_h16;
				data_h = (uint8_t) (data_h16 >> 8);
				

			/********* write data in eepromm  and read data from eeprom ***********/
				
			// !!! wait a little between i2c instruction !!! 

				// temp
				addr++;
				i2c_eeprom_write_byte(base_add_EEP1,addr, data_l); clock_delay_msec(5);
				data_l = 0;
				data_l = i2c_eeprom_read_byte(base_add_EEP1, addr); clock_delay_msec(5);

				addr++;
				i2c_eeprom_write_byte(base_add_EEP1,addr, data_h); clock_delay_msec(5);
				data_h = 0;
				data_h = i2c_eeprom_read_byte(base_add_EEP1, addr); 

				raw_temp = (data_h << 8) + data_l;   
				printf("read eeprom temp : %u\n", raw_temp);


				// humidity
				data_h16 = (uint16_t)(sum_raw_h / total_samples);
				data_l = mask_l & data_h16;
				data_h = (uint8_t) (data_h16 >> 8);


				addr++;
				i2c_eeprom_write_byte(base_add_EEP1,addr, data_l); clock_delay_msec(5);
				data_l = 0;
				data_l = i2c_eeprom_read_byte(base_add_EEP1, addr); clock_delay_msec(5);

				addr++;
				i2c_eeprom_write_byte(base_add_EEP1,addr, data_h); clock_delay_msec(5);
				data_h = 0;
				data_h = i2c_eeprom_read_byte(base_add_EEP1, addr); 

				raw_humidity = (data_h << 8) + data_l;   
				printf("read eeprom humidity : %u\n", raw_humidity);

			/***********************************************************************/

			/************  write date and time in eeprom ***************************/
				for(i=0;i<6; i++)
				{
					addr++;
					i2c_eeprom_write_byte(base_add_EEP1,addr, time_In_Eeprom[i]); 
					clock_delay_msec(5);			
				}
			/***********************************************************************/	
#ifdef SENSOR				
				printf("node_sending;uid=%d.%d;payload=temp:%u.%u:humidity:%u.%u\n",
#if RIMEADDR_SIZE == 8
				rimeaddr_node_addr.u8[6], 
				rimeaddr_node_addr.u8[7], 
#else
				rimeaddr_node_addr.u8[0], 
				rimeaddr_node_addr.u8[1],
#endif 				
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


#endif				
				count=0;
#ifdef SENSOR			
				sum_temp=0;
				sum_humidity=0;
				
				sum_raw_h=0;
				sum_raw_t=0;
#endif
			}


		}
	}
	
	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
