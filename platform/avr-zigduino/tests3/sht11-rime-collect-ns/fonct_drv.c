#include "contiki.h"
#include <stdio.h>
#include "fonct_drv.h"
#include "dev/i2c-drv.h"
#include "dev/spi.h"

char val_tempsDate[15];

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
	  spi_init();   //SPI.begin();
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


