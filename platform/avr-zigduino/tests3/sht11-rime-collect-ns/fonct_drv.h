uint8_t time_In_Eeprom[6];

void spi_init(void); 
void RTC_init(void); 
void SetTimeDate(int d, int mo, int y, int h, int mi, int s);
char * ReadTimeDate(void);

