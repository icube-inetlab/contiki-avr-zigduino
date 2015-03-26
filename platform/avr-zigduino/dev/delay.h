/*
 *  delay.h
 *  
 *
 *  Created by Raphaël LUHAHE  on 19/03/15.
 *  Copyright 2015 __MyCompanyName__. All rights reserved.
 *
 */
#ifndef F_CPU
#define F_CPU 16000000
#endif 

#define K_DELAY_100us   F_CPU/61349
#define K_DELAY_1ms             F_CPU/6013
#define K_DELAY_10ms    F_CPU/600

void Delay_100us(unsigned int t);
void Delay_1ms(unsigned int t);
void Delay_10ms(unsigned int t);

