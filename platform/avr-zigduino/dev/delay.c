/*
 *  delay.c
 *  
 *
 *  Created by Raphaël LUHAHE  on 19/03/15.
 *  Copyright 2015 __MyCompanyName__. All rights reserved.
 *
 */

#include "delay.h"
 
//----------------------------------------------------------------------------
// Wait for a specific time in 100 uSec
// (15 + t*( ((K_DELAY_100us-1)*6)+5 ))
//----------------------------------------------------------------------------
void Delay_100us(unsigned int t) {
  unsigned int i;
  if (t==0) return;
  while (t--) for(i=0;i<K_DELAY_100us; i++);
}
//----------------------------------------------------------------------------
// Wait for a specific time in 1 mSec
// (15 + t*( ((K_DELAY_1ms-1)*6)+5 ))
//----------------------------------------------------------------------------
void Delay_1ms(unsigned int t) {
  unsigned int i;
  if (t==0) return;
  while (t--) for(i=0;i<K_DELAY_1ms; i++);
}
//----------------------------------------------------------------------------
// Wait for a specific time in 10 mSec
// (15 + t*( ((K_DELAY_10ms-1)*6)+5 ))
//----------------------------------------------------------------------------
void Delay_10ms(unsigned int t) {
  unsigned int i;
  if (t==0) return;
  while (t--) for(i=0;i<K_DELAY_10ms; i++);
}

