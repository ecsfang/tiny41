#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"

#include "timemod.h"

#ifdef USE_TIME_MODULE
static char bcd[14+1];
void CTime::tick()
{
  // 0.01s = 10ms = 10000us
  int x, n,i, tic;
  uint64_t  tm1 = time_us_64();
  if( (tm1 - tm) > 10000 ) {
    tm1 = tm1-tm;
    tic = tm1 / 1000;
    sprintf(bcd, "%014llx", reg[0].clock);

    x = 14;
    for(i=0; i<x; i++)
      bcd[i] -= '0';
    while( tic ) {
      n = tic % 10;
      tic /= 10;
      for( int i = 13; n && i>0; i--) {
        bcd[i] += n;
        if( bcd[i] <= 9 )
          break;
        bcd[i] -= 10;
        n = 1;
      }
    }
    reg[0].clock = 0;
    for( i = 0; i<14; i++ )
      reg[0].clock = (reg[0].clock<<4) | bcd[i];
    tm += tm1;
  }
}
#endif//USE_TIME_MODULE
