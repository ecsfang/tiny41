#ifndef __TIMEMOD_H__
#define __TIMEMOD_H__

#include "core_bus.h"

// Memory and registers for Time module
typedef struct {
  uint64_t  clock;
  uint64_t  alarm;
  uint64_t  scratch;
} CRegs_t;
enum {
  PT_A,
  PT_B
};

enum {
  ALMA  = 1<<0,    // 0 set if ALARM A register is the same as CLOCK A 
  DTZA  = 1<<1,    // 1 set if an overflow has occurred in CLOCK A
  ALMB  = 1<<2,    // 2 set if ALARM B register is the same as CLOCK B
  DTZB  = 1<<3,    // 3 set if an overflow has occurred in CLOCK B
  DTZIT = 1<<4,    // 4 set if the interval timer has counted a whole interval
  PUS   = 1<<5,    // 5 set if timer chip supply voltage has been low
  CKAEN = 1<<6,    // 6 set if CLOCK A is counting forwards
  CKBEN = 1<<7,    // 7 set if CLOCK B is counting forwards
  ALAEN = 1<<8,    // 8 set if ALARM A is enabled (usually set)
  ALBEN = 1<<9,    // 9 set if ALARM B is enabled (usually clear)
  ITEN  = 1<<10,   // 10 set if the interval timer is running
  TESTB = 1<<11,   // 11 timer is in TEST B mode
  TESTA = 1<<12    // 12 timer is in TEST A mode
};

class CTime {
  uint64_t tm;
public:
  CTime() {
    pt = PT_A;
    memset((void*)reg, 0xFF, sizeof(CRegs_t)*2);
    interval = 0x111111;
    accuracy = 0x2222;
    status   = PUS;
    bwr = 0;
  }
  volatile static CRegs_t reg[2];    // 56 bits time regs
  volatile static uint32_t  interval;  // 20 bits interval timer
  volatile static uint16_t  accuracy;  // 13 bits
  volatile static uint16_t  status;    // 20 bits
  int       pt;   // Current pointer
  int       bwr;  // Delayed write
  inline void write(uint64_t *data) {
    switch( bwr-1 ) {
    case 000: // WRTIME
      reg[pt].clock = *data;
      tm = time_us_64();
      break;
    case 001: // WDTIME
      reg[pt].clock = *data;
      tm = time_us_64();
      break;
    case 002: // WRALM
      reg[pt].alarm = *data;
      break;
    case 003: // WRSTS
      if( pt == PT_A ) {
        // Only first 6 bits can be cleard, rest untuched ...
        status &= *data | 0x1FC0;
      } else {
        accuracy = (*data >> 4) & 0x1FFF;
      }
      break;
    case 004: // WRSCR
      reg[pt].scratch = *data;
      break;
    case 005: // WSINT
      interval = *data & 0xFFFFF;
      break;
    case 007: // Stop interval timer
      status &= ~ITEN;
      break;
    case 010: // Clear test mode
#define CFLAG(a,b) status &= ~((pt==PT_A)?a:b)
#define SFLAG(a,b) status |= ((pt==PT_A)?a:b)
      CFLAG(TESTA,TESTB);
      break;
    case 011: // Set test mode
      SFLAG(TESTA,TESTB);
      break;
    case 012: // Disable alarm
      CFLAG(ALAEN,ALBEN);
      break;
    case 013: // Enable alarm
      SFLAG(ALAEN,ALBEN);
      break;
    case 014: // Stop clock
      CFLAG(CKAEN,CKBEN);
      break;
    case 015: // Start clock
      SFLAG(CKAEN,CKBEN);
      break;
    case 016: // PT=B
      pt=PT_B;
      break;
    case 017: // PT=A
      pt=PT_A;
      break;
    }
    bwr = 0;
  }
  inline uint64_t read(int r) {
    switch( r ) {
    case 000: // RDTIME
      return reg[pt].clock;
    case 001: // RCTIME
      return reg[pt].clock;
    case 002: // RDALM
      return reg[pt].alarm;
    case 003: // WRSTS
      if( pt == PT_A ) {
        return status & 0x1FFF;
      } else {
        return (accuracy & 0x1FFF)  << 4;
      }
      break;
    case 004: // RDSCR
      return reg[pt].scratch;
    case 005: // RDINT
      return interval & 0xFFFFF;
    }
    return 0;
  }
  void tick();
};

#endif//__TIMEMOD_H__
