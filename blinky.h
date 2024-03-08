#ifndef __BLINKY_H__
#define __BLINKY_H__

#include "core_bus.h"

#define TIMER_CNT1  75
#define TIMER_CNT2  825
#define BUSY_CNT    8

#define BLINKY_RAM_ENABLE   BIT_4
#define BLINKY_CLK_ENABLE   BIT_6
#define BLINKY_ENABLE       BIT_7

// Memory and registers for Blinky module
class CBlinky {
public:
  CBlinky() {
  }
  volatile static uint64_t  reg[8];    // First 8 registers
  volatile static uint8_t   reg8[16];  // Last 8 registers (0-7 not used)
  uint8_t   flags;
  int       nAlm;
  int       cntTimer;
  int       bwr;
  int       busyCnt;
  uint8_t   timerCnt() { return reg[8] & 0x40 ? TIMER_CNT2 : TIMER_CNT1; }
  int       tick() {
    // Timer is clocked by a 85Hz (or XXX Hz) or clock, which means that the
    // timer value should decrement every ~75th (or 825) bus cycle.
    // nAlm keeps track of the bus cycle counting, and when 0
    // timer value is decremented and nAlm restored.
    if( nAlm && (flags & BLINKY_CLK_ENABLE) ) {
      if( --nAlm == 0 ) {
        if( cntTimer ) {
          // Decrement and continue counting ...
          cntTimer--;
          nAlm = timerCnt(); //reg8[8] & 0x40 ? TIMER_CNT2 : TIMER_CNT1;
        } else {
          // Set FI flag when counter reaches zero ...
          return 1;
        }
      }
    }
    // Timer idle or still running ...
    return 0;
  }
  void wrStatus(uint8_t st) {
    flags = reg8[14] = st;
  }
  inline void write(int r, uint64_t data, bool bRam = false) {
    if( r < 8 ) {
      // 56 bit register
      reg[r] = data;
    } else {
      // 8 bit register
      reg8[r] = (uint8_t)(data & 0xFF);
      if( !bRam ) {
        switch(r) {
        case  8:
          if( reg8[8] & BLINKY_ENABLE )
            set(BLINKY_ENABLE);
          break;
        case 10:
          cntTimer = reg8[10];
          if( flags & BLINKY_CLK_ENABLE )
            cntTimer--;
          // Writing results in clearing of FI[12]
          clrFI(FI_PRT_BUSY);
          clrFI(FI_PRT_TIMER);
          // Reset timer countdown
          // Flag 6 in reg 8 indicates slow clock
          nAlm = timerCnt();
          break;
        case 11:
          if( flags & BLINKY_CLK_ENABLE ) {
            // Write to out buffer - set buffer flag
            // Maybe we should just keep 8 LSB
            prtBuf[wprt++] = reg8[11];
            if( busyCnt ) // Already busy ... ?
              setFI(FI_PRT_BUSY);
            busyCnt = BUSY_CNT;
          }
          break;
        }
      }
    }
  }
  inline uint64_t read(int r) {
    // Enable data driver ...
    if( r < 8 ) {
      return reg[r];
    } else {
      // Update hardware registers
      switch(r) {
      case 10:  // Timer register
        reg8[10] = cntTimer;
        break;
      case 14:  // HW flag register
        reg8[14] = flags;
        break;
      }
      return reg8[r];
    }
  }
  void func(int c) {
    switch( c ) {
    case 2: // Enable timer clock
      set(BLINKY_CLK_ENABLE);
      break;
    case 3: // Disable timer clock
      clr(BLINKY_CLK_ENABLE);
      break;
    case 4: // Enable RAM write
      set(BLINKY_RAM_ENABLE);
      break;
    case 5: // Disable RAM write
      clr(BLINKY_RAM_ENABLE);
      break;
    case 7: // Reset
      flags = 0;
      clrFI(FI_PRT_BUSY);
      break;
    case 8: // Clear buffer
      clrFI(FI_PRT_BUSY);
      clrFI(FI_PRT_TIMER);
      cntTimer = 0;
      break;
    }
  }
  void inline busy(void) {
    if(busyCnt) {
      busyCnt--;
      if( !busyCnt )
        clrFI(FI_PRT_BUSY);
    }
  }  
  inline void set(uint8_t f) {
    flags |= f;
  }
  inline void clr(uint8_t f) {
    flags &= ~f;
  }
};
#endif//__BLINKY_H__
