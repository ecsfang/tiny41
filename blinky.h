#ifndef __BLINKY_H__
#define __BLINKY_H__

#include "core_bus.h"

#define TIMER_FAST  75
#define TIMER_SLOW  825
#define BUSY_CNT    8

// Trace of real HP82242 shows that FUNC(2) set bit 4
// and FUNC(4) sets bit 6
#define BLINKY_CLK_ENABLE   BIT_4 // Handled by FUNC(2/3)
#define BLINKY_RAM_ENABLE   BIT_6 // Handled by FUNC(4/5)
#define BLINKY_ENABLE       BIT_7
#define BLINKY_SLOW_CLK     BIT_6

extern bool bT0Carry;

// Memory and registers for Blinky module
class CBlinky {
  uint16_t  fiFlags;
  bool      bSelected;
public:
  CBlinky() {
    fiFlags = 0;
    bSelected = true;
  }
  void select(bool bSel) {
    bSelected = bSel;
  }
  volatile static uint64_t  reg[8];    // First 8 registers
  volatile static uint8_t   reg8[16];  // Last 8 registers (0-7 not used)
  uint8_t   flags;
  int       nAlm;
  int       cntTimer;
  int       bwr;
  int       busyCnt;
  // Flag 6 in reg 8 indicates slow or fast clock
  uint16_t  timerCnt() { return reg8[8] & BLINKY_SLOW_CLK ? TIMER_SLOW : TIMER_FAST; }
  bool      tick() {
    // Timer is clocked by a 85Hz (or 7.6Hz) clock, which means that the
    // timer value should decrement every ~75th (or ~825) bus cycle.
    // nAlm keeps track of the bus cycle counting, and when 0
    // timer value is decremented and nAlm restored.
    // Note - this only works when bus is running (i.e not in sleep)
    if( nAlm && (flags & BLINKY_CLK_ENABLE) ) {
      if( --nAlm == 0 ) {
        if( cntTimer ) {
          // Decrement and continue counting ...
          cntTimer--;
          nAlm = timerCnt();
        } else {
          // Set FI flag when counter reaches zero ...
          fiSet(FI_PRT_TIMER);
          return true;
        }
      }
    }
    // Timer idle or still running ...
    return false;
  }
  void wrStatus(uint8_t st) {
    flags = reg8[14] = st;
  }
  inline void write(int r, uint64_t data, bool bRam = false) {
    if( r < 8 ) {
      // Lower 56 bit register 0-7
      reg[r] = data;
    } else if( r < 14 ) {
      // Upper 8 bit register 8-13
      reg8[r] = (uint8_t)(data & 0xFF);
      if( !bRam ) {
        switch(r) {
        case  8:
          if( reg8[8] & BLINKY_ENABLE )
            set(BLINKY_ENABLE);
          break;
        case 10:
          cntTimer = reg8[10];
          if( cntTimer && (flags & BLINKY_CLK_ENABLE) )
            cntTimer--;
          // Writing results in clearing of FI[12]
          fiClr(FI_PRT_BUSY);
          fiClr(FI_PRT_TIMER);
          // Reset timer countdown
          nAlm = timerCnt();
          break;
        case 11:
          if( flags & BLINKY_CLK_ENABLE ) {
            // Write to out buffer - set buffer flag
            // Maybe we should just keep 8 LSB
            prtBuf[wprt++] = reg8[11];
            if( busyCnt ) // Already busy ... ?
              fiSet(FI_PRT_BUSY);
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
      set(BLINKY_ENABLE|BLINKY_CLK_ENABLE);
      break;
    case 3: // Disable timer clock
      clr(BLINKY_CLK_ENABLE);
      break;
    case 4: // Enable RAM write
      set(BLINKY_ENABLE|BLINKY_RAM_ENABLE);
      break;
    case 5: // Disable RAM write
      clr(BLINKY_RAM_ENABLE);
      fiClr(FI_PRT_BUSY);
      fiClr(FI_PRT_TIMER);
      bT0Carry = false;
      break;
    case 7: // Reset
      flags = 0;
      fiClr(FI_PRT_BUSY);
      break;
    case 8: // Clear busy state
      fiClr(FI_PRT_BUSY);
      break;
    }
  }
  void inline busy(void) {
    if(busyCnt) {
      busyCnt--;
      if( !busyCnt )
        fiClr(FI_PRT_BUSY);
    }
  }
  inline void fi(volatile uint16_t *f) {
    // Clear FI-flags
    *f &= ~(FI_PRT_BUSY|FI_PRT_TIMER);
    // Set flags if printer is selected
    if( bSelected )
      *f |= fiFlags;
  }
  inline void set(uint8_t f) {
    flags |= f;
  }
  inline void clr(uint8_t f) {
    flags &= ~f;
  }
  inline void fiSet(uint16_t f) {
    fiFlags |= f;
  }
  inline void fiClr(uint16_t f) {
    fiFlags &= ~f;
  }
};
#endif//__BLINKY_H__
