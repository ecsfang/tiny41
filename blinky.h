#ifndef __BLINKY_H__
#define __BLINKY_H__

#include "tiny41.h"
#include "ramdev.h"
#include "xfmem.h"

#define TIMER_FAST  75
#define TIMER_SLOW  825
#define BUSY_CNT    80    // Delay of about 12.4 ms

// Trace of real HP82242 shows that FUNC(2) set bit 4
// and FUNC(4) sets bit 6
#define BLINKY_CLK_ENABLE   BIT_4 // Handled by FUNC(2/3)
#define BLINKY_RAM_ENABLE   BIT_6 // Handled by FUNC(4/5)
#define BLINKY_ENABLE       BIT_7
#define BLINKY_SLOW_CLK     BIT_6

#define STATUS_REG          8
#define TIMER_REG           10
#define TX_REG              11
#define FLAGS_REG           14

// Memory and registers for Blinky module
class CBlinky : public CRamDev {
  uint16_t  m_fiFlags;  // Current FI flags
  bool      bSelected;  // True when selected (FI visible)
  bool      bOutBuf;    // True when character is in output buffer
public:
  CBlinky() {
    m_fiFlags = 0;
    bSelected = true;
    bOutBuf = false;
  }
  void select(bool bSel) {
    bSelected = bSel;
  }
  volatile static uint64_t  reg[8];    // First 8 registers
  volatile static uint8_t   reg8[16];  // Last 8 registers (0-7 not used)
  int       nAlm;
  int       busyCnt;
  // Flag 6 in reg 8 indicates slow or fast clock
  uint16_t  timerCnt() { return reg8[STATUS_REG] & BLINKY_SLOW_CLK ? TIMER_SLOW : TIMER_FAST; }
  bool      tick() {
    // Timer is clocked by a 85Hz (or 7.6Hz) clock, which means that the
    // timer value should decrement every ~75th (or ~825) bus cycle.
    // nAlm keeps track of the bus cycle counting, and when 0
    // timer value is decremented and nAlm restored.
    // Note - this only works when bus is running (i.e not in sleep)
    if( nAlm && (reg8[FLAGS_REG] & BLINKY_CLK_ENABLE) ) {
      if( --nAlm == 0 ) {
        // Check timer register
        if( reg8[TIMER_REG] ) {
          // Decrement and continue counting ...
          reg8[TIMER_REG]--;
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
    reg8[FLAGS_REG] = st;
  }
  inline void write(int r, uint64_t *data, bool bRam = false) {
    if( r < 8 ) {
      // Lower 56 bit register 0-7
      reg[r] = *data;
    } else if( r < 14 ) {
      // Upper 8 bit register 8-13
      reg8[r] = (uint8_t)(*data & 0xFF);
      if( !bRam ) {
        switch(r) {
        case STATUS_REG:
          if( reg8[STATUS_REG] & BLINKY_ENABLE )
            set(BLINKY_ENABLE);
          break;
        case TIMER_REG:
          if( reg8[TIMER_REG] && (reg8[FLAGS_REG] & BLINKY_CLK_ENABLE) )
            reg8[TIMER_REG]--;
          // Writing results in clearing of FI[12]
          fiClr(FI_PRT_TIMER);
          // Reset timer countdown
          nAlm = timerCnt();
          break;
        case TX_REG:
          if( reg8[FLAGS_REG] & BLINKY_CLK_ENABLE ) {
            // Write to out buffer - set buffer flag
            // Maybe we should just keep 8 LSB
            prtBuf[wprt++] = reg8[TX_REG];
            fiSet(FI_PRT_BUSY);
            bOutBuf = true; // Character in buffer to be sent
          }
          break;
        }
      }
    }
  }
  inline uint64_t read(int r) {
    // 8 or 64 bit register ... ?
    if( r < 8 ) {
      return reg[r];  // 64-bit
    } else {
      // Update hardware registers
      return reg8[r]; // 8-bit
    }
  }
  inline uint64_t operator[](int index) {
    return read(index);
  }
  void func(int c) {
    switch( c ) {
    case 2: // Enable timer clock
      set(BLINKY_ENABLE|BLINKY_CLK_ENABLE);
      reg8[9] = reg8[14];
      break;
    case 3: // Disable timer clock
      clr(BLINKY_CLK_ENABLE);
      break;
    case 4: // Enable RAM write
      set(BLINKY_ENABLE|BLINKY_RAM_ENABLE);
      break;
    case 5: // Disable RAM write and de-select the printer
      clr(BLINKY_RAM_ENABLE);
      select(false);
      break;
    case 7: // Reset
      reg8[FLAGS_REG] = 0;
      fiClr(FI_PRT_BUSY);
      break;
    case 8: // De-select the printer
      select(false);
      break;
    }
  }
  void inline busy(void) {
    if( bOutBuf ) { // Character in buffer
      if( busyCnt ) {
        // HW is still busy sending character
        fiSet(FI_PRT_BUSY);
      } else {
        // HW is free - start next transmission
        busyCnt = BUSY_CNT;
        // Empty buffer and clear FI-flag
        bOutBuf = false;
        fiClr(FI_PRT_BUSY);
      }
    }
    if( busyCnt ) {
      busyCnt--;
      // When done and no character waiting - clear busy flag
      if( !busyCnt && !bOutBuf)
        fiClr(FI_PRT_BUSY);
    }
  }
  inline void fi(volatile uint16_t *f) {
    // Only update flags if we are selected
    if( bSelected ) {
      // Clear FI-flags
      *f &= ~(FI_PRT_BUSY|FI_PRT_TIMER);
      // Set flags if printer is selected
      *f |= m_fiFlags;
    }
  }
  inline uint8_t flags(void) {
    return reg8[FLAGS_REG];
  }
  inline uint8_t timer(void) {
    return reg8[TIMER_REG];
  }
  inline uint16_t fiFlags(void) {
    return m_fiFlags;
  }
  inline void set(uint8_t f) {
    reg8[FLAGS_REG] |= f;
  }
  inline void clr(uint8_t f) {
    reg8[FLAGS_REG] &= ~f;
  }
  inline void fiSet(uint16_t f) {
    m_fiFlags |= f;
  }
  inline void fiClr(uint16_t f) {
    m_fiFlags &= ~f;
  }

  void write(uint64_t *dta) {
    if( bwr ) {
      bwr--;
      if( bwr == BLINKY_ADDR ) {
        // This is a special DATA WRITE to reg 0x20
        wrStatus((uint8_t)((*dta >> 48LL) & 0xFF));
      } else {
        write(bwr, dta, true);
      }
      bwr = 0;
    }
  }
  uint32_t write(uint32_t addr, int r) {
    // Update ram select pointer
    addr = BLINKY_ADDR | r;
    if( flags() & BLINKY_RAM_ENABLE)
      bwr = r+1; // Delayed write ... bwr = [0x0,0xF] + 1
    return addr;
  }
  void delaydWrite(uint32_t addr) {
    bwr = addr;
  }
  uint64_t read(uint32_t addr, int r) {
    return read(r);
  }
  bool isAddress(int a) {
    return (a & 0x3F0) == BLINKY_ADDR;
  }
  uint16_t getAddress(uint16_t a) {
    return (a == BLINKY_ADDR) ? BLINKY_ADDR+1 : (a&0x0F)+1;
  }
  RamDevice_e devID() { return BLINKY_DEV; }
};
#endif//__BLINKY_H__
