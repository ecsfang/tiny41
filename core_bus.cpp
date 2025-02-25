#include <stdio.h>
#include <string.h>
#include <time.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/sync.h"
#include "pico/multicore.h"
#include "tiny41.h"
#include "core_bus.h"
#include "blinky.h"
#include "disasm.h"
#include "serial.h"
#include "flash.h"
#include <malloc.h>
#ifdef USE_PIO
#include "ir_led.h"
#endif
#include "usb/cdc_helper.h"

#include "module.h"
#include "modfile.h"
#include "wand.h"

#include "voysegment.h"

//#define RESET_FLASH
//#define RESET_RAM

extern CDisplay    disp41;
bool display_on = false;

CBlinky blinky;
volatile uint64_t  CBlinky::reg[8];    // First 8 registers
volatile uint8_t   CBlinky::reg8[16];  // Last 8 registers (0-7 not used)

#ifdef USE_TIME_MODULE
CTime mTime;
volatile CRegs_t   CTime::reg[2];    // 56 bits time regs
volatile uint32_t  CTime::interval;  // 20 bits interval timer
volatile uint16_t  CTime::accuracy;  // 13 bits
volatile uint16_t  CTime::status;    // 20 bits
#endif//USE_TIME_MODULE

volatile uint64_t gDisp9 = 0;
volatile uint64_t gDisp10 = 0;
volatile uint64_t _gDisp9 = 1L;
volatile uint64_t _gDisp10 = 1L;
volatile bool bUpdDisp9 = false;
volatile bool bUpdDisp10 = false;
volatile uint16_t gDispUpdate = 0;
volatile uint16_t lDispUpd = 0;

// Special command buffer from Tiny
volatile char g_cmd[32];
volatile int  g_pCmd = 0;
volatile int  g_num = 0;
volatile int  g_ret = 0;
volatile bool g_do_cmd = 0;

volatile uint16_t voyAddr[4] = {
  0x081, 0x103,
  0x005, 0x102
};

CBreakpoint brk;

uint8_t keyMap[0x100] = {
  0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
  0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F,
  0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F,
  0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x3F,
  0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F,
  0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5A, 0x5B, 0x5C, 0x5D, 0x5E, 0x5F,
  0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6A, 0x6B, 0x6C, 0x6D, 0x6E, 0x6F,
  0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7A, 0x7B, 0x7C, 0x7D, 0x7E, 0x7F,
  0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8A, 0x8B, 0x8C, 0x8D, 0x8E, 0x8F,
  0x90, 0x91, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9A, 0x9B, 0x9C, 0x9D, 0x9E, 0x9F,
  0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xAB, 0xAC, 0xAD, 0xAE, 0xAF,
  0xB0, 0xB1, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7, 0xB8, 0xB9, 0xBA, 0xBB, 0xBC, 0xBD, 0xBE, 0xBF,
  0xC0, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8, 0xC9, 0xCA, 0xCB, 0xCC, 0xCD, 0xCE, 0xCF,
  0xD0, 0xD1, 0xD2, 0xD3, 0xD4, 0xD5, 0xD6, 0xD7, 0xD8, 0xD9, 0xDA, 0xDB, 0xDC, 0xDD, 0xDE, 0xDF,
  0xE0, 0xE1, 0xE2, 0xE3, 0xE4, 0xE5, 0xE6, 0xE7, 0xE8, 0xE9, 0xEA, 0xEB, 0xEC, 0xED, 0xEE, 0xEF,
  0xF0, 0xF1, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6, 0xF7, 0xF8, 0xF9, 0xFA, 0xFB, 0xFC, 0xFD, 0xFE, 0xFF
};
void keyMapInit(void);

// Keep track of FI flags (bitmask, bit0-bit13)
volatile uint16_t carry_fi = 0;
// When true - set PBSY in next bus cycle
volatile bool bPBusy = false;
// When true - clear PBSY in next bus cycle
volatile bool bClrPBusy = false;

void power_on()
{
  // Drive ISA high to turn on the calculator ...
  gpio_put(P_ISA_DRV, 1);
  gpio_put(P_ISA_OE, ENABLE_OE); // Enable ISA driver
  // Set PBSY-flag on to indicate someone needs service ...
  _setFI_PBSY();
  sleep_us(10);
  gpio_put(P_ISA_OE, DISABLE_OE); // Disable ISA driver
  gpio_put(P_ISA_DRV, 0);
  sleep_ms(10);
  _clrFI_PBSY();
}

#if CF_DBG_DISP_INST
extern const char *inst50disp[16];
extern const char *inst70disp[16];
#endif

// Extract display data from data56
#define CH9_B03(c) ((c >> 0) & 0x00F) // Bit 0-3
#define CH9_B47(c) ((c >> 4) & 0x00F) // Bit 4-7
#define CH9_B8(c)  ((c >> 8) & 0x001) // Bit 9

////////////////////////////////////////////////////////////////////////////////
//
// Core 1 sits in a loop grabbing bus cycles
//

//#define RISING_EDGE(SIGNAL) ((last_##SIGNAL == 0) && (SIGNAL == 1))
//#define FALLING_EDGE(SIGNAL) ((last_##SIGNAL == 1) && (SIGNAL == 0))

// Do we drive ROM data?
volatile int drive_isa_flag = 0;  // Drive ISA in next cycle
volatile int output_isa = 0;      // Output ongoing on ISA...
volatile int output_data = 0;     // Output ongoing on DATA...

// The data we drive
uint16_t drive_isa = 0;

void handle_bus(volatile Bus_t *pBus);

////////////////////////////////////////////////////////////////////////////////
//
// Core1 main functions
//
// Various different core 1 main functions used to test functionality
//
////////////////////////////////////////////////////////////////////////////////

// Data transfer to core 0

#define NUM_BUS_MASK (NUM_BUS_T-1)
#define INC_BUS_PTR(d) d = (d+1) & NUM_BUS_MASK

volatile int queue_overflow = 0;

volatile int data_wr = 0;
volatile int data_rd = 0;

#define DATA_OUTPUT(x) do { output_data = 1; data56_out = x;} while(0)
volatile uint64_t data56_out = 0;
bool bET11967 = false;
volatile uint64_t data2FI = 0;

volatile Bus_t bus[NUM_BUS_T];

volatile uint8_t prtBuf[PRT_BUF_LEN];
volatile uint8_t wprt = 0;
volatile uint8_t rprt = 0;

volatile uint8_t sendBuf[PRT_BUF_LEN];
volatile uint8_t wSend = 0;
volatile uint8_t rSend = 0;
volatile uint16_t sendSize = 0;
volatile uint16_t rxSize = 0;

int last_sync = 0;
int gpio_states = 0;

// Copy of the 41 LCD buffer
char dtext[2 * NR_CHARS + 1];
// The punctation part of the 41 LCD buffer
bool bPunct[2 * NR_CHARS + 1];

char cpu2buf[256];
int nCpu2 = 0;

extern void dispOverflow(bool bOvf);

void reset_bus_buffer(void)
{
  // Reset buffer if overflow has occured ...
  if( queue_overflow ) {
    queue_overflow = 0;
    data_rd = data_wr = 0;
    dispOverflow(false);
  }
}

bool bT0Carry = false;

volatile Mode_e cpuMode;
void logC(const char *s);

void core1_main_3(void)
{
  static int bit_no = 0;        // Current bit-number in bus cycle
  static int bIsaEn = 0;        // True if ISA output is enabled
  static int isDataEn = 0;      // True if DATA output is enabled
  static uint64_t tinyIsa = 0LL;// Current bit-pattern on the ISA bus
  static uint64_t isa = 0LL;    // Current bit-pattern on the ISA bus
  static uint64_t bit = 0LL;    // Bit mask for current bit in cycle
  static uint64_t data56 = 0LL; // Current bit-pattern on DATA bus
  static uint8_t fiShift = 0;   // Current shift of FI
  static uint16_t dataFI = 0;   // State of FI to be sent
  static int sync = 0;          // True if sync detected
  static bool bIsSync = false;  // True if current cycle have sync
  static bool bSelPa = false;   // True if peripheral address is in next cycle
  static uint8_t perph = 0;     // Selected pheripheral
  static bool bSelRam = false;  // True if RAM address is in next cycle
  static uint32_t ramad = 0;    // Current RAM pointer
  static bool bErr = false;     // True if corrupt cycle is detected
  static bool bPrinter = false; // True if printer is selected
  static bool bTiny = false;    // True if Tiny41 is selected
  static CModule *mp = NULL;    // Pointer to current module (if selected)
  volatile Bus_t *pBus;         // Pointer into trace buffer
  static int last_data_wr;      // Keep track of trace overflow
  static uint16_t iCnt = 0;     // Cycle counter for trace
  static uint32_t ramAddr = 0;
  static uint8_t r = 0;
  static uint16_t cmd = 0;
  static uint8_t cmd6 = 0;
  static bool bPWO = false;
  static bool bPullIsa = false;
  static uint16_t isBase = ARITHM_UKN;
  static CRamDev *pRamDev = NULL;
  static CModule *voyager = modules.port(5);
  static int key = 0;
  static uint16_t vKey = 0;

  keyMapInit();

  pBus = &bus[data_wr];

  if( !flash_safe_execute_core_init() ) {
    sprintf(cbuff, "*** Flash initfailed! ***\n\r");
    cdc_send_string_and_flush(ITF_CONSOLE, cbuff);
  }

//  irq_set_mask_enabled(0xffffffff, true);

  last_sync = GPIO_PIN(P_SYNC);

#ifdef MEASURE_TIME
  uint64_t tick = time_us_64();
#endif


  // This is the main loop where everything happens regarding the BUS
  // All 56 cycles of the bus is taken care of, and data is pulled from
  // the bus or injected on the bus accordingly.
  while (1) {
    // Check if module is active ...
    bPWO = GPIO_PIN_RD(P_PWO);
    if( !bPWO ) {
      // No, we are sleeping ...
      gpio_put(LED_PIN_B, LED_OFF);
      cpuMode = GPIO_PIN(P_SYNC) ? LIGHT_SLEEP : DEEP_SLEEP;
      // Update Blinky after each bus-cycle (160 us)
      uint64_t now = time_us_64();
      if( (uint32_t)(now - tick) >= ONE_BUS_CYCLE ) {
        if( blinky.tick() ) {
          // Time to wake the CPU, pull ISA high until PWO ...
          bPullIsa = true;
        }
        // Reset bus-cycle counter
        tick = now;
      }
      // But don't wake the CPU if we are in deep sleep ...
      if( cpuMode == LIGHT_SLEEP && bPullIsa && !bIsaEn ) {
        // Pull ISA high to wake the calculator ...
        gpio_put(P_ISA_DRV, 1);
        gpio_put(P_ISA_OE, (bIsaEn = ENABLE_OE));
      }
      // PWO is low, so nothing to do, just ...
      continue;
    } else {
      // If ISA is active, then turn it off since PWO is active ...
      if( bPullIsa && bIsaEn ) {
        gpio_put(P_ISA_OE, (bIsaEn = DISABLE_OE));
        bPullIsa = false;
      }
      cpuMode = RUNNING;
      gpio_put(LED_PIN_B, LED_ON);
    }
    if( bErr )
      cpuMode = NO_MODE;

    // Wait for CLK2 to have a falling edge
    WAIT_FALLING(P_CLK2);

    // NOTE! Bit numbers are out by one as bit_no hasn't been incremented yet.

// Clk1  ____/\_______/\_______/\_______/\_______/\_______/\_______/\_______/\___
// Clk2  _______/\_______/\_______/\_______/\_______/\_______/\_______/\_______/\___
// DATA  |  T51   |  T52   |  T53   |  T54   |  T55   |  T0    |  T1    |  T2    |

    if( bit_no >= LAST_CYCLE ) {
      // Setup FI-signal for next round ...
      if( bET11967 ) {
        // Service module installed, shortcut DATA and FI
        data2FI = ~data56;
        gpio_put(P_FI_OE, (data2FI>>(fiShift*4)) & 1);
        dataFI = 0;
      } else {
        dataFI = getFI();
        // Give Blinky opportunity to update the FI flags
        blinky.fi(&dataFI);
      }
      // Clear for next cykle ...
      isa = data56 = 0LL;
      fiShift = 0;
      clrFI();
      // Check if ISA should be active during T0 (peripherial carry)
      if( bT0Carry && !bIsaEn ) {
        gpio_put(P_ISA_DRV, 1);
        gpio_put(P_ISA_OE, (bIsaEn = ENABLE_OE));
      }
    }

    // Drive the FI carry flags for each nibble
    if( (bit_no & 0b11) == 0b11 ) {
      if( bET11967 )
        gpio_put(P_FI_OE, (data2FI>>(fiShift*4)) & 1);
      else
        gpio_put(P_FI_OE, (dataFI>>fiShift) & 1);
      fiShift++;
    }

    // Do we drive the DATA line (bit 0-55)?
    if (output_data) {
      // Expose the next bit on the data line ...
      gpio_put(P_DATA_DRV, data56_out & 1);
      data56_out >>= 1;
    }
    if( bT0Carry && bIsaEn && bit_no > 2 && bPWO ) {
      gpio_put(P_ISA_OE, (bIsaEn = DISABLE_OE));
      bT0Carry = false;
    }

    // Do we drive the ISA line (bit 43-53)?
    if (output_isa) {
      // Drive the ISA line for theses data bit
      switch (bit_no) {
      case 43-1:
        // Prepare data for next round ...
        gpio_put(P_ISA_DRV, drive_isa & 1);
        break;
      case LAST_CYCLE-2:
        // Don't drive data any more
        gpio_put(P_ISA_OE, (bIsaEn = DISABLE_OE));
        // ... so no more data after this ...
        output_isa = drive_isa_flag = 0;
        break;
      default: // Drive during bit 43->52
        // Enable ISA driver (if not already enable) ...
        if( bIsaEn == DISABLE_OE )
          gpio_put(P_ISA_OE, (bIsaEn = ENABLE_OE));
        // Expose the bit on the ISA line and prepare the next ...
        gpio_put(P_ISA_DRV, drive_isa & 1);
        drive_isa >>= 1;
      }
    }

    // Wait for CLK1 to have a rising edge
    WAIT_RISING(P_CLK1);

    // Another bit, check SYNC to find bit number
    sync = GPIO_PIN(P_SYNC);

    // Increment the bit number (or sync ...)
    if (sync && !last_sync) {
      bIsSync = true;
      bit_no = ISA_SHIFT;
    } else {
      bit_no = (bit_no >= LAST_CYCLE) ? 0 : bit_no+1;
    }

    // NOTE! Bit numbers are now ok since bit_no has been incremented!

    // Save all bits in data and isa reg ...
    bit = 1LL << bit_no;
    if (GPIO_PIN(P_DATA))
      data56 |= bit;
    if (GPIO_PIN(P_ISA))
      isa |= bit;

    switch (bit_no) {
    case 0:
#ifdef MEASURE_TIME
      tick = time_us_64();
#endif
      pBus = &bus[data_wr];
      memset((void*)pBus, 0, sizeof(Bus_t));
      // Report next FI signal to trace ...
      pBus->fi = dataFI;
      // Check if still busy ...
      blinky.busy();
      break;

    case 7: // 8 first bits are read ...
      if( bSelPa ) {
        // Get the selected peripheral
        perph = data56 & 0xFF;
        bSelPa = false;
      }
      break;

    case 9: // 10 first bits are read ...
      if( bSelRam ) {
        // Get the selected peripheral
        ramad = data56 & 0x3FF;
        bSelRam = false;
      }
      pRamDev = NULL;
      if( !perph ) {
        // Check if Blinky RAM is being accessed ...
        if( blinky.isAddress(ramad) ) {
          pRamDev = &blinky;
        }
        else if( ram.isEnabled() && ram.isAddress(ramad) ) {
          pRamDev = &ram;
        }
#ifdef USE_XF_MODULE
        // ... else it might be XMemory ...
        else if( xmem.isAddress(ramad) ) {
          pRamDev = &xmem;
        }
#endif
      }
      break;

    case 11:
      if( pRamDev ) {
        ramAddr = pRamDev->getAddress(ramad);
      }
      break;

    case 19:
      key = keyMap[(data56>>12) & 0xFF];
      break;

    case 29:
      // Got address. If the address is that of the embedded ROM then we
      // flag that we have to put an instruction on the bus later
      pBus->addr = (isa >> 14) & ADDR_MASK;
      mp = modules.at(pBus->addr);
      break;

    case 30:
      // Check if we should emulate any modules ...
      if (mp->isInserted()) {
        pBus->cmd = drive_isa = (*mp)[pBus->addr];
        drive_isa_flag = 1;
      }
      break;

/*    case 31:
      // Check if we should emulate any modules ...
      if (modules.isInserted(4) && pBus->addr < 4) {
        pBus->cmd = drive_isa = voyAddr[pBus->addr];
        drive_isa_flag = 1;
      }
      break;
*/
    case 42: // Enable ISA output in next loop (cycle 43)
      if( drive_isa_flag )
        output_isa = 1;
      break;

    case LAST_CYCLE-3:
      // Check blinky timer counter
      if( blinky.tick() )
        bT0Carry = true;
      break;

    case LAST_CYCLE-2:
      // Check if more data to be read from the wand ...
      if( pBar->available() ) {
        setFI(FI_PBSY | FI_WNDB);
      } else {
        clrFI(FI_WNDB);
      }
      break;

    case LAST_CYCLE-1:
#ifndef LOG_FI // Log something else ...
      //pBus->fi = ramAddr-1;
      //pBus->fi = ramad;
      //pBus->fi = (blinky.nAlm & 0xFF) << 8;
      //pBus->fi = blinky.timer() << 8;
      pBus->fi = blinky.fiFlags();
      //pBus->fi |= blinky.cntTimer & 0xFF;
#endif
      // Prepare command word ...
      // Is instruction fetched from flash?
      if( !pBus->cmd ) {
        // If not, get instruction from the bus ...
        pBus->cmd = (isa >> ISA_SHIFT) & INST_MASK;
      }
      // Local copy of the instruction ...
      cmd = pBus->cmd;
      // Extract the low 6 bit instruction class ...
      cmd6 = cmd & 0x3F;
      // Extract register/address information from the instruction ...
      r = (cmd >> 6) & 0x0F;
      if( bIsSync ) {
        // Save info about sync in logged data ..
        pBus->cmd |= CMD_SYNC;
        // Check for special instructions ...
        switch( cmd ) {
        // Keep track of the arithmetic mode
        case INST_SETHEX:
          isBase = ARITHM_HEX;
          cmd = 0;
          break;
        case INST_SETDEC:
          isBase = ARITHM_DEC;
          cmd = 0;
          break;
        case INST_RAM_SLCT:
          // Next time, data contains the selected RAM address
          // Fetch selected RAM in the next run ...
          bSelRam = true;
          // ... and reset any chip previously enabled ...
          perph = 0;
          cmd = 0;
          break;
        case INST_PRPH_SLCT:
          // Next time, data contains the selected peripheral address
          // Fetch selected peripheral from DATA in the next run ...
          bSelPa = true;
          blinky.select(false);
          cmd = 0;
          break;
        case INST_SEL_PRT:
          // Blinky Printer is selected
          // Handle NPIC commands in next cykle
          blinky.select(true);
          bPrinter = true;
          cmd = 0;
          break;
        case INST_SEL_TINY:
          // Tiny41 special instructions
          // Handle NPIC commands in next cykle
          bTiny = true;
          cmd = 0;
          vKey = (*voyager)[(data56&0xFFF)+1];
          //voyager = modules.at(0x4000);
          //vKey = (*voyager)[key];
          //tinyIsa = isa;
          break;
        case INST_ENBANK1:
        case INST_ENBANK2:
        case INST_ENBANK3:
        case INST_ENBANK4:
            // Switch bank (if any) ...
            mp->selectBank(cmd);
          cmd = 0;
          break;
        }
      }
      // Remember last queue write pointer ...
      last_data_wr = data_wr;
      break;

    case LAST_CYCLE:
      // If bitno == LAST_CYCLE then we have another frame, save the transaction
      // A 56 bit frame has completed, we send some information to core0 so it
      // can update the display and anything else it needs to do ...

      // Assume no data drive any more ...
      output_data = 0;
      data56 &= MASK_56_BIT;

      if( bUpdDisp9 ) {
        gDisp9 = data56;
        bUpdDisp9 = false;
        gDispUpdate++;
      }
      if( bUpdDisp10 ) {
        gDisp10 = data56;
        bUpdDisp10 = false;
        gDispUpdate++;
      }

      // Check for any pending writes
      if (pRamDev ) {
	      pRamDev->write(&data56);
#ifdef USE_TIME_MODULE
      } else if( mTime.bwr ) { // Check for pending writes to Time Module
        mTime.write(&data56);
#endif//USE_TIME_MODULE
      }

      if( bIsSync ) {
        if( cmd ) {
          // If perph == 0 then RAM is accessible
          // Handle any device in the normal RAM address space
          // - Memory module (not done yet)
          // - XMemory
          // - Blinky RAM
          bUpdDisp9 = cmd == 0x268;
          bUpdDisp10 = cmd == 0x2A8;
          if( !perph ) {
            if( pRamDev ) {
              // Update any RAM device (QUAD, XMEM etc)
              if( cmd == INST_WDATA ) {
                pRamDev->delayedWrite(ramAddr);
              } else if( cmd6 == INST_READ_DATA ) {
                DATA_OUTPUT(pRamDev->read(ramAddr,r));
              } else if( cmd6 == INST_WRITE_DATA ) {
                // Update ram select pointer
                ramad = pRamDev->write(ramad, r);
              }
            }
          } else {
            // A peripherial is selected ...
            switch( perph ) {
            case WAND_ADDR:
              if( cmd == INST_WANDRD && pBar->available() ) {
                // Output next byte from the wand-buffer!
                DATA_OUTPUT(pBar->get());
              }
              break;
#ifdef USE_TIME_MODULE
            case TIMR_ADDR:
              if( cmd6 == INST_READ_DATA ) {
                DATA_OUTPUT(mTime.read(r));
              } else if( cmd6 == INST_WRITE_DATA ) {
                // Update ram select pointer
                mTime.bwr = r+1;
              }
              break;
#endif//USE_TIME_MODULE
            }
          }
        }
        bIsSync = false;
      } else {
        // No real instruction .. (no sync)
        // Handle the printer if it is selected
        if( bPrinter ) {
          switch(cmd & 0b110) {
            case NPIC_IR_PRT_WRITE:  // Write ...
              blinky.write(r, &data56);
              break;
            case NPIC_IR_PRT_READ:  // Read ...
              DATA_OUTPUT(blinky[r]);
              break;
            case NPIC_IR_PRT_CMD:  // Func ...
              blinky.func(r);
              break;
          }
          // Check if return-bit is set
          if( cmd & 1)
            bPrinter = false;
        }
        if( bTiny ) {
          // Handle any specific Tiny41 instructions
#ifdef VOYAGER
#define VOYAGER_PORT  0x5000
          switch(cmd & 0x3FE) {
            case 0x03A: // IR_RD 0
              DATA_OUTPUT((VOYAGER_PORT<<12) | ((*voyager)[key]<<12));
              break;
            case 0x07A: // IR_RD 1
              DATA_OUTPUT(VOYAGER_PORT | (*voyager)[data56&0xFFF]);
              break;
            case 0x0BA: // IR_RD 2
              DATA_OUTPUT((VOYAGER_PORT<<12)| 0x3F0000 | ((data56<<4)&0xFFF0));
              break;
            case 0x0FA: // IR_RD 3
              DATA_OUTPUT((data56 & 0xF0000000000LL)<<12 | vKey); //(*voyager)[data56&0xFFF+1]);
              break;
            case 0x13A: // IR_RD 4
              DATA_OUTPUT((VOYAGER_PORT<<12) | 0x800000);
              break;
            case 0x17A: // IR_RD 5
              DATA_OUTPUT((VOYAGER_PORT | 0x39) << 12);
              break;
/*
            case 0b1100:
              g_do_cmd = 1;
              _setFI_PBSY();  // Indicate that we are busy!
              break;
            case 0b1000:
              g_num = data56 & 0xFF; // Save command byte
              break;
            case 0b0100:
              g_cmd[g_pCmd++] = data56 & 0xFF; // Save character
              g_cmd[g_pCmd] = 0;
              break;
            case 0b0000:
              g_pCmd = 0;           // Reset command
              g_cmd[g_pCmd] = 0;
              break;
*/
          }
#endif
#if 0
          switch(cmd & 0x3FE) {
            case 0x03A:
              {
                int key = (*voyager)[(data56>>12) & 0x3FF]<<12;
                DATA_OUTPUT(0x4000000LL | key);
              }
              break;
            case 0b1100:
              g_do_cmd = 1;
              _setFI_PBSY();  // Indicate that we are busy!
              break;
            case 0b1000:
              g_num = data56 & 0xFF; // Save command byte
              break;
            case 0b0100:
              g_cmd[g_pCmd++] = data56 & 0xFF; // Save character
              g_cmd[g_pCmd] = 0;
              break;
            case 0b0000:
              g_pCmd = 0;           // Reset command
              g_cmd[g_pCmd] = 0;
              break;
          }
#endif
#if 0
          switch(cmd & 0b1110) {
            case NPIC_TINY_RAW_INIT:
              // Init raw file transfer...
              sendSize = data56 & 0xFFFF;
              rxSize = 0;
              wSend = rSend = 0;
              break;
            case NPIC_TINY_RAW_BYTE:
              // Send program byte ...
              sendBuf[wSend++] = data56 & 0xFF;
              break;
            case NPIC_TINY_RAW_DONE:
              // Done with raw file transfer!
              break;
          }
#endif
          // Check if return-bit is set
          if( cmd & 1)
            bTiny = false;
        }
      }
#ifdef MEASURE_TIME
      {
        uint16_t tCycle = (uint16_t)(time_us_64() - tick);
        // Floating input-pin ... ?
        // A normal cycle (56 clock cycles @ 366kHz) should take ~158us.
        // So, if outside that range, we have an invalid cycle ...
        if( tCycle < 150 || tCycle > 170 ) {
          bErr = true;
          continue;
        }
        //pBus->fi = tCycle;
      }
      if( !(pBus->cmd|pBus->addr) )
        continue;
      bErr = false;
#endif

      // Prepare logging data ...
      pBus->data = data56;
      // Report selected peripheral to trace ...
      pBus->data |= ((uint64_t)perph)<<PA_SHIFT;
      // Report selected arithmetic mode ...
      pBus->cmd |= isBase;
#ifdef TRACE_ISA
      pBus->isa = isa;
#endif
      // Add bus-counter to log ...
      pBus->cnt = iCnt++;
      // Cleanup, fix trace buffer pointer and
      // check for overflow ...
      INC_BUS_PTR(data_wr);
      if (data_rd == data_wr) {
        // No space left in ring-buffer ...
        queue_overflow++;
        // Restore last pointer (overwriting this cycle)
        data_wr = last_data_wr;
        // Turn at least disasm off ...
        bTrace &= ~TRACE_DISASM;
      }
      // Enable DATA for next cycle ... ?
      if( output_data != isDataEn ) {
        isDataEn = output_data;
        gpio_put(P_DATA_OE, isDataEn);
      }
      break;
    }
    last_sync = sync;
  }
}

void post_handling(uint16_t addr)
{
  // Check if we have more data to send and that the buffer is empty ...
  // pBar->empty() if all data in buffer have been read
  // FI_WNDB (T2) is low if buffer is empty
  if( pBar->isDone() && !chkFI(FI_WNDB) ) {
    // We have no more data to send!
    pBar->done();
  }
#if 0
  if( (iGetNextWndData > iSendNextWndData) && pBar->empty() && !chkFI(FI_WNDB) ) {
    // We have more data to send!
    iSendNextWndData++;
    wand_scan();
  }
#endif
  if (wprt != rprt) {
    uint8_t c = prtBuf[rprt++];
    switch(c) {
      case 0x04:
      case 0x0A:
        cdc_send_printport('\n');
        cdc_send_printport('\r');
        break;
      case 0x86: cdc_send_printport('*'); break;
      default:
//        if( c < 0x20 || c > 0x7F )
//          printf("%02X ", c);
//        else
          cdc_send_printport(c);
    }
    //cdc_send_printport(c);
#ifdef USE_PIO
    send_to_printer(c);
#endif
  }
  if (wSend != rSend) {
    uint8_t c = sendBuf[rSend++];
    if( rxSize == 0 && c == 0x00 ) {
      // Skip leading NULL bytes
      sendSize--;
    } else {
      char bb[8];
      sprintf(bb,"%02X", c);
      if( rxSize == 0 ) {
        // Send 'S' to start the transfer
        cdc_send_printport('S');
      }
      // Send each byte as ascii
      cdc_send_printport(bb[0]);
      cdc_send_printport(bb[1]);
      if( rxSize == sendSize ) {
        // When done - end transfer with a 'q'
        cdc_send_printport('q');
      }
      cdc_send_printport('\n');
      cdc_send_printport('\r');
      rxSize++;
    }
  }
#ifdef USE_TIME_MODULE
  // Update time in Time module
  mTime.tick();
#endif//USE_TIME_MODULE
  if( g_do_cmd ) {
    switch(g_do_cmd ) {
      case 1:
        sprintf(cbuff, "Load page with [%s] @ %d\n\r", g_cmd, g_num);
        cdc_send_string_and_flush(ITF_CONSOLE, cbuff);
        g_ret = loadModule((char*)g_cmd,g_num) ? 0 : 1;
        _clrFI_PBSY(); // Done with the plugging!
        break;
    }
    g_do_cmd = 0;
  }
  if( lDispUpd < gDispUpdate ) {
    if( gDisp9 != _gDisp9 || gDisp10 != _gDisp10) {
      CLCD *pLcd = new CLCD(gDisp9, gDisp10, true);
      uint16_t ch[10], sign;
      sign = pLcd->hasSign() ? '_' : ' ';
      int n = 0;
      char pt;
      sprintf(cbuff, "\n\r");
      cdc_send_string_and_flush(ITF_CONSOLE, cbuff);
      for(int i=0; i<10; i++) {
        n += sprintf(cbuff+n, "   %c", pLcd->segment(i, SEG_a) ? '_' : ' ');
      }
      sprintf(cbuff+n, "\n\r");
      cdc_send_string_and_flush(ITF_CONSOLE, cbuff);
      n = sprintf(cbuff, "%c", sign);
      for(int i=0; i<10; i++) {
        n += sprintf(cbuff+n, " %c", pLcd->segment(i, SEG_f) ? '|' : ' ');
        n += sprintf(cbuff+n,  "%c", pLcd->segment(i, SEG_g) ? '_' : ' ');
        n += sprintf(cbuff+n,  "%c", pLcd->segment(i, SEG_b) ? '|' : ' ');
      }
      sprintf(cbuff+n, "\n\r");
      cdc_send_string_and_flush(ITF_CONSOLE, cbuff);
      n = sprintf(cbuff, "  ");
      for(int i=0; i<10; i++) {
        pt = pLcd->point(i);
        n += sprintf(cbuff+n, "%c",   pLcd->segment(i, SEG_e) ? '|' : ' ');
        n += sprintf(cbuff+n, "%c",   pLcd->segment(i, SEG_d) ? '_' : ' ');
        n += sprintf(cbuff+n, "%c%c", pLcd->segment(i, SEG_c) ? '|' : ' ', pt);
      }
      //sprintf(cbuff+n, "\n\r%014llX %014llX\n\n\r", gDisp10, gDisp9);
      sprintf(cbuff+n, "\n\r");
      cdc_send_string_and_flush(ITF_CONSOLE, cbuff);
/*
           _   _       _   _   _   _   _   _
    _   |  _|  _| |_| |_  |_    | |_| |_| | |
        |,|_   _|   |, _| |_|   |,|_|  _| |_|.
          USER  f g BEGIN  GRAD  D.MY  C  PRGM  
*/

      n = 0;
      n += sprintf(cbuff+n, "      ");
      n += sprintf(cbuff+n, "%.4s  ", pLcd->annun(ANN_USER)  ? "USER" : "" );
      n += sprintf(cbuff+n, "%c ",    pLcd->annun(ANN_F)     ? 'f' : ' ' );
      n += sprintf(cbuff+n, "%c ",    pLcd->annun(ANN_G)     ? 'g' : ' ' );
      n += sprintf(cbuff+n, "%.5s  ", pLcd->annun(ANN_BEGIN) ? "BEGIN" : "" );
      n += sprintf(cbuff+n, "%c",     pLcd->annun(ANN_GRAD)  ? 'G' : ' ' );
      n += sprintf(cbuff+n, "%.3s  ", pLcd->annun(ANN_RAD)   ? "RAD" : "" );
      n += sprintf(cbuff+n, "%.4s  ", pLcd->annun(ANN_DMY)   ? "D.MY" : "" );
      n += sprintf(cbuff+n, "%c  ",   pLcd->annun(ANN_C)     ? 'C' : ' ' );
      n += sprintf(cbuff+n, "%.4s",   pLcd->annun(ANN_PRGM)  ? "PRGM" : "" );
      sprintf(cbuff+n, "\n\n\r");
      cdc_send_string_and_flush(ITF_CONSOLE, cbuff);
      //sprintf(cbuff, "\n\r%014llX %014llX\n\n\r", gDisp10, gDisp9);
      //sprintf(cbuff, "\n\r");
      //cdc_send_string_and_flush(ITF_CONSOLE, cbuff);
      _gDisp10 = gDisp10;
      _gDisp9 = gDisp9;

      char lcd[32];

      // Get the sign of the mantissa
      lcd[0] = pLcd->getSign();
      n = 1;
      int p = 0;
      for(int i=0; i<10; i++) {
        p += sprintf(cbuff+p, " %02X", ch[i]);
        lcd[n++] = pLcd->getChar(i);
        if( pLcd->hasPoint(i) )
          lcd[n++] = pLcd->point(i);
      }
      while(n < 30)
        lcd[n++] = ' ';
      lcd[n] = 0;
      bool bp[24];
      memset(bp, 1, 24);
      Write41String(disp41.buf(), 5, LCD_ROW, lcd, bp);
      // Turn on annunciators ...
      UpdateAnnun(pLcd->getAnnun(), false);
  
      disp41.rend(REND_LCD);
      disp41.render();
    }
    lDispUpd++;
  }
}

void keyMapInit() {
  keyMap[0x70] = 0x13; // sqrt
  keyMap[0xC0] = 0x33; // e^x
  keyMap[0x80] = 0x73; // 10^x
  keyMap[0xC6] = 0xc3; // y^x  (USER)
  keyMap[0x30] = 0x83; // 1/x
  keyMap[0x73] = 0x82; // CHS
  keyMap[0x34] = 0xc2; // 7
  keyMap[0x74] = 0x72; // 8
  keyMap[0x84] = 0x32; // 9
  keyMap[0x17] = 0x12; // divide

  keyMap[0xC4] = 0x10; // %   (ALPHA)
  keyMap[0x32] = 0x30; // GTO (XEQ)
  keyMap[0x71] = 0x70; // SIN
  keyMap[0x81] = 0xc0; // COS
  keyMap[0xC1] = 0x80; // TAN
  keyMap[0x83] = 0x87; // EEX
  keyMap[0x35] = 0xC7; // 4
  keyMap[0x75] = 0x77; // 5
  keyMap[0x85] = 0x37; // 6
  keyMap[0x16] = 0x17; // multiply
  
  keyMap[0x87] = 0x11; // R/S
  keyMap[0xC2] = 0x31; // SST
  keyMap[0x31] = 0x71; // Rdn
  keyMap[0x11] = 0xc1; // x<>y
  keyMap[0xC3] = 0x81; // CLx
  keyMap[0x13] = 0x84; // ENTER
  keyMap[0x36] = 0xc4; // 1
  keyMap[0x76] = 0x74; // 2
  keyMap[0x86] = 0x34; // 3
  keyMap[0x14] = 0x14; // subtract
  
  keyMap[0x18] = 0x18; // ON
  keyMap[0x12] = 0x38; // f
  keyMap[0xC5] = 0x78; // P/R
  keyMap[0x72] = 0xc8; // STO
  keyMap[0x82] = 0x88; // RCL
  //keyMap[0x] = 0x17; // n/a
  keyMap[0x37] = 0xc5; // 0
  keyMap[0x77] = 0x75; // decimal
  keyMap[0x10] = 0x35; // Sigma+
  keyMap[0x15] = 0x15; // add
}

#ifdef DUMP_CYCLE
void dumpCycle(Bus_t *p)
{
  /*printf("\n%d: Addr:%04X %07o  Inst: %06o PeriAd:%02X Data56:%014llX ISA:%014llX",
  data_rd,
  bus[data_rd].addr,
  bus[data_rd].addr,
  bus[data_rd].cmd,
  bus[data_rd].pa,
  bus[data_rd].data,
  bus[data_rd].isa);
  */
  printf("\n%4d: Addr:%04X Inst: %03X PeriAd:%02X Data56:%014llX ISA:%014llX",
    data_rd,
    bus[data_rd].addr,
    bus[data_rd].cmd,
    bus[data_rd].pa,
    bus[data_rd].data,
    bus[data_rd].isa);
  ISA_t *s = (ISA_t*)&bus[data_rd].isa;
  printf("\nAddr: %04X Inst: %03X\n", s->addr, s->inst);
}
#endif
unsigned int skipClk = 0;

void process_bus(void)
{
  BrkMode_e br; // Breakpoint

  // Pointer to current trace element ...
  volatile Bus_t *pb = NULL;

  // Process data coming in from the bus via core 1
  while (data_wr != data_rd) {
    // Point at current element ...
    pb = &bus[data_rd];

    if( cdc_connected(ITF_TRACE) ) {
#ifdef DUMP_CYCLE
      dumpCycle(pb);
#else
      // Handle the bus traffic
  /*    if( pb->addr < 0x3000 ) {
        if( IS_TRACE() )
          skipClk = 0;
        END_TRACE();
      } else
        START_TRACE();*/
      br = brk.isBrk(pb->addr);
      if( br == BRK_START )
        START_TRACE();
      handle_bus(pb);
      skipClk++;
      if( br == BRK_END ) {
        if( IS_TRACE() )
          skipClk = 0;
        END_TRACE();
      }
#endif
    }
    // Any handling outside bus traceing
    // Note that instructions can be missed if the buffer is overflowed!
    // Better handle these cases here ...
    post_handling(pb->addr);

    // Point at next element ...
    INC_BUS_PTR(data_rd);

    if( disp41.needRendering() ) {
      // Render only if buffer is not too full ...
      int remaining = (data_wr - data_rd) + (-((int) (data_wr <= data_rd)) & NUM_BUS_MASK);
      if( remaining < (NUM_BUS_MASK>>2))
        disp41.render();
    }
    cdc_flush(ITF_TRACE);
    tud_task();
  }
}

uint64_t dreg_a = 0, dreg_b = 0, dreg_c = 0;

void dump_dregs(void)
{
  dreg_a &= MASK_48_BIT;
  dreg_b &= MASK_48_BIT;
  dreg_c &= REG_C_48_MASK;

#if CF_DUMP_DBG_DREGS
  printf(" A:%012llX B:%012llX C:%012llX", dreg_a, dreg_b, dreg_c);
#endif

  // Build a text form of the display
  int j = 0;
  char cc = 0;
  char cl = 0;

  for (int i = 0; i < NR_CHARS; i++) {

    int b = ((NR_CHARS - 1) - i) << 2;

    cc =   (dreg_a >> b) & 0x0F;
    cc |= ((dreg_b >> b) & 0x03) << 4;
    cl =  ((dreg_b >> b) & 0x0C) >> 2;

    if ((dreg_c >> b) & 1) {
      // Upper ROM character
      cc |= 0x80;
    } else {
      // Convert to ASCII character
      if (cc < 0x20)
        cc |= 0x40;
    }

    bPunct[j] = false;
    dtext[j++] = cc;

    if (cl) {
      // Add punctuation
      bPunct[j] = true;
      dtext[j++] = " .:,"[cl];
    }
  }

  dtext[j] = '\0';

#if CF_DISPLAY_LCD
  //UpdateLCD(dtext, bPunct, display_on);
#endif
}

void updateDispReg(uint64_t data, uint8_t r)
{
  const Inst_t  *inst = &inst50cmd[r];

  uint64_t *ra = (inst->reg & REG_A) ? &dreg_a : NULL;
  uint64_t *rb = (inst->reg & REG_B) ? &dreg_b : NULL;
  uint64_t *rc = (inst->reg & REG_C) ? &dreg_c : NULL;

#if CF_DBG_DISP_INST
  printf("\n%s %016llX -->", inst50disp[r], data);
#endif

  if ((inst->shft & D_LONG) && inst->len == 48) {
    if (ra) *ra = data & (MASK_48_BIT);
    if (rb) *rb = data & (MASK_48_BIT);
    if (rc) *rc = data & (REG_C_48_MASK);
  } else {
    int n = (inst->shft & D_LONG) ? 48 / inst->len : 1;
    for (int i = 0; i < n; i++) {
      if (inst->shft & SHIFT_R) {
        // Shift RIGHT
        if (ra) *ra = (*ra >> 4) | (((uint64_t)CH9_B03(data)) << 44);
        if (rb) *rb = (*rb >> 4) | (((uint64_t)CH9_B47(data)) << 44);
        if (rc) *rc = (*rc >> 4) | (((uint64_t)CH9_B8(data)) << 44);
      } else {
        // Shift LEFT
        if (ra) *ra = (*ra << 4) | (CH9_B03(data));
        if (rb) *rb = (*rb << 4) | (CH9_B47(data));
        if (rc) *rc = (*rc << 4) | (CH9_B8(data));
      }
      data >>= inst->len; // shift 8 or 12 bits ...
    }
    if (ra) *ra &= MASK_48_BIT;
    if (rb) *rb &= MASK_48_BIT;
    if (rc) *rc &= REG_C_48_MASK;
  }
  dump_dregs();
}

void rotateDispReg(uint8_t r)
{
  const Inst_t  *inst = &inst70cmd[r];

  uint64_t *ra = (inst->reg & REG_A) ? &dreg_a : NULL;
  uint64_t *rb = (inst->reg & REG_B) ? &dreg_b : NULL;
  uint64_t *rc = (inst->reg & REG_C) ? &dreg_c : NULL;

#if CF_DBG_DISP_INST
  printf("\n%s -->", inst70disp[r]);
#endif

  for (int i = 0; i < inst->len; i++) {
    if (inst->shft & SHIFT_R) {
      // Shift RIGHT
      if (ra) ROTATE_RIGHT(*ra);
      if (rb) ROTATE_RIGHT(*rb);
      if (rc) ROTATE_RIGHT(*rc);
    } else {
      // Shift LEFT
      if (ra) ROTATE_LEFT(*ra);
      if (rb) ROTATE_LEFT(*rb);
      if (rc) ROTATE_LEFT(*rc);
    }
  }
  dump_dregs();
}

CPeripherial peripheral;

////////////////////////////////////////////////////////////////////////////////
//
// This function is passed all of the traffic on the bus. It can do
// various things with that.
//

const char arithMode[] = " dx?";

// Handle bus-trace ...
void handle_bus(volatile Bus_t *pBus)
{
  static int oCnt = -1;
  static int pending_data_inst = 0;
  static int oAddr = ADDR_MASK;
  static char mode = arithMode[ARITHM_UKN];   // Dec or Hex mode

  int addr = pBus->addr;
  int inst = pBus->cmd & INST_MASK;
  int base = (pBus->cmd & ARITHM_MSK) >> ARITHM_SHFT;
  int sync = (pBus->cmd & CMD_SYNC) ? 1 : 0;
  int fi   = pBus->fi;
  int cnt  = pBus->cnt;
  uint64_t data56 = pBus->data & MASK_56_BIT;
#ifdef TRACE_ISA
  uint64_t isa = pBus->isa;
#endif

  nCpu2 = 0;

  // Current peripheral is always included in the msb of data.
  peripheral.set((pBus->data & PA_MASK) >> PA_SHIFT);

  if( IS_TRACE() ) {
    if( oCnt == -1)
      oCnt = cnt-1;
    oCnt = ++oCnt & 0xFFFF; // Increment and wrap ...
    // Check if we missed/skipped any trace logs
    if( oCnt != cnt ) {
      nCpu2 += sprintf(cpu2buf+nCpu2, "\n\r\n\r###### SKIPPED %d TRACE CYCLES #########################\n\r", skipClk);
      pending_data_inst = 0;
      oCnt = cnt;
    }
  }

#if 1
#ifdef QUEUE_STATUS
  int remaining = (data_wr - data_rd) + (-((int) (data_wr <= data_rd)) & NUM_BUS_MASK);
  nCpu2 += sprintf(cpu2buf+nCpu2, "\n\r%c", queue_overflow ? '#' : ' ');
#else
  nCpu2 += sprintf(cpu2buf+nCpu2, "\n\r");
#endif
#else
#ifdef QUEUE_STATUS
  int remaining = (data_wr - data_rd) + (-((int) (data_wr <= data_rd)) & NUM_BUS_MASK);
  if( queue_overflow )
    nCpu2 += sprintf(cpu2buf+nCpu2, "\n\r[****]");
  else
    nCpu2 += sprintf(cpu2buf+nCpu2, "\n\r[%4d]", remaining);
#else
  nCpu2 += sprintf(cpu2buf+nCpu2, "\n\r");
#endif
#endif

  nCpu2 += peripheral.print(cpu2buf+nCpu2);

  uint8_t quad = (addr >> 10) & 0b1111;
#ifdef TRACE_ISA
  if( PAGE(addr) < 3 ) {
    // Dump information about which quad rom (easy find in VASM)
    nCpu2 += sprintf(cpu2buf + nCpu2, " %014llX %014llX | %04X (Q%2d:%03X) %03X",
            data56, isa, addr, quad, addr & 0x3FF, inst);
  } else {
    // No need in an external rom
    nCpu2 += sprintf(cpu2buf + nCpu2, " %014llX %014llX | %04X           %03X",
            data56, isa, addr, inst);
  }
#else
#if 1
  mode = arithMode[base];
  // Show current artihmetic mode
//  if( base == ARITHM_DEC )  mode = 'd';
//  if( base == ARITHM_HEX )  mode = 'x';

  nCpu2 += sprintf(cpu2buf + nCpu2, "%04X>%04X|%014llX %c%c %04X ",
            cnt, fi, data56, sync ? '*':' ', mode, addr);

  if( PAGE(addr) < 3 ) {
    // Dump information about which quad rom (easy find in VASM)
    nCpu2 += sprintf(cpu2buf + nCpu2, "(Q%2d:%03X) %03X", quad, addr & 0x3FF, inst);
  } else {
    // No need in an external rom
    nCpu2 += sprintf(cpu2buf + nCpu2, "          %03X", inst);
  }
#else
  if( PAGE(addr) < 3 ) {
    // Dump information about which quad rom (easy find in VASM)
    nCpu2 += sprintf(cpu2buf + nCpu2, " %014llX %X %c| %04X (Q%2d:%03X) %03X",
            data56, fi, sync ? '*':' ', addr, quad, addr & 0x3FF, inst);
  } else {
    // No need in an external rom
    nCpu2 += sprintf(cpu2buf + nCpu2, " %014llX %X %c| %04X           %03X",
            data56, fi, sync ? '*':' ', addr, inst);
  }
#endif
#endif

  // Handle special output - or just disassemble the instruction ...
  switch (pending_data_inst) {
  case INST_FETCH:
    if( IS_TRACE() )
      nCpu2 += sprintf(cpu2buf+nCpu2, "   > @%04X --> %03X (%04o)", addr, inst, inst);
    break;
  case INST_WRITE:
    {
      int wAddr = (data56 >> 12) & ADDR_MASK;
      int wDat = data56 & INST_MASK;
      CModule *ram = modules.at(wAddr);
      // TBD - Should we allow writes to images not inserted?
      if (ram->isLoaded() && ram->isQROM()) {
        // Page active and defined as ram - update!
        ram->writeQROM(wAddr, wDat);
        nCpu2 = sprintf(cpu2buf, "   W> %03X -> @%04X !!\n\r", wDat, wAddr);
      } else {
        nCpu2 = sprintf(cpu2buf, "   W> %03X -> @%04X (Not RAM!)\n\r", wDat, wAddr);
      }
    }
    break;
  default:
    if( IS_FULLTRACE() ) {
      char *dp = disAsm(inst, addr, data56, sync);
      if( oAddr != addr ) {
        // Make room for newline ...
        memmove(cpu2buf+1, cpu2buf, ++nCpu2);
        cpu2buf[0] = 0x0A;  // Add newline if a jump occurred ...
      }
      if( dp )
        nCpu2 += sprintf(cpu2buf + nCpu2, " - %s", dp);
      oAddr = addr + 1;
    }
    break;
  }

#ifndef DISABLE_DISPRINT
  if( IS_TRACE() ) {
    //printf("%s", cpu2buf);
    //cdc_printf_(ITF_TRACE, "%s", cpu2buf);
    cdc_send_string(ITF_TRACE, cpu2buf);
    //cdc_send_console(cpu2buf);
  }
#endif
  cpu2buf[0] = 0;
  nCpu2 = 0;

  // Check for a pending instruction from the previous cycle
  switch (pending_data_inst) {
  case INST_WRITE_ANNUNCIATORS:
#if CF_DBG_DISP_INST
    printf("\n\r%s %03llX", "WRTEN", data56 & 0xFFF);
#endif
#ifndef VOYAGER
    UpdateAnnun((uint16_t)(data56 & 0xFFF), true);
#endif
    break;

  case INST_SRLDA:  // All ...50 instructions
  case INST_SRLDB:
  case INST_SRLDC:
  case INST_SRLDAB:
  case INST_SRLABC:
  case INST_SLLDAB:
  case INST_SLLABC:
  case INST_SRSDA:
  case INST_SRSDB:
  case INST_SRSDC:
  case INST_SLSDA:
  case INST_SLSDB:
  case INST_SRSDAB:
  case INST_SLSDAB:
  case INST_SRSABC:
  case INST_SLSABC:
    updateDispReg(data56, pending_data_inst >> 6);
    break;
  case INST_WANDRD:
    if( IS_TRACE() && peripheral.get() == WAND_ADDR )
      printf(" WB -> %03X", (int)(data56 & 0xFFF));
    break;
  }
  // Clear pending flag ...
  pending_data_inst = 0;

  // Check for instructions
  if (sync) {
    switch (inst) {
    case INST_DISPLAY_OFF:
      display_on = !false; // Toggles to false below ...:P
    case INST_DISPLAY_TOGGLE:
      display_on = !display_on;
#if CF_DBG_DISP_ON
      printf(" - Display %s", display_on ? "ON" : "OFF");
#endif
      dump_dregs();
      break;
#if CF_DBG_SLCT
    case INST_PRPH_SLCT:
    case INST_RAM_SLCT:
      printf("\n\r%s_SLCT: PA=%02X", inst == INST_PRPH_SLCT ? "PRPH" : "RAM", pa);
      break;
#endif
    case INST_WRITE:
    case INST_FETCH:
      // Handle instruction in next cycle ...
      pending_data_inst = inst;
      break;
    case INST_POWOFF:
      dump_dregs();
      // Any QROM that needs to be saved ... ?
      // TBD - Should this be done here? Or manually?
      for(int p=FIRST_PAGE; p<NR_PAGES; p++) {
        CModule *m = modules[p];
        if( m->isLoaded() && m->isQROM() && m->isDirty() ) {
          //saveRam(p);
          //sprintf(cbuff, "Updated QROM in page #X\n\r", p);
          //cdc_send_string_and_flush(ITF_CONSOLE, cbuff);
        }
      }
#ifdef USE_XF_MODULE
      if( xmem.dirty() ) {
        extern void dump_xmem(void);
        dump_xmem();
        saveXMem(0);
      }
#endif
      break;
    }
  }

  if( peripheral.get() && sync) {
    switch(peripheral.get()) {
    case WAND_ADDR:
      // Check for Wand transactions
      if (inst == INST_WANDRD)
        pending_data_inst = inst;
      break;
    case DISP_ADDR:
      // Check for display transactions
      if (inst == INST_WRITE_ANNUNCIATORS)
        pending_data_inst = inst;
      else {
        switch (inst & INST_RW_MASK) {
        case INST_WRITE_DATA:
#if CF_DBG_DISP_INST
          printf("\n\r%s", inst50disp[inst >> 6]);
#endif
          pending_data_inst = inst;
          break;
        case INST_READ_DATA:
#if CF_DBG_DISP_INST
          printf("\n\r%s", inst70disp[inst >> 6]);
#endif
          rotateDispReg(inst >> 6);
          break;
        }
      }
      break;
    }
  }
}

