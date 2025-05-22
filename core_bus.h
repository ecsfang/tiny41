#ifndef __CORE_BUS_H__
#define __CORE_BUS_H__
////////////////////////////////////////////////////////////////////////////////
//
// HP41C stuff
//
////////////////////////////////////////////////////////////////////////////////

#include "instr.h"
#include "ramdev.h"
//#include "fltools/fltools.h"
#include "fltools/flconfig.h"
#include "usb/cdc_helper.h"
//#include "hardware/flash.h"
#include "flash.h"

#include "tiny41.h"

#define MASK_64_BIT    (0xFFFFFFFFFFFFFFFFL)
#define MASK_56_BIT    (0x00FFFFFFFFFFFFFFL)
#define MASK_48_BIT    (0x0000FFFFFFFFFFFFL)
#define REG_C_48_MASK  (0x0000111111111111L)

#define PA_MASK        (~MASK_56_BIT)
#define PA_SHIFT       56

#define ROTATE_RIGHT(V) {uint64_t t = (V & 0xF); V >>= 4; V |= t<<44;}
#define ROTATE_LEFT(V)  {uint64_t t = (V>>44) & 0xF; V <<= 4; V |= t;}

// #define EMBED_RAM
#define INST_MASK   (BIT_10-1)
#define FIRST_PAGE  0x00 //0x04
#define PAGE(p)     (p>>12)
#define ISA_SHIFT   44

//#define TRACE_ISA
#define QUEUE_STATUS

#define MAX_ID  0x1F    // Max ID # of a module
#define MAX_FNS 0x40    // Max functions entries in a module FAT

#define PAGEn(n,i) (FLASH_START + (2 * (n) + i) * FLASH_SECTOR_SIZE)
#define PAGE1(n) PAGEn(n,0)
#define PAGE2(n) PAGEn(n,1)

#define XMEM_XF_START   0x40
#define XMEM_XF_END     0xC0
#define XMEM_XF_SIZE    (XMEM_XF_END-XMEM_XF_START)
#define XMEM_XF_OFFS    0x000
#define XMEM_XM1_START  0x201
#define XMEM_XM1_END    0x2F0
#define XMEM_XM1_SIZE   (XMEM_XM1_END-XMEM_XM1_START)
#define XMEM_XM1_OFFS   XMEM_XF_SIZE
#define XMEM_XM2_START  0x301
#define XMEM_XM2_END    0x3F0
#define XMEM_XM2_SIZE   (XMEM_XM2_END-XMEM_XM2_START)
#define XMEM_XM2_OFFS   (XMEM_XF_SIZE+XMEM_XM1_SIZE)
// Start of XMEMORY
#define XF_PAGE         (0)
#define XF_PAGES        (8)
// Start of module FAT
//#define FAT_PAGE        (XF_PAGE+XF_PAGES)
//#define FAT_PAGES       (4)
// Start of module pages
//#define MOD_PAGE        (FAT_PAGE+FAT_PAGES)

// One bus cycle is about 1/6300 = 160us
#define ONE_BUS_CYCLE 160 //us

// We're going to erase and reprogram a region 256k from the start of flash.
// Once done, we can access this at XIP_BASE + 256k.
///#define FLASH_TARGET_OFFSET (512 * 1024)

uint32_t getTotalHeap(void);
uint32_t getFreeHeap(void);

extern char cbuff[CDC_PRINT_BUFFER_SIZE];
extern int send2console(const char* dispBuf, bool bClear);

typedef enum {
  NO_MODE,
  DEEP_SLEEP,
  LIGHT_SLEEP,
  RUNNING
} Mode_e;

enum {
    BIT_0   = 1 << 0,
    BIT_1   = 1 << 1,
    BIT_2   = 1 << 2,
    BIT_3   = 1 << 3,
    BIT_4   = 1 << 4,
    BIT_5   = 1 << 5,
    BIT_6   = 1 << 6,
    BIT_7   = 1 << 7,
    BIT_8   = 1 << 8,
    BIT_9   = 1 << 9,
    BIT_10  = 1 << 10,
    BIT_11  = 1 << 11,
    BIT_12  = 1 << 12,
    BIT_13  = 1 << 13,
    BIT_14  = 1 << 14,
    BIT_15  = 1 << 15
};

enum {
    FI_NONE  = 0,
    FI_PBSY  = BIT_0,  // Printer BuSY
    FI_CRDR  = BIT_1,  // CaRD Reader
    FI_WNDB  = BIT_2,  // WaND Byte available
    FI_PF3   = BIT_3,  // 3 - not used
    FI_PF4   = BIT_4,  // 4 - not used
    FI_EDAV  = BIT_5,  // Emitter Diode AVailable
    FI_IFCR  = BIT_6,  // InterFace Clear Received
    FI_SRQR  = BIT_7,  // Service ReQuest Received
    FI_FRAV  = BIT_8,  // FRame AVailable
    FI_FRNS  = BIT_9,  // Frame Received Not as Sent
    FI_ORAV  = BIT_10, // Output Register AVailable
    FI_TFAIL = BIT_11, //
    FI_ALM   = BIT_12, // ALarM
    FI_SERV  = BIT_13, // SERVice
    FI_MASK = (BIT_14)-1
};

#define FI_PRT_TIMER  FI_ALM
#define FI_PRT_BUSY   FI_TFAIL

#define CMD_SYNC    BIT_15

#define ARITHM_UKN  0x0000
#define ARITHM_DEC  BIT_13
#define ARITHM_HEX  BIT_14
#define ARITHM_MSK  (BIT_14|BIT_13)
#define ARITHM_SHFT 13

typedef struct {
  uint64_t  data; // 56 bit data bus
                  // 8 MSB used to indicate selected peripherial
#ifdef TRACE_ISA
  uint64_t  isa;
#endif
  uint16_t  addr; // Address from ISA
  uint16_t  cmd;  // Command from ISA (bit 0-8)
                  // bit 13+14 indicates arithmetic mode (DEC/HEX)
                  // bit 15 indicates SYNC
  uint16_t  fi;   // FI flags
  uint16_t  cnt;  // Trace counter
} __attribute__((packed)) Bus_t;

// Size of trace buffer should be a power of 2 (for the mask)
#ifdef TRACE_ISA
#define NUM_BUS_T 0x400
#else
#define NUM_BUS_T (0x1000<<2)
#endif

#define CHK_GPIO(x) (sio_hw->gpio_in & (1 << x))

#define INIT_PIN(n,io,def)      \
    do {                        \
        gpio_init(n);           \
        gpio_set_dir(n, io);    \
        if( io == GPIO_OUT )    \
            gpio_put(n, def);   \
    } while(0)

// Check a GPIO state from current state
#define GPIO_PIN(x) (gpio_states & (1 << x))
// Check a GPIO state and update current state
#define GPIO_PIN_RD(x) ((gpio_states = sio_hw->gpio_in) & (1 << x))

#define LOW 0
#define HIGH 1
#define FI(x) GPIO_PIN_RD(x)
// Check low, then high, then wait until low again
#define WAIT_FALLING(x)     \
  do {                      \
    while(FI(x) == LOW ) {  \
    }                       \
    while(FI(x) == HIGH ) { \
    }                       \
  } while(0)

// Wait for tranision from low to high
#define WAIT_RISING(x) do { while(FI(x) == LOW ) {} } while(0)

extern void core1_main_3(void);

extern void process_bus(void);

extern volatile int data_wr;
extern volatile int data_rd;

#define BRK_BANKS 0
#if BRK_BANKS > 0
#define BPBR 2 // Bits per breakpoint
#define BPBK 2 // Bits per bank (4 banks)
#define BRK_APW (32/(1<<(BPBR+BPBK))) // Addresses Per Word (8)
#define BRK_SIZE ((ADDR_MASK+1)/BRK_APW) // Total size
#define BRK_WORD(a) (a >> 2) // /8
#define BRK_SHFT(a,b) ((a & 0x3)*8 + b*2)
#else
#define BPB 2 // Bits per breakpoint (2 type + 0 bank)
#define BRK_APW (32/BPB) // Addresses Per Word
#define BRK_SIZE ((ADDR_MASK+1)/BRK_APW) // 2 bits per brkpt
#define BRK_WORD(a) (a >> 4) // /16
#define BRK_SHFT(a,b) ((a & 0xF)<<1) // *2
#endif

#define START_TRACE() bTrace |= TRACE_BRK
#define END_TRACE()   bTrace &= ~TRACE_BRK

typedef enum {
  BRK_NONE,
  BRK_START,
  BRK_END,
  BRK_MASK
} BrkMode_e;

class CBreakpoint {
  uint32_t m_brkpt[BRK_SIZE];
public:
  // Check if breakpoint is set for given address
  inline BrkMode_e isBrk(uint16_t addr, int bank = 0) {
    uint32_t w = m_brkpt[BRK_WORD(addr)];
    return w ? (BrkMode_e)((w >> BRK_SHFT(addr,bank)) & BRK_MASK) : BRK_NONE;
  }
  // Clear breakpoint on given address
  void clrBrk(uint16_t addr, int bank = 0) {
    m_brkpt[BRK_WORD(addr)] &= ~(BRK_MASK << BRK_SHFT(addr,bank));
  }
  void clrAllBrk(void) {
    sprintf(cbuff, "Clear all breakpoints\r\n");
    cdc_send_string(ITF_CONSOLE, cbuff);
    memset(m_brkpt, 0, sizeof(uint32_t) * BRK_SIZE);
  }
  // Set breakpoint on given address
  void setBrk(uint16_t addr, int bank = 0) {
    clrBrk(addr, bank); // Remove previous settings
    m_brkpt[BRK_WORD(addr)] |= BRK_START << BRK_SHFT(addr,bank);
  }
  void stopBrk(uint16_t addr, int bank = 0) {
    clrBrk(addr, bank); // Remove previous settings
    sprintf(cbuff, "Clear breakpoint @ %04X\r\n", addr);
    cdc_flush_console();
    m_brkpt[BRK_WORD(addr)] |= BRK_END << BRK_SHFT(addr,bank);
  }
  void list_brks(void) {
    int n = 0;
    int i=0;
    for(int w=0; w<BRK_SIZE; w++) {
      if( m_brkpt[w] ) {
        for(int a=w*BRK_APW; a<(w+1)*BRK_APW; a++) {
          int br = isBrk(a);
          if( br ) {
#if BRK_BANKS > 0
            i = sprintf(cbuff, "#%d bank%d: %04X -> ", ++n, br>>2, a);
#else
            i = sprintf(cbuff, "#%d: %04X -> ", ++n, a);
#endif
            switch( br ) {
              case 1: i += sprintf(cbuff+i, "start"); break;
              case 2: i += sprintf(cbuff+i, "stop"); break;
              default: i += sprintf(cbuff+i, "?%X", br); break;
            }
            i += sprintf(cbuff+i, "\n\r");
            cdc_send_string(ITF_CONSOLE, cbuff);
          }
        }
      }
    }
    if( !n )
      cdc_send_string(ITF_CONSOLE, (char*)"No breakpoints\n\r");
    cdc_flush_console();
  }
};

// Keep track of selected peripheral ...
class CPeripherial {
  int m_pa;
public:
  CPeripherial() {
    m_pa = 0;
  }
  void set(int pa) {
    switch (pa) {
    case TIMR_ADDR:
    case CRDR_ADDR:
    case DISP_ADDR:
    case WAND_ADDR:
      m_pa = pa;
      break;
    default:
      m_pa = NONE_ADDR;
    }
  }
  int get(void) {
    return m_pa;
  }
  int print(char *pb) {
    int n;
    switch (get()) {
    case TIMR_ADDR: // Time Module
    case CRDR_ADDR: // 82104 Card reader
    case DISP_ADDR: // 41 LCD display
    case WAND_ADDR: // 82153 Wand
      n = sprintf(pb, "%c ", "TCDW"[get() - TIMR_ADDR]);
      break;
    default:
      n = sprintf(pb, "  "); // None selected
    }
    return n;
  }
};

// carry_fi is holding the 14 bits corresponding
// to the flags in the FI bus signal
extern volatile uint16_t carry_fi;
// When true - set PBSY in next bus cycle
extern volatile bool bPBusy;
// When true - clear PBSY in next bus cycle
extern volatile bool bClrPBusy;

// Set a FI flag (set using a bitmask)
inline void setFI(int flag) {
  carry_fi |= flag;
}
// Clear a FI flag (bitmask, default all flags)
inline void clrFI(int flag = FI_MASK) {
  carry_fi &= ~flag;
}
// Check if a specified FI flag is set
inline uint16_t chkFI(int flag)
{
  return carry_fi & flag;
}
// Indicate to set PBSY next time ...
inline void _setFI_PBSY(void)
{
  bPBusy = true;
}
// Indicate to clear PBSY next time ...
inline void _clrFI_PBSY(void)
{
  bClrPBusy = true;
}
// Get the value corresponding to all FI flags
// to be set during the current bus-cycle
inline uint16_t getFI(void)
{
  if( bPBusy )
    setFI(FI_PBSY);
  else
    clrFI(FI_PBSY);
  if( bClrPBusy ) {
    bPBusy = false;
    bClrPBusy = false;
  }
  return carry_fi & FI_MASK;
}

#define PRT_BUF_LEN 256
extern volatile uint8_t prtBuf[PRT_BUF_LEN];
extern volatile uint8_t wprt;

#ifdef USE_XF_MODULE
#include "xfmem.h"
#endif
#ifdef USE_TIME_MODULE
#include "timemod.h"
#endif

#endif//__CORE_BUS_H__
