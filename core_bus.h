#ifndef __CORE_BUS_H__
#define __CORE_BUS_H__
////////////////////////////////////////////////////////////////////////////////
//
// HP41C stuff
//
////////////////////////////////////////////////////////////////////////////////

#include "instr.h"


#define MASK_64_BIT    (0xFFFFFFFFFFFFFFFFL)
#define MASK_56_BIT    (0x00FFFFFFFFFFFFFFL)
#define MASK_48_BIT    (0x0000FFFFFFFFFFFFL)
#define REG_C_48_MASK  (0x0000111111111111L)

#define PA_MASK        (~MASK_56_BIT)
#define PA_SHIFT       56

#define ROTATE_RIGHT(V) {uint64_t t = (V & 0xF); V >>= 4; V |= t<<44;}
#define ROTATE_LEFT(V)  {uint64_t t = (V>>44) & 0xF; V <<= 4; V |= t;}

// #define EMBED_RAM
#define RAM_SIZE    0x1000
#define PAGE_SIZE   0x1000
#define PAGE_MASK   0x0FFF
#define ADDR_MASK   0xFFFF
#define INST_MASK   (BIT_10-1)
#define FIRST_PAGE  0x04
#define NR_PAGES    0x10
#define LAST_PAGE   (NR_PAGES - 1)
#define PAGE(p)     (p>>12)
#define ISA_SHIFT   44
//#define TRACE_ISA
#define QUEUE_STATUS

// One bus cycle is about 1/6300 = 160us
#define ONE_BUS_CYCLE 160 //us

// We're going to erase and reprogram a region 256k from the start of flash.
// Once done, we can access this at XIP_BASE + 256k.
#define FLASH_TARGET_OFFSET (512 * 1024)

uint32_t getTotalHeap(void);
uint32_t getFreeHeap(void);
const uint8_t *flashPointer(int offs);

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
    FI_PBSY  = 1 << 0,  // Printer BuSY
    FI_CRDR  = 1 << 1,  // CaRD Reader
    FI_WNDB  = 1 << 2,  // WaND Byte available
    FI_PF3   = 1 << 3,  // 3 - not used
    FI_PF4   = 1 << 4,  // 4 - not used
    FI_EDAV  = 1 << 5,  // Emitter Diode AVailable
    FI_IFCR  = 1 << 6,  // InterFace Clear Received
    FI_SRQR  = 1 << 7,  // Service ReQuest Received
    FI_FRAV  = 1 << 8,  // FRame AVailable
    FI_FRNS  = 1 << 9,  // Frame Received Not as Sent
    FI_ORAV  = 1 << 10, // Output Register AVailable
    FI_TFAIL = 1 << 11, //
    FI_ALM   = 1 << 12, // ALarM
    FI_SERV  = 1 << 13, // SERVice
    FI_MASK = (1 << 14)-1
};

#define FI_PRT_TIMER  FI_ALM
#define FI_PRT_BUSY   FI_TFAIL

#define CMD_SYNC    BIT_15

#define ARITHM_UKN  0x0000
#define ARITHM_DEC  BIT_13
#define ARITHM_HEX  BIT_14
#define ARITHM_MSK  (BIT_14|BIT_13)

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
#define NUM_BUS_T (0x1000/2)
#endif


#define CHK_GPIO(x) (sio_hw->gpio_in & (1 << x))

#define INIT_PIN(n,io,def)      \
    do {                        \
        gpio_init(n);           \
        gpio_set_dir(n, io);    \
        if( io == GPIO_OUT )    \
            gpio_put(n, def);   \
    } while(0)

#define GPIO_PIN(x) (gpio_states & (1 << x))
#define GPIO_PIN_RD(x) ((gpio_states = sio_hw->gpio_in) & (1 << x))
#define FI(x) GPIO_PIN_RD(x)
#define WAIT_FALLING(x) do { while(FI(x) == LOW ) {} while(FI(x) == HIGH ) {} } while(0)
#define WAIT_RISING(x) do { while(FI(x) == LOW ) {} } while(0)
#define LOW 0
#define HIGH 1

extern void core1_main_3(void);

extern void process_bus(void);

extern volatile int data_wr;
extern volatile int data_rd;

#define BRK_SIZE ((ADDR_MASK+1)/(32/2)) // 2 bits per brkpt
#define BRK_MASK(a,w) ((m_brkpt[a>>4]w >> ((a & 0xF)<<1)) & 0b11)
#define BRK_SHFT(a) ((a & 0xF)<<1)
#define BRK_WORD(a) (a >> 4)

#define START_TRACE() bTrace |= TRACE_BRK
#define END_TRACE()   bTrace &= ~TRACE_BRK

typedef enum {
  BRK_NONE,
  BRK_START,
  BRK_END,
  BRK_CLR
} BrkMode_e;

class CBreakpoint {
  uint32_t m_brkpt[BRK_SIZE];
public:
  // Check if breakpoint is set for given address
  inline BrkMode_e isBrk(uint16_t addr) {
    uint32_t w = m_brkpt[BRK_WORD(addr)];
    return w ? (BrkMode_e)((w >> BRK_SHFT(addr)) & 0b11) : BRK_NONE;
  }
  // Clear breakpoint on given address
  void clrBrk(uint16_t addr) {
    m_brkpt[BRK_WORD(addr)] &= ~(0b11 << BRK_SHFT(addr));
  }
  void clrAllBrk(void) {
    printf("Clear all breakpoints\n");
    memset(m_brkpt, 0, sizeof(uint32_t) * BRK_SIZE);
  }
  // Set breakpoint on given address
  void setBrk(uint16_t addr) {
    clrBrk(addr); // Remove previous settings
    m_brkpt[BRK_WORD(addr)] |= BRK_START << BRK_SHFT(addr);
  }
  void stopBrk(uint16_t addr) {
    clrBrk(addr); // Remove previous settings
    m_brkpt[BRK_WORD(addr)] |= BRK_END << BRK_SHFT(addr);
  }
  void list_brks(void) {
    int n = 0;
    for(int w=0; w<BRK_SIZE; w++) {
      if( m_brkpt[w] ) {
        for(int a=w*16; a<(w+1)*16; a++) {
          int br = isBrk(a);
          if( br ) {
            printf("#%d: %04X -> ", ++n, a);
            switch( br ) {
              case 1: printf("start"); break;
              case 2: printf("stop"); break;
              default: printf("???"); break;
            }
            printf("\n");
          }
        }
      }
    }
    if( !n )
      printf("No breakpoints\n");
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
    case DISP_ADDR:
    case WAND_ADDR:
    case TIMR_ADDR:
    case CRDR_ADDR:
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
    case DISP_ADDR:
    case WAND_ADDR:
    case TIMR_ADDR:
    case CRDR_ADDR:
      n = sprintf(pb, "%c ", "TCDW"[get() - TIMR_ADDR]);
      break;
    default:
      n = sprintf(pb, "  ");
    }
    return n;
  }
};

extern volatile uint16_t carry_fi;

inline void setFI(int flag) {
  carry_fi |= flag;
}
inline void clrFI(int flag = FI_MASK) {
  carry_fi &= ~flag;
}

#define PRT_BUF_LEN 256
extern volatile uint8_t prtBuf[PRT_BUF_LEN];
extern volatile uint8_t wprt;

void initRoms(void);

#ifdef USE_XF_MODULE
#include "xfmem.h"
#endif
#ifdef USE_TIME_MODULE
#include "timemod.h"
#endif
#include "blinky.h"

#endif//__CORE_BUS_H__
