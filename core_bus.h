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
#define  ROTATE_LEFT(V) {uint64_t t = (V>>44) & 0xF; V <<= 4; V |= t;}

// #define EMBED_RAM
#define RAM_SIZE    0x1000
#define PAGE_SIZE   0x1000
#define PAGE_MASK   0x0FFF
#define ADDR_MASK   0xFFFF
#define INST_MASK   0x3FF
#define FIRST_PAGE  0x04
#define NR_PAGES    0x10
#define LAST_PAGE   (NR_PAGES - 1)
#define PAGE(p)     (p>>12)
#define ISA_SHIFT   44
//#define TRACE_ISA
#define QUEUE_STATUS

#define TIMER_CNT1  75
#define TIMER_CNT2  825
#define BUSY_CNT    8

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
    BIT_13  = 1 << 13
};

#define BLINKY_RAM_ENABLE   BIT_4
#define BLINKY_CLK_ENABLE   BIT_6
#define BLINKY_ENABLE       BIT_7

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

#define CMD_SYNC    0x8000

typedef struct {
  uint64_t  data;
#ifdef TRACE_ISA
  uint64_t  isa;
#endif
  uint16_t  addr;
  uint16_t  cmd;
  uint16_t  fi;
  uint16_t  cnt;
} __attribute__((packed)) Bus_t;

// Size of trace buffer should be a power of 2 (for the mask)
#ifdef TRACE_ISA
#define NUM_BUS_T 0x400
#else
#define NUM_BUS_T (0x1000/2)
#endif

enum {
    IMG_NONE = 0x00,
    IMG_INSERTED = 0x01,
    IMG_RAM = 0x02,
    IMG_DIRTY = 0x04
};

static uint8_t nBank[4] = {0,2,1,3};

class CModule {
  uint16_t  *m_banks[4];
  uint16_t  *m_img;
  uint16_t  m_flgs;
  int       m_bank;
  int       m_port;
public:
  CModule() {
    m_img = NULL;
    m_bank = 0;
    m_port = 0;
  }
  void set(uint16_t *image, int bank = 0) {
    m_banks[bank] = image;
    m_img = m_banks[0];
    m_flgs = IMG_INSERTED;
  }
  void bank(int bank) {
    // Convert from instruction to bank #
    bank = nBank[(bank>>6)&0b11];
    if(m_banks[bank])
      m_img = m_banks[bank];
  }
  void port(int p) {
    m_port = p;
  }
  bool haveBank(void) {
    return m_banks[1] ? true : false;
  }
  void clr(void) {
    m_img = NULL;
    m_flgs = IMG_NONE;
  }
  bool isDirty(void) {
    return m_flgs & IMG_DIRTY;
  }
  void clear(void) {
    m_flgs &= ~IMG_DIRTY;
  }
  bool isInserted(void) {
    return m_flgs & IMG_INSERTED;
  }
  bool isLoaded(void) {
    return m_img != NULL;
  }
  bool isRam(void) {
    return m_flgs & IMG_RAM;
  }
  void togglePlug(void) {
    m_flgs ^= IMG_INSERTED;
  }
  void plug(void) {
    m_flgs |= IMG_INSERTED;
  }
  void unplug(void) {
    m_flgs &= ~IMG_INSERTED;
  }
  void toggleRam(void) {
    m_flgs ^= IMG_RAM;
  }
  void setRam(void) {
    m_flgs |= IMG_RAM;
  }
  void setRom(void) {
    m_flgs &= ~IMG_RAM;
  }
  uint16_t read(uint16_t addr) {
    return m_img[addr & PAGE_MASK] & INST_MASK;
  }
  uint16_t operator [](uint16_t addr) {
    return m_img[addr & PAGE_MASK] & INST_MASK;
  }
  void write(uint16_t addr, uint16_t dta) {
    m_img[addr & PAGE_MASK] = dta;
    m_flgs |= IMG_DIRTY;
  }
};

class CModules {
  CModule m_modules[NR_PAGES];
public:
  CModules() {
    clearAll();
  }
  void add(int port, uint16_t *image, int bank) {
	  if( image ) {
      m_modules[port].set(image, bank);
      printf("Add ROM @ %04X - %04X [bank %d]\n", port * PAGE_SIZE, (port * PAGE_SIZE)|PAGE_MASK, bank);
  	} 
  }
  void clearAll() {
    memset(m_modules, 0, sizeof(CModule) * NR_PAGES);
    for(int p=0; p<NR_PAGES; p++)
      m_modules[p].port(p);
  }
  void remove(int port) {
    m_modules[port].clr();
	}
  bool isDirty(int port) {
    return m_modules[port].isDirty();
  }
  void clear(int port) {
    m_modules[port].clear();
  }
  bool isInserted(int port) {
    return m_modules[port].isInserted();
  }
  bool isLoaded(int port) {
    return m_modules[port].isLoaded();
  }
  bool isRam(int port) {
    return m_modules[port].isRam();
  }
  void togglePlug(int port) {
    m_modules[port].togglePlug();
  }
  void unplug(int port) {
    m_modules[port].unplug();
  }
  void toggleRam(int port) {
    m_modules[port].toggleRam();
  }
  void setRam(int port) {
    m_modules[port].setRam();
  }
  void setRom(int port) {
    m_modules[port].setRom();
  }
  CModule *at(int addr) {
    return &m_modules[PAGE(addr)];
  }
  CModule *operator [](uint16_t addr) {
    return &m_modules[addr & 0xF];
  }

};


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
/**
// Memory and registers for Blinky module
typedef struct {
  uint64_t reg[8];    // First 8 registers
  uint8_t  reg8[16];  // Last 8 registers (0-7 not used)
//  uint64_t ram[16];
  uint8_t  flags;
  int nAlm;
  int cntTimer;
  int bwr;
  int busyCnt;
} Blinky_t;
**/
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

#define USE_XFUNC
#define USE_XMEM1
#define USE_XMEM2

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
#define XF_PAGE         (NR_PAGES+1)

// Memory for Extended Memory module
class CXFM {
  uint8_t m_chksum;
public:
  uint64_t m_mem[XMEM_XF_SIZE+XMEM_XM1_SIZE+XMEM_XM2_SIZE];
//  volatile uint64_t mem[XMEM_XF_SIZE+XMEM_XM1_SIZE+XMEM_XM2_SIZE];
  volatile uint64_t *mem;
  bool bDirty;
  int bwr;
  bool dirty() {
    return memcmp((void*)mem, (void*)m_mem, sizeof(m_mem))?true:false;
  }
  void saveMem(void) {
    memcpy((void*)m_mem, (void*)mem, sizeof(m_mem));
    doChksum();
  }
  int size(void) {
    return (int)sizeof(m_mem);
  }
  void doChksum(void) {
    uint8_t *p = (uint8_t*)m_mem;
    m_chksum = 0;
    for(int i=0; i<size()/sizeof(uint8_t); i++)
      m_chksum += *p++;
  }
  uint8_t *pChkSum(void) {
    return (uint8_t*)&m_chksum;
  }
  uint8_t chkSum(void) {
    return m_chksum;
  }
  int chkSize(void) {
    return (int)sizeof(m_chksum);
  }
};

void initRoms(void);
void initXMem(int xpg);

#endif//__CORE_BUS_H__
