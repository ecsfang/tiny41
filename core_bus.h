#ifndef __CORE_BUS_H__
#define __CORE_BUS_H__
////////////////////////////////////////////////////////////////////////////////
//
// HP41C stuff
//
////////////////////////////////////////////////////////////////////////////////

#include "instr.h"
#include "ramdev.h"
#include "fltools/fltools.h"
#include "fltools/flconfig.h"
#include "usb/cdc_helper.h"
#include "hardware/flash.h"

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
#define RAM_SIZE    0x1000
#define PAGE_SIZE   0x1000
#define ROM_SIZE    0x1000
#define PAGE_MASK   0x0FFF
#define ADDR_MASK   0xFFFF
#define INST_MASK   (BIT_10-1)
#define FIRST_PAGE  0x04
#define NR_PAGES    0x10
#define NR_BANKS    4
#define LAST_PAGE   (NR_PAGES - 1)
#define PAGE(p)     (p>>12)
#define ISA_SHIFT   44
//#define TRACE_ISA
#define QUEUE_STATUS

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
#define NUM_BUS_T (0x1000)
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

#define FLASH_PAGE(n) (FLASH_START + (n) * FLASH_SECTOR_SIZE)

/*
There is a very simple FAT table, consiting of:
name (<24 characters describing the entry)
offset - offset in flash for the start of the data
type   - type of entry: MOD/RAM/ROM
Last entry have offset == 0
4 pages are reserved for the FAT:
--> 4 * 4 * 1024 / 32 bytes -> 512 entries (modules)
*/

#if defined(PIMORONI_PICOLIPO_16MB)
#define FLASH_SIZE  ((16*1024*1024)-1)
#elif defined(PIMORONI_TINY2040_8MB)
#define FLASH_SIZE  ((8*1024*1024)-1)
#elif defined(RASPBERRYPI_PICO2)
#define FLASH_SIZE  ((4*1024*1024)-1)
#else
#error("Must define a board!")
#endif

class CFat_t {
  FL_Head_t *p_fatEntry;
  int m_pos;
public:
  CFat_t(FL_Head_t *p=NULL) {
    init(p);
  }
  void init(FL_Head_t *p=NULL) {
    p_fatEntry = p;
    m_pos = 0; // Not used - we just init the pointer ...
    // TBD - We could search for the entry with the current pointer?
  }
  // Read next entry in the FAT (and increment pointer)
  void next() {
    // Reads FAT entry
    p_fatEntry = (FL_Head_t*)(FAT_START + m_pos*sizeof(FL_Head_t));
    m_pos++;
  }
  // Read first entry in the FAT
  void first() {
    m_pos = 0;
    next();
  }
  // Find a given module name in the FAT.
  // Return position [1..n] if found, otherwise 0
  int find(const char *mod) {
    first();
    while( offset() ) {
      if( strcmp(p_fatEntry->name, mod) == 0 )
        return m_pos;
      next();
    }
    return 0;
  }
  // Return offset to the file data for current FAT entry
  const char *offset(void) {
    if( p_fatEntry->offs > (FLASH_SIZE+XIP_BASE) ||
        p_fatEntry->offs < XIP_BASE )
      return NULL;
    return (char*)p_fatEntry->offs;
  }
  FL_Head_t *fatEntry() {
    return p_fatEntry;
  }
  // Return the module name from the FAT
  char *name(void) {
    return p_fatEntry->name;
  }
  // Return the module type from the FAT
  int type(void) {
    return p_fatEntry->type;
  }
};

#define CONF_DESC_LEN 64
// With current size (16*(4*4+4)+64+4 = 388 bytes), there is room
// for at least 10 configurations in one flash page (216 bytes left)
typedef struct {
  FL_Head_t *fat[NR_BANKS];             // Pointer to FAT entry
  uint8_t    filePage[NR_BANKS];        // Page in the file image
} ModuleConfig_t;
typedef struct {
  char            desc[CONF_DESC_LEN];  // Description of the config
  ModuleConfig_t  mod[NR_PAGES];        // The actual page config
  uint32_t        chkSum;               // Checksum of the config
} Config_t;

typedef struct {
  uint32_t config;
  uint32_t chkSum;               // Checksum of the config
} Setup_t;

#define CONF_SIZE     sizeof(Config_t)  // Total size of the config
#define CONF_OFFS(n)  (n*CONF_SIZE)     // Offset in flash for the config

// Place setup directly after config (up to 216 bytes)
#define SETUP_SIZE    sizeof(Setup_t)   // Total size of the config
#define SETUP_OFFS    (10*CONF_SIZE)    // Offset in flash for the config

extern void initRoms(void);

#define PRT_BUF_LEN 256
extern volatile uint8_t prtBuf[PRT_BUF_LEN];
extern volatile uint8_t wprt;

extern void readFlash(int offs, uint8_t *data, uint16_t size);
extern uint16_t *writeROMMAP(int p, int b, uint16_t *data);
extern uint16_t *writePage(int addr, uint8_t *data);
extern uint16_t *writeConfig(Config_t *data, int set=0, bool bChk=true);
extern bool readConfig(Config_t *data, int set);
extern uint16_t *writeSetup(Setup_t *data);
extern bool      readSetup(Setup_t *data);

#ifdef USE_XF_MODULE
#include "xfmem.h"
#endif
#ifdef USE_TIME_MODULE
#include "timemod.h"
#endif

#endif//__CORE_BUS_H__
