#ifndef __CORE_BUS_H__
#define __CORE_BUS_H__
////////////////////////////////////////////////////////////////////////////////
//
// HP41C stuff
//
////////////////////////////////////////////////////////////////////////////////

#include "instr.h"

#define MASK_48_BIT    (0xFFFFFFFFFFFFL)
#define REG_C_48_MASK  (0x111111111111L)

#define ROTATE_RIGHT(V) {uint64_t t = (V & 0xF); V >>= 4; V |= t<<44;}
#define  ROTATE_LEFT(V) {uint64_t t = (V & (0xFLL << 44)); V <<= 4; V |= t>>44;}

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

//#define TRACE_ISA
#define QUEUE_STATUS

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

typedef struct {
  uint64_t  data;
#ifdef TRACE_ISA
  uint64_t  isa;
#endif
  uint16_t  addr;
  uint16_t  cmd;
  //uint16_t  flag;
  uint16_t  fi;
  uint8_t   pa;
  uint8_t   sync;
} Bus_t;

enum {
    IMG_NONE = 0x00,
    IMG_INSERTED = 0x01,
    IMG_RAM = 0x02,
    IMG_DIRTY = 0x04
};

typedef struct {
    uint16_t *image;
    uint16_t flags;
} Module_t;


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
#define LOW 0
#define HIGH 1

extern void core1_main_3(void);

extern void process_bus(void);
extern void capture_bus_transactions(void);

extern volatile int sync_count;
extern volatile int embed_seen;

extern volatile int data_wr;
extern volatile int data_rd;

// Set breakpoint on given address
void setBrk(uint16_t addr);     // Start trace
void stopBrk(uint16_t addr);    // Stop trace
// Clear breakpoint on given address
void clrBrk(uint16_t addr);

#endif//__CORE_BUS_H__
