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
#define FIRST_PAGE  0x04
#define NR_PAGES    0x10
#define LAST_PAGE   (NR_PAGES - 1)
#define PAGE(p)     (p>>12)

//#define TRACE_ISA
#define QUEUE_STATUS

enum {
    FI_NONE = -1,
    FI_PBSY,    // Printer BuSY
    FI_CRDR,    // CaRD Reader
    FI_WNDB,    // WaND Byte available
    FI_PF3,     // 3
    FI_PF4,     // 4
    FI_EDAV,    // Emitter Diode AVailable
    FI_IFCR,    // InterFace Clear Received
    FI_SRQR,    // Service ReQuest Received
    FI_FRAV,    // FRame AVailable
    FI_FRNS,    // Frame Received Not as Sent
    FI_ORAV,    // Output Register AVailable
    FI_TFAIL,   //
    FI_ALM,     // ALarM
    FI_SERV     // SERVice
};

enum {
    T0 = 1 << 0,
    T1 = 1 << 1,
    T2 = 1 << 2,
    T3 = 1 << 3,
    T4 = 1 << 4,
    T5 = 1 << 5,
    T6 = 1 << 6,
    T7 = 1 << 7,
    T8 = 1 << 8,
    T9 = 1 << 9,
    T10 = 1 << 10 ,
    T11 = 1 << 11,
    T12 = 1 << 12,
    T13 = 1 << 13
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

/*
typedef struct {
    uint8_t sign    : 4;
    uint8_t d1      : 4;
    uint8_t d2      : 4;
    uint8_t d3      : 4;
    uint8_t d4      : 4;
    uint8_t d5      : 4;
    uint8_t d6      : 4;
    uint8_t d7      : 4;
    uint8_t d8      : 4;
    uint8_t d9      : 4;
    uint8_t d10     : 4;
    uint8_t esgn    : 4;
    uint8_t exp     : 8;
} Data_t;

typedef struct __attribute__ ((__packed__)) {
    uint16_t        : 14;
    uint16_t addr   : 16;
    uint32_t        : 14;
    uint16_t inst   : 10;
    uint8_t         : 2;
} ISA_t;

typedef union {
    ISA_t    x;
    uint64_t data;
} ISA_u;
*/
enum {
    IMG_NONE = 0x00,
    IMG_INSERTED = 0x01,
    IMG_RAM = 0x02,
    IMG_DIRTY = 0x04
};

typedef struct {
    //uint16_t start;
    //uint16_t end;
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
//extern void __not_in_flash_func(core1_main_3)(void);

extern void process_bus(void);
extern void capture_bus_transactions(void);

extern volatile int sync_count;
extern volatile int embed_seen;

extern volatile int data_wr;
extern volatile int data_rd;

// Set breakpoint on given address
void setBrk(uint16_t addr);
void stopBrk(uint16_t addr);
// Clear breakpoint on given address
void clrBrk(uint16_t addr);

#endif//__CORE_BUS_H__
