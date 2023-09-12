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

typedef struct {
  uint64_t  data;
  uint16_t  addr;
  uint16_t  cmd;
  uint8_t   pa;
  uint8_t   sync;
} Bus_t;

typedef struct {
    uint16_t start;
    uint16_t end;
    uint16_t *image;
    bool     inserted;
    bool     isRam;
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

extern volatile int data_in;
extern volatile int data_out;

#endif//__CORE_BUS_H__
