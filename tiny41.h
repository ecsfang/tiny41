#undef PICO_DEFAULT_I2C_SDA_PIN
#undef PICO_DEFAULT_I2C_SCL_PIN
//#undef PICO_DEFAULT_I2C

#define PICO_DEFAULT_I2C_SDA_PIN 26
#define PICO_DEFAULT_I2C_SCL_PIN 27
//#define PICO_DEFAULT_I2C 1

#include "ssd1306.h"

#define USE_40190

#ifdef USE_40190
#define ENABLE_OE   1
#define DISABLE_OE  0
#else
#define ENABLE_OE   0
#define DISABLE_OE  1
#endif


#define PIN_A2  28
#define PIN_A3  29

//#define CARD_1
#define CARD_2

#ifdef CARD_1
#define P_DATA    2
#define P_ISA     3
#define P_SYNC    4
#define P_CLK2    5
#define P_CLK1    6

#define P_FI_OE   1

#define P_ISA_OE  29
#define P_ISA_DRV 7
#endif

#ifdef CARD_2
#define P_DATA      2
#define P_DATA_DRV  7
#define P_DATA_OE   1

#define P_ISA       3
#define P_ISA_DRV   0
#define P_ISA_OE    PIN_A3

#define P_FI_OE     PIN_A2

#define P_SYNC      4
#define P_CLK2      5
#define P_CLK1      6
#endif


//#define P_DTA_DRV 0
//#define P_DTA_OE  28

#define LED_ON    0
#define LED_OFF   1

#define LED_PIN_R TINY2040_LED_R_PIN
#define LED_PIN_G TINY2040_LED_G_PIN
#define LED_PIN_B TINY2040_LED_B_PIN

#define NR_CHARS  12
#define NR_ANNUN  12

#define BUS_CYCLES 56
#define LAST_CYCLE (BUS_CYCLES-1)

extern void UpdateLCD(char *, bool *, bool);
extern void UpdateAnnun(uint16_t ann);

//#define TRACE
#define USE_FLASH

