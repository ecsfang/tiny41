#undef PICO_DEFAULT_I2C_SDA_PIN
#undef PICO_DEFAULT_I2C_SCL_PIN
//#undef PICO_DEFAULT_I2C

#define PICO_DEFAULT_I2C_SDA_PIN 26
#define PICO_DEFAULT_I2C_SCL_PIN 27
//#define PICO_DEFAULT_I2C 1

#include "ssd1306.h"

#define P_DATA  2
#define P_ISA   3
#define P_SYNC  4
#define P_CLK2  5
#define P_CLK1  6
#define P_VBAT  7

