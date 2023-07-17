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

#define LED_ON  0
#define LED_OFF 1

#define LED_PIN_R TINY2040_LED_R_PIN
#define LED_PIN_G TINY2040_LED_G_PIN
#define LED_PIN_B TINY2040_LED_B_PIN

