#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/multicore.h"
#include "tiny41.h"

const uint LED_PIN = TINY2040_LED_B_PIN;


extern void core1_main_3(void);
extern volatile int sync_count;
extern volatile int embed_seen;


void bus_init(void)
{
  gpio_init(P_CLK1);
  gpio_set_dir(P_CLK1, GPIO_IN);
  gpio_init(P_CLK2);
  gpio_set_dir(P_CLK2, GPIO_IN);
  gpio_init(P_SYNC);
  gpio_set_dir(P_SYNC, GPIO_IN);
  gpio_init(P_ISA);
  gpio_set_dir(P_ISA, GPIO_IN);
  gpio_init(P_DATA);
  gpio_set_dir(P_DATA, GPIO_IN);
  gpio_init(P_VBAT);
  gpio_set_dir(P_VBAT, GPIO_IN);
}


int main()
{
    stdio_init_all();

    bus_init();

    printf("Hello, SSD1306 OLED display! Look at my raspberries..\n");

    // I2C is "open drain", pull ups to keep signal high when no data is being
    // sent
    i2c_init(i2c_default, SSD1306_I2C_CLK * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    // run through the complete initialization process
    SSD1306_init();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    multicore_launch_core1(core1_main_3);

    // Initialize render area for entire frame (SSD1306_WIDTH pixels by SSD1306_NUM_PAGES pages)
    struct render_area frame_area = {
        start_col: 0,
        end_col : SSD1306_WIDTH - 1,
        start_page : 0,
        end_page : SSD1306_NUM_PAGES - 1
        };

    calc_render_area_buflen(&frame_area);

    // zero the entire display
    uint8_t buf[SSD1306_BUF_LEN];
    memset(buf, 0, SSD1306_BUF_LEN);
    render(buf, &frame_area);
/*
    // intro sequence: flash the screen 3 times
    for (int i = 0; i < 3; i++) {
        SSD1306_send_cmd(SSD1306_SET_ALL_ON);    // Set all pixels on
        sleep_ms(500);
        SSD1306_send_cmd(SSD1306_SET_ENTIRE_ON); // go back to following RAM for pixel state
        sleep_ms(500);
    }
*/
    const char *text[] = {
        "HELLO WORLD\x01",
        "3.1415926539"
    };
    for( int x=0; x<12; x++) {
        Write41Char(buf, 5+x*10, 0, (char)text[0][x]);
        //Write41Char(buf, 5+x*10, 16, (char)x+48);
    }
    render(buf, &frame_area);

    int z=0;
    char sBuf[16];
    while (1) {
        gpio_put(LED_PIN, 0);
        sleep_ms(250);
        gpio_put(LED_PIN, 1);
        //printf("Hello HP41!\n");
        sleep_ms(250);
        for( int x=0; x<12; x++) {
            //Write41Char(buf, 5+x*10, 0, (char)x+z);
        }
        for( int x=2; x<8; x++) {
            if( gpio_get(x) )
                Write41Char(buf, 5+x*10, 16, (char)48);
            else
                Write41Char(buf, 5+x*10, 16, (char)49);
        }
        sprintf(sBuf, "S:%d E:%d", sync_count, embed_seen);
        WriteString(buf, 5, 32, sBuf);
        render(buf, &frame_area);
        z++;
        if( z > 127 )
            z = 0;
    }
}