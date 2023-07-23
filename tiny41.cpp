#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/multicore.h"
#include "tiny41.h"

extern void core1_main_3(void);
extern void process_bus(void);
extern void capture_bus_transactions(void);

extern volatile int sync_count;
extern volatile int embed_seen;

extern volatile int data_in;
extern volatile int data_out;

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
  gpio_pull_down(P_ISA);
  gpio_init(P_DATA);
  gpio_set_dir(P_DATA, GPIO_IN);
  gpio_init(P_ISA_OE);
  gpio_set_dir(P_ISA_OE, GPIO_OUT);
  gpio_put(P_ISA_OE, 1);
  gpio_init(P_ISA_DRV);
  gpio_set_dir(P_ISA_DRV, GPIO_OUT);
  gpio_put(P_ISA_DRV, 0);
}

uint8_t buf[SSD1306_BUF_LEN];

int main()
{
#if 1
  ////////////////////////////////////////////////////////////////////////////////
  //
  // Overclock as needed
  //
  ////////////////////////////////////////////////////////////////////////////////
  
  //#define OVERCLOCK 135000
  //#define OVERCLOCK 200000
#define OVERCLOCK 270000
  //#define OVERCLOCK 360000
  
#if OVERCLOCK > 270000
  /* Above this speed needs increased voltage */
  vreg_set_voltage(VREG_VOLTAGE_1_20);
  sleep_ms(1000);
#endif
  
  /* Overclock */
  set_sys_clock_khz( OVERCLOCK, 1 );
#endif

  stdio_init_all();

  sleep_ms(2000);

  printf("\n*************");
  printf("\n*  Tiny 41  *");
  printf("\n*************");
  printf("\n");
  
    bus_init();

    // I2C is "open drain", pull ups to keep signal high when no data is being
    // sent
    i2c_init(i2c_default, SSD1306_I2C_CLK * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    // run through the complete initialization process
    SSD1306_init();

    gpio_init(LED_PIN_R);
    gpio_init(LED_PIN_B);
    gpio_set_dir(LED_PIN_R, GPIO_OUT);
    gpio_set_dir(LED_PIN_B, GPIO_OUT);
    gpio_put(LED_PIN_R, LED_OFF);
    gpio_put(LED_PIN_B, LED_OFF);

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
    char *text[] = {
        (char*)" \x2c Tiny41 \x2e",
        (char*)"\x5e 3.1416"
    };
    bool bp[12];
    memset(bp, 0, 12);

    Write41String(buf, 5, 0, text[0], bp);
    bp[1] = true;
    Write41String(buf, 5, 16, text[1], bp);
/*    for( int x=0; x<12; x++) {
        Write41Char(buf, 5+x*10, 0, (char)text[0][x]);
        //Write41Char(buf, 5+x*10, 16, (char)x+48);
    }*/

    char sBuf[64];
    sprintf(sBuf, "B US GR SH 0123 PG AL");
    WriteString(buf, 0, 32, sBuf);

    render(buf, &frame_area);

    int z=0;
    while (1) {
        //capture_bus_transactions();
        process_bus();
        // Update the USB CLI
        //serial_loop();
        
/*        gpio_put(LED_PIN, 0);
        sleep_ms(250);
        gpio_put(LED_PIN, 1);
//        printf("Hello HP41!\n");
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
        */
        sprintf(sBuf, "S:%d E:%d", sync_count, embed_seen);
        //printf("%s\n", sBuf);
        WriteString(buf, 5, 48, sBuf);
        sprintf(sBuf, "I:%d O:%d", data_in, data_out);
        WriteString(buf, 5, 56, sBuf);
        render(buf, &frame_area);
        z++;
        if( z > 127 )
            z = 0;
    }
}

void UpdateLCD(char *txt, bool *bp)
{
    if( txt )
        Write41String(buf, 3, 16, txt, bp);
    else {
        Write41String(buf, 3, 16, NULL, NULL);
        UpdateAnnun(0);
    }

}

typedef struct {
    uint8_t len;
    char ann[4];
    bool sp;
} Annu_t;
Annu_t annu[NR_ANNUN] = {
    { 1, "B",  true },
    { 2, "US", true },
    { 1, "G",  false },
    { 1, "R",  true },
    { 2, "SH", true },
    { 1, "0",  false },
    { 1, "1",  false },
    { 1, "2",  false },
    { 1, "3",  false },
    { 1, "4",  true },
    { 1, "P",  true },
    { 2, "AL", false }
};

void UpdateAnnun(uint16_t ann)
{
    char sBuf[24];
    memset(sBuf, ' ', 24);
    int pa = 0;
    for (int a = 0; a < NR_ANNUN; a++)
    {
        if (ann & 1 << ((NR_ANNUN - 1) - a))
        {
            for (int i = 0; i < annu[a].len; i++)
                sBuf[pa + i] = annu[a].ann[i];
        }
        // Add space is needed ...
        pa += annu[a].len + (annu[a].sp ? 1 : 0);
    }
    sBuf[pa] = 0;
	printf("\nAnnunciators: [%s]", sBuf);
    WriteString(buf, 0, 32, sBuf);
}
