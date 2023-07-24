#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/multicore.h"
#include "tiny41.h"

#define DEBUG_ANALYZER

extern void core1_main_3(void);
extern void process_bus(void);
extern void capture_bus_transactions(void);

extern volatile int sync_count;
extern volatile int embed_seen;

extern volatile int data_in;
extern volatile int data_out;

#define INIT_PIN(n,io,def)      \
    do {                        \
        gpio_init(n);           \
        gpio_set_dir(n, io);    \
        if( io == GPIO_OUT )    \
            gpio_put(n, def);   \
    } while(0)

void bus_init(void)
{
    // Init the input pins ...
    INIT_PIN(P_CLK1,    GPIO_IN,  0);
    INIT_PIN(P_CLK2,    GPIO_IN,  0);
    INIT_PIN(P_SYNC,    GPIO_IN,  0);
    INIT_PIN(P_ISA,     GPIO_IN,  0);
    INIT_PIN(P_DATA,    GPIO_IN,  0);
    // Init the ISA driver ...
    INIT_PIN(P_ISA_OE,  GPIO_OUT, 1);
    INIT_PIN(P_ISA_DRV, GPIO_OUT, 0);
    // Init leds ...
    INIT_PIN(LED_PIN_R, GPIO_OUT, LED_OFF);
    INIT_PIN(LED_PIN_B, GPIO_OUT, LED_OFF);
}

uint8_t buf[SSD1306_BUF_LEN];

// Initialize render area for entire frame (SSD1306_WIDTH pixels by SSD1306_NUM_PAGES pages)
struct render_area frame_area = {
    start_col : 0,
    end_col : SSD1306_WIDTH - 1,
    start_page : 0,
    end_page : SSD1306_NUM_PAGES - 1
};

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

    multicore_launch_core1(core1_main_3);

    calc_render_area_buflen(&frame_area);

    // zero the entire display
    memset(buf, 0, SSD1306_BUF_LEN);
    render(buf, &frame_area);

    char *text[] = {
        (char*)" \x2c Tiny41 \x2e",
        (char*)"\x5e 3.1416"
    };
    bool bp[12];
    memset(bp, 0, 12);

    Write41String(buf, 5, 0, text[0], bp);
    bp[1] = true;
    Write41String(buf, 5, 16, text[1], bp);

    // Turn on all annunciators ...
    UpdateAnnun(0xFFF);

    render(buf, &frame_area);

#ifdef DEBUG_ANALYZER
    char sBuf[32];
    int oSync, oEmbed, oDin, oDout;
    int bRend;

    oSync = oEmbed = oDin = oDout = -1;
    bRend = 0;
#endif

    while (1) {
        //capture_bus_transactions();
        process_bus();
        // Update the USB CLI
        //serial_loop();

#ifdef DEBUG_ANALYZER
        if( oSync != sync_count || oEmbed != embed_seen ) {
            sprintf(sBuf, "S:%d E:%d", sync_count, embed_seen);
            WriteString(buf, 5, 48, sBuf);
            oSync = sync_count;
            oEmbed = embed_seen;
            bRend++;
        }
        if( oDin != data_in || oDout != data_out ) {
            sprintf(sBuf, "I:%d O:%d", data_in, data_out);
            WriteString(buf, 5, 56, sBuf);
            oDin = data_in;
            oDout = data_out;
            bRend++;
        }
        if( bRend ) {
            render(buf, &frame_area);
            bRend = 0;
        }
#endif//DEBUG_ANALYZER
    }
}

void UpdateLCD(char *txt, bool *bp)
{
    if( txt ) {
        Write41String(buf, 3, 16, txt, bp);
//        render(buf, &frame_area);
    } else {
        // Turn of the display
        Write41String(buf, 3, 16, NULL, NULL);
        UpdateAnnun(0);
    }
}

typedef struct {
    uint8_t len;    // Length of the text
    char ann[4];
    bool sp;        // True if followed by a space
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
//    render(buf, &frame_area);
}
