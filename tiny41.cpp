#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/multicore.h"
#include "tiny41.h"
#include "core_bus.h"
#include "serial.h"
#include "disasm.h"

////////////////////////////////////////////////////////////////////////////////
//
// Overclock as needed
//
////////////////////////////////////////////////////////////////////////////////

// #define OVERCLOCK 135000
// #define OVERCLOCK 200000
#define OVERCLOCK 270000
// #define OVERCLOCK 360000

// #define DEBUG_ANALYZER

int bRend = REND_NONE;

// Init trace - no trace/no disassembler
volatile uint8_t bTrace = 0;

void bus_init(void)
{
    // Init the input pins ...
    INIT_PIN(P_CLK1, GPIO_IN, 0);
    INIT_PIN(P_CLK2, GPIO_IN, 0);
    INIT_PIN(P_SYNC, GPIO_IN, 0);
    INIT_PIN(P_ISA, GPIO_IN, 0);
    INIT_PIN(P_DATA, GPIO_IN, 0);
//    INIT_PIN(P_POW, GPIO_IN, 0);
    // Init the ISA driver ...
    INIT_PIN(P_ISA_OE, GPIO_OUT, DISABLE_OE);
    INIT_PIN(P_ISA_DRV, GPIO_OUT, 0);
    // Init the ISA driver ...
    INIT_PIN(P_DATA_OE, GPIO_OUT, DISABLE_OE);
    INIT_PIN(P_DATA_DRV, GPIO_OUT, 0);
    // Init the FI driver ...
    INIT_PIN(P_FI_OE, GPIO_OUT, DISABLE_OE);
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

#define DISP_START (2 * SSD1306_WIDTH)
struct render_area disp_area = {
    start_col : 0,
    end_col : SSD1306_WIDTH - 1,
    start_page : 2,
    end_page : 5
};
#define LCD_START (2 * SSD1306_WIDTH)
struct render_area lcd_area = {
    start_col : 0,
    end_col : SSD1306_WIDTH - 1,
    start_page : 2,
    end_page : 3
};
#define ANNUN_START (4 * SSD1306_WIDTH)
struct render_area annun_area = {
    start_col : 0,
    end_col : SSD1306_WIDTH - 1,
    start_page : 4,
    end_page : 5
};

void updateFullDisplay(void)
{
    render(buf, &frame_area);
}
void updateRow(void)
{
    render(buf+LCD_START, &lcd_area);
}
void updateAnnun(void)
{
    render(buf+ANNUN_START, &annun_area);
}
void updateDisp(void)
{
    render(buf+DISP_START, &disp_area);
}
void updateDisplay(void)
{
    switch(bRend) {
    case REND_NONE:
        return;
    case REND_LCD:
        updateRow();
        break;
    case REND_ANNUN:
        updateAnnun();
        break;
    case REND_LCD | REND_ANNUN:
        updateDisp();
        break;
    default:
        updateFullDisplay();
    }
    bRend = REND_NONE;
}

extern void initRoms(void);

uint16_t prtInst[] = {
    0x398, 0x24c, 0x360, 0x05b, 0x398, 0x24c, 0x360, 0x264,
    0x003, 0x027, 0x264, 0x043, 0x013, 0x248, 0x130, 0x020,
    0x266, 0x01b, 0x248, 0x38b, 0x264, 0x043, 0x3d3, 0x264,
    0x03a, 0x005, 0x24c, 0x07b, 0x264, 0x003, 0x387, 0x244,
    0x373, 0x221, 0x1b4, 0x10b, 0x171, 0x1b4, 0x00c, 0x013,
    0x248, 0x3d8, 0x37c, 0x3d8, 0x3e0, 0x264, 0x083, 0x3a0,
    0x3b8, 0x2cc, 0x027, 0x358, 0x04c, 0x02b, 0x11c, 0x1e2,
    0x1e2, 0x3a0, 0x244, 0x1b0, 0x23a, 0x1e0, 0x221, 0x1b4,
    0x023, 0x171, 0x1b4, 0x173, 0x020, 0x3e0, 0x221, 0x1b4,
    0x3e3, 0x264, 0x083, 0x3b7, 0x2c4, 0x3bb, 0x104, 0x221,
    0x1b4, 0x39b, 0x1f1, 0x1b4, 0x20c, 0x31d, 0x1a5, 0x103,
    0x108, 0x3b3, 0x108, 0x264, 0x083, 0x3a0, 0x244, 0x171,
    0x00
};

void dispOverflow(bool bOvf)
{
    gpio_put(LED_PIN_R, bOvf ? LED_ON : LED_OFF);
    if( bOvf )
        WriteString(buf, 5, 56, (char *)"Buffer overflow!");
    else
        WriteString(buf, 5, 56, (char *)"                ");
    bRend = REND_ALL;
}

int main()
{
#if OVERCLOCK > 270000
    /* Above this speed needs increased voltage */
    vreg_set_voltage(VREG_VOLTAGE_1_20);
    sleep_ms(1000);
#endif

    stdio_init_all();

    sleep_ms(2000);

    printf("\n*************");
    printf("\n*  Tiny 41  *");
    printf("\n*************");
    printf("\n");


    // Must read flash before we goto highspeed ...
    initRoms();
/*
#define DIS_ASM(i) printf("%04X %03X - %s\n", a, i, disAsm(i, a, 0)); a++;
    int a = 0xF000;
    int pi = 0;
    extern uint16_t readRom(int a);
    while(a < ADDR_MASK ) {
        pi = readRom(a);
        DIS_ASM(pi);
    }*/
//    extern void dumpRom(int p);
//    dumpRom(0xF);

    /* Overclock */
    set_sys_clock_khz(OVERCLOCK, 1);

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
    calc_render_area_buflen(&lcd_area);
    calc_render_area_buflen(&annun_area);
    calc_render_area_buflen(&disp_area);

    // zero the entire display
    memset(buf, 0, SSD1306_BUF_LEN);
    render(buf, &frame_area);

    char *text[] = {
        (char *)" \x2c Tiny41 \x2e",
        (char *)"\x5e 3.1415"
    };
    bool bp[12];
    memset(bp, 0, 12);

    Write41String(buf, 5, 0, text[0], bp);
    bp[3] = true;
    Write41String(buf, 5, 16, text[1], bp);

    // Turn on all annunciators ...
    UpdateAnnun(0xFFF);

    render(buf, &frame_area);

/*    uint64_t isa = 0xAA3456789ABCDELL;
    ISA_t *s = (ISA_t*)&isa;
    printf("ISA:%014llX", isa);

    printf("\nAddr: %04X Inst: %03X\n", s->addr, s->inst);
*/
#ifdef DEBUG_ANALYZER
    char sBuf[32];
    int oSync, oEmbed, oDin, oDout;

    oSync = oEmbed = oDin = oDout = -1;
    bRend = 0;
#endif

    // Remove some loops from trace ...
    // Reset keyboard (RST10)
    stopBrk(0x00A0);
    setBrk(0x00A1);

    // Ignore key at DISOFF
    stopBrk(0x089D);
    setBrk(0x089E);

    // Ignore key
    stopBrk(0x009A);
    setBrk(0x009B);

    // Ignore MEMLFT routine
    stopBrk(0x05A1);
    setBrk(0x05B6);

    // FOR DEBOUNCE (DRSY30)
    stopBrk(0x0178);
    setBrk(0x0179);

    // Tone in wand
    stopBrk(0xF3D4);
    setBrk(0xF3D5);

    // Tone in Wand
    stopBrk(0xF3E0);
    setBrk(0xF3E1);

    bool bErr = false;
    while (1)
    {
        // capture_bus_transactions();
        process_bus();
        // Update the USB CLI
        serial_loop();

        extern int queue_overflow;
        if( queue_overflow ) {
            if( !bErr ) {
                dispOverflow(true);
                bTrace &= 0b011; // Turn disasm off ...
                bErr = true;
            }
        } else {
            if( bErr ) {
                dispOverflow(false);
                bErr = false;
            }
        }

#ifdef DEBUG_ANALYZER
        WriteString(buf, 5, 40, (char *)(CHK_GPIO(P_POW)?"    ":"IDLE"));
        if (oSync != sync_count || oEmbed != embed_seen)
        {
            sprintf(sBuf, "S:%d E:%d", sync_count, embed_seen);
            WriteString(buf, 5, 48, sBuf);
            oSync = sync_count;
            oEmbed = embed_seen;
        }
        if (oDin != data_in || oDout != data_out)
        {
            sprintf(sBuf, "I:%d O:%d", data_in, data_out);
            WriteString(buf, 5, 56, sBuf);
            oDin = data_in;
            oDout = data_out;
        }
        bRend = REND_ALL;
#endif // DEBUG_ANALYZER
        if( bRend )
            updateDisplay();
    }
}

static int cAnn = 0;

#define ANN_OFF 0x5117

void UpdateLCD(char *txt, bool *bp, bool on)
{
    if (on) {
        Write41String(buf, 3, 16, txt, bp);
        UpdateAnnun(cAnn);
    } else {
        // Turn off the display
        Write41String(buf, 3, 16, NULL, NULL);
        UpdateAnnun(ANN_OFF);
    }
    bRend |= REND_LCD;
    //printf(" - R:%03X", bRend);
}

typedef struct
{
    uint8_t len; // Length of the text
    char ann[4];
    bool sp; // True if followed by a space
} Annu_t;

Annu_t annu[NR_ANNUN] = {
    {1, "B", true},
    {2, "US", true},
    {1, "G", false},
    {1, "R", true},
    {2, "SH", true},
    {1, "0", false},
    {1, "1", false},
    {1, "2", false},
    {1, "3", false},
    {1, "4", true},
    {1, "P", true},
    {2, "AL", false}};

void UpdateAnnun(uint16_t ann)
{
    char sBuf[24];
    memset(sBuf, ' ', 24);
    int pa = 0;
    uint16_t oAnn = 0xFFFF;

    // Same ... ?
    if( ann == oAnn )
        return;

    // Update annuncioators ...
    if (ann == ANN_OFF)
        ann = 0;
    else
        cAnn = ann;
    for (int a = 0; a < NR_ANNUN; a++) {
        if (ann & 1 << ((NR_ANNUN - 1) - a)) {
            for (int i = 0; i < annu[a].len; i++)
                sBuf[pa + i] = annu[a].ann[i];
        }
        // Add space if needed ...
        pa += annu[a].len + (annu[a].sp ? 1 : 0);
    }
    sBuf[pa] = 0;

    if( IS_TRACE() )
        printf("\nAnnunciators: [%s] (%d)", sBuf, cAnn);

    WriteString(buf, 0, 32, sBuf);
    bRend |= REND_ANNUN;
    oAnn = ann;
}
