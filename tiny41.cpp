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
volatile uint8_t bTrace = TRACE_NONE;

extern CBreakpoint brk;

void bus_init(void)
{
    // Init the input pins ...
    INIT_PIN(P_CLK1, GPIO_IN, 0);
    INIT_PIN(P_CLK2, GPIO_IN, 0);
    INIT_PIN(P_SYNC, GPIO_IN, 0);
    INIT_PIN(P_ISA, GPIO_IN, 0);
    INIT_PIN(P_DATA, GPIO_IN, 0);
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


uint8_t CDisplay::m_dispBuf[SSD1306_BUF_LEN+1];
uint8_t *CDisplay::m_buf = &CDisplay::m_dispBuf[1];

CDisplay    disp41;

void Render(int r)
{
    disp41.rend(r);
}

extern void initRoms(void);

void dispOverflow(bool bOvf)
{
    gpio_put(LED_PIN_R, bOvf ? LED_ON : LED_OFF);
    const char *bErr = bOvf ? "Buffer overflow!" : "                ";
    WriteString(disp41.buf(), 5, (STATUS_START+2)*8, (char *)bErr);
    bRend |= REND_STATUS;
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

    // zero the entire display
    memset(disp41.buf(), 0, SSD1306_BUF_LEN);
    disp41.rend(REND_ALL);
    disp41.render();

    char *text[] = {
        (char *)" \x2c Tiny41 \x2e",
        (char *)"\x5e 3.1415"
    };
    bool bp[12];
    memset(bp, 0, 12);

    Write41String(disp41.buf(), 5, TITLE_ROW, text[0], bp);
    bp[3] = true;
    Write41String(disp41.buf(), 5, LCD_ROW, text[1], bp);

    // Turn on all annunciators ...
    UpdateAnnun(0xFFF, false);

    disp41.rend(REND_ALL);
    disp41.render();

#ifdef DEBUG_ANALYZER
    char sBuf[32];
    int oSync, oEmbed, oDin, oDout;

    oSync = oEmbed = oDin = oDout = -1;
    bRend = 0;
#endif

// Remove some loops from trace ...
//  - add a stop at last address in loop
//  - add a start at first address after loop
#define IGNORE_LOOP(a) do {         \
                        brk.stopBrk(a); \
                        brk.setBrk(a+1);\
                       } while (0)

// Ignore loop with conditional exit from loop
#define IGNORE_LOOP_COND(a,c) do {         \
                        brk.stopBrk(a); \
                        brk.setBrk(a+1);\
                        brk.setBrk(c);\
                       } while (0)

// Ignore function with loop
#define IGNORE_FUNC(a,c) do {         \
                        brk.stopBrk(a); \
                        brk.setBrk(c);\
                       } while (0)

    // Reset keyboard (RST10)
    IGNORE_LOOP(0x00A0);
    // Ignore key at DISOFF
    IGNORE_LOOP(0x089D);
    // Ignore key at ...
    IGNORE_LOOP_COND(0x0BB2, 0x0BA2);
    // Ignore key at ...
    IGNORE_LOOP_COND(0x0E9E, 0x0EA3);
    // Ignore key at ...
    IGNORE_LOOP_COND(0x0ECE, 0x009B);
    // Ignore key in Zenrom ...
    IGNORE_LOOP_COND(0xA20B, 0xA210);
    // Ignore key in Zenrom ...
    IGNORE_LOOP_COND(0xA8F0, 0xA8F8);
    // Ignore key
    IGNORE_LOOP(0x009A);
    // FOR DEBOUNCE (DRSY30)
    IGNORE_LOOP(0x0178);
    // Tone in wand
    IGNORE_LOOP(0xF3D4);
    // Tone in Wand
    IGNORE_LOOP(0xF3E0);
    // Ignore MEMLFT routine (calculate free memory)
    IGNORE_FUNC(0x05A1, 0x05B6);

    // Ignore loop in Blinky ...
    IGNORE_LOOP_COND(0x6F42, 0x66CA);
    // Waiting for carry in Blinky
    IGNORE_LOOP(0x6F27);
    // Ignore checksum loop in Blinky
    IGNORE_LOOP(0x657D);
    IGNORE_LOOP(0x6768);

    bool bErr = false;
    while (1)
    {
        process_bus();
        // Update the USB CLI
        serial_loop();

        extern int queue_overflow;
        if( queue_overflow ) {
            if( !bErr ) {
                dispOverflow(true);
                bTrace &= ~TRACE_DISASM; // Turn disasm off ...
                bErr = true;
            }
        } else {
            if( bErr ) {
                dispOverflow(false);
                bErr = false;
            }
        }

#ifdef DEBUG_ANALYZER
        WriteString(buf, 5, STATUS_ROW, (char *)(CHK_GPIO(P_POW)?"    ":"IDLE"));
        if (oSync != sync_count || oEmbed != embed_seen)
        {
            sprintf(sBuf, "S:%d E:%d", sync_count, embed_seen);
            WriteString(buf, 5, (STATUS_START+1)*8, sBuf);
            oSync = sync_count;
            oEmbed = embed_seen;
        }
        if (oDin != data_in || oDout != data_out)
        {
            sprintf(sBuf, "I:%d O:%d", data_in, data_out);
            WriteString(buf, 5, (STATUS_START+2)*8, sBuf);
            oDin = data_in;
            oDout = data_out;
        }
        bRend = REND_ALL;
#endif // DEBUG_ANALYZER
        // Need to update the display ... ?
        disp41.render();
    }
}

static int cAnn = 0;

#define ANN_OFF 0x5117

void UpdateLCD(char *txt, bool *bp, bool on)
{
    if( IS_TRACE() )
        printf("\n\n[%s] (%s)", txt, on ? "ON" : "OFF");
    if (on) {
        Write41String(disp41.buf(), 3, LCD_ROW, txt, bp);
        UpdateAnnun(cAnn, false);
    } else {
        // Turn off the display
        Write41String(disp41.buf(), 3, LCD_ROW, NULL, NULL);
        UpdateAnnun(ANN_OFF, false);
    }
    disp41.rend(REND_LCD);
}

typedef struct
{
    char ann[4];
} Annu_t;

Annu_t annu[NR_ANNUN] = {
    {"B " },    // Battery
    {"US "},    // USer
    {"G"  },    // G in Grad
    {"R " },    // Rad
    {"SH "},    // SHift
    {"0"  },    // Flag 0-4
    {"1"  },
    {"2"  },
    {"3"  },
    {"4 " },
    {"P " },    // Prgm
    {"AL" }     // ALpha
};

void UpdateAnnun(uint16_t ann, bool nl)
{
    char sBuf[24];
    memset(sBuf, ' ', 24);
    int i, pa = 0;
    static uint16_t oAnn = 0xFFFF;

    // Update annunciators ...
    if (ann == ANN_OFF)
        ann = 0;
    else
        cAnn = ann;

    // Same ... ?
    if( ann == oAnn )
        return;

    for (int a = 0; a < NR_ANNUN; a++) {
        if (ann & 1 << ((NR_ANNUN - 1) - a)) {
            i = 0;
            while(annu[a].ann[i])
                sBuf[pa++] = annu[a].ann[i++];
        } else
            pa += strlen(annu[a].ann);
    }
    sBuf[pa] = 0;

    if( IS_TRACE() ) {
        if( nl )
            printf("\n");
        printf("\n[%s] (%d)\n", sBuf, cAnn);
    }

    WriteString(disp41.buf(), 0, ANNUN_ROW, sBuf);
    disp41.rend(REND_ANNUN);
    oAnn = ann;
}
