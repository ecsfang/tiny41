#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/multicore.h"
#include "tiny41.h"
#include "core_bus.h"
#include "serial.h"
#include "disasm.h"
#include "ir_led.h"
#include "tusb_config.h"
#include <tusb.h>
#include "usb/cdc_helper.h"

////////////////////////////////////////////////////////////////////////////////
//
// Overclock as needed
//
////////////////////////////////////////////////////////////////////////////////

// #define OVERCLOCK 135000
// #define OVERCLOCK 200000
#define OVERCLOCK 270000        // 270 MHz
// #define OVERCLOCK 360000

#define DEBUG_ANALYZER

int bRend = REND_NONE;

// Init trace - no trace/no disassembler
//volatile uint8_t bTrace = TRACE_NONE;
volatile uint8_t bTrace = TRACE_ON | TRACE_DISASM;
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
    INIT_PIN(P_ISA_OE,  GPIO_OUT, DISABLE_OE);
    INIT_PIN(P_ISA_DRV, GPIO_OUT, 0);
    // Init the ISA driver ...
    INIT_PIN(P_DATA_OE,  GPIO_OUT, DISABLE_OE);
    INIT_PIN(P_DATA_DRV, GPIO_OUT, 0);
    // Init the FI driver ...
    INIT_PIN(P_FI_OE, GPIO_OUT, DISABLE_OE);
#if defined(PIMORONI_PICOLIPO_16MB)
    // Init leds ...
    INIT_PIN(LED_PIN_B, GPIO_OUT, LED_OFF);
    INIT_PIN(P_IR_LED, GPIO_OUT, LED_OFF);
    INIT_PIN(P_PWO, GPIO_IN, 0);
//    gpio_pull_down(P_PWO);
//    gpio_pull_down(P_SYNC);
#elif defined(PIMORONI_TINY2040_8MB)
    // Init leds ...
    INIT_PIN(LED_PIN_R, GPIO_OUT, LED_OFF);
    INIT_PIN(LED_PIN_B, GPIO_OUT, LED_OFF);
#elif defined(RASPBERRYPI_PICO2)
    INIT_PIN(LED_PIN_B, GPIO_OUT, LED_OFF);
    INIT_PIN(P_IR_LED, GPIO_OUT, LED_OFF);
    INIT_PIN(P_PWO, GPIO_IN, 0);
#else
#error("Must define a board!")
#endif
}


uint8_t CDisplay::m_dispBuf[SSD1306_BUF_LEN+1];
uint8_t *CDisplay::m_buf = &CDisplay::m_dispBuf[1]; // Leave first byte for command

CDisplay    disp41;

void Render(int r)
{
    disp41.rend(r);
}

void dispOverflow(bool bOvf)
{
#if defined(PIMORONI_PICOLIPO_16MB)
    gpio_put(LED_PIN_B, bOvf ? LED_ON : LED_OFF);
#elif defined(PIMORONI_TINY2040_8MB)
    gpio_put(LED_PIN_R, bOvf ? LED_ON : LED_OFF);
#elif defined(RASPBERRYPI_PICO2)
    gpio_put(LED_PIN_B, bOvf ? LED_ON : LED_OFF);
#endif
    printLCD("OV_FLOW", 1, STATUS_ROW);
    const char *bErr = bOvf ? "Buffer overflow!" : "                ";
    WriteString(disp41.buf(), 5, STATUS2_ROW, (char *)bErr);
    bRend |= REND_STATUS;
}

// Current Wand state
typedef enum {
    W_IDLE,
    W_DATA,
    W_DONE
} WState_e;
WState_e wState = W_IDLE;

char cc[32];
int wn = 0;

void log(const char *s)
{
    sprintf( cbuff, "%s\n\r", s);
    cdc_send_string_and_flush(ITF_TRACE, cbuff);
}
void logC(const char *s)
{
    sprintf( cbuff, "%s\n\r", s);
    cdc_send_string_and_flush(ITF_CONSOLE, cbuff);
}

bool bReady = false;

int main()
{
#if OVERCLOCK > 270000
    /* Above this speed needs increased voltage */
    vreg_set_voltage(VREG_VOLTAGE_1_20);
    sleep_ms(1000);
#endif

    stdio_init_all();

    // initialize the USB CDC devices
    usbd_serial_init();
    tusb_init();

    sleep_ms(500);

    serial_loop();

    while( cdc_read_byte(ITF_CONSOLE) != -1 )
        ;
 
    sleep_ms(100);

    logC("\r\nHello world!\r\n");
    //logC("Hello world - again!\r\n");

//    cdc_send_console((char*)"\n\r*************");
//    cdc_send_console((char*)"\n\r*  Tiny 41  *");
//    cdc_send_console((char*)"\n\r*************");
//    cdc_send_console((char*)"\n\r");


    // Must read flash before we goto highspeed ...
    logC("Init ROMs ...");
    initRoms();

    logC("Init XMemory ...");
#ifdef USE_XFUNC
    initXMem(0);
#endif
//    serial_loop();

    /* Overclock */
    logC("Overclock ...");
    set_sys_clock_khz(OVERCLOCK, 1);

    logC("Init all pins ...");
    bus_init();

#ifdef USE_PIO
    // Init IR-led for printer
    init_ir();
#endif

    // I2C is "open drain", pull ups to keep signal high when no data is being
    // sent
    logC("Init I2C ...");
    i2c_init(i2c_default, SSD1306_I2C_CLK * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    //sleep_ms(1000);
    // run through the complete initialization process
    logC(" - init OLED display");
    SSD1306_init();

//    sleep_ms(5000);

//    logC("Hello world - again!");

    logC("Launch CORE 1 ...");
    multicore_launch_core1(core1_main_3);

    // zero the entire display
    logC("Clear and init display ...");
  
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
#if 1
    // Clear all breakpoints ...
    brk.clrAllBrk();
    // Reset keyboard (RST10)
    IGNORE_LOOP(0x00A0);
    // Ignore key at DISOFF
    IGNORE_LOOP(0x089D);
    // Ignore key at DISOFF
    IGNORE_LOOP(0x03E9);
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
    // Tone
    IGNORE_LOOP(0x170F);
    // Tone in wand
    IGNORE_LOOP(0xF3D4);
    // Tone in Wand
    IGNORE_LOOP(0xF3E0);
    // Ignore MEMLFT routine (calculate free memory)
    IGNORE_FUNC(0x05A1, 0x05B6);

    // Timer (CX)
    IGNORE_LOOP(0x5546);

    // XFunction
    IGNORE_LOOP(0xCC52);

    // Ignore loop in Blinky ...
    IGNORE_LOOP_COND(0x6F42, 0x66CA);
    // Waiting for carry in Blinky
    IGNORE_LOOP(0x6F27);
    // Ignore checksum loop in Blinky
    IGNORE_LOOP(0x657D);    // Bank 0
    IGNORE_LOOP(0x6768);    // Bank 1

    // Ignore loop in Service ROM ...
    IGNORE_LOOP(0x4018);
    IGNORE_LOOP(0x4032);
    IGNORE_LOOP(0x4050);
    IGNORE_LOOP(0x4057);
    IGNORE_LOOP(0x405A);
    IGNORE_LOOP(0x41B1);
    IGNORE_LOOP(0x41D9);
    IGNORE_LOOP(0x424B);
    IGNORE_LOOP(0x43E5);
    IGNORE_LOOP(0x462C);
    IGNORE_LOOP(0x47E1);
    IGNORE_LOOP(0x49ED);
    IGNORE_LOOP(0x49F8);
    IGNORE_LOOP(0x49F8);
    IGNORE_FUNC(0x42D8, 0x42E5);
    IGNORE_FUNC(0x4050, 0x406B);
    IGNORE_FUNC(0x4C55, 0x4C71);
#else
    // Just testing ...
    IGNORE_LOOP(0x5117);
#endif
    bool bErr = false;

//    send_to_printer((char *)"Hello from Tiny2040!\n");

    while( cdc_read_byte(ITF_CONSOLE) != -1 )
        ;

    sprintf( cbuff, "LED @ pin %d\n\r", LED_PIN_B);
    cdc_send_string_and_flush(ITF_CONSOLE, cbuff);

    gpio_put(LED_PIN_B, LED_OFF);

    int pwo = 0;
    int w;
    while (1) {
        // Process the USB interfaces
        tud_task();
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

        w = cdc_read_byte(ITF_WAND);
        if( w != -1 ) {
            switch( wState ) {
            case W_IDLE:    // Got length
                cc[0] = w;
                sprintf(cbuff, "Got len: %d\n\r", cc[0]);
                cdc_send_console(cbuff);
                wState = W_DATA;
                wn = 0;
                break;
            case W_DATA:
                cc[1+wn++] = w;
                if( wn < cc[0] )
                    break;
                wState = W_DONE;
            case W_DONE:
                cdc_send_console((char*)"Got barcode: ");
                {
                    int n = sprintf(cbuff,"[");
                    for( int i=0; i<cc[0]; i++ )
                        n += sprintf(cbuff+n,"%02X.", cc[i+1]);
                    sprintf(cbuff+n-1,"]\n\r");
                    cdc_send_console(cbuff);
                }
                cdc_send_string_and_flush(ITF_WAND, (char*)"OK");
                wState = W_IDLE;
                extern void wand_data(unsigned char *dta);
                wand_data((unsigned char*)cc);
            }
        }

#ifdef DEBUG_ANALYZER
        {
            extern volatile Mode_e cpuMode;
            static Mode_e oldMode = NO_MODE;

            char mm[8];
            if( oldMode != cpuMode ) {
                switch( cpuMode ) {
                case NO_MODE:     strcpy(mm, "---    "); break;
                case RUNNING:     strcpy(mm, "Running"); break;
                case LIGHT_SLEEP: strcpy(mm, "Light  "); break;
                case DEEP_SLEEP:  strcpy(mm, "Deep   "); break;
                }
                WriteString(disp41.buf(), 5, STATUS2_ROW, mm);
                disp41.rend(REND_STATUS);
                oldMode = cpuMode;
            }
        }   
#endif // DEBUG_ANALYZER
        if( !bReady ) {
            cdc_flush(ITF_CONSOLE);
            cdc_flush(ITF_TRACE);
        }
        bReady = true;
        // Need to update the display ... ?
        disp41.render();
    }
}

static int cAnn = 0;

#define ANN_OFF 0x5117

void UpdateLCD(char *txt, bool *bp, bool on)
{
    if( IS_TRACE() ) {
        sprintf( cbuff, "\n\r\n\r[%s] (%s)\n\r", txt, on ? "ON" : "OFF");
        cdc_send_string_and_flush(ITF_TRACE, cbuff);
    }
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

void printLCD(const char *txt, int x, int row)
{
    WriteString(disp41.buf(), x, row, (char*)txt);
    disp41.rend(REND_STATUS);
    disp41.render();
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
            printf("\n\r");
        printf("\n[%s] (%d)\n\r", sBuf, cAnn);
    }

    WriteString(disp41.buf(), 0, ANNUN_ROW, sBuf);
    disp41.rend(REND_ANNUN);
    oAnn = ann;
}
