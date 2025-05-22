#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/multicore.h"
#include "pico/flash.h"
#include "tiny41.h"
#include "core_bus.h"
#include "serial.h"
#include "disasm.h"
#include "ir_led.h"
#include "tusb_config.h"
#include <tusb.h>
#include "usb/cdc_helper.h"
#include "wand.h"
#include "modfile.h"
//#include <wiringPi.h>
#include <vector>

#include "hardware/structs/qmi.h"
#include "hardware/structs/xip_ctrl.h"
#include "hardware/clocks.h"

////////////////////////////////////////////////////////////////////////////////
//
// Overclock as needed
//
////////////////////////////////////////////////////////////////////////////////

#define DO_OVERCLOCK
#define DEBUG_ANALYZER

#ifdef DO_OVERCLOCK
// #define OVERCLOCK 135000
// #define OVERCLOCK 180000
// #define OVERCLOCK 200000
#define OVERCLOCK 270000        // 270 MHz
// #define OVERCLOCK 360000
#endif

#ifdef PIMORONI_PICO_PLUS2_RP2350
bool initPSRam(void);
void psramTest(void);
#endif

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
//    gpio_pull_up(P_SYNC);
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
#elif defined(PIMORONI_PICO_PLUS2_RP2350)
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

const char *text[] = {
    " \x2c Tiny41 \x2e",
    "\x5e 3.1415",
    " \x2c Voyager \x2e",
    "    HP-10C"
};

void setMenu(int r1, int r2)
{
    bool bp[12];
    memset(bp, 0, 12);

    Write41String(disp41.buf(), 5, TITLE_ROW, (char*)text[r1], bp);
    bp[3] = true;
    Write41String(disp41.buf(), 5, LCD_ROW, (char*)text[r2], bp);

    // Turn on all annunciators ...
    UpdateAnnun(0xFFF, false);

    disp41.rend(REND_ALL);
    disp41.render();
}

#ifdef PIMORONI_PICO_PLUS2_RP2350
uint8_t* psram_mem = NULL;

void __no_inline_not_in_flash_func(setup_psram)(uint cs_pin) {
    gpio_set_function(cs_pin, GPIO_FUNC_XIP_CS1);

    // Enable direct mode, PSRAM CS, clkdiv of 10.
    qmi_hw->direct_csr = 10 << QMI_DIRECT_CSR_CLKDIV_LSB | \
                        QMI_DIRECT_CSR_EN_BITS | \
                        QMI_DIRECT_CSR_AUTO_CS1N_BITS;
    while (qmi_hw->direct_csr & QMI_DIRECT_CSR_BUSY_BITS)
        ;

    // Enable QPI mode on the PSRAM
    const uint CMD_QPI_EN = 0x35;
    qmi_hw->direct_tx = QMI_DIRECT_TX_NOPUSH_BITS | CMD_QPI_EN;

    while (qmi_hw->direct_csr & QMI_DIRECT_CSR_BUSY_BITS)
        ;

    // Set PSRAM timing for APS6404
    //
    // Using an rxdelay equal to the divisor isn't enough when running the APS6404 close to 133MHz.
    // So: don't allow running at divisor 1 above 100MHz (because delay of 2 would be too late),
    // and add an extra 1 to the rxdelay if the divided clock is > 100MHz (i.e. sys clock > 200MHz).
    const int max_psram_freq = 133000000;
    const int clock_hz = clock_get_hz(clk_sys);
    int divisor = (clock_hz + max_psram_freq - 1) / max_psram_freq;
    if (divisor == 1 && clock_hz > 100000000) {
        divisor = 2;
    }
    int rxdelay = divisor;
    if (clock_hz / divisor > 100000000) {
        rxdelay += 1;
    }

    // - Max select must be <= 8us.  The value is given in multiples of 64 system clocks.
    // - Min deselect must be >= 18ns.  The value is given in system clock cycles - ceil(divisor / 2).
    const int clock_period_fs = 1000000000000000ll / clock_hz;
    const int max_select = (125 * 1000000) / clock_period_fs;  // 125 = 8000ns / 64
    const int min_deselect = (18 * 1000000 + (clock_period_fs - 1)) / clock_period_fs - (divisor + 1) / 2;

    qmi_hw->m[1].timing = 1 << QMI_M1_TIMING_COOLDOWN_LSB |
        QMI_M1_TIMING_PAGEBREAK_VALUE_1024 << QMI_M1_TIMING_PAGEBREAK_LSB |
        max_select << QMI_M1_TIMING_MAX_SELECT_LSB |
        min_deselect << QMI_M1_TIMING_MIN_DESELECT_LSB |
        rxdelay << QMI_M1_TIMING_RXDELAY_LSB |
        divisor << QMI_M1_TIMING_CLKDIV_LSB;

    // Set PSRAM commands and formats
    qmi_hw->m[1].rfmt =
        QMI_M0_RFMT_PREFIX_WIDTH_VALUE_Q << QMI_M0_RFMT_PREFIX_WIDTH_LSB |\
        QMI_M0_RFMT_ADDR_WIDTH_VALUE_Q   << QMI_M0_RFMT_ADDR_WIDTH_LSB |\
        QMI_M0_RFMT_SUFFIX_WIDTH_VALUE_Q << QMI_M0_RFMT_SUFFIX_WIDTH_LSB |\
        QMI_M0_RFMT_DUMMY_WIDTH_VALUE_Q  << QMI_M0_RFMT_DUMMY_WIDTH_LSB |\
        QMI_M0_RFMT_DATA_WIDTH_VALUE_Q   << QMI_M0_RFMT_DATA_WIDTH_LSB |\
        QMI_M0_RFMT_PREFIX_LEN_VALUE_8   << QMI_M0_RFMT_PREFIX_LEN_LSB |\
        6                                << QMI_M0_RFMT_DUMMY_LEN_LSB;

    qmi_hw->m[1].rcmd = 0xEB;

    qmi_hw->m[1].wfmt =
        QMI_M0_WFMT_PREFIX_WIDTH_VALUE_Q << QMI_M0_WFMT_PREFIX_WIDTH_LSB |\
        QMI_M0_WFMT_ADDR_WIDTH_VALUE_Q   << QMI_M0_WFMT_ADDR_WIDTH_LSB |\
        QMI_M0_WFMT_SUFFIX_WIDTH_VALUE_Q << QMI_M0_WFMT_SUFFIX_WIDTH_LSB |\
        QMI_M0_WFMT_DUMMY_WIDTH_VALUE_Q  << QMI_M0_WFMT_DUMMY_WIDTH_LSB |\
        QMI_M0_WFMT_DATA_WIDTH_VALUE_Q   << QMI_M0_WFMT_DATA_WIDTH_LSB |\
        QMI_M0_WFMT_PREFIX_LEN_VALUE_8   << QMI_M0_WFMT_PREFIX_LEN_LSB;

    qmi_hw->m[1].wcmd = 0x38;

    // Disable direct mode
    qmi_hw->direct_csr = 0;

    // Enable writes to PSRAM
    hw_set_bits(&xip_ctrl_hw->ctrl, XIP_CTRL_WRITABLE_M1_BITS);

    psram_mem = (uint8_t*)PSRAM_ADDR;
}
#endif


int main()
{
#ifdef DO_OVERCLOCK
#if OVERCLOCK > 270000
    /* Above this speed needs increased voltage */
    vreg_set_voltage(VREG_VOLTAGE_1_20);
    sleep_ms(1000);
#endif
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
    initRoms(); // Load all current modules ...

    logC("Init XMemory ...");
#ifdef USE_XFUNC
    initXMem(0);
#endif
//    serial_loop();

#ifdef DO_OVERCLOCK
    /* Overclock */
    logC("Overclock ...");
    set_sys_clock_khz(OVERCLOCK, 1);
#endif

    logC("Init all pins ...");
    bus_init();

#ifdef USE_PIO
    // Init IR-led for printer
    init_ir();
#endif

    init_wand();

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

    setMenu(0, 1);

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
    // Ignore key in HP10C ...
    IGNORE_LOOP_COND(0x5729, 0x572C);
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

    for(int i=0; i<10; i++) {
        gpio_put(LED_PIN_B, LED_ON);
        sleep_ms(100);
        gpio_put(LED_PIN_B, LED_OFF);
        sleep_ms(100);
    }

    gpio_put(LED_PIN_B, LED_OFF);

#ifdef PIMORONI_PICO_PLUS2_RP2350
    initPSRam();
#endif
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
                //extern void wand_data(unsigned char *dta);
                pBar->data((unsigned char*)cc);
            }
        }

#ifdef DEBUG_ANALYZER
        {
            extern volatile Mode_e cpuMode;
            extern bool display_on;
            static Mode_e oldMode = NO_MODE;

            char mm[8];
            if( oldMode != cpuMode ) {
                switch( cpuMode ) {
                case NO_MODE:       strcpy(mm, "---    "); break;
                case RUNNING:       strcpy(mm, "Running"); break;
                case LIGHT_SLEEP:   strcpy(mm, "Light  "); break;
                case DEEP_SLEEP:
                    strcpy(mm, "Deep   ");
                    if( display_on ) {
                        // Turn off display when going to deep sleep
                        display_on = false;
                        UpdateLCD(NULL, NULL, display_on);
                    }
                    break;
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
    char ann[6];
} Annu_t;

#ifdef VOYAGER
#define NR_ANNUN  9
Annu_t annu[NR_ANNUN] = {
    {"U " },    // Battery
    {"f "},    // USer
    {"g "  },    // G in Grad
    {"B " },    // Rad
    {"G"},    // SHift
    {"RD "  },    // Flag 0-4
    {"DMY "  },
    {"C "  },
    {"PM"  }
};
#else
#define NR_ANNUN  12
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
#endif

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

#ifdef PIMORONI_PICO_PLUS2_RP2350
bool initPSRam(void)
{
    int n = 0;
    int err = 0;
    if( !psram_mem )
        setup_psram(PIMORONI_PICO_PLUS2_PSRAM_CS_PIN);

    if( psram_mem )
        return true;

    cdc_send_string_and_flush(ITF_CONSOLE, (char*)"Failed to init PSRAM!\n\r");
    return false;
}
void psramTest(void)
{
    int n = 0;
    int err = 0;
    if( !psram_mem )
        setup_psram(PIMORONI_PICO_PLUS2_PSRAM_CS_PIN);

    if( !psram_mem ) {
        cdc_send_string_and_flush(ITF_CONSOLE, (char*)"Failed to init PSRAM!\n\r");
        return;
    }

    uint32_t* pRam = (uint32_t*)psram_mem;

    sprintf( cbuff, "Start PSRAM test @ pin:%d!\n\r", PIMORONI_PICO_PLUS2_PSRAM_CS_PIN);
    cdc_send_string_and_flush(ITF_CONSOLE, cbuff);
    sprintf( cbuff, "Size: %d MB\n\r", PSRAM_SIZE >> 20);
    cdc_send_string_and_flush(ITF_CONSOLE, cbuff);

    for (uint32_t i = 0; i < (PSRAM_SIZE/sizeof(uint32_t)); ++i) {
        pRam[i] = i;
    }
    //n += sprintf( cbuff+n, "%d\n\r", pRam[0]);
    //n += sprintf( cbuff+n, "%d\n\r", pRam[1000]);
    sprintf( cbuff, "Test address %X to %X ... \n\r", 0, PSRAM_SIZE-1);
    cdc_send_string_and_flush(ITF_CONSOLE, cbuff);
    for (int i = 0; i < (PSRAM_SIZE/sizeof(uint32_t)); ++i) {
        uint32_t read_val = pRam[i];
        if (read_val != i ) {
            if( !err) {
                sprintf( cbuff, "Memory mismatch at %d: %08x != %08x\n\r", i, read_val, i);
                cdc_send_string_and_flush(ITF_CONSOLE, cbuff);
            }
            err++;
        }
    }
    if( err ) {
        sprintf( cbuff, "%d memory errors!\n\r", err);
        cdc_send_string_and_flush(ITF_CONSOLE, cbuff);
    } else {
        cdc_send_string_and_flush(ITF_CONSOLE, (char*)"No errors!\n\r");
    }
    cdc_send_string_and_flush(ITF_CONSOLE, (char*)"Done\n\r");
}
#endif

