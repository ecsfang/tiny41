//#undef PICO_DEFAULT_I2C_SDA_PIN
//#undef PICO_DEFAULT_I2C_SCL_PIN
//#undef PICO_DEFAULT_I2C

//#define PICO_DEFAULT_I2C_SDA_PIN 26
//#define PICO_DEFAULT_I2C_SCL_PIN 27
//#define PICO_DEFAULT_I2C 1

#define CF_DUMP_DBG_DREGS 0
#define CF_DISPLAY_LCD 1
#define CF_DISPLAY_OLED 0
#define CF_DBG_DISP_ON 0
#define CF_DBG_SLCT 0
#define CF_DBG_DISP_INST 0
#define CF_USE_DISP_ON 0
#define CF_DBG_KEY 1

#include "ssd1306.h"

#define USE_40190
#define MEASURE_TIME
#define MEASURE_COUNT

//#define DISABLE_DISPRINT

#ifdef USE_40190
#define ENABLE_OE   1
#define DISABLE_OE  0
#else
#define ENABLE_OE   0
#define DISABLE_OE  1
#endif


#ifdef PIMORONI_TINY2040_8MB
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
#endif
#ifdef PIMORONI_PICOLIPO_16MB
#define P_DATA      16
#define P_DATA_DRV  20
#define P_DATA_OE   21

#define P_ISA       14
#define P_ISA_DRV   18
#define P_ISA_OE    19

#define P_FI_OE     22

#define P_PWO       17
#define P_SYNC      15
#define P_CLK2      13
#define P_CLK1      12

#define P_IR_LED    26
#endif


#ifdef PIMORONI_PICOLIPO_16MB
// On Pico Lipo, the LED is tied to GND
#define LED_ON      1
#define LED_OFF     0

#define LED_PIN_B PICO_DEFAULT_LED_PIN
// User switch is tied to GND (active low)
#define USER_SW     23
#endif

#ifdef PIMORONI_TINY2040_8MB
// On Pico Tiny, the RGB LED is tied to 3V3
#define LED_ON    0
#define LED_OFF   1

#define LED_PIN_R TINY2040_LED_R_PIN
#define LED_PIN_G TINY2040_LED_G_PIN
#define LED_PIN_B TINY2040_LED_B_PIN
// User switch is tied to GND (active low)
#define USER_SW     23
#endif

#define NR_CHARS  12
#define NR_ANNUN  12

#define BUS_CYCLES 56
#define LAST_CYCLE (BUS_CYCLES-1)

extern void UpdateLCD(char *, bool *, bool);
extern void UpdateAnnun(uint16_t ann, bool nl);

enum {
    TRACE_NONE =    0b000, // No tracing
    TRACE_ON =      0b001, // Tracing enabled
    TRACE_BRK =     0b010, // Trace on (from breakpoint)
    TRACE_ACTIVE =  0b011, // Trace active (enabled and on)
    TRACE_DISASM =  0b100, // Disassembler active
    TRACE_ALL =     0b111  // Full trace
};

#define IS_TRACE()      ((bTrace & TRACE_ACTIVE) == TRACE_ACTIVE)
#define IS_FULLTRACE()  ((bTrace & TRACE_ALL) == TRACE_ALL)
#define IS_DISASM()     ((bTrace & TRACE_DISASM) == TRACE_DISASM)

//#define TRACE
#define USE_FLASH
#define USE_PIO     // Enable IR usage

enum {
    REND_NONE =     0b000,
    REND_LCD =      0b001,
    REND_ANNUN =    0b010,
    REND_DISP =     0b011,
    REND_STATUS =   0b100,
    REND_ALL =      0b111
};

#define TITLE_START     0   // Top of the display
#define LCD_START       2   // Just the number display
#define ANNUN_START     4   // Just the annunciators
#define DISP_START      2   // Number and annunciator display combined
#define STATUS_START    5   // Status area

#define TITLE_ROW   (TITLE_START*8)  // Starting row for each displaysegment
#define LCD_ROW     (LCD_START*8)    // Starting row for each displaysegment
#define ANNUN_ROW   (ANNUN_START*8)  // Starting row for each displaysegment
#define DISP_ROW    (DISP_START*8)   // Starting row for each displaysegment
#define STATUS_ROW  (STATUS_START*8) // Starting row for each displaysegment

class CRendArea {
    struct render_area m_area;
    uint8_t *m_buf;
public:
    CRendArea(uint8_t *b, int s, int e) {
        m_buf = b + s*SSD1306_WIDTH;
        m_area.start_col = 0;
        m_area.end_col = SSD1306_WIDTH - 1;
        m_area.start_page = s;
        m_area.end_page = e;
        calc_render_area_buflen(&m_area);
    }
    void render(void) {
        ::render(m_buf, &m_area);
    }
};

class CDisplay {
    CRendArea   disLcd;
    CRendArea   disAnnun;
    CRendArea   disDisp;
    CRendArea   disStatus;
    CRendArea   disFull;
    int         m_rend;
public:
    static uint8_t m_dispBuf[SSD1306_BUF_LEN+1];
    static uint8_t *m_buf;
    CDisplay() :
        // Initialize render area for parts of frame (SSD1306_WIDTH pixels by SSD1306_NUM_PAGES pages)
        //m_buf = &CDisplay::m_dispBuf[1];
        disLcd(m_buf, LCD_START, LCD_START+1),
        disAnnun(m_buf, ANNUN_START, ANNUN_START+1),
        disDisp(m_buf, DISP_START, ANNUN_START+1),
        disStatus(m_buf, STATUS_START, SSD1306_NUM_PAGES-1),
        disFull(m_buf, 0, SSD1306_NUM_PAGES-1) {
        m_rend = 0;
    }
    void render(void) {
        switch(m_rend) {
        case REND_NONE:                         return;
        case REND_LCD:      disLcd.render();    break;
        case REND_ANNUN:    disAnnun.render();  break;
        case REND_DISP:     disDisp.render();   break;
        case REND_STATUS:   disStatus.render(); break;
        default:            disFull.render();
        }
        m_rend = REND_NONE;
    }
    void rend(int r) {
        m_rend |= r;
    }
    bool needRendering(void) {
        return m_rend != 0;
    }
    uint8_t *buf() { return m_buf; }
};

