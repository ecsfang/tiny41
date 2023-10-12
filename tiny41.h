#undef PICO_DEFAULT_I2C_SDA_PIN
#undef PICO_DEFAULT_I2C_SCL_PIN
//#undef PICO_DEFAULT_I2C

#define PICO_DEFAULT_I2C_SDA_PIN 26
#define PICO_DEFAULT_I2C_SCL_PIN 27
//#define PICO_DEFAULT_I2C 1

#include "ssd1306.h"

#define USE_40190
//#define MEASURE_TIME
#define MEASURE_COUNT

#ifdef USE_40190
#define ENABLE_OE   1
#define DISABLE_OE  0
#else
#define ENABLE_OE   0
#define DISABLE_OE  1
#endif


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


//#define P_DTA_DRV 0
//#define P_DTA_OE  28

#define LED_ON    0
#define LED_OFF   1

#define LED_PIN_R TINY2040_LED_R_PIN
#define LED_PIN_G TINY2040_LED_G_PIN
#define LED_PIN_B TINY2040_LED_B_PIN

#define NR_CHARS  12
#define NR_ANNUN  12

#define BUS_CYCLES 56
#define LAST_CYCLE (BUS_CYCLES-1)

extern void UpdateLCD(char *, bool *, bool);
extern void UpdateAnnun(uint16_t ann);

#define IS_TRACE() ((bTrace & 0b011) == 0b011)
#define IS_FULLTRACE() ((bTrace & 0b111) == 0b111)
#define IS_DISASM() (bTrace & 0b100)

//#define TRACE
#define USE_FLASH

enum {
    REND_NONE,
    REND_LCD = 0b001,
    REND_ANNUN = 0b010,
    REND_DISP = 0b011,
    REND_STATUS = 0b100,
    REND_ALL = 0b111
};

#define LCD_START 2     // Just the number display
#define ANNUN_START 4   // Just the annunciators
#define DISP_START 2    // Number and annunciator display combined
#define STATUS_START 5  // Status area

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

extern uint8_t *buf;

class CDisplay {
    CRendArea   disLcd;
    CRendArea   disAnnun;
    CRendArea   disDisp;
    CRendArea   disStatus;
    CRendArea   disFull;
    int         m_rend;
public:
    static uint8_t m_dispBuf[SSD1306_BUF_LEN+1];
    CDisplay() :
        // Initialize render area for parts of frame (SSD1306_WIDTH pixels by SSD1306_NUM_PAGES pages)
        disLcd(buf, LCD_START, LCD_START+1),
        disAnnun(buf, ANNUN_START, ANNUN_START+1),
        disDisp(buf, DISP_START, ANNUN_START+1),
        disStatus(buf, STATUS_START, SSD1306_NUM_PAGES-1),
        disFull(buf, 0, SSD1306_NUM_PAGES-1) {
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
};

