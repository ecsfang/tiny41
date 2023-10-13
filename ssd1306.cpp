#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "ssd1306.h"
#include "ssd1306_font.h"

// HP41 font bitmaps, char(0) - char(255).
// Each character is 13 pixels high and 7 pixels wide
// Theses are defined vertically to make them quick to copy to FB
#include "font41.h"

void calc_render_area_buflen(struct render_area *area) {
    // calculate how long the flattened buffer will be for a render area
    area->buflen = (area->end_col - area->start_col + 1) * (area->end_page - area->start_page + 1);
}

//#define i2c_default
//#ifdef i2c_default

void SSD1306_send_cmd(uint8_t cmd) {
    // I2C write process expects a control byte followed by data
    // this "data" can be a command or data to follow up a command
    // Co = 1, D/C = 0 => the driver expects a command
    uint8_t buf[2] = {0x80, cmd};
    i2c_write_blocking(i2c_default, SSD1306_I2C_ADDR, buf, 2, false);
}

void SSD1306_send_cmd_list(uint8_t *buf, int num) {
    for (int i=0;i<num;i++)
        SSD1306_send_cmd(buf[i]);
}

void SSD1306_send_X_cmd_list(uint8_t *buf, int num) {
    // I2C write process expects a control byte followed by data
    i2c_write_blocking(i2c_default, SSD1306_I2C_ADDR, buf, num, false);
}

void SSD1306_send_buf(uint8_t buf[], int buflen) {
    // in horizontal addressing mode, the column address pointer auto-increments
    // and then wraps around to the next page, so we can send the entire frame
    // buffer in one gooooooo!

    // NOTE - REWROTE - SO BUFFER MUST HAVE PLACE FOR CONTROL BYTE 
    // copy our frame buffer into a new buffer because we need to add
    // the control byte to the beginning

    uint8_t *temp_buf = &buf[-1];

    uint8_t old = temp_buf[0];
    temp_buf[0] = 0x40;

    i2c_write_blocking(i2c_default, SSD1306_I2C_ADDR, temp_buf, buflen + 1, false);
    temp_buf[0] = old;
}

void SSD1306_init() {
    // Some of these commands are not strictly necessary as the reset
    // process defaults to some of these but they are shown here
    // to demonstrate what the initialization sequence looks like
    // Some configuration values are recommended by the board manufacturer

    uint8_t cmds[] = {
        SSD1306_SET_DISP,               // set display off
        /* memory mapping */
        SSD1306_SET_MEM_MODE,           // set memory address mode 0 = horizontal, 1 = vertical, 2 = page
        0x00,                           // horizontal addressing mode
        /* resolution and layout */
        SSD1306_SET_DISP_START_LINE,    // set display start line to 0
        SSD1306_SET_SEG_REMAP | 0x01,   // set segment re-map, column address 127 is mapped to SEG0
        SSD1306_SET_MUX_RATIO,          // set multiplex ratio
        SSD1306_HEIGHT - 1,             // Display height - 1
        SSD1306_SET_COM_OUT_DIR | 0x08, // set COM (common) output scan direction. Scan from bottom up, COM[N-1] to COM0
        SSD1306_SET_DISP_OFFSET,        // set display offset
        0x00,                           // no offset
        SSD1306_SET_COM_PIN_CFG,        // set COM (common) pins hardware configuration. Board specific magic number. 
                                        // 0x02 Works for 128x32, 0x12 Possibly works for 128x64. Other options 0x22, 0x32
#if ((SSD1306_WIDTH == 128) && (SSD1306_HEIGHT == 32))
        0x02,                           
#elif ((SSD1306_WIDTH == 128) && (SSD1306_HEIGHT == 64))
        0x12,
#else
        0x02,
#endif
        /* timing and driving scheme */
        SSD1306_SET_DISP_CLK_DIV,       // set display clock divide ratio
        0x80,                           // div ratio of 1, standard freq
        SSD1306_SET_PRECHARGE,          // set pre-charge period
        0xF1,                           // Vcc internally generated on our board
        SSD1306_SET_VCOM_DESEL,         // set VCOMH deselect level
        0x30,                           // 0.83xVcc
        /* display */
        SSD1306_SET_CONTRAST,           // set contrast control
        0xFF,
        SSD1306_SET_ENTIRE_ON,          // set entire display on to follow RAM content
        SSD1306_SET_NORM_DISP,           // set normal (not inverted) display
        SSD1306_SET_CHARGE_PUMP,        // set charge pump
        0x14,                           // Vcc internally generated on our board
        SSD1306_SET_SCROLL | 0x00,      // deactivate horizontal scrolling if set. This is necessary as memory writes will corrupt if scrolling was enabled
        SSD1306_SET_DISP | 0x01, // turn display on
    };

    SSD1306_send_cmd_list(cmds, count_of(cmds));
}

void SSD1306_scroll(bool on) {
    // configure horizontal scrolling
    uint8_t cmds[] = {
        SSD1306_SET_HORIZ_SCROLL | 0x00,
        0x00, // dummy byte
        0x00, // start page 0
        0x00, // time interval
        0x03, // end page 3 SSD1306_NUM_PAGES ??
        0x00, // dummy byte
        0xFF, // dummy byte
        (uint8_t)(SSD1306_SET_SCROLL | (on ? 0x01 : 0)) // Start/stop scrolling
    };

    SSD1306_send_cmd_list(cmds, count_of(cmds));
}

void render(uint8_t *buf, struct render_area *area) {
    // update a portion of the display with a render area
    uint8_t cmds[] = {
        0x80, SSD1306_SET_COL_ADDR,
        0x80, area->start_col,
        0x80, area->end_col,
        0x80, SSD1306_SET_PAGE_ADDR,
        0x80, area->start_page,
        0x80, area->end_page
    };
    
    SSD1306_send_X_cmd_list(cmds, count_of(cmds));
    SSD1306_send_buf(buf, area->buflen);
}

static void SetPixel(uint8_t *buf, int x,int y, bool on) {
    assert(x >= 0 && x < SSD1306_WIDTH && y >=0 && y < SSD1306_HEIGHT);

    // The calculation to determine the correct bit to set depends on which address
    // mode we are in. This code assumes horizontal

    // The video ram on the SSD1306 is split up in to 8 rows, one bit per pixel.
    // Each row is 128 long by 8 pixels high, each byte vertically arranged, so byte 0 is x=0, y=0->7,
    // byte 1 is x = 1, y=0->7 etc

    // This code could be optimised, but is like this for clarity. The compiler
    // should do a half decent job optimising it anyway.

    const int BytesPerRow = SSD1306_WIDTH ; // x pixels, 1bpp, but each row is 8 pixel high, so (x / 8) * 8

    int byte_idx = (y / 8) * BytesPerRow + x;
    uint8_t byte = buf[byte_idx];

    if (on)
        byte |=  1 << (y % 8);
    else
        byte &= ~(1 << (y % 8));

    buf[byte_idx] = byte;
}
// Basic Bresenhams.
static void DrawLine(uint8_t *buf, int x0, int y0, int x1, int y1, bool on) {

    int dx =  abs(x1-x0);
    int sx = x0<x1 ? 1 : -1;
    int dy = -abs(y1-y0);
    int sy = y0<y1 ? 1 : -1;
    int err = dx+dy;
    int e2;

    while (true) {
        SetPixel(buf, x0, y0, on);
        if (x0 == x1 && y0 == y1)
            break;
        e2 = 2*err;

        if (e2 >= dy) {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y0 += sy;
        }
    }
}

static inline int GetFontIndex(uint8_t ch) {
    if (ch >= 'A' && ch <='Z') {
        return  ch - 'A' + 1;
    }
    else if (ch >= '0' && ch <='9') {
        return  ch - '0' + 27;
    }
    else return  0; // Not got that char so space.
}

static uint8_t reversed[sizeof(font)] = {0};
static uint16_t reversed41[sizeof(font41)/sizeof(uint16_t)] = {0};

static uint8_t reverse(uint8_t b) {
   b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
   b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
   b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
   return b;
}
static uint16_t reverse41(uint16_t b) {
    uint16_t r = 0;
    for(int i=0; i<16; i++) {
        if( b & (1<<(15-i)))
            r |= 1<<i;
    }
   return r;
}
static void FillReversedCache() {
    // calculate and cache a reversed version of fhe font, because I defined it upside down...doh!
    for (int i=0;i<sizeof(font);i++)
        reversed[i] = reverse(font[i]);
    for (int i=0;i<sizeof(font41)/sizeof(uint16_t);i++)
        reversed41[i] = reverse41(font41[i]);
}

static void WriteChar(uint8_t *buf, int16_t x, int16_t y, uint8_t ch) {
    if (reversed[0] == 0) 
        FillReversedCache();

    if (x > SSD1306_WIDTH - 8 || y > SSD1306_HEIGHT - 8)
        return;

    // For the moment, only write on Y row boundaries (every 8 vertical pixels)
    y = y/8;

    ch = toupper(ch);
    int idx = ch;
    int fb_idx = y * 128 + x;

    for (int i=0;i<5;i++) {
        buf[fb_idx++] = font[idx * 5 + i];
    }
}

// 7 columns of bitmap
// 1 column spare
// 1 column for colon and decimal point
// 1 column for character space
// --> 10 columns for 12 characters
inline void _write41char(uint8_t *buf, int p, uint16_t c) {
    buf[p]               = c & 0xFF;
    buf[p+SSD1306_WIDTH] = (c>>8) & 0xFF;
}

int16_t Write41Char(uint8_t *buf, int16_t x, int16_t y, uint8_t ch, bool bp) {
    if ((x > SSD1306_WIDTH - (bp?3:8)) || y > (SSD1306_HEIGHT>>3 - 1) )
        return 0;

    if( bp && (ch == 0x2c || ch == 0x2e || ch == 0x3A) ) {
        int fb_idx = y * SSD1306_WIDTH + x - 3;
        int cp = (ch == 0x2e ? 3 : (ch == 0x3a ? 6 : 0));
        for (int i=0;i<3;i++)
            _write41char(buf, fb_idx++, punct41[cp++]);
        return 0;
    } else {
        int fb_idx = y * SSD1306_WIDTH + x;
        int cp = ch*FONT41_WIDTH;
        int i=0;
        // Character 0x2C, 0x2E and 0x3A should be treated differently ...
        for (;i<FONT41_WIDTH;i++) {
            _write41char(buf, fb_idx++, font41[cp++]);
        }
        // Clear last columns (should be used for punctation (.,:;))
        for (;i<CHAR41_WIDTH;i++) {
            _write41char(buf, fb_idx++, 0);
        }
        return CHAR41_WIDTH;
    }
}

void WriteString(uint8_t *buf, int16_t x, int16_t y, char *str) {
    // Cull out any string off the screen
    if (x > SSD1306_WIDTH - 8 || y > SSD1306_HEIGHT - 8)
        return;

    while (*str) {
        WriteChar(buf, x, y, *str++);
        x+=6;
    }
}

extern void Render(int r);

void Write41String(uint8_t *buf, int16_t x, int16_t y, char *str, bool *pbp) {
    // Call out any string off the screen
    if (x > SSD1306_WIDTH - CHAR41_WIDTH || y > SSD1306_HEIGHT - 2*8)
        return;

    // For the moment, only write on Y row boundaries (every 8 vertical pixels)
    y >>= 3;

    if( str ) {
        while (*str) {
            x += Write41Char(buf, x, y, *str++, *pbp++);
        }
    } else {
        for(int c=0; c<12; c++) {
            x += Write41Char(buf, x, y, ' ', false);
        }
    }
    Render(0b001);
}


//#endif
