#include <stdio.h>

typedef unsigned short uint16_t;


static uint16_t segment41[] = {
0x8000, 0x8000, 0x8000, 0x8000, 0x8000, 0x8000, 0x8000,     // 0
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x7e00,     // 1
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x01F0,     // 2
0x0008, 0x0008, 0x0008, 0x0008, 0x0008, 0x0008, 0x0008,     // 3
0x01f0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,     // 4
0x7e00, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,     // 5
0x0000, 0x0200, 0x0200, 0x0200, 0x0000, 0x0000, 0x0000,     // 6
0x0000, 0x0000, 0x0000, 0x0200, 0x0200, 0x0200, 0x0000,     // 7
0x0000, 0x0000, 0x0000, 0x7c00, 0x0000, 0x0000, 0x0000,     // 8
0x0000, 0x0000, 0x0000, 0x01F0, 0x0000, 0x0000, 0x0000,     // 9
0x0000, 0x0000, 0x0000, 0x0400, 0x1800, 0x6000, 0x0000,     // 10
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,     // 11
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,     // 12
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,     // 13
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000      // 14
};


#define s0  (1 << 0)
#define s1  (1 << 1)
#define s2  (1 << 2)
#define s3  (1 << 3)
#define s4  (1 << 4)
#define s5  (1 << 5)
#define s6  (1 << 6)
#define s7  (1 << 7)
#define s8  (1 << 8)
#define s9  (1 << 9)
#define s10 (1 << 10)
#define s11 (1 << 11)
#define s12 (1 << 12)
#define s13 (1 << 13)
#define star (0x3FFF)

uint16_t seg41[] = {
    // 0x00-0x0F
    s0,
    s0|s8|s6|s7|s13|s12,
    star,
    star,
    s0|s8|s13|s12,
    s0|s8|s6|s13|s12,
    s0|s8|s13,
    star,
    star,
    star,
    star,
    star,
    s1|s8|s7|s13,
    s10|s13|s9|s3,
    star,
    star,
    // 0x10-0x1F
    star,
    star,
    star,
    star,
    star,
    star,
    star,
    star,
    star,
    star,
    star,
    star,
    star,
    s10|s13|s6|s7|s3,
    star,
    star,
    // 0x20-0x2F
    0,
    s8|s9,
    s5|s8,
    s8|s9|s1|s2|s6|s7|s3,
    s0|s5|s6|s7|s2|s3|s8|s9,
    s10|s13|s5|s6|s11|s7|s2|s12|s8, // ???
    s0|s10|s11|s13|s12|s3|s2,
    s8,
    s10|s12,
    s11|s13,
    s10|s11|s12|s13|s6|s7|s8|s9,
    s6|s7|s8|s9,
    s13,
    s6|s7,
    0, // ???????????
    s10|s13,
    // 0x30-0x3F
    s0|s1|s2|s3|s4|s5|s10|s13,    // 0
    s1|s2,    // 1
    s0|s1|s3|s4|s6|s7,    // 2
    s0|s1|s2|s3|s6|s7,    // 3
    s1|s2|s5|s6|s7,    // 4
    s0|s2|s3|s7|s11,    // 5
    s0|s2|s3|s4|s5|s6|s7,    // 6
    s0|s1|s2,    // 7
    s0|s1|s2|s3|s4|s5|s6|s7,    // 8
    s0|s1|s2|s3|s5|s6|s7,    // 9
    s0|s1|s2|s3|s4|s5|s6|s7,    // 8
    s0|s1|s2|s3|s4|s5|s6|s7,    // 8
    s0|s1|s2|s3|s4|s5|s6|s7,    // 8
    s0|s1|s2|s3|s4|s5|s6|s7,    // 8
    // 0x40-0x4F
    // 0x50-0x5F
    // 0x60-0x6F
    // 0x70-0x7F
    // 0x80-0x8F
    // 0x90-0x9F
    // 0xA0-0xAF
    // 0xB0-0xBF
    // 0xC0-0xCF
    // 0xD0-0xDF
    // 0xE0-0xEF
    // 0xF0-0xFF
    s0|s1|s2|s3|s4|s5,

}

uint16_t char41[10][7];

