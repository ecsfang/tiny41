#include <stdio.h>
#include <string.h>

#define BYTES_PER_CHAR  7

typedef unsigned short uint16_t;

// Pixel information about each segment in the character
// Note that generated bitmap is upside down to easier be written to the OLED
/********
   ---0---
  |\  8  /|
  5 10|11 1
  |  \|/  |
  |-6- -7-|
  |  /|\  |
  4 13|12 2
  |/  9  \|
   ---3--- 
  ********/
  
static uint16_t segment41[14][BYTES_PER_CHAR] = {
0x0000, 0x8000, 0x8000, 0x8000, 0x8000, 0x8000, 0x0000,     // 0
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x7e00,     // 1
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x01F0,     // 2
0x0000, 0x0008, 0x0008, 0x0008, 0x0008, 0x0008, 0x0000,     // 3
0x01f0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,     // 4
0x7e00, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,     // 5
0x0000, 0x0200, 0x0200, 0x0200, 0x0000, 0x0000, 0x0000,     // 6
0x0000, 0x0000, 0x0000, 0x0200, 0x0200, 0x0200, 0x0000,     // 7
0x0000, 0x0000, 0x0000, 0x7c00, 0x0000, 0x0000, 0x0000,     // 8
0x0000, 0x0000, 0x0000, 0x01F0, 0x0000, 0x0000, 0x0000,     // 9
0x0000, 0x6000, 0x1800, 0x0400, 0x0000, 0x0000, 0x0000,     // 10
0x0000, 0x0000, 0x0000, 0x0400, 0x1800, 0x6000, 0x0000,     // 11
0x0000, 0x0000, 0x0000, 0x0100, 0x00C0, 0x0030, 0x0000,     // 12
0x0000, 0x0030, 0x00C0, 0x0100, 0x0000, 0x0000, 0x0000      // 13
};

// Bitmap information about the punctation characters
// Already upside down
static uint16_t punct41[3*3] = {
  0x6000, 0x3000, 0x0000, // ,
  0x0000, 0x3000, 0x0000, // .
  0x0000, 0x3018, 0x0000  // :
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
    s0,                         // Upper bar
    s0|s8|s6|s7|s13|s12,        // Full man
    star,
    star,
    s0|s8|s13|s12,              // No arms
    s0|s8|s6|s13|s12,           // One arm
    s0|s8|s13,                  // One leg
    star,
    star,
    star,
    star,
    star,
    s1|s8|s7|s13,               // Mu
    s11|s13|s9|s3,              // Angle
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
    s11|s13|s6|s7|s3,           // Not equal
    star,
    star,
    // 0x20-0x2F
    0,                          // Space
    s8|s9,                      // |
    s5|s8,                      // "
    s8|s9|s1|s2|s6|s7|s3,       // #
    s0|s5|s6|s7|s2|s3|s8|s9,    // $
    s10|s13|s5|s6|s11|s7|s2|s12,// %
    s0|s10|s11|s13|s12|s3|s2,   // &
    s8,                         // '
    s11|s12,                    // (
    s10|s13,                    // )
    s10|s11|s12|s13|s6|s7|s8|s9,// *
    s6|s7|s8|s9,                // +
    s6|s7|s11|s12,              // SPECIAL: .
    s6|s7,                      // -
    s6|s7|s10|s13,              // SPECIAL: ,
    s11|s13,                    // /
    // 0x30-0x3F
    s0|s1|s2|s3|s4|s5|s11|s13,  // 0
    s1|s2,                      // 1
    s0|s1|s3|s4|s6|s7,          // 2
    s0|s1|s2|s3|s6|s7,          // 3
    s1|s2|s5|s6|s7,             // 4
    s0|s2|s3|s7|s10,            // 5
    s0|s2|s3|s4|s5|s6|s7,       // 6
    s0|s1|s2,                   // 7
    s0|s1|s2|s3|s4|s5|s6|s7,    // 8
    s0|s1|s2|s3|s5|s6|s7,       // 9
    star,                       // SPECIAL: :
    s6|s13,                     // ;
    s11|s13|s3,                 // <
    s3|s6|s7,                   // =
    s10|s12|s3,                 // >
    s0|s1|s7|s5|s9|s7,          // ?
    // 0x40-0x4F
    s0|s1|s3|s4|s5|s8|s7,       // @
    s0|s1|s2|s4|s5|s6|s7,       // A
    s0|s1|s2|s3|s8|s9|s7,       // B
    s0|s3|s4|s5,                // C
    s0|s1|s2|s3|s8|s9,          // D
    s0|s3|s4|s5|s6|s7,          // E
    s0|s4|s5|s6|s7,             // F
    s0|s2|s3|s4|s5|s7,          // G
    s1|s2|s4|s5|s6|s7,          // H
    s0|s3|s8|s9,                // I
    s1|s2|s3|s4,                // J
    s4|s5|s6|s11|s12,           // K
    s3|s4|s5,                   // L
    s1|s2|s4|s5|s10|s11,        // M
    s1|s2|s4|s5|s10|s12,        // N
    s0|s1|s2|s3|s4|s5,          // O
    // 0x50-0x5F
    s0|s1|s4|s5|s6|s7,          // P
    s0|s1|s2|s3|s4|s5|s12,      // Q
    s0|s1|s4|s5|s6|s7|s12,      // R
    s0|s2|s3|s5|s6|s7,          // S
    s0|s8|s9,                   // T
    s1|s2|s3|s4|s5,             // U
    s4|s5|s11|s13,              // V
    s1|s2|s4|s5|s12|s13,        // W
    s10|s11|s12|s13,            // X
    s10|s11|s9,                 // Y
    s0|s3|s11|s13,              // Z
    s0|s3|s4|s5,                // [
    s10|s12,                    // \ backslash
    s0|s1|s2|s3,                // ]
    s0|s1|s11|s13,              // Up
    s3,                         // _
    // 0x60-0x6F
    s0|s8,                      // Little T
    s2|s3|s4|s6|s12,            // a
    s2|s3|s4|s5|s6|s7,          // b
    s3|s4|s6|s7,                // c
    s1|s2|s3|s4|s6|s7,          // d
    s3|s4|s6|s13,               // e
    s7|s9|s11,                  // f
    s7|s2|s3|s12,               // g
    s2|s4|s5|s6|s7,             // h
    s9,                         // i
    s2|s3,                      // j
    s4|s5|s6|s7|s12,            // k
    s4|s5,                      // l
    s2|s4|s6|s7|s9,             // m
    s2|s4|s6|s7,                // n
    s2|s3|s4|s6|s7,             // o
    // 0x70-0x7F
    s4|s5|s6|s10,               // p
    s5|s6|s10|s12,              // q
    s4|s6|s7,                   // r
    s3|s7|s12,                  // s
    s6|s7|s8|s12,               // t
    s2|s3|s4,                   // u
    s4|s13,                     // v
    s2|s4|s12|s13,              // w
    s6|s7|s9|s13,               // x
    s2|s3|s12,                  // y
    s3|s6|s13,                  // z
    star,
    star,
    star,
    s0|s3|s10|s13,              // Sigma
    s4|s5|s6|s7,                // Append
    // 0x80-0x8F
    s4|s5|s6|s7,                // Append
    s2|s3|s4|s6|s12,            // a
    s2|s3|s4|s5|s6|s7,          // b
    s3|s4|s6|s7,                // c
    s1|s2|s3|s4|s6|s7,          // d
    s3|s4|s6|s13,               // e
    s0,                         // Head
    s0|s8,                      // Head and neck
    s0|s8|s13,                  // One leg
    s0|s8|s12|s13,              // Two legs
    s0|s6|s8|s12|s13,           // One arm
    s0|s6|s7|s8|s12|s13,        // Full body
    s1|s8|s7|s13,               // Mu
    s11|s13|s6|s7|s3,           // Not equal
    s0|s3|s10|s13,              // Sigma
    s11|s13|s9|s3,              // Angle
    // 0x90-0x9F
    s6|s7|s12|s13,              // Pi?
    s3|s12|s13,                 // delta
    s0|s3|s7|s11|s12|s13,       // ?
    s6|s7|s9,                   // small T
    s5|s6|s7|s9|s12,            // l/x?
    s4|s6|s7|s13,               // small sigma
    s0,                         // Head
    s0|s1|s4|s5,                // "Galge"
    s0|s8|s13,                  // One leg
    s0|s8|s12|s13,              // Two legs
    s0|s6|s8|s12|s13,           // One arm
    s0|s6|s7|s8|s12|s13,        // Full body
    s1|s8|s7|s13,               // Mu
    s11|s13|s6|s7|s3,           // Not equal
    s0|s3|s10|s13,              // Sigma
    s11|s13|s9|s3,              // Angle
    // 0xA0-0xAF
    s0|s8,                      // Little T
    s2|s3|s4|s6|s12,            // a
    s2|s3|s4|s5|s6|s7,          // b
    s3|s4|s6|s7,                // c
    s1|s2|s3|s4|s6|s7,          // d
    s3|s4|s6|s13,               // e
    s7|s9|s11,                  // f
    s7|s2|s3|s12,               // g
    s2|s4|s5|s6|s7,             // h
    s9,                         // i
    s2|s3,                      // j
    s4|s5|s6|s7|s12,            // k
    s4|s5,                      // l
    s2|s4|s6|s7|s9,             // m
    s2|s4|s6|s7,                // n
    s2|s3|s4|s6|s7,             // o
    // 0xB0-0xBF
    s4|s5|s6|s10,               // p
    s5|s6|s10|s12,              // q
    s4|s6|s7,                   // r
    s3|s7|s12,                  // s
    s6|s7|s8|s12,               // t
    s2|s3|s4,                   // u
    s4|s13,                     // v
    s2|s4|s12|s13,              // w
    s6|s7|s9|s13,               // x
    s2|s3|s12,                  // y
    s3|s6|s13,                  // z
    s7|s11|s12,                 // (-
    s3|s11|s12|s13,             // delta?
    s6|s10|s13,                 // -)
    s0|s3|s10|s13,              // Sigma
    s4|s5|s6|s7,                // Append
    // 0xC0-0xCF
    s4|s5|s6|s7,                // Append
    s2|s3|s4|s6|s12,            // a
    s2|s3|s4|s5|s6|s7,          // b
    s3|s4|s6|s7,                // c
    s1|s2|s3|s4|s6|s7,          // d
    s3|s4|s6|s13,               // e
    s0,                         // Head
    s0|s8,                      // Head and neck
    s0|s8|s13,                  // One leg
    s0|s8|s12|s13,              // Two legs
    s0|s6|s8|s12|s13,           // One arm
    s0|s6|s7|s8|s12|s13,        // Full body
    s1|s8|s7|s13,               // Mu
    s11|s13|s6|s7|s3,           // Not equal
    s0|s3|s10|s13,              // Sigma
    s11|s13|s9|s3,              // Angle
    // 0xD0-0xDF
    s6|s7|s12|13,               // Pi?
    s3|s12|s13,                 // delta
    s0|s3|s7|s11|s12|s13,       // ?
    s6|s7|s9,                   // small T
    s5|s6|s7|s9|s12,            // l/x?
    s4|s6|s7|s13,               // small sigma
    s0,                         // Head
    s0|s1|s4|s5,                // "Galge"
    s0|s8|s13,                  // One leg
    s0|s8|s12|s13,              // Two legs
    s0|s6|s8|s12|s13,           // One arm
    s0|s6|s7|s8|s12|s13,        // Full body
    s1|s8|s7|s13,               // Mu
    s11|s13|s6|s7|s3,           // Not equal
    s0|s3|s10|s13,              // Sigma
    s11|s13|s9|s3,              // Angle
    // 0xE0-0xEF
    s0|s8,                      // Little T
    s2|s3|s4|s6|s12,            // a
    s2|s3|s4|s5|s6|s7,          // b
    s3|s4|s6|s7,                // c
    s1|s2|s3|s4|s6|s7,          // d
    s3|s4|s6|s13,               // e
    s7|s9|s11,                  // f
    s7|s2|s3|s12,               // g
    s2|s4|s5|s6|s7,             // h
    s9,                         // i
    s2|s3,                      // j
    s4|s5|s6|s7|s12,            // k
    s4|s5,                      // l
    s2|s4|s6|s7|s9,             // m
    s2|s4|s6|s7,                // n
    s2|s3|s4|s6|s7,             // o
    // 0xF0-0xFF
    s4|s5|s6|s10,               // p
    s5|s6|s10|s12,              // q
    s4|s6|s7,                   // r
    s3|s7|s12,                  // s
    s6|s7|s8|s12,               // t
    s2|s3|s4,                   // u
    s4|s13,                     // v
    s2|s4|s12|s13,              // w
    s6|s7|s9|s13,               // x
    s2|s3|s12,                  // y
    s3|s6|s13,                  // z
    s7|s11|s12,                 // (-
    s3|s11|s12|s13,             // delta?
    s6|s10|s13,                 // -)
    s0|s3|s10|s13,              // Sigma
    s4|s5|s6|s7                 // Append
};
#define NR_CHARS (sizeof(seg41)/sizeof(uint16_t))

uint16_t char41[256][BYTES_PER_CHAR];

//  -----
// |\ | /|
// | \|/ |
// |-- --|
// | /|\ |
// |/ | \|
//  -----
void printChar(int ch)
{
    switch( ch ) {
    case 0x2C: printf("  // SPECIAL [.]\n"); break;
    case 0x2E: printf("  // SPECIAL [,]\n"); break;
    case 0x3A: printf("  // SPECIAL [:]\n"); break;
    }
    {
        char cc[7][7];
        memset(cc,' ', 7*7);
        uint16_t c41 = seg41[ch];

        for(int s=0; s<14;s++) {
            switch(c41&(1<<s)) {
            // Fill each segment ...
            case s0: sprintf(cc[0], " -----"); break;
            case s1: cc[1][6] = cc[2][6] = '|'; break;
            case s2: cc[4][6] = cc[5][6] = '|'; break;
            case s3: sprintf(cc[6], " -----"); break;
            case s4: cc[4][0] = cc[5][0] = '|'; break;
            case s5: cc[1][0] = cc[2][0] = '|'; break;
            case s6: cc[3][1] = cc[3][2] = '-'; break;
            case s7: cc[3][4] = cc[3][5] = '-'; break;
            case s8: cc[1][3] = cc[2][3] = '|'; break;
            case s9: cc[4][3] = cc[5][3] = '|'; break;
            case s10: cc[1][1] = cc[2][2] = '\\'; break;
            case s11: cc[1][5] = cc[2][4] = '/'; break;
            case s12: cc[4][4] = cc[5][5] = '\\'; break;
            case s13: cc[4][2] = cc[5][1] = '/'; break;
            }
        }
        for(int r=0; r<7; r++) {
            printf("  //  %-7.7s\n", cc[r]);
        }
    }
}

uint16_t reverse(uint16_t b) {
    uint16_t r = 0;
    for(int i=0; i<16; i++) {
        if( b & (1<<(15-i)))
            r |= 1<<i;
    }
   return r;
}

//#define TEST
int main(int argv, char *argc[])
{
#ifdef TEST
    int row = 3;
    for(int c=0; c<NR_CHARS; c++) {
        printChar(c);
    }
#else
    printf("#define FONT41_WIDTH %d\n", 7);     // 7 pixels width
    printf("#define CHAR41_WIDTH %d\n", 10);    // 10 pixels including :;
    printf("#define FIRST_CHAR41 %d\n", 0);     // char(0) -->
    printf("#define LAST_CHAR41  %d\n", (int)(NR_CHARS/7));    // char(x-1)
    printf("static uint16_t font41[%d] = {", (int)NR_CHARS*7);
    for(int c=0; c<NR_CHARS; c++) {
        if( (c & 0x0F) == 0 )
            printf("\n  // ***** 0x%02X-0x%02X\n\n", c, c+0xF);
        int segs = seg41[c];
        printChar(c);
        printf("  ");
        for(int i=0; i<BYTES_PER_CHAR;i++) {
            uint16_t b = 0;
            for(int s=0; s<14; s++) {
                if( segs & (1<<s) )
                    b |= segment41[s][i];
            }
            // Turn bimap upside down
            b = reverse(b);
            char41[c][i] = b;
            printf("0x%04X%c ", b, c==(NR_CHARS-1) && i==(BYTES_PER_CHAR-1)?' ':',');
        }
        printf("// 0x%02X (%d)\n", c, c);
    }
    printf("};\n");
#endif
    printf("static uint16_t punct41[%d] = {\n", 3*3);
    for(int c=0; c<3; c++) {
        printf("  ");
        for(int i=0; i<3;i++) {
            printf("0x%04X%c ", punct41[c*3+i], c==2 && i==2?' ':',');
        }
        printf(" // %c\n", ",.:"[c]);
    }
    printf("};\n");
    return 0;
}