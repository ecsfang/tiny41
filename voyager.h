/*

Display register 9 and 10 in Voyager models
Digits 0-A(10) - 0 is sign only

HP10C
==================================================================================

0        5555|5544|4444|4444|3333|3333|3322|2222|2222|1111|1111|1100|0000|0000
Reg 9:   5432|1098|7654|3210|9876|5432|1098|7654|3210|9876|5432|1098|7654|3210
Digit:   6666|6666|7777|7777|8888|8899|9999|AAAA|AAAA|A 88|9 99|8 7 |6   |    
Segment: ceaf|bghi|ceaf|bghi|ceaf|bgce|afbg|ceaf|bghi|d*hi|d*hi|d*d*|d*  |    
Annun:       |    |    |    |    |    |    |    |    | P  | C  | D R| G  |    

1        5555|5544|4444|4444|3333|3333|3322|2222|2222|1111|1111|1100|0000|0000
Reg 10:  5432|1098|7654|3210|9876|5432|1098|7654|3210|9876|5432|1098|7654|3210
Digit:   3 22|2 11|1011|1111|5555|5544|4444|4433|555 |4 22|2222|3333|33--|----
Segment: d*hi|d*hi|dgce|afbg|ceaf|bghi|ceaf|bghi|hid*|d*ce|afbg|ceaf|bg--|----
Annun:    f  | U  | S  |    |    |    |    |    |   B| g  |    |    |    |     


HP12C/HP15C/HP16C
==================================================================================

0        5555|5544|4444|4444|3333|3333|3322|2222|2222|1111|1111|1100|0000|0000
Reg 9:   5432|1098|7654|3210|9876|5432|1098|7654|3210|9876|5432|1098|7654|3210
Digit:   3333|3333|1055|5555|1144|4444|4422|2222|1111|1122|2233|5544|55  |    
Segment: hibg|afce|dgbg|afce|hihi|bgaf|cebg|afce|bgaf|ced*|hid*|hid*|d*  |    
Annun:       |    | S  |    |    |    |    |    |    |   ?|   ?|   ?| ?  |    

1        5555|5544|4444|4444|3333|3333|3322|2222|2222|1111|1111|1100|0000|0000
Reg 10:  5432|1098|7654|3210|9876|5432|1098|7654|3210|9876|5432|1098|7654|3210
Digit:   AAAA|AA99|9999|AAAA|8888|9999|8888|8877|7777|7766|6666|6666|77  |    
Segment: bgaf|cebg|afce|hid*|d*hi|hid*|bgaf|cehi|bgaf|cehi|d*bg|afce|d*  |    
Annun:       |    |    |   ?| ?  |   ?|    |    |    |    | ?  |    | ?  |    

*/

#define REG09   0
#define REG10   1

// Define the hardcoded port for the Voyager ROM code
#define VOYAGER_PORT    0x5000
#define VROM(x)         ( (*voyager)[x] )
#define VOY_PORT(x)     ( VOYAGER_PORT | (x) )
// Shift data into mantissa
#define MANT(x)         ( (x)<<12 )
#define VOY_CTABLE1     MANT(VOY_PORT(0x3F0))
#define VOY_CTABLE2     MANT(VOY_PORT(0x3E0))

// Defines the offset to the display character table 1
#define VTABLE_1(x)     ( VOY_CTABLE1 | ((x) << 4) )
// Defines the offset to the display character table 2 (HP16C only)
#define VTABLE_2(x)     ( VOY_CTABLE2 | ((x) << 4) )
// Defines the offset to the instruction address offset (HP10C only?)
#define CMD10_TABLE(x)  MANT(VOY_PORT(0x800 + x))
// Defines the offset to the instruction address offset (HP16C only?)
#define CMD16_TABLE(x)  MANT(VOY_PORT(0xC00 + x))

extern void keyMapInit(int mod);
extern const uint32_t voyMain[6];
extern uint8_t keyMap[0x100];

typedef enum {
    V_NONE,
    V_10C,
    V_11C,
    V_12C,
    V_15C,
    V_16C
} Voyager_e;

extern const Voyager_e voyModel[16];

enum {
    SEG_a = 0, 
    SEG_b = 1, 
    SEG_c = 2, 
    SEG_d = 3, 
    SEG_e = 4, 
    SEG_f = 5, 
    SEG_g = 6, 
    SEG_h = 7, 
    SEG_i = 8,
    SEG_j = 9
};

// Annunciators connected to segment 'j'
enum {
    ANN_USER,
    ANN_F,
    ANN_G,
    ANN_BEGIN,
    ANN_GRAD,
    ANN_RAD,
    ANN_DMY,
    ANN_C,
    ANN_PRGM
};

// Struct to point to each segment for a given digit
typedef struct {
    uint8_t reg;     // Index to disp reg (0/1) (single digit resides in one register)
    uint8_t bit[10]; // The index to each segment for this digit in this register
} Digit_t;

// Struct to point to each annunciator for a given digit
// These are spread out between the two register.
typedef struct {
    uint8_t reg; // Index to disp reg (0/1)
    uint8_t bit; // The bit holding the information
} Annun_t;

typedef Digit_t AllDigits_t[11];

// Segment definition for all Voyagers (except HP10C)
const AllDigits_t _hp1Xseg = {
    // Digit 0  a   b   c   d   e   f   g   h   i   j
    { REG09, {  0,  0,  0,  0,  0,  0, 46,  0,  0,  0 } },
    // Digit 1
    { REG09, { 21, 23, 19, 47, 18, 20, 22, 39, 38,  0 } },
    // Digit 2
    { REG09, { 27, 29, 25, 17, 24, 26, 28, 15, 14, 16 } },
    // Digit 3
    { REG09, { 51, 53, 49, 13, 48, 50, 52, 55, 54, 12 } },
    // Digit 4
    { REG09, { 33, 35, 31,  9, 30, 32, 34, 37, 36,  8 } },
    // Digit 5
    { REG09, { 43, 45, 41,  7, 40, 42, 44, 11, 10,  6 } },
    // Digit 6
    { REG10, { 11, 13,  9, 15,  8, 10, 12, 17, 16, 14 } },
    // Digit 7
    { REG10, { 21, 23, 19,  7, 18, 20, 22, 25, 24,  6 } },
    // Digit 8
    { REG10, { 29, 31, 27, 39, 26, 28, 30, 37, 36, 38 } },
    // Digit 9
    { REG10, { 47, 49, 45, 33, 44, 46, 48, 35, 34, 32 } },
    // Digit 10
    { REG10, { 53, 55, 51, 41, 50, 52, 54, 43, 42, 40 } }
};

// Segment definition for the HP10C
const Digit_t _hp10seg[11] = {
    // Digit 0  a   b   c   d   e   f   g   h   i   j
    { REG10, {  0,  0,  0,  0,  0,  0, 46,  0,  0,  0 } },
    // Digit 1
    { REG10, { 43, 41, 45, 47, 44, 42, 40, 49, 48,  0 } },
    // Digit 2
    { REG10, { 15, 13, 17, 51, 16, 14, 12, 53, 52, 50 } },
    // Digit 3
    { REG10, {  9,  7, 11, 55, 10,  8,  6, 25, 24, 54 } },
    // Digit 4
    { REG10, { 29, 27, 31, 19, 30, 28, 26, 33, 32, 18 } },
    // Digit 5
    { REG10, { 37, 35, 39, 21, 38, 36, 34, 23, 22, 20 } },
    // Digit 6
    { REG09, { 53, 51, 55,  7, 54, 52, 50, 49, 48,  6 } },
    // Digit 7
    { REG09, { 45, 43, 47,  9, 46, 44, 42, 41, 40,  8 } },
    // Digit 8
    { REG09, { 37, 35, 39, 11, 38, 36, 34, 17, 16, 10 } },
    // Digit 9
    { REG09, { 31, 29, 33, 15, 32, 30, 28, 13, 12, 14 } },
    // Digit 10
    { REG09, { 25, 23, 27, 19, 26, 24, 22, 21, 20, 18 } }
};

class CLCD {
    const AllDigits_t *pDigits;
    uint64_t m_r[2];
    Voyager_e m_mod;
public:
    CLCD(uint64_t r0, uint64_t r1, Voyager_e vMod) {
        m_mod = vMod;
        pDigits = (m_mod == V_10C) ? &_hp10seg : &_hp1Xseg;
        m_r[0] = r0;
        m_r[1] = r1;
    }
    // Get segment for given digit (0-9)
    // Return true if segment is on
    bool segment(int d, int seg) {
        d += 1; // Actually digit 1-10
        uint64_t p = m_r[(*pDigits)[d].reg];
        return p & (1LL << (*pDigits)[d].bit[seg]) ? true : false;
    }
    bool hasSign(void) {
        return segment(-1, SEG_g);
    }
    char getSign(void) {
        return hasSign()?'-':' ';
    }
    // Get decimal point for digit (0-9)
    char point(int d) {
        if( segment(d, SEG_i) ) return ',';
        if( segment(d, SEG_h) ) return '.';
        return ' ';
    }
    bool hasPoint(int d) {
        return segment(d, SEG_i) || segment(d, SEG_h);
    }
    // Get annunciator (segment 'j' in each digit)
    // Return true if annunciator is on
    bool annun(int n) {
        return segment(n+1, SEG_j);
    }
    int getAnnun(void) {
        int ann = 0;
        if( annun(ANN_USER) )  ann |= 1<<8;
        if( annun(ANN_F) )     ann |= 1<<7;
        if( annun(ANN_G) )     ann |= 1<<6;
        if( annun(ANN_BEGIN) ) ann |= 1<<5;
        if( annun(ANN_GRAD) )  ann |= 1<<4;
        if( annun(ANN_RAD) )   ann |= 1<<3;
        if( annun(ANN_DMY) )   ann |= 1<<2;
        if( annun(ANN_C) )     ann |= 1<<1;
        if( annun(ANN_PRGM) )  ann |= 1<<0;
        return ann;
    }
    int getSegments(int d) {
        int c = 0;
        if( segment(d, SEG_g) ) c |= 0b000000001;
        if( segment(d, SEG_b) ) c |= 0b000000010;
        if( segment(d, SEG_f) ) c |= 0b000000100;
        if( segment(d, SEG_a) ) c |= 0b000001000;
        if( segment(d, SEG_e) ) c |= 0b000010000;
        if( segment(d, SEG_c) ) c |= 0b000100000;
        if( segment(d, SEG_d) ) c |= 0b001000000;
        return c;
    }
    char getChar(int d) {
        switch( getSegments(d)) {
            case 0x00: return ' ';
            case 0x7E: return '0';
            case 0x22: return '1';
            case 0x5B: return '2';
            case 0x6B: return '3';
            case 0x27: return '4';
            case 0x6D: return '5';
            case 0x7D: return '6';
            case 0x2A: return '7';
            case 0x7F: return '8';
            case 0x6F: return '9';
            case 0x3F: return 'A';
            case 0x75: return 'B';
            case 0x4D: return 'C';
            case 0x73: return 'D';
            case 0x5D: return 'E';
            case 0x1D: return 'F';
            case 0x0C: return 'r'; // r in running
            case 0x11: return 'r'; // r in Error
            case 0x07: return 'u';
            case 0x02: return 'i';
            case 0x0E: return 'n';
            case 0x71: return 'o';
            case 0x1F: return 'P';
            case 0x01: return '-';
            case 0x35: return 'H';
            case 0x53: return 'O';
        }
        return '?';
    }
};

