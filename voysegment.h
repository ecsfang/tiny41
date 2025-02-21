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
    { REG09, { 33, 35,  3,  9, 30, 32, 34, 37,  3,  8 } },
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
public:
    CLCD(uint64_t r0, uint64_t r1, bool b10C=false) {
        m_r[0] = r0;
        m_r[1] = r1;
        pDigits = b10C ? &_hp10seg : &_hp1Xseg;
    }
    // Get segment for given digit (0-9)
    // Return true if segment is on
    bool segment(int d, int seg) {
        d += 1; // Actually digit 1-10
        uint64_t p = m_r[(*pDigits)[d].reg];
        return p & (1LL << (*pDigits)[d].bit[seg]) ? true : false;
    }
    bool sign(void) {
        return segment(-1, SEG_g);
    }
    // Get decimal point for digit (0-9)
    char point(int d) {
        if( segment(d, SEG_i) ) return ',';
        if( segment(d, SEG_h) ) return '.';
        return ' ';
    }
    // Get annunciator (segment 'j' in each digit)
    // Return true if annunciator is on
    bool annun(int n) {
        return segment(n+1, SEG_j);
    }
};
