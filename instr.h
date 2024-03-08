#ifndef __INSTR_H__
#define __INSTR_H__

// Bus instructions
// Beware: These are in octal

#define INST_WRITE_DATA                00050    // 0000 0010 1000   0x28
#define INST_READ_DATA                 00070    // 0000 0011 1000   0x38

#define INST_RW_MASK                   0x2F
#define INST_READ_OR_WRITE             (INST_READ_DATA & INST_WRITE_DATA)
#define INST_RDATA                     INST_READ_DATA
#define INST_WDATA                     01360    // 001 011 110 000   0x2F0

#define INST_LDI                       00460
#define INST_FETCH                     01460
#define INST_WRITE                     00100

// FI flag instructions
#define INST_TFAIL                     00654
#define INST_WNDB                      01054
#define INST_ALM                       01554
#define INST_PBSY                      01654

#define INST_WANDRD                    INST_RDATA

#define INST_DISPLAY_OFF               01340
#define INST_DISPLAY_TOGGLE            01440
#define INST_COMPENSATION_INSTRUCTION  01774
//#define INST_C_PFAD                    01760
#define INST_WRITE_ANNUNCIATORS        INST_WDATA
#define INST_READ_ANNUNCIATORS         00570

#define INST_SETHEX                    0x260
#define INST_SETDEC                    0x2A0


#define INST_PRPH_SLCT                 01760 // CPFAD
#define INST_RAM_SLCT                  01160 // CDADD
#define INST_POWOFF                    00140

#define INST_SELP                      00044 // Select peripherial
#define INST_SEL_PRT                   ((9<<6) | INST_SELP) // Select printer

// Display
#define INST_SRLDA                     (00000 | INST_WRITE_DATA)
#define INST_SRLDB                     (00100 | INST_WRITE_DATA)
#define INST_SRLDC                     (00200 | INST_WRITE_DATA)
#define INST_SRLDAB                    (00300 | INST_WRITE_DATA)
#define INST_SRLABC                    (00400 | INST_WRITE_DATA)
#define INST_SLLDAB                    (00500 | INST_WRITE_DATA)
#define INST_SLLABC                    (00600 | INST_WRITE_DATA)
#define INST_SRSDA                     (00700 | INST_WRITE_DATA)
#define INST_SRSDB                     (01000 | INST_WRITE_DATA)
#define INST_SRSDC                     (01100 | INST_WRITE_DATA)
#define INST_SLSDA                     (01200 | INST_WRITE_DATA)
#define INST_SLSDB                     (01300 | INST_WRITE_DATA)
#define INST_SRSDAB                    (01400 | INST_WRITE_DATA)
#define INST_SLSDAB                    (01500 | INST_WRITE_DATA)
#define INST_SRSABC                    (01600 | INST_WRITE_DATA)
#define INST_SLSABC                    (01700 | INST_WRITE_DATA)

#define INST_FLLDA                     (00000 | INST_READ_DATA)
#define INST_FLLDB                     (00100 | INST_READ_DATA)
#define INST_FLLDC                     (00200 | INST_READ_DATA)
#define INST_FLLDAB                    (00300 | INST_READ_DATA)
#define INST_FLLDABC                   (00400 | INST_READ_DATA)
#define INST_FLSDC                     (00600 | INST_READ_DATA)
#define INST_FRSDA                     (00700 | INST_READ_DATA)
#define INST_FRSDB                     (01000 | INST_READ_DATA)
#define INST_FRSDC                     (01100 | INST_READ_DATA)
#define INST_FLSDA                     (01200 | INST_READ_DATA)
#define INST_FLSDB                     (01300 | INST_READ_DATA)
#define INST_FRSDAB                    (01400 | INST_READ_DATA)
#define INST_FLSDAB                    (01500 | INST_READ_DATA)
#define INST_FRSDABC                   (01600 | INST_READ_DATA)
#define INST_FLSDABC                   (01700 | INST_READ_DATA)

// Timer module
#define INST_WRTIME                    (00000 | INST_WRITE_DATA)
#define INST_WDTIME                    (00100 | INST_WRITE_DATA)
#define INST_WRALM                     (00200 | INST_WRITE_DATA)
#define INST_WRSTS                     (00300 | INST_WRITE_DATA)
#define INST_WRSCR                     (00400 | INST_WRITE_DATA)
#define INST_WSINT                     (00500 | INST_WRITE_DATA)
//#define INST_                        (00600 | INST_WRITE_DATA)
#define INST_STPINT                    (00700 | INST_WRITE_DATA)
#define INST_DSWKUP                    (01000 | INST_WRITE_DATA)
#define INST_SETTST                    (01100 | INST_WRITE_DATA)
#define INST_DSALM                     (01200 | INST_WRITE_DATA)
#define INST_ENALM                     (01300 | INST_WRITE_DATA)
#define INST_STOPC                     (01400 | INST_WRITE_DATA)
#define INST_STARTC                    (01500 | INST_WRITE_DATA)
#define INST_PT_B                      (01600 | INST_WRITE_DATA)
#define INST_PT_A                      (01700 | INST_WRITE_DATA)

#define INST_RDTIME                    (00000 | INST_READ_DATA)
#define INST_RCTIME                    (00100 | INST_READ_DATA)
#define INST_RDALM                     (00200 | INST_READ_DATA)
#define INST_RDSTS                     (00300 | INST_READ_DATA)
#define INST_RDSCR                     (00400 | INST_READ_DATA)
#define INST_RDINT                     (00500 | INST_READ_DATA)
//#define INST_                     (00700 | INST_READ_DATA)
//#define INST_                     (01000 | INST_READ_DATA)
//#define INST_                     (01100 | INST_READ_DATA)
//#define INST_                     (01200 | INST_READ_DATA)
//#define INST_                     (01300 | INST_READ_DATA)
//#define INST_                     (01400 | INST_READ_DATA)
//#define INST_                     (01500 | INST_READ_DATA)
//#define INST_                     (01600 | INST_READ_DATA)
//#define INST_                     (01700 | INST_READ_DATA)

// bank switching instructions
#define INST_ENBANK1                   0x100
#define INST_ENBANK2                   0x180
#define INST_ENBANK3                   0x140
#define INST_ENBANK4                   0x1C0

#define INST_ENROM1                    0x150
#define INST_ENROM2                    0x198
#define INST_ENROM3                    0x1D4
#define INST_ENROM4                    0x1C0

#define INST_EADD_C                    0x000  // new instruction to access Expanded Memory (above 0x400)

#define TIMR_ADDR                      0xFB   // The Timer
#define CRDR_ADDR                      0xFC   // The Card Reader
#define DISP_ADDR                      0xFD   // The display
#define WAND_ADDR                      0xFE   // The Wand
#define BLINKY_ADDR                    0x20   // The IR printer module
#define NONE_ADDR                      0x00   // None ...

#define NPIC_IR_PRT_WRITE               0b000
#define NPIC_IR_PRT_READ                0b010
#define NPIC_IR_PRT_CMD                 0b100

#endif//__INSTR_H__