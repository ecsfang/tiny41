#ifndef __INSTR_H__
#define __INSTR_H__

// Bus instructions
// Beware: These are in octal

#define INST_WRITE_DATA                00050    // 0000 0010 1000   0x28
#define INST_READ_DATA                 00070    // 0000 0011 1000   0x38

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
#define INST_C_PFAD                    01760
#define INST_WRITE_ANNUNCIATORS        INST_WDATA
#define INST_READ_ANNUNCIATORS         00570

#define INST_SETHEX                    0x260
#define INST_SETDEC                    0x2A0

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

#define INST_PRPH_SLCT                 01760 // CPFAD
#define INST_RAM_SLCT                  01160 // CDADD
#define INST_SEL_PRT                   01144 // Select printer
#define INST_POWOFF                    00140

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

// bank switching instructions
#define INST_ENBANK1                   0x100
#define INST_ENBANK2                   0x180
#define INST_ENBANK3                   0x140
#define INST_ENBANK4                   0x1C0

#define INST_ENROM1                    0x1500
#define INST_ENROM2                    0x1980
#define INST_ENROM3                    0x1D40
#define INST_ENROM4                    0x1C0

#define TIMR_ADDR                      0xFB   // The Timer
#define CRDR_ADDR                      0xFC   // The Card Reader
#define DISP_ADDR                      0xFD   // The display
#define WAND_ADDR                      0xFE   // The Wand
#define BLINKY_ADDR                    0x20   // The IR printer module
#define NONE_ADDR                      0x00   // None ...

#endif//__INSTR_H__