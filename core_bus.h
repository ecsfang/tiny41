#ifndef __CORE_BUS_H__
#define __CORE_BUS_H__
////////////////////////////////////////////////////////////////////////////////
//
// HP41C stuff
//
////////////////////////////////////////////////////////////////////////////////

// Bus instructions
// Beware: These are in octal

#define INST_DISPLAY_OFF               01340
#define INST_DISPLAY_TOGGLE            01440
#define INST_COMPENSATION_INSTRUCTION  01774
#define INST_C_PFAD                    01760
#define INST_WRITE_ANNUNCIATORS        01360
#define INST_READ_ANNUNCIATORS         00570
#define INST_SRLAD                     00050
#define INST_SRLDB                     00150
#define INST_SRLDC                     00250
#define INST_SRLDAB                    00350
#define INST_SRLDABC                   00450
#define INST_SLLDAB                    00550
#define INST_SLLDABC                   00650
#define INST_SRSDA                     00750
#define INST_SRSDB                     01050
#define INST_SRSDC                     01150
#define INST_SLSDA                     01250
#define INST_SLSDB                     01350
#define INST_SRSDAB                    01450
#define INST_SLSDAB                    01550
#define INST_SRSDABC                   01650
#define INST_SLSDABC                   01750
#define INST_PRPH_SLCT                 01760
#define INST_RAM_SLCT                  01160

#define INST_FLLDA                     00070
#define INST_FLLDB                     00170
#define INST_FLLDC                     00270
#define INST_FLLDAB                    00370
#define INST_FLLDABC                   00470
#define INST_FLSDC                     00670
#define INST_FRSDA                     00770
#define INST_FRSDB                     01070
#define INST_FRSDC                     01170
#define INST_FLSDA                     01270
#define INST_FLSDB                     01370
#define INST_FRSDAB                    01470
#define INST_FLSDAB                    01570
#define INST_FRSDABC                   01670
#define INST_FLSDABC                   01770

// bank switching instructions
#define INST_ENBANK1                   0x100
#define INST_ENBANK2                   0x180
#define INST_ENBANK3                   0x140
#define INST_ENBANK4                   0x1C0

#define INST_ENROM1                    0x1500
#define INST_ENROM2                    0x1980
#define INST_ENROM3                    0x1D40
#define INST_ENROM4                    0x1C0



#define MASK_48_BIT    (0xFFFFFFFFFFFFL)

int display_ce = 0;
char dtext[40];
bool bPunct[40];

#define ROTATE_RIGHT(V) {uint64_t t = (V & 0xF); V >>= 4; V |= t<<44;}
#define  ROTATE_LEFT(V) {uint64_t t = (V & (0xFLL << 44)); V <<= 4; V |= t>>44;}

#endif//__CORE_BUS_H__
