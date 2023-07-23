#ifndef __DISASM_H__
#define __DISASM_H__

#define REG_A	0x01
#define REG_B	0x02
#define REG_C	0x04
#define SHIFT_R	2
#define SHIFT_L	0
#define D_LONG	1
#define D_SHORT	0

const char *inst50[] = {
	"SRLDA",	"SRLDB",	"SRLDC",	"SRLDAB",
	"SRLABC",	"SLLDAB",	"SLLABC",	"SRSDA",
	"SRSDB",	"SRSDC",	"SLSDA",	"SLSDB",
	"SRSDAB",	"SLSDAB",	"SRSABC",	"SLSABC"
};
typedef struct {
	uint8_t shft;
	uint8_t reg;
	uint8_t len;
} Inst_t;
Inst_t inst50cmd[16] = {
	{ SHIFT_R|D_LONG,  REG_A, 						48 },
	{ SHIFT_R|D_LONG,  REG_B, 						48 },
	{ SHIFT_R|D_LONG,  REG_C, 						48 },
	{ SHIFT_R|D_LONG,  REG_A|REG_B, 			 8 },
	{ SHIFT_R|D_LONG,  REG_A|REG_B|REG_C, 12 },
	{ SHIFT_L|D_LONG,  REG_A|REG_B, 			 8 },
	{ SHIFT_L|D_LONG,  REG_A|REG_B|REG_C, 12 },
	{ SHIFT_R|D_SHORT, REG_A, 						 1 },
	{ SHIFT_R|D_SHORT, REG_B, 						 1 },
	{ SHIFT_R|D_SHORT, REG_C, 						 1 },
	{ SHIFT_L|D_SHORT, REG_A, 						 1 },
	{ SHIFT_L|D_SHORT, REG_B, 						 1 },
	{ SHIFT_R|D_SHORT, REG_A|REG_B, 			 1 },
	{ SHIFT_L|D_SHORT, REG_A|REG_B, 			 1 },
	{ SHIFT_R|D_SHORT, REG_A|REG_B|REG_C,  1 },
	{ SHIFT_L|D_SHORT, REG_A|REG_B|REG_C,  1 }
};

const char *inst70[] = {
	"FLLDA",	"FLLDB",	"FLLDC",	"FLLDAB",
	"FLLDABC","00570",	"FLSDC",	"FRSDA",
	"FRSDB",	"FRSDC",	"FLSDA",	"FLSDB",
	"FRSDAB",	"FLSDAB",	"FRSDABC","FLSDABC",
};
Inst_t inst70cmd[16] = {
	{ SHIFT_L, REG_A, 						 0 },
	{ SHIFT_L, REG_B, 						 0 },
	{ SHIFT_L, REG_C, 						 0 },
	{ SHIFT_L, REG_A|REG_B, 			 6 },
	{ SHIFT_L, REG_A|REG_B|REG_C,  4 },
	{ 0, 			 0, 			  				 0 },
	{ SHIFT_L, REG_C, 						 1 },
	{ SHIFT_R, REG_A, 						 1 },
	{ SHIFT_R, REG_B, 						 1 },
	{ SHIFT_R, REG_C, 						 1 },
	{ SHIFT_L, REG_A, 						 1 },
	{ SHIFT_L, REG_B, 						 1 },
	{ SHIFT_R, REG_A|REG_B, 			 1 },
	{ SHIFT_L, REG_A|REG_B, 			 1 },
	{ SHIFT_R, REG_A|REG_B|REG_C,  1 },
	{ SHIFT_L, REG_A|REG_B|REG_C,  1 }
};

const char *cmd2param[8] = {
    "@PT", "S&X", "PT<", "ALL", "P-Q", "XS", "M", "MS"
};

const char *mcl0[0x10] = {
	"3",	"4",	"5",	"10",
	"8",	"6",	"11",	"Unused",
	"2",	"9",	"7",	"13",
	"1",	"12",	"0",	"Special"
};

const char *cmd00[16] = {
    "NOP", 			// no operation
    "WRIT S&X", 	// writes word in C[2;0] to system memory given in C[6;3]
    "-unused-",
    "-unused-",
    "ENBANK1",		// enables primary bank
    "ENBANK3",		// enables third bank
    "ENBANK2",		// enables secondary bank
    "ENBANK4",		// enables forth bank
    "HPIL=C 0",
    "HPIL=C 1",
    "HPIL=C 2",
    "HPIL=C 3",
    "HPIL=C 4",
    "HPIL=C 5",
    "HPIL=C 6",
    "HPIL=C 7"
};

const char *cmd06[16] = {
    "-unused-",
    "G=C @PT,+", 	// copy C register digits at PT and PT+1 to G register
    "C=G @PT,+", 	// copy G register to C register digits at PT and PT+1
    "C<>G @PT,+", 	// exchange C register digits at PT and PT+1 with G register
    "-unused-",
    "M=C ALL", 		// copy C register to M
    "C=M ALL", 		// copy M register to C
    "C<>M ALL", 	// exchange C and M registers
    "-unused-",
    "T=ST", 		// copy ST to T
    "ST=T", 		// copy T to ST
    "ST<>T", 		// exchange ST and T
    "-unused-",
    "ST=C XP", 		// copy C[1;0] to ST
    "C=ST XP", 		// copy ST to C[1;0]
    "C<>ST XP"   	// exchange ST and C[1;0]
};

const char *cmd08[16] = {
    "XQ>GO",    // pop return stack, turns the latest XQ into a GO
    "POWOFF", 	// disp on: stop CPU, disp off: turn HP41 off, must be followed by NOP
    "SLCT P",   // select P as active pointer
    "SLCT Q",   // select Q as active pointer
    "?P=Q",     // set carry if P and Q have the same value
    "?LOWBAT", 	// set carry if battery is low
    "A=B=C=0", 	// clear A, B and C registers
    "GOTO ADR", // jumps to address in C[6;3]
    "C=KEY KY", // copy key code from KY to C[4;3]
    "SETHEX", 	// set CPU to hexadecimal mode
    "SETDEC", 	// set CPU to decimal mode
    "DSPOFF", 	// turns display off
    "DSPTOG", 	// toggles display on/off
    "?C RTN", 	// return to address in STK if carry is set
    "?NC RTN", 	// return to address in STK if carry is clear
    "RTN"  		// return to address in STK
};

const char *cmd0b[16] = {
    "?PF 3",
    "?PF 4",
    "?EDAV", 			// set carry if the diode of IR module is available
    "?ORAV", 			// set carry if output register is available
    "?FRAV", 			// set carry if a frame is available from HP-IL interface
    "?IFCR", 			// set carry if HP-IL interface is ready
    "?TFAIL",
    "-unused-",
    "?WNDB", 			// set carry if there is data in the wand buffer
    "?FRNS", 			// set carry if the frame transmitted (HPIL) not returns as sent
    "?SRQR", 			// set carry if HPIL interface needs service
    "?SERV", 			// set carry if any peripheral unit needs service
    "?CRDR", 			// used with card reader
    "?ALM", 			// set carry if an alarm from the timer has occured
    "?PBSY", 			// set carry if HP82143 printer is busy
    "-unused-"
};

const char *cmd0c[16] = {
    "ROM BLK", 	// moves HEPAX ROM to memroy block specified in C[0]
    "N=C ALL", 	// copy C register to N
    "C=N ALL", 	// copy N register to C
    "C<>N ALL", // exchange C and N registers
    "LDI S&X",	// load the 10-bit word at next address to C[2;0]
    "PUSH ADR", // push STK up and store C[6;3] in bottom STK
    "POP ADR", 	// copy bottom STK to C[6;3] and STK drops
    "WPTOG", 	// toggles write protection of HEPAX RAM specified in C[0]
    "GOTO KEY", // KEY register is written into lowets byte of PC
    "RAM SLCT", // select user memory register specified in C[2;0]
    "-unused-",
    "WRIT DATA", // copy C to active user memory register
    "FETCH S&X", // fetches the word at system memory given in C[6;3] to C[2;0]
    "C=C OR A",  // do logical OR on C and A registers and store result in C
    "C=C AND A", // do logical AND on C and A registers and store result in C
    "PRPH SLCT"  // select peripheral unit specified in C[2;0]
};

#endif