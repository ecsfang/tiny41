#ifndef __DISSTR_H__
#define __DISSTR_H__

// All disassembler mnemonics

// Display instructions
const char *inst50disp[] = {
  "SRLDA",    // 000050
  "SRLDB",    // 000150
  "SRLDC",    // 000250
  "SRLDAB",   // 000350
  "SRLABC",   // 000450
  "SLLDAB",   // 000550
  "SLLABC",   // 000650
  "SRSDA",    // 000750
  "SRSDB",    // 001050
  "SRSDC",    // 001150
  "SLSDA",    // 001250
  "SLSDB",    // 001350
  "SRSDAB",   // 001450
  "SLSDAB",   // 001550
  "SRSABC",   // 001650
  "SLSABC"    // 001750
};

const char *inst70disp[] = {
  "FLLDA",    // 000070
  "FLLDB",    // 000170
  "FLLDC",    // 000270
  "FLLDAB",   // 000370
  "FLLDABC",  // 000470
  NULL,       // 000570
  "FLSDC",    // 000670
  "FRSDA",    // 000770
  "FRSDB",    // 001070
  "FRSDC",    // 001170
  "FLSDA",    // 001270
  "FLSDB",    // 001370
  "FRSDAB",   // 001470
  "FLSDAB",   // 001570
  "FRSDABC",  // 001670
  "FLSDABC",  // 001770
};

// Cardreader instructions
const char *inst50crd[] = {
  "ENWRIT",   // 000050
  "STWRIT",   // 000150
  "ENREAD",   // 000250
  "STREAD",   // 000350
  NULL,       // 000450
  "CRDWPF",   // 000550
  NULL,       // 000650
  "CRDOHF",   // 000750
  NULL,       // 001050
  "CRDINF",   // 001150
  NULL,       // 001250
  "TSTBUF",   // 001350
  "TRPCRD",   // 001450
  "TCLCRD",   // 001550
  NULL,       // 001650
  "CRDFLG"    // 001750
};

// Timer instructions
const char *inst50timer[] = {
  "WRTIME",   // 000050
  "WDTIME",   // 000150
  "WRALM",    // 000250
  "WRSTS",    // 000350
  "WRSCR",    // 000450
  "WSINT",    // 000550
  NULL,       // 000650
  "STPINT",   // 000750
  "DSWKUP",   // 001050
  "ENWKUP",   // 001150
  "DSALM",    // 001250
  "ENALM",    // 001350
  "STOPC",    // 001450
  "STARTC",   // 001550
  "PT=B",     // 001650
  "PT=A"      // 001750
};

const char *inst70timer[] = {
  "RDTIME",   // 000070
  "RCTIME",   // 000170
  "RDALM",    // 000270
  "RDSTS",    // 000370
  "RDSCR",    // 000470
  "RDINT",    // 000570
  NULL,       // 000670
  NULL,       // 000770
  NULL,       // 001070
  NULL,       // 001170
  NULL,       // 001270
  NULL,       // 001370
  NULL,       // 001470
  NULL,       // 001570
  NULL,       // 001670
  NULL,       // 001770
};

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


const char *cmd1[4] = {
    "?NC XQ", "?C XQ", "?NC GO", "?C GO"
};

/*
   3   6   1   7   5   0   2   4
  ALL  M  S&X MS  XS  @PT PT← P-Q
  00E 01A 006 01E 016 002 00A 012
  011 110 001 111 101 000 010 100
*/
const char *cmd2param[8] = {
    "@PT", "S&X", "PT<", "ALL", "P-Q", "XS", "M", "MS"
};

const char *cmd2[32] = {
	"A=0",    //Clear A
	"B=0",    //Clear B
	"C=0",    //Clear C
	"A<>B",   //Exchange A and B
	"B=A",    //Copy B to A
	"A<>C",   //Exchange A and C
	"C=B",    //Copy B to C
	"C<>B",   //Exchange C and B
	"A=C",    //Copy C to A
	"A=A+B",  //Add B to A
	"A=A+C",  //Add C to A
	"A=A+1",  //Increment A
	"A=A-B",  //Subtract B from A
	"A=A-1",  //Decrement A
	"A=A-C",  //Subtract C from A
	"C=C+C",  //Shift C 1 bit left
	"C=C+A",  //Add A to C
	"C=C+1",  //Increment C
	"C=A-C",  //Subtract C from A
	"C=C-1",  //Decrement C
	"C=0-C",  //1’s complement
	"C=-C-1", //2’s comp
	"?B!=0",  //Carry if B≠0
	"?C!=0",  //Carry if C≠0
	"?A<C",   //Carry if A<C
	"?A<B",   //Carry if A<B
	"?A!=0",  //Carry if A≠0
	"?A!=C",  //Carry if A≠C
	"RSHFA",  //Shift A 1 digit
	"RSHFB",  //Shift B 1 digit right
	"RSHFC",  //Shift C 1 digit right
	"LSHFA"   //Shift A 1 digit left
};

#endif