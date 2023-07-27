#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"

#include "disasm.h"
#include "disstr.h"

#include "instr.h" // Instruction codes ...

extern int peripheral_ce;

void prtCl2Cmd(int inst)
{
	printf("%-8s %s", cmd2[(inst>>5)&0x1F], cmd2param[(inst>>2)&0x07]);
}

void cl0mod(int m, bool bA)
{
	if( bA ) {
		printf("%2d", m);
		if( m>9 )
			printf(" (%X)", m);
	} else {
		printf("%s", mcl0[m]);
	}
}

bool bLDICON = false;

void prtCl0Cmd(int inst, int data, int con)
{
	int mod = (inst>>6) & 0x0F;
	int sub = (inst>>2) & 0x0F;
	switch(sub) {
		case 0x0:
			printf(cmd00[mod]);
			break;
		case 0x1:
			if( mod == 0xF )
				 printf("ST=0");	// clears ST (CPU flags 0-7)
			else {
				printf("CF ");
				cl0mod(mod, false);
			}
			break;
		case 0x2:
			if( mod == 0xF )
				 printf("CLRKEY"); // clears the keydown flag (immediately set if key is down)
			else {
				printf("SF ");
				cl0mod(mod, false);
			}
			break;
		case 0x3:
			if( mod == 0xF )
				 printf("?KEY"); // set carry if keydown flag is set
			else {
				printf("?FS ");
				cl0mod(mod, false);
			}
			break;
		case 0x4:
			printf("LC ");
			cl0mod(mod, true);
			break;
		case 0x5:
			if( mod == 0xF )
				printf("PT=PT-1");	// decrement pointer (if PT=0 then PT=13)
			else {
				printf("?PT= ");
				cl0mod(mod, false);
			}
			break;
		case 0x6:
			printf(cmd06[mod]);
			break;
		case 0x7:
			if( mod == 0xF )
				printf("PT=PT+1");	// increment pointer (if PT=13 then PT=0)
			else {
				printf("PT= ");
				cl0mod(mod, false);
			}
			break;
		case 0x8:
			printf(cmd08[mod]);
			break;
		case 0x9:
			printf("PERTCT ");
			cl0mod(mod, true);
			break;
		case 0xA:
			printf("REG=C ");
			cl0mod(mod, true);
			break;
		case 0xB:
			printf(cmd0b[mod]);
			break;
		case 0xC:
			printf(cmd0c[mod]);
			switch( mod ) {
			case 0x4:
				bLDICON = true;
				break;
			case 0x9:
			case 0xF:
				// Don't know yet - data is pending ...
				// printf(" (PA=%02X)", con&0xFF);
				break;
			case 0xB:	// 001360
				switch( peripheral_ce ) {
				case PH_DISPLAY:
					printf(" (WRTEN - writes annunciators)");
					break;
				case PH_CRDR:
					printf(" (write to cardreader)");
					break;
				}
				break;
			}
			break;
		case 0xD:
			printf("-unused-");
			break;
		case 0xE:
			if( mod == 0x0 )
				printf("READ DATA");	// copy active user memory register to C
			else {
				if ( mod == 5 && peripheral_ce == PH_DISPLAY)
					printf("READEN");
				else {
					printf("C=REG ");
					cl0mod(mod, true);
				}
			}
			break;
		case 0xF:
			if( mod == 0xF )
				printf("???");
			else {
				printf("RCR ");
				cl0mod(mod, false);
			}
			break;
	}
}

bool disAsmPeripheral(int inst)
{
	const char *dBuf = NULL;
  int cmd = inst & 0077;
	switch (peripheral_ce) {
	case PH_DISPLAY:
    if( inst == INST_WRITE_ANNUNCIATORS )
      dBuf = "WRTEN";
    else if( inst == INST_READ_ANNUNCIATORS )
      dBuf = "READEN";
    else if( cmd == 0050 )
      dBuf = inst50disp[inst>>6];
    else if( cmd == 0070 )
      dBuf = inst70disp[inst>>6];
		break;
	case PH_CRDR:
    if( cmd == 0050 )
      dBuf = inst50crd[inst>>6];
		break;
	case PH_PRINTER:
    switch( inst ) {
    case 000007: dBuf = "PRINTC"; break;
    case 000072: dBuf = "RDPTRN"; break;
    case 000073: dBuf = "RDPTRR"; break;
    case 000005: dBuf = "RTNCPU"; break;
    }
		break;
	case PH_WAND:
  	if( inst == INST_WANDRD )
	  	dBuf = "WANDRD";
    break;
	case PH_TIMER:
    switch( inst ) {
    case 000654: dBuf = "FAIL?"; break;
    case 001554: dBuf = "ALARM?"; break;
    }
    if( cmd == 0050 )
      dBuf = inst50timer[inst>>6];
    else if( cmd == 0070 )
      dBuf = inst70timer[inst>>6];
		break;
	}
	if( dBuf ) {
		printf("%s", dBuf);
		return true;
	}
	return false;
}

void disAsm(int inst, int addr, uint64_t data)
{
	static int prev;
	static bool bClass1 = false;
	static int con;
//	printf(" %04X [%02o %04o] %03X %014llX - ", addr, addr>>10, addr & 0x3FF, inst, data);
	printf(" - ");

	// Second word of LDI instruction ...
	if( bLDICON ) {
		con = inst & 0x3FF;
		printf("CON: %03X (%04o)", con, con);
		bLDICON = false;
		return;
	}

	// Second word of jump instruction ...
	if( bClass1 ) {
		prev |= (inst << 6) & 0xFF00;
		printf("%s %04X", cmd1[inst&0x03], prev);
		bClass1 = false;
		return;
	}

	// Check for peripheral instructions
  if( disAsmPeripheral(inst) )
    return;

	switch(inst & 0x03 ) {
		case 0b00:
			prtCl0Cmd(inst, data & 0x3FF, con);
			break;
		case 0b01:
			prev = (inst >> 2) & 0xFF;
			bClass1 = true;
			printf("-->");
			break;
		case 0b10:
			prtCl2Cmd(inst);
			break;
		case 0b11:
		{
			int ad, dir;
			if( inst & 0x200 ) {
				dir = '-';
				prev = 0x40 - ((inst&0x1F8) >> 3);
				ad = addr - prev;
			} else {
				dir = '+';
				prev = (inst&0x1F8) >> 3;
				ad = addr + prev;
			}
			printf("%-8s %c%02X [%04X]", inst&0x100?"JNC":"JC", dir, prev, ad);
		}
	}
}