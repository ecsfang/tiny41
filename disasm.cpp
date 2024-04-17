/*
 * disasm.cpp
 *
 * This file is part of the Tiny41 HP41/Pico project.
 * Copyright (C) 2023 Thomas Fänge <ecsfang@hotmail.com>
 *
 * This is free software: you are free to change and redistribute it.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 */

#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "tiny41.h"
#include "core_bus.h"
#include "disasm.h"
#include "disstr.h"

#include "instr.h" // Instruction codes ...

//extern int selectedPeripheral(void);
extern CPeripherial peripheral;

// Buffer for the disassembled string
static char disBuf[256];
static int  pDis = 0;

enum {
	PER_NONE = 0,
	PER_HPIL_0 = 0+1,
	PER_HPIL_1 = 1+1,
	PER_HPIL_2 = 2+1,
	PER_HPIL_3 = 3+1,
	PER_HPIL_4 = 4+1,
	PER_HPIL_5 = 5+1,
	PER_HPIL_6 = 6+1,
	PER_HPIL_7 = 7+1,
	PER_PRT = 9+1,
};

int selPer = PER_NONE;	// Selected peripheral (1-16)

void prtCl2Cmd(int inst)
{
	pDis += sprintf(disBuf+pDis, "%-8s %s", cmd2[(inst>>5)&0x1F], cmd2param[(inst>>2)&0x07]);
}

void addDis(const char* dis)
{
	pDis += sprintf(disBuf+pDis, dis);
}

void cl0mod(int m, bool bA)
{
	if( bA ) {
		pDis += sprintf(disBuf+pDis, "%2d", m);
		if( m>9 )
			pDis += sprintf(disBuf+pDis, " (%X)", m);
	} else {
		pDis += sprintf(disBuf+pDis, "%s", mcl0[m]);
	}
}

bool bLDICON = false;

void prtCl0Cmd(int inst, int data, int con)
{
	int mod = (inst>>6) & 0x0F;
	int sub = (inst>>2) & 0x0F;
	switch(sub) {
		case 0x0:
			pDis += sprintf(disBuf+pDis, cmd00[mod]);
			break;
		case 0x1:
			if( mod == 0xF )
				 addDis("ST=0");	// clears ST (CPU flags 0-7)
			else {
				addDis("CF ");
				cl0mod(mod, false);
			}
			break;
		case 0x2:
			if( mod == 0xF )
				 addDis("CLRKEY"); // clears the keydown flag (immediately set if key is down)
			else {
				addDis("SF ");
				cl0mod(mod, false);
			}
			break;
		case 0x3:
			if( mod == 0xF )
				 addDis("?KEY"); // set carry if keydown flag is set
			else {
				addDis("?FS ");
				cl0mod(mod, false);
			}
			break;
		case 0x4:
			addDis("LC ");
			cl0mod(mod, true);
			break;
		case 0x5:
			if( mod == 0xF )
				addDis("PT=PT-1");	// decrement pointer (if PT=0 then PT=13)
			else {
				addDis("?PT= ");
				cl0mod(mod, false);
			}
			break;
		case 0x6:
			addDis(cmd06[mod]);
			break;
		case 0x7:
			if( mod == 0xF )
				addDis("PT=PT+1");	// increment pointer (if PT=13 then PT=0)
			else {
				addDis("PT= ");
				cl0mod(mod, false);
			}
			break;
		case 0x8:
			addDis(cmd08[mod]);
			break;
		case 0x9:
			addDis("PERTCT ");
			cl0mod(mod, true);
			selPer = mod+1;
			break;
		case 0xA:
			addDis("REG=C ");
			cl0mod(mod, true);
			break;
		case 0xB:
			addDis(cmd0b[mod]);
			break;
		case 0xC:
			addDis(cmd0c[mod]);
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
				switch( peripheral.get() ) {
				case DISP_ADDR:
					addDis(" (WRTEN - writes annunciators)");
					break;
				case CRDR_ADDR:
					addDis(" (write to cardreader)");
					break;
				}
				break;
			}
			break;
		case 0xD:
			addDis("-unused-");
			break;
		case 0xE:
			if( mod == 0x0 )
				addDis("READ DATA");	// copy active user memory register to C
			else {
				if ( mod == 5 && peripheral.get() == DISP_ADDR)
					addDis("READEN");
				else {
					addDis("C=REG ");
					cl0mod(mod, true);
				}
			}
			break;
		case 0xF:
			if( mod == 0xF )
				addDis("???");
			else {
				addDis("RCR ");
				cl0mod(mod, false);
			}
			break;
	}
}

bool disAsmPeripheral(int inst)
{
	const char *dBuf = NULL;
  int cmd = inst & 0077;
  int mod = inst >> 6;
	switch (peripheral.get()) {
	case DISP_ADDR:
		if( inst == INST_WRITE_ANNUNCIATORS )
			dBuf = "WRTEN";
		else if( inst == INST_READ_ANNUNCIATORS )
			dBuf = "READEN";
		else if( cmd == 0050 )
			dBuf = inst50disp[mod];
		else if( cmd == 0070 )
			dBuf = inst70disp[mod];
		break;
	case CRDR_ADDR:
		if( cmd == 0050 )
			dBuf = inst50crd[mod];
		break;
/*
	case PH_PRINTER:
    	switch( inst ) {
    	case 000007: dBuf = "PRINTC"; break;
    	case 000072: dBuf = "RDPTRN"; break;
    	case 000073: dBuf = "RDPTRR"; break;
    	case 000005: dBuf = "RTNCPU"; break;
    	}
			break;
*/
	case WAND_ADDR:
  		if( inst == INST_WANDRD )
	  		dBuf = "WANDRD";
    	break;
	case TIMR_ADDR:
		if( inst == INST_TFAIL )
			dBuf = "?TFAIL";
		else if( inst == INST_ALM )
			dBuf = "?ALM";
		else if( cmd == 0050 )
			dBuf = inst50timer[mod];
		else if( cmd == 0070 )
			dBuf = inst70timer[mod];
		break;
	}
	if( dBuf ) {
		addDis(dBuf);
		return true;
	}
	return false;
}

int disMod(const char *fmt, int m)
{
	return sprintf(disBuf, fmt, m);
}

char *disAsm(int inst, int addr, uint64_t data, uint8_t sync)
{
	static int prev;
	static bool bClass1 = false;
	static int con;
	int mod = inst >> 6;
	static bool bRead = false;

	pDis = 0;

#ifdef DISABLE_DISPRINT
	if( bRead ) {
		printf(" --> %014llX\n", data);
		bRead = false;
	}
#endif

	if( selPer ) {
		// Execute specific peripheral instruction
		switch( selPer ) {
			case PER_PRT:	// Printer
				if( inst == 0x005 ) {
					pDis = sprintf(disBuf, "RTNCPU"); break;
				} else {
					int r = (inst >> 6) & 0x0F;
          int cmd = (inst >> 1) & 0x3;

					switch(cmd) {
					case 0b00:  // Write ...
						pDis = sprintf(disBuf, "WRITE (%d)", r);
#ifdef DISABLE_DISPRINT
						//if( r == 5 || r == 10 || r == 11)
						if( r == 10 )
							printf("WRITE (%d) <-- %014llX\n", r, data);
#endif
						if( r == 11 ) {
							// Add C X to print buffer!
						}
						break;
					case 0b01:  // Read ...
						pDis = sprintf(disBuf, "READ (%d)", r);
#ifdef DISABLE_DISPRINT
						//if( r == 5 || r == 10 || r == 11)
						if( r == 10 )
						{
							printf("READ (%d) ", r);
							bRead = true;	// Fetch data in next cycle ...
						}
#endif
						break;
					case 0b10:
						pDis = sprintf(disBuf, "FUNC0 (%d)", r);
#ifdef DISABLE_DISPRINT
						printf("FUNC (%d)\n", r);
#endif
						break;
					case 0b11:  // ???
						pDis = sprintf(disBuf, "FUNC1 (%d)", r);
						break;
					}
				}
				break;
			default:
				switch( inst & 0x3F ) {
				case 0x03:	// Check peripheral flag N
					pDis = disMod("?PFSET %X", mod);
					break;
				case 0x3A:
				case 0x3B:
					// Read data line into C from reg N
					pDis = disMod("C=DATA[%X]", mod);
					break;
				default:
					if( inst & 0x2 ) {
						// Load 8 bit character to peripheral
						pDis = sprintf(disBuf, "LC %02X", inst>>2);
					}
				}
		}
		if( inst & 0x01 ) { // Bit0? Return control to CPU
			selPer = PER_NONE;
			if( inst != 0x005 )
				pDis += sprintf(disBuf+pDis," R");
		}
		return disBuf;
	}

	if( sync ) {
		bLDICON = false;
		bClass1 = false;
	}
	// Second word of LDI instruction ...
	if( bLDICON ) {
		con = inst & 0x3FF;
		pDis = sprintf(disBuf, "CON: %03X (%04o)", con, con);
		bLDICON = false;
		return disBuf;
	}

	// Second word of jump instruction ...
	if( bClass1 ) {
		prev |= (inst << 6) & 0xFF00;
		pDis = sprintf(disBuf, "%s %04X", cmd1[inst&0x03], prev);
		bClass1 = false;
		return disBuf;
	}

	// Check for peripheral instructions
  if( disAsmPeripheral(inst) )
    return disBuf;

	switch(inst & 0x03 ) {
		case 0b00:
			prtCl0Cmd(inst, data & 0x3FF, con);
			break;
		case 0b01:
			prev = (inst >> 2) & 0xFF;
			bClass1 = true;
			addDis("-->");
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
			pDis += sprintf(disBuf+pDis, "%-8s %c%02X [%04X]", inst&0b100?"JC":"JNC", dir, prev, ad);
		}
	}
	return disBuf;
}

uint16_t fat[64];
extern uint16_t readRom(int a);

void dumpRom(int p)
{
	int xr = readRom(p<<12);
	int nrf = readRom(p<<12 | 1);
	int i, l;
	printf("XROM %02X.xx - %d functions\n", xr, nrf);

	for(i=0; i<64; i++) {
		fat[i] = 0;
	}

	int x,y;
	for(i=0; i<nrf; i++) {
		x = readRom(p<<12 | (2+i*2));
		y = readRom(p<<12 | (2+i*2+1));
		printf("#%d - %03X %03X -> %04X - ", i, x, y, (p<<12)|(x&0xF)<<8|y&0xFF);
		fat[i] = (x & 0x300)<<4|(x&0xF)<<8|y&0xFF;
		int a = (fat[i]&0xFFF | p<<12) - 1;
		printf("%04X ", a);
		if( !(fat[i] & 0x3000) ) {
			int n=0;
			do {
				l = readRom(a);
				if( (l&0x7F) < 0x20) l += '@';
				printf("%c", l&0x7F);
				a--;
				n++;
			} while(!(l&0x80));
			if( n>7 )
				fat[i] |= 0x1000; // Rom label!
		} else
			printf("User code");
		printf("\n");
	}
	x = readRom(p<<12 | (2+i*2));
	y = readRom(p<<12 | (2+i*2+1));
	if( x || y) {
		printf("FAT not ending in 0 (%X:%X)!\n", x, y);
	}
	for(i=0; i<nrf; i++) {
		if( fat[i] & 0x3000 )
			continue;
		uint16_t s = fat[i] | p<<12;
		uint16_t e = fat[i+1] & 0xFFF;
		if( !e )
			e = 0xFF0;
		e |= p<<12;
		int a = s-1;
		do {
			l = readRom(a);
			if( (l&0x7F) < 0x20) l += '@';
			printf("%c", l&0x7F);
			a--;
		} while(!(l&0x80));
		printf("\n===================\n");
		for(uint16_t adr=s; adr<e; adr++) {
			x = readRom(adr);
			printf("%04X %03X - %s\n", a, x, disAsm(x, adr, 0, 0));
		}
		printf("\n===================\n");
	}
}