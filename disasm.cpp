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

#include "disasm.h"
#include "disstr.h"

#include "instr.h" // Instruction codes ...

extern int peripheral_ce;

static char disBuf[256];
static int  pDis = 0;

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
				switch( peripheral_ce ) {
				case PH_DISPLAY:
					addDis(" (WRTEN - writes annunciators)");
					break;
				case PH_CRDR:
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
				if ( mod == 5 && peripheral_ce == PH_DISPLAY)
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
	switch (peripheral_ce) {
	case PH_DISPLAY:
		if( inst == INST_WRITE_ANNUNCIATORS )
			dBuf = "WRTEN";
		else if( inst == INST_READ_ANNUNCIATORS )
			dBuf = "READEN";
		else if( cmd == 0050 )
			dBuf = inst50disp[mod];
		else if( cmd == 0070 )
			dBuf = inst70disp[mod];
		break;
	case PH_CRDR:
		if( cmd == 0050 )
			dBuf = inst50crd[mod];
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

char *disAsm(int inst, int addr, uint64_t data)
{
	static int prev;
	static bool bClass1 = false;
	static int con;
//	printf(" %04X [%02o %04o] %03X %014llX - ", addr, addr>>10, addr & 0x3FF, inst, data);

	pDis = 0;

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
			pDis += sprintf(disBuf+pDis, "%-8s %c%02X [%04X]", inst&0x100?"JNC":"JC", dir, prev, ad);
		}
	}
	return disBuf;
}