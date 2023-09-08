/*
 * disasm.h
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

#ifndef __DISASM_H__
#define __DISASM_H__

#define REG_A	0x01
#define REG_B	0x02
#define REG_C	0x04
#define SHIFT_R	2
#define SHIFT_L	0
#define D_LONG	1
#define D_SHORT	0

enum {
  PH_NONE     = 0b00000,
  PH_TIMER    = 0b00001,
  PH_CRDR     = 0b00010,
  PH_DISPLAY  = 0b00100,
  PH_WAND     = 0b01000,
  PH_PRINTER  = 0b10000
};

typedef struct {
	uint8_t shft;
	uint8_t reg;
	uint8_t len;
} Inst_t;

void disAsm(int inst, int addr, uint64_t data);

extern Inst_t inst50cmd[16];
extern Inst_t inst70cmd[16];

#endif