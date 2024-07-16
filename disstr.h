/*
 * disstr.h
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

#ifndef __DISSTR_H__
#define __DISSTR_H__

// All disassembler mnemonics

// Display instructions
extern const char *inst50disp[16];

extern const char *inst70disp[16];

// Cardreader instructions
extern const char *inst50crd[16];

// Timer instructions
extern const char *inst50timer[16];

extern const char *inst70timer[16];

extern const Inst_t inst50cmd[16];

extern const Inst_t inst70cmd[16];

extern const char *mcl0[16];

extern const char *cmd00[16];

extern const char *cmd06[16];

extern const char *cmd08[16];

extern const char *cmd0b[16];

extern const char *cmd0c[16];


extern const char *cmd1[4];

/*
   3   6   1   7   5   0   2   4
  ALL  M  S&X MS  XS  @PT PT← P-Q
  00E 01A 006 01E 016 002 00A 012
  011 110 001 111 101 000 010 100
*/
extern const char *cmd2param[8];

extern const char *cmd2[32];

#endif