#ifndef __CDC_HELPER_H__
#define __CDC_HELPER_H__

// Roputines for control and access of the FRAM memory

#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <inttypes.h>
#include <malloc.h>
#include <stdarg.h>
/*
 * cdc_helper.h
 *
 * This file is part of the TUP4041 project.
 * Copyright (C) 2024 Meindert Kuipers
 *
 * This is free software: you are free to change and redistribute it.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * 
 * USE AT YOUR OWN RISK
 *
 */


#include "pico/stdlib.h"
#include "pico/platform.h"
//#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "hardware/timer.h"
#include "pico/multicore.h"
#include "hardware/pio.h"
#include "pico/util/queue.h"                    // used for safe FIFO management
#include "hardware/structs/systick.h"
#include "hardware/uart.h"                      // used for UART0 Printer port
//#include "hp41_defs.h"

#include <tusb_option.h>
#include <tusb.h>

#define CDC_PRINT_BUFFER_SIZE 256

#define ITF_CONSOLE     0           // CDC port 0: console, user interface
#define ITF_PRINT       1           // CDC port 1: printer bytes
#define ITF_TRACE       2           // CDC port 2: trace listing
#define ITF_WAND        3           // CDC port 3: HP-IL/ PILBox frames
//#define ITF_ILSCOPE     4           // CDC port 4: HP-IL scope, PILBox frame monitor

extern const char* __in_flash() ITF_str[];

void cdc_printf_console(const char *format, ...);       // prevent using this function
void cdc_printf_(int itf, const char *format, ...);     // prevent using this function

int  cdc_send_console(char* buffer); //, int len);
void cdc_send_string(int itf, char* buffer, int len);
void cdc_send_char(int itf, char c);
void cdc_send_char_flush(int itf, char c);

void cdc_send_printport(char c);
bool cdc_connected(int itf);

void cdc_flush_console();
void cdc_flush(int itf);

uint32_t cdc_available(int itf);
int cdc_read_byte(int itf);
int cdc_read_buf(int itf, char* buffer, int len);


#endif