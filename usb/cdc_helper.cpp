/*
 * cdc_helper.c
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

// cdc_helper.c
// helper functions for the multi CDC USB interface
// ITF assignments:
//      0 - console, user interface (currently user interface running on UART)
//          console is now used for the tracer output
//      1 - HP82143 printer port
//      2 - TRACE listing
//      3 - Wand data

#include "cdc_helper.h"

const char* __in_flash() ITF_str[] = {  "ITF_CONSOLE",
                                        "ITF_PRINT",
                                        "ITF_TRACE",
                                        "ITF_WAND",
//                                      "ITF_ILSCOPE",
};

#define ITF_CONSOLE     0           // CDC port 0: console, user interface
#define ITF_PRINT       1           // CDC port 1: printer bytes
#define ITF_TRACE       2           // CDC port 2: trace listing
#define ITF_WAND        3           // CDC port 3: Wand data
//#define ITF_ILSCOPE     4           // CDC port 4: HP-IL scope, PILBox frame monitor

// printf version for printing into the console
// and flushes the buffer
void cdc_printf_console(const char *format, ...) {

    char buffer[CDC_PRINT_BUFFER_SIZE];

    int i = CDC_PRINT_BUFFER_SIZE;
    int len;
    
    len = sprintf(buffer, format);

    if (len >= 0) 
    {
        tud_cdc_n_write(ITF_CONSOLE, buffer, len);
        // tud_cdc_n_write_flush(ITF_CONSOLE);
    }
}

static void wait_for_write(int itf, uint32_t len)
{
    uint32_t avail;
    do {
        tud_task();
        avail = tud_cdc_n_write_available(itf);
    } while (avail < len);

}

// printf version for printing a formatted string
// does not flush
void cdc_printf_(int itf, const char *format, ...) {

    char buffer[CDC_PRINT_BUFFER_SIZE];
    int len;
    
    len = sprintf(buffer, format);
    if (len >= 0) 
    {
        wait_for_write(itf, len);
        tud_cdc_n_write(itf, buffer, len);
    }
}

int cdc_send_console(char* buffer) //, int len)
{
    int len = strlen(buffer);
    int sent;       // number of bytes really sent

    wait_for_write(ITF_CONSOLE, len);

    sent = tud_cdc_n_write(ITF_CONSOLE, buffer, len);

    if (sent != len) {
        // something going on
        printf("\n** len: %d - sent: %d  %.10s\n", len, sent, buffer);
    }
    return len;
}

void cdc_flush_console()
{
    tud_cdc_n_write_flush(ITF_CONSOLE);
}

// send the string in buffer with len characters
// the function does not flush
void cdc_send_string(int itf, char* buffer, int len)
{
    // while (len) {
    //     wait_for_write(ITF_CONSOLE, len + 50);
    //     uint32_t w = tud_cdc_n_write(ITF_CONSOLE, buffer, len);
    //     buffer += w;
    //     len -= w;
    // }
    int sent;       // number of bytes really sent

    wait_for_write(itf, len);                   // wait until len bytes available in the send buffer
    sent = tud_cdc_n_write(itf, buffer, len);
    if (sent != len)
    {
        // something going on
        printf("\n** len: %d - sent: %d  %.10s\n", len, sent, buffer);
    }
}

// function to send one char to the printerport, and flush buffer
void cdc_send_printport(char c)
{

    // tud_cdc_n_connected cannot be used when sending into 
    // the HP82240 printer simulator due to its control of DTR
	tud_cdc_n_write_char(ITF_PRINT, c);
	tud_cdc_n_write_flush(ITF_PRINT);
}

// function to send one char to a port, no flush
void cdc_send_char(int itf, char c)
{
	tud_cdc_n_write_char(itf, c);
}

// function to send one char to a port and flush
void cdc_send_char_flush(int itf, char c)
{
	tud_cdc_n_write_char(itf, c);
	tud_cdc_n_write_flush(itf);
}

// flushes the CDC port
void cdc_flush(int itf)
{
    tud_cdc_n_write_flush(itf);
}

bool cdc_connected(int itf)
{
    return tud_cdc_n_connected(itf);
}

void usbd_init()
{
    usbd_serial_init();
}


// functions to read from a CDC port
// returns -1 if no byte available
int cdc_read_byte(int itf)
{
    char c;
    
    if (tud_cdc_n_available(itf))               // return number of bytes available
    {
        tud_cdc_n_read(itf, &c, 1);
        return c;
    }
    else
    {
        return -1;
    }
}

// read len bytes from the CDC buffer
// the function returns the number of bytes actually read
int cdc_read_buf(int itf, char* buffer, int len)
{
    return tud_cdc_n_read(itf, buffer, len);
}


// function to return number of bytes available in the CDC buffer
uint32_t cdc_available(int itf)
{
    return tud_cdc_n_available(itf);
}

