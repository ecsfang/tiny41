////////////////////////////////////////////////////////////////////////////////
//
// Serial interface
//
////////////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/multicore.h"
#include "tiny41.h"
#include "serial.h"
#include "core_bus.h"

int pcount = 0;
int periodic_read = 0;

// Memory that emulates a pak
typedef unsigned char BYTE;
typedef void (*FPTR)(void);
typedef void (*CMD_FPTR)(char *cmd);

// Serial loop command structure
typedef struct
{
  char key;
  const char *desc;
  FPTR fn;
} SERIAL_COMMAND;

void serial_help(void);
void toggle_trace(void)
{
  bTrace = !bTrace;
  printf("Turn trace %s\n", bTrace ? "on":"off");
}
void toggle_disasm(void)
{
  bDisasm = !bDisasm;
  printf("Turn disassemble %s\n", bDisasm ? "on":"off");
}
#define BAR() printf("+------+------+------+-----+\n")

void list_modules(void)
{
  extern Module_t modules[LAST_PAGE + 1];

  printf("\nLoaded modules\n");
  BAR();
  printf("| Page | Port | XROM | RAM |\n");
  BAR();
  for(int i=FIRST_PAGE; i<=LAST_PAGE; i++) {
    printf("|  #%X  | ", i);
    if( i>=8)
      printf("%d %s | ", (i-6)/2, i&1 ? "Hi":"Lo");
    else 
      printf("     | ");

    if( modules[i].flags & IMG_INSERTED ) {
      printf(" %02X  | ", modules[i].image[0]);
      if( modules[i].flags & IMG_RAM )
        printf("yes |");
      else
        printf("    |");
    } else {
      printf(" --  |     |");
    }
    printf("\n");
  }
  BAR();
}

extern void list_brks(void);
extern void clrAllBrk(void);
extern void power_on(void);
extern void wand_on(void);

static uint16_t sBrk = 0;
static int nBrk = 0;

typedef enum {
  BRK_NONE,
  BRK_START,
  BRK_END,
  BRK_CLR
} BrkMode_e;

BrkMode_e brk_mode = BRK_NONE;

static void sbrkpt(BrkMode_e b)
{
  brk_mode = b;
  switch(b) {
  case BRK_START: printf("\nStart"); break;
  case BRK_END:   printf("\nEnd"); break;
  case BRK_CLR:   printf("\nClear"); break;
  }
  printf(" breakpoint: ----\b\b\b\b");
  sBrk = 0;
  nBrk = 1;
}
void set_brk(void)
{
  sbrkpt(BRK_START);
}
void end_brk(void)
{
  sbrkpt(BRK_END);
}
void clr_brk(void)
{
  sbrkpt(BRK_CLR);
}


extern void clrBrk(uint16_t addr);
extern void setBrk(uint16_t addr);
extern void stopBrk(uint16_t addr);

SERIAL_COMMAND serial_cmds[] = {
  {
    'h',
    "Serial command help",
    serial_help,
  },
  {
    '?',
    "Serial command help",
    serial_help,
  },
  {
    'd',
    "Toggle disassembler",
    toggle_disasm,
  },
  {
    't',
    "Toggle trace",
    toggle_trace,
  },
  {
    'b',
    "List breakpoints",
    list_brks,
  },
  {
    'B',
    "Set breakpoint",
    set_brk,
  },
  {
    'E',
    "End breakpoint",
    end_brk,
  },
  {
    'C',
    "Clear breakpoint",
    clr_brk,
  },
  {
    'x',
    "Clear breakpoints",
    clrAllBrk,
  },
  {
    'l',
    "List modules",
    list_modules,
  },
  {
    'o',
    "Power On",
    power_on,
  },
  {
    'w',
    "Wand On",
    wand_on,
  },
};

void serial_help(void)
{
  for (int i = 0; i < sizeof(serial_cmds) / sizeof(SERIAL_COMMAND); i++)
  {
    printf("\n%c: %s", serial_cmds[i].key, serial_cmds[i].desc);
  }
  printf("\n");
}
void serial_loop(void)
{
  int key;
  int periodic_key = 0;

  if (periodic_read)
  {
    pcount++;
  }

  if (pcount >= 500)
  {
    periodic_key = '0';
    pcount = 0;
  }

  if (((key = getchar_timeout_us(1000)) != PICO_ERROR_TIMEOUT) || (periodic_key != 0))
  {
    if (periodic_key != 0)
    {
      key = periodic_key;
      periodic_key = 0;
    }

    if( nBrk && ((key >= '0' && key <= '9') || (key >= 'a' && key <= 'f'))) {
      int x = key - '0';
      if( key >= 'a' )
        x = key -'a' + 0xa;
      sBrk <<= 4;
      sBrk |= x;
      nBrk++;
      printf("%X", x);
      if( nBrk > 4 ) {
        switch(brk_mode) {
        case BRK_START:
          printf("\rStart breakpoint @ %04X\n", sBrk);
          setBrk(sBrk);
          break;
        case BRK_END:
          printf("\rEnd breakpoint @ %04X\n", sBrk);
          stopBrk(sBrk);
          break;
        case BRK_CLR:
          printf("\rClear breakpoint @ %04X\n", sBrk);
          clrBrk(sBrk);
          break;
        }
        brk_mode = BRK_NONE;
        nBrk = 0;
        sBrk = 0;
      }
    } else {
      for (int i = 0; i < sizeof(serial_cmds) / sizeof(SERIAL_COMMAND); i++)
      {
        if (serial_cmds[i].key == key)
        {
          (*serial_cmds[i].fn)();
          break;
        }
      }
    }
  }
  else
  {
    // I have found that I need to send something if the serial USB times out
    // otherwise I get lockups on the serial communications.
    // So, if we get a timeout we send a spoace and backspace it. And
    // flush the stdio, but that didn't fix the problem but seems like a good idea.
    stdio_flush();
    printf(" \b");
  }
}