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

extern CModules modules;
extern CBreakpoint brk;

// Memory that emulates a pak
typedef unsigned char BYTE;
typedef void (*FPTR)(void);
typedef void (*CMD_FPTR)(char *cmd);

// Serial loop command structure
typedef struct
{
  char key;
  FPTR fn;
  const char *desc;
} SERIAL_COMMAND;

void serial_help(void);
// Toggle over-all trace (off -> no trace at all)
void toggle_trace(void)
{
  bTrace ^= TRACE_ON;
  if( bTrace & TRACE_ON ) bTrace |= TRACE_BRK;
  printf("Turn trace %s\n", bTrace & TRACE_ON ? "on":"off");
}
// Toggle disassembler (on -> call disassembler if trace is on)
void toggle_disasm(void)
{
  bTrace ^= TRACE_DISASM;
  printf("Turn disassemble %s\n", IS_DISASM() ? "on":"off");
}
#define BAR() printf("+------+------+------+-----+\n")

void list_modules(void)
{
  extern CModules modules;

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

    if( modules.isInserted(i) ) {
      printf(" %02X  | ", (*modules[i])[0]);
      if( modules.isRam(i) )
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
extern void wand_scan(void);

static uint16_t sBrk = 0;
static uint16_t nBrk = 0;

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

bool bQram = false;
void sel_qram(void)
{
  printf("\nSelect QRAM page: -\b");
  bQram = true;
}

bool bPlug = false;
void plug_unplug(void)
{
  printf("\nSelect page to plug or unplug: -\b");
  bPlug = true;
}

void listBreakpoints(void)
{
  brk.list_brks();
}
void clrBreakpoints(void)
{
  brk.clrAllBrk();
}

extern void reset_bus_buffer(void);

SERIAL_COMMAND serial_cmds[] = {
  { 'h', serial_help,       "Serial command help"  },
  { '?', serial_help,       "Serial command help"  },
  { 'd', toggle_disasm,     "Toggle disassembler"  },
  { 't', toggle_trace,      "Toggle trace"  },
  { 'b', listBreakpoints,   "List breakpoints"  },
  { 'B', set_brk,           "Set breakpoint"  },
  { 'E', end_brk,           "End breakpoint"  },
  { 'C', clr_brk,           "Clear breakpoint"  },
  { 'x', clrBreakpoints,    "Clear all breakpoints"  },
  { 'l', list_modules,      "List modules"  },
  { 'r', reset_bus_buffer,  "Reset trace buffer"  },
  { 'o', power_on,          "Power On"  },
  { 'w', wand_scan,         "Wand scan"  },
  { 'q', sel_qram,          "Select QRAM page"  },
  { 'p', plug_unplug,       "Plug or unplug module"  },
};

const int helpSize = sizeof(serial_cmds) / sizeof(SERIAL_COMMAND);

void serial_help(void)
{
  printf("\nCmd | Description");
  printf("\n----+-----------------");
  for (int i = 0; i < helpSize; i++)
    printf("\n  %c | %s", serial_cmds[i].key, serial_cmds[i].desc);
  printf("\n----+-----------------");
  printf("\n");
}

int getHexKey(int ky)
{
  int x = -1;
  if( ky >= '0' && ky <= '9' )
    x = ky - '0';
  else if( ky >= 'a' && ky <= 'f' ) 
    x = ky - ('a'-0xa);
  else if( ky >= 'A' && ky <= 'F' )
    x = ky - ('A'-0xa);
  return x;
}


void serial_loop(void)
{
  int key;
  int periodic_key = 0;

  if (periodic_read)
    pcount++;

  if (pcount >= 500) {
    periodic_key = '0';
    pcount = 0;
  }

  if (((key = getchar_timeout_us(1000)) != PICO_ERROR_TIMEOUT) || (periodic_key != 0)) {
    if (periodic_key != 0) {
      key = periodic_key;
      periodic_key = 0;
    }
    if( nBrk || bQram || bPlug ) {
      // Input of hex-digit ... ?
      if( key == 0x1b ) { // ESC - abort any key sequence ...
        bQram = bPlug = false;
        nBrk = sBrk = 0;
        printf("\nAbort ...\n");
      } else {
        int x = getHexKey(key);
        if( x >= 0 && x <= 0xF ) {
          CModule *m = modules[x];
          if( m->isLoaded() ) {
            if( bQram ) {
              m->toggleRam();
              printf("\rModule in page #%X marked as %s     \n",
                x, m->isRam() ? "QRAM" : "ROM");
              bQram = false;
            }
            if( bPlug ) {
              m->togglePlug();
              printf("\rModule in page #%X %s           \n",
                x, m->isInserted() ? "inserted":"removed");
              bPlug = false;
            }
          } else {
            printf("\rNo image at page #%X!!               \n", x);
          }
          if( nBrk ) {
            sBrk <<= 4;
            sBrk |= x;
            nBrk++;
            printf("%X", x);
            if( nBrk > 4 ) {
              switch(brk_mode) {
              case BRK_START:
                printf("\rStart breakpoint @ %04X\n", sBrk);
                brk.setBrk(sBrk);
                break;
              case BRK_END:
                printf("\rEnd breakpoint @ %04X\n", sBrk);
                brk.stopBrk(sBrk);
                break;
              case BRK_CLR:
                printf("\rClear breakpoint @ %04X\n", sBrk);
                brk.clrBrk(sBrk);
                break;
              }
              brk_mode = BRK_NONE;
              sBrk = nBrk = 0;
            }
          }
        }
      }
    } else {
      for (int i = 0; i < helpSize; i++) {
        if (serial_cmds[i].key == key) {
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
    // So, if we get a timeout we send a space and backspace it. And
    // flush the stdio, but that didn't fix the problem but seems like a good idea.
    stdio_flush();
    printf(" \b");
  }
}