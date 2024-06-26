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
#include "pico/bootrom.h"
#include "tiny41.h"
#include "serial.h"
#include "core_bus.h"
#include "usb/cdc_helper.h"
#include "hardware/flash.h"
#include "module.h"

#define CON_PRINTF cdc_printf_console
//#define CON_PRINTF printf

char cbuff[CDC_PRINT_BUFFER_SIZE];

/*void xxcdc_printf_console(const char *format, ...) {

    char buffer[CDC_PRINT_BUFFER_SIZE];

   va_list arg_ptr;                                                             
                                                                                
   va_start(arg_ptr, format);                                                      
   vsprintf(buffer, format, arg_ptr);                                              
   va_end(arg_ptr); 

  cdc_send_console(buffer);
}*/

int pcount = 0;
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
  sprintf(cbuff,"Turn trace %s\n\r", bTrace & TRACE_ON ? "on":"off");
  cdc_send_console(cbuff);
}
// Toggle disassembler (on -> call disassembler if trace is on)
void toggle_disasm(void)
{
  bTrace ^= TRACE_DISASM;
  sprintf(cbuff,"Turn disassemble %s\n\r", IS_DISASM() ? "on":"off");
  cdc_send_console(cbuff);
}
#define BAR()   cdc_send_console((char*)"+------+------+------+-----+-----+\n\r")

void list_modules(void)
{
  extern CModules modules;
  int n;

  cdc_send_console((char*)"\n\rLoaded modules\n\r");
  BAR();
  cdc_send_console((char*)"| Page | Port | XROM | RAM | Bank|\n\r");
  BAR();
  for(int i=FIRST_PAGE; i<=LAST_PAGE; i++) {
    n = sprintf(cbuff,"|  #%X  | ", i);
    if( i>=8)
      n += sprintf(cbuff+n,"%d %s | ", (i-6)/2, i&1 ? "Hi":"Lo");
    else 
      n += sprintf(cbuff+n,"     | ");

    if( modules.isInserted(i) ) {
      n += sprintf(cbuff+n," %02X  | ", (*modules[i])[0]);
      n += sprintf(cbuff+n,"%3.3s |", modules.isRam(i) ? "Yes" : "" );
    } else {
      n += sprintf(cbuff+n," --  |     |");
    }
    n += sprintf(cbuff+n," %3.3s |\n\r", modules[i]->haveBank() ? "Yes" : "" );
    cdc_send_console(cbuff);
  }
  BAR();
}

extern void list_brks(void);
extern void clrAllBrk(void);
extern void power_on(void);
extern void wand_data(uint8_t *);

uint8_t barTest[] = {
  //Hello world!
  0x0E, 0xdd, 0x7c, 0x48, 0x65, 0x6c, 0x6c, 0x6f, 0x20, 0x77, 0x6f, 0x72, 0x6c, 0x64, 0x21
  //0x05, 0xFC, 0x6A, 0x6B, 0x14, 0x13
  //0x02, 0xE0, 0x86
};

void wand_test(void)
{
  cdc_send_console((char*)"Send barcode!\n\r");
  wand_data(barTest);
}

static uint16_t sBrk = 0;
static uint16_t nBrk = 0;

BrkMode_e brk_mode = BRK_NONE;

typedef enum {
  ST_NONE,
  ST_QRAM,
  ST_PLUG,
  ST_BRK
} State_e;

State_e state = ST_NONE;

static void sbrkpt(BrkMode_e b)
{
  brk_mode = b;
  switch(b) {
  case BRK_START: cdc_send_console((char*)"\n\rStart"); break;
  case BRK_END:   cdc_send_console((char*)"\n\rEnd"); break;
  case BRK_CLR:   cdc_send_console((char*)"\n\rClear"); break;
  }
  cdc_send_console((char*)" breakpoint: ----\b\b\b\b");
  cdc_flush_console();
  sBrk = 0;
  nBrk = 1;
  state = ST_BRK; // Expect 4 nibble addres
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

void sel_qram(void)
{
  cdc_send_console((char*)"\n\rSelect QRAM page: -\b");
  cdc_flush_console();
  state = ST_QRAM;  // Expect port address
}

void plug_unplug(void)
{
  cdc_send_console((char*)"\n\rSelect page to plug or unplug: -\b");
  cdc_flush_console();
  state = ST_PLUG;  // Expect port address
}

void listBreakpoints(void)
{
  brk.list_brks();
}
void clrBreakpoints(void)
{
  brk.clrAllBrk();
}

//extern volatile Blinky_t blinky;

void dump_blinky(void)
{
  extern CBlinky blinky;
  int i;
  cdc_send_console((char*)"Blinky registers and RAM\n\r=====================================\n\r");
  for(i=0; i<0x08; i++) {
    sprintf(cbuff,"Reg%02d: %014llx (56)\n\r", i, blinky.reg[i]);
    cdc_send_console(cbuff);
  }
  for(; i<0x10; i++) {
    sprintf(cbuff,"Reg%02d: %12.12c%02x  (8)\n\r", i, ' ', blinky.reg8[i]);
    cdc_send_console(cbuff);
  }
  cdc_send_console((char*)"\n\r\r");
}

#ifdef USE_TIME_MODULE
void dump_time(void)
{
  extern CTime mTime;
  int i;
  cdc_send_console((char*)"Time Module registers\n\r=====================================\n\r");
  for(i=0; i<2; i++) {
    sprintf(cbuff,"Reg %c:\n\r", i==0?'A':'B');
    cdc_send_console(cbuff);
    sprintf(cbuff,"  Clock:   %014llx (56)\n\r", mTime.reg[i].clock);
    cdc_send_console(cbuff);
    sprintf(cbuff,"  Alarm:   %014llx (56)\n\r", mTime.reg[i].alarm);
    cdc_send_console(cbuff);
    sprintf(cbuff,"  Scratch: %014llx (56)\n\r", mTime.reg[i].scratch);
    cdc_send_console(cbuff);
  }
  sprintf(cbuff,"\n\r  Interval: %05X  (20)\n\r", mTime.interval);
  cdc_send_console(cbuff);
  sprintf(cbuff,"  Accuracy: %05X  (13)\n\r", mTime.accuracy);
  cdc_send_console(cbuff);
  sprintf(cbuff,"  Status:   %05X  (20)\n\r", mTime.status);
  cdc_send_console(cbuff);
  cdc_send_console((char*)"\n\r\r");
}
#endif//USE_TIME_MODULE

#ifdef USE_XF_MODULE
extern CXFM xmem;

void dump_xmem(void)
{
  int n;
#ifdef USE_XMEM2

  cdc_send_console((char*)"XMemory Module 2\n\r=====================================\n\r");
  for(int i=XMEM_XM2_SIZE-1; i>=0; i--) {
    n = 0;
    if( xmem.mem[i+XMEM_XM2_OFFS] || xmem.m_mem[i+XMEM_XM2_OFFS] ) {
      n += sprintf(cbuff+n,"Reg %03X: %014llx %014llx", i+XMEM_XM2_START,
          xmem.mem[i+XMEM_XM2_OFFS], xmem.m_mem[i+XMEM_XM2_OFFS]);
      if( xmem.mem[i+XMEM_XM2_OFFS] != xmem.m_mem[i+XMEM_XM2_OFFS] )
        n += sprintf(cbuff+n," ***");
      sprintf(cbuff+n,"\n\r");
      cdc_send_console(cbuff);
    }
  }
  cdc_send_console((char*)"\n\r");
#endif
#ifdef USE_XMEM1
  sprintf(cbuff,"XMemory Module 1\n\r=====================================\n\r");
  for(int i=XMEM_XM1_SIZE-1; i>=0; i--) {
    n = 0;
    if( xmem.mem[i+XMEM_XM1_OFFS] || xmem.m_mem[i+XMEM_XM1_OFFS] ) {
      n += sprintf(cbuff+n,"Reg %03X: %014llx %014llx", i+XMEM_XM1_START,
          xmem.mem[i+XMEM_XM1_OFFS], xmem.m_mem[i+XMEM_XM1_OFFS]);
      if( xmem.mem[i+XMEM_XM1_OFFS] != xmem.m_mem[i+XMEM_XM1_OFFS] )
        n += sprintf(cbuff+n," ***");
      n += sprintf(cbuff+n,"\n\r");
      cdc_send_console(cbuff);
    }
  }
  cdc_send_console((char*)"\n\r");
#endif
#ifdef USE_XFUNC
  sprintf(cbuff,"XFunction Memory\n\r=====================================\n\r");
  for(int i=XMEM_XF_SIZE-1; i>=0; i--) {
    n = 0;
    if( xmem.mem[i] || xmem.m_mem[i] ) {
      n += sprintf(cbuff+n,"Reg %03X: %014llx %014llx", i+XMEM_XF_START,
          xmem.mem[i], xmem.m_mem[i]);
      if( xmem.mem[i] != xmem.m_mem[i] )
        n += sprintf(cbuff+n," ***");
      n += sprintf(cbuff+n,"\n\r");
      cdc_send_console(cbuff);
    }
  }
  cdc_send_console((char*)"\n\r");
#endif
}
void clr_xmem(void)
{
  cdc_send_console((char*)"Clear XMemory\n\r");
  memset((void*)xmem.mem,0,xmem.size());
  xmem.saveMem();
}
#endif//USE_XF_MODULE

void rp2040_bootsel()
{
  cdc_send_console((char*)"\n\r RESETTING THE RP2040-TUP to BOOTSEL mode!!\n\r");
  cdc_flush_console();
  sleep_ms(1000);

  // reboots the RP2040, uses the standard LED for activity monitoring
  reset_usb_boot(1<<PICO_DEFAULT_LED_PIN, 0) ;
}

void quit_log()
{
  cdc_send_console((char*)"\n\r Stopping logging\n\r");
  cdc_flush_console();
  cdc_send_string(ITF_TRACE, (char*)"\nQuit\n", 6);
  cdc_flush(ITF_TRACE);
}

void info(void)
{
  sprintf(cbuff, "Flash offset: 0x%X\r\n", FLASH_TARGET_OFFSET);
  cdc_send_console(cbuff);
  sprintf(cbuff, "XIP_BASE:     0x%X\r\n", XIP_BASE);
  cdc_send_console(cbuff);
  sprintf(cbuff, "Sector size:  0x%X (%d)\r\n", FLASH_SECTOR_SIZE, FLASH_SECTOR_SIZE);
  cdc_send_console(cbuff);
  sprintf(cbuff, "Page size:    0x%X (%d)\r\n", FLASH_PAGE_SIZE, FLASH_PAGE_SIZE);
  cdc_send_console(cbuff);
  sprintf(cbuff, "%d bytes total heap\r\n", getTotalHeap());
  cdc_send_console(cbuff);
  sprintf(cbuff, "%d bytes free heap\r\n", getFreeHeap());
  cdc_send_console(cbuff);
  sprintf(cbuff, "Trace element: %d bytes\r\n", sizeof(Bus_t));
  cdc_send_console(cbuff);
  sprintf(cbuff, "Trace buffer: %d bytes\r\n", sizeof(Bus_t)*NUM_BUS_T);
  cdc_send_console(cbuff);
}

void tag(void) {
  cdc_printf_(ITF_TRACE, "\n\r###  TAG ###\n\r\n\r");
  cdc_flush(ITF_TRACE);
}

extern void reset_bus_buffer(void);

SERIAL_COMMAND serial_cmds[] = {
  { 'h', serial_help,       "Serial command help"  },
  { '?', serial_help,       "Serial command help"  },
  { 'i', info,              "Memory info"  },
  { 'd', toggle_disasm,     "Toggle disassembler"  },
  { 't', toggle_trace,      "Toggle trace"  },
  { 'b', listBreakpoints,   "List breakpoints"  },
  { 'B', set_brk,           "Set breakpoint"  },
  { 'E', end_brk,           "End breakpoint"  },
#ifdef USE_XF_MODULE
  { 'f', dump_xmem,         "Dump XMemory"  },
  { 'X', clr_xmem,          "Clear XMemory"  },
#endif//USE_XF_MODULE
  { 'C', clr_brk,           "Clear breakpoint"  },
  { 'x', clrBreakpoints,    "Clear all breakpoints"  },
  { 'l', list_modules,      "List modules"  },
  { 'r', reset_bus_buffer,  "Reset trace buffer"  },
  { 'R', rp2040_bootsel,    "Put into bootsel mode"  },
  { 'o', power_on,          "Power On"  },
  { 'w', wand_test,         "Example bar code"  },
  { 'q', sel_qram,          "Select QRAM page"  },
  { 'Q', quit_log,          "Stop logging"  },
  { 'p', plug_unplug,       "Plug or unplug module"  },
  { 'k', dump_blinky,       "Dump Blinky registers and memory"  },
  { 'g', tag,               "Enter trace tag"  },
#ifdef USE_TIME_MODULE
  { 'T', dump_time,         "Dump Time registers"  },
#endif//USE_TIME_MODULE
};

const int helpSize = sizeof(serial_cmds) / sizeof(SERIAL_COMMAND);

void serial_help(void)
{
  int n = sprintf(cbuff,"\n\rCmd | Description");
  n += sprintf(cbuff+n,"\n\r----+-----------------");
  cdc_send_console(cbuff);
  for (int i = 0; i < helpSize; i++) {
    sprintf(cbuff,"\n\r  %c | %s", serial_cmds[i].key, serial_cmds[i].desc);
    cdc_send_console(cbuff);
  }
  cdc_send_console((char*)"\n\r----+-----------------\n\r");
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
  return x; // Value of hex or -1
}

extern bool bReady;
void serial_loop(void)
{
//  int key = getchar_timeout_us(1000);
//  if( key != PICO_ERROR_TIMEOUT ) {

  int key = cdc_read_byte(ITF_CONSOLE);

  if( key != -1 ) {
    {
      static unsigned long k=0;
      char kk[16];
      extern CDisplay    disp41;
      k = (k<<8) | (key&0xFF);
      sprintf(kk, "[%08lX] ", k);
      WriteString(disp41.buf(), 0, STATUS1_ROW, kk);
      disp41.rend(REND_STATUS);
      if( !bReady )
        return;
    }
    if( state != ST_NONE ) {
      // Assume input of some kind.
      if( key == 0x1b ) { // ESC - abort any key sequence ...
        state = ST_NONE;
        nBrk = sBrk = 0;
        cdc_send_console((char*)"\n\rAbort!\n\r");
      } else {
        // Input of hex-digit ... ?
        int x = getHexKey(key);
        if( x >= 0 ) {
          CModule *m = modules[x];
          switch( state ) {
          case ST_QRAM:
            if( m->isLoaded() ) {
              m->toggleRam();
              sprintf(cbuff,"\rModule in page #%X marked as %s", x, m->isRam() ? "QRAM" : "ROM");
            } else {
              sprintf(cbuff,"\rNo image at page #%X!!", x);
            }
            cdc_send_console(cbuff);
            cdc_send_console((char*)"              \n\r");  // Clean previous content on page ...
            state = ST_NONE;
            break;
          case ST_PLUG:
            if( m->isLoaded() ) {
              m->togglePlug();
              sprintf(cbuff,"\rModule in page #%X %s", x, m->isInserted() ? "inserted":"removed");
            } else {
              sprintf(cbuff,"\rNo image at page #%X!!", x);
            }
            cdc_send_console(cbuff);
            cdc_send_console((char*)"              \n\r");  // Clean previous content on page ...
            state = ST_NONE;
            break;
          case ST_BRK:
            sBrk <<= 4;
            sBrk |= x;
            nBrk++;
            sprintf(cbuff,"%X", x);
            cdc_send_console(cbuff);
            if( nBrk > 4 ) {
              switch(brk_mode) {
              case BRK_START:
                sprintf(cbuff,"\rStart breakpoint @ %04X\n\r", sBrk);
                brk.setBrk(sBrk);
                break;
              case BRK_END:
                sprintf(cbuff,"\rEnd breakpoint @ %04X\n\r", sBrk);
                brk.stopBrk(sBrk);
                break;
              case BRK_CLR:
                sprintf(cbuff,"\rClear breakpoint @ %04X\n\r", sBrk);
                brk.clrBrk(sBrk);
                break;
              }
              cdc_send_console(cbuff);
              brk_mode = BRK_NONE;
              sBrk = nBrk = 0;
              state = ST_NONE;
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
    //cdc_flush_console();
    //stdio_flush();
//    sprintf(cbuff," \b");
  }
}