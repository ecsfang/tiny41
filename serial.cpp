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
#include "pico/flash.h"
#include "tiny41.h"
#include "serial.h"
#include "core_bus.h"
#include "blinky.h"
#include "usb/cdc_helper.h"
#include "hardware/flash.h"
#include "module.h"
#include "xfmem.h"
#include "fltools/fltools.h"

char cbuff[CDC_PRINT_BUFFER_SIZE];

int pcount = 0;
extern CModules modules;
extern CBreakpoint brk;

//extern CMem ram;
//extern unsigned int nMemMods;

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
  cdc_send_string_and_flush(ITF_CONSOLE, cbuff);
}
// Toggle disassembler (on -> call disassembler if trace is on)
void toggle_disasm(void)
{
  bTrace ^= TRACE_DISASM;
  sprintf(cbuff,"Turn disassemble %s\n\r", IS_DISASM() ? "on":"off");
  cdc_send_string_and_flush(ITF_CONSOLE, cbuff);
}

extern bool bET11967;
void service(void)
{
  // Toggle module
  bET11967 = !bET11967;
  sprintf(cbuff,"ET_11967 %sabled.\n\r", bET11967 ? "en":"dis");
  cdc_send_string_and_flush(ITF_CONSOLE, cbuff);
}

#define BAR_T()   cdc_send_console((char*)"/------+------+------+-----+------------------+\n\r")
#define BAR_M()   cdc_send_console((char*)"+------+------+------+-----+------------------+\n\r")
#define BAR_B()   cdc_send_console((char*)"+------+------+------+-----+------------------/\n\r")

extern int mod_info(CFat_t *pFat, char *buf);

extern CFat_t fat;

const char *moduleType(int typ)
{
  switch( typ ) {
  case FL_MOD: return "MOD";
  case FL_RAM: return "RAM";
  case FL_ROM: return "ROM";
  default:     return "???";
  }
}

/*
 * List all modules that are installed in flash
 */
void all_modules(void)
{
  int i, n = 0;
  char typ[5];
  fat.first();
  if( fat.offset() ) {
    cdc_send_console((char*)"\n\rInstalled modules\n\r");
    while( fat.offset() ) {
      n++;
      sprintf(typ, "%c%s", fat.type() == FL_RAM ? '+':' ', moduleType(fat.type()));
      /**switch( fat.type() ) {
      case FL_MOD: strcpy(typ, " MOD"); break;
      case FL_RAM: strcpy(typ, "+RAM"); break;
      case FL_ROM: strcpy(typ, " ROM"); break;
      default:     strcpy(typ, " ???");
      }*/
      i = sprintf(cbuff,"| %3d | %-16.16s |%4.4s | %08X | ",
        n, fat.name(), typ, fat.offset() );
      i += mod_info(&fat, cbuff+i);
      i += sprintf(cbuff+i,"\n\r");
      cdc_send_console(cbuff);
      fat.next();
    }
  } else {
    cdc_send_console((char*)"\n\rNo installed modules!\n\r");
  }
}

/*
 * List all modules that are plugged into the emulator
 */
void list_modules(void)
{
  extern CModules modules;
  int n;

#ifdef DBG_PRINT
  cdc_send_string_and_flush(ITF_TRACE,(char*)"\n\rDump modules\n\r");
  modules.dump();
#endif
  cdc_send_console((char*)"\n\rLoaded modules\n\r");
  BAR_T();
  cdc_send_console((char*)"| Page | Port | XROM | RAM | Name             | Banks\n\r");
  BAR_M();
  for(int i=FIRST_PAGE; i<=LAST_PAGE; i++) {
    n = sprintf(cbuff,"|  #%X  | ", i);
    switch( i ) {
    case 4: n += sprintf(cbuff+n,"Srv  | "); break;
    case 5: n += sprintf(cbuff+n,"Tmr  | "); break;
    case 6: n += sprintf(cbuff+n,"Prt  | "); break;
    case 7: n += sprintf(cbuff+n,"HPIL | "); break;
    default:
      n += sprintf(cbuff+n,"%d %s | ", (i-6)/2, i&1 ? "Hi":"Lo");
    }

    if( modules.isInserted(i) ) {
      if( i == 4 )
        n += sprintf(cbuff+n," --  | ");  // No XROM values in page 4
      else
        n += sprintf(cbuff+n," %02X  | ", (*modules[i])[0]);
      n += sprintf(cbuff+n,"%3.3s%c|", modules.isQROM(i) ? "Yes" : "", modules.isDirty(i)?'*':' ' );
      n += sprintf(cbuff+n," %-16.16s |", modules[i]->getName() );
      if( modules[i]->haveBank(1))
        n += sprintf(cbuff+n," %s", modules[i]->getName(1) );
    } else {
      n += sprintf(cbuff+n,"     |     |");
      n += sprintf(cbuff+n," %16.16s |", "" );
    }
    n += sprintf(cbuff+n,"\n\r" );
    cdc_send_console(cbuff);
  }
  BAR_B();
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
  ST_QROM,
  ST_PLUG,
  ST_BRK,
  ST_MEM,
  ST_INST,  // Get module to install
  ST_DESC,  // Get description of config
  ST_MPLUG,
  ST_RCONF,
  ST_WCONF,
  ST_DELC   // Delete configuration
} State_e;

State_e state = ST_NONE;

static void sbrkpt(BrkMode_e b)
{
  brk_mode = b;
  switch(b) {
  case BRK_START: cdc_send_console((char*)"\n\rStart"); break;
  case BRK_END:   cdc_send_console((char*)"\n\rEnd"); break;
  case BRK_NONE:   cdc_send_console((char*)"\n\rClear"); break;
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
  sbrkpt(BRK_NONE);
}

void listBreakpoints(void)
{
  brk.list_brks();
}
void clrBreakpoints(void)
{
  brk.clrAllBrk();
}


void sel_qrom(void)
{
  cdc_send_console((char*)"\n\rSelect QROM page: -\b");
  cdc_flush_console();
  state = ST_QROM;  // Expect port address
}

void plug_unplug(void)
{
  cdc_send_console((char*)"\n\rSelect page to plug or unplug: -\b");
  cdc_flush_console();
  state = ST_PLUG;  // Expect port address
}

// Local buffer to hold a entered text string
#define TEXT_BUFFER_LEN   64
class CText {
  char m_text[TEXT_BUFFER_LEN];
  int  m_pText = 0;  // Pointer to next character
  int  m_pMax = 0;   // Max length of current string
public:
  CText() {
    init(TEXT_BUFFER_LEN);
  }
// Init buffer stating max length of wanted string
  void init(int len) {
    memset(m_text, 0, TEXT_BUFFER_LEN);
    m_pText = 0;
    m_pMax = len-1;
  }
  // Add character to buffer
  void add(char ch) {
    if( m_pText >= m_pMax ) {
      // Overflow - remove last character
      m_pText = m_pMax-1;
      send2console("\b", false);
    }
    m_text[m_pText++] = ch;
  }
  // Back space - remove last character
  void del(void) {
    if( m_pText ) {
      m_text[--m_pText] = 0;
      send2console("\b \b", false);  // Clean previous content on page ...
      cdc_flush_console();
    }
  }
  char *buf(void) {
    return m_text;
  }
  int len(void) {
    return m_pText;
  }
};

static CText m_text;

extern bool loadModule(const char *mod, int page);

void inst_module(void)
{
  cdc_send_console((char*)"\n\rWhich module to install: -\b");
  cdc_flush_console();
  state = ST_INST;  // Expect port address
  m_text.init(MOD_NAME_LEN);
}
void plug_module(void)
{
  cdc_send_console((char*)"Select page for module: -\b");
  cdc_flush_console();
  state = ST_MPLUG;  // Expect port address
}
int find_module(void)
{
  // Search for the module and install!
  int n = 0;
  if( fat.find(m_text.buf()) ) {
    sprintf(cbuff,"\rFind module: %s ", m_text.buf());
    n = send2console(cbuff, true);  // Clean previous content on page ...
    plug_module();
  } else {
    sprintf(cbuff,"\rModule [%s] not found!", m_text.buf());
    n = send2console(cbuff, true);  // Clean previous content on page ...
    state = ST_NONE;
  }
  return n;
}

void mem_emulate(void)
{
  ram.enable( !ram.isEnabled() );
  if( ram.isEnabled() ) {
    sprintf(cbuff, "\n\r%d memory modules enabled\n\r", ram.modules());
    cdc_send_console(cbuff);
    cdc_flush_console();
  } else {
    cdc_send_console((char*)"\n\rMemory modules disabled\n\r");
    cdc_flush_console();
  }
}

void mem_modules(void)
{
  if( ram.isEnabled() ) {
    cdc_send_console((char*)"\n\rNumber of memory modules: -\b");
    cdc_flush_console();
    state = ST_MEM;  // Expect port address
  } else {
    cdc_send_console((char*)"\n\rMemory modules not emulated!\n\r");
    cdc_flush_console();
  }
}

void dump_blinky(void)
{
  extern CBlinky blinky;
  int i;
  cdc_send_console((char*)"Blinky registers and RAM\n\r=====================================\n\r");
  for(i=0; i<0x08; i++) {
    sprintf(cbuff,"Reg%02d: %014llx (56)\n\r", i, blinky.reg[i]);
    cdc_send_string(ITF_CONSOLE, cbuff);
  }
  for(; i<0x10; i++) {
    sprintf(cbuff,"Reg%02d: %12.12c%02x  (8)\n\r", i, ' ', blinky.reg8[i]);
    cdc_send_string(ITF_CONSOLE, cbuff);
  }
  cdc_send_console((char*)"\n\r\r");
  cdc_flush_console();
}

#ifdef USE_TIME_MODULE
void dump_time(void)
{
  extern CTime mTime;
  int i;
  cdc_send_console((char*)"Time Module registers\n\r=====================================\n\r");
  for(i=0; i<2; i++) {
    sprintf(cbuff,"Reg %c:\n\r", i==0?'A':'B');
    cdc_send_string(ITF_CONSOLE, cbuff);
    sprintf(cbuff,"  Clock:   %014llx (56)\n\r", mTime.reg[i].clock);
    cdc_send_string(ITF_CONSOLE, cbuff);
    sprintf(cbuff,"  Alarm:   %014llx (56)\n\r", mTime.reg[i].alarm);
    cdc_send_string(ITF_CONSOLE, cbuff);
    sprintf(cbuff,"  Scratch: %014llx (56)\n\r", mTime.reg[i].scratch);
    cdc_send_string(ITF_CONSOLE, cbuff);
  }
  sprintf(cbuff,"\n\r  Interval: %05X  (20)\n\r", mTime.interval);
  cdc_send_string(ITF_CONSOLE, cbuff);
  sprintf(cbuff,"  Accuracy: %05X  (13)\n\r", mTime.accuracy);
  cdc_send_string(ITF_CONSOLE, cbuff);
  sprintf(cbuff,"  Status:   %05X  (20)\n\r", mTime.status);
  cdc_send_string(ITF_CONSOLE, cbuff);
  cdc_send_console((char*)"\n\r\r");
  cdc_flush_console();
}
#endif//USE_TIME_MODULE

#ifdef USE_XF_MODULE
extern CXFM xmem;

static void dumpXmem(const char *lbl, int start, int size, int offs)
{
  int n;
  sprintf(cbuff,"%s\n\r=====================================\n\r", lbl);
  cdc_send_console(cbuff);
  for(int i=size-1; i>=0; i--) {
    n = 0;
    if( xmem.m_ram->mem[i+offs] || xmem.m_mem.mem[i+offs] ) {
      n += sprintf(cbuff+n,"Reg %03X: %014llx %014llx", i+start,
          xmem.m_ram->mem[i+offs], xmem.m_mem.mem[i+offs]);
      if( xmem.m_ram->mem[i+offs] != xmem.m_mem.mem[i+offs] )
        n += sprintf(cbuff+n," ***");
      sprintf(cbuff+n,"\n\r");
      cdc_send_console(cbuff);
    }
  }
  cdc_send_console((char*)"\n\r");
}

// Dump any non-zero registers in XMemory
void dump_xmem(void)
{
#ifdef USE_XMEM2
  dumpXmem("XMemory Module 2", XMEM_XM2_START, XMEM_XM2_SIZE, XMEM_XM2_OFFS);
#endif
#ifdef USE_XMEM1
  dumpXmem("XMemory Module 1", XMEM_XM1_START, XMEM_XM1_SIZE, XMEM_XM1_OFFS);
#endif
#ifdef USE_XFUNC
  dumpXmem("XFunction Memory", XMEM_XF_START,  XMEM_XF_SIZE,  XMEM_XF_OFFS);
#endif
  cdc_flush_console();
}

// Clear the whole XMemory
void clr_xmem(void)
{
  cdc_send_console((char*)"Clear XMemory\n\r");
  cdc_flush_console();
  memset((void*)xmem.m_ram->mem,0,xmem.size());
  xmem.saveMem();
}
// Init XMemory, i.e. read latest copy from flash
void init_xmem(void)
{
  cdc_send_console((char*)"Init XMemory\n\r");
  cdc_flush_console();
  initXMem(0);
  cdc_send_console((char*)"Done with Init XMemory\n\r");
  cdc_flush_console();
}
#endif//USE_XF_MODULE

// Put the Pico in bootsel mode
void pico_bootsel()
{
/**
  cdc_send_console((char*)"\n\r************************************************");
  cdc_send_console((char*)"\n\r** RESETTING THE RP2040-TUP to BOOTSEL mode!! **");
  cdc_send_console((char*)"\n\r************************************************\n\r");
  **/
  for(int port=0; port<ITF_MAX; port++) {
    cdc_flush(port);
    while( cdc_read_byte(port) != -1 )
      ;
  }
  set_sys_clock_khz(133000, 1);
  sleep_ms(100);
  // reboots the RP2040, uses the standard LED for activity monitoring
  reset_usb_boot(1<<PICO_DEFAULT_LED_PIN, 0) ;
}

void quit_log()
{
  cdc_send_console((char*)"\n\r Stopping logging\n\r");
  cdc_flush_console();
  cdc_send_string(ITF_TRACE, (char*)"\nQuit\n");
  cdc_flush(ITF_TRACE);
}

void addString(const char *str)
{
  strcpy(cbuff, str);
  cdc_send_string(ITF_CONSOLE, cbuff);
}
void addString(const char *fmt, int val)
{
  sprintf(cbuff, fmt, val);
  cdc_send_string(ITF_CONSOLE, cbuff);
}
void addString(const char *fmt, const char *str)
{
  sprintf(cbuff, fmt, str);
  cdc_send_string(ITF_CONSOLE, cbuff);
}
void addString(const char *fmt, int val, int val2)
{
  sprintf(cbuff, fmt, val, val2);
  cdc_send_string(ITF_CONSOLE, cbuff);
}

void info(void)
{
  addString("\r\nSystem information (core %d)\r\n=========================\r\n", get_core_num());
  addString("Flash offset:  0x%X\r\n", FLASH_START);
  addString("XIP_BASE:      0x%X\r\n", XIP_BASE);
  addString("Sector size:   %6d (0x%X)\r\n", FLASH_SECTOR_SIZE, FLASH_SECTOR_SIZE);
  addString("Page size:     %6d (0x%X)\r\n", FLASH_PAGE_SIZE, FLASH_PAGE_SIZE);
  addString("Total heap:    %6d bytes \r\n", getTotalHeap());
  addString("Free heap:     %6d bytes\r\n", getFreeHeap());
  addString("Trace element: %6d bytes\r\n", sizeof(Bus_t));
  addString("Trace buffer:  %6d bytes (%d entries)\r\n", sizeof(Bus_t)*NUM_BUS_T, NUM_BUS_T);

  addString("\r\nHP41 information\r\n-------------------------\r\n");
#ifdef USE_XF_MODULE
  addString("XMemory:       %6d bytes (%d registers)\r\n",
                  sizeof(uint64_t)*(XMEM_XF_SIZE+XMEM_XM1_SIZE+XMEM_XM2_SIZE),
                  XMEM_XF_SIZE+XMEM_XM1_SIZE+XMEM_XM2_SIZE);
#endif
  if( ram.isEnabled() ) {
    addString("Quad Memory:   %6d bytes (%d registers)\r\n",
                    sizeof(uint64_t)*(MEM_MOD_SIZE)*ram.modules(),
                    MEM_MOD_SIZE*ram.modules());
    if( ram.modules() )
      addString("Memory:        %6d module%c\r\n", ram.modules(), ram.modules() > 1 ? 's':' ');
  } else {
    addString("Quad Memory:   Not enabled\r\n");
  }
  cdc_flush_console();
}

void tag(void) {
  static unsigned int nTag = 0;
  nTag++;
  sprintf(cbuff, "\n\r###  TAG %d ###\n\r\n\r", nTag);
  cdc_send_string_and_flush(ITF_TRACE, cbuff);
  cdc_send_string_and_flush(ITF_CONSOLE, cbuff);
}

extern void reset_bus_buffer(void);
extern void initRoms(void);

void read_config(void)
{
  cdc_send_console((char*)"\n\rSelect config to read [0-9|Clr|List]: -\b");
  cdc_flush_console();
  state = ST_RCONF;  // Expect port address
}

void desc_config(void)
{
  cdc_send_console((char*)"\n\rModule config description: -\b");
  cdc_flush_console();
  state = ST_DESC;  // Get configuration description
  m_text.init(CONF_DESC_LEN);
}

void save_config(void)
{
  cdc_send_console((char*)"\n\rSelect config to save [0-9|Del|List]: -\b");
  cdc_flush_console();
  state = ST_WCONF;  // Expect port address
}

void clr_config(void)
{
  send2console((char*)"\rClear configuration!", true);  // Clean previous content on page ...
  cdc_send_string_and_flush(ITF_CONSOLE, (char*)"\r\n");
  modules.clearAll();
}

void del_config(void)
{
  cdc_send_console((char*)"\n\rSelect config to delete [0-9|List]: -\b");
  cdc_flush_console();
  state = ST_DELC;  // Expect port address
}

void list_config(void)
{
  Config_t *conf = new Config_t;
  //CFat_t *pFat = new CFat_t();
  int n;
  send2console((char*)"\rList all configurations", true);  // Clean previous content on page ...
  cdc_send_string_and_flush(ITF_CONSOLE, (char*)"---+--------------------------\r\n");
  // TBD - List all available saved module configuration
  for(int i=0; i<10; i++) {
    n = sprintf(cbuff," %d | ", i);
    if( readConfig(conf, i) ) {
      n += sprintf(cbuff+n,"%s ", conf->desc);
    } else {
      n += sprintf(cbuff+n,"-");
    }
    n += sprintf(cbuff+n,"\r\n");
    cdc_send_string_and_flush(ITF_CONSOLE, cbuff);
  }
  delete conf;
}

SERIAL_COMMAND serial_cmds[] = {
  { 'h', serial_help,       "Serial command help"  },
  { '?', serial_help,       "Serial command help"  },
  { 'i', info,              "Memory info"  },

  { 'd', toggle_disasm,     "Toggle disassembler"  },
  { 't', toggle_trace,      "Toggle trace"  },
  { 'g', tag,               "Enter trace tag"  },
  { 'Q', quit_log,          "Stop logging"  },

  { 'b', listBreakpoints,   "List breakpoints"  },
  { 'B', set_brk,           "Set breakpoint"  },
  { 'E', end_brk,           "End breakpoint"  },
  { 'C', clr_brk,           "Clear breakpoint"  },
  { 'x', clrBreakpoints,    "Clear all breakpoints"  },
  { 'r', reset_bus_buffer,  "Reset trace buffer"  },
  { 'R', pico_bootsel,      "Put the Pico into bootsel mode"  },
  { 'o', power_on,          "Power On"  },
  { 'w', wand_test,         "Example bar code"  },
  { 'q', sel_qrom,          "Select QROM page"  },
  { 'l', list_modules,      "List modules"  },
  { 'L', all_modules,       "All modules"  },
  { 'p', plug_unplug,       "Plug or unplug module"  },
  { 'P', inst_module,       "Install a module"  },
  { 'M', mem_emulate,       "Enable Memory modules"  },
  { 'm', mem_modules,       "Number of Memory modules"  },
#ifdef USE_XF_MODULE
  { 'f', dump_xmem,         "Dump XMemory"  },
  { 'X', clr_xmem,          "Clear XMemory"  },
  { 'Z', init_xmem,         "Init XMemory"  },
#endif//USE_XF_MODULE
  { 'k', dump_blinky,       "Dump Blinky registers and memory"  },
  { 'S', service,           "Service ROM (11967) FI+DATA" },
#ifdef USE_TIME_MODULE
  { 'T', dump_time,         "Dump Time registers"  },
#endif//USE_TIME_MODULE
  { 'I', list_config,       "List all saved configurations" },
  { 'O', read_config,       "Load module configuration" },
  { 'W', desc_config,       "Save current configuration" },
};

const int helpSize = sizeof(serial_cmds) / sizeof(SERIAL_COMMAND);

void serial_help(void)
{
  sprintf(cbuff,"\n\rCmd | Description\n\r----+------------------------");
  cdc_send_string(ITF_CONSOLE, cbuff);
  for (int i = 0; i < helpSize; i++) {
    sprintf(cbuff,"\n\r  %c | %s", serial_cmds[i].key, serial_cmds[i].desc);
    cdc_send_string(ITF_CONSOLE, cbuff);
  }
  sprintf(cbuff, (char*)"\n\r====+========================\n\r");
  cdc_send_string_and_flush(ITF_CONSOLE, cbuff);
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

int send2console(const char* dispBuf, bool bClear = false)
{
  int n = cdc_send_console((char*)dispBuf);
  if( bClear )
    n += cdc_send_console((char*)"                   \n\r");
  return n;
}

//extern void printLCD(const char *txt, int row);

extern bool bReady;
void serial_loop(void)
{
//  int key = getchar_timeout_us(1000);
//  if( key != PICO_ERROR_TIMEOUT ) {

  int key = cdc_read_byte(ITF_CONSOLE);
  int conChars = 0;

  if( key != -1 ) {
    {
      static unsigned long k=0;
      char kk[16];
      k = (k<<8) | (key&0xFF);
      if( !bReady )
        return;
    }
    if( state != ST_NONE ) {
      // Assume input of some kind.
      if( key == 0x1b ) { // ESC - abort any key sequence ...
        state = ST_NONE;
        nBrk = sBrk = 0;
        conChars += send2console("\n\rAbort!\n\r");
     } else {
        // Input of hex-digit ... ?
        if( state == ST_MPLUG) {
          int x = getHexKey(key);
          if( x >= 0 ) {
            sprintf(cbuff,"\rPlug %s into port %d", m_text.buf(), x);
            conChars += send2console(cbuff, true);  // Clean previous content on page ...
            // Load the module!
            if( loadModule(m_text.buf(), x) )
              sprintf(cbuff,"Success!");
            else
              sprintf(cbuff,"Failed!");
            conChars += send2console(cbuff, true);  // Clean previous content on page ...
            state = ST_NONE;
          }
        } else if( state == ST_INST || state == ST_DESC ) {
          switch(key) {
          case 0x0D: // Return
            if( state == ST_DESC ) {
              // Save the config with description!
              save_config();
            } else {
              conChars += find_module();
            }
            break;
          case 0x08: // Back space
            m_text.del();
            break;
          default:  // Text entered ...
            m_text.add(key);
            sprintf(cbuff,"%c", key);
            conChars += send2console(cbuff, false);  // Clean previous content on page ...
            cdc_flush_console();
          }
        } else {
          int x = getHexKey(key);
          if( x >= 0 ) {
            CModule *m = modules[x];
            switch( state ) {
            case ST_QROM:
              if( m->isLoaded() ) {
                m->toggleRam();
                sprintf(cbuff,"\rModule in page #%X marked as %sROM", x, m->isQROM() ? "Q" : "");
              } else {
                sprintf(cbuff,"\rNo image at page #%X!!", x);
              }
              conChars += send2console(cbuff, true);  // Clean previous content on page ...
              state = ST_NONE;
              break;
            case ST_PLUG:
              if( m->isLoaded() ) {
                m->togglePlug();
                sprintf(cbuff,"\rModule in page #%X %s", x, m->isInserted() ? "inserted":"removed");
              } else {
                sprintf(cbuff,"\rNo image at page #%X!!", x);
              }
              conChars += send2console(cbuff, true);  // Clean previous content on page ...
              state = ST_NONE;
              break;
            case ST_BRK:
              sBrk <<= 4;
              sBrk |= x;
              nBrk++;
              sprintf(cbuff,"%X", x);
              conChars += send2console(cbuff);
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
                case BRK_NONE:
                  sprintf(cbuff,"\rClear breakpoint @ %04X\n\r", sBrk);
                  brk.clrBrk(sBrk);
                  break;
                }
                conChars += send2console(cbuff);
                brk_mode = BRK_NONE;
                sBrk = nBrk = 0;
                state = ST_NONE;
              }
              break;
            case ST_MEM:
              if( ram.isEnabled() ) {
                if( x >= 0 && x <= 4 ) {
                  ram.modules(x);
                  switch( x ) {
                  case 0:
                    sprintf(cbuff,"\rNo memory modules emulated  ");
                    break;
                  case 1:
                    sprintf(cbuff,"\r1 Memory module emulated   ");
                    break;
                  default:
                    sprintf(cbuff,"\r%d Memory modules emulated  ", x);
                  }
                  ram.modules(x);
                } else {
                  sprintf(cbuff,"\rInvalid number of memory modules - %d!!", x);
                }
                conChars += send2console(cbuff);
                conChars += send2console("\n\r");
              }
              state = ST_NONE;
              break;
            case ST_RCONF:
            case ST_WCONF:
            case ST_DELC:
              if( x >= 0 && x <= 9 ) {
                int n = sprintf(cbuff,"%d\r\n", x);
                switch( state ) {
                case ST_RCONF:
                    // Read configuration
                    sprintf(cbuff+n,"Read configuration\r\n");
                    conChars += send2console(cbuff);
                    modules.readConfig(x);
                    break;
                case ST_WCONF:
                    // Save configuration
                    sprintf(cbuff+n,"Save configuration\r\n");
                    conChars += send2console(cbuff);
                    modules.saveConfig(x, m_text.len()>0?m_text.buf():NULL);
                    break;
                case ST_DELC:
                    // Delete configuration
                    sprintf(cbuff+n,"Delete configuration\r\n");
                    conChars += send2console(cbuff);
                    modules.deleteConfig(x);
                }
                state = ST_NONE;
              } else {
                if( state == ST_RCONF && (key == 'c' || key == 'C') ) {
                  clr_config();
                  state = ST_NONE;
                } else if( state == ST_WCONF && (key == 'd' || key == 'D') ) {
                  del_config();
                } else {
                  sprintf(cbuff,"\rInvalid configuration - %d!!", x);
                  conChars += send2console(cbuff, true);
                  state = ST_NONE;
                }
              }
            }
          } else {
            if( state == ST_RCONF ) {
              if( key == 'c' || key == 'C' ) {
                // Clear config (remove all modules)
                clr_config();
              }
              if( key == 'l' || key == 'L' || key == '?') {
                // List all available configurations
                list_config();
              }
              state = ST_NONE;
            }
            if( state == ST_WCONF ) {
              if( key == 'd' || key == 'D' ) {
                // Delete configuration
                del_config();
              }
              if( key == 'l' || key == 'L' || key == '?') {
                // List all available configurations
                list_config();
                state = ST_NONE;
              } else
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
  // Remember to update the console!
  if( conChars )
    cdc_flush_console();
}
