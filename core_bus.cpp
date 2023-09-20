#include <stdio.h>
#include <string.h>
#include <time.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/multicore.h"
#include "tiny41.h"
#include "core_bus.h"
#include "disasm.h"
#include "serial.h"
// #include "disstr.h"
#include <malloc.h>
#ifdef USE_FLASH
#include "hardware/flash.h"
#endif

//#define RESET_FLASH
//#define RESET_RAM
#define DRIVE_CARRY
#define DRIVE_ISA

// We're going to erase and reprogram a region 256k from the start of flash.
// Once done, we can access this at XIP_BASE + 256k.
#define FLASH_TARGET_OFFSET (512 * 1024)

uint32_t getTotalHeap(void)
{
  extern char __StackLimit, __bss_end__;
  return &__StackLimit - &__bss_end__;
}

uint32_t getFreeHeap(void)
{
  struct mallinfo m = mallinfo();
  return getTotalHeap() - m.uordblks;
}

clock_t clock()
{
    return (clock_t) time_us_64() / 10000;
}

inline uint16_t swap16(uint16_t b)
{
  return __builtin_bswap16(b);
}

#define BRK_SIZE (0x10000/(32/2)) // 2 bits per brkpt
uint32_t brkpt[BRK_SIZE];
enum {
  BRK_NONE,
  BRK_START,
  BRK_STOP,
};
#define BRK_MASK(a,w) ((brkpt[a>>4]w >> ((a & 0xF)<<1)) & 0x3)
#define BRK_SHFT(a) ((a & 0xF)<<1)
#define BRK_WORD(a) (a >> 4)

// Check if breakpoint is set for given address
inline int isBrk(uint16_t addr)
{
  return (brkpt[BRK_WORD(addr)] >> BRK_SHFT(addr)) & 0b11;
}
// Clear breakpoint on given address
void clrBrk(uint16_t addr)
{
  brkpt[BRK_WORD(addr)] &= ~(0b11 << BRK_SHFT(addr));
}
void clrAllBrk(void)
{
  for(int a=0; a<BRK_SIZE; a++)
    brkpt[a] = 0x0;
  printf("Clear all breakpoints\n");
}
// Set breakpoint on given address
void setBrk(uint16_t addr)
{
  clrBrk(addr);
  brkpt[BRK_WORD(addr)] |= BRK_START << BRK_SHFT(addr);
}
void stopBrk(uint16_t addr)
{
  clrBrk(addr);
  brkpt[BRK_WORD(addr)] |= BRK_STOP << BRK_SHFT(addr);
}
void list_brks(void)
{
  int n = 0;
  for(int w=0; w<BRK_SIZE; w++) {
    if( brkpt[w] ) {
      for(int a=w*16; a<(w+1)*16; a++) {
        int br = isBrk(a);
        if( br ) {
          printf("#%d: %04X -> ", ++n, a);
          switch( br ) {
            case 1: printf("start"); break;
            case 2: printf("stop"); break;
            default: printf("???"); break;
          }
          printf("\n");
        }
      }
    }
  }
  if( !n )
    printf("No breakpoints\n");
}
void swapRam(uint16_t *dta, int n);

void power_on(void)
{
  printf("Try to power on the calculator ...\n");
  gpio_put(P_ISA_OE, 0); // Enable ISA driver
  // Expose the next bit on the ISA line ...
  gpio_put(P_ISA_DRV, 1);
  sleep_ms(1);
  gpio_put(P_ISA_DRV, 0);
  gpio_put(P_ISA_OE, 1); // Disable ISA driver
}

volatile int carry_fi_t0 = 0;

void fi(int flag)
{
  carry_fi_t0 = 1;
}

void wand_on(void)
{
  printf("Simulate Wand to turn on the calculator ...\n");
  gpio_put(P_ISA_OE, 0); // Enable ISA driver
  // Expose the next bit on the ISA line ...
  gpio_put(P_ISA_DRV, 1);
  fi(FI_PBSY);
  sleep_ms(1);
  gpio_put(P_ISA_DRV, 0);
  gpio_put(P_ISA_OE, 1); // Disable ISA driver
}

extern const char *inst50disp[16];
extern const char *inst70disp[16];

#define PAGE1(n) (FLASH_TARGET_OFFSET + 2 * n * FLASH_SECTOR_SIZE)
#define PAGE2(n) (FLASH_TARGET_OFFSET + (2 * n + 1) * FLASH_SECTOR_SIZE)

#ifdef USE_FLASH
void erasePort(int n)
{
  printf("\nErasing target region %X ", n);
  printf("-- [%05X - %05X]\n", PAGE1(n), PAGE2(n) + FLASH_SECTOR_SIZE);
  uint32_t ints = save_and_disable_interrupts();
  flash_range_erase(PAGE1(n), FLASH_SECTOR_SIZE);
  flash_range_erase(PAGE2(n), FLASH_SECTOR_SIZE);
  restore_interrupts(ints); // Note that a whole number of sectors must be erased at a time.
  printf("Done!\n");
}

void writePort(int n, uint16_t *data)
{
  // Swap 16-bit word to get right endian for flash ...
  swapRam(data, FLASH_SECTOR_SIZE);
  uint8_t *dp = (uint8_t*)data;
  printf("\nProgramming target region %X ", n);
  printf("-- [%05X - %05X]\n", PAGE1(n), PAGE2(n) + FLASH_SECTOR_SIZE);
  uint32_t ints = save_and_disable_interrupts();
  flash_range_program(PAGE1(n), dp, FLASH_SECTOR_SIZE);
  flash_range_program(PAGE2(n), dp + FLASH_SECTOR_SIZE, FLASH_SECTOR_SIZE);
  restore_interrupts(ints); // Note that a whole number of sectors must be erased at a time.
  // Swap back ...
  swapRam(data, FLASH_SECTOR_SIZE);
  printf("Done!\n");
}

// Read flash into ram with right endian
void readPort(int n, uint16_t *data)
{
  // Point at the wanted flash image ...
  const uint16_t *fp = (uint16_t*)(XIP_BASE + PAGE1(n));
  for (int i = 0; i < FLASH_SECTOR_SIZE; ++i) {
    // Swap order to get right endian of 16-bit word ...
    data[i] = swap16(*fp++);
  }
}
#endif


// Allocate memory for all flash images ...
static uint16_t rom_pages[NR_PAGES - FIRST_PAGE][PAGE_SIZE];

uint16_t readRom(int a) {
  if( PAGE(a) >= FIRST_PAGE )
    return rom_pages[PAGE(a)-FIRST_PAGE][a&PAGE_MASK];
  return 0;
}

// Make space for information about all flash images
Module_t modules[NR_PAGES];
void addRom(int port, uint16_t *image)
{
	if( image ) {
    //modules[port].start = port * PAGE_SIZE;
    modules[port].image = image;
    modules[port].flags = IMG_INSERTED;
    //printf("Add ROM @ %04X - %04X\n", modules[port].start, modules[port].start|PAGE_MASK);
    printf("Add ROM @ %04X - %04X\n", port * PAGE_SIZE, (port * PAGE_SIZE)|PAGE_MASK);
	} else {
//    modules[port].start = 0;
    modules[port].image = NULL;
    modules[port].flags = IMG_NONE;
	}
}

#ifdef USE_FLASH
//int dirty = 0;
void saveRam(int port, int ovr = 0)
{
  if (ovr | (modules[port].flags & IMG_DIRTY)) {
    modules[port].flags &= ~IMG_DIRTY;
    erasePort(port);
    writePort(port, rom_pages[port - FIRST_PAGE]);
    printf("Wrote RAM[%C] to flash\n", port);
  }
}
#endif

// Swap 16-bit word (n - number of 16-bit words)
void swapRam(uint16_t *dta, int n)
{
  for (int i = 0; i < n; i++) {
    dta[i] = swap16(dta[i]);
  }
}

void initRoms()
{
#ifdef USE_FLASH
  printf("Flash offset: 0x%X\n", FLASH_TARGET_OFFSET);
  printf("Sector size:  0x%X (%d)\n", FLASH_SECTOR_SIZE, FLASH_SECTOR_SIZE);
  printf("Page size:    0x%X (%d)\n", FLASH_PAGE_SIZE, FLASH_PAGE_SIZE);
#endif
  printf("%d bytes total heap\n", getTotalHeap());
  printf("%d bytes free heap\n", getFreeHeap());
#ifdef RESET_RAM
  for (int i = 0; i < RAM_SIZE; i++)
    rom_pages[0xC - FIRST_PAGE][i] = 0x0000;
  printf("Clear MLDL RAM-page ...\n");
  saveRam(0xC, 1);
#endif
#ifdef RESET_FLASH
  for (int p = 0; p < 16; p++)
    erasePort(p);
#endif
  // Remove all modules ...
  for (int p = 0; p <= LAST_PAGE; p++)
		addRom(p, NULL);

  // Check for existing images and load them ...
  for (int port = FIRST_PAGE; port <= LAST_PAGE; port++) {
    int pIdx = port - FIRST_PAGE;
    // Read flash image
    readPort(port, rom_pages[pIdx]);
		int nErr = 0; // Nr of errors
		int nClr = 0; // Nr of erased words
    // Check that image is valid (10 bits)
		for(int i=0; i<PAGE_SIZE; i++) {
			if( rom_pages[pIdx][i] > 0x3FF ) {
				nErr++;
        if( rom_pages[pIdx][i] == 0xFFFF )
          nClr++;
      }
    }
#if 1
    // Verify loaded code
    if( nClr == PAGE_SIZE ) {
      printf("%04X -> Image is cleared!", port*PAGE_SIZE);
    } else {
      if( nErr ) printf("%d errors in image 0x%X!\n", nErr, port);
      printf("%04X ->", port*PAGE_SIZE);
      for(int i=0; i<8; i++)
        printf(" %03X", rom_pages[pIdx][0x000+i]);
      printf(" ...", port*PAGE_SIZE+0xFF0);
      for(int i=0; i<8; i++)
        printf(" %03X", rom_pages[pIdx][0xff8+i]);
    }
    printf("\n");
#endif
    if( !nErr ) {
      addRom(port, rom_pages[pIdx]);
    }
  }
  // TBD - Now RAM-page is hardcoded to port C
  int port = 0xC;
  if (modules[port].flags & IMG_INSERTED) {
    modules[port].flags |= IMG_RAM;
    printf("Add RAM[%X] @ %04X - %04X\n", port, port*PAGE_SIZE, (port*PAGE_SIZE)|PAGE_MASK);
  }
}

char *disAsm(int inst, int addr, uint64_t data);

#define CF_DUMP_DBG_DREGS 0
#define CF_DISPLAY_LCD 1
#define CF_DISPLAY_OLED 0
#define CF_DBG_DISP_ON 0
#define CF_DBG_SLCT 0
#define CF_DBG_DISP_INST 0
#define CF_USE_DISP_ON 0
#define CF_DBG_KEY 1

#define CH9(c) (c & 0x1FF)
#define CH9_B03(c) ((c & 0x00F) >> 0) // Bit 0-3
#define CH9_B47(c) ((c & 0x0F0) >> 4) // Bit 4-7
#define CH9_B8(c) ((c & 0x100) >> 8)  // Bit 9

////////////////////////////////////////////////////////////////////////////////
//
// Core 1 sits in a loop grabbing bus cycles
//

#define RISING_EDGE(SIGNAL) ((last_##SIGNAL == 0) && (SIGNAL == 1))
#define FALLING_EDGE(SIGNAL) ((last_##SIGNAL == 1) && (SIGNAL == 0))

// Do we drive ROM data?
int drive_data_flag = 0;
int output_isa = 0; // Output ongoing ...

// The data we drive
int drive_data = 0;

// How many times have we driven ISA?
volatile int driven_isa = 0;
volatile int embed_seen = 0;
volatile int sync_count = 0;

void handle_bus(volatile Bus_t *pBus);

////////////////////////////////////////////////////////////////////////////////
//
// Core1 main functions
//
// Various different core 1 main functions used to test functionality
//
////////////////////////////////////////////////////////////////////////////////

// Data transfer to core 0

#ifdef TRACE_ISA
#define NUM_BUS_T 0x400 //Shoudl be power of 2! 3000 // 7000
#else
#define NUM_BUS_T 0x1000 //Shoudl be power of 2! 3000 // 7000
#endif
#define NUM_BUS_MASK (NUM_BUS_T-1)
#define INC_BUS_PTR(d) d = (d+1) & NUM_BUS_MASK

int queue_overflow = 0;

volatile int data_in = 0;
volatile int data_out = 0;

volatile Bus_t bus[NUM_BUS_T];

int last_sync = 0;
int gpio_states = 0;

int peripheral_ce = 0;
char dtext[2 * NR_CHARS + 1];
bool bPunct[2 * NR_CHARS + 1];

char cpu2buf[256];
int nCpu2 = 0;

// void __not_in_flash_func(core1_main_3)(void)
void core1_main_3(void)
{
  static int bIsaEn = 0;
  static int bIsa = 0;
  static int bit_no = 0;
  static uint64_t isa = 0LL;
  static uint64_t bit = 0LL;
  int sync = 0;
  static uint64_t data56 = 0;
  //static uint16_t inst = 0;
  int last_data_in;
  static uint16_t romAddr = 0;
  Module_t *mp = NULL;
  volatile Bus_t *pBus = &bus[data_in];

  irq_set_mask_enabled(0xffffffff, false);

  last_sync = GPIO_PIN(P_SYNC);

  while (1) {
    // Wait for CLK2 to have a falling edge
    WAIT_FALLING(P_CLK2);
    uint64_t tm = time_us_64();

    // NOTE! Bit numbers are out by one as bit_no hasn't been incremented yet.

#ifdef DRIVE_CARRY
    // Do we drive the carry bit on FI line?
    if (carry_fi_t0) {
      switch(bit_no) {
      case LAST_CYCLE:  // Bit 0 - start of T0
        // Enable FI (input tied low)
        gpio_put(P_FI_OE, 0);
        break;
      case (4-1):       // Bit 3 - end of T0
        // Don't drive carry any more ...
        gpio_put(P_FI_OE, 1);
        //carry_fi_t0 = 0;
        break;
      }
    }
#endif
    // Do we drive the ISA line (bit 43-53)?
    if (output_isa) {
      // Drive the ISA line for theses data bit
      switch (bit_no) {
      case 43-1:
        // Blue led indicates external rom-reading ...
        gpio_put(LED_PIN_B, LED_ON);
        // Prepare data for next round ...
        gpio_put(P_ISA_DRV, drive_data & 1);
        break;
      case 54-1:
        // Don't drive data any more
        gpio_put(P_ISA_OE, 1);
        bIsaEn = 0;
        // ... so no more data after this ...
        output_isa = drive_data_flag = 0;
        break;
      default: // Drive during bit 43->52
        if (!bIsaEn) {
          gpio_put(P_ISA_OE, 0); // Enable ISA driver
          bIsaEn = 1;
        }
        // Expose the next bit on the ISA line ...
        gpio_put(P_ISA_DRV, drive_data & 1);
        drive_data >>= 1;
        driven_isa++;
      }
    }

    // Wait for CLK1 to have a falling edge
    WAIT_FALLING(P_CLK1);

    // Another bit, check SYNC to find bit number
    sync = GPIO_PIN(P_SYNC);

    // Increment the bit number (or sync ...)
    if (sync && !last_sync) {
      pBus->sync = 1;
      sync_count++;
      bit_no = 44;
    } else {
      bit_no = (bit_no >= LAST_CYCLE) ? 0 : bit_no+1;
    }

    // NOTE! Bit numbers are ok since bit_no has been incremented!

    // Save all bits in data56 reg ...
    bit = 1LL << bit_no;
    if (GPIO_PIN(P_DATA))
      data56 |= bit;
    if (GPIO_PIN(P_ISA))
      isa |= bit;

    switch (bit_no) {
    case 0:
      pBus = &bus[data_in];
      pBus->sync = 0;
      pBus->cmd = 0;
      romAddr = 0;
      break;

    case 7:
      pBus->pa = data56 & 0xFF;
      break;

    case 29:
      // Got address. If the address is that of the embedded ROM then we flag that we have
      // to put an instruction on the bus later
      pBus->addr = (isa >> 14) & 0xFFFF;
      mp = &modules[PAGE(pBus->addr)];
      if (mp->flags & IMG_INSERTED)
        romAddr = pBus->addr & PAGE_MASK; // - mp->start;
      break;

    case 30:
#ifdef DRIVE_ISA
      // Check if we should emulate any modules ...
      if (mp->flags & IMG_INSERTED) {
        embed_seen++;
        pBus->cmd = drive_data = mp->image[romAddr];
        drive_data_flag = 1;
#if 0 // Test to change instruction in system ROM
      } else {
        if( pBus->addr == 016066 ) {
          inst = drive_data = 001;
          drive_data_flag = 1;
        } else
          drive_data_flag = 0;
#endif
      }
#endif
      break;
    case 42: // Enable ISA output in next loop (cycle 43)
      if( drive_data_flag )
        output_isa = 1;
      break;
    case LAST_CYCLE:
      // If bitno = LAST_CYCLE then we have another frame, store the transaction
      // A 56 bit frame has completed, we send some information to core0 so it can update
      // the display and anything else it needs to
      if( pBus->addr || isa || pBus->cmd ) {
        // Is instruction fetched from flash?
        if( !pBus->cmd )
          pBus->cmd = (isa >> 44) & 0x3FF;
        pBus->data = data56;
#ifdef TRACE_ISA
        pBus->isa = isa;
#endif

#ifdef DRIVE_CARRY
        if( pBus->cmd == INST_PBSY && pBus->sync ) {
          // Drive FI T0 low 
          //carry_fi_t0 = 0;
        }
#endif

        if( (time_us_64() - tm) < 2 )
          break;

        isa = data56 = 0LL;
  
        last_data_in = data_in;
  
        INC_BUS_PTR(data_in);
  
        if (data_out == data_in) {
          // No space left in ring-buffer ...
          queue_overflow = 1;
          data_in = last_data_in;
          gpio_put(LED_PIN_R, LED_ON);
        }
        gpio_put(LED_PIN_B, LED_OFF);
      }
      break;
    }
    last_sync = sync;
  }
}

void process_bus(void)
{
  int br; // Breakpoint

  // Process data coming in from the bus via core 1
  while (data_in != data_out) {
#if 0
    /*printf("\n%d: Addr:%04X %07o  Inst: %06o PeriAd:%02X Data56:%014llX ISA:%014llX",
     data_out,
     bus[data_out].addr,
     bus[data_out].addr,
     bus[data_out].cmd,
     bus[data_out].pa,
     bus[data_out].data,
     bus[data_out].isa);
     */
    printf("\n%4d: Addr:%04X Inst: %03X PeriAd:%02X Data56:%014llX ISA:%014llX",
     data_out,
     bus[data_out].addr,
     bus[data_out].cmd,
     bus[data_out].pa,
     bus[data_out].data,
     bus[data_out].isa);
    ISA_t *s = (ISA_t*)&bus[data_out].isa;
    printf("\nAddr: %04X Inst: %03X\n", s->addr, s->inst);
#else
    // Handle the bus traffic
    br = isBrk(bus[data_out].addr);
    if( br == BRK_START )
      bTrace = true;
    handle_bus(&bus[data_out]);
    if( br == BRK_STOP )
      bTrace = false;
#endif
    INC_BUS_PTR(data_out);
  }
}

////////////////////////////////////////////////////////////////////////////////
//
// This function is passed all of the traffic on the bus. It can do
// various things with that.
//

uint64_t dreg_a = 0, dreg_b = 0, dreg_c = 0;
bool display_on = false;

void dump_dregs(void)
{
  dreg_a &= MASK_48_BIT;
  dreg_b &= MASK_48_BIT;
  dreg_c &= REG_C_48_MASK;

#if CF_DUMP_DBG_DREGS
  printf(" A:%012llX B:%012llX C:%012llX", dreg_a, dreg_b, dreg_c);
#endif

  // Build a text form of the display
  int j = 0;

  for (int i = 0; i < NR_CHARS; i++) {
    char cc = 0;
    char cl = 0;
    int u = 0;

    int b = ((NR_CHARS - 1) - i) << 2;

    cc = (dreg_a >> b) & 0x0F;
    cc |= ((dreg_b >> b) & 0x0F) << 4;
    u = (dreg_c >> b) & 1;

    cl = (cc & 0xc0) >> 6;
    cc &= 0x3f;

    if (u) {
      // Upper ROM character
      cc |= 0x80;
    } else {
      // Convert to ASCII character
      if (cc < 0x20)
        cc |= 0x40;
    }

    bPunct[j] = false;
    dtext[j++] = cc;

    if (cl) {
      // Add punctuation
      bPunct[j] = true;
      dtext[j++] = " .:,"[cl];
    }
  }

  dtext[j++] = '\0';

#if CF_DISPLAY_LCD
  UpdateLCD(dtext, bPunct, display_on);
  if( bTrace )
    printf("\n[%s] (%s)", dtext, display_on ? "ON" : "OFF");
#endif
}

void updateDispReg(uint64_t data, uint8_t r)
{
  int bShift = inst50cmd[r].shft;
  int bReg = inst50cmd[r].reg;
  int bits = inst50cmd[r].len;

  uint64_t *ra = (bReg & REG_A) ? &dreg_a : NULL;
  uint64_t *rb = (bReg & REG_B) ? &dreg_b : NULL;
  uint64_t *rc = (bReg & REG_C) ? &dreg_c : NULL;

#if CF_DBG_DISP_INST
  printf("\n%s %016llX -->", inst50disp[r], data);
#endif

  if ((bShift & D_LONG) && bits == 48) {
    if (ra)
      *ra = data & (MASK_48_BIT);
    if (rb)
      *rb = data & (MASK_48_BIT);
    if (rc)
      *rc = data & (REG_C_48_MASK);
  } else {
    int n = (bShift & D_LONG) ? 48 / bits : 1;
    for (int i = 0; i < n; i++) {
      uint16_t ch9 = CH9(data);
      if (bShift & SHIFT_R) { // Shift RIGHT
        if (ra)
          *ra = (*ra >> 4) | (((uint64_t)CH9_B03(ch9)) << 44);
        if (rb)
          *rb = (*rb >> 4) | (((uint64_t)CH9_B47(ch9)) << 44);
        if (rc)
          *rc = (*rc >> 4) | (((uint64_t)CH9_B8(ch9)) << 44);
      } else { // Shift LEFT
        if (ra)
          *ra = (*ra << 4) | (CH9_B03(ch9));
        if (rb)
          *rb = (*rb << 4) | (CH9_B47(ch9));
        if (rc)
          *rc = (*rc << 4) | (CH9_B8(ch9));
      }
      data >>= bits; // shift 8 or 12 bits ...
    }
    if (ra)
      *ra &= MASK_48_BIT;
    if (rb)
      *rb &= MASK_48_BIT;
    if (rc)
      *rc &= REG_C_48_MASK;
  }
  dump_dregs();
}

void rotateDispReg(uint8_t r)
{
  int bShift = inst70cmd[r].shft;
  int bReg = inst70cmd[r].reg;
  int ch = inst70cmd[r].len;

  uint64_t *ra = (bReg & REG_A) ? &dreg_a : NULL;
  uint64_t *rb = (bReg & REG_B) ? &dreg_b : NULL;
  uint64_t *rc = (bReg & REG_C) ? &dreg_c : NULL;

#if CF_DBG_DISP_INST
  printf("\n%s -->", inst70disp[r]);
#endif

  for (int i = 0; i < ch; i++) {
    if (bShift & SHIFT_R) { // Shift RIGHT
      if (ra)
        ROTATE_RIGHT(*ra);
      if (rb)
        ROTATE_RIGHT(*rb);
      if (rc)
        ROTATE_RIGHT(*rc);
    } else { // Shift LEFT
      if (ra)
        ROTATE_LEFT(*ra);
      if (rb)
        ROTATE_LEFT(*rb);
      if (rc)
        ROTATE_LEFT(*rc);
    }
  }
  dump_dregs();
}

void handle_bus(volatile Bus_t *pBus)
{
  int addr = pBus->addr;
  int inst = pBus->cmd;
  int pa = pBus->pa;
  int sync = pBus->sync;
  uint64_t data56 = pBus->data;
#ifdef TRACE_ISA
  uint64_t isa = pBus->isa;
#endif
  bool bLdi = false;
  static int pending_data_inst = 0;
  static int oAddr = 0xFFFF;

#ifdef CPU2_PRT
  // Any printouts from the other CPU ... ?
  if (cpu2buf[0]) {
    printf("\n[%s]", cpu2buf);
    cpu2buf[0] = 0;
  }
#endif

  if (pending_data_inst == INST_PRPH_SLCT) {
#if CF_DBG_SLCT
    printf("\nPF AD:%02X", pa);
#endif
    switch (pa) {
    case DISP_ADDR:
      peripheral_ce = PH_DISPLAY;
      break;
    case WAND_ADDR:
      peripheral_ce = PH_WAND;
      break;
    case TIMR_ADDR:
      peripheral_ce = PH_TIMER;
      break;
    case CRDR_ADDR:
      peripheral_ce = PH_CRDR;
      break;
    default:
      peripheral_ce = PH_NONE;
    }
  }

#ifdef QUEUE_STATUS
  int remaining = (data_in - data_out) + (-((int) (data_in <= data_out)) & NUM_BUS_MASK);
  if( queue_overflow )
    nCpu2 = sprintf(cpu2buf, "\n[****]");
  else
    nCpu2 = sprintf(cpu2buf, "\n[%4d]", remaining);
#else
  nCpu2 = sprintf(cpu2buf, "\n");
#endif

  switch (peripheral_ce) {
  case PH_DISPLAY:
    nCpu2 += sprintf(cpu2buf + nCpu2, "DISP");
    break;
  case PH_WAND:
    nCpu2 += sprintf(cpu2buf + nCpu2, "WAND");
    break;
  case PH_TIMER:
    nCpu2 += sprintf(cpu2buf + nCpu2, "TIMR");
    break;
  case PH_CRDR:
    nCpu2 += sprintf(cpu2buf + nCpu2, "CRDR");
    break;
  default:
    nCpu2 += sprintf(cpu2buf + nCpu2, "    ");
  }

  //  printf("- DCE:%d ADDR:%04X (%02o %04o) INST=%04X (%04o) PA=%02X (%03o) sync=%d DATA=%016llX", display_ce, addr, addr>>10, addr&0x3FF, inst, inst, pa, pa, sync, data56);
  //  printf(" (%02o %04o) sync=%d DATA=%016llX", addr>>10, addr&0x3FF, sync, data56);
  uint8_t q = (addr >> 10) & 0b1111;
  #ifdef TRACE_ISA
  if( PAGE(addr) < 3 ) {
    nCpu2 += sprintf(cpu2buf + nCpu2, " %014llX %014llX | %04X (Q%2d:%03X) %03X",
            data56, isa, addr, q, addr & 0x3FF, inst);
  } else {
    nCpu2 += sprintf(cpu2buf + nCpu2, " %014llX %014llX | %04X           %03X",
            data56, isa, addr, inst);
  }
  #else
  nCpu2 += sprintf(cpu2buf + nCpu2, " %014llX %c| %04X (%X:%d %04o) %03X",
            data56, sync ? '*':' ', addr, PAGE(addr), (addr >> 10) & 0b11, addr & 0x3FF, inst);
  #endif
  //nCpu2 += sprintf(cpu2buf + nCpu2, "| %04X (%X:%d %04o) %03X", addr, addr >> 12, (addr >> 10) & 0b11, addr & 0x3FF, inst);

  switch (pending_data_inst) {
  case INST_FETCH:
    if( bTrace )
      printf("%s   > @%04X --> %03X (%04o)", cpu2buf, addr, inst, inst);
    break;
  case INST_WRITE:
    if (1) {
      int wAddr = (data56 >> 12) & 0xFFFF;
      int wDat = data56 & 0x3FF;
      Module_t *ram = &modules[PAGE(wAddr)];
      if (ram->flags & IMG_INSERTED && ram->flags & IMG_RAM) {
        // Page active and defined as ram - update!
        ram->image[wAddr & PAGE_MASK] = wDat;
        ram->flags |= IMG_DIRTY;
        printf("   W> %03X -> @%04X !!\n", wDat, wAddr);
      } else {
        printf("   W> %03X -> @%04X (Not RAM!)\n", wDat, wAddr);
      }
    }
    break;
  case INST_PBSY:
    carry_fi_t0 = 0;  // We have read the flag - clear it ...
    // Fall through!
  default:
    if( bTrace ) {
      char *dp = disAsm(inst, addr, data56);
      if( oAddr != addr )
        printf("\n%s", cpu2buf);
      else
        printf("%s", cpu2buf);
      if( dp )
        printf(" - %s", dp);
      oAddr = addr + 1;
    }
    break;
  }

  cpu2buf[0] = 0;
  nCpu2 = 0;

  // Check for a pending instruction from the previous cycle
  switch (pending_data_inst) {
  case INST_LDI:
    bLdi = true;
    break;

  case INST_WRITE_ANNUNCIATORS:
#if CF_DBG_DISP_INST
    printf("\n%s %03llX", "WRTEN", data56 & 0xFFF);
#endif
    UpdateAnnun((uint16_t)(data56 & 0xFFF));
    break;

  case INST_SRLDA:
  case INST_SRLDB:
  case INST_SRLDC:
  case INST_SRLDAB:
  case INST_SRLABC:
  case INST_SLLDAB:
  case INST_SLLABC:
  case INST_SRSDA:
  case INST_SRSDB:
  case INST_SRSDC:
  case INST_SLSDA:
  case INST_SLSDB:
  case INST_SRSDAB:
  case INST_SLSDAB:
  case INST_SRSABC:
  case INST_SLSABC:
    updateDispReg(data56, pending_data_inst >> 6);
    break;
  case INST_WANDRD:
    printf("\n%s %03llX", "WAND BUF", data56 & 0xFFF);
  }
  // Clear pending flag ...
  pending_data_inst = 0;

  // Check for instructions
  if (sync) {
    switch (inst) {
    case INST_DISPLAY_OFF:
      display_on = true; // Turns fff below ...:P
    case INST_DISPLAY_TOGGLE:
      display_on = !display_on;
#if CF_DBG_DISP_ON
      printf(" - Display %s", display_on ? "ON" : "OFF");
#endif
      dump_dregs();
      break;
    case INST_PRPH_SLCT:
    case INST_RAM_SLCT:
#if CF_DBG_SLCT
      printf("\n%s_SLCT: PA=%02X", inst == INST_PRPH_SLCT ? "PRPH" : "RAM", pa);
#endif
      pending_data_inst = inst;
      break;
    case INST_LDI:
      bLdi = true;
      // Falls through
    case INST_WRITE:
    case INST_FETCH:
    case INST_PBSY:
      pending_data_inst = inst;
      break;
    case INST_POWOFF:
      dump_dregs();
      saveRam(0xC);
      break;
    }
  }

  if( peripheral_ce && !bLdi ) {
    switch(peripheral_ce) {
    case PH_WAND:
      // Check for Wand transactions
      if (inst == INST_WANDRD)
        pending_data_inst = inst;
      break;
    case PH_DISPLAY:
      // Check for display transactions
      if (inst == INST_WRITE_ANNUNCIATORS)
        pending_data_inst = inst;
      else {
        switch (inst & 000077) {
        case 000050:
#if CF_DBG_DISP_INST
          printf("\n%s", inst50disp[inst >> 6]);
#endif
          pending_data_inst = inst;
          break;
        case 000070:
#if CF_DBG_DISP_INST
          printf("\n%s", inst70disp[inst >> 6]);
#endif
          rotateDispReg(inst >> 6);
          break;
        }
      }
      break;
    }
  }
}
