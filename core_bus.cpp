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
#define DRIVE_DATA
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

inline uint16_t swap16(uint16_t b)
{
  return __builtin_bswap16(b);
}

volatile uint8_t wandBuf[16]; // Max size of a barcode
volatile uint8_t nWBuf = 0;
volatile uint8_t pWBuf = 0;

void sendWand(uint8_t *w, uint8_t n)
{
  for(int i=0; i<n; i++)
    wandBuf[i] = w[i];
  nWBuf = n;
  pWBuf = 0;
}

#define BRK_SIZE (0x10000/(32/2)) // 2 bits per brkpt
uint32_t brkpt[BRK_SIZE];
enum {
  BRK_NONE,
  BRK_START,
  BRK_STOP,
};
#define BRK_MASK(a,w) ((brkpt[a>>4]w >> ((a & 0xF)<<1)) & 0b11)
#define BRK_SHFT(a) ((a & 0xF)<<1)
#define BRK_WORD(a) (a >> 4)

// Check if breakpoint is set for given address
inline int isBrk(uint16_t addr)
{
  uint32_t w = brkpt[BRK_WORD(addr)];
  return w ? (w >> BRK_SHFT(addr)) & 0b11 : 0;
}
// Clear breakpoint on given address
void clrBrk(uint16_t addr)
{
  brkpt[BRK_WORD(addr)] &= ~(0b11 << BRK_SHFT(addr));
}
void clrAllBrk(void)
{
  printf("Clear all breakpoints\n");
  memset(brkpt, 0, sizeof(uint32_t) * BRK_SIZE);
}
// Set breakpoint on given address
void setBrk(uint16_t addr)
{
  clrBrk(addr); // Remove previous settings
  brkpt[BRK_WORD(addr)] |= BRK_START << BRK_SHFT(addr);
}
void stopBrk(uint16_t addr)
{
  clrBrk(addr); // Remove previous settings
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

// Keep track of FI flags (0-13)
volatile uint16_t carry_fi = 0;
volatile bool bPBusy = false;

void _setFI_PBSY(void)
{
  bPBusy = true;
}
void _clrFI_PBSY(void)
{
  bPBusy = true;
}

// Set FI flag (T0-T13)
inline void setFI(int flag)
{
  if( bPBusy )
    flag |= FI_PBSY;
  carry_fi |= flag;
}
inline void clrFI(int flag = 0)
{
  carry_fi &= (flag ? ~flag : 0);
}
inline uint16_t getFI(int flag)
{
  return carry_fi & flag;
}
inline uint16_t getFI(void)
{
  if( bPBusy )
    setFI(FI_PBSY);
  return carry_fi & FI_MASK;
}

volatile int wDelayBuf = 0;
volatile bool bSendWndData = false;

void _power_on()
{
  // Drive ISA high to turn on the calculator ...
  gpio_put(P_ISA_DRV, 1);
  gpio_put(P_ISA_OE, ENABLE_OE); // Enable ISA driver
  // Set PBSY-flag on to indicate someone needs service ...
  _setFI_PBSY();
  sleep_us(10);
  gpio_put(P_ISA_OE, DISABLE_OE); // Disable ISA driver
  gpio_put(P_ISA_DRV, 0);
  sleep_ms(10);
  _clrFI_PBSY();
}
void power_on(void)
{
  printf("Try to power on the calculator ...\n");
  _power_on();
}

#if 0
uint8_t wdata[] = {
  16, 0x50, 0x10, 0x02, 0xC0, 0x00, 0xF3, 0x00, 0x44, 0x43, 0x6F, 0x1B, 0x13, 0x00, 0x43, 0x11, 0x10,
  16, 0x9D, 0x11, 0x11, 0x00, 0x40, 0xCE, 0x7E, 0xAA, 0x13, 0xA8, 0x14, 0xAA, 0x12, 0xA8, 0x13, 0xAA,
  16, 0xBD, 0x12, 0x10, 0x11, 0xA8, 0x12, 0xAA, 0x0F, 0xA8, 0x11, 0xAA, 0x0E, 0xA8, 0x10, 0xCE, 0x7E,
  16, 0x4E, 0x13, 0x01, 0x91, 0x75, 0xF3, 0x7F, 0x00, 0x00, 0x77, 0x91, 0x76, 0xF2, 0x7F, 0x41, 0xCE,
  11, 0x23, 0x14, 0x10, 0x76, 0x87, 0xCE, 0x75, 0x7E, 0xC0, 0x00, 0x2F,
   0
};
#endif

uint8_t wdata[] = {
/* ROW:  1 */  16, 0x01, 0x10, 0x05, 0xC6, 0x00, 0xF5, 0x00, 0x43, 0x4F, 0x44, 0x45, 0xF6, 0x43, 0x4F, 0x44, 0x45,
/* ROW:  2 */  16, 0x17, 0x11, 0x22, 0x3D, 0x3F, 0x8C, 0x84, 0x8B, 0xC8, 0x02, 0xF3, 0x00, 0x43, 0x4F, 0xF8, 0x7F,
/* ROW:  3 */  16, 0xCD, 0x12, 0x70, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x1A, 0x10, 0x10, 0x16, 0x91, 0x74,
/* ROW:  4 */  16, 0xA5, 0x13, 0x01, 0xC0, 0x03, 0xF3, 0x00, 0x48, 0x42, 0x11, 0x90, 0x77, 0xCE, 0x7E, 0x71, 0xA9,
/* ROW:  5 */  16, 0xDF, 0x14, 0x10, 0x00, 0xA9, 0x01, 0xA9, 0x02, 0xAA, 0x03, 0xB2, 0x00, 0xA8, 0x04, 0xAB, 0x07,
/* ROW:  6 */  16, 0x78, 0x15, 0x00, 0xA8, 0x07, 0xAC, 0x07, 0x54, 0xAC, 0x06, 0x54, 0xA8, 0x06, 0x66, 0xA9, 0x06,
/* ROW:  7 */  16, 0x5D, 0x16, 0x01, 0x66, 0xA8, 0x05, 0x02, 0x94, 0x73, 0xAA, 0x04, 0xA8, 0x00, 0xAA, 0x05, 0xA8,
/* ROW:  8 */  16, 0xA6, 0x17, 0x10, 0x01, 0xAA, 0x06, 0xA8, 0x02, 0xAA, 0x07, 0xA8, 0x03, 0xAA, 0x0B, 0xB2, 0x00,
/* ROW:  9 */  16, 0xAE, 0x18, 0x01, 0xA8, 0x0C, 0xAB, 0x0F, 0xA8, 0x0F, 0xAC, 0x0F, 0x54, 0xAC, 0x0E, 0x54, 0xA8,
/* ROW: 10 */  16, 0x31, 0x19, 0x11, 0x0E, 0x66, 0xA9, 0x0E, 0x66, 0xA8, 0x0D, 0x02, 0xAA, 0x0C, 0xA8, 0x04, 0xAA,
/* ROW: 11 */  16, 0x2C, 0x1A, 0x11, 0x0D, 0xA8, 0x05, 0xAA, 0x0E, 0xA8, 0x06, 0xAA, 0x0F, 0xA8, 0x07, 0x75, 0xCE,
/* ROW: 12 */  16, 0x62, 0x1B, 0x11, 0x7E, 0x90, 0x77, 0x90, 0x76, 0x90, 0x75, 0x91, 0x76, 0x74, 0x91, 0x75, 0xF2,
/* ROW: 13 */  16, 0x28, 0x1C, 0x23, 0x7F, 0x2A, 0xCE, 0x76, 0x74, 0x91, 0x77, 0x74, 0x91, 0x76, 0xF3, 0x7F, 0x2A,
/* ROW: 14 */  16, 0x34, 0x1D, 0x10, 0x2A, 0x90, 0x71, 0x91, 0x75, 0x96, 0x74, 0x1D, 0xF2, 0x48, 0x42, 0x90, 0x75,
/* ROW: 15 */  12, 0x11, 0x1E, 0x00, 0x87, 0x91, 0x75, 0x7E, 0x9F, 0x09, 0xC4, 0x15, 0x2F,
  0
};

static int row = -1;
static int wp = 0;

void wand_on(void)
{
  if( wp == 0 )
    row=1;
  printf("Simulate Wand Barcode scan - Row:%d ... ", row++);
  _power_on();
  // Set carry to service the wand
  _setFI_PBSY();
  // Fill buffer with next batch of bytes ...
  sendWand(wdata+wp+1,*(wdata+wp));
  // Update pointer to next batch ...
  wp += *(wdata+wp)+1;
  if( *(wdata+wp) == 0 )
    wp = 0;
  if( !wp ) {
    printf("Done!");
    row = -1;
  }
  sleep_ms(10);
  _clrFI_PBSY();
  printf("\n");
}

#if CF_DBG_DISP_INST
extern const char *inst50disp[16];
extern const char *inst70disp[16];
#endif

#define PAGEn(n,i) (FLASH_TARGET_OFFSET + (2 * n + i) * FLASH_SECTOR_SIZE)
#define PAGE1(n) PAGEn(n,0)
#define PAGE2(n) PAGEn(n,1)

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

// Make space for information about all flash images
Module_t modules[NR_PAGES];

// Add image to the list of modules
// A pointer to the image and which port to attatch it to
void addRom(int port, uint16_t *image)
{
	if( image ) {
    modules[port].image = image;
    modules[port].flags = IMG_INSERTED;
    printf("Add ROM @ %04X - %04X\n", port * PAGE_SIZE, (port * PAGE_SIZE)|PAGE_MASK);
	} else {
    modules[port].image = NULL;
    modules[port].flags = IMG_NONE;
	}
}

#ifdef USE_FLASH
// Save a ram image to flash (emulate Q-RAM)
void saveRam(int port, int ovr = 0)
{
  if( port >= FIRST_PAGE && port < NR_PAGES ) {
    if (ovr | (modules[port].flags & IMG_DIRTY)) {
      modules[port].flags &= ~IMG_DIRTY;
      erasePort(port);
      writePort(port, rom_pages[port - FIRST_PAGE]);
      printf("Wrote RAM[%X] to flash\n", port);
    }
  } else {
      printf("Invalid page! [%X]\n", port);
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

void qRam(int page)
{
  if (modules[page].flags & IMG_INSERTED) {
    modules[page].flags |= IMG_RAM;
    printf("Add QRAM[%X] @ %04X - %04X\n", page, page*PAGE_SIZE, (page*PAGE_SIZE)|PAGE_MASK);
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
  // Erase all images from flash
  for (int p = 0; p < NR_PAGES; p++)
    erasePort(p);
#endif
  // Remove all modules ...
  memset(modules, 0, sizeof(Module_t) * NR_PAGES);
  //for (int p = 0; p <= LAST_PAGE; p++)
	//	addRom(p, NULL);

  // Check for existing images and load them ...
  for (int port = FIRST_PAGE; port <= LAST_PAGE; port++) {
    int pIdx = port - FIRST_PAGE;
    // Read flash image
    readPort(port, rom_pages[pIdx]);
		int nErr = 0; // Nr of errors
		int nClr = 0; // Nr of erased words
    // Check that image is valid (10 bits)
		for(int i=0; i<PAGE_SIZE; i++) {
			if( rom_pages[pIdx][i] > INST_MASK ) {
				nErr++;
        if( rom_pages[pIdx][i] == ADDR_MASK )
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
  qRam(0xC);
}

char *disAsm(int inst, int addr, uint64_t data, uint8_t sync);

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
volatile int drive_data_flag = 0;
volatile int output_isa = 0;  // Output ongoing on ISA...
volatile int output_data = 0; // Output ongoing on DATA...

// The data we drive
int drive_data = 0;

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

// Size of trace buffer should be a power of 2 (for the mask)
#ifdef TRACE_ISA
#define NUM_BUS_T 0x400
#else
#define NUM_BUS_T 0x1000
#endif

#define NUM_BUS_MASK (NUM_BUS_T-1)
#define INC_BUS_PTR(d) d = (d+1) & NUM_BUS_MASK

volatile int queue_overflow = 0;

volatile int data_wr = 0;
volatile int data_rd = 0;

volatile uint64_t data56_out = 0;

volatile Bus_t bus[NUM_BUS_T];

int last_sync = 0;
int gpio_states = 0;

int peripheral_ce = 0;
char dtext[2 * NR_CHARS + 1];
bool bPunct[2 * NR_CHARS + 1];

char cpu2buf[256];
int nCpu2 = 0;
char cpu3buf[256];
int nCpu3 = 0;

void reset_bus_buffer(void)
{
  // Reset buffer if overflow has occured ...
  if( queue_overflow ) {
    queue_overflow = 0;
    data_rd = data_wr = 0;
    gpio_put(LED_PIN_R, LED_OFF);
  }
}

void core1_main_3(void)
{
  static int bIsaEn = 0;
  static int bDataEn = 0;
  static int isDataEn = 0;
  static int bIsa = 0;
  static int bit_no = 0;
  static uint64_t isa = 0LL;
  static uint64_t bit = 0LL;
  int sync = 0;
  static uint64_t data56 = 0;
  static uint16_t dataFI = 0;
  int last_data_wr;
  static uint16_t romAddr = 0;
  Module_t *mp = NULL;
  volatile Bus_t *pBus = &bus[data_wr];
  static uint8_t perph = 0;
  static uint8_t ramad = 0;
  static bool bSelPa = false;
  static bool bSelRam = false;
  static bool bErr = false;
  static uint8_t cnt = 0;

  irq_set_mask_enabled(0xffffffff, false);

  last_sync = GPIO_PIN(P_SYNC);

#ifdef MEASURE_TIME
  uint64_t tm = time_us_64();
#endif

  while (1) {
    // Wait for CLK2 to have a falling edge
    WAIT_FALLING(P_CLK2);

    // NOTE! Bit numbers are out by one as bit_no hasn't been incremented yet.

    // Drive the FI carry flags (if any ...)
    if( (bit_no & 0b11) == 0b11 ) {
      gpio_put(P_FI_OE, dataFI & 1);
      dataFI >>= 1;
    }

#ifdef DRIVE_DATA
    // Do we drive the DATA line (bit 0-55)?
    if (output_data) {
      // Expose the next bit on the data line ...
      gpio_put(P_DATA_DRV, data56_out & 1);
      data56_out >>= 1;
    }
#endif

    // Do we drive the ISA line (bit 43-53)?
    if (output_isa) {
      // Drive the ISA line for theses data bit
      switch (bit_no) {
      case 43-1:
        // Blue led indicates external rom-reading ...
        if( !bErr )
          gpio_put(LED_PIN_B, LED_ON);
        // Prepare data for next round ...
        gpio_put(P_ISA_DRV, drive_data & 1);
        break;
      case 54-1:
        // Don't drive data any more
        gpio_put(P_ISA_OE, DISABLE_OE);
        bIsaEn = 0;
        // ... so no more data after this ...
        output_isa = drive_data_flag = 0;
        break;
      default: // Drive during bit 43->52
        if (!bIsaEn) {
          gpio_put(P_ISA_OE, ENABLE_OE); // Enable ISA driver
          bIsaEn = 1;
        }
        // Expose the next bit on the ISA line ...
        gpio_put(P_ISA_DRV, drive_data & 1);
        drive_data >>= 1;
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
#ifdef MEASURE_TIME
      tm = time_us_64();
#endif
      pBus = &bus[data_wr];
      pBus->sync = 0;
      pBus->cmd = 0;
      pBus->fi = 0;
      romAddr = 0;
      break;

    case 7:
      if( bSelPa ) {
        // Get the selected peripheral
        perph = data56 & 0xFF;
        bSelPa = false;
      } else if( bSelRam ) {
        // Get the selected peripheral
        ramad = data56 & 0xFF;
        bSelRam = false;
      }
      break;

    case 29:
      // Got address. If the address is that of the embedded ROM then we flag that we have
      // to put an instruction on the bus later
      pBus->addr = (isa >> 14) & ADDR_MASK;
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
      }
#endif
      break;
    case 42: // Enable ISA output in next loop (cycle 43)
      if( drive_data_flag )
        output_isa = 1;
      break;
    case LAST_CYCLE-2:
      // Check if more data to be read from the wand ...
      if( nWBuf ) {
        setFI(FI_PBSY | FI_WNDB);
      } else {
        clrFI(FI_WNDB);
      }
      break;
    case LAST_CYCLE-1:
      // Report next FI signal to trace ...
      pBus->fi = getFI();
      // Remember last queue write pointer ...
      last_data_wr = data_wr;
      // Turn off MLDL-led ...
      gpio_put(LED_PIN_B, LED_OFF);
      break;
    case LAST_CYCLE:
      // If bitno = LAST_CYCLE then we have another frame, store the transaction
      // A 56 bit frame has completed, we send some information to core0 so it can update
      // the display and anything else it needs to

      // Assume no data drive any more ...
      bDataEn = 0;
      output_data = 0;

      if( pBus->addr || isa || pBus->cmd ) {
        // Is instruction fetched from flash?
        if( !pBus->cmd )
          pBus->cmd = (isa >> 44) & INST_MASK;
        pBus->data = data56;
#ifdef TRACE_ISA
        pBus->isa = isa;
#endif

        switch( pBus->cmd ) {
        case INST_RAM_SLCT:
          // Fetch selected RAM in the next run ...
          bSelRam = true;
          // ... and reset any chip enabled ...
          perph = 0;
          break;
        case INST_PRPH_SLCT:
          // Fetch selected peripheral in the next run ...
          bSelPa = true;
          break;
#ifdef DRIVE_CARRY
        case INST_WANDRD:
          if( perph == WAND_ADDR && nWBuf && pBus->sync  ) {
            // Enable data driver ...
            output_data = bDataEn = 1;
            // Get next byte from the wand-buffer!
            data56_out = wandBuf[pWBuf++];
            nWBuf--;
            if( !nWBuf ) {
              // Ready to receive next batch of data
              bSendWndData = true;
              // Number of instructions delay after buffer is emptied ...
              wDelayBuf = 0;
            }
          }
          break;
#endif
        }
#ifdef MEASURE_TIME
        {
          uint16_t dt = (uint16_t)(time_us_64() - tm);
          pBus->fi = dt;
          // Floating input-pin ... ?
          if( dt < 0x40 || dt > 0x100 ) {
            bErr = true;
            break;
          }
        }
        bErr = false;
#endif
#ifdef MEASURE_COUNT
        // Report FI-status to trace ...
        pBus->fi |= cnt++ << 8;
#endif
        // Report selected peripheral to trace ...
        pBus->pa = perph;

        // Setup FI-signal for next round ...
        dataFI = getFI();

        // Cleanup, fix trace buffer pointer and
        // check for overflow ...
        isa = data56 = 0LL;
        bit = 1LL;

        INC_BUS_PTR(data_wr);
        if (data_rd == data_wr) {
          // No space left in ring-buffer ...
          queue_overflow++;
          // Restore last pointer (overwriting this cycle)
          data_wr = last_data_wr;
        }
      }
      // Enable DATA for next cycle ... ?
      if( bDataEn != isDataEn ) {
        isDataEn = bDataEn;
        gpio_put(P_DATA_OE, isDataEn);
      }
      break;
    }
    last_sync = sync;
  }
}

void post_handling(uint16_t addr)
{
  if( bSendWndData ) {
    // Make a delay between the old and new batch of data ...
    //if( wDelayBuf > 3 )
    {
      // Check if we have more data to send and buffer is empty ...
      // row > 0 if we have more data to send
      // nWBuf == 0 if all data in buffer have been read
      // FI_WNDB (T2) is low if buffer is empty
      if( row < 0 && nWBuf == 0 && getFI(FI_WNDB) == 0 ) {
        printf("End wand busy flag ...\n");
        clrFI(FI_PBSY);
        bSendWndData = false;
      } else
      if( row > 0 && nWBuf == 0 && getFI(FI_WNDB) == 0 && (addr& 0x0FFF) == 0xA23) {
        // Update buffer with next batch of data ...
        if( wDelayBuf > 3 ) {
          wand_on();
          bSendWndData = false;
        }
        wDelayBuf++;
      }
    }
    // else
//      wDelayBuf--;
  } 
}

void process_bus(void)
{
  int br; // Breakpoint

  // Process data coming in from the bus via core 1
  while (data_wr != data_rd) {
#if 0
    /*printf("\n%d: Addr:%04X %07o  Inst: %06o PeriAd:%02X Data56:%014llX ISA:%014llX",
     data_rd,
     bus[data_rd].addr,
     bus[data_rd].addr,
     bus[data_rd].cmd,
     bus[data_rd].pa,
     bus[data_rd].data,
     bus[data_rd].isa);
     */
    printf("\n%4d: Addr:%04X Inst: %03X PeriAd:%02X Data56:%014llX ISA:%014llX",
     data_rd,
     bus[data_rd].addr,
     bus[data_rd].cmd,
     bus[data_rd].pa,
     bus[data_rd].data,
     bus[data_rd].isa);
    ISA_t *s = (ISA_t*)&bus[data_rd].isa;
    printf("\nAddr: %04X Inst: %03X\n", s->addr, s->inst);
#else
    // Handle the bus traffic
    br = isBrk(bus[data_rd].addr);
    if( br == BRK_START )
      bTrace |= 0b010;
    handle_bus(&bus[data_rd]);
    if( br == BRK_STOP )
      bTrace &= 0b101;
#endif
    INC_BUS_PTR(data_rd);

    // Any handling outside bus traceing
    // Note that instructions can be missed if the buffer is overflowed!
    // Better handle these cases here ...
    post_handling(bus[data_rd].addr);
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
  if( IS_TRACE() )
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
  int fi = pBus->fi;
  static int oCnt = 0xFF;
  int cnt = (pBus->fi>>8) & 0xFF;
  uint64_t data56 = pBus->data;
#ifdef TRACE_ISA
  uint64_t isa = pBus->isa;
#endif
  bool bLdi = false;
  static int pending_data_inst = 0;
  static int oAddr = ADDR_MASK;

#ifdef MEASURE_COUNT
  if( ((++oCnt) & 0xFF) != cnt ) {
    printf("\n\n ###### LOST TRACE CYCLES (%d-%d) #########################\n\n", oCnt, cnt);
    oCnt = cnt;
  }
#endif

#ifdef CPU2_PRT
  // Any printouts from the other CPU ... ?
  if (cpu2buf[0]) {
    printf("\n[%s]", cpu2buf);
    cpu2buf[0] = 0;
  }
#endif

  switch (pa) {
  case DISP_ADDR:
  case WAND_ADDR:
  case TIMR_ADDR:
  case CRDR_ADDR:
    peripheral_ce = pa;
    break;
  default:
    peripheral_ce = NONE_ADDR;
  }

#if 1
#ifdef QUEUE_STATUS
  int remaining = (data_wr - data_rd) + (-((int) (data_wr <= data_rd)) & NUM_BUS_MASK);
  nCpu2 = sprintf(cpu2buf, "\n%c", queue_overflow ? '#' : ' ');
#else
  nCpu2 = sprintf(cpu2buf, "\n");
#endif
#else
#ifdef QUEUE_STATUS
  int remaining = (data_wr - data_rd) + (-((int) (data_wr <= data_rd)) & NUM_BUS_MASK);
  if( queue_overflow )
    nCpu2 = sprintf(cpu2buf, "\n[****]");
  else
    nCpu2 = sprintf(cpu2buf, "\n[%4d]", remaining);
#else
  nCpu2 = sprintf(cpu2buf, "\n");
#endif
#endif

  switch (peripheral_ce) {
  case DISP_ADDR:
  case WAND_ADDR:
  case TIMR_ADDR:
  case CRDR_ADDR:
    nCpu2 += sprintf(cpu2buf + nCpu2, "%c", "TCDW"[peripheral_ce - TIMR_ADDR]);
    break;
  default:
    nCpu2 += sprintf(cpu2buf + nCpu2, " ");
  }

  uint8_t q = (addr >> 10) & 0b1111;
#ifdef TRACE_ISA
  if( PAGE(addr) < 3 ) {
    // Dump information about which quad rom (easy find in VASM)
    nCpu2 += sprintf(cpu2buf + nCpu2, " %014llX %014llX | %04X (Q%2d:%03X) %03X",
            data56, isa, addr, q, addr & 0x3FF, inst);
  } else {
    // No need in an external rom
    nCpu2 += sprintf(cpu2buf + nCpu2, " %014llX %014llX | %04X           %03X",
            data56, isa, addr, inst);
  }
#else
#if 1
  if( PAGE(addr) < 3 ) {
    // Dump information about which quad rom (easy find in VASM)
    nCpu2 += sprintf(cpu2buf + nCpu2, "%014llX %04X|%c %04X (Q%2d:%03X) %03X",
            data56, fi, sync ? '*':' ', addr, q, addr & 0x3FF, inst);
  } else {
    // No need in an external rom
    nCpu2 += sprintf(cpu2buf + nCpu2, "%014llX %04X|%c %04X           %03X",
            data56, fi, sync ? '*':' ', addr, inst);
  }
#else
  if( PAGE(addr) < 3 ) {
    // Dump information about which quad rom (easy find in VASM)
    nCpu2 += sprintf(cpu2buf + nCpu2, " %014llX %X %c| %04X (Q%2d:%03X) %03X",
            data56, fi, sync ? '*':' ', addr, q, addr & 0x3FF, inst);
  } else {
    // No need in an external rom
    nCpu2 += sprintf(cpu2buf + nCpu2, " %014llX %X %c| %04X           %03X",
            data56, fi, sync ? '*':' ', addr, inst);
  }
#endif
#endif

  // Handle special output - or just disassemble the instruction ...
  switch (pending_data_inst) {
  case INST_FETCH:
    if( IS_TRACE() )
      nCpu2 += sprintf(cpu2buf+nCpu2, "   > @%04X --> %03X (%04o)", addr, inst, inst);
    break;
  case INST_WRITE:
    if (1) {
      int wAddr = (data56 >> 12) & ADDR_MASK;
      int wDat = data56 & INST_MASK;
      Module_t *ram = &modules[PAGE(wAddr)];
      // TBD - Should we allow writes to images not inserted?
      if (ram->flags & IMG_INSERTED && ram->flags & IMG_RAM) {
        // Page active and defined as ram - update!
        ram->image[wAddr & PAGE_MASK] = wDat;
        ram->flags |= IMG_DIRTY;
        nCpu2 = sprintf(cpu2buf, "   W> %03X -> @%04X !!\n", wDat, wAddr);
      } else {
        nCpu2 = sprintf(cpu2buf, "   W> %03X -> @%04X (Not RAM!)\n", wDat, wAddr);
      }
    }
    break;
  default:
    if( IS_FULLTRACE() ) {
      char *dp = disAsm(inst, addr, data56, sync);
      if( oAddr != addr )
        printf("\n");
      if( dp )
        nCpu2 += sprintf(cpu2buf + nCpu2, " - %s", dp);
      oAddr = addr + 1;
    }
    break;
  }

  if( IS_TRACE() )
    printf("%s", cpu2buf);
  cpu2buf[0] = 0;
  nCpu2 = 0;

  // Check for a pending instruction from the previous cycle
  switch (pending_data_inst) {
  case INST_WNDB:
    break;
  case INST_PBSY:
    break;
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
    if( IS_TRACE() && peripheral_ce == WAND_ADDR )
      printf(" WB -> %03X", (int)(data56 & 0xFFF));
    break;
  }
  // Clear pending flag ...
  pending_data_inst = 0;

  // Check for instructions
  if (sync) {
    switch (inst) {
    case INST_DISPLAY_OFF:
      display_on = !false; // Toggles to false below ...:P
    case INST_DISPLAY_TOGGLE:
      display_on = !display_on;
#if CF_DBG_DISP_ON
      printf(" - Display %s", display_on ? "ON" : "OFF");
#endif
      dump_dregs();
      break;
#if CF_DBG_SLCT
    case INST_PRPH_SLCT:
    case INST_RAM_SLCT:
      printf("\n%s_SLCT: PA=%02X", inst == INST_PRPH_SLCT ? "PRPH" : "RAM", pa);
      break;
#endif
    case INST_LDI:
      bLdi = true;
      // Falls through
    case INST_WRITE:
    case INST_FETCH:
    case INST_PBSY:
    case INST_WNDB:
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
    case WAND_ADDR:
      // Check for Wand transactions
      if (inst == INST_WANDRD)
        pending_data_inst = inst;
      break;
    case DISP_ADDR:
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
