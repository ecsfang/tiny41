#include <stdio.h>
#include <string.h>
#include <time.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "pico/multicore.h"
#include "tiny41.h"
#include "core_bus.h"
#include "disasm.h"
#include "serial.h"
#include <malloc.h>
#ifdef USE_FLASH
#include "hardware/flash.h"
#endif
#ifdef USE_PIO
#include "ir_led.h"
#endif
#include "usb/cdc_helper.h"

//#define RESET_FLASH
//#define RESET_RAM
#define DRIVE_CARRY
#define DRIVE_DATA
#define WAND_EMU

extern CDisplay    disp41;

CXFM xmem;

static volatile uint64_t __mem[XMEM_XF_SIZE+XMEM_XM1_SIZE+XMEM_XM2_SIZE];

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

#define BAR_MAXLEN 0x10
class CBarcode {
  volatile uint8_t bb[BAR_MAXLEN]; // Max size of a barcode
  volatile uint8_t nb = 0;         // Bytes left in buffer
  volatile uint8_t pb = 0;         // Byte pointer into buffer
public:
  bool empty(void) { return nb == 0; }
  bool available(void) { return nb != 0; }
  void set(uint8_t *bc) {
    nb = *bc++;
    if( nb > 0 && nb <= BAR_MAXLEN )
      memcpy((void *)bb, (void *)bc, nb);
    pb = 0;
  }
  uint8_t get(void) {
    nb--;
    return bb[pb++];
  }
};

// Hold current barcode to scan
CBarcode cBar;


CBreakpoint brk;

void swapRam(uint16_t *dta, int n);

// Keep track of FI flags (0-13)
volatile uint16_t carry_fi = 0;
volatile bool bPBusy = false;
volatile bool bClrPBusy = false;

// Indicate to set PBSY next time ...
void _setFI_PBSY(void)
{
  bPBusy = true;
}
void _clrFI_PBSY(void)
{
  //bPBusy = false;
  bClrPBusy = true;
}

inline uint16_t chkFI(int flag)
{
  return carry_fi & flag;
}
inline uint16_t getFI(void)
{
  if( bPBusy )
    setFI(FI_PBSY);
  else
    clrFI(FI_PBSY);
  if( bClrPBusy ) {
    bPBusy = false;
    bClrPBusy = false;
  }
  return carry_fi & FI_MASK;
}

volatile int wDelayBuf = 0;
//volatile uint16_t iGetNextWndData = 0;
//volatile uint16_t iSendNextWndData = 0;

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
}
void power_on(void)
{
  printf("Try to power on the calculator ...\n");
  _power_on();
}

bool bWdata = false;

#if 0
// Dedicated page to contain current flashed barcode
static uint8_t  wand_page[PAGE_SIZE];

static int row = -1;
static int wp = 0;

// Get pointer to barcode data for Row r
int barRow(int r)
{
  int bc = 0;
  uint8_t n;
  while( r > 1 ) {
    // Get length of this row
    n = wand_page[bc];
    if( n > 0 && n <= BAR_MAXLEN )
      bc += n+1;
    else
      return 0; // End of rows ...
    r--;
  }
  return bc;
}


void wand_scan(void)
{
  if( bWdata ) {
    // Clear Wand carry
    _clrFI_PBSY();
    bWdata = false;
    return;
  }
  if( wand_page[0] > 0 && wand_page[0] <= BAR_MAXLEN ) {
    if( wp == 0 ) {
      // Start with first row!
      _power_on();
      // Set carry to service the wand
      _setFI_PBSY();
      // Start with first row ...
      row = 1;
      wp = barRow(row);
    }
    printf("\nSimulate Wand Barcode scan - Row:%d ... ", row++);
    // Fill barcode buffer with data for current row ...
    cBar.set(&wand_page[wp]);
    // Update pointer to next row of data ...
    wp = barRow(row);
    if( wp == 0 ) {
      printf("Done!");
      // Clear Wand carry
      _clrFI_PBSY();
      row = -1;
    }
    printf("\n");
  } else {
    printf("No barcode to scan in flash memory!!\n");
  }
}
#endif

void wand_done(void)
{
  if( bWdata ) {
    // Clear Wand carry
    _clrFI_PBSY();
    bWdata = false;
    return;
  }
}

void wand_data(unsigned char *dta)
{
  // Start with first row!
  _power_on();
  // Set carry to service the wand
  _setFI_PBSY();
  cBar.set(dta);
  bWdata = true;
  //_clrFI_PBSY();
}

#if CF_DBG_DISP_INST
extern const char *inst50disp[16];
extern const char *inst70disp[16];
#endif

// Each port contains two 4k flash pages since 10 bits are stored (4k * uint16_t)
// ROM is not packed in flash due to compatibility with ROM file format
// And packed file would still be larger than 4k ...
#define PAGEn(n,i) (FLASH_TARGET_OFFSET + (2 * n + i) * FLASH_SECTOR_SIZE)
#define PAGE1(n) PAGEn(n,0)
#define PAGE2(n) PAGEn(n,1)

#ifdef USE_FLASH
// Writing to flash can only be done page wise!
int pageAdjust(int addr)
{
  if( addr % FLASH_PAGE_SIZE ) {
    addr |= FLASH_PAGE_SIZE-1;
    addr++;
  }
  return addr;
}

void erasePort(int n, bool bPrt=true)
{
  if( bPrt ) {
    printf("\nErasing target region %X ", n);
    printf("-- [%05X - %05X]\n", PAGE1(n), PAGE2(n) + FLASH_SECTOR_SIZE);
  }
  uint32_t ints = save_and_disable_interrupts();
  flash_range_erase(PAGE1(n), FLASH_SECTOR_SIZE);
  flash_range_erase(PAGE2(n), FLASH_SECTOR_SIZE);
  restore_interrupts(ints); // Note that a whole number of sectors must be erased at a time.
  if( bPrt ) {
    printf("Done!\n");
  }
}

void writeFlash(int offs, uint8_t *data, int sz)
{
  uint32_t ints = save_and_disable_interrupts();
  flash_range_program(offs, data, sz);
  restore_interrupts(ints);
  printf(" --> Write flash @ %08X %d bytes\n", offs, sz);
}

// Write data to given flash port
// Max 2*FLASH_SECTOR_SIZE of bytes to be written
void write8Port(int n, uint8_t *data, int sz)
{
  int sz1 = sz > FLASH_SECTOR_SIZE ? FLASH_SECTOR_SIZE : sz;
  sz1 = pageAdjust(sz1);
  int sz2 = sz > FLASH_SECTOR_SIZE ? sz-FLASH_SECTOR_SIZE : 0;
  sz2 = pageAdjust(sz2);
  writeFlash(PAGE1(n), data, sz1);
  if( sz2 )
    writeFlash(PAGE2(n), data + sz1, sz2);
}


void writePort(int n, uint16_t *data)
{
  // Swap 16-bit word to get right endian for flash ...
  swapRam(data, FLASH_SECTOR_SIZE);
  uint8_t *dp = (uint8_t*)data;
  printf("\nProgramming target region %X ", n);
  printf("-- [%05X - %05X]\n", PAGE1(n), PAGE2(n) + FLASH_SECTOR_SIZE);
  write8Port(n, (uint8_t*)data, 2*FLASH_SECTOR_SIZE);
  swapRam(data, FLASH_SECTOR_SIZE);
  printf("Done!\n");
}

const uint8_t *flashPointer(int offs)
{
  return (const uint8_t*)(XIP_BASE + offs);
}

// Read flash into ram with right endian
void readPort(int n, uint16_t *data)
{
  // Point at the wanted flash image ...
  const uint16_t *fp = (uint16_t*)flashPointer(PAGE1(n));
  for (int i = 0; i < FLASH_SECTOR_SIZE; ++i) {
    // Swap order to get right endian of 16-bit word ...
    data[i] = swap16(*fp++);
  }
}
// Read flash into ram for wand use
void readFlash(int offs, uint8_t *data, uint16_t size)
{
  // Point at the wanted flash image ...
  const uint8_t *fp = flashPointer(offs);
  memcpy(data, fp, size);
  printf(" <-- Read flash @ %08X %d bytes\n", offs, size);
}
#endif

// Allocate memory for all flash images ...
static uint16_t rom_pages[NR_PAGES - FIRST_PAGE][PAGE_SIZE];
static uint16_t bank_page[PAGE_SIZE];

// Make space for information about all flash images
CModules modules;

#ifdef USE_FLASH
// Save a ram image to flash (emulate Q-RAM)
void saveRam(int port, int ovr = 0)
{
  if( port >= FIRST_PAGE && port < NR_PAGES ) {
    if (ovr || modules.isDirty(port)) {
      modules.clear(port);
      erasePort(port, false);
      writePort(port, rom_pages[port - FIRST_PAGE]);
      printf("Wrote RAM[%X] to flash\n", port);
    }
  } else {
      printf("Invalid page! [%X]\n", port);
  }
}

void saveXMem(int xpg)
{
  xpg += XF_PAGE;
  // Save copy - clear dirty ...
  uint8_t pg[FLASH_PAGE_SIZE];
  xmem.saveMem();
  erasePort(xpg);
  // Save all XMemory ...
  write8Port(xpg, (uint8_t*)xmem.mem, xmem.size());
  // Save checksum too ...
  int chkAddr = pageAdjust(PAGE1(xpg) + xmem.size());
  pg[0] = xmem.chkSum();
  writeFlash(chkAddr, pg, FLASH_PAGE_SIZE);
  printf(" -- Saved XMemory (%d bytes [%02X]) to flash!\n", xmem.size(), xmem.chkSum());
}

void initXMem(int xpg)
{
  xpg += XF_PAGE;
  xmem.mem = __mem;
  uint8_t chk = 0;
  int chkAddr = pageAdjust(PAGE1(xpg) + xmem.size());
  readFlash(PAGE1(xpg), (uint8_t*)xmem.mem, xmem.size());
  readFlash(chkAddr, &chk, 1);
  printf(" -- Read XMemory (%d bytes [%02X]) from flash!\n", xmem.size(), chk);
  // Save copy - clear dirty - check checksum ...
  xmem.saveMem();
  if( xmem.chkSum() != chk ) {
    printf("XMem checksum failure: %02X != %02X!\n", chk, xmem.chkSum());
    printf("ERROR: MEMORY LOST of XMemory!\n");
    memset((void*)xmem.mem, 0, xmem.size());
    xmem.saveMem();
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
  CModule *m = modules[page];
  if (m->isInserted()) {
    m->setRam();
    printf("Add QRAM[%X] @ %04X - %04X\n", page, page*PAGE_SIZE, (page*PAGE_SIZE)|PAGE_MASK);
  }
}

void loadPage(int page, uint16_t *img, int bank = 0)
{
  // Read flash image
  readPort(page+(bank?NR_PAGES:0), img);
  int nErr = 0; // Nr of errors
  int nClr = 0; // Nr of erased words
  // Check that image is valid (10 bits)
  for(int i=0; i<PAGE_SIZE; i++) {
    if( img[i] > INST_MASK ) {
      nErr++;
      if( img[i] == ADDR_MASK )
        nClr++;
    }
  }
#if 1
  // Verify loaded code
  if( nClr == PAGE_SIZE ) {
    printf("%04X -> Image is cleared!", page*PAGE_SIZE);
  } else {
    if( nErr ) printf("%d errors in image 0x%X!\n", nErr, page);
    printf("%04X ->", page*PAGE_SIZE);
    for(int i=0; i<8; i++)
      printf(" %03X", img[0x000+i]);
    printf(" ...", page*PAGE_SIZE+0xFF0);
    for(int i=0; i<8; i++)
      printf(" %03X", img[0xff8+i]);
  }
  printf("\n");
#endif
  if( !nErr ) {
    modules.add(page, img, bank);
    printf("HaveBank: %d\n", modules[page]->haveBank());
  }
}

void initRoms()
{
#ifdef USE_FLASH
  printf("Flash offset: 0x%X\n", FLASH_TARGET_OFFSET);
  printf("XIP_BASE:     0x%X\n", XIP_BASE);
  printf("Sector size:  0x%X (%d)\n", FLASH_SECTOR_SIZE, FLASH_SECTOR_SIZE);
  printf("Page size:    0x%X (%d)\n", FLASH_PAGE_SIZE, FLASH_PAGE_SIZE);
#endif
  printf("%d bytes total heap\n", getTotalHeap());
  printf("%d bytes free heap\n", getFreeHeap());
  printf("Trace element: %d bytes\n", sizeof(Bus_t));
  printf("Trace buffer: %d bytes\n", sizeof(Bus_t)*NUM_BUS_T);

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
  modules.clearAll();

  // Check for existing images and load them ...
  for (int port = FIRST_PAGE; port <= LAST_PAGE; port++) {
    int pIdx = port - FIRST_PAGE;
    loadPage(port, rom_pages[pIdx]);
  }
#ifdef WAND_EMU
//    readFlash(PAGE1(NR_PAGES), wand_page, FLASH_SECTOR_SIZE);
#endif
  // Load second bank of printer-ROM (if available)
  loadPage(6, bank_page, 1);
  // TBD - Now RAM-page is hardcoded to port C
  qRam(0xC);
}

char *disAsm(int inst, int addr, uint64_t data, uint8_t sync);

// Extract display data from data56
#define CH9_B03(c) ((c >> 0) & 0x00F) // Bit 0-3
#define CH9_B47(c) ((c >> 4) & 0x00F) // Bit 4-7
#define CH9_B8(c)  ((c >> 8) & 0x001) // Bit 9

////////////////////////////////////////////////////////////////////////////////
//
// Core 1 sits in a loop grabbing bus cycles
//

#define RISING_EDGE(SIGNAL) ((last_##SIGNAL == 0) && (SIGNAL == 1))
#define FALLING_EDGE(SIGNAL) ((last_##SIGNAL == 1) && (SIGNAL == 0))

// Do we drive ROM data?
volatile int drive_isa_flag = 0;  // Drive ISA in next cycle
volatile int output_isa = 0;      // Output ongoing on ISA...
volatile int output_data = 0;     // Output ongoing on DATA...

// The data we drive
uint16_t drive_isa = 0;

void handle_bus(volatile Bus_t *pBus);

////////////////////////////////////////////////////////////////////////////////
//
// Core1 main functions
//
// Various different core 1 main functions used to test functionality
//
////////////////////////////////////////////////////////////////////////////////

// Data transfer to core 0

#define NUM_BUS_MASK (NUM_BUS_T-1)
#define INC_BUS_PTR(d) d = (d+1) & NUM_BUS_MASK

volatile int queue_overflow = 0;

volatile int data_wr = 0;
volatile int data_rd = 0;

#define DATA_OUTPUT(x) do { output_data = 1; data56_out = x;} while(0)
volatile uint64_t data56_out = 0;

volatile Bus_t bus[NUM_BUS_T];

volatile uint8_t prtBuf[PRT_BUF_LEN];
volatile uint8_t wprt = 0;
volatile uint8_t rprt = 0;

int last_sync = 0;
int gpio_states = 0;

char dtext[2 * NR_CHARS + 1];
bool bPunct[2 * NR_CHARS + 1];

char cpu2buf[256];
int nCpu2 = 0;
char cpuXbuf[256];
int nCpuX = 0;

extern void dispOverflow(bool bOvf);

void reset_bus_buffer(void)
{
  // Reset buffer if overflow has occured ...
  if( queue_overflow ) {
    queue_overflow = 0;
    data_rd = data_wr = 0;
    dispOverflow(false);
  }
}

bool isXmemAddr(uint16_t addr)
{
#if defined(USE_XFUNC)
  if( (addr >= XMEM_XF_START && addr < XMEM_XF_END) )
    return true;
#if defined(USE_XMEM1) || defined(USE_XMEM2)
  if( (addr >= XMEM_XM1_START && addr < XMEM_XM1_END) )
    return true;
#endif
#if defined(USE_XMEM2)
  if( (addr >= XMEM_XM2_START && addr < XMEM_XM2_END) )
    return true;
#endif
#endif
  return false;
}

// Return XMemAddress in range [1..MaxAddr+1]
int getXmemAddr(uint16_t addr)
{
#if defined(USE_XFUNC)
  if( (addr >= XMEM_XF_START && addr < XMEM_XF_END) )
    return addr - XMEM_XF_START + 1;
#if defined(USE_XMEM1) || defined(USE_XMEM2)
  if( (addr >= XMEM_XM1_START && addr < XMEM_XM1_END) )
    return addr - XMEM_XM1_START + XMEM_XM1_OFFS + 1;
#endif
#if defined(USE_XMEM2)
  if( (addr >= XMEM_XM2_START && addr < XMEM_XM2_END) )
    return addr - XMEM_XM2_START + XMEM_XM2_OFFS + 1;
#endif
#endif
  return 0;
}

CBlinky blinky;
volatile uint64_t  CBlinky::reg[8];    // First 8 registers
volatile uint8_t   CBlinky::reg8[16];  // Last 8 registers (0-7 not used)

CTime mTime;
volatile CRegs_t   CTime::reg[2];    // 56 bits time regs
volatile uint32_t  CTime::interval;  // 20 bits interval timer
volatile uint16_t  CTime::accuracy;  // 13 bits
volatile uint16_t  CTime::status;    // 20 bits

void core1_main_3(void)
{
  static int bit_no = 0;        // Current bit-number in bus cycle
  static int bIsaEn = 0;        // True if ISA output is enabled
  static int isDataEn = 0;      // True if DATA output is enabled
  static int isFiEn = 0;        // True if FI output is enabled
  static uint64_t isa = 0LL;    // Current bit-pattern on the ISA bus
  static uint64_t bit = 0LL;    // Bit mask for current bit in cycle
  static uint64_t data56 = 0LL; // Current bit-pattern on DATA bus
  static uint16_t dataFI = 0;   // State of FI to be sent
  static int sync = 0;          // True if sync detected
  static bool bIsSync = false;  // True if current cycle have sync
  static bool bSelPa = false;   // True if peripheral address is in next cycle
  static uint8_t perph = 0;     // Selected pheripheral
  static bool bSelRam = false;  // True if RAM address is in next cycle
  static uint32_t ramad = 0;    // Current RAM pointer
  static bool bErr = false;     // True if corrupt cycle is detected
  static bool bPrinter = false; // True if printer is selected
  static CModule *mp = NULL;    // Pointer to current module (if selected)
  volatile Bus_t *pBus;         // Pointer into trace buffer
  static int last_data_wr;      // Keep track of trace overflow
  static uint16_t iCnt = 0;     // Cycle counter for trace
  static uint32_t xmAddr = 0;
  static uint32_t blinkyAddr = 0;
  static uint8_t r = 0;
  static uint16_t cmd = 0;
  static uint8_t cmd6 = 0;

  pBus = &bus[data_wr];

  irq_set_mask_enabled(0xffffffff, false);

  last_sync = GPIO_PIN(P_SYNC);

#ifdef MEASURE_TIME
  uint64_t tm = time_us_64();
#endif

  // This is the mail loop where everything happens regarding the BUS
  // All 56 cycles of the bus is taken care of, and data is pulled from
  // the bus or injected on the bus accordingly.
  while (1) {
    // Wait for CLK2 to have a falling edge
    WAIT_FALLING(P_CLK2);

    // NOTE! Bit numbers are out by one as bit_no hasn't been incremented yet.

// Clk1  ____/\_______/\_______/\_______/\_______/\_______/\_______/\_______/\___
// Clk2  _______/\_______/\_______/\_______/\_______/\_______/\_______/\_______/\___
// DATA  |  T51   |  T52   |  T53   |  T54   |  T55   |  T0    |  T1    |  T2    |

    // Drive the FI carry flags for each nibble
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
        if( !bErr ) {
          gpio_put(LED_PIN_B, LED_ON);
        }
        // Prepare data for next round ...
        gpio_put(P_ISA_DRV, drive_isa & 1);
        break;
      case 54-1:
        // Don't drive data any more
        gpio_put(P_ISA_OE, (bIsaEn = DISABLE_OE));
        // ... so no more data after this ...
        output_isa = drive_isa_flag = 0;
        break;
      default: // Drive during bit 43->52
        // Enable ISA driver (if not already enable) ...
        if( bIsaEn == DISABLE_OE )
          gpio_put(P_ISA_OE, (bIsaEn = ENABLE_OE));
        // Expose the bit on the ISA line and prepare the next ...
        gpio_put(P_ISA_DRV, drive_isa & 1);
        drive_isa >>= 1;
      }
    }

    // Wait for CLK1 to have a falling edge
    //WAIT_FALLING(P_CLK1);
    WAIT_RISING(P_CLK1);

    // Another bit, check SYNC to find bit number
    sync = GPIO_PIN(P_SYNC);

    // Increment the bit number (or sync ...)
    if (sync && !last_sync) {
      bIsSync = true;
      bit_no = ISA_SHIFT;
    } else {
      bit_no = (bit_no >= LAST_CYCLE) ? 0 : bit_no+1;
    }

    // NOTE! Bit numbers are now ok since bit_no has been incremented!

    // Save all bits in data and isa reg ...
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
      memset((void*)pBus, 0, sizeof(Bus_t));

      // Check if still busy ...
      blinky.busy();
      break;

    case 7: // 8 first bits are read ...
      if( bSelPa ) {
        // Get the selected peripheral
        perph = data56 & 0xFF;
        bSelPa = false;
      }
      break;

    case 9: // 10 first bits are read ...
      if( bSelRam ) {
        // Get the selected peripheral
        ramad = data56 & 0x3FF;
        bSelRam = false;
      }
      xmAddr = 0;
      blinkyAddr = 0;
      if( !perph ) {
        xmAddr = getXmemAddr(ramad);  // Address = [1..XF_END+1]
        if( (ramad & 0x3F0) == BLINKY_ADDR ) {
          if( ramad == BLINKY_ADDR )
            blinkyAddr = BLINKY_ADDR + 1;
          else
            blinkyAddr = (ramad & 0x0F) + 1;
        }
      }
      break;

    case 29:
      // Got address. If the address is that of the embedded ROM then we
      // flag that we have to put an instruction on the bus later
      pBus->addr = (isa >> 14) & ADDR_MASK;
      mp = modules.at(pBus->addr);
      break;

    case 30:
      // Check if we should emulate any modules ...
      if (mp->isInserted()) {
        pBus->cmd = drive_isa = (*mp)[pBus->addr];
        drive_isa_flag = 1;
      }
      break;

    case 42: // Enable ISA output in next loop (cycle 43)
      if( drive_isa_flag )
        output_isa = 1;
      break;

    case LAST_CYCLE-3:
      // Check blinky timer counter
      if( blinky.tick() ) {
        // Set FI flag when counter reaches zero ...
        setFI(FI_PRT_TIMER);
      }
      break;

    case LAST_CYCLE-2:
      // Check if more data to be read from the wand ...
      if( cBar.available() ) {
        setFI(FI_PBSY | FI_WNDB);
      } else {
        clrFI(FI_WNDB);
      }
      break;

    case LAST_CYCLE-1:
#if 0 // Select FI or debug info in fi-field
      // Report next FI signal to trace ...
      pBus->fi = getFI();
#else
      //pBus->fi = xmAddr-1;
      pBus->fi = ramad;
      //pBus->fi = (blinky.nAlm & 0xFF) << 8;
      //pBus->fi |= blinky.cntTimer & 0xFF;
      //pBus->fi = blinky.flags;
#endif
      // Remember last queue write pointer ...
      last_data_wr = data_wr;
      // Turn off MLDL-led ...
      gpio_put(LED_PIN_B, LED_OFF);
      break;

    case LAST_CYCLE:
      // If bitno == LAST_CYCLE then we have another frame, save the transaction
      // A 56 bit frame has completed, we send some information to core0 so it
      // can update the display and anything else it needs to do ...

      // Assume no data drive any more ...
      output_data = 0;

      // Is instruction fetched from flash?
      if( !pBus->cmd )
        pBus->cmd = (isa >> ISA_SHIFT) & INST_MASK;
      pBus->data = data56 & MASK_56_BIT;
      r = (pBus->cmd >> 6) & 0x0F;
      cmd = pBus->cmd;
      cmd6 = cmd & 0x3F;
#ifdef TRACE_ISA
      pBus->isa = isa;
#endif

      // Check for pending writes to Blinky
      if( blinky.bwr ) {
        // bwr is incremented to distinguish it from zero (0)
        blinky.bwr--;
        if( blinky.bwr == BLINKY_ADDR ) {
          // This is a special DATA WRITE to reg 0x20
          blinky.wrStatus((uint8_t)((pBus->data >> 48LL) & 0xFF));
        } else {
          blinky.write(blinky.bwr, pBus->data, true);
        }
        blinky.bwr = 0;
      }

      // Check for pending writes to XMemory
      if( xmem.bwr ) {
        __mem[xmem.bwr-1] = pBus->data;
        xmem.bwr = 0;
      }

      // Check for pending writes to XMemory
      if( mTime.bwr ) {
        mTime.write(mTime.bwr-1, pBus->data);
        mTime.bwr = 0;
      }

      if( bIsSync ) {
        switch( cmd ) {
        case INST_RAM_SLCT:
          // Fetch selected RAM in the next run ...
          bSelRam = true;
          // ... and reset any chip previously enabled ...
          perph = 0;
          break;
        case INST_PRPH_SLCT:
          // Fetch selected peripheral from DATA in the next run ...
          bSelPa = true;
          break;
        case INST_SEL_PRT:
          bPrinter = true;
          break;
        case INST_ENBANK1:
        case INST_ENBANK2:
        case INST_ENBANK3:
        case INST_ENBANK4:
            mp->bank(cmd);
          break;
        default:
          // If perph == 0 then RAM is accessible
          if( !perph ) {
            // Look for read/write towards the Blinky RAM
            if( blinkyAddr ) {
              // Accessing RAM 0x20-0x2F
              if( cmd == INST_WDATA ) {
                // Note! A write to BLINKY_ADDR (0x20) should be treated special
                // Should be interpreted as a Write Status instruction
                blinky.bwr = blinkyAddr; // (0x20 or 0x01-0x0F) + 1
              } else if( cmd6 == INST_READ_DATA ) {
                DATA_OUTPUT(r < 8 ? blinky.reg[r] : blinky.reg8[r]);
              } else if( cmd6 == INST_WRITE_DATA ) {
                ramad = BLINKY_ADDR | r;  // Update ram select pointer
                if( blinky.flags & BLINKY_RAM_ENABLE)
                  blinky.bwr = r+1; // Delayed write ... bwr = [0x0,0xF] + 1
              }
            } else if( xmAddr ) {
              // X-Function module
              if( cmd == INST_WDATA ) {
                xmem.bwr = xmAddr;
              } else if( cmd6 == INST_READ_DATA ) {
                DATA_OUTPUT(__mem[xmAddr-1]);
              } else if( cmd6 == INST_WRITE_DATA ) {
                // Update ram select pointer
                ramad = (ramad & 0x3F0) | r;
                xmAddr = getXmemAddr(ramad);
                xmem.bwr = xmAddr;
              }
            }
          } else {
            // A peripherial is selected ...
            switch( perph ) {
            case WAND_ADDR:
              if( cmd == INST_WANDRD && cBar.available() ) {
                // Output next byte from the wand-buffer!
                DATA_OUTPUT(cBar.get());
              }
              break;
            case TIMR_ADDR:
              if( cmd6 == INST_READ_DATA ) {
                DATA_OUTPUT(mTime.read(r));
              } else if( cmd6 == INST_WRITE_DATA ) {
                // Update ram select pointer
                mTime.bwr = r+1;
              }
              break;
            }
          }
        }
      } else {
        // No real instruction .. (no sync)
        // Handle the printer if it is selected
        if( bPrinter ) {
          int cmd2 = (cmd >> 1) & 0x3;
          switch(cmd2) {
            case NPIC_IR_PRT_WRITE:  // Write ...
              blinky.write(r, pBus->data);
              break;
            case NPIC_IR_PRT_READ:  // Read ...
              DATA_OUTPUT(blinky.read(r));
              break;
            case NPIC_IR_PRT_CMD:  // Func ...
              blinky.func(r);
              break;
          }
          // Check if return-bit is set
          if( cmd & 1)
            bPrinter = false;
        }
      }
#ifdef MEASURE_TIME
      {
        uint16_t dt = (uint16_t)(time_us_64() - tm);
        // Floating input-pin ... ?
        // A normal cycle (56 clock cycles @ 366kHz) should take ~158us (0x9E).
        // So, if outside that range, we have an invalid cycle ...
        if( dt < 0x40 || dt > 0x100 ) {
          bErr = true;
          break;
        }
        //pBus->fi = dt;
      }
      bErr = false;
#endif
#ifdef MEASURE_COUNT
      // Report FI-status to trace ...
      pBus->cnt |= iCnt++;
#endif
      // Report selected peripheral to trace ...
      pBus->data |= ((uint64_t)perph)<<PA_SHIFT;

      if( bIsSync )
        pBus->cmd |= CMD_SYNC;
      bIsSync = false;

      // Setup FI-signal for next round ...
      dataFI = getFI();

      // Cleanup, fix trace buffer pointer and
      // check for overflow ...
      isa = data56 = 0LL;

      INC_BUS_PTR(data_wr);
      if (data_rd == data_wr) {
        // No space left in ring-buffer ...
        queue_overflow++;
        // Restore last pointer (overwriting this cycle)
        data_wr = last_data_wr;
        // Turn at least disasm off ...
        bTrace &= ~TRACE_DISASM;
      }
      // Enable DATA for next cycle ... ?
      if( output_data != isDataEn ) {
        isDataEn = output_data;
        gpio_put(P_DATA_OE, isDataEn);
      }
      break;
    }
    last_sync = sync;
  }
}

void post_handling(uint16_t addr)
{
  // Check if we have more data to send and that the buffer is empty ...
  // cBar.empty() if all data in buffer have been read
  // FI_WNDB (T2) is low if buffer is empty
  if( bWdata && cBar.empty() && !chkFI(FI_WNDB) ) {
    // We have no more data to send!
    // Clear Wand carry
    _clrFI_PBSY();
    bWdata = false;
  }
#if 0
  if( (iGetNextWndData > iSendNextWndData) && cBar.empty() && !chkFI(FI_WNDB) ) {
    // We have more data to send!
    iSendNextWndData++;
    wand_scan();
  }
#endif
  if (wprt != rprt) {
    uint8_t c = prtBuf[rprt++];
    switch(c) {
      case 0x04:
      case 0x0A:
        cdc_send_printport('\n');
        cdc_send_printport('\r');
        break;
      case 0x86: cdc_send_printport('*'); break;
      default:
//        if( c < 0x20 || c > 0x7F )
//          printf("%02X ", c);
//        else
          cdc_send_printport(c);
    }
    //cdc_send_printport(c);
#ifdef USE_PIO
    send_to_printer(c);
#endif
  }
}

#ifdef DUMP_CYCLE
void dumpCycle(Bus_t *p)
{
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
}
#endif
unsigned int skipClk = 0;

void process_bus(void)
{
  BrkMode_e br; // Breakpoint

  // Pointer to current trace element ...
  volatile Bus_t *pb = NULL;

  // Process data coming in from the bus via core 1
  while (data_wr != data_rd) {
    // Point at current element ...
    pb = &bus[data_rd];

    if( cdc_connected(ITF_TRACE) ) {
#ifdef DUMP_CYCLE
      dumpCycle(pb);
#else
      // Handle the bus traffic
  /*    if( pb->addr < 0x3000 ) {
        if( IS_TRACE() )
          skipClk = 0;
        END_TRACE();
      } else
        START_TRACE();*/
      br = brk.isBrk(pb->addr);
      if( br == BRK_START )
        START_TRACE();
      handle_bus(pb);
      skipClk++;
      if( br == BRK_END ) {
        if( IS_TRACE() )
          skipClk = 0;
        END_TRACE();
      }
#endif
    }
    // Any handling outside bus traceing
    // Note that instructions can be missed if the buffer is overflowed!
    // Better handle these cases here ...
    post_handling(pb->addr);

    // Point at next element ...
    INC_BUS_PTR(data_rd);

    if( disp41.needRendering() ) {
      // Render only if buffer is not too full ...
      int remaining = (data_wr - data_rd) + (-((int) (data_wr <= data_rd)) & NUM_BUS_MASK);
      if( remaining < (NUM_BUS_MASK>>2))
        disp41.render();
    }
    cdc_flush(ITF_TRACE);
    tud_task();
  }
}

////////////////////////////////////////////////////////////////////////////////
//
// This function is passed all of the traffic on the bus. It can do
// various things with that.
//

uint64_t dreg_a = 0, dreg_b = 0, dreg_c = 0;
bool display_on = false;
int base = 0;

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
  char cc = 0;
  char cl = 0;

  for (int i = 0; i < NR_CHARS; i++) {

    int b = ((NR_CHARS - 1) - i) << 2;

    cc =   (dreg_a >> b) & 0x0F;
    cc |= ((dreg_b >> b) & 0x03) << 4;
    cl =  ((dreg_b >> b) & 0x0C) >> 2;

    if ((dreg_c >> b) & 1) {
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

  dtext[j] = '\0';

#if CF_DISPLAY_LCD
  UpdateLCD(dtext, bPunct, display_on);
#endif
}

void updateDispReg(uint64_t data, uint8_t r)
{
  Inst_t  *inst = &inst50cmd[r];

  uint64_t *ra = (inst->reg & REG_A) ? &dreg_a : NULL;
  uint64_t *rb = (inst->reg & REG_B) ? &dreg_b : NULL;
  uint64_t *rc = (inst->reg & REG_C) ? &dreg_c : NULL;

#if CF_DBG_DISP_INST
  printf("\n%s %016llX -->", inst50disp[r], data);
#endif

  if ((inst->shft & D_LONG) && inst->len == 48) {
    if (ra) *ra = data & (MASK_48_BIT);
    if (rb) *rb = data & (MASK_48_BIT);
    if (rc) *rc = data & (REG_C_48_MASK);
  } else {
    int n = (inst->shft & D_LONG) ? 48 / inst->len : 1;
    for (int i = 0; i < n; i++) {
      if (inst->shft & SHIFT_R) {
        // Shift RIGHT
        if (ra) *ra = (*ra >> 4) | (((uint64_t)CH9_B03(data)) << 44);
        if (rb) *rb = (*rb >> 4) | (((uint64_t)CH9_B47(data)) << 44);
        if (rc) *rc = (*rc >> 4) | (((uint64_t)CH9_B8(data)) << 44);
      } else {
        // Shift LEFT
        if (ra) *ra = (*ra << 4) | (CH9_B03(data));
        if (rb) *rb = (*rb << 4) | (CH9_B47(data));
        if (rc) *rc = (*rc << 4) | (CH9_B8(data));
      }
      data >>= inst->len; // shift 8 or 12 bits ...
    }
    if (ra) *ra &= MASK_48_BIT;
    if (rb) *rb &= MASK_48_BIT;
    if (rc) *rc &= REG_C_48_MASK;
  }
  dump_dregs();
}

void rotateDispReg(uint8_t r)
{
  Inst_t  *inst = &inst70cmd[r];

  uint64_t *ra = (inst->reg & REG_A) ? &dreg_a : NULL;
  uint64_t *rb = (inst->reg & REG_B) ? &dreg_b : NULL;
  uint64_t *rc = (inst->reg & REG_C) ? &dreg_c : NULL;

#if CF_DBG_DISP_INST
  printf("\n%s -->", inst70disp[r]);
#endif

  for (int i = 0; i < inst->len; i++) {
    if (inst->shft & SHIFT_R) {
      // Shift RIGHT
      if (ra) ROTATE_RIGHT(*ra);
      if (rb) ROTATE_RIGHT(*rb);
      if (rc) ROTATE_RIGHT(*rc);
    } else {
      // Shift LEFT
      if (ra) ROTATE_LEFT(*ra);
      if (rb) ROTATE_LEFT(*rb);
      if (rc) ROTATE_LEFT(*rc);
    }
  }
  dump_dregs();
}

// Keep track of selected peripheral ...
int peripheral_ce = 0;
void setPeripheral(int pa)
{
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
}
int selectedPeripheral(void)
{
  return peripheral_ce;
}
void printPeripheral(void)
{
  switch (selectedPeripheral()) {
  case DISP_ADDR:
  case WAND_ADDR:
  case TIMR_ADDR:
  case CRDR_ADDR:
    nCpu2 += sprintf(cpu2buf + nCpu2, "%c ", "TCDW"[selectedPeripheral() - TIMR_ADDR]);
    break;
  default:
    nCpu2 += sprintf(cpu2buf + nCpu2, "  ");
  }
}

// Handle bus-trace ...
void handle_bus(volatile Bus_t *pBus)
{
  static int oCnt = -1;
  static int pending_data_inst = 0;
  static int oAddr = ADDR_MASK;

//  static uint64_t rr[16];
//  static uint64_t bStat = 0;

  int addr = pBus->addr;
  int inst = pBus->cmd & INST_MASK;
  int sync = (pBus->cmd & CMD_SYNC) ? 1 : 0;
  int fi   = pBus->fi;
  int cnt  = pBus->cnt;
  uint64_t data56 = pBus->data & MASK_56_BIT;
#ifdef TRACE_ISA
  uint64_t isa = pBus->isa;
#endif

  nCpu2 = 0;

  setPeripheral((pBus->data & PA_MASK) >> PA_SHIFT);

#ifdef MEASURE_COUNT
  if( IS_TRACE() ) {
    if( oCnt == -1)
      oCnt = cnt-1;
    oCnt = ++oCnt & 0xFFFF;
    if( oCnt != cnt ) {
      nCpu2 += sprintf(cpu2buf+nCpu2, "\n\r\n\r###### SKIPPED %d TRACE CYCLES #########################\n\r", skipClk);
      pending_data_inst = 0;
      oCnt = cnt;
    }
  }
#endif
#if 0
  for( int i=0; i<16; i++ ) {
    if( rr[i] != blinkyRAM[i] ) {
      printf("\n\rBLKR%d: %014llX\n\r", i, blinkyRAM[i]);
      rr[i] = blinkyRAM[i];
    }
  }
  if( bStat != blinky[0] ) {
    printf("\n\rBLINKY Stat: %014llX\n\r", blinky[0]);
    bStat = blinky[0];
  }
#endif
#define CPUX_PRT
#ifdef CPUX_PRT
  // Any printouts from the other CPU ... ?
  if (cpuXbuf[0]) {
    printf("\nCPUX[%s]", cpuXbuf);
    cpuXbuf[0] = 0;
  }
#endif


#if 1
#ifdef QUEUE_STATUS
  int remaining = (data_wr - data_rd) + (-((int) (data_wr <= data_rd)) & NUM_BUS_MASK);
  nCpu2 += sprintf(cpu2buf+nCpu2, "\n\r%c", queue_overflow ? '#' : ' ');
#else
  nCpu2 += sprintf(cpu2buf+nCpu2, "\n\r");
#endif
#else
#ifdef QUEUE_STATUS
  int remaining = (data_wr - data_rd) + (-((int) (data_wr <= data_rd)) & NUM_BUS_MASK);
  if( queue_overflow )
    nCpu2 += sprintf(cpu2buf+nCpu2, "\n\r[****]");
  else
    nCpu2 += sprintf(cpu2buf+nCpu2, "\n\r[%4d]", remaining);
#else
  nCpu2 += sprintf(cpu2buf+nCpu2, "\n\r");
#endif
#endif

  printPeripheral();

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
  char mode = '?';
  if( base == 10 )  mode = 'd';
  if( base == 16 )  mode = 'x';
  if( PAGE(addr) < 3 ) {
    // Dump information about which quad rom (easy find in VASM)
    nCpu2 += sprintf(cpu2buf + nCpu2, "%04X>%04X|%014llX %c%c %04X (Q%2d:%03X) %03X",
            cnt, fi, data56, sync ? '*':' ', mode, addr, q, addr & 0x3FF, inst);
  } else {
    // No need in an external rom
    nCpu2 += sprintf(cpu2buf + nCpu2, "%04X>%04X|%014llX %c%c %04X           %03X",
            cnt, fi, data56, sync ? '*':' ', mode, addr, inst);
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
    {
      int wAddr = (data56 >> 12) & ADDR_MASK;
      int wDat = data56 & INST_MASK;
      CModule *ram = modules.at(wAddr);
      // TBD - Should we allow writes to images not inserted?
      if (ram->isLoaded() && ram->isRam()) {
        // Page active and defined as ram - update!
        ram->write(wAddr, wDat);
        nCpu2 = sprintf(cpu2buf, "   W> %03X -> @%04X !!\n\r", wDat, wAddr);
      } else {
        nCpu2 = sprintf(cpu2buf, "   W> %03X -> @%04X (Not RAM!)\n\r", wDat, wAddr);
      }
    }
    break;
  default:
    if( IS_FULLTRACE() ) {
      char *dp = disAsm(inst, addr, data56, sync);
      if( oAddr != addr ) {
        // Make room for newline ...
        memmove(cpu2buf+1, cpu2buf, ++nCpu2);
        cpu2buf[0] = 0x0A;  // Add newline if a jump occurred ...
      }
      if( dp )
        nCpu2 += sprintf(cpu2buf + nCpu2, " - %s", dp);
      oAddr = addr + 1;
    }
    break;
  }

#ifndef DISABLE_DISPRINT
  if( IS_TRACE() ) {
    //printf("%s", cpu2buf);
    //cdc_printf_(ITF_TRACE, "%s", cpu2buf);
    cdc_send_string(ITF_TRACE, cpu2buf, nCpu2);
    //cdc_send_console(cpu2buf);
  }
#endif
  cpu2buf[0] = 0;
  nCpu2 = 0;

  // Check for a pending instruction from the previous cycle
  switch (pending_data_inst) {
  case INST_WRITE_ANNUNCIATORS:
#if CF_DBG_DISP_INST
    printf("\n\r%s %03llX", "WRTEN", data56 & 0xFFF);
#endif
    UpdateAnnun((uint16_t)(data56 & 0xFFF), true);
    break;

  case INST_SRLDA:  // All ...50 instructions
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
    if( IS_TRACE() && selectedPeripheral() == WAND_ADDR )
      printf(" WB -> %03X", (int)(data56 & 0xFFF));
    break;
  }
  // Clear pending flag ...
  pending_data_inst = 0;

  // Check for instructions
  if (sync) {
    switch (inst) {
    case INST_SETHEX:
      base = 16;
      break;
    case INST_SETDEC:
      base = 10;
      break;
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
      printf("\n\r%s_SLCT: PA=%02X", inst == INST_PRPH_SLCT ? "PRPH" : "RAM", pa);
      break;
#endif
    case INST_WRITE:
    case INST_FETCH:
      // Handle instruction in next cycle ...
      pending_data_inst = inst;
      break;
    case INST_POWOFF:
      dump_dregs();
      // Any QRAM that needs to be saved ... ?
      for(int p=FIRST_PAGE; p<=LAST_PAGE; p++) {
        CModule *m = modules[p];
        if( m->isLoaded() && m->isRam() && m->isDirty() ) {
          saveRam(p);
          printf("Updated QRAM in page #X\n\r", p);
        }
      }
      if( xmem.dirty() ) {
        extern void dump_xmem(void);
        dump_xmem();
        saveXMem(0);
      }
      break;
    }
  }

  if( selectedPeripheral() && sync) {
    switch(selectedPeripheral()) {
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
          printf("\n\r%s", inst50disp[inst >> 6]);
#endif
          pending_data_inst = inst;
          break;
        case 000070:
#if CF_DBG_DISP_INST
          printf("\n\r%s", inst70disp[inst >> 6]);
#endif
          rotateDispReg(inst >> 6);
          break;
        }
      }
      break;
    }
  }
}

