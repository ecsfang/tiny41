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
#include "hardware/flash.h"
#ifdef USE_PIO
#include "ir_led.h"
#endif
#include "usb/cdc_helper.h"
#include "module.h"
#include "modfile.h"

//#define RESET_FLASH
//#define RESET_RAM

extern CDisplay    disp41;

CBlinky blinky;
volatile uint64_t  CBlinky::reg[8];    // First 8 registers
volatile uint8_t   CBlinky::reg8[16];  // Last 8 registers (0-7 not used)

#ifdef USE_TIME_MODULE
CTime mTime;
volatile CRegs_t   CTime::reg[2];    // 56 bits time regs
volatile uint32_t  CTime::interval;  // 20 bits interval timer
volatile uint16_t  CTime::accuracy;  // 13 bits
volatile uint16_t  CTime::status;    // 20 bits
#endif//USE_TIME_MODULE

#ifdef USE_XF_MODULE
CXFM xmem;
static volatile uint64_t __mem[XMEM_XF_SIZE+XMEM_XM1_SIZE+XMEM_XM2_SIZE];
#endif


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
  blinky.fi(&carry_fi);
  return carry_fi & FI_MASK;
}

volatile int wDelayBuf = 0;

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

// Make space for information about all flash images
CModules modules;

// Read flash into ram with right endian
void readPort(int n, uint16_t *data)
{
  // Point at the wanted flash image ...
  const char *fp8 = (char*)flashPointer(PAGE1(n));
  if( get_file_format(fp8) ) {
    // Read MOD format
    extract_roms(fp8, modules.port(n));
    return;
  }
  const uint16_t *fp16 = (uint16_t*)fp8;
  for (int i = 0; i < FLASH_SECTOR_SIZE; ++i) {
    // Swap order to get right endian of 16-bit word ...
    data[i] = swap16(*fp16++);
  }
}
// Read flash into ram whitout swapping
void readFlash(int offs, uint8_t *data, uint16_t size)
{
  // Point at the wanted flash image ...
  const uint8_t *fp = flashPointer(offs);
  memcpy(data, fp, size);
  cdc_printf_(ITF_TRACE, " <-- Read flash @ %08X %d bytes\n\r", offs, size);
}

// Save a ram image to flash (emulate Q-RAM)
void saveRam(int port, int ovr = 0)
{
  if( port >= FIRST_PAGE && port < NR_PAGES ) {
    if (ovr || modules.isDirty(port)) {
      modules.clear(port);
      erasePort(port, false);
      writePort(port, modules.image(port,0));
      cdc_printf_(ITF_TRACE, "Wrote RAM[%X] to flash\n\r", port);
    }
  } else {
      cdc_printf_(ITF_TRACE, "Invalid page! [%X]\n\r", port);
  }
}

#ifdef USE_XFUNC
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
#endif//USE_XFUNC

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

void loadPage(int page, int bank = 0)
{
  // Read flash image
  uint16_t *img = new uint16_t[PAGE_SIZE];
  if( !img )
    return;

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
    cdc_printf_(ITF_TRACE, "%04X -> Image is cleared!", page*PAGE_SIZE);
  } else {
    if( nErr ) cdc_printf_(ITF_TRACE, "%d errors in image 0x%X!\n\r", nErr, page);
    cdc_printf_(ITF_TRACE, "%04X ->", page*PAGE_SIZE);
    for(int i=0; i<8; i++)
      cdc_printf_(ITF_TRACE, " %03X", img[0x000+i]);
    cdc_printf_(ITF_TRACE, " ...", page*PAGE_SIZE+0xFF0);
    for(int i=0; i<8; i++)
      cdc_printf_(ITF_TRACE, " %03X", img[0xff8+i]);
  }
  cdc_printf_(ITF_TRACE, "\n\r");
#endif
  if( !nErr ) {
    modules.add(page, img, bank);
    if( modules[page]->haveBank() )
      cdc_printf_(ITF_TRACE, "HaveBank: %d\n\r", modules[page]->haveBank());
  } else
    delete[] img;
  cdc_flush(ITF_TRACE);
}

void initRoms()
{

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
    loadPage(port);
  }
  // Load second bank of printer-ROM (if available)
  loadPage(6, 1);
  // Unplug Service ROM if inserted (has to be done manually)
  if( modules.isInserted(4) )
    modules.unplug(4);
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

volatile uint8_t sendBuf[PRT_BUF_LEN];
volatile uint8_t wSend = 0;
volatile uint8_t rSend = 0;
volatile uint16_t sendSize = 0;
volatile uint16_t rxSize = 0;

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

bool bT0Carry = false;

volatile Mode_e cpuMode;
void logC(const char *s);

void core1_main_3(void)
{
  static int bit_no = 0;        // Current bit-number in bus cycle
  static int bIsaEn = 0;        // True if ISA output is enabled
  static int isDataEn = 0;      // True if DATA output is enabled
  static int isFiEn = 0;        // True if FI output is enabled
  static uint64_t isa = 0LL;    // Current bit-pattern on the ISA bus
  static uint64_t bit = 0LL;    // Bit mask for current bit in cycle
  static uint64_t data56 = 0LL; // Current bit-pattern on DATA bus
  static uint8_t fiShift = 0;   // Current shift of FI
  static uint16_t dataFI = 0;   // State of FI to be sent
  static int sync = 0;          // True if sync detected
  static bool bIsSync = false;  // True if current cycle have sync
  static bool bSelPa = false;   // True if peripheral address is in next cycle
  static uint8_t perph = 0;     // Selected pheripheral
  static bool bSelRam = false;  // True if RAM address is in next cycle
  static uint32_t ramad = 0;    // Current RAM pointer
  static bool bErr = false;     // True if corrupt cycle is detected
  static bool bPrinter = false; // True if printer is selected
  static bool bTiny = false;    // True if Tiny41 is selected
  static CModule *mp = NULL;    // Pointer to current module (if selected)
  volatile Bus_t *pBus;         // Pointer into trace buffer
  static int last_data_wr;      // Keep track of trace overflow
  static uint16_t iCnt = 0;     // Cycle counter for trace
  static RamDevice_e ramDev = RAM_DEV;
  static uint32_t ramAddr = 0;
  static uint8_t r = 0;
  static uint16_t cmd = 0;
  static uint8_t cmd6 = 0;
  static bool bPWO = false;
  static bool bPullIsa = false;
  static uint16_t isBase = ARITHM_UKN;

  pBus = &bus[data_wr];

  irq_set_mask_enabled(0xffffffff, false);

  last_sync = GPIO_PIN(P_SYNC);

#ifdef MEASURE_TIME
  uint64_t tick = time_us_64();
#endif


  // This is the main loop where everything happens regarding the BUS
  // All 56 cycles of the bus is taken care of, and data is pulled from
  // the bus or injected on the bus accordingly.
  while (1) {
    // Check if module is active ...
    bPWO = GPIO_PIN_RD(P_PWO);
    if( !bPWO ) {
      // No, we are sleeping ...
      gpio_put(LED_PIN_B, LED_OFF);
      cpuMode = GPIO_PIN(P_SYNC) ? LIGHT_SLEEP : DEEP_SLEEP;
      // Update Blinky after each bus-cycle (160 us)
      uint64_t now = time_us_64();
      if( (uint32_t)(now - tick) >= ONE_BUS_CYCLE ) {
        if( blinky.tick() ) {
          // Time to wake the CPU, pull ISA high until PWO ...
          bPullIsa = true;
        }
        // Reset bus-cycle counter
        tick = now;
      }
      // But don't wake the CPU if we are in deep sleep ...
      if( cpuMode == LIGHT_SLEEP && bPullIsa && !bIsaEn ) {
        // Pull ISA high to wake the calculator ...
        gpio_put(P_ISA_DRV, 1);
        gpio_put(P_ISA_OE, (bIsaEn = ENABLE_OE));
      }
      // PWO is low, so nothing to do, just ...
      continue;
    } else {
      // If ISA is active, then turn it off since PWO is active ...
      if( bPullIsa && bIsaEn ) {
        gpio_put(P_ISA_OE, (bIsaEn = DISABLE_OE));
        bPullIsa = false;
      }
      cpuMode = RUNNING;
      gpio_put(LED_PIN_B, LED_ON);
    }
    if( bErr )
      cpuMode = NO_MODE;

    // Wait for CLK2 to have a falling edge
    WAIT_FALLING(P_CLK2);

    // NOTE! Bit numbers are out by one as bit_no hasn't been incremented yet.

// Clk1  ____/\_______/\_______/\_______/\_______/\_______/\_______/\_______/\___
// Clk2  _______/\_______/\_______/\_______/\_______/\_______/\_______/\_______/\___
// DATA  |  T51   |  T52   |  T53   |  T54   |  T55   |  T0    |  T1    |  T2    |

    if( bit_no >= LAST_CYCLE ) {
      // Clear for next cykle ...
      isa = data56 = 0LL;
      // Setup FI-signal for next round ...
      dataFI = getFI();
      fiShift = 0;
      carry_fi = 0;
      // Check if ISA should be active during T0 (peripherial carry)
      if( bT0Carry && !bIsaEn ) {
        gpio_put(P_ISA_DRV, 1);
        gpio_put(P_ISA_OE, (bIsaEn = ENABLE_OE));
      }
    }

    // Drive the FI carry flags for each nibble
    if( (bit_no & 0b11) == 0b11 ) {
      gpio_put(P_FI_OE, (dataFI>>fiShift) & 1);
      //dataFI >>= 1;
      fiShift++;
    }

    // Do we drive the DATA line (bit 0-55)?
    if (output_data) {
      // Expose the next bit on the data line ...
      gpio_put(P_DATA_DRV, data56_out & 1);
      data56_out >>= 1;
    }
    if( bT0Carry && bIsaEn && bit_no > 2 && bPWO ) {
      gpio_put(P_ISA_OE, (bIsaEn = DISABLE_OE));
      bT0Carry = false;
    }

    // Do we drive the ISA line (bit 43-53)?
    if (output_isa) {
      // Drive the ISA line for theses data bit
      switch (bit_no) {
      case 43-1:
        // Prepare data for next round ...
        gpio_put(P_ISA_DRV, drive_isa & 1);
        break;
      case LAST_CYCLE-2:
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

    // Wait for CLK1 to have a rising edge
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
      tick = time_us_64();
#endif
      pBus = &bus[data_wr];
      memset((void*)pBus, 0, sizeof(Bus_t));
      // Report next FI signal to trace ...
      pBus->fi = dataFI;
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
      ramDev = RAM_DEV;
      if( !perph ) {
        // Check if Blinky RAM is being accessed ...
        if( (ramad & 0x3F0) == BLINKY_ADDR ) {
          ramAddr = (ramad == BLINKY_ADDR) ? BLINKY_ADDR+1 : (ramad&0x0F)+1;
          ramDev = BLINKY_DEV;
        } else {
          // ... else it might be XMemory ...
          if( (ramAddr = getXmemAddr(ramad)) )  // Address = [1..XF_END+1]
            ramDev = XMEM_DEV;
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
      if( blinky.tick() )
        bT0Carry = true;
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
#ifndef LOG_FI // Log something else ...
      //pBus->fi = ramAddr-1;
      //pBus->fi = ramad;
      //pBus->fi = (blinky.nAlm & 0xFF) << 8;
      //pBus->fi = blinky.timer() << 8;
      pBus->fi = blinky.fiFlags();
      //pBus->fi |= blinky.cntTimer & 0xFF;
#endif
      // Prepare command word ...
      // Is instruction fetched from flash?
      if( !pBus->cmd ) {
        // If not, get instruction from the bus ...
        pBus->cmd = (isa >> ISA_SHIFT) & INST_MASK;
      }
      // Local copy of the instruction ...
      cmd = pBus->cmd;
      // Extract the low 6 bit instruction class ...
      cmd6 = cmd & 0x3F;
      // Extract register/address information from the instruction ...
      r = (cmd >> 6) & 0x0F;
      // Save info about sync in logged data ..
      if( bIsSync )
        pBus->cmd |= CMD_SYNC;

      // Remember last queue write pointer ...
      last_data_wr = data_wr;
      break;

    case LAST_CYCLE:
      // If bitno == LAST_CYCLE then we have another frame, save the transaction
      // A 56 bit frame has completed, we send some information to core0 so it
      // can update the display and anything else it needs to do ...

      // Assume no data drive any more ...
      output_data = 0;
      data56 &= MASK_56_BIT;


      // Check for pending writes to Blinky
      if( blinky.bwr ) {
        // bwr is incremented to distinguish it from zero (0)
        blinky.bwr--;
        if( blinky.bwr == BLINKY_ADDR ) {
          // This is a special DATA WRITE to reg 0x20
          blinky.wrStatus((uint8_t)((data56 >> 48LL) & 0xFF));
        } else {
          blinky.write(blinky.bwr, data56, true);
        }
        blinky.bwr = 0;
#ifdef USE_XF_MODULE
      } else if( xmem.bwr ) { // Check for pending writes to XMemory
        __mem[xmem.bwr-1] = data56;
        xmem.bwr = 0;
#endif
#ifdef USE_TIME_MODULE
      } else if( mTime.bwr ) { // Check for pending writes to Time Module
        mTime.write(mTime.bwr-1, data56);
        mTime.bwr = 0;
#endif//USE_TIME_MODULE
      }

      if( bIsSync ) {
        switch( cmd ) {
        case INST_SETHEX:
          isBase = ARITHM_HEX;
          break;
        case INST_SETDEC:
          isBase = ARITHM_DEC;
          break;
        case INST_RAM_SLCT:
          // Fetch selected RAM in the next run ...
          bSelRam = true;
          // ... and reset any chip previously enabled ...
          perph = 0;
          break;
        case INST_PRPH_SLCT:
          // Fetch selected peripheral from DATA in the next run ...
          bSelPa = true;
          blinky.select(false);
          break;
        case INST_SEL_PRT:
          // Blinky Printer is selected
          // Handle NPIC commands in next cykle
          blinky.select(true);
          bPrinter = true;
          break;
        case INST_SEL_TINY:
          // Tiny41 special instructions
          // Handle NPIC commands in next cykle
          bTiny = true;
          break;
        case INST_ENBANK1:
        case INST_ENBANK2:
        case INST_ENBANK3:
        case INST_ENBANK4:
            // Switch bank (if any) ...
            mp->bank(cmd);
          break;
        default:
          // If perph == 0 then RAM is accessible
          // Handle any device in the normal RAM address space
          // - Memory module (not done yet)
          // - XMemory
          // - Blinky RAM
          if( !perph ) {
            // Look for read/write towards the Blinky RAM
            if( ramDev == BLINKY_DEV ) {
              // Accessing RAM 0x20-0x2F
              if( cmd == INST_WDATA ) {
                // Note! A write to BLINKY_ADDR (0x20) should be treated special
                // Should be interpreted as a Write Status instruction
                blinky.bwr = ramAddr; // (0x20 or 0x01-0x0F) + 1
              } else if( cmd6 == INST_READ_DATA ) {
                //DATA_OUTPUT(r < 8 ? blinky.reg[r] : blinky.reg8[r]);
                DATA_OUTPUT( blinky[r] );
              } else if( cmd6 == INST_WRITE_DATA ) {
                // Update ram select pointer
                ramad = BLINKY_ADDR | r;
                if( blinky.flags() & BLINKY_RAM_ENABLE)
                  blinky.bwr = r+1; // Delayed write ... bwr = [0x0,0xF] + 1
              }
            }
#ifdef USE_XF_MODULE
            // Handle emulated XMemory
            else if( ramDev == XMEM_DEV ) {
              // X-Function module
              if( cmd == INST_WDATA ) {
                xmem.bwr = ramAddr;
              } else if( cmd6 == INST_READ_DATA ) {
                DATA_OUTPUT(__mem[ramAddr-1]);
              } else if( cmd6 == INST_WRITE_DATA ) {
                // Update ram select pointer
                ramad = (ramad & 0x3F0) | r;
                xmem.bwr = getXmemAddr(ramad);
              }
            }
#endif
#ifdef USE_QUAD_MODULE
            // Handle emulated QUAD memory module (for HP41C)
            // TBD - Is there a way to detect RAM?
            else if( ramAddr ) {
              // QUAD RAM module
              if( cmd == INST_WDATA ) {
                ram.bwr = ramAddr;
              } else if( cmd6 == INST_READ_DATA ) {
                DATA_OUTPUT(__ram[ramAddr-1]);
              } else if( cmd6 == INST_WRITE_DATA ) {
                // Update ram select pointer
                ramad = (ramad & 0x3F0) | r;
                ramAddr = getRamAddr(ramad);
                ram.bwr = ramAddr;
              }
            }
#endif
          } else {
            // A peripherial is selected ...
            switch( perph ) {
            case WAND_ADDR:
              if( cmd == INST_WANDRD && cBar.available() ) {
                // Output next byte from the wand-buffer!
                DATA_OUTPUT(cBar.get());
              }
              break;
#ifdef USE_TIME_MODULE
            case TIMR_ADDR:
              if( cmd6 == INST_READ_DATA ) {
                DATA_OUTPUT(mTime.read(r));
              } else if( cmd6 == INST_WRITE_DATA ) {
                // Update ram select pointer
                mTime.bwr = r+1;
              }
              break;
#endif//USE_TIME_MODULE
            }
          }
        }
        bIsSync = false;
      } else {
        // No real instruction .. (no sync)
        // Handle the printer if it is selected
        if( bPrinter ) {
          switch(cmd & 0b110) {
            case NPIC_IR_PRT_WRITE:  // Write ...
              blinky.write(r, data56);
              break;
            case NPIC_IR_PRT_READ:  // Read ...
              DATA_OUTPUT(blinky[r]);
              break;
            case NPIC_IR_PRT_CMD:  // Func ...
              blinky.func(r);
              break;
          }
          // Check if return-bit is set
          if( cmd & 1)
            bPrinter = false;
        }
        if( bTiny ) {
          // Handle any specific Tiny41 instructions
          switch(cmd & 0b1110) {
            case NPIC_TINY_RAW_INIT:
              // Init raw file transfer...
              sendSize = data56 & 0xFFFF;
              rxSize = 0;
              wSend = rSend = 0;
              break;
            case NPIC_TINY_RAW_BYTE:
              // Send program byte ...
              sendBuf[wSend++] = data56 & 0xFF;
              break;
            case NPIC_TINY_RAW_DONE:
              // Done with raw file transfer!
              break;
          }
          // Check if return-bit is set
          if( cmd & 1)
            bTiny = false;
        }
      }
#ifdef MEASURE_TIME
      {
        uint16_t tCycle = (uint16_t)(time_us_64() - tick);
        // Floating input-pin ... ?
        // A normal cycle (56 clock cycles @ 366kHz) should take ~158us.
        // So, if outside that range, we have an invalid cycle ...
        if( tCycle < 150 || tCycle > 170 ) {
          bErr = true;
          continue;
        }
        //pBus->fi = tCycle;
      }
      if( !(pBus->cmd|pBus->addr) )
        continue;
      bErr = false;
#endif

      // Prepare logging data ...
      pBus->data = data56;
      // Report selected peripheral to trace ...
      pBus->data |= ((uint64_t)perph)<<PA_SHIFT;
      // Report selected arithmetic mode ...
      pBus->cmd |= isBase;
#ifdef TRACE_ISA
      pBus->isa = isa;
#endif
      // Add bus-counter to log ...
      pBus->cnt = iCnt++;
      // Cleanup, fix trace buffer pointer and
      // check for overflow ...
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

#ifdef USE_TIME_MODULE
static char bcd[14+1];
void CTime::tick()
{
  // 0.01s = 10ms = 10000us
  int x, n,i, tic;
  uint64_t  tm1 = time_us_64();
  if( (tm1 - tm) > 10000 ) {
    tm1 = tm1-tm;
    tic = tm1 / 1000;
    sprintf(bcd, "%014llx", reg[0].clock);

    x = 14;
    for(i=0; i<x; i++)
      bcd[i] -= '0';
    while( tic ) {
      n = tic % 10;
      tic /= 10;
      for( int i = 13; n && i>0; i--) {
        bcd[i] += n;
        if( bcd[i] <= 9 )
          break;
        bcd[i] -= 10;
        n = 1;
      }
    }
    reg[0].clock = 0;
    for( i = 0; i<14; i++ )
      reg[0].clock = (reg[0].clock<<4) | bcd[i];
    tm += tm1;
  }
}
#endif//USE_TIME_MODULE

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
  if (wSend != rSend) {
    uint8_t c = sendBuf[rSend++];
    if( rxSize == 0 && c == 0x00 ) {
      // Skip leading NULL bytes
      sendSize--;
    } else {
      char bb[8];
      sprintf(bb,"%02X", c);
      if( rxSize == 0 ) {
        // Send 'S' to start the transfer
        cdc_send_printport('S');
      }
      // Send each byte as ascii
      cdc_send_printport(bb[0]);
      cdc_send_printport(bb[1]);
      if( rxSize == sendSize ) {
        // When done - end transfer with a 'q'
        cdc_send_printport('q');
      }
      cdc_send_printport('\n');
      cdc_send_printport('\r');
      rxSize++;
    }
  }
#ifdef USE_TIME_MODULE
  // Update time in Time module
  mTime.tick();
#endif//USE_TIME_MODULE
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

CPeripherial peripheral;

/*
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
*/

////////////////////////////////////////////////////////////////////////////////
//
// This function is passed all of the traffic on the bus. It can do
// various things with that.
//

// Handle bus-trace ...
void handle_bus(volatile Bus_t *pBus)
{
  static int oCnt = -1;
  static int pending_data_inst = 0;
  static int oAddr = ADDR_MASK;

  int addr = pBus->addr;
  int inst = pBus->cmd & INST_MASK;
  int base = pBus->cmd & ARITHM_MSK;
  int sync = (pBus->cmd & CMD_SYNC) ? 1 : 0;
  int fi   = pBus->fi;
  int cnt  = pBus->cnt;
  uint64_t data56 = pBus->data & MASK_56_BIT;
#ifdef TRACE_ISA
  uint64_t isa = pBus->isa;
#endif

  nCpu2 = 0;

  peripheral.set((pBus->data & PA_MASK) >> PA_SHIFT);

  if( IS_TRACE() ) {
    if( oCnt == -1)
      oCnt = cnt-1;
    oCnt = ++oCnt & 0xFFFF;
    // Check if we missed/skipped any trace logs
    if( oCnt != cnt ) {
      nCpu2 += sprintf(cpu2buf+nCpu2, "\n\r\n\r###### SKIPPED %d TRACE CYCLES #########################\n\r", skipClk);
      pending_data_inst = 0;
      oCnt = cnt;
    }
  }

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

  nCpu2 += peripheral.print(cpu2buf+nCpu2);

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
  // Show current artihmetic mode
  char mode = '?';
  if( base == ARITHM_DEC )  mode = 'd';
  if( base == ARITHM_HEX )  mode = 'x';
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
    if( IS_TRACE() && peripheral.get() == WAND_ADDR )
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
#ifdef USE_XF_MODULE
      if( xmem.dirty() ) {
        extern void dump_xmem(void);
        dump_xmem();
        saveXMem(0);
      }
#endif
      break;
    }
  }

  if( peripheral.get() && sync) {
    switch(peripheral.get()) {
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

