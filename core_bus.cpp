#include <stdio.h>
#include <string.h>
#include <time.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/sync.h"
#include "pico/multicore.h"
#include "pico/flash.h"
#include "tiny41.h"
#include "core_bus.h"
#include "blinky.h"
#include "disasm.h"
#include "serial.h"
#include <malloc.h>
#ifdef USE_PIO
#include "ir_led.h"
#endif
#include "usb/cdc_helper.h"

#include "module.h"
#include "modfile.h"

//#define RESET_FLASH
//#define RESET_RAM

CFat_t fat; // Holds the current FAT entry

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

// Special command buffer from Tiny
volatile char g_cmd[32];
volatile int  g_pCmd = 0;
volatile int  g_num = 0;
volatile int  g_ret = 0;
volatile bool g_do_cmd = 0;

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

void swapPage(uint16_t *dta, int n);

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
  _clrFI_PBSY();
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

// Writing to flash can only be done page wise!
int pageAdjust(int addr)
{
  if( addr % FLASH_PAGE_SIZE ) {
    addr |= FLASH_PAGE_SIZE-1;
    addr++;
  }
  return addr;
}

#define FLASH_LOCK_TIMEOUT_MS 5*1000
typedef struct {
  int     addr;
  uint8_t *buf;
} flash_write_t;

bool diffPage(void *p1, void *p2)
{
  return memcmp(p1, p2, sizeof(uint16_t)*ROM_SIZE) ? true : false;
}

bool invalidFlashPtr(int fptr)
{
  return fptr < (FLASH_START+XIP_BASE) || fptr > (FLASH_SIZE+XIP_BASE);
}

static void ffs_write_flash(void *param) {
  flash_write_t *tmp = (flash_write_t *)param;
  if( invalidFlashPtr(tmp->addr) ) {
      sprintf(cbuff, "ffs_write_flash: Bad address: %08X\n\r", tmp->addr);
      cdc_send_string_and_flush(ITF_CONSOLE, cbuff);
      return;
  }
  // Offset the flash pointer
  int fp1 = tmp->addr-XIP_BASE;
  int fp2 = tmp->addr-XIP_BASE+PG_SIZE;
  uint32_t ints = save_and_disable_interrupts();
  flash_range_erase(fp1, PG_SIZE);
  flash_range_erase(fp2, PG_SIZE);
  flash_range_program(fp1, tmp->buf, PG_SIZE);
  flash_range_program(fp2, tmp->buf+PG_SIZE, PG_SIZE);
  restore_interrupts(ints); // Note that a whole number of sectors must be erased at a time.
}

static void write_flash_page(void *param) {
  flash_write_t *tmp = (flash_write_t *)param;
  // Offset the flash pointer
  int fp = tmp->addr-XIP_BASE;
  uint32_t ints = save_and_disable_interrupts();
  flash_range_erase(fp, PG_SIZE);
  flash_range_program(fp, tmp->buf, PG_SIZE);
  restore_interrupts(ints); // Note that a whole number of sectors must be erased at a time.
}

// Earse a port - two pages (8KB) of flash
void erasePort(int n, bool bPrt=true)
{
  if( bPrt ) {
    int i = sprintf(cbuff, "\n\rErasing target region %X ", n);
    sprintf(cbuff+i, "-- [%05X - %05X]\n\r", PAGE1(n), PAGE2(n) + FLASH_SECTOR_SIZE);
    cdc_send_string_and_flush(ITF_CONSOLE, cbuff);
  }
  uint32_t ints = save_and_disable_interrupts();
  flash_range_erase(PAGE1(n), FLASH_SECTOR_SIZE);
  flash_range_erase(PAGE2(n), FLASH_SECTOR_SIZE);
  restore_interrupts(ints); // Note that a whole number of sectors must be erased at a time.
  if( bPrt ) {
    cdc_send_string_and_flush(ITF_CONSOLE, (char*)"Done!\n\r");
  }
}

void writeFlash(int offs, uint8_t *data, int sz)
{
  uint32_t ints = save_and_disable_interrupts();
  flash_range_program(offs, data, sz);
  restore_interrupts(ints);
  sprintf(cbuff, " --> Write flash @ %08X %d bytes\n\r", offs, sz);
  cdc_send_string_and_flush(ITF_CONSOLE, cbuff);
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

uint16_t *writePage(flash_write_t *pDta)
{
  int r = flash_safe_execute(write_flash_page, pDta, FLASH_LOCK_TIMEOUT_MS);
  if (r != PICO_OK) {
    sprintf(cbuff, "error calling write_flash_page: %d", r);
    cdc_send_string_and_flush(ITF_CONSOLE, cbuff);
#ifdef DBG_PRINT
  } else {
    sprintf(cbuff, "Wrote flash 0x%08X @ 0x%08X:%X\n\r", pDta->buf, pDta->addr, PG_SIZE);
    cdc_send_string_and_flush(ITF_CONSOLE, cbuff);
#endif
  }
  return (uint16_t*)pDta->addr;
}

// Write data to given flash port (page and bank) in ROMMAP
// One ROM page is always written (4K 10-bit data - two flash pages)
// Returns pointer to flash image (or NULL if failed)
uint16_t *writeROMMAP(int p, int b, uint16_t *data)
{
  flash_write_t tmp = {FFS_PAGE(p,b), (uint8_t*)data};
  if( invalidFlashPtr(tmp.addr) ) {
      sprintf(cbuff, "writeROMMAP: Bad address: %08X\n\r", tmp.addr);
      cdc_send_string_and_flush(ITF_CONSOLE, cbuff);
      return NULL;
  }
  // Check if images differs ...
  if( diffPage((void*)tmp.addr, tmp.buf) ) {
    // Yes, so lets update the image to flash
    writePage(&tmp);
  }
  // Return address to the flash image
  return (uint16_t*)tmp.addr;
}

uint16_t *writePage(int addr, uint8_t *data)
{
  flash_write_t tmp = {addr, data};
  return writePage(&tmp);
}

// Function to calculate a simple checksum
static uint32_t calcChecksum(const uint8_t *data, size_t len) {
    uint32_t checksum = 0xBADC0DE; // Start with a non-zero value
    for (size_t i = 0; i < len-sizeof(uint32_t); i++) {
        checksum += data[i];
    }
    return ~checksum;
}

// Function to verify the checksum
static bool verifyChecksum(const uint8_t *data, size_t len, uint8_t expected_checksum) {
    uint32_t chk = calcChecksum(data, len);
    return chk == expected_checksum;
}

bool readConfig(Config_t *data, int set)
{
  uint32_t cp = CONF_PAGE + CONF_OFFS(set);
  memcpy(data, (uint8_t*)cp, CONF_SIZE);
  uint32_t chk = calcChecksum((const uint8_t*)data, sizeof(Config_t));
#ifdef DBG_PRINT
  sprintf(cbuff, "Read config(%d) @ 0x%08X:%X\n\r", set, cp, CONF_SIZE);
  cdc_send_string_and_flush(ITF_CONSOLE, cbuff);
#endif
  return chk == data->chkSum;
}

// Read the config flashpage and updated the
// corresponding part and write back the page
uint16_t *writeConfigPage(int offs, void *data, int size)
{
  uint16_t *ret = NULL;
  uint8_t *pg = new uint8_t[FLASH_SECTOR_SIZE];
  // Read whole config page
  memcpy(pg, (uint8_t*)CONF_PAGE, FLASH_SECTOR_SIZE);
  // Update current part of the page
  memcpy(pg+offs, data, size);
  // Write back updated configuration
  ret = writePage(CONF_PAGE, pg);
  delete[] pg;
  return ret;
}

// Update configuration #set in config page
uint16_t *writeConfig(Config_t *data, int set, bool bChk)
{
  if( bChk ) {
    // Yes, update checksum
    uint32_t chk = calcChecksum((const uint8_t*)data, sizeof(Config_t));
    data->chkSum = chk;
  }
  return writeConfigPage(CONF_OFFS(set), data, CONF_SIZE);
}

// Update the setup information in the config page
uint16_t *writeSetup(Setup_t *data)
{
  uint32_t chk = calcChecksum((const uint8_t*)data, sizeof(Setup_t));
  data->chkSum = chk;
  return writeConfigPage(SETUP_OFFS, data, SETUP_SIZE);
}

bool readSetup(Setup_t *data)
{
  uint32_t cp = CONF_PAGE + SETUP_OFFS;
  memcpy(data, (uint8_t*)cp, SETUP_SIZE);
  uint32_t chk = calcChecksum((const uint8_t*)data, sizeof(Setup_t));
#ifdef DBG_PRINT
  sprintf(cbuff, "Read setup @ 0x%08X:%X\n\r", cp, SETUP_SIZE);
  cdc_send_string_and_flush(ITF_CONSOLE, cbuff);
#endif
  return chk == data->chkSum;
}

/*void writePort(int n, uint16_t *data)
{
  // Swap 16-bit word to get right endian for flash ...
  swapPage(data, FLASH_SECTOR_SIZE);
  uint8_t *dp = (uint8_t*)data;
  printf("\nProgramming target region %X ", n);
  printf("-- [%05X - %05X]\n", PAGE1(n), PAGE2(n) + FLASH_SECTOR_SIZE);
  write8Port(n, (uint8_t*)data, 2*FLASH_SECTOR_SIZE);
  swapPage(data, FLASH_SECTOR_SIZE);
  printf("Done!\n");
}*/


// Make space for information about all flash images
CModules modules;

// Read flash into ram whitout swapping
void readFlash(int offs, uint8_t *data, uint16_t size)
{
  // Point at the wanted flash image ...
  const uint8_t *fp = (const uint8_t*)(XIP_BASE + offs);
  if( invalidFlashPtr((int)fp) ) {
    sprintf(cbuff, "BAD FLASH POINTER: %p!\n\r", fp);
    cdc_send_string_and_flush(ITF_CONSOLE, cbuff);
    return;
  }
  memcpy(data, fp, size);
}

// Save a given ram image to flash (emulate Q-RAM)
// TBD - This must be updated to update file in FFS!
// TBD - Should check all banks as well
// If loaded from ROM, then that page should be updated
// If loaded from MOD, then we must check which page etc that
// needs to be updated.
void saveRam(int port, int ovr = 0)
{
  int n = 0;
  if( port >= FIRST_PAGE && port < NR_PAGES ) {
    if (ovr || modules.isDirty(port)) {
      modules.clear(port);
      writeROMMAP(port, 0, modules.getImage(port));
      n = sprintf(cbuff, "Wrote RAM[%X] to flash\n\r", port);
    }
  } else {
      n = sprintf(cbuff, "Invalid page! [%X]\n\r", port);
  }
  if( n )
    cdc_send_string_and_flush(ITF_CONSOLE, cbuff);
  sprintf(cbuff, "Save RAM @ page %d implemented yet! [%X]\n\r", port);
  cdc_send_string_and_flush(ITF_CONSOLE, cbuff);
}


// Swap 16-bit word (n - number of 16-bit words)
void swapPage(uint16_t *dta, int n)
{
  for (int i = 0; i < n; dta++, i++)
    *dta = __builtin_bswap16(*dta);
}

bool loadModule(const char *mod, int page)
{
  // Check that the module is loaded in flash
  // FAT is updated to point to this entry
  if( !fat.find(mod) )
    return false;
  return extract_mod(&fat, page) ? false : true;
}

void initRoms(void)
{
  // Read the saved setup and load
  // previous configuration from previous session
  modules.restore();
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
bool bET11967 = false;
volatile uint64_t data2FI = 0;

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
  static uint32_t ramAddr = 0;
  static uint8_t r = 0;
  static uint16_t cmd = 0;
  static uint8_t cmd6 = 0;
  static bool bPWO = false;
  static bool bPullIsa = false;
  static uint16_t isBase = ARITHM_UKN;
  static CRamDev *pRamDev = NULL;

  pBus = &bus[data_wr];

  if( !flash_safe_execute_core_init() ) {
    sprintf(cbuff, "*** Flash initfailed! ***\n\r");
    cdc_send_string_and_flush(ITF_CONSOLE, cbuff);
  }

//  irq_set_mask_enabled(0xffffffff, true);

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
      // Setup FI-signal for next round ...
      if( bET11967 ) {
        // Service module installed, shortcut DATA and FI
        data2FI = ~data56;
        gpio_put(P_FI_OE, (data2FI>>(fiShift*4)) & 1);
        dataFI = 0;
      } else {
        dataFI = getFI();
      }
      // Clear for next cykle ...
      isa = data56 = 0LL;
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
      if( bET11967 )
        gpio_put(P_FI_OE, (data2FI>>(fiShift*4)) & 1);
      else
        gpio_put(P_FI_OE, (dataFI>>fiShift) & 1);
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
      pRamDev = NULL;
      if( !perph ) {
        // Check if Blinky RAM is being accessed ...
        if( blinky.isAddress(ramad) ) {
          pRamDev = &blinky;
        }
        else if( ram.isEnabled() && ram.isAddress(ramad) ) {
          pRamDev = &ram;
        }
#ifdef USE_XF_MODULE
        // ... else it might be XMemory ...
        else if( xmem.isAddress(ramad) ) {
          pRamDev = &xmem;
        }
#endif
      }
      break;

    case 11:
      if( pRamDev ) {
        ramAddr = pRamDev->getAddress(ramad);
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
      if( bIsSync ) {
        pBus->cmd |= CMD_SYNC;
        switch( cmd ) {
        case INST_SETHEX:
          isBase = ARITHM_HEX;
          cmd = 0;
          break;
        case INST_SETDEC:
          isBase = ARITHM_DEC;
          cmd = 0;
          break;
        case INST_RAM_SLCT:
          // Fetch selected RAM in the next run ...
          bSelRam = true;
          // ... and reset any chip previously enabled ...
          perph = 0;
          cmd = 0;
          break;
        case INST_PRPH_SLCT:
          // Fetch selected peripheral from DATA in the next run ...
          bSelPa = true;
          blinky.select(false);
          cmd = 0;
          break;
        case INST_SEL_PRT:
          // Blinky Printer is selected
          // Handle NPIC commands in next cykle
          blinky.select(true);
          bPrinter = true;
          cmd = 0;
          break;
        case INST_SEL_TINY:
          // Tiny41 special instructions
          // Handle NPIC commands in next cykle
          bTiny = true;
          cmd = 0;
          break;
        case INST_ENBANK1:
        case INST_ENBANK2:
        case INST_ENBANK3:
        case INST_ENBANK4:
            // Switch bank (if any) ...
            mp->selectBank(cmd);
          cmd = 0;
          break;
        }
      }
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

      // Check for any pending writes
      if (pRamDev ) {
	      pRamDev->write(&data56);
#ifdef USE_TIME_MODULE
      } else if( mTime.bwr ) { // Check for pending writes to Time Module
        mTime.write(&data56);
#endif//USE_TIME_MODULE
      }

      if( bIsSync ) {
        if( cmd ) {
          // If perph == 0 then RAM is accessible
          // Handle any device in the normal RAM address space
          // - Memory module (not done yet)
          // - XMemory
          // - Blinky RAM
          if( !perph ) {
            if( pRamDev ) {
              // Update any RAM device (QUAD, XMEM etc)
              if( cmd == INST_WDATA ) {
                pRamDev->delaydWrite(ramAddr);
              } else if( cmd6 == INST_READ_DATA ) {
                DATA_OUTPUT(pRamDev->read(ramAddr,r));
              } else if( cmd6 == INST_WRITE_DATA ) {
                // Update ram select pointer
                ramad = pRamDev->write(ramad, r);
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
              blinky.write(r, &data56);
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
#if 1
          switch(cmd & 0x3FE) {
            case 0x03A:
              DATA_OUTPUT(g_ret);
              break;
            case 0b1100:
              g_do_cmd = 1;
              _setFI_PBSY();  // Indicate that we are busy!
              break;
            case 0b1000:
              g_num = data56 & 0xFF; // Save command byte
              break;
            case 0b0100:
              g_cmd[g_pCmd++] = data56 & 0xFF; // Save character
              g_cmd[g_pCmd] = 0;
              break;
            case 0b0000:
              g_pCmd = 0;           // Reset command
              g_cmd[g_pCmd] = 0;
              break;
          }
#else
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
#endif
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
  if( g_do_cmd ) {
    switch(g_do_cmd ) {
      case 1:
        sprintf(cbuff, "Load page with [%s] @ %d\n\r", g_cmd, g_num);
        cdc_send_string_and_flush(ITF_CONSOLE, cbuff);
        g_ret = loadModule((char*)g_cmd,g_num) ? 0 : 1;
        _clrFI_PBSY(); // Done with the plugging!
        break;
    }
    g_do_cmd = 0;
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
  const Inst_t  *inst = &inst50cmd[r];

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
  const Inst_t  *inst = &inst70cmd[r];

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

////////////////////////////////////////////////////////////////////////////////
//
// This function is passed all of the traffic on the bus. It can do
// various things with that.
//

const char arithMode[] = " dx?";

// Handle bus-trace ...
void handle_bus(volatile Bus_t *pBus)
{
  static int oCnt = -1;
  static int pending_data_inst = 0;
  static int oAddr = ADDR_MASK;
  static char mode = arithMode[ARITHM_UKN];   // Dec or Hex mode

  int addr = pBus->addr;
  int inst = pBus->cmd & INST_MASK;
  int base = (pBus->cmd & ARITHM_MSK) >> ARITHM_SHFT;
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

  uint8_t quad = (addr >> 10) & 0b1111;
#ifdef TRACE_ISA
  if( PAGE(addr) < 3 ) {
    // Dump information about which quad rom (easy find in VASM)
    nCpu2 += sprintf(cpu2buf + nCpu2, " %014llX %014llX | %04X (Q%2d:%03X) %03X",
            data56, isa, addr, quad, addr & 0x3FF, inst);
  } else {
    // No need in an external rom
    nCpu2 += sprintf(cpu2buf + nCpu2, " %014llX %014llX | %04X           %03X",
            data56, isa, addr, inst);
  }
#else
#if 1
  mode = arithMode[base];
  // Show current artihmetic mode
//  if( base == ARITHM_DEC )  mode = 'd';
//  if( base == ARITHM_HEX )  mode = 'x';

  nCpu2 += sprintf(cpu2buf + nCpu2, "%04X>%04X|%014llX %c%c %04X ",
            cnt, fi, data56, sync ? '*':' ', mode, addr);

  if( PAGE(addr) < 3 ) {
    // Dump information about which quad rom (easy find in VASM)
    nCpu2 += sprintf(cpu2buf + nCpu2, "(Q%2d:%03X) %03X", quad, addr & 0x3FF, inst);
  } else {
    // No need in an external rom
    nCpu2 += sprintf(cpu2buf + nCpu2, "          %03X", inst);
  }
#else
  if( PAGE(addr) < 3 ) {
    // Dump information about which quad rom (easy find in VASM)
    nCpu2 += sprintf(cpu2buf + nCpu2, " %014llX %X %c| %04X (Q%2d:%03X) %03X",
            data56, fi, sync ? '*':' ', addr, quad, addr & 0x3FF, inst);
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
      if (ram->isLoaded() && ram->isQROM()) {
        // Page active and defined as ram - update!
        ram->writeQROM(wAddr, wDat);
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
    cdc_send_string(ITF_TRACE, cpu2buf);
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
      // Any QROM that needs to be saved ... ?
      // TBD - Should this be done here? Or manually?
      for(int p=FIRST_PAGE; p<=LAST_PAGE; p++) {
        CModule *m = modules[p];
        if( m->isLoaded() && m->isQROM() && m->isDirty() ) {
          //saveRam(p);
          //sprintf(cbuff, "Updated QROM in page #X\n\r", p);
          //cdc_send_string_and_flush(ITF_CONSOLE, cbuff);
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
        switch (inst & INST_RW_MASK) {
        case INST_WRITE_DATA:
#if CF_DBG_DISP_INST
          printf("\n\r%s", inst50disp[inst >> 6]);
#endif
          pending_data_inst = inst;
          break;
        case INST_READ_DATA:
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

