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
#include <malloc.h>
#ifdef USE_FLASH
#include "hardware/flash.h"
#endif

//#define RESET_FLASH
//#define RESET_RAM
#define DRIVE_CARRY
#define DRIVE_DATA
#define DRIVE_ISA
#define WAND_EMU

extern CDisplay    disp41;

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

// Indicate to set PBSY next time ...
void _setFI_PBSY(void)
{
  bPBusy = true;
}
void _clrFI_PBSY(void)
{
  bPBusy = false;
}

// Set FI flag (T0-T13)
inline void setFI(int flag)
{
  carry_fi |= flag;
}
inline void clrFI(int flag = FI_MASK)
{
  carry_fi &= ~flag;
}
inline uint16_t getFI(int flag)
{
  return carry_fi & flag;
}
inline uint16_t getFI(void)
{
  if( bPBusy )
    setFI(FI_PBSY);
  else
    clrFI(FI_PBSY);
  return carry_fi & FI_MASK;
}

volatile int wDelayBuf = 0;
volatile uint16_t iGetNextWndData = 0;
volatile uint16_t iSendNextWndData = 0;

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
// Read flash into ram for wand use
void read8Port(int n, uint8_t *data)
{
  // Point at the wanted flash image ...
  const uint8_t *fp = (uint8_t*)(XIP_BASE + PAGE1(n));
  for (int i = 0; i < FLASH_SECTOR_SIZE; ++i) {
    // Swap order to get right endian of 16-bit word ...
    data[i] = *fp++;
  }
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
    read8Port(NR_PAGES, wand_page);
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

#define NUM_BUS_MASK (NUM_BUS_T-1)
#define INC_BUS_PTR(d) d = (d+1) & NUM_BUS_MASK

volatile int queue_overflow = 0;

volatile int data_wr = 0;
volatile int data_rd = 0;

volatile uint64_t data56_out = 0;

volatile Bus_t bus[NUM_BUS_T];

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

static uint64_t blinky[16];

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
  bool bIsSync = false;
  static uint64_t data56 = 0;
  static uint16_t dataFI = 0;
  int last_data_wr;
  static uint16_t romAddr = 0;
  CModule *mp = NULL;
  volatile Bus_t *pBus = &bus[data_wr];
  static uint8_t perph = 0;
  static uint8_t ramad = 0;
  static bool bSelPa = false;
  static bool bSelRam = false;
  static bool bErr = false;
  static uint8_t cnt = 0;
  bool bPrt = false;
  static int nAlm = 0;
  static int outB = 0;

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
    WAIT_FALLING(P_CLK1);

    // Another bit, check SYNC to find bit number
    sync = GPIO_PIN(P_SYNC);

    // Increment the bit number (or sync ...)
    if (sync && !last_sync) {
      bIsSync = true;
      sync_count++;
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
      mp = modules.at(pBus->addr);
      break;

    case 30:
#ifdef DRIVE_ISA
      // Check if we should emulate any modules ...
      if (mp->isInserted()) {
        embed_seen++;
        pBus->cmd = drive_isa = (*mp)[pBus->addr];
        drive_isa_flag = 1;
      }
#endif
      break;
    case 42: // Enable ISA output in next loop (cycle 43)
      if( drive_isa_flag )
        output_isa = 1;
      break;
    case LAST_CYCLE-2:
      // Check if more data to be read from the wand ...
      if( cBar.available() ) {
        setFI(FI_PBSY | FI_WNDB);
      } else {
        clrFI(FI_WNDB);
      }
      if( nAlm ) {
        nAlm--;
        if( !nAlm )
          setFI(FI_ALM);
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
      // If bitno == LAST_CYCLE then we have another frame, save the transaction
      // A 56 bit frame has completed, we send some information to core0 so it can update
      // the display and anything else it needs to

      // Assume no data drive any more ...
      bDataEn = 0;
      output_data = 0;

      if( pBus->addr || isa || pBus->cmd ) {
        // Is instruction fetched from flash?
        if( !pBus->cmd )
          pBus->cmd = (isa >> ISA_SHIFT) & INST_MASK;
        pBus->data = data56 & MASK_56_BIT;
#ifdef TRACE_ISA
        pBus->isa = isa;
#endif

        if( bPrt ) {
            int r = (pBus->cmd >> 6) & 0x0F;
            int cmd = (pBus->cmd >> 1) & 0x3;
            switch(cmd) {
              case 0b00:  // Write ...
                blinky[r] = pBus->data;
                
                switch(r) {
                case 10:  // 2B9 - 1010 111 00 1 r = 10
                  // Writing 0x01 results in clearing of FI[12]
                  if( pBus->data & 0x01 ) {
                    clrFI(FI_TFAIL);
                    clrFI(FI_ALM);
                    nAlm = 69;  // Set flag after some cycles
                  }
                  break;
                case 11:  // 2F9 - 1011 111 00 1 r = 11
                  // Write to out buffer - set buffer flag
                  if( outB )
                    setFI(FI_TFAIL);
                  outB++;
                  break;
                }
                break;
              case 0b01:  // Read ...
                // Enable data driver ...
                output_data = bDataEn = 1;
                // Get next byte from the wand-buffer!
                if( r == 10 )
                  data56_out = blinky[10] - outB;
                else
                  data56_out = blinky[r];
                break;
              case 0b10:  // Func ...
                switch( r ) {
                case 2: // 0BC 0010 111 10 0 r = 2
                case 4: // 13C 0100 111 10 0 r = 4
                  // This result in that r[14] == 0b11.1....
                  blinky[14] = 0b11010000;
                  break;
                case 3: // 0FC 0011 111 10 0 r = 3
                case 5: // 17C 0101 111 10 0 r = 5
                  // This result in that r[14] == 0b10.0....
                  blinky[14] = 0b10000000;
                  break;
                case 7: // 1FC 0111 111 10 0 r = 7
                  // Reset
                  blinky[14] = 0L;
                  clrFI(FI_TFAIL);
                  break;
                case 8: // 23D 1000 111 10 1 r = 8
                  // Reset
                  clrFI(FI_TFAIL);
                  clrFI(FI_ALM);
                  outB = 0;
                  break;
                }
              case 0b11:  // ???
                break;
            }
            if( pBus->cmd & 1)
              bPrt = false;
        } else {
        switch( pBus->cmd ) {
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
          bPrt = true;
          break;
        case INST_ENBANK1:
          if( bIsSync )
            mp->bank(0);
          break;
        case INST_ENBANK2:
          if( bIsSync )
            mp->bank(1);
          break;
#ifdef DRIVE_CARRY
        case INST_WANDRD:
          if( perph == WAND_ADDR && cBar.available() && bIsSync  ) {
            // Enable data driver ...
            output_data = bDataEn = 1;
            // Get next byte from the wand-buffer!
            data56_out = cBar.get();
            if( cBar.empty() ) {
              // Buffer empty, ready to receive next batch of data
              iGetNextWndData++;
            }
          }
          break;
#endif
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
        }
        bErr = false;
#endif
#ifdef MEASURE_COUNT
        // Report FI-status to trace ...
        pBus->fi |= cnt++ << 8;
#endif
        // Report selected peripheral to trace ...
        //pBus->pa = perph;
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
  // Check if we have more data to send and that the buffer is empty ...
  // cBar.empty() if all data in buffer have been read
  // FI_WNDB (T2) is low if buffer is empty
  if( (iGetNextWndData > iSendNextWndData) && cBar.empty() && !getFI(FI_WNDB) ) {
    // We have more data to send!
    iSendNextWndData++;
    wand_scan();
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

#ifdef DUMP_CYCLE
    dumpCycle(pb);
#else
    // Handle the bus traffic
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

  int addr = pBus->addr;
  int inst = pBus->cmd & INST_MASK;
  int sync = (pBus->cmd & CMD_SYNC) ? 1 : 0;
  int fi = pBus->fi;
  int cnt = (pBus->fi>>8) & 0xFF;
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
    oCnt = ++oCnt & 0xFF;
    if( oCnt != cnt ) {
      nCpu2 += sprintf(cpu2buf+nCpu2, "\n\n###### SKIPPED %d TRACE CYCLES #########################\n", skipClk);
      oCnt = cnt;
    }
  }
#endif

#ifdef CPU2_PRT
  // Any printouts from the other CPU ... ?
  if (cpu2buf[0]) {
    printf("\n[%s]", cpu2buf);
    cpu2buf[0] = 0;
  }
#endif


#if 1
#ifdef QUEUE_STATUS
  int remaining = (data_wr - data_rd) + (-((int) (data_wr <= data_rd)) & NUM_BUS_MASK);
  nCpu2 += sprintf(cpu2buf+nCpu2, "\n%c", queue_overflow ? '#' : ' ');
#else
  nCpu2 += sprintf(cpu2buf+nCpu2, "\n");
#endif
#else
#ifdef QUEUE_STATUS
  int remaining = (data_wr - data_rd) + (-((int) (data_wr <= data_rd)) & NUM_BUS_MASK);
  if( queue_overflow )
    nCpu2 += sprintf(cpu2buf+nCpu2, "\n[****]");
  else
    nCpu2 += sprintf(cpu2buf+nCpu2, "\n[%4d]", remaining);
#else
  nCpu2 += sprintf(cpu2buf+nCpu2, "\n");
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
  if( PAGE(addr) < 3 ) {
    // Dump information about which quad rom (easy find in VASM)
    nCpu2 += sprintf(cpu2buf + nCpu2, "%04X|%014llX %c %04X (Q%2d:%03X) %03X",
            fi, data56, sync ? '*':' ', addr, q, addr & 0x3FF, inst);
  } else {
    // No need in an external rom
    nCpu2 += sprintf(cpu2buf + nCpu2, "%04X|%014llX %c %04X           %03X",
            fi, data56, sync ? '*':' ', addr, inst);
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
        printf("\n"); // Add spave if a jump occurred ...
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
  case INST_WRITE_ANNUNCIATORS:
#if CF_DBG_DISP_INST
    printf("\n%s %03llX", "WRTEN", data56 & 0xFFF);
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
          printf("Updated QRAM in page #X\n", p);
        }
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
