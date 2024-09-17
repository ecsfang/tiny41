#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/multicore.h"
#include "pico/bootrom.h"
#include "tiny41.h"
#include "ramdev.h"
#include "xfmem.h"
#include "hardware/flash.h"

class CTest : public CRamDev {
};


#ifdef USE_XF_MODULE
CXFM xmem;
static volatile uint64_t __mem[XMEM_XF_SIZE+XMEM_XM1_SIZE+XMEM_XM2_SIZE];
#endif

CMem ram(0);
//unsigned int nMemMods = 0;
static volatile uint64_t __ram[QUAD_MEM_MOD_SIZE];

extern void erasePort(int n, bool bPrt=true);
extern void write8Port(int n, uint8_t *data, int sz);
extern int pageAdjust(int addr);
extern void writeFlash(int offs, uint8_t *data, int sz);
extern void readFlash(int offs, uint8_t *data, uint16_t size);

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
  sprintf(cbuff, " -- Saved XMemory (%d bytes [%02X]) to flash!\n\r", xmem.size(), xmem.chkSum());
  cdc_send_string_and_flush(ITF_CONSOLE, cbuff);
}

void initXMem(int xpg)
{
  xpg += XF_PAGE;
  xmem.mem = __mem;
  uint8_t chk = 0;
  int chkAddr = pageAdjust(PAGE1(xpg) + xmem.size());
  readFlash(PAGE1(xpg), (uint8_t*)xmem.mem, xmem.size());
  readFlash(chkAddr, &chk, 1);
  sprintf(cbuff, " -- Read XMemory (%d bytes [%02X]) from flash!\n\r", xmem.size(), chk);
  // Save copy - clear dirty - check checksum ...
  xmem.saveMem();
  if( xmem.chkSum() != chk ) {
    sprintf(cbuff, "XMem checksum failure: %02X != %02X!\n\r", chk, xmem.chkSum());
    cdc_send_string(ITF_CONSOLE, cbuff);
    sprintf(cbuff, "ERROR: MEMORY LOST of XMemory!\n\r");
    memset((void*)xmem.mem, 0, xmem.size());
    xmem.saveMem();
  }
  cdc_send_string_and_flush(ITF_CONSOLE, cbuff);
}

bool isXmemAddr(uint16_t addr)
{
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
  return false;
}

// Return XMemAddress in range [1..MaxAddr+1]
int getXmemAddr(uint16_t addr)
{
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
  return 0;
}

void CXFM::write(uint64_t *dta) {
  __mem[bwr-1] = *dta;
  bwr = 0;
}
uint32_t CXFM::write(uint32_t addr, int r) {
  addr = (addr & 0x3F0) | r;
  bwr = getXmemAddr(addr);
  return addr;
}
uint64_t CXFM::read(uint32_t addr, int r) {
  return __mem[addr-1];
}
#endif//USE_XFUNC

void CMem::write(uint64_t *dta) {
  __ram[bwr-MEM_MOD_START] = *dta;
  bwr = 0;
}
uint32_t CMem::write(uint32_t addr, int r) {
  addr = (addr & 0x3F0) | r;
  bwr = addr;
  return addr;
}
uint64_t CMem::read(uint32_t addr, int r) {
  return __ram[addr-MEM_MOD_START];
}
