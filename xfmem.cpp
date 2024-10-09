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


#ifdef USE_XF_MODULE
CXFM xmem;
static volatile XMem_t __mem;
#endif

CMem ram(0);
static volatile uint64_t __ram[QUAD_MEM_MOD_SIZE];

extern void erasePort(int n, bool bPrt=true);
extern void write8Port(int n, uint8_t *data, int sz);
extern void readFlash(int offs, uint8_t *data, uint16_t size);

#ifdef USE_XFUNC
void saveXMem(int xpg)
{
  xpg += XF_PAGE;
  // Save copy - clear dirty ...
  xmem.saveMem();
  erasePort(xpg);
  // Save all XMemory incl chkSum ...
  write8Port(xpg, (uint8_t*)&xmem.m_mem, xmem.size());
  sprintf(cbuff, " -- Saved XMemory (%d bytes [%02X(%02X)]) to flash!\n\r", xmem.size(), xmem.chkSum(), xmem.m_ram->chkSum);
  cdc_send_string_and_flush(ITF_CONSOLE, cbuff);
}

void initXMem(int xpg)
{
  xpg += XF_PAGE;
  xmem.m_ram = &__mem;
  readFlash(PAGE1(xpg), (uint8_t*)xmem.m_ram, xmem.size());
  // Save copy - clear dirty - check checksum ...
  xmem.saveMem();
  if( xmem.chkSum() == xmem.m_ram->chkSum ) {
    sprintf(cbuff, " -- Read XMemory (%d bytes [%02X]) from flash!\n\r", xmem.size(), xmem.m_ram->chkSum);
  } else {
    sprintf(cbuff, "XMem checksum failure: %02X != %02X!\n\r", xmem.m_ram->chkSum, xmem.chkSum());
    cdc_send_string(ITF_CONSOLE, cbuff);
    sprintf(cbuff, "ERROR: MEMORY LOST of XMemory!\n\r");
    memset((void*)xmem.m_ram->mem, 0, xmem.memSize());
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
  __mem.mem[bwr-1] = *dta;
  bwr = 0;
}
uint32_t CXFM::write(uint32_t addr, int r) {
  addr = (addr & 0x3F0) | r;
  bwr = getXmemAddr(addr);
  return addr;
}
uint64_t CXFM::read(uint32_t addr, int r) {
  return __mem.mem[addr-1];
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
