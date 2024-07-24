#ifndef __XFMEM_H__
#define __XFMEM_H__

#include "core_bus.h"

// Each port contains two 4k flash pages since 10 bits are stored (4k * uint16_t)
// ROM is not packed in flash due to compatibility with ROM file format
// And packed file would still be larger than 4k ...
#define PAGEn(n,i) (FLASH_TARGET_OFFSET + (2 * n + i) * FLASH_SECTOR_SIZE)
#define PAGE1(n) PAGEn(n,0)
#define PAGE2(n) PAGEn(n,1)


#define XMEM_XF_START   0x40
#define XMEM_XF_END     0xC0
#define XMEM_XF_SIZE    (XMEM_XF_END-XMEM_XF_START)
#define XMEM_XF_OFFS    0x000
#define XMEM_XM1_START  0x201
#define XMEM_XM1_END    0x2F0
#define XMEM_XM1_SIZE   (XMEM_XM1_END-XMEM_XM1_START)
#define XMEM_XM1_OFFS   XMEM_XF_SIZE
#define XMEM_XM2_START  0x301
#define XMEM_XM2_END    0x3F0
#define XMEM_XM2_SIZE   (XMEM_XM2_END-XMEM_XM2_START)
#define XMEM_XM2_OFFS   (XMEM_XF_SIZE+XMEM_XM1_SIZE)
#define XF_PAGE         (NR_PAGES+1)

#ifdef USE_XFUNC
void saveXMem(int xpg);
void initXMem(int xpg);
bool isXmemAddr(uint16_t addr);
int getXmemAddr(uint16_t addr);
#endif//USE_XFUNC

#ifdef USE_QUAD_MODULE
extern unsigned int nMemMods;
#endif//USE_QUAD_MODULE

#ifdef USE_XF_MODULE
// Memory for Extended Memory module
class CXFM : public CRamDev {
  uint8_t m_chksum;
public:
  uint64_t m_mem[XMEM_XF_SIZE+XMEM_XM1_SIZE+XMEM_XM2_SIZE];
  volatile uint64_t *mem;
  bool dirty() {
    return memcmp((void*)mem, (void*)m_mem, sizeof(m_mem))?true:false;
  }
  void saveMem(void) {
    memcpy((void*)m_mem, (void*)mem, sizeof(m_mem));
    doChksum();
  }
  int size(void) {
    return (int)sizeof(m_mem);
  }
  void doChksum(void) {
    uint8_t *p = (uint8_t*)m_mem;
    m_chksum = 0;
    for(int i=0; i<size()/sizeof(uint8_t); i++)
      m_chksum += *p++;
  }
  uint8_t *pChkSum(void) {
    return (uint8_t*)&m_chksum;
  }
  uint8_t chkSum(void) {
    return m_chksum;
  }
  int chkSize(void) {
    return (int)sizeof(m_chksum);
  }
  void write(uint64_t *dta);
  uint32_t write(uint32_t addr, int r);
  void delaydWrite(uint32_t addr) { bwr = addr; }
  uint64_t read(uint32_t addr, int r);
  bool isAddress(int a) {
    return isXmemAddr(a);
  }
  uint16_t getAddress(uint16_t a) {
    return getXmemAddr(a);
  }
  RamDevice_e devID() { return XMEM_DEV; }
};
extern CXFM xmem;
#endif

#ifdef USE_QUAD_MODULE

#define MEM_MOD_START     0x100             // Start of first memory module
#define MEM_MOD_SIZE      0x40              // Size of one memory module
#define QUAD_MEM_MOD_SIZE (MEM_MOD_SIZE*4)  // Max size of memory module

// Memory for Quad Memory module
class CMem : public CRamDev {
  int m_mods;
  int m_start;
  int m_end;
public:
  int bwr;  // True if a delayed write to the device
  CMem(int m) {
    modules(m);
  }
  // Set number of emulated memory modules
  void modules(int x) {
    m_mods = x;
    m_start = MEM_MOD_START;
    m_end = m_start + m_mods*MEM_MOD_SIZE;
  }
  // True if address is within the memory modules
  bool isAddress(int a) {
    return nMemMods && a >= m_start && a < m_end;
  }
  uint16_t getAddress(uint16_t a) {
    // No translation needed ...
    return a;
  }
  RamDevice_e devID() { return QUAD_DEV; }

  void write(uint64_t *dta);
  uint32_t write(uint32_t addr, int r);
  void delaydWrite(uint32_t addr) { bwr = addr; }
  uint64_t read(uint32_t addr, int r);
};

extern CMem ram;
#endif

#endif//__XFMEM_H__
