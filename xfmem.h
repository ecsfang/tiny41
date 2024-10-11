#ifndef __XFMEM_H__
#define __XFMEM_H__

#include "core_bus.h"

// Each port contains two 4k flash pages since 10 bits are stored (4k * uint16_t)
// ROM is not packed in flash due to compatibility with ROM file format
// And packed file would still be larger than 4k ...

#ifdef USE_XFUNC
extern void saveXMem(int xpg);
extern void initXMem(int xpg);
extern bool isXmemAddr(uint16_t addr);
extern int getXmemAddr(uint16_t addr);
#endif//USE_XFUNC

#ifdef USE_XF_MODULE
typedef struct {
  uint64_t mem[XMEM_XF_SIZE+XMEM_XM1_SIZE+XMEM_XM2_SIZE];
  uint8_t  chkSum;
} XMem_t;

// Memory for Extended Memory module
class CXFM : public CRamDev {
public:
  XMem_t  m_mem;          // Copy 
  volatile XMem_t *m_ram;
  bool dirty() {
    return memcmp((void*)m_ram->mem, (void*)m_mem.mem, memSize())?true:false;
  }
  void saveMem(void) {
    // Copy to flash image
    memcpy((void*)m_mem.mem, (void*)m_ram->mem, memSize());
    // Update the checksum of the image
    doChksum();
  }
  // Return total size to save in flash (data+chksum)
  int size(void) {
    return (int)sizeof(m_mem);
  }
  // Return total size of memory
  int memSize(void) {
    return (int)sizeof(m_mem.mem);
  }
  int chkSize(void) {
    return (int)sizeof(m_mem.chkSum);
  }
  void doChksum(void) {
    uint8_t *p = (uint8_t*)m_mem.mem;
    m_mem.chkSum = 0;
    for(int i=0; i<memSize(); i++)
      m_mem.chkSum += *p++;
  }
  uint8_t chkSum(void) {
    return m_mem.chkSum;
  }
  void write(uint64_t *dta);
  uint32_t write(uint32_t addr, int r);
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
  // Get the number of emulated memory modules
  int modules(void) {
    return m_mods;
  }
  // True if address is within the memory modules
  bool isAddress(int a) {
    return isEnabled() && a >= m_start && a < m_end;
  }
  uint16_t getAddress(uint16_t a) {
    // No translation needed ...
    return a;
  }
  RamDevice_e devID() { return QUAD_DEV; }
  void write(uint64_t *dta);
  uint32_t write(uint32_t addr, int r);
  uint64_t read(uint32_t addr, int r);
};

extern CMem ram;

#endif//__XFMEM_H__
