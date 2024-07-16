#ifndef __XFMEM_H__
#define __XFMEM_H__

#include "core_bus.h"

#define USE_XFUNC
#define USE_XMEM1
#define USE_XMEM2

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

// Memory for Extended Memory module
class CXFM {
  uint8_t m_chksum;
public:
  uint64_t m_mem[XMEM_XF_SIZE+XMEM_XM1_SIZE+XMEM_XM2_SIZE];
  volatile uint64_t *mem;
  int bwr;
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
};

void initXMem(int xpg);

#define MEM_MOD_START 0x100
#define MEM_MOD_END   0x200
#define MEM_MOD_SIZE  (MEM_MOD_END-MEM_MOD_START)

// Memory for Quad Memory module
class CMem {
public:
  int bwr;
};

#endif//__XFMEM_H__
