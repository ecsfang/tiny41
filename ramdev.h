#ifndef __RAMDEV_H__
#define __RAMDEV_H__

typedef enum {
  NO_RAM_DEV,
  XMEM_DEV,
  QUAD_DEV,
  BLINKY_DEV
} RamDevice_e;


class CRamDev {
  bool m_bEnabled;
public:
  CRamDev() {
    m_bEnabled = false;
  }
  int bwr;
  virtual void write(uint64_t *dta) = 0;
  virtual uint32_t write(uint32_t addr, int r) = 0;
  virtual uint64_t read(uint32_t addr, int r=0) = 0;
  virtual void delaydWrite(uint32_t addr) = 0;
  virtual uint16_t getAddress(uint16_t a) = 0;
  virtual bool isAddress(int a) = 0;
  virtual RamDevice_e devID() = 0;
  void enable(bool bEna) {
    m_bEnabled = bEna;
  }
  inline bool isEnabled(void) {
    return m_bEnabled;
  }
};


#endif