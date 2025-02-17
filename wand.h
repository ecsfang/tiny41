#ifndef __WAND_H__
#define __WAND_H__

extern void _clrFI_PBSY(void);
extern void power_on();
extern void _setFI_PBSY(void);

#define BAR_MAXLEN 0x10
class CBarcode {
  volatile uint8_t bb[BAR_MAXLEN]; // Max size of a barcode
  volatile uint8_t nb = 0;         // Bytes left in buffer
  volatile uint8_t pb = 0;         // Byte pointer into buffer
  bool bBusy;
  inline void busy(bool bsy) {
    bBusy = bsy;
    if(bBusy)
      _setFI_PBSY();
    else
      _clrFI_PBSY();
  }
public:
  CBarcode() : bBusy(false) {}
  inline bool empty(void) { return nb == 0; }
  inline bool isDone(void) { return bBusy && empty(); }
  inline bool available(void) { return !empty(); }
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
  void done(void) {
    if( bBusy ) {
      // Clear Wand carry
      busy(false);
    }
  }
  void data(unsigned char *dta) {
    // Start with first row!
    power_on();
    // Set carry to service the wand
    busy(true);
    set(dta);
  }
};

// Hold current barcode to scan
extern CBarcode *pBar;
extern void init_wand(void);

#endif//__WAND_H__