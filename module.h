#ifndef __MODULE_H__
#define __MODULE_H__

// Bit-filed for current module status
enum {
    IMG_NONE = 0x00,
    IMG_INSERTED = 0x01,  // Module is inserted in address space
    IMG_RAM = 0x02,       // Module is configured as RAM (editable)
    IMG_DIRTY = 0x04      // Module is dirty, changes should be saved
};

// Helper to decode the bank instruction
static uint8_t nBank[NR_BANKS] = {0,2,1,3};

// This class handles a single module (with up to 4 banks)
class CModule {
  uint16_t  *m_banks[NR_BANKS];      // Pointer to each bank (up to 4 images)
  char       m_name[NR_BANKS][16+1]; // Name of the module
  int        offset;                 // Offset to file in flash
  uint16_t  *m_img;                  // Pointer to current image
  uint8_t    m_flgs;                 // Status of the module
  uint8_t    m_cBank;                // Currently selected bank
public:
  void dump(int p) {
    sprintf(cbuff, "%X: [%X:%X:%X:%X][%s:%s:%s:%s][%X]-%02X-%d\n\r",
      p,
      m_banks[0],m_banks[1],m_banks[2],m_banks[3],
      m_name[0],m_name[1],m_name[2],m_name[3],
      m_img, m_flgs,m_cBank);
    cdc_send_string_and_flush(ITF_TRACE, cbuff);
  }
  CModule() {
    m_img = NULL;
    memset(m_banks, 0, NR_BANKS*sizeof(uint16_t));
    memset(m_name,  0, NR_BANKS*(16+1));
    m_cBank = 0;
  }
  void removeBank(int bank) {
    if( image(bank) ) {
      delete[] m_banks[bank];
      m_banks[bank] = 0;
      m_name[bank][0] = 0;
    }
  }
  void remove() {
    m_flgs = m_cBank = 0;
    m_img = NULL;
    for(int b=0; b<NR_BANKS; b++)
      removeBank(b);
    memset(m_name,  0, NR_BANKS*(16+1));
  }
  // Connect an image to the correct bank of the module
  void setImage(uint16_t *img, int bank = 0, char *name=NULL) {
    // Remove any previous image and name
    removeBank(bank);
    m_banks[bank] = img;
    m_img = m_banks[0];
    if( name )
      strncpy(m_name[bank], name, 16);
    if( bank == 0 )
      m_flgs = IMG_INSERTED;
  }
  // Select current bank given bank-instruction
  void selectBank(int bank) {
    // Convert from instruction to bank #
    bank = nBank[(bank>>6)&0b11];
    // If bank is present - update image pointer
    if( image(bank) ) {
      m_cBank = bank;
      m_img = image(m_cBank);
    }
  }
  // Return pointer to bank image
  uint16_t *image(int bank) {
    return m_banks[bank];
  }
  //
  char *getName(int bank = 0) {
    return m_name[bank];
  }
  // True if more than the default bank
  bool haveBank(int bank = 0) {
    if( bank )
      return m_banks[bank] ? true : false;
    return (m_banks[1]||m_banks[2]||m_banks[3]) ? true : false;
  }
  // Deselect any bank and pull from address space
  void clr(void) {
    m_img = NULL;
    m_flgs = IMG_NONE;
  }
  // Module is updated and not saved
  bool isDirty(void) {
    return m_flgs & IMG_DIRTY;
  }
  void clear(void) {
    m_flgs &= ~IMG_DIRTY;
  }
  bool isInserted(void) {
    return m_flgs & IMG_INSERTED;
  }
  bool isLoaded(void) {
    return m_img != NULL;
  }
  bool isRam(void) {
    return m_flgs & IMG_RAM;
  }
  void togglePlug(void) {
    m_flgs ^= IMG_INSERTED;
  }
  void plug(void) {
    m_flgs |= IMG_INSERTED;
  }
  void unplug(void) {
    m_flgs &= ~IMG_INSERTED;
  }
  void toggleRam(void) {
    m_flgs ^= IMG_RAM;
  }
  void setRam(void) {
    m_flgs |= IMG_RAM;
  }
  void setRom(void) {
    m_flgs &= ~IMG_RAM;
  }
  // Read 10-bit instruction from image given address
  uint16_t read(uint16_t addr) {
    return m_img[addr & PAGE_MASK] & INST_MASK;
  }
  // Read 10-bit instruction from image given address
  uint16_t operator [](uint16_t addr) {
    return m_img[addr & PAGE_MASK] & INST_MASK;
  }
  // Write 10-bit instruction to current image given address
  void write(uint16_t addr, uint16_t dta) {
    m_img[addr & PAGE_MASK] = dta;
    m_flgs |= IMG_DIRTY;
  }
};

// This class handles all ports and inserted modules
class CModules {
  CModule m_modules[NR_PAGES];
public:
  CModules() {
    clearAll();
  }
  void addImage(int port, uint16_t *img, int bank, char *name) {
	  if( img ) {
      m_modules[port].setImage(img, bank, name);
  	} 
  }
  void dump(void) {
    for(int p=0; p<NR_PAGES; p++)
      m_modules[p].dump(p);
  }
  uint16_t *image(int port, int bank=0) {
    return m_modules[port].image(bank);
  }
  // Remove all modules and clear all settings
  void clearAll() {
    for(int p=0; p<NR_PAGES; p++)
      m_modules[p].remove();
    memset(m_modules, 0, sizeof(CModule) * NR_PAGES);
  }
  void remove(int port) {
    m_modules[port].clr();
	}
  bool isDirty(int port) {
    return m_modules[port].isDirty();
  }
  void clear(int port) {
    m_modules[port].clear();
  }
  bool isInserted(int port) {
    return m_modules[port].isInserted();
  }
  bool isLoaded(int port) {
    return m_modules[port].isLoaded();
  }
  bool isRam(int port) {
    return m_modules[port].isRam();
  }
  void togglePlug(int port) {
    m_modules[port].togglePlug();
  }
  void unplug(int port) {
    m_modules[port].unplug();
  }
  void toggleRam(int port) {
    m_modules[port].toggleRam();
  }
  void setRam(int port) {
    m_modules[port].setRam();
  }
  void setRom(int port) {
    m_modules[port].setRom();
  }
  CModule *port(int p) {
    return &m_modules[p];
  }
  CModule *at(int addr) {
    return port(PAGE(addr));
  }
  CModule *operator [](uint16_t addr) {
    return &m_modules[addr & 0xF];
  }
};

#endif//__MODULE_H__