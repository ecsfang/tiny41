#ifndef __MODULE_H__
#define __MODULE_H__

extern char *getPageName(const char *flashPtr, int page);

// Bit-filed for current module status
enum {
    IMG_NONE = 0x00,
    IMG_INSERTED = 0x01,  // Module is inserted in address space
    IMG_RAM = 0x02,       // Module is configured as RAM (editable)
    IMG_DIRTY = 0x04      // Module is dirty, changes should be saved
};

// Helper to decode the bank instruction
//static uint8_t nBank[NR_BANKS] = {0,2,1,3};

// This class handles a single module (with up to 4 banks)
class CModule {
  uint16_t  *m_banks[NR_BANKS];      // Pointer to each bank (up to 4 images)
  uint16_t  *m_img;                  // Pointer to current image
  char       m_name[NR_BANKS][16+1]; // Name of the module
  uint8_t    m_flgs;                 // Status of the module
  uint8_t    m_cBank;                // Currently selected bank
  FL_Head_t *m_fat[NR_BANKS];        // Pointer to FAT entry
  uint8_t    m_filePage[NR_BANKS];   // Page in the file image
public:
  void dump(int p);
  CModule() {eraseData(); }
  void removeBank(int bank);
  void removeBanks();
  void remove();
  void eraseData();
  void clearBank(int b) {
    m_banks[b] = NULL;
  }
  void clearBanks(void) {
    memset(m_banks, 0, NR_BANKS*sizeof(uint16_t*));
  }
  void getConfig(ModuleConfig_t *mc) {
    memcpy(mc->fat, m_fat, sizeof(FL_Head_t*)*NR_BANKS);
    memcpy(mc->filePage, m_filePage, sizeof(uint8_t)*NR_BANKS);
  }
  void setConfig(ModuleConfig_t *mc) {
    memcpy(m_fat, mc->fat, sizeof(FL_Head_t*)*NR_BANKS);
    memcpy(m_filePage, mc->filePage, sizeof(uint8_t)*NR_BANKS);
  }
  // Connect an image to the correct bank of the module
  void setImage(uint16_t *img, int bank, FL_Head_t *fat, char *name, int page);
  void setImage(uint16_t *img, int bank);
  // Select current bank given bank-instruction
  void selectBank(int bank);
  // Return pointer to bank image
  uint16_t *getImage(int bank) {
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
  // Module is updated and not saved
  bool isDirty(void) {
    return m_flgs & IMG_DIRTY;
  }
  int type(int b) {
    return m_fat[b]->type;
  }
  FL_Head_t *fat(int b) {
    return m_fat[b];
  }
  //Return the page number in the (MOD) file
  int filePage(int b) {
    return m_filePage[b];
  }
  void clear(void) {
    m_flgs &= ~IMG_DIRTY;
  }
  void dirty(void) {
    m_flgs |= IMG_DIRTY;
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
  void setRam(bool bRam = true) {
    m_flgs &= ~IMG_RAM;
    if( bRam )
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
  void writeQROM(uint16_t addr, uint16_t dta);
};

// This class handles all ports and inserted modules
class CModules {
  CModule m_modules[NR_PAGES];
public:
  CModules() {
    clearAll();
  }
  void addImage(int port, bool bRam, uint16_t *img, int bank, FL_Head_t *fat, char *name, int page=0) {
	  if( img ) {
      m_modules[port].setImage(img, bank, fat, name, page);
      m_modules[port].setRam( bRam );
  	} 
  }
  void dump(void) {
    for(int p=0; p<NR_PAGES; p++)
      m_modules[p].dump(p);
  }
  uint16_t *getImage(int port, int bank=0) {
    return m_modules[port].getImage(bank);
  }
  // Remove all modules and clear all settings
  void clearAll() {
    for(int p=0; p<NR_PAGES; p++)
      m_modules[p].remove();
    memset(m_modules, 0, sizeof(CModule) * NR_PAGES);
  }
  void invalidate() {
    // Clear all bank pointers
    for(int p=0; p<NR_PAGES; p++)
      m_modules[p].clearBanks();
  }

  // Save class to flash at page n
  void saveConfig(int n);
  // Read class from flash at page n
  void readConfig(int n);

//  void remove(int port) {
//    m_modules[port].clr();
//	}
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
  // Get module at address addr (MXXX)
  CModule *at(int addr) {
    return port(PAGE(addr));
  }
  // Get module at given page (X)
  CModule *operator [](uint16_t addr) {
    return &m_modules[addr & 0xF];
  }
};

#endif//__MODULE_H__