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

#define NAME_W 10 // Width when dumping name
#define MOD_NAME_LEN  16
// Class to hold information about image in a bank
class CBank {
  uint16_t  *m_img;         // Pointer to image in flash or RAM
  char       m_name[MOD_NAME_LEN+1];  // Name of the module
  FL_Head_t *m_fat;         // Pointer to FAT entry
  uint8_t    m_filePage;    // Page in the file image (MOD file)
public:
  void set(uint16_t *i, char *n, FL_Head_t *f, uint8_t p) {
    m_img = i;
    m_fat = f;              // Point to original file in FFS
    m_filePage = p;         // Which page in the original file
    strncpy(m_name, n, MOD_NAME_LEN); // Name of module/bank
  }
  void clear(void) {
    m_img = NULL;
  }
  void erase(void) {
    m_img = NULL;
    memset(m_name, 0, (MOD_NAME_LEN+1)*sizeof(char));
    m_fat =  NULL;
  }
  void free(void) {
    if( m_img )
      delete[] m_img;
    clear();
  }
  FL_Head_t *fat(void) {
    return m_fat;
  }
  void fat(FL_Head_t *ft) {
    m_fat = ft;
  }
  int type(void) {
    return m_fat->type;
  }
  void page(uint8_t pg) {
    m_filePage = pg;
  }
  uint8_t   page(void) {
    return m_filePage;
  }
  uint16_t *image(void) {
    return m_img;
  }
  char *name(void) {
    return m_name;
  }
  int dump(char *buf) {
    int n = 0;
    if( fat() ) {
      switch( type() ) {
      case FL_ROM: n += sprintf(buf+n, "R"); break;
      case FL_MOD: n += sprintf(buf+n, "M"); break;
      case FL_RAM: n += sprintf(buf+n, "Q"); break;
      default:     n += sprintf(buf+n, "?");
      }
      n += sprintf(buf+n, ":%*s %X:%2d{%03X}", NAME_W, name(), image(), page(), image()[0]);
    } else {
      n += sprintf(buf+n, " ");
    }
    return n;
  }
};

// This class handles a single module (with up to 4 banks)
class CModule {
  CBank     m_banks[NR_BANKS];
  uint16_t *m_img;                  // Pointer to current image
  uint8_t   m_flgs;                 // Status of the module
  uint8_t   m_cBank;                // Currently selected bank
public:
  void dump(int p);
  CModule() {eraseData(); }
  void freeBank(int bank);
  void remove();
  void eraseData();
  void clearBanks(void) {
    for(int b=0; b<NR_BANKS; b++)
      m_banks[b].clear();
  }
  // Get data to save to flash
  void getConfig(ModuleConfig_t *mc) {
    for(int b=0; b<NR_BANKS; b++) {
      mc->fat[b] = m_banks[b].fat();
      mc->filePage[b] = m_banks[b].page();
    }
  }
  void setConfig(ModuleConfig_t *mc) {
    for(int b=0; b<NR_BANKS; b++) {
      m_banks[b].fat(mc->fat[b]);
      m_banks[b].page(mc->filePage[b]);
    }
  }
  // Connect an image to the correct bank of the module
  void setImage(uint16_t *img, int bank, FL_Head_t *fat, char *name, int page);
  // Select current bank given bank-instruction
  void selectBank(int bank);
  // Return pointer to bank image
  uint16_t *getImage(int bank) {
    return m_banks[bank].image();
  }
  // Get name of module in given bank
  char *getName(int bank = 0) {
    return m_banks[bank].name();
  }
  // True if more than the default bank
  bool haveBank(int bank = 0) {
    if( bank )
      return m_banks[bank].image() ? true : false;
    return (m_banks[1].image()||m_banks[2].image()||m_banks[3].image()) ? true : false;
  }
  // Module is updated and not saved
  bool isDirty(void) {
    return m_flgs & IMG_DIRTY;
  }
  int type(int b) {
    return m_banks[b].type();
  }
  FL_Head_t *fat(int b) {
    return m_banks[b].fat();
  }
  //Return the page number in the (MOD) file
  int filePage(int b) {
    return m_banks[b].page();
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
  bool isQROM(void) {
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
  // Read 10-bit instruction from current image given address
  uint16_t read(uint16_t addr) {
    return m_img[addr & PAGE_MASK] & INST_MASK;
  }
  // Read 10-bit instruction from current image given address
  uint16_t operator [](uint16_t addr) {
    return m_img[addr & PAGE_MASK] & INST_MASK;
  }
  // Write 10-bit instruction to current image given address
  void writeQROM(uint16_t addr, uint16_t dta);
};

// This class handles all ports and inserted modules
class CModules {
  CModule m_modules[NR_PAGES];
  Setup_t m_setup;
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

  // Save the current config to flash as config #n
  void saveConfig(int n, char *desc=NULL);
  // Read and restore config #n from flash
  bool readConfig(int n);
  // Read and restore config according to saved setup
  bool readConfig(void) {
    return readConfig(m_setup.config);
  }
  // Delete config #n from list of configurations
  void deleteConfig(int n);

  // Save given configuration as current setup to flash
  // Saved in same page after list of configuration
  void saveSetup(int set);
  // Restore setup from flash
  // This is read during startup to restore current state
  bool restore(void);

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
  bool isQROM(int port) {
    return m_modules[port].isQROM();
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

extern CModules modules;

#endif//__MODULE_H__