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
static uint8_t nBank[4] = {0,2,1,3};

// This class handles a single module (with up to 4 banks)
class CModule {
  uint16_t  *m_banks[4];  // Pointer to each bank (up to 4 images)
  uint16_t  *m_img;       // Pointer to current image
  uint8_t   m_flgs;       // Status of the module
//  uint8_t   m_bank;
//  uint8_t   m_port;
public:
  CModule() {
    m_img = NULL;
//    m_bank = 0;
//    m_port = 0;
    for( int i=0; i<4; i++ )
      m_banks[i] = NULL;
  }
  // Connect an image to the correct bank of the module
  void set(uint16_t *image, int bank = 0) {
    // Remove any previous image
    if( m_banks[bank] )
      delete[] m_banks[bank];
    m_banks[bank] = image;
    m_img = m_banks[0];
    m_flgs = IMG_INSERTED;
  }
  // Select current bank given bank-instruction
  void bank(int bank) {
    // Convert from instruction to bank #
    bank = nBank[(bank>>6)&0b11];
    // If bank is present - update image pointer
    if( image(bank) )
      m_img = image(bank);
  }
  // Return pointer to bank image
  uint16_t *image(int bank) {
    return m_banks[bank];
  }
  void port(int p) {
//    m_port = p;
  }
  // True if more than the default bank
  bool haveBank(void) {
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
  void add(int port, uint16_t *image, int bank) {
	  if( image ) {
      m_modules[port].set(image, bank);
      printf("Add ROM @ %04X - %04X [bank %d]\n", port * PAGE_SIZE, (port * PAGE_SIZE)|PAGE_MASK, bank);
  	} 
  }
  uint16_t *image(int port, int bank) {
    return m_modules[port].image(bank);
  }
  void clearAll() {
    memset(m_modules, 0, sizeof(CModule) * NR_PAGES);
    for(int p=0; p<NR_PAGES; p++)
      m_modules[p].port(p);
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