#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "core_bus.h"

#include "module.h"
#include "modfile.h"

extern bool diffPage(uint16_t *p1, uint16_t *p2);

// Helper to decode the bank instruction
static uint8_t nBank[NR_BANKS] = {0,2,1,3};

//*********************************
// class CModule
//*********************************

// Dump the content of the class
void CModule::dump(int p)
{
  int n = sprintf(cbuff, "%X: [ ", p);
  for(int b=0; b<NR_BANKS; b++) {
    n += sprintf(cbuff+n, "[");
    n += m_banks[b].dump(cbuff+n);
    n += sprintf(cbuff+n, "] ");
  }
  sprintf(cbuff+n, "]\n\r");
  cdc_send_string_and_flush(ITF_TRACE, cbuff);
}

// Clear the class
// Assumes all allocations are freed
void CModule::eraseData()
{
  m_img = NULL;
  for(int b=0; b<NR_BANKS; b++)
    m_banks[b].erase();
  m_flgs = m_cBank = 0;
}
// Free any QROM pointers to the given bank
void CModule::freeBank(int bank) {
  if( haveBank(bank) && isQROM() ) {
    m_banks[bank].free();
  }
}
// Remove any plugged in modules
void CModule::remove() {
  // Free all QROM banks for the current page
  for(int b=0; b<NR_BANKS; b++)
    freeBank(b);
  eraseData();    // Clear the whole class
}

// Connect an image to the correct bank of the module
// img  - Pointer to image in flash or RAM
// bank - the target bank for the image
// fat  - Pointer to the FFS entry for the image
// name - The name of the image
// page - The page number on the original file (MOD)
void CModule::setImage(uint16_t *img, int bank, FL_Head_t *fat, char *name, int page) {
  // Update image pointer for this bank
  freeBank(bank);
  m_banks[bank].set(img, name, fat, page);
  m_img = m_banks[0].image();
#ifdef DBG_PRINT
  sprintf(cbuff,"Set image %X @ %X [%d] -> Bank %d <%s>\n\r", img, fat->offs, fat->type, bank, m_name[bank]);
  cdc_send_string_and_flush(ITF_CONSOLE, cbuff);
#endif
  if( bank == 0 )
    m_flgs = IMG_INSERTED;
}

// Select current bank given NUT bank-instruction
void CModule::selectBank(int bank) {
  // Convert from instruction to bank #
  bank = nBank[(bank>>6)&0b11];
  // If bank is present - update image pointer
  if( haveBank(bank) ) {
    m_cBank = bank;
    m_img = getImage(m_cBank);
  }
}

// Write 10-bit instruction to current image given address
void CModule::writeQROM(uint16_t addr, uint16_t dta) {
  if( m_img[addr & PAGE_MASK] != dta ) {
    m_img[addr & PAGE_MASK] = dta;
    dirty(); // Mark for saving later
  }
}

// Save configuration #set in flash
void CModules::saveConfig(int set)
{
  // Check for any dirty QROM pages to update 
  for(int p=FIRST_PAGE; p<NR_PAGES; p++) {
    CModule *cm = &m_modules[p];
    // Check if page is QROM and if changed
    if( cm->isQROM() && cm->isDirty() ) {
      // If so, save all used banks
      for(int b=0; b<NR_BANKS; b++) {
        if( cm->haveBank(b) )
          writeROMMAP(p,b,cm->getImage(b));
      }
      // Saved - clear dirty flag
      cm->clear();
    }
  }
  // Get the configuration to save
  Config_t *conf = new Config_t;
  for(int p=0; p<NR_PAGES; p++)
    m_modules[p].getConfig(&conf->mod[p]);
  // Save the whole object to flash
  writeConfig(conf, set);
  delete conf;
#ifdef DBG_PRINT
  sprintf(cbuff, "Saved config i#%d (%d bytes) @ %X\n\r", set, sizeof(conf), CONF_PAGE);
  cdc_send_string_and_flush(ITF_CONSOLE, cbuff);
#endif
}

// Read configuration #set from flash
// TBD - should we save current if dirty?
bool CModules::readConfig(int set)
{
  bool ret = true;
  // Read the saved configuration ...
  Config_t *conf = new Config_t;
  CFat_t *pFat = new CFat_t();
  if( !::readConfig(conf, set) ) {
    sprintf(cbuff, "Bad configuration for set %d\n\r", set);
    cdc_send_string_and_flush(ITF_CONSOLE, cbuff);
    ret = false;
  }
  if( ret ) {
    // Remove everything from the class ...
    clearAll();
    // Restore all modules configuration
    for(int p=0; ret && p<NR_PAGES; p++) {
      CModule *cm = &m_modules[p];
      // Restore configuration
      cm->setConfig(&conf->mod[p]);
      // Verify and restore flash pages in ROMMAP from FAT for each bank
      // Check that FAT entries really points to correct images
      for(int b=0; ret &&b<NR_BANKS; b++) {
        pFat->init(cm->fat(b));
        if( pFat->fatEntry() ) {
          // This page has a FAT, verify and load image into ROMMAP
          // given FAT entry and page in file to load
          if( extract_mod(pFat, p, cm->filePage(b)) )
            ret = false;
        }
      }
    }
  }
#ifdef DBG_PRINT
  sprintf(cbuff, "Read config #%d (%d bytes) @ %X\n\r", set, sizeof(conf), CONF_PAGE);
  cdc_send_string_and_flush(ITF_CONSOLE, cbuff);
#endif
  delete pFat;
  delete conf;
  return ret;
}
