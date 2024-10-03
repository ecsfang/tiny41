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

#define NAME_W 10
// Dump the content of the class
void CModule::dump(int p)
{
  int n = sprintf(cbuff, "%X: [ ", p);
  for(int b=0; b<NR_BANKS; b++) {
    n += sprintf(cbuff+n, "[");
    if( m_fat[b] ) {
      switch( m_fat[b]->type ) {
      case FL_ROM: n += sprintf(cbuff+n, "R"); break;
      case FL_MOD: n += sprintf(cbuff+n, "M"); break;
      case FL_RAM: n += sprintf(cbuff+n, "Q"); break;
      default:     n += sprintf(cbuff+n, "?");
      }
      n += sprintf(cbuff+n, ":%*s %X:%2d{%03X}", NAME_W, m_name[b], m_banks[b], m_filePage[b], m_banks[b][0]);
    } else {
      n += sprintf(cbuff+n, " ");
    }
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
  memset(m_banks, 0, NR_BANKS*sizeof(uint16_t*));
  memset(m_name,  0, NR_BANKS*(16+1)*sizeof(char));
  memset(m_fat,  0, NR_BANKS*(sizeof(FL_Head_t*)));
  m_flgs = m_cBank = 0;
}
// Free any QROM pointers to the given bank
void CModule::removeBank(int bank) {
  if( haveBank(bank) && isRam() ) {
    delete[] m_banks[bank];
    m_banks[bank] = NULL;
  }
}
// Free all QROM banks for the current page
void CModule::removeBanks() {
  for(int b=0; b<NR_BANKS; b++)
    removeBank(b);
}
// Remove any plugged in modules
void CModule::remove() {
  removeBanks();  // Free any RAM pointers
  eraseData();    // Clear the whole class
}
// Save the given ROM image to the specified bank
// Free any previous QROM pointer
void CModule::setImage(uint16_t *img, int bank) {
  removeBank(bank);     // Remove any previous image and name
  m_banks[bank] = img;
  m_img = m_banks[0];
}
// Connect an image to the correct bank of the module
// img  - Pointer to image in flash or RAM
// bank - the target bank for the image
// fat  - Pointer to the FFS entry for the image
// name - The name of the image
// page - The page number on the original file (MOD)
void CModule::setImage(uint16_t *img, int bank, FL_Head_t *fat, char *name, int page) {
  setImage(img, bank);      // Update image pointer for this bank
  m_fat[bank] = fat;        // Point to original file in FFS
  m_filePage[bank] = page;  // Which page in the original file
  strncpy(m_name[bank], name, 16);  // Remember the name
#ifdef DBG_PRINT
  sprintf(cbuff,"Set image %X @ %X [%d] -> Bank %d <%s>\n\r", img, fat->offs, fat->type, bank, m_name[bank]);
  cdc_send_string_and_flush(ITF_CONSOLE, cbuff);
#endif
  if( bank == 0 )
    m_flgs = IMG_INSERTED;
}
// Select current bank given bank-instruction
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

// Save configuration in flash at page n
// TBD - page n is not used - only one configuration saved
void CModules::saveConfig(int set)
{
  // Check for any dirty QROM pages to update 
  for(int p=FIRST_PAGE; p<NR_PAGES; p++) {
    CModule *cm = &m_modules[p];
    // Check if page is QROM and if changed
    if( cm->isRam() && cm->isDirty() ) {
      // If so, save all used banks
      for(int b=0; b<NR_BANKS; b++) {
        uint16_t *img = cm->getImage(b);
        if( img ) {
          writeROMMAP(p,b,img);
        }
      }
      // Clear dirty flag
      cm->clear();
    }
  }
  // Get the configuration to save
  Config_t conf;
  for(int p=0; p<NR_PAGES; p++)
    m_modules[p].getConfig(&conf.mod[p]);
  // Save the whole object to flash
  //writePage(CONF_PAGE, (uint8_t*)&m_conf); //this);
  writeConfig(&conf, set);
#ifdef DBG_PRINT
//  sprintf(cbuff, "Saved config (%d bytes) @ %X\n\r", sizeof(*this), CONF_PAGE);
  sprintf(cbuff, "Saved config (%d bytes) @ %X\n\r", sizeof(conf), CONF_PAGE);
  cdc_send_string_and_flush(ITF_CONSOLE, cbuff);
#endif
}

// Read configuration from flash at page n
// TBD - Only one configuration saved now (n not used)
void CModules::readConfig(int set)
{
  // Read the saved configuration ...
  Config_t conf;
  if( !::readConfig(&conf, set) ) {
    sprintf(cbuff, "Bad configuration for set %d\n\r", set);
    cdc_send_string_and_flush(ITF_CONSOLE, cbuff);
    return;
  }
  // Remove everything from the class ...
  clearAll();

  for(int p=0; p<NR_PAGES; p++)
    m_modules[p].setConfig(&conf.mod[p]);

#ifdef DBG_PRINT
  sprintf(cbuff, "Read config (%d bytes) @ %X\n\r", sizeof(conf), CONF_PAGE);
  cdc_send_string_and_flush(ITF_CONSOLE, cbuff);
#endif
  // Verify and restore flash pages in ROMMAP from FAT
  // Check that FAT entries really points to correct images
  for(int p=FIRST_PAGE; p<NR_PAGES; p++) {
    CModule *cm = &m_modules[p];
    for(int b=0; b<NR_BANKS; b++) {
      CFat_t pFat(cm->fat(b));
      if( pFat.fatEntry() ) {
        // This page has a FAT, verify and load image into ROMMAP
        // given FAT entry and page in file to load
        // If QROM, RAM pointer is added to class 
        extract_mod(&pFat, p, cm->filePage(b));
      }
    }
  }
}
