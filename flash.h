#ifndef __FLASH_H__
#define __FLASH_H__

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/flash.h"
#include "hardware/flash.h"

#include "fltools/flconfig.h"
#include "fltools/fltools.h"

/*
 * This is a very simple FAT table, consiting of:
 *  - name (<24 characters describing the entry)
 *  - offset - offset in flash for the start of the data
 *  - type   - type of entry: MOD/RAM/ROM
 * Last entry have offset == 0
 * 4 pages are reserved for the FAT:
 *  --> 4 * 4 * 1024 / 32 bytes -> 512 entries (modules)
*/

class CFat {
  FL_Head_t *p_fatEntry;
  int m_pos;
public:
  CFat() {
    first();
  }
  CFat(FL_Head_t *p) {
    init(p);
  }
  void init(FL_Head_t *p=NULL) {
    p_fatEntry = p;
    m_pos = 0; // Not used - we just init the pointer ...
    // TBD - We could search and update m_pos with the
    // position of the entry with the current pointer?
  }
  // Read next entry in the FAT (and increment pointer)
  void next() {
    // Reads FAT entry
    p_fatEntry = (FL_Head_t*)(FAT_START + m_pos*sizeof(FL_Head_t));
    m_pos++;
  }
  // Read first entry in the FAT
  void first() {
    m_pos = 0;
    next();
  }
  // Find a given module name in the FAT.
  // Return position [1..n] if found, otherwise 0
  int find(const char *mod) {
    first();
    while( offset() ) {
      if( strcmp(p_fatEntry->name, mod) == 0 )
        return m_pos;
      next();
    }
    return 0;
  }
  // Return offset to the file data for current FAT entry
  const char *offset(void) {
    if( p_fatEntry->offs > (FLASH_SIZE+XIP_BASE) ||
        p_fatEntry->offs < XIP_BASE )
      return NULL;
    return (char*)p_fatEntry->offs;
  }
  FL_Head_t *fatEntry() {
    return p_fatEntry;
  }
  // Return the module name from the FAT
  char *name(void) {
    return p_fatEntry->name;
  }
  // Return the module type from the FAT
  int type(void) {
    return p_fatEntry->type;
  }
};

#define CONF_DESC_LEN 64
// With current size (16*(4*4+4)+64+4 = 388 bytes), there is room
// for at least 10 configurations in one flash page (216 bytes left)
typedef struct {
  FL_Head_t *fat[NR_BANKS];             // Pointer to FAT entry
  uint8_t    filePage[NR_BANKS];        // Page in the file image
} ModuleConfig_t;
typedef struct {
  char            desc[CONF_DESC_LEN];  // Description of the config
  ModuleConfig_t  mod[NR_PAGES];        // The actual page config
  uint32_t        chkSum;               // Checksum of the config
} Config_t;

//#define CONF_OFFS(n)  (n*sizeof(Config_t))     // Offset in flash for the config

typedef struct {
  uint32_t config;
  uint32_t chkSum;               // Checksum of the config
} Setup_t;

// Place setup directly after config (up to 216 bytes)
#define SETUP_SIZE    sizeof(Setup_t)   // Total size of the config
#define SETUP_OFFS    (10*sizeof(Config_t))    // Offset in flash for the config

extern uint16_t *writeROMMAP(int p, int b, uint16_t *data);
extern uint16_t *writePage(int addr, uint8_t *data, uint8_t pgs);
extern uint16_t *writeConfig(Config_t *data, int set=0, bool bChk=true);
extern bool readConfig(Config_t *data, int set);
extern uint16_t *writeSetup(Setup_t *data);
extern bool      readSetup(Setup_t *data);
//extern void readFlash(int offs, uint8_t *data, uint16_t size);

#endif//__FLASH_H__
