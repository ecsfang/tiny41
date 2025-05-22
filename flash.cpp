#include <stdio.h>
#include <string.h>
//#include <time.h>
//#include "pico/stdlib.h"
//#include "hardware/gpio.h"
//#include "hardware/pio.h"
//#include "hardware/clocks.h"
//#include "hardware/sync.h"
//#include "pico/multicore.h"
//#include "hardware/flash.h"
#include "pico/flash.h"
#include "core_bus.h"
#include "flash.h"

#include "module.h"
#include "modfile.h"

typedef struct {
  int     addr;
  uint8_t *buf;
  uint8_t pgs;  // Number of flash pages
} flash_write_t;

// Writing to flash can only be done page wise!
int pageAdjust(int addr)
{
  if( addr % FLASH_PAGE_SIZE ) {
    addr |= FLASH_PAGE_SIZE-1;
    addr++;
  }
  return addr;
}

#define FLASH_LOCK_TIMEOUT_MS 5*1000

bool diffPage(void *p1, void *p2)
{
  return memcmp(p1, p2, sizeof(uint16_t)*ROM_SIZE) ? true : false;
}

bool invalidFlashPtr(int fptr)
{
  return fptr < (FLASH_START+XIP_BASE) || fptr > (FLASH_SIZE+XIP_BASE);
}

// Write one or more flash pages (4k)
// Input is a struct with address,
// data pointer and number of pages to write
static void write_flash_page(void *param) {
  flash_write_t *tmp = (flash_write_t *)param;
  // Offset the flash pointer
  int fp = tmp->addr-XIP_BASE;
  uint8_t *data = tmp->buf;
  uint32_t ints = save_and_disable_interrupts();
  for(int i=0; i<tmp->pgs; i++ ) {
    flash_range_erase(fp, PG_SIZE);
    flash_range_program(fp, data, PG_SIZE);
    fp += PG_SIZE;
    data += PG_SIZE;
  }
  restore_interrupts(ints); // Note that a whole number of sectors must be erased at a time.
}

// Safe way to flash some pages to flash
uint16_t *writePage(flash_write_t *pDta)
{
  int r = flash_safe_execute(write_flash_page, pDta, FLASH_LOCK_TIMEOUT_MS);
  if (r != PICO_OK) {
    sprintf(cbuff, "error calling write_flash_page: %d", r);
    cdc_send_string_and_flush(ITF_CONSOLE, cbuff);
#ifdef DBG_PRINT
  } else {
    sprintf(cbuff, "Wrote 0x%08X to flash @ 0x%08X:%X\n\r", pDta->buf, pDta->addr, pDta->pgs*PG_SIZE);
    cdc_send_string_and_flush(ITF_CONSOLE, cbuff);
#endif
  }
  return (uint16_t*)pDta->addr;
}

uint16_t *writePage(int addr, uint8_t *data, uint8_t pgs)
{
  flash_write_t tmp = {addr, data, pgs};
  return writePage(&tmp);
}

// Write data to given flash port (page and bank) in ROMMAP
// One ROM page is always written (4K 10-bit data - two flash pages (8K))
// Returns pointer to flash image (or NULL if failed)
uint16_t *writeROMMAP(int p, int b, uint16_t *data)
{
  flash_write_t tmp = {ROMMAP_PAGE(p,b), (uint8_t*)data, 2};
  if( invalidFlashPtr(tmp.addr) ) {
      sprintf(cbuff, "writeROMMAP: Bad address: %08X\n\r", tmp.addr);
      cdc_send_string_and_flush(ITF_CONSOLE, cbuff);
      return NULL;
  }
  // Check if images differs ...
  if( diffPage((void*)tmp.addr, tmp.buf) ) {
    // Yes, so lets update the image to flash
    writePage(&tmp);
  }
  // Return address to the flash image
  return (uint16_t*)tmp.addr;
}

// Read flash into ram whitout swapping
void readFlash(int offs, uint8_t *data, uint16_t size)
{
  // Point at the wanted flash image ...
  const uint8_t *fp = (const uint8_t*)(XIP_BASE + offs);
  if( invalidFlashPtr((int)fp) ) {
    sprintf(cbuff, "BAD FLASH POINTER: %p!\n\r", fp);
    cdc_send_string_and_flush(ITF_CONSOLE, cbuff);
    return;
  }
  memcpy(data, fp, size);
}

// Function to calculate a simple checksum
static uint32_t calcChecksum(const uint8_t *data, size_t len) {
    uint32_t checksum = 0xBADC0DE; // Start with a non-zero value
    for (size_t i = 0; i < len-sizeof(uint32_t); i++) {
        checksum += data[i];
    }
    return ~checksum;
}

// Function to verify the checksum
static bool verifyChecksum(const uint8_t *data, size_t len, uint32_t exp) {
  uint32_t chk = calcChecksum((const uint8_t*)data, len);
  return chk == exp;
}
static bool verifyChecksum(Config_t *data) {
  return verifyChecksum((const uint8_t*)data, sizeof(Config_t), data->chkSum);
}
static bool verifyChecksum(Setup_t *data) {
  return verifyChecksum((const uint8_t*)data, sizeof(Setup_t), data->chkSum);
}

bool readConfig(Config_t *data, int set)
{
  uint32_t cp = CONF_PAGE(0) + set*sizeof(Config_t);
  memcpy(data, (uint8_t*)cp, sizeof(Config_t));
  //uint32_t chk = calcChecksum((const uint8_t*)data, sizeof(Config_t));
#ifdef DBG_PRINT
  sprintf(cbuff, "Read config(%d) @ 0x%08X:%X\n\r", set, cp, sizeof(Config_t));
  cdc_send_string_and_flush(ITF_CONSOLE, cbuff);
#endif
  return verifyChecksum(data);
}

// Read the config flashpage and updated the
// corresponding part and write back the page
uint16_t *writeConfigPage(int offs, void *data, int size)
{
  uint16_t *ret = NULL;
  uint8_t *pg = new uint8_t[FLASH_SECTOR_SIZE];
  // Read whole config page
  memcpy(pg, (uint8_t*)CONF_PAGE(0), FLASH_SECTOR_SIZE);
  // Update current part of the page
  memcpy(pg+offs, data, size);
  // Write back updated configuration
  ret = writePage(CONF_PAGE(0), pg, 1);
  delete[] pg;
  return ret;
}

// Update configuration #set in config page
uint16_t *writeConfig(Config_t *data, int set, bool bChk)
{
  if( bChk ) {
    // Yes, update checksum
    uint32_t chk = calcChecksum((const uint8_t*)data, sizeof(Config_t));
    data->chkSum = chk;
  }
  return writeConfigPage(set*sizeof(Config_t), data, sizeof(Config_t));
}

// Update the setup information in the config page
uint16_t *writeSetup(Setup_t *data)
{
  uint32_t chk = calcChecksum((const uint8_t*)data, sizeof(Setup_t));
  data->chkSum = chk;
  return writeConfigPage(SETUP_OFFS, data, SETUP_SIZE);
}

bool readSetup(Setup_t *data)
{
  uint32_t cp = CONF_PAGE(0) + SETUP_OFFS;
  memcpy(data, (uint8_t*)cp, SETUP_SIZE);
#ifdef DBG_PRINT
  sprintf(cbuff, "Read setup @ 0x%08X:%X\n\r", cp, SETUP_SIZE);
  cdc_send_string_and_flush(ITF_CONSOLE, cbuff);
#endif
return verifyChecksum(data);
}

// Save a given ram image to flash (emulate Q-RAM)
// TBD - This must be updated to update file in FFS!
// TBD - Should check all banks as well
// If loaded from ROM, then that page should be updated
// If loaded from MOD, then we must check which page etc that
// needs to be updated.
void saveRam(int port, int ovr = 0)
{
  int n = 0;
  if( port >= FIRST_PAGE && port < NR_PAGES ) {
    if (ovr || modules.isDirty(port)) {
      modules.clear(port);
      writeROMMAP(port, 0, modules.getImage(port));
      n = sprintf(cbuff, "Wrote RAM[%X] to flash\n\r", port);
    }
  } else {
      n = sprintf(cbuff, "Invalid page! [%X]\n\r", port);
  }
  if( n )
    cdc_send_string_and_flush(ITF_CONSOLE, cbuff);
  sprintf(cbuff, "Save RAM @ page %d implemented yet! [%X]\n\r", port);
  cdc_send_string_and_flush(ITF_CONSOLE, cbuff);
}


