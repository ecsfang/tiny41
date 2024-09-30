/*=======================================================================

Author: Warren Furlow (email: warren@furlow.org)

License: PUBLIC DOMAIN - May be freely copied and incorporated into any work

Description:  Contains routines for conversion between HP-41 ROM image file formats

Background:
Each HP-41 ROM page is 4096 words in size and each word is 10 bits.  There are
16 pages in the address space, but pages can be bank switched by up to 4 banks.
The pages 0-7 are for system use and pages that go there are hard coded to that
location.  Pages 8-F are for plug-in module usage through the four physical
ports.  Each port takes up two pages (Page8=Port1 Lower,Page9=Port1 Upper, etc.).
Note that some plug-in modules and peripherals are hard coded to map into certain
system pages (ex: time module).

Supported File Formats:
ROM - This format is used by V41 Release 7 and prior (Warren Furlow).
      It is always 8192 bytes with the High 2 bits followed by the Low 8 bits.

BIN - This format is used by Emu41 (J-F Garnier) and HP41EPC (HrastProgrammer).
      Note: HP41EPC uses BIN format but names them .ROM files.
      All bits are packed into 5120 bytes, but several consecutive pages may
      occupy the same file, so the file size could be a multiple of 5120.
      4 machine words are packed into 5 bytes:
      Byte0=Word0[7-0]
      Byte1=Word1[5-0]<<2 | Word0[9-8]
      Byte2=Word2[3-0]<<4 | Word1[9-6]
      Byte3=Word3[1-0]<<6 | Word2[9-4]
      Byte4=Word3[9-2]

HPX - HEPAX ROM File Format
      4 machine words are packed into 5 bytes:
      Byte0=Word0[9-2]
      Byte1=Word0[1-0]<<6 | Word1[9-4]
      Byte2=Word1[3-0]<<4 | Word2[9-6]
      Byte3=Word2[5-0]<<2 | Word3[9-8]
      Byte4=Word3[7-0]

ECO - ERAMCO ROM File Format
      4 machine words are packed into 5 bytes:
      Byte0=Word3[9-8]<<6 | Word2[9-8]<<4 | Word1[9-8]<<2 | Word0[9-8]
      Byte1=Word3[7-0]
      Byte2=Word2[7-0]
      Byte3=Word1[7-0]
      Byte4=Word0[7-0]

LST - This format is a text file dump of the disassembled machine code generated
      by the Zenrom MCED utility.  The first 4 digit hex number is the absolute
      address and the second 3 digit hex number is the machine word.  The mnemonic
      normally appears after that.  For the purposes of file conversion, only the
      machine word is actually used and only the first page is read from the file
      with the provided routines.
      Example:
      8000 158 M=C
      8001 398 C=ST
      8002 056 C=0    XS
      8003 284 CF     7

MOD - MOD File format is a more advanced multi-page format for containing an entire module or
      all operating system pages.  See MODFile.h  This format is used by V41 Release 8.

Sample usage: convert a lst file to a bin file:
  word *ROM;
  ROM=read_lst_file("test.lst");
  if (ROM==NULL)
    return;
  write_bin_file("test.bin",ROM);
  free(ROM);

Convert a bin file to a rom file:
  ROM=read_bin_file("test.bin",0);
  if (ROM==NULL)
    return;
  write_rom_file("test.rom",ROM);
  free(ROM);

MODULE FILE LOADER - for emulators etc
The exact loading procedure will be dependent on the emulator's implementation.  V41 uses a
three pass process to find empty pages and ensure that the ROM attributes are correctly followed.
Feel free to copy and adapt the algorithm in LoadMOD() to your software.

First Pass:  Validate variables, go through each page in the mod file.  If there any PageGroups,
count the number of pages in each group using the appropriate array (LowerGroup[8], UpperGroup[0], etc).
This count is stored as a negative number.  In the second pass this value will be replaced with
a positive number which will represent the actual page to be loaded

Second Pass: Go through each page again and find free locations for any that are grouped.
If a page is the first one encountered in a group, find a block of free space for the group.
For instance, Odd/Even will require two contiguous spaces.  If a page is the second or subsequent
one encountered in a group, then we already have the block of space found and simply need to
stick it in the right place.  For instance, the lower page has already been loaded, then the upper
page goes right after it.

Third Pass: Find a free location for any non-grouped pages.  This includes system pages which have
their page number hardcoded.

=========================================================================*/

#define _CRT_SECURE_NO_WARNINGS
#define _CRT_NONSTDC_NO_WARNINGS

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <memory.h>
#include "tiny41.h"
#include "core_bus.h"
#include "module.h"
#include "modfile.h"

extern CModules modules;
extern void qRam(int page);

/*******************************/
#if 0
write_rom_file(const char *FullFileName, const word *ROM)
{
  FILE *File;
  long SizeWritten;
  word *ROM2;
  int i;

  if (ROM == NULL)
    return (0);

  File = fopen(FullFileName, "wb");

  if (File == NULL) {
    fprintf(stderr, "ERROR: File Open Failed: %s\n", FullFileName);
    return (0);
  }

  ROM2 = (word *)malloc(sizeof(word) * 0x1000);

  if (ROM2 == NULL) {
    fclose(File);
    fprintf(stderr, "ERROR: Memory Allocation\n");
    return (0);
  }

  for (i = 0; i < 0x1000; i++)
    ROM2[i] = (ROM[i] << 8) | (ROM[i] >> 8);

  SizeWritten = (long)fwrite(ROM2, 1, 8192, File);
  fclose(File);
  free(ROM2);
  if (SizeWritten != 8192) {
    fprintf(stderr, "ERROR: File Write Failed: %s\n", FullFileName);
    return (0);
  }
  return (1);
}
#endif

#define ROM10_LENGTH (4096+1024)  // Packed 10 bits
/******************************/
void unpack_image( word *ROM, const byte *BIN)
{
  int i;
  word *ptr = ROM;
  if ((ROM == NULL) || (BIN == NULL))
    return;
  for (i = 0; i < ROM10_LENGTH; i += 5) {
    *ptr++ = ((BIN[i + 1] & 0x03) << 8) |   BIN[i + 0];
    *ptr++ = ((BIN[i + 2] & 0x0F) << 6) | ((BIN[i + 1] & 0xFC) >> 2);
    *ptr++ = ((BIN[i + 3] & 0x3F) << 4) | ((BIN[i + 2] & 0xF0) >> 4);
    *ptr++ = ( BIN[i + 4]         << 2) | ((BIN[i + 3] & 0xC0) >> 6);
  }
}
/****************************/
// Returns 0 for unknown format, 1 for MOD1, 2 for MOD2
/****************************/
int get_file_format(const char *lpszFormat)
{
  if( !strcmp(lpszFormat, MOD_FORMAT) )
    return MOD1_FMT;
  if( !strcmp(lpszFormat, MOD_FORMAT2) )
    return MOD2_FMT;
  return 0;
}

// Read module information from a MOD file in flash
int mod_info(const char *flashPtr, char *buf)
{
  ModuleFileHeader *pMFH = (ModuleFileHeader *)flashPtr;
  return sprintf(buf,"%s", pMFH->Title);
}


/******************************
 * extract_mod
 * Takes a FAT to a MOD file entry and load the module given a port(0-F) number
 * The MOD file might override the port (if hardcoded)
 * Returns:
 *  0 for success
 *  1 for open fail
 *  2 for read fail 
 *  3 for invalid file
 *  4 for allocation error
 ******************************/
int extract_mod( CFat_t *pFat, int port)
{
#ifdef DBG_PRINT
  sprintf(cbuff,"Extract ROM (%d) @ 0x%X\n\r", port, pFat->offs());
  cdc_send_string_and_flush(ITF_TRACE, cbuff);
#endif

  if( pFat->type() == FL_ROM ) {
    CRomFile  *pRom = new CRomFile(pFat);
    int ret = 0;
    int bank = 0;
    if( pRom->verify() ) {
      word *img = pRom->getRomImage();
      if( img ) {
        // If no errors - insert into the specified port
        sprintf(cbuff,"Load ROM %s to port %X:%d ...\n\r", pFat->name(), port, bank);
        cdc_send_string_and_flush(ITF_TRACE, cbuff);
        // Save info about ROM-file with offset to file
        modules.addImage(port, img, bank, pFat->fatEntry(), pFat->name());
      } else
        ret = 4;
    }
    cdc_flush(ITF_TRACE);
    return ret;
  }
  if( pFat->type() != FL_MOD && pFat->type() != FL_RAM ) {
    sprintf(cbuff,"Unknown FAT type: 0x%02X!\n\r", pFat->type());
    cdc_send_string_and_flush(ITF_CONSOLE, cbuff);
    return 3;
  }

  CModFile *modFile = new CModFile(pFat);
  // check header
  if( !modFile->verify() )
    return (3);

  // go through each page
  ModuleFilePage *pMFP;
  int bank = 0;
  int nPort = 0;
  // Loop over all images in the file
  for (int nPage = 0; nPage < modFile->nrPages(); nPage++) {
    // Point to the specified page in the MOD file
    pMFP = modFile->getPage(nPage);
    // Which bank (0-3)?
    bank = pMFP->header.Bank - 1;
    // Any hardcoded port ... ?
    if( pMFP->header.Page <= 0x0F ) {
      port = pMFP->header.Page;  // Hardcoded port
      nPort = 0;  // Ignore previous page
    }
    if( bank == 0 ) {
      // First bank - increase the page number according the previous page (if any)
      port += nPort;
      {
        // Now, decide how the next update should be done ...
        // TBD Look for free page?
        // Now we just raplace any previous loaded module
        switch(pMFP->header.Page) {
        case POSITION_ANY:     /* position in any port page (8-F) */
        case POSITION_LOWER:   /* position in lower port page relative to any upper image(s) (8-F) */
        case POSITION_UPPER:   /* position in upper port page */
        case POSITION_EVEN:    /* position in any even port page (8,A,C,E) */
        case POSITION_ODD:     /* position in any odd port page (9,B,D,F) */
        case POSITION_ORDERED: /* position sequentially in order of MOD file loading, one image per page regardless of bank */
          break;
        }
        nPort = 1; // Increase page with 1 for next page
      }
    }
    // Read  the ROM file
    word *ROM = modFile->getRomImage(nPage);
    if( ROM ) {
#ifdef DBG_PRINT
      sprintf(cbuff,"Load MOD page %d [%s] to page %d bank %d (%X)...\n\r", nPage, pMFP->header.Name, port, bank, pFat->fatEntry());
      cdc_send_string_and_flush(ITF_CONSOLE, cbuff);
#endif
      // Add module with info about FAT and with page number in file
      modules.addImage(port, ROM, bank, pFat->fatEntry(), modFile->getPageName(nPage));
      if( pMFP->header.RAM )
        qRam(port);
    } else
      return (4);
  }
  return (0);
}
