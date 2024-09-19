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
    return 1;
  if( !strcmp(lpszFormat, MOD_FORMAT2) )
    return 2;
  return 0;
}

int mod_info(const char *flashPtr, char *buf)
{
  ModuleFileHeader *pMFH;
  pMFH = (ModuleFileHeader *)flashPtr;
  return sprintf(buf,"%s", pMFH->Title);
}

/******************************/
/* Returns 0 for success, 1 for open fail, 2 for read fail, 3 for invalid file, 4 for allocation error */
/******************************/
int extract_roms( const char *flashPtr, int port) //CModule *mod )
{
  int nFileFormat;
  ModuleFileHeader *pMFH;
  dword dwModulePageSize;

  // get file format and module page size
  pMFH = (ModuleFileHeader *)flashPtr;
  nFileFormat = get_file_format(pMFH->FileFormat);
  dwModulePageSize = sizeof(ModuleHeader_t);
  dwModulePageSize += (1 == nFileFormat) ? sizeof(V1_t) : sizeof(V2_t);

  sprintf(cbuff,"Extract ROM (%d) @ 0x%X - %s (%d)\n\r", port, flashPtr, pMFH->FileFormat, nFileFormat);
  cdc_send_string_and_flush(ITF_TRACE, cbuff);

  // check header
  if( (1 != nFileFormat && 2 != nFileFormat) || pMFH->MemModules > 4 || pMFH->XMemModules > 3 ||
      pMFH->Original > 1 || pMFH->AppAutoUpdate > 1 || pMFH->Category > CATEGORY_MAX ||
      pMFH->Hardware > HARDWARE_MAX) { /* out of range */
    return (3);
  }

  // go through each page
  ModuleFilePage *pMFP;
  #define ROM_SIZE  0x1000
  word *pwImage;
  int nBank = 0;
  // Loop over all images in the file
  for (int bank = 0; bank < pMFH->NumPages; bank++) {
    nBank = 0;
    word *ROM = new word[ROM_SIZE];
    if( !ROM ) {
      sprintf(cbuff,"No memory in extract_roms!\n\r");
      cdc_send_string_and_flush(ITF_CONSOLE, cbuff);
      return 4;
    }
    pMFP = (ModuleFilePage *)(flashPtr + sizeof(ModuleFileHeader) + dwModulePageSize * bank);
    // Any hardcoded port ... ?
    if( pMFP->header.Page <= 0x0F )
      port = pMFP->header.Page;
    // Which bank?
    if( pMFP->header.Bank )
      nBank = pMFP->header.Bank - 1;
    // write the ROM file
    pwImage = (word *)&pMFP->image;
    switch(nFileFormat) {
    case 1:   // MOD1
      unpack_image(ROM, (byte*)pwImage);
      break;
    case 2:   // MOD2
      for(int j = 0; j < ROM_SIZE; ++j ) {
        // swap bytes
        ROM[j] = (pwImage[j] >> 8) | (pwImage[j] << 8);
      }
      break;
    default:  // Unknown format ...
      sprintf(cbuff,"Error: Unknown format (%d)!\n\r",nFileFormat);
      cdc_send_string_and_flush(ITF_CONSOLE, cbuff);
      delete[] ROM;
      ROM = NULL;
    }
    if( ROM ) {
      // Add image to module ...
      sprintf(cbuff,"Load MOD %s to port %d bank %d ...\n\r", pMFP->header.Name, port, nBank);
      cdc_send_string_and_flush(ITF_TRACE, cbuff);
      modules.addImage(port, ROM, nBank, pMFP->header.Name);
      if( pMFP->header.RAM )
        qRam(port);
      port++;
    }
  }
  return (0);
}
