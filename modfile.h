#ifndef __MODFILE_H__
#define __MODFILE_H__
/*=======================================================================

Author: Warren Furlow (email: warren@furlow.org)

License: PUBLIC DOMAIN - May be freely copied and incorporated into any work

Description:  Describes the structure of the MODULE file for HP-41 ROM images

.MOD File Structure:
These structures define the .MOD file format which replaces the older ROM
image dump formats (ROM, BIN).  MOD format allows the definition of entire
plug-in modules which may be composed of ROM images, RAM, special hardware
etc.  The HP-41C, -CV and -CX base operating system is defined in one MOD
file (ie The CX base includes 4 memory modules, the timer hardware, and
XFuns/XMem registers as well as 6 ROM images).  Additionally, Single Memory,
Quad Memory, XFuns/XMem, XMem, Timer modules can be defined each in their own
MOD file.  Obviously certain configurations do not make sense any more than
with the real hardware and may return an error (ie an HP-41CV AND a Quad
Memory Module).  It is also possible to define MLDL RAM using a blank page.

Strings are null terminated and all unused bytes are set to zero.  Fields are
strictly limited to valid values defined below.  Some combinations of values
would make no sense and not represent any actual hardware.  
File size=sizeof(ModuleFileHeader)+NumPages*sizeof(ModuleFilePage)
=======================================================================*/

#pragma once

#define INCLUDE_EXTERNAL

#ifdef NO_EXTERNAL
#undef INCLUDE_EXTERNAL
#endif

#ifdef INCLUDE_EXTERNAL
extern char *getPageName(const char *flashPtr, int page);
extern int extract_mod( CFat *pFat, int page, int pg=-1);
extern bool loadModule(const char *mod, int page);
extern int get_file_format(const char *lpszFormat);
extern void initRoms(void);
#endif

typedef unsigned char byte;
typedef unsigned short word;
typedef unsigned long dword;

#define ROM_FMT   0
#define MOD1_FMT  1
#define MOD2_FMT  2

#define MOD_FORMAT  "MOD1"
#define MOD_FORMAT2 "MOD2"

/* Module type codes */
#define CATEGORY_UNDEF          0  /* not categorized */
#define CATEGORY_OS             1  /* base Operating System for C,CV,CX */
#define CATEGORY_APP_PAC        2  /* HP Application PACs */
#define CATEGORY_HPIL_PERPH     3  /* any HP-IL related modules and devices */
#define CATEGORY_STD_PERPH      4  /* standard Peripherals: Wand, Printer, Card Reader, XFuns/Mem, Service, Time, IR Printer */
#define CATEGORY_CUSTOM_PERPH   5  /* custom Peripherals: AECROM, CCD, HEPAX, PPC, ZENROM, etc */
#define CATEGORY_BETA           6  /* BETA releases not fully debugged and finished */
#define CATEGORY_EXPERIMENTAL   7  /* test programs not meant for normal usage */
#define CATEGORY_MAX            7  /* maximum CATEGORY_ define value */

/* Hardware codes */
#define HARDWARE_NONE                0  /* no additional hardware specified */
#define HARDWARE_PRINTER             1  /* 82143A Printer */
#define HARDWARE_CARDREADER          2  /* 82104A Card Reader */
#define HARDWARE_TIMER               3  /* 82182A Time Module or HP-41CX built in timer */
#define HARDWARE_WAND                4  /* 82153A Barcode Wand */
#define HARDWARE_HPIL                5  /* 82160A HP-IL Module */
#define HARDWARE_INFRARED            6  /* 82242A Infrared Printer Module */
#define HARDWARE_HEPAX               7  /* HEPAX Module - has special hardware features (write protect, relocation) */
#define HARDWARE_WWRAMBOX            8  /* W&W RAMBOX - has special hardware features (RAM block swap instructions) */
#define HARDWARE_MLDL2000            9  /* MLDL2000 */
#define HARDWARE_CLONIX              10 /* CLONIX-41 Module */
#define HARDWARE_MAX                 10 /* maximum HARDWARE_ define value */

/* relative position codes- do not mix these in a group except ODD/EVEN and UPPER/LOWER */
/* ODD/EVEN, UPPER/LOWER can only place ROMS in 16K blocks */
#define POSITION_MIN      0x1f   /* minimum POSITION_ define value */
#define POSITION_ANY      0x1f   /* position in any port page (8-F) */
#define POSITION_LOWER    0x2f   /* position in lower port page relative to any upper image(s) (8-F) */
#define POSITION_UPPER    0x3f   /* position in upper port page */
#define POSITION_EVEN     0x4f   /* position in any even port page (8,A,C,E) */
#define POSITION_ODD      0x5f   /* position in any odd port page (9,B,D,F) */
#define POSITION_ORDERED  0x6f   /* position sequentially in order of MOD file loading, one image per page regardless of bank */
#define POSITION_MAX      0x6f   /* maximum POSITION_ define value */

/* Module header */
typedef struct {
  char FileFormat[5];     // constant value defines file format and revision
  char Title[50];         // the full module name (the short name is the name of the file itself)
  char Version[10];       // module version, if any
  char PartNumber[20];    // module part number
  char Author[50];        // author, if any
  char Copyright[100];    // copyright notice, if any
  char License[200];      // license terms, if any
  char Comments[255];     // free form comments, if any
  byte Category;          // module category, see codes below
  byte Hardware;          // defines special hardware that module contains
  byte MemModules;        // defines number of main memory modules (0-4)
  byte XMemModules;       // defines number of extended memory modules (0=none, 1=Xfuns/XMem, 2,3=one or two additional XMem modules)
  byte Original;          // allows validation of original contents:
                          //    1 = images and data are original
                          //    0 = this file has been updated by a user application (data in RAM written back to MOD file, etc)
  byte AppAutoUpdate;     // tells any application to: 1=overwrite this file automatically when saving other data, 0=do not update
  byte NumPages;          // the number of pages in this file (0-255, but normally between 1-6)
  byte HeaderCustom[32];  // for special hardware attributes
} ModuleFileHeader;

// page struct for MOD1
typedef struct {
  byte Image[5*1024];     // the image in packed format (.BIN file format)
  byte PageCustom[32];    // for special hardware attributes
} V1_t;

// page struct for MOD2
typedef struct {
  word Image[4*1024];     // the image in unpacked format (.ROM file format)
  byte PageCustom[32];    // for special hardware attributes
} V2_t;

// page struct for module
typedef struct {
  char Name[20];          // normally the name of the original .ROM file, if any
  char ID[9];             // ROM ID code, normally two letters and a number are ID and last letter is revision - if all zeros, will show up as @@@@
  byte Page;              // the page that this image must be in (0-F, although 8-F is not normally used) or defines each page's
                          // position relative to other images in a page group, see codes below
  byte PageGroup;         // 0=not grouped, otherwise images with matching PageGroup values (1..8) are grouped according to POSITION code
  byte Bank;              // the bank that this image must be in (1-4)
  byte BankGroup;         // 0=not grouped, otherwise images with matching BankGroup values (1..8) are bankswitched with each other
  byte RAM;               // 0=ROM, 1=RAM - normally RAM pages are all blank if Original=1
  byte WriteProtect;      // 0=No or N/A, 1=protected - for HEPAX RAM and others that might support it
  byte FAT;               // 0=no FAT, 1=has FAT
} ModuleHeader_t;

// page struct for module
typedef struct {
  ModuleHeader_t header;
  union {
    byte image;
    V1_t v1;              // page struct for MOD1
    V2_t v2;              // page struct for MOD2
  };
} ModuleFilePage;

#ifdef INCLUDE_EXTERNAL
extern void unpack_image( word *ROM, const byte *BIN);

// Class to hanlde a ROM file
class CFile {
protected:
  const uint8_t *m_fPtr;
  char *m_name;
  bool m_ok;
public:
  CFile(CFat *pFat) {
    m_fPtr = (const uint8_t*)pFat->offset();
    m_name = pFat->name();
    m_ok = false;
  }
  virtual bool verify(void) = 0;
  // Allocate memory for the image and read it
  // Returns pointer to the allocated image
  virtual word *getRomImage(int page = 0) = 0;
  word *saveROMMAP(int p, int b, word *img, bool bQROM=false) {
    // Save the image to ROMMAP (release RAM if not QROM)
    word *flRom = writeROMMAP(p, b, img);
    if( !bQROM ) {
      // If ROM - use flash pointer - no need for RAM!
      delete[] img;
      img = flRom;
    }
    return img;
  }
  // Swap order to get right endian of 16-bit word ...
  void swapRom(word *dst, const word *src) {
    for (int i = 0; i < ROM_SIZE; ++i)
      *dst++ = __builtin_bswap16(*src++);
  }
};

// Class to hanlde a ROM file
class CRomFile : public CFile {
 public:
  CRomFile(CFat *pFat) : CFile(pFat) {
  }
  bool verify(void) {
    m_ok = true;
    const word *fp16 = (word*)m_fPtr;
    for (int i = 0; m_ok && i < ROM_SIZE; ++i) {
      // Check that only 10 bit instructions are used ...
      if( *fp16++ & 0x00F0 )
        m_ok = false;
    }
    return m_ok;
  }
  // Allocate memory for the image and read it
  // Returns pointer to the allocated image
  word *getRomImage(int page = 0) {
    word *ROM = new word[ROM_SIZE];
    const word *fp16 = (word*)m_fPtr;
    if( !ROM ) {
      sprintf(cbuff,"No memory in CRomFile!\n\r");
      cdc_send_string_and_flush(ITF_CONSOLE, cbuff);
      return NULL;
    }
    // Read page and swap order to get right endian ...
    swapRom(ROM, fp16);
    return ROM;
  }
};

// Class to hanlde a MOD file
class CModFile : public CFile {
  ModuleFileHeader *pMFH;
  int   m_fileFmt;
  dword m_pageSize;
 public:
  CModFile(CFat *pFat) : CFile(pFat) {
    pMFH = (ModuleFileHeader *)m_fPtr;
    m_fileFmt = get_file_format(pMFH->FileFormat);
    m_pageSize = sizeof(ModuleHeader_t) + (MOD1_FMT == m_fileFmt ? sizeof(V1_t) : sizeof(V2_t));
  }
  void dump() {
#ifdef DBG_PRINT
    sprintf(cbuff,"m_fPtr: %X: %d p0:%X p1:%X\n\r", m_fPtr, sizeof(ModuleFileHeader), getPage(0), getPage(1));
    cdc_send_string_and_flush(ITF_TRACE, cbuff);
#endif

  }
  ModuleFilePage *getPage(int page) {
    return (ModuleFilePage *)(m_fPtr + sizeof(ModuleFileHeader) + m_pageSize * page);
  }
  char *getPageName(int page) {
    return getPage(page)->header.Name;
  }
  char *getTitle() {
    return pMFH->Title;
  }
  int getInfo(char *buf) {
    return sprintf(buf,"%s", pMFH->Title);
  }
  bool verify() {
    // check header
    m_ok = true;
    if( (MOD1_FMT != m_fileFmt && MOD2_FMT != m_fileFmt) ||
         pMFH->MemModules > 4 || pMFH->XMemModules > 3 ||
         pMFH->Original > 1 || pMFH->AppAutoUpdate > 1 || pMFH->Category > CATEGORY_MAX ||
         pMFH->Hardware > HARDWARE_MAX ) { /* out of range */
      sprintf(cbuff,"Bad format of MOD file %s @ 0x%X\n\r", m_name, m_fPtr);
      cdc_send_string_and_flush(ITF_CONSOLE, cbuff);
      m_ok = false;
    }
    return m_ok;
  }
  int nrPages(void) {
    return pMFH->NumPages;
  }
  // Allocate memory for the image and read it
  // Returns pointer to the allocated image
  word *getRomImage(int page) {
    ModuleFilePage *pMFP = getPage(page);
    word *ROM = new word[ROM_SIZE];
    if( !ROM ) {
      sprintf(cbuff,"No memory in extract_mod!\n\r");
      cdc_send_string_and_flush(ITF_CONSOLE, cbuff);
      return NULL;
    }
    word *pwImage = (word *)&pMFP->image;
    switch(m_fileFmt) {
    case MOD1_FMT:   // MOD1 - packed data
      unpack_image(ROM, (byte*)pwImage);
      break;
    case MOD2_FMT:   // MOD2 - unpacked - needs to be swapped
      // Swap order to get right endian of 16-bit word ...
      swapRom(ROM, pwImage);
      break;
    default:  // Unknown format ...
      sprintf(cbuff,"Error: Unknown format (%d)!\n\r",m_fileFmt);
      cdc_send_string_and_flush(ITF_CONSOLE, cbuff);
      delete[] ROM;
      ROM = NULL;
    }
    return ROM;
  }
};
#endif
#endif//__MODFILE_H__
