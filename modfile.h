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

typedef unsigned char byte;
typedef unsigned short word;
typedef unsigned long dword;

#define MOD_FORMAT  "MOD1"
#define MOD_FORMAT2 "MOD2"

#define CATEGORY_MAX            7  /* maximum CATEGORY_ define value */
#define HARDWARE_MAX            10 /* maximum HARDWARE_ define value */

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
  byte Image[5120];       // the image in packed format (.BIN file format)
  byte PageCustom[32];    // for special hardware attributes
} V1_t;

// page struct for MOD2
typedef struct {
  word Image[4096];       // the image in unpacked format (.ROM file format)
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

int extract_roms( const char *flashPtr, int page);
int get_file_format(const char *lpszFormat);
