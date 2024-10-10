#define NO_EXTERNAL
#include <stdio.h>
#include <string.h>
#include <memory.h>
#include <stdint.h>
#include "fltools.h"
#include <sys/stat.h>
#include "../modfile.h"
#include <getopt.h>

bool bFat = false;
bool bDump = false;

long GetFileSize(const char *filename)
{
    struct stat stat_buf;
    int rc = stat(filename, &stat_buf);
    return rc == 0 ? stat_buf.st_size : -1;
}

const char *category(int c)
{
  static char b[32];
  switch(c) {
    case CATEGORY_UNDEF:        return "";
    case CATEGORY_OS:           return "HP Operating System";
    case CATEGORY_APP_PAC:      return "HP Application Pac";
    case CATEGORY_HPIL_PERPH:   return "HP-IL Peripheral";
    case CATEGORY_STD_PERPH:    return "HP Standard Peripheral";
    case CATEGORY_CUSTOM_PERPH: return "Custom Peripheral";
    case CATEGORY_BETA:         return "BETA";
    case CATEGORY_EXPERIMENTAL: return "Experimental";
    default:
      sprintf(b, "Unknown: 0x%02X", c);
      return b;
  }
}

const char *hardware(int h)
{
  static char b[32];
  switch(h) {
    case HARDWARE_NONE:       return "";
    case HARDWARE_PRINTER:    return "82143A Printer";
    case HARDWARE_CARDREADER: return "82104A Card Reader";
    case HARDWARE_TIMER:      return "82182A Time Module or HP41CX build in timer";
    case HARDWARE_WAND:       return "82153A Barcode Wand";
    case HARDWARE_HPIL:       return "82160A HP-IL Module";
    case HARDWARE_INFRARED:   return "82242A Infrared Printer Module";
    case HARDWARE_HEPAX:      return "HEPAX Module";
    case HARDWARE_WWRAMBOX:   return "W&W RAMBOX";
    case HARDWARE_MLDL2000:   return "MLDL2000";
    case HARDWARE_CLONIX:     return "CLONIX-41 Module";
    default:
      sprintf(b, "Unknown: 0x%02X", h);
      return b;
  }
}

const char *page(int p)
{
  static char b[32];
  switch(p) {
    case 0x1F: return "Any page 8-F";
    case 0x2F: return "Lower page (8-F)";
    case 0x3F: return "Upper page (8-F)";
    case 0x4F: return "Any even page (8,A,C,E)";
    case 0x5F: return "Any odd page (9,B,D,F)";
    case 0x6F: return "Position seq in order";
    default:
      sprintf(b, "Hardcoded to page %X", p);
      return b;
  }
}

#define ROM10_LENGTH (4096+1024)  // Packed 10 bits

typedef struct {
	unsigned int b0:10;
	unsigned int b1:10;
	unsigned int b2:10;
	unsigned int b3:10;
} __attribute__((packed)) Int10_x;

void unpack_image( word *ROM, const byte *BIN)
{
  word *ptr = ROM;
  Int10_x *p10;
  if ((ROM == NULL) || (BIN == NULL))
    return;
  for (int i = 0; i < (ROM10_LENGTH/5); i++) {
    p10 = (Int10_x*)BIN;
    *ptr++ = p10->b0;
    *ptr++ = p10->b1;
    *ptr++ = p10->b2;
    *ptr++ = p10->b3;
    BIN += 5;
  }
}

/*
x00E 204 UserCode
x00F 002 Function address x402
:
x400 009 9 regs to copy
x401 220
x402 1C2 LBL
x403 001
x404 0F5 
x405 000
x406 054 T
x407 045 E
x408 053 S
x409 054 T
x40A 19C FIX 
*/

unsigned char ch41[128+1] =
"@ABCDEFGHIJKLMNO"
"PQRSTUVWXYZ[\\]^_"
" |\"#$%&'()*+{-}/"
"0123456789#,<=>?"
"+abcde~#######S#"
"################"
"'abcdefghijklmno"
"pqrstuvwxyz{#}S+";

char ch2ascii(unsigned char ch)
{
  return ch41[ch&0x7F];
/*  ch &= 0x7F;
  if( ch < 0x20 ) return ch + 'A' - 1;
  if( ch >= 0x30 && ch < 0x3A ) return ch;
  if( ch >= 0x40 && ch < 0x60 ) return ch - 0x40 + 'a' - 1;
  switch(ch) {
  case 0x00:  return '@';
  case 0x20:  return ' ';
  case 0x24:  return '$';
  case 0x2B:  return '+';
  case 0x2D:  return '-';
  case 0x3C:  return '<';
  case 0x3E:  return '>';
  case 0x3F:  return '?';
  case 0x3D:  return '=';
  case 0x4E:  return 'S';
  default:
    return '^';
  }*/
}
void dumpLbl(int offs, word *rom)
{
  int addr;
  unsigned char ch;
  switch( (offs & 0xF0000) >> 16 ) {
  case 0x0: // MCode
    printf("'");
    addr = offs-1;
    do {
      ch = rom[addr];
      printf("%c", ch2ascii(ch));
      addr--;
    } while(!(ch & 0x80));
    printf("'");
    break;
  case 0x2: // User code
    {
      printf("\"");
      offs &= 0xFFF;
      addr = offs+4;
      int n = rom[offs+2] - 0xF1;
      while( n-- ) {
        ch = rom[addr];
        printf("%c", ch);
        addr++;
      };
      printf("\"");
    }
    break;
  }
}

const char *service(int addr)
{
  switch(addr) {
  case 0x0FF4: return "Interrupt checked during Pause";
  case 0x0FF5: return "Interrupt checked if system flag 53 set";
  case 0x0FF6: return "Interrupt checked on wakeup /not ON key";
  case 0x0FF7: return "Interrupt checked when HP41 is turned off";
  case 0x0FF8: return "Interrupt checked just before CPU stops";
  case 0x0FF9: return "Interrupt checked on wakeup /ON key";
  case 0x0FFA: return "Interrupt checked on MEMORY LOST";
  }
  return "???";
}

void dump1(FILE *fp, int fat)
{
  V1_t  v1;
  word  *rom = new word[0x1000];
  fread(&v1, sizeof(V1_t), 1, fp);
  unpack_image(rom, v1.Image);  
  word  chk = 0;

  for(int i=0; i<0x1000; i++) {
    chk += rom[i];
    if( chk > 0x3FF )
      chk = (chk+1) & 0x3FF;
  }

  printf("\nDump image (v1)\n"
           "---------------------------------------------------\n");
  int a=0;
  if( fat && bFat ) {
    int xrom = rom[0];
    int fcns = rom[1];

    printf("x%03X %03X\tXROM\t%02d\n", 0, rom[0], xrom);
    printf("x%03X %03X\tFCNS\t%d\n", 1, rom[1], fcns);

    for(a=0; a<fcns; a++) {
      int offs = rom[2+2*a] << 8 | rom[3+2*a];
      printf("x%03X %03X\n", 2+2*a, rom[2+2*a]);
      printf("x%03X %03X\tDEFR4K\t[x%03X] ", 3+2*a, rom[3+2*a], offs & 0xFFF);
      dumpLbl(offs, rom);
      printf("\n");
    }
    a = 2+2*a;
  }
  if( !fat && !bDump ) {
    a = 0xFF0;
  } else {
    if( fat && !bDump ) { // Only dump fat and footer of a module
      do { 
        printf("x%03X %03X\n", a, rom[a]);
        a++;
      } while( rom[a-2] != 0 && rom[a-1] == 0 );
      printf("\n...\n\n");
      a = 0xFF0;
    }
  }
  for(int i=a; i<0x1000; i++) {
    printf("x%03X %03X", i, rom[i]);
    if( i >= 0xFF4 && i <= 0xFFA )
      printf("\t\t; %s", service(i));
    if( i >= 0xFFB && i <= 0xFFE )
      printf("\t\t; '%c'", ch2ascii(rom[i]));
    if( i == 0xFFF )
      printf("\t\t; ROM Checksum (%s)", chk==1?"OK":"Error!");
    printf("\n");
  }
  printf("\nPageCustom:    ");
  for(int i=0; i<32; i++) {
    if( i>0 && (i & 0xF) == 0x0 )
      printf("\n               ");
    printf("%02X ", v1.PageCustom[i]);
  }
  printf("\n");

}

void dump2(FILE *fp, int fat)
{
  V2_t  v2;
  fread(&v2, sizeof(V2_t), 1, fp);

}

#define LINE_LEN  60
void printTab(char *txt)
{
  int l = strlen(txt);
  char *p = txt;
  int i=0;

  if( l==0 ) {
    printf("\n");
    return;
  }

  while( l > LINE_LEN ) {
    p += LINE_LEN;
    while(*p != ' ' && p > txt)
      p--;
    *p = 0;
    if( i )
      printf("               ");
    printf("%s\n", txt);
    txt = ++p;
    l = strlen(txt);
    i++;
  }
  if( l>0 ) {
    if( i )
      printf("               ");
    printf("%s\n", txt);
  }
}
int dump(FILE *fp, long sz)
{
  ModuleFileHeader MFH;
  ModuleFilePage MFP;
  int ver = 0;
  fread(&MFH, sizeof(ModuleFileHeader), 1, fp);

  if( !strcmp(MFH.FileFormat, "MOD1"))
    ver = 1;
  else if( !strcmp(MFH.FileFormat, "MOD2"))
    ver = 2;

  printf("File size:     %ld\n", sz);
  printf("FileFormat:    %s\n", MFH.FileFormat);
  printf("Title:         %s\n", MFH.Title);
  printf("Version:       %s\n", MFH.Version);
  printf("PartNumber:    %s\n", MFH.PartNumber);
  printf("Author:        %s\n", MFH.Author);
  printf("Copyright:     %s\n", MFH.Copyright);
  printf("License:       "); printTab(MFH.License);
  printf("Comments:      "); printTab(MFH.Comments);
  printf("Category:      %s\n", category(MFH.Category));
  printf("Hardware:      %s\n", hardware(MFH.Hardware));
  printf("MemModules:    0x%02X\n", MFH.MemModules);
  printf("XMemModules:   0x%02X\n", MFH.XMemModules);
  printf("Original:      0x%02X\n", MFH.Original);
  printf("AppAutoUpdate: 0x%02X\n", MFH.AppAutoUpdate);
  printf("NumPages:      0x%02X\n", MFH.NumPages);
  printf("HeaderCustom:  ");
  for(int i=0; i<32; i++) {
    if( i>0 && (i & 0xF) == 0x0 )
      printf("\n               ");
    printf("%02X ", MFH.HeaderCustom[i]);
  }
  printf("\n");

  if( !MFH.NumPages )
    printf("No pages!\n");

  for( int p=0; p<MFH.NumPages; p++) {
    // Skip past previous modules ...
    fseek(fp, sizeof(ModuleFileHeader), SEEK_SET);
    for( int n=0; n<p; n++ ) {
      fseek(fp, sizeof(ModuleHeader_t), SEEK_CUR);
      switch( ver ) {
      case 1: fseek(fp, sizeof(V1_t), SEEK_CUR); break;
      case 2: fseek(fp, sizeof(V2_t), SEEK_CUR); break;
      default:
        printf("Unhandled version %d!\n", ver);
        return(-1);
      }
    }
    // Dump page header ...
    fread(&MFP, sizeof(ModuleHeader_t), 1, fp);
    printf("Header page %d\n------------------\n", p+1);
    printf("  Name:         %s\n", MFP.header.Name);
    printf("  ID:           %s\n", MFP.header.ID);
    printf("  Page:         %s\n", page(MFP.header.Page));
    printf("  PageGroup:    %d\n", MFP.header.PageGroup);
    printf("  Bank:         %d (%d)\n", MFP.header.Bank, MFP.header.BankGroup);
    printf("  RAM:          [%c]\n", MFP.header.RAM ? 'X':' ');
    printf("  WriteProtect: [%c]\n", MFP.header.WriteProtect ? 'X' : ' ');
    printf("  FAT:          [%c]\n", MFP.header.FAT ? 'X':' ');

    // Dump the module image ...
    if( bFat || bDump ) {
      switch( ver ) {
      case 1: dump1(fp, MFP.header.FAT); break;
      case 2: dump2(fp, MFP.header.FAT); break;
      }
    }
  }
  printf("\n=============================================================\n");
  return 0;
}

#define PAGE_SIZE   (4*1024)
#define FLASH_START (128*PAGE_SIZE)  // Offset to flash area (512kB)
#define FAT_START   (8*PAGE_SIZE)    // Offset to FAT
#define FAT_SIZE    (4)           // Size of FAT (pages)

int main(int argc, char *argv[])
{
  option longopts[] = {
    {"fat", optional_argument, NULL, 'f'}, 
    {"dump", optional_argument, NULL, 'd'},
    {0}
  };

  while (1) {
    const int opt = getopt_long(argc, argv, "fd::", longopts, 0);

    if (opt == -1)
      break;

    switch (opt) {
    case 'd':
      bDump = true;
      // Fall through ...
    case 'f':
      bFat = true;
      break;
    }
  }

  char *file = argv[optind];
  FILE *mod = fopen(file, "r");

  if( mod ) {
    long sz = GetFileSize(file);
    dump(mod, sz);
    fclose(mod);
  } else {
    printf("Can't open <%s>!\n", file);
  }
  return 0;
}