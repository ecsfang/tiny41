#define NO_EXTERNAL
#include <stdio.h>
#include <string.h>
#include <memory.h>
#include <stdint.h>
#include "fltools.h"
#include <sys/stat.h>
#include "../modfile.h"

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

#define LINE_LEN  80
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
  fread(&MFH, sizeof(ModuleFileHeader), 1, fp);
  printf("File size:     %ld\n", sz);
  printf("FileFormat:    %s\n", MFH.FileFormat);
  printf("Title:         %s\n", MFH.Title);
  printf("Version:       %s\n", MFH.Version);
  printf("PartNumber:    %s\n", MFH.PartNumber);
  printf("Author:        %s\n", MFH.Author);
  printf("Copyright:     %s\n", MFH.Copyright);
  printf("License:       ");
  printTab(MFH.License);
  printf("Comments:      ");
  printTab(MFH.Comments);
  printf("Category:      %s\n", category(MFH.Category));
  printf("Hardware:      %s\n", hardware(MFH.Hardware));
  printf("MemModules:    0x%02X\n", MFH.MemModules);
  printf("XMemModules:   0x%02X\n", MFH.XMemModules);
  printf("Original:      0x%02X\n", MFH.Original);
  printf("AppAutoUpdate: 0x%02X\n", MFH.AppAutoUpdate);
  printf("NumPages:      0x%02X\n", MFH.NumPages);
  printf("HeaderCustom:  ");
  for(int i=0; i<32; i++)
    printf("%02X%c", MFH.HeaderCustom[i], i<31?'.':'\n');

  if( !MFH.NumPages )
    printf("No pages!\n");

  for( int p=0; p<MFH.NumPages; p++) {
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

    fseek(fp, sizeof(ModuleFileHeader), SEEK_SET);
    for( int n=0; n<=p; n++ ) {
      fseek(fp, sizeof(ModuleHeader_t), SEEK_CUR);
      if( !strcmp(MFH.FileFormat, "MOD1"))
        fseek(fp, sizeof(V1_t), SEEK_CUR);
      if( !strcmp(MFH.FileFormat, "MOD2"))
        fseek(fp, sizeof(V2_t), SEEK_CUR);
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
  FILE *mod = fopen(argv[1], "r");

  if( mod ) {
    long sz = GetFileSize(argv[1]);
    dump(mod, sz);
    fclose(mod);
  } else {
    printf("Can't open <%s>!\n", argv[1]);
  }
  return 0;
}