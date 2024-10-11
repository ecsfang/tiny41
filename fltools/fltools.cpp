#define NO_EXTERNAL
#include <stdio.h>
#include <string.h>
#include <memory.h>
#include <stdint.h>
#include "fltools.h"
#include <sys/stat.h>
#include "flconfig.h"
#include "../modfile.h"



void initBin(FILE *bin, int offs)
{
  fprintf(bin, "#!/bin/bash\n\n");

  fprintf(bin, "# Load a ROM module at offset 512KB in flash\n");
  fprintf(bin, "# With 4MB flash this lead to 3584KB for images\n");
  fprintf(bin, "# Every page takes 8KB -> 448 ROM images!\n\n");
  fprintf(bin, "picotool=~/Projects/picotool/build/picotool\n\n");
}
void initConf(FILE *bin, int offs)
{
  fprintf(bin, "# Clear default module configuration\n");
  fprintf(bin, "echo Erase module configuration\n");
  fprintf(bin, "sudo $picotool load -v iconf.ini -t bin -o 0x%X\n", offs);
}
void initFat(FILE *bin, int offs)
{
  fprintf(bin, "# Flash default FAT configuration\n");
  fprintf(bin, "echo Flash FAT\n");
  fprintf(bin, "sudo $picotool load -v iflash.ini -t bin -o 0x%X\n", offs);
}

void saveBin(FILE *bin, int x, int offs, char *rom, long size )
{
  fprintf(bin, "\necho Flash module %d to 0x%X\n", x, offs);
  fprintf(bin, "mod=\"%s\" # The ROM file to flash (%ld bytes)\n", rom, size );
  fprintf(bin, "sudo $picotool load -v $mod -t bin -o 0x%X\n", offs);
}

long GetFileSize(const char *filename)
{
    struct stat stat_buf;
    int rc = stat(filename, &stat_buf);
    return rc == 0 ? stat_buf.st_size : -1;
}

int isRamModule(char *m)
{
  FILE *fp = fopen(m,"r");
  ModuleFileHeader MFH;
  ModuleFilePage MFP;
  fread(&MFH, sizeof(ModuleFileHeader), 1, fp);
  fread(&MFP, sizeof(ModuleFilePage), 1, fp);
//  printf("%s --> %s\n", MFP.header.Name, MFP.header.RAM ? "RAM" : "ROM");
  fclose(fp);
  return MFP.header.RAM;
}

void dumpConfig(void)
{
  printf("Flash configuration\n========================================\n");
  printf("Page size:       0x%04X (%d)\n", PG_SIZE, PG_SIZE);
  printf("Port size:       0x%04X (%d)\n", PORT_SIZE, PORT_SIZE);
  printf("Flash start:     0x%08X (%d)\n", FLASH_START, FLASH_START);
  printf("Number of banks: %d\n", NR_BANKS);
  printf("--------------------------------\n");
  printf("XMEM page   %3d - %3d @ 0x%06X\n", XMEM_OFFS, XMEM_OFFS+XMEM_SIZE-1, XMEM_PAGE(0));
  printf("CONF page   %3d - %3d @ 0x%06X\n", CONF_OFFS, CONF_OFFS+CONF_SIZE-1, CONF_PAGE(0));
  printf("FAT page    %3d - %3d @ 0x%06X\n", FAT_OFFS, FAT_OFFS+FAT_SIZE-1, FAT_PAGE(0));
  printf("ROMMAP page %3d - %3d @ 0x%06X\n", ROMMAP_OFFS, ROMMAP_OFFS+ROMMAP_SIZE-1, ROMMAP_PAGE(0,0));
  printf("FS page     %3d - xxx @ 0x%06X\n", FS_OFFS, FS_START);
  printf("========================================\n");
  for(int p=0; p<XMEM_SIZE; p++) {
    printf("0x%06X XMEM(%d)\n", XMEM_PAGE(p), p);
  }
  for(int p=0; p<CONF_SIZE; p++) {
    printf("0x%06X CONF(%d)\n", CONF_PAGE(p), p);
  }
  for(int p=0; p<FAT_SIZE; p++) {
    printf("0x%06X FAT(%d)\n", FAT_PAGE(p), p);
  }
  for(int p=0; p<16; p++) {
    printf("0x%06X - 0x%06X ROMMAP(%d)\n", ROMMAP_PAGE(p,0), ROMMAP_PAGE(p,3), p);
  }
  printf("0x%06X FS\n", FS_START);
}

void dumpMap(char *m, int size)
{
  int rows=size/32;
  int s = 0;
  printf("\nFlash MAP (total %dMB)\n", size*PG_SIZE/(1024*1024));
  printf("         +--------------------------------+\n");
  for(int r=0; r<rows; r++) {
    if( s && (s&0xFF) == 0 )
      printf("         +--------------------------------+\n");

    printf("0x%06X |%32.32s|\n", s*PG_SIZE, m+s);
    s += 32;
  }
  printf("         +--------------------------------+\n");
}

int main(int argc, char *argv[])
{
  char buf[1024];
  FL_Head_t fl;
  int x = 8;
  int startOffs = FAT_START;
  char *p;
  char *np;
  char *name;
  long fz;
  int mod = 0;
  char *flMap = new char[1024];

  memset(flMap,' ',1024);

  FILE *cfg = fopen("fltools.cfg","r");
  FILE *bin = fopen("iflash.sh", "w");
  FILE *ini = fopen("iflash.ini", "w");
  FILE *cnf = fopen("iconf.ini", "w");
 
  dumpConfig();

  int pgOffs = (FLASH_START/PG_SIZE);
  for(int pg=0; pg<pgOffs; pg++)
    flMap[pg] = '.'; // Used for application

  for(int pg=pgOffs+XMEM_OFFS; pg<(pgOffs+XMEM_OFFS+XMEM_SIZE); pg++)
    flMap[pg] = 'X'; // Used for XMemory
  for(int pg=pgOffs+CONF_OFFS; pg<(pgOffs+CONF_OFFS+CONF_SIZE); pg++)
    flMap[pg] = 'C'; // Used for configuration
  for(int pg=pgOffs+FAT_OFFS; pg<(pgOffs+FAT_OFFS+FAT_SIZE); pg++)
    flMap[pg] = 'F'; // Used for FAT
  for(int pg=pgOffs+ROMMAP_OFFS; pg<(pgOffs+ROMMAP_OFFS+ROMMAP_SIZE); pg++)
    flMap[pg] = 'R'; // Used for ROMMAP

  initBin(bin, startOffs);
  initConf(bin, CONF_PAGE(0));
  initFat(bin, FAT_PAGE(0));

  // Point to start of FS (after FAT and FFS)
  // x += FAT_SIZE + ROMMAP_SIZE;
  x = 0;
  // Read file info from config file
	while(fgets(buf, 1024, cfg)) {
    if( buf[0] == '#' )
      continue;
    memset(&fl, 0, sizeof(FL_Head_t));
    fl.type = FL_NONE;
    p = &buf[strlen(buf)-1];
    if( *p == 0x0A )
      *p = 0;
    //printf("Check <%s>\n", buf);
    np = strchr(buf,' ');
    if( np )  {
      *np = 0;
      p = np-1;
      np++;
      while(*np == ' ')
        np++;
      name = np;
      //printf("Found name! <%s>\n", np);
    } else {
      name = 0;
//      printf("No space found ...\n");
    }
    fz = GetFileSize(buf);
    if( fz == -1 ) {
      char *n=&buf[strlen(buf)-1];
      while( n>buf && *(n-1) != '/')
        n--;
      printf("Warning: File <%s> not found ...\n",  n);
      continue;
    }
    // Get offset to next free flash page
    //fl.offs = startOffs + x * PG_SIZE;
    fl.offs = FS_PAGE(x) ;
    mod++;
    saveBin(bin, mod, fl.offs, buf, fz);
    while(*p != '.')
      p--;
    if( !strcasecmp(p+1, "MOD") ) {
      if( isRamModule(buf) )
        fl.type = FL_RAM;
      else
        fl.type = FL_MOD;
    }
    if( !strcasecmp(p+1, "ROM") )
      fl.type = FL_ROM;
    *p-- = 0;
    while(*p != '/')
      p--;
    if( name )
      sprintf(fl.name,"%.16s", name);
    else
      sprintf(fl.name,"%.16s", p+1);
    // Write the FAT entry
    fwrite(&fl, sizeof(FL_Head_t), 1, ini);
    // Count and skip number of 4k pages
    while(fz>0) {
      flMap[pgOffs+FS_OFFS+x] = 'M'; // Used for ROMMAP
      x++;
      fz -= PG_SIZE;
    }
  }
  // Write the last (empty) FAT entry
  memset(&fl, 0, sizeof(FL_Head_t));
  fwrite(&fl, sizeof(FL_Head_t), 1, ini);
  uint8_t *conf = new uint8_t[CONF_SIZE*PG_SIZE];
  memset(conf, 0, CONF_SIZE*PG_SIZE);
  fwrite(conf, sizeof(FL_Head_t), 1, cnf);

  dumpMap(flMap, 1024);

  delete[] flMap;

  fclose(cnf);
  fclose(ini);
  fclose(bin);
  fclose(cfg);
}