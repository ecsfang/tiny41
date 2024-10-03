#include <stdio.h>
#include <string.h>
#include <memory.h>
#include <stdint.h>
#include "fltools.h"
#include <sys/stat.h>
#include "flconfig.h"
#define NO_EXTERNAL
#include "../modfile.h"



void initBin(FILE *bin, int offs)
{
  fprintf(bin, "#!/bin/bash\n\n");

  fprintf(bin, "# Load a ROM module at offset 512KB in flash\n");
  fprintf(bin, "# With 4MB flash this lead to 3584KB for images\n");
  fprintf(bin, "# Every page takes 8KB -> 448 ROM images!\n\n");
  fprintf(bin, "picotool=~/Projects/picotool/build/picotool\n\n");

  fprintf(bin, "# Flash default load configuration\n");
  fprintf(bin, "echo Flash load configuration\n");
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

int isRam(char *m)
{
  FILE *fp = fopen(m,"r");
  ModuleFileHeader MFH;
  ModuleFilePage MFP;
  fread(&MFH, sizeof(ModuleFileHeader), 1, fp);
  fread(&MFP, sizeof(ModuleFilePage), 1, fp);
  printf("%s --> %s\n", MFP.header.Name, MFP.header.RAM ? "RAM" : "ROM");
  fclose(fp);
  return MFP.header.RAM;
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

  FILE *cfg = fopen("fltools.cfg","r");
  FILE *bin = fopen("iflash.sh", "w");
  FILE *ini = fopen("iflash.ini", "w");
 
  initBin(bin, startOffs);
  // Point to start of FS (after FAT and FFS)
  x += FAT_SIZE + FFS_SIZE;
  // Read file info from config file
	while(fgets(buf, 1024, cfg)) {
    if( buf[0] == '#' )
      continue;
    memset(&fl, 0, sizeof(FL_Head_t));
    fl.type = FL_NONE;
    p = &buf[strlen(buf)-1];
    if( *p == 0x0A )
      *p = 0;
    printf("Check <%s>\n", buf);
    np = strchr(buf,' ');
    if( np )  {
      *np = 0;
      p = np-1;
      np++;
      while(*np == ' ')
        np++;
      name = np;
      printf("Found name! <%s>\n", np);
    } else {
      name = 0;
//      printf("No space found ...\n");
    }
    fz = GetFileSize(buf);
    if( fz == -1 ) {
      printf("Warning: File <%s> not found ...\n",  buf);
      continue;
    }
    // Get offset to next free flash page
    fl.offs = startOffs + x * PG_SIZE;
    mod++;
    saveBin(bin, mod, fl.offs, buf, fz);
    while(*p != '.')
      p--;
    if( !strcasecmp(p+1, "MOD") ) {
      if( isRam(buf) )
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
      x++;
      fz -= PG_SIZE;
    }
  }
  // Write the last (empty) FAT entry
  memset(&fl, 0, sizeof(FL_Head_t));
  fwrite(&fl, sizeof(FL_Head_t), 1, ini);
 
  fclose(ini);
  fclose(bin);
  fclose(cfg);
}