#include <stdio.h>
#include <string.h>
#include <memory.h>
#include <stdint.h>
#include "fltools.h"
#include <sys/stat.h>

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
  fprintf(bin, "\necho Flash to %d\n", x);
  fprintf(bin, "mod=\"%s\" # The ROM file to flash (%ld bytes)\n", rom, size );
  fprintf(bin, "sudo $picotool load -v $mod -t bin -o 0x%X\n", offs);
}

long GetFileSize(const char *filename)
{
    struct stat stat_buf;
    int rc = stat(filename, &stat_buf);
    return rc == 0 ? stat_buf.st_size : -1;
}

int main(int argc, char *argv[])
{
  char buf[1024];
  FL_Head_t fl;
  int x = 0;
  int startOffs = 0x10080000;
  char *p;
  long fz;

  FILE *cfg = fopen("fltools.cfg","r");
  FILE *bin = fopen("iflash.sh", "w");
  FILE *ini = fopen("iflash.ini", "w");
 
  initBin(bin, startOffs);
  x += 2;
	while(fgets(buf, 1024, cfg)) {
    if( buf[0] == '#' )
      continue;
    memset(&fl, 0, sizeof(FL_Head_t));
    fl.type = FL_NONE;
    p = &buf[strlen(buf)-1];
    if( *p == 0x0A )
      *p = 0;
    fz = GetFileSize(buf);
    fl.offs = startOffs + x * 4 * 1024;
    saveBin(bin, x, fl.offs, buf, fz);
    while(*p != '.')
      p--;
    if( !strcasecmp(p+1, "MOD") )
      fl.type = FL_MOD;
    if( !strcasecmp(p+1, "ROM") )
      fl.type = FL_ROM;
    *p-- = 0;
    while(*p != '/')
      p--;
    sprintf(fl.name,"%.16s", p+1);
    fwrite(&fl, sizeof(FL_Head_t), 1, ini);
    // Count number of 4k pages
    while(fz>0) {
      x++;
      fz -= 0x1000;
    }
  }
  memset(&fl, 0, sizeof(FL_Head_t));
  fwrite(&fl, sizeof(FL_Head_t), 1, ini);
 
  fclose(ini);
  fclose(bin);
  fclose(cfg);
}