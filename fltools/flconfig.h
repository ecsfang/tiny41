#ifndef __FLCONFIG_H__
#define __FLCONFIG_H__

#define PICO_FLASH_START 0x10000000

#define PG_SIZE     (4*1024)
#define FLASH_START (128*PG_SIZE)  // Offset to flash area (512kB)

#define CONF_PAGE   ((7*PG_SIZE)+FLASH_START+PICO_FLASH_START)

// Offset to FAT
#define FAT_START   (8*PG_SIZE)+FLASH_START+PICO_FLASH_START
#define FAT_SIZE    (4)           // Size of FAT (pages)

#define FFS_START   FAT_START+(FAT_SIZE*PG_SIZE)
#define FFS_SIZE    (16*4*2)

#define FS_START    FFS_START+(FFS_SIZE*PG_SIZE)

#ifndef NR_BANKS
#define FFS_PAGE(p,b) (FFS_START+((p*4+b)*2*PG_SIZE))
#else
#define FFS_PAGE(p,b) (FFS_START+((p*NR_BANKS+b)*2*PG_SIZE))
#endif
#define FFS_PAGE_PTR(p,b) (uint16_t*)FFS_PAGE(p,b)

#endif//__FLCONFIG_H__
