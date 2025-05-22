#ifndef __FLCONFIG_H__
#define __FLCONFIG_H__

#if defined(PIMORONI_PICOLIPO_16MB)
#define FLASH_SIZE  ((16*1024*1024)-1)
#elif defined(PIMORONI_TINY2040_8MB)
#define FLASH_SIZE  ((8*1024*1024)-1)
#elif defined(RASPBERRYPI_PICO2)
#define TULIP_MODULE
#define FLASH_SIZE  ((4*1024*1024)-1)
#elif defined(PIMORONI_PICO_PLUS2_RP2350)
#define FLASH_SIZE  (PICO_FLASH_SIZE_BYTES-1)
#ifdef PIMORONI_PICO_PLUS2_PSRAM_CS_PIN
#define PSRAM_SIZE  (8*1024*1024)
#define PSRAM_ADDR  0x11000000
#endif
#else
# ifdef NO_EXTERNAL
    // Assume a simple Pico2 board
    #define FLASH_SIZE  ((4*1024*1024)-1)
# else
#   error("No or bad board defined!")
# endif
#endif

#define PICO_FLASH_START 0x10000000

//#define RAM_SIZE    0x1000
#define PAGE_SIZE   0x1000
#define ROM_SIZE    0x1000
#define PAGE_MASK   0x0FFF
#define ADDR_MASK   0xFFFF
#define NR_PAGES    0x10
#define NR_BANKS    4
//#define LAST_PAGE   (NR_PAGES - 1)

#define PG_SIZE           (4*1024)
#define PORT_SIZE         (2*PG_SIZE)
#define FLASH_START       (128*PG_SIZE)  // Offset to flash area (512kB)

#define FL_ADDR(x)        (((x)*PG_SIZE)+FLASH_START+PICO_FLASH_START)

// Pages defines for XMemory strage
#define XMEM_OFFS         0
#define XMEM_PAGE(x)      FL_ADDR(x+XMEM_OFFS)
#define XMEM_SIZE         7

// Page to hold the saved configuration and setup
#define CONF_OFFS         (XMEM_OFFS+XMEM_SIZE)
#define CONF_PAGE(x)      FL_ADDR(x+CONF_OFFS)
#define CONF_SIZE         1

// Offset to the FFS FAT
#define FAT_OFFS          (CONF_OFFS+CONF_SIZE)
#define FAT_PAGE(x)       FL_ADDR(x+FAT_OFFS)
#define FAT_START         FL_ADDR(FAT_OFFS)
#define FAT_SIZE          4           // Size of FAT (pages)

// Offset to the ROMMAP
#define ROMMAP_OFFS       (FAT_OFFS+FAT_SIZE)
#define ROMMAP_START      FL_ADDR(ROMMAP_OFFS)
#define ROMMAP_PAGE(p,b)  FL_ADDR(ROMMAP_OFFS+(p*NR_BANKS+b)*2)
//#define ROMMAP_PAGE(p,b)  (ROMMAP_START+((p*NR_BANKS+b)*PORT_SIZE))
#define ROMMAP_SIZE       (16*4*2) // 16 pages x 4 banks x 2*4kb

#define FS_OFFS           (ROMMAP_OFFS+ROMMAP_SIZE)
#define FS_START          FL_ADDR(FS_OFFS)
#define FS_PAGE(x)        FL_ADDR(x+FS_OFFS)

#ifndef NR_BANKS
#define NR_BANKS          4
#endif

//#define FFS_PAGE_PTR(p,b) (uint16_t*)ROMMAP_PAGE(p,b)

#endif//__FLCONFIG_H__
