#ifndef __FLTOOLS_H__
#define __FLTOOLS_H__

typedef enum {
  FL_NONE = 0,
  FL_ROM  = 1<<0,
  FL_MOD  = 1<<1,
  FL_RAM  = 1<<2,
} FL_Type_e;

typedef struct {
  uint32_t  offs;       //  4 bytes
  char      name[23+1]; // 24 bytes
  FL_Type_e type;       //  4 bytes
} FL_Head_t;            // 32 bytes in total

#endif//__FLTOOLS_H__