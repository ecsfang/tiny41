#ifndef __FLTOOLS_H__
#define __FLTOOLS_H__

typedef enum {
  FL_NONE = 0,
  FL_ROM  = 1<<0,
  FL_MOD  = 1<<1,
  FL_RAM  = 1<<2,
} FL_Type_e;

typedef struct {
  uint32_t  offs;
  char      name[23+1];
  FL_Type_e type;
} FL_Head_t;

#endif//__FLTOOLS_H__