#ifndef __SERIAL_H__
#define __SERIAL_H__

extern void serial_loop(void);
extern volatile uint8_t bTrace;

void toggle_trace(void);
void toggle_disasm(void);
void list_modules(void);
int send2console(const char* dispBuf, bool bClear);

#endif//__SERIAL_H__