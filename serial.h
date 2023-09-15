#ifndef __SERIAL_H__
#define __SERIAL_H__

extern void serial_loop(void);
extern bool bTrace;

void toggle_trace(void);
void list_modules(void);

#endif//__SERIAL_H__