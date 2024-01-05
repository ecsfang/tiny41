#ifndef __IR_LED_H__
#define __IR_LED_H__

#include "hp41_pio_irout.pio.h"

void send_to_printer(char *pByte);
void send_to_printer(uint8_t pByte);
uint16_t calculate_frame_payload(uint8_t data);
int32_t construct_frame(uint16_t data);
void send_ir_frame(uint32_t frame);
void init_ir(void);

#endif//__IR_LED_H__