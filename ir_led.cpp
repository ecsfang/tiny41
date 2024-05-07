#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/multicore.h"
#include "tiny41.h"
#include "ir_led.h"
#include "usb/cdc_helper.h"

char irBuf[32];
int nIr;
// code for the main loop in core0
// gets PrintChar from a queue in core1
void send_to_printer(char *pByte)
{
  while( *pByte ) {
    send_to_printer((uint8_t)*pByte);
    pByte++;
  }
}

void dump_frame(uint32_t frame)
{
  nIr = 0;
  for(int i=31; i>4; i--) {
    nIr += sprintf(irBuf+nIr, "%c", frame & (1<<i) ? '#':'_');
  }
  cdc_send_console(irBuf);
}
void send_to_printer(uint8_t pByte)
{
  // send the printcharacter to the IR LED
  uint16_t ir_code = calculate_frame_payload(pByte);
  uint32_t ir_frame = construct_frame(ir_code);
  send_ir_frame(ir_frame);
  // line below for debugging the construction of the IR frame
#if 0
  sprintf(irBuf, "IR char = %02X, code = %04X, frame = ", pByte, ir_code);
  cdc_send_console(irBuf);
  dump_frame(ir_frame);
  cdc_send_console((char*)"\n\r");
  cdc_flush_console();
#endif
}

PIO pio;
uint32_t irout_sm;      // claim a state machine in pio1    
uint32_t irout_offset;  // in pio 1 !!

// to initialize the state machine
void init_ir(void)
{
    // Choose PIO instance (0 or 1)
    pio = pio0;

    irout_sm = pio_claim_unused_sm(pio, true);                          // claim a state machine in pio1    
    irout_offset = pio_add_program(pio, &hp41_pio_irout_program);       // in pio 1 !!
    hp41_pio_irout_program_init(pio, irout_sm, irout_offset, 0, P_IR_LED, 0);    // uses only sideset
}

// functions for IR output

// Calculate Frame Payload with Parity
// from https://github.com/vogelchr/avr-redeye/blob/master/avr-redeye.c
   
   /*   And one frame consists of <start> 12*(<one>|<zero>), the order for
    the 12 payload bits in a frame is:

          d c b a 7 6 5 4 3 2 1 0

   7..0 are data, a,b,c,d are parity bits, so that, if the frame payload
   is written as a 12-bit integer the following subsets of bits must have
   a even numbers of bits set:

   +---+---+---+---++---+---+---+---++---+---+---+---++--------+
   : d : c : b : a :: 7 : 6 : 5 : 4 :: 3 : 2 : 1 : 0 ::   Hex  :
   +---+---+---+---++---+---+---+---++---+---+---+---++--------+
   : X :   :   :   ::   : X : X : X :: X :   :   :   :: 0x0878 :
   +---+---+---+---++---+---+---+---++---+---+---+---++--------+
   :   : X :   :   :: X : X : X :   ::   : X : X :   :: 0x04e6 :
   +---+---+---+---++---+---+---+---++---+---+---+---++--------+
   :   :   : X :   :: X : X :   : X ::   : X :   : X :: 0x02d5 :
   +---+---+---+---++---+---+---+---++---+---+---+---++--------+
   :   :   :   : X :: X :   :   :   :: X :   : X : X :: 0x018b :
   +---+---+---+---++---+---+---+---++---+---+---+---++--------+

   */

char parity(uint8_t byte)
{
	byte = byte ^ (byte >> 4); /* merge 7..4 -> 3..0 */
	byte = byte ^ (byte >> 2); /* merge 3..2 -> 1..0 */
	byte = byte ^ (byte >> 1); /* merge    1 ->    0 */
	return byte & 0x01;
}

uint16_t calculate_frame_payload(uint8_t data)
{
    // this payload has the msb as the first bit to be sent!
	int16_t frame = data;
	if(parity(data & 0x78)) frame |= 0x800;
	if(parity(data & 0xe6)) frame |= 0x400;
	if(parity(data & 0xd5)) frame |= 0x200;
	if(parity(data & 0x8b)) frame |= 0x100;
	return frame;
}


int32_t construct_frame(uint16_t data)
{
    // data contains a 12 bit value:
    //  d c b a 7 6 5 4 3 2 1 0          (dcba= checksum, 76543210 is the data payload)

    uint32_t frame = 0b111;                 // start bits           
                                            // frame = 0000.0111
    for (int i = 0; i < 12 ; i++)
    {
        // go through all 12 bits and construct new frame
        // a 0 bit expands to 10
        // a 1 bit expands to 01
        frame = frame << 2;                 
        if ((data & 0x800) == 0x800) {           
            // msb is set
            frame = frame | 0x2;              // OR with 0b10
        }                  
        else
        {
            // msb is not set
            frame = frame | 0x01;             // OR with 0b01
        }
        data = data << 1;
    }

    // frame now contains the following for character A for example:
    // 0000.0111.1010.0110.0110.0101.0101.0110
    //       ^^^ start bits
    //           ^^^........................^^  payload 24 bits
    // this must now be left aligned for proper sending, else the first 0's are sent
    // ensure in the state machine that only 24+3=27 bits are transitted !
    frame = frame << 5;

    return frame;
}

//  to send a frame, the bits need to be split into half-bits:
//      start frame :  3 hi-lo transitions
//      1-bit       :  1 hi-lo, 1 lo-lo 
//      0-bit       :  1 lo-lo, 1 hi-lo
//  the irout state machine sends the following:
//      input 0-bit :  send lo-lo frame
//      input 1-bit :  send hi-lo frame
//
// for sending a 1 bit: put 01 in the output frame (lsb sent first)
// for sending a 0 bit: put 10 in the output frame (lsb sent first)

void send_ir_frame(uint32_t frame)
{
    // not much to do, just dump the formatted frame in the TX FIFO of the irout state machine
    // note that this is a blocking function!
    pio_sm_put_blocking(pio, irout_sm, frame);  // send the data
}
