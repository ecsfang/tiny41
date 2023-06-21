#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/multicore.h"
#include "tiny41.h"

#include "zenrom.h"

////////////////////////////////////////////////////////////////////////////////
//
// Core 1 sits in a loop grabbing bus cycles
//

int last_clk1 = 0;
int last_clk2 = 0;
int last_isa = 0;
int last_data = 0;

int clk1 = 0;
int clk2 = 0;
int isa = 0;

#define RISING_EDGE(SIGNAL) ((last_##SIGNAL == 0) && (SIGNAL == 1))
#define FALLING_EDGE(SIGNAL) ((last_##SIGNAL == 1) && (SIGNAL == 0))

#define NUM_DUMP 100
#define NUM_DUMP_SIGNAL 300

int dump_i = 0;
long dump_frames[NUM_DUMP];

int dump_signal_i = 0;
int dump_sync[NUM_DUMP_SIGNAL];

#define DIRECT_GPIO 1

// Do we drive ROM data?
int drive_data_flag = 0;

// The data we drive
int drive_data = 0;

// How many times have we driven ISA?
volatile int driven_isa = 0;
volatile int embed_seen = 0;
volatile int sync_count = 0;

////////////////////////////////////////////////////////////////////////////////
//
// Core1 main functions
//
// Various different core 1 main functions used to test functionality
//
////////////////////////////////////////////////////////////////////////////////

// Data transfer to core 0

#define NUM_BUS_T 7000

int queue_overflow = 0;

volatile int data_in = 0;
volatile int data_out = 0;

volatile int bus_addr[NUM_BUS_T];
volatile int bus_inst[NUM_BUS_T];
volatile int bus_pa[NUM_BUS_T];
volatile uint64_t bus_data56[NUM_BUS_T];
int last_sync = 0;
int trans_i = 0;
int data = 0;
int periph_addr = 0;
int instruction = 0;
int bit_no = 0;
int gpio_states = 0;
int sync = 0;
int address = 0;
uint64_t data56 = 0;

void core1_main_3(void)
{
	int bIsaEn = 0;
	int bIsa = 0;

	irq_set_mask_enabled(0xffffffff, false);

	gpio_states = sio_hw->gpio_in;
	last_sync = gpio_states & (1 << P_SYNC);

	while (1)
	{
		// Wait for CLK2 to have a falling edge
		// Wait while low
		while (((gpio_states = sio_hw->gpio_in) & (1 << P_CLK2)) == 0)
		{
		}

		while (((gpio_states = sio_hw->gpio_in) & (1 << P_CLK2)) != 0)
		{
		}

		// Another bit, check SYNC to find bit number
		sync = gpio_states & (1 << P_SYNC);

		// Do we drive the ISA line?
		// Bit numbers are out by one as bit_no hasn't been incremented yet.

		if ((bit_no >= 43) && (bit_no <= 52))
		{
#if 1
			if (/*(sync!=0) &&*/ drive_data_flag)
			{
				// Drive the ISA line for this data bit
				if (!bIsaEn)
				{
					gpio_set_dir(P_ISA, GPIO_OUT);
					bIsaEn = 1;
				}

				//  gpio_put(P_ISA_OE, 0);
				bIsa = drive_data & 1;
				gpio_put(P_ISA, bIsa);
				drive_data >>= 1;
				driven_isa++;
			}
			else
			{
				// Do not drive data
				// gpio_put(P_ISA_OE, 1);
				if (bIsaEn)
				{
					gpio_set_dir(P_ISA, GPIO_IN);
					bIsaEn = 0;
				}
			}
#endif
		}

		if (bit_no == 52)
		{
			// Don't drive data any more
			drive_data_flag = 0;
		}

		if (bit_no == 53)
		{
			// Don't drive data any more
			// gpio_put(P_ISA_OE, 1);
			if (bIsaEn)
			{
				gpio_set_dir(P_ISA, GPIO_IN);
				bIsaEn = 0;
			}
		}

		// Now high, wait for falling edge
		while (((gpio_states = sio_hw->gpio_in) & (1 << P_CLK1)) == 0)
		{
		}

		// Now high, wait for falling edge
		while (((gpio_states = sio_hw->gpio_in) & (1 << P_CLK1)) != 0)
		{
		}

		// Another bit, check SYNC to find bit number
		sync = gpio_states & (1 << P_SYNC);

		if ((last_sync == 0) && (sync != 0))
		{
			sync_count++;
			bit_no = 44;
		}
		else
		{
			bit_no = (bit_no + 1) % 56;
		}

		// Get data and put into data or instruction if bit no is OK for those fields
		// If the SYNC is high and we are to drive the ISA line then if we have embedded data
		// then we drive the ISA line.

		if ((bit_no >= 44) && (bit_no <= 53))
		{
			instruction |= ((gpio_states & (1 << P_ISA)) >> (P_ISA)) << (bit_no - 44);
		}

		if ((bit_no >= 14) && (bit_no <= 29))
		{
			address |= ((gpio_states & (1 << P_ISA)) >> (P_ISA)) << ((bit_no - 14));
		}

		// Got address. If the address is that of the embedded ROM then we flag that we have
		// to put an instruction on the bus later
		if (bit_no == 29)
		{
			drive_data_flag = 0;

#if EMBED_8000_ROM
			if ((address >= LOW_EMBED_ROM_ADDR) && (address <= HIGH_EMBED_ROM_ADDR))
			{
				embed_seen++;
				drive_data_flag = 1;
				drive_data = embed_rom[address - LOW_EMBED_ROM_ADDR];
			}
#endif
#ifdef EMBED_9000_ROM
			if ((address >= LOW_EMBED_ROM2_ADDR) && (address <= HIGH_EMBED_ROM2_ADDR))
			{
				embed_seen++;
				drive_data_flag = 1;
				drive_data = embed_zenrom_rom[address - LOW_EMBED_ROM2_ADDR];
			}

#endif
		}
		if ((bit_no >= 0) && (bit_no <= 7))
		{
			periph_addr |= ((gpio_states & (1 << P_DATA)) >> (P_DATA)) << ((bit_no - 0));
		}

		if ((bit_no >= 0) && (bit_no <= 55))
		{
			uint64_t v;
			v = ((gpio_states & (1 << P_DATA)) >> (P_DATA));
			v <<= ((bit_no - 0));
			data56 |= v;
		}

		// If bitno = 55 then we have another frame, store the transaction
		if (bit_no == 55)
		{
			// A 56 bit frame has completed, we send some information to core0 so it can update
			// the display and anything else it needs to

			bus_addr[data_in] = address;
			bus_inst[data_in] = instruction;
			bus_pa[data_in] = periph_addr;
			bus_data56[data_in] = data56;

			address = 0;
			instruction = 0;
			periph_addr = 0;
			data56 = 0;

			int last_data_in = data_in;

			data_in = (data_in + 1) % NUM_BUS_T;

			if (data_out == data_in)
			{
				// No space
				queue_overflow = 1;
				data_in = last_data_in;
			}
			else
			{
			}
		}

		last_sync = sync;
	}
}
