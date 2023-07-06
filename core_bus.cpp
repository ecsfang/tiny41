#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/multicore.h"
#include "tiny41.h"
#include "core_bus.h"

//#include "zenrom.h"
//#include "embedrom.h"

#define CF_DUMP_DBG_DREGS  1
#define CF_DISPLAY_LCD     1
#define CF_DISPLAY_OLED    0
#define CF_DBG_DISP_ON     0
#define CF_DBG_SLCT        0
#define CF_DBG_DISP_INST   1
#define CF_USE_DISP_ON     1
#define CF_DBG_KEY         1

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

void handle_bus(int addr, int inst, int pa, uint64_t data56);

#define NUM_FRAG 100


int fclk1[NUM_FRAG];
int fclk2[NUM_FRAG];
int fsync[NUM_FRAG];
int fisa[NUM_FRAG];
int fdata[NUM_FRAG];
int fcycno[NUM_FRAG];

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
	static int bIsaEn = 0;
	static int bIsa = 0;

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
#if 0
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

#ifdef EMBED_8000_ROM
			if ((address >= LOW_EMBED_ROM_ADDR) && (address <= HIGH_EMBED_ROM_ADDR))
			{
				embed_seen++;
				drive_data_flag = 1;
				drive_data = embed_rom[address - LOW_EMBED_ROM_ADDR];
				printf("ROM: %04X -> [%03o]\n", address, drive_data);
			}
#endif
#ifdef EMBED_9000_ROM
			if ((address >= LOW_EMBED_ROM2_ADDR) && (address <= HIGH_EMBED_ROM2_ADDR))
			{
				embed_seen++;
				drive_data_flag = 1;
				drive_data = embed_zenrom_rom[address - LOW_EMBED_ROM2_ADDR];
				printf("ZEN: %04X -> [%03o] (%03o)\n", address, drive_data, instruction);
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

const uint LED_PIN = TINY2040_LED_R_PIN;

void process_bus(void)
{
    gpio_put(LED_PIN, 0);
	// Process data coming in from the bus via core 1
	while (data_in != data_out)
	{
#if 0
	  printf("\n%d: Addr:%04X %07o  Inst: %06o PeriAd:%02X Data56:%016llX",
		 data_out,
		 bus_addr[data_out],
		 bus_addr[data_out],
		 bus_inst[data_out],
		 bus_pa[data_out],
		 bus_data56[data_out]);
#else
		// Handle the bus traffic
		handle_bus(bus_addr[data_out], bus_inst[data_out], bus_pa[data_out], bus_data56[data_out]);
#endif
		data_out = (data_out + 1) % NUM_BUS_T;
	}
    gpio_put(LED_PIN, 1);
}

int every_4th_bit(uint64_t data, int n)
{
  int val = 0;
  
  for(int i=0; i<n; i++)
    {
      val |= ((data & (1<< (i*4))) >> (i*4))<<i;
    }
  
  return(val);
}

////////////////////////////////////////////////////////////////////////////////
//
// This function is passed all of the traffic on the bus. It can do
// various things with that.
//


unsigned char upperChars[64] = {
	7f 61 62 63 64 65 00 60 06 04 05 01 0c 1d 7e 0d
	 
}
int pending_data = 0;
int pending_data_inst = 0;

uint64_t dreg_a = 0, dreg_b = 0;
uint64_t dreg_c = 0;
int display_on = 0;

void dump_dregs(void)
{
	int i;

	// If the display is off, we don't up date anything
	if (!display_on)
	{
		return;
	}

	dreg_a &= MASK_48_BIT;
	dreg_b &= MASK_48_BIT;

#if CF_DUMP_DBG_DREGS
	printf("\nA:%016llX B:%016llX C:%016llX", dreg_a, dreg_b, dreg_c);
#endif

	// Build a text form of the display

#if CF_DUMP_DBG_DREGS
	printf("\n'");
#endif

	int j = 0;

	for (i = 0; i < 12; i++)
	{
		char cc = 0;
		char cl = 0;
		int u = 0;

		uint64_t m = 0xf;
		int b1 = (11 - i);
		int b = 4 * b1;

		cc |= (((m << b) & dreg_a) >> b) << 0;
		cc |= (((m << b) & dreg_b) >> b) << 4;
//		u = ((1 << b1) & dreg_c) >> b1;
		u = (dreg_c >> b) & 1;

		cl = cc & 0xc0;
		cc &= 0x3f;
		printf("(%X %02X %02X)", u,  cl, cc);

		bPunct[j] = false;

		if ( (cc >= 0x00) && (cc <= 0x3F) )
		{
			if (u)
			{
#if CF_DUMP_DBG_DREGS
				printf("(UPPER)");
#endif
				// Upper ROM character
				switch (cc)
				{
				case 0x0d:
					cc = '#';
					break;
				}
			}
			else
			{
				// Convert to ASCII character
				if( cc < 0x20 )
					cc |= 0x40;
				dtext[j++] = cc;
#if CF_DUMP_DBG_DREGS
				printf("[%02X%c]", cc, cc);
#endif
			}
		}
/*		else
		{

			switch (cc)
			{

			case 0x00:
				cc = '@';
				dtext[j++] = cc;
#if CF_DUMP_DBG_DREGS
//				printf("%c", cc);
#endif
				break;

			case 0x2c:
			case 0x2e:
			case 0x3a:
				dtext[j++] = cc;
#if CF_DUMP_DBG_DREGS
//				printf("%c", cc);
#endif
				break;

			case 0x1E:	// ^
			case 0x1F:  // _
				cc |= 0x40; //'_';
				dtext[j++] = cc;
#if CF_DUMP_DBG_DREGS
//				printf("%c", cc);
#endif
				break;

			default:
				dtext[j++] = cc & 0x3f;
#if CF_DUMP_DBG_DREGS
//				printf("%c", cc & 0x3f);
#endif
				break;
			}
		}
*/
		if (cl)
		{
			switch (cl)
			{
			case 0x40:
				bPunct[j] = true;
				dtext[j++] = '.';
#if CF_DUMP_DBG_DREGS
//				printf(".");
#endif
				break;
			case 0x80:
				bPunct[j] = true;
				dtext[j++] = ':';
#if CF_DUMP_DBG_DREGS
//				printf(":");
#endif
				break;
			case 0xC0:
				bPunct[j] = true;
				dtext[j++] = ',';
#if CF_DUMP_DBG_DREGS
//				printf(",");
#endif
				break;
			}
		}
	}
#if CF_DUMP_DBG_DREGS
	printf("'");
#endif
	dtext[j++] = '\0';

#if CF_DISPLAY_LCD
	printf("\n[%s]", dtext);
	extern void UpdateLCD(char *, bool *);
	UpdateLCD(dtext, bPunct);
#endif

	while (strlen(dtext) < 15)
	{
		strcat(dtext, " ");
	}

#if CF_DISPLAY_OLED
	oled_set_xy(&oled0, 0, 16);
	oled_display_string(&oled0, dtext);
#endif
}

uint64_t x;
uint64_t z;
uint64_t y;

void handle_bus(int addr, int inst, int pa, uint64_t data56)
{
	//printf("\nDCE:%d ADDR:%04X (%o) INST=%04X (%o)", display_ce, addr, addr, inst, inst);

	// Check for a pending instruction from the previous cycle
	if (pending_data)
	{
		pending_data = 0;

		switch (pending_data_inst)
		{
		case INST_PRPH_SLCT:
#if CF_DBG_SLCT
			printf("\nPF AD:%02X", pa);
#endif
			if (pa == 0xFD)
			{
				display_ce = 1;
			}
			else
			{
				display_ce = 0;
			}
			break;

		case INST_SRLAD:
#if CF_DBG_DISP_INST
			printf("\nSRLAD %016llX", data56);
#endif
			// Load 48 bits into A register
			dreg_a = data56 & (MASK_48_BIT);
			dump_dregs();
			break;

		case INST_SRLDB:
#if CF_DBG_DISP_INST
			printf("\nSRLDB %016llX", data56);
#endif
			dreg_b = data56 & (MASK_48_BIT);
			dump_dregs();

			break;

		case INST_SRLDC:
#if CF_DBG_DISP_INST
			printf("\nSRLDC %016llX", data56);
#endif
			//dreg_c = every_4th_bit(data56 & (MASK_48_BIT), 12);
			dreg_c = data56 & (MASK_48_BIT);
			dump_dregs();
			break;

		case INST_SRSDABC:
#if CF_DBG_DISP_INST
			printf("\nSRSDABC %llX", data56);
#endif
			dreg_a >>= 4;
			dreg_b >>= 4;
			dreg_c >>= 4;

			y = data56;
#if CF_DBG_DISP_INST
			printf("  data56:%llX", data56);
#endif

			y = data56 & 0xF;
#if CF_DBG_DISP_INST
			printf("  Y:%016llX", y);
#endif

			y <<= 44;
#if CF_DBG_DISP_INST
			printf("  Y:%016llX", y);
#endif

			// y = (x & z) << 44;
			// printf("\nY:%016llX", y);

			// dreg_a |= (data56 & 0xF)  << (11*4);
			dreg_a |= y;
			dreg_b |= (data56 & 0xF0) << 40;
			dreg_c |= (data56 & 0x100) << (4*11-8);
			dump_dregs();
			break;

		case INST_SRLDABC:
#if CF_DBG_DISP_INST
			printf("\nSRLDABC %llX", data56);
#endif

			for (int i = 0; i < 4; i++)
			{
				dreg_a >>= 4;
				dreg_b >>= 4;
				dreg_c >>= 4;

				y = data56;
#if CF_DBG_DISP_INST
				printf("  data56:%llX", data56);
#endif

				y = data56 & 0xF;
#if CF_DBG_DISP_INST
				printf("  Y:%016llX", y);
#endif

				y <<= 44;
#if CF_DBG_DISP_INST
				printf("  Y:%016llX", y);
#endif

				// y = (x & z) << 44;
				// printf("\nY:%016llX", y);

				// dreg_a |= (data56 & 0xF)  << (11*4);
				dreg_a |= y;
				dreg_b |= (data56 & 0xF0) << 40;
				dreg_c |= (data56 & 0x100) << (4*11-8);

				data56 >>= 12;
			}
			dump_dregs();
			break;

		case INST_SLSDABC:
#if CF_DBG_DISP_INST
			printf("\nSLSDABC %llX", data56);
#endif
			dreg_a <<= 4;
			dreg_b <<= 4;
			dreg_c <<= 4;

			y = data56;
#if CF_DBG_DISP_INST
			printf("  data56:%llX", data56);
#endif

			y = data56 & 0xF;
#if CF_DBG_DISP_INST
//			printf("  Y:%016llX", y);
#endif

			y <<= 0;
#if CF_DBG_DISP_INST
//			printf("  Y:%016llX", y);
#endif

			dreg_a |= y;
			dreg_b |= (data56 & 0xF0) >> 4;
			dreg_c |= (data56 & 0x100) >> 8;

#if CF_DUMP_DBG_DREGS
	printf("--> A:%016llX B:%016llX C:%016llX", dreg_a, dreg_b, dreg_c);
#endif

			dump_dregs();
			break;
		}
	}

	switch (inst)
	{
	case INST_DISPLAY_OFF:
#if CF_DBG_DISP_ON
		printf("\nDisplay off");
#endif
		display_on = 0;
		break;

	case INST_DISPLAY_TOGGLE:
#if CF_DBG_DISP_ON
		printf("\nDisplay toggle");
#endif
		display_on = !display_on;
		break;

	case INST_PRPH_SLCT:
		pending_data = 1;
		pending_data_inst = INST_PRPH_SLCT;
		break;
	}

	// Check for instructions
	switch (inst)
	{
	case INST_PRPH_SLCT:
#if CF_DBG_SLCT
		printf("\nPRPH_SLCT: PA=%02X", pa);
#endif
		break;

	case INST_RAM_SLCT:
#if CF_DBG_SLCT
		printf("\nRAM_SLCT: PA=%02X", pa);
#endif
		break;
	}

	// Check for display transactions
	if (display_ce)
	{
		switch (inst)
		{
		case INST_SRLDAB:
#if CF_DBG_DISP_INST
			printf("\nSRLDAB");
#endif
			break;

		case INST_SRLDABC:
#if CF_DBG_DISP_INST
			printf("\nSRLDABC");
#endif
			pending_data = 1;
			pending_data_inst = INST_SRLDABC;
			break;

		case INST_SRLAD:
#if CF_DBG_DISP_INST
			printf("\nSRLAD");
#endif
			pending_data = 1;
			pending_data_inst = INST_SRLAD;
			break;

		case INST_SRLDB:
#if CF_DBG_DISP_INST
			printf("\nSRLDB");
#endif
			pending_data = 1;
			pending_data_inst = INST_SRLDB;
			break;

		case INST_SRLDC:
#if CF_DBG_DISP_INST
			printf("\nSRLDC");
#endif
			pending_data = 1;
			pending_data_inst = INST_SRLDC;
			break;

		case INST_SLLDABC:
#if CF_DBG_DISP_INST
			printf("\nSLLDABC");
#endif
			break;

		case INST_SRSDA:
#if CF_DBG_DISP_INST
			printf("\nSRSDA");
#endif
			break;

		case INST_SRSDB:
#if CF_DBG_DISP_INST
			printf("\nSRSDB");
#endif
			break;

		case INST_SRSDC:
#if CF_DBG_DISP_INST
			printf("\nSRSDC");
#endif
			break;

		case INST_SLSDA:
#if CF_DBG_DISP_INST
			printf("\nSLSDA");
#endif
			break;

		case INST_SLSDB:
#if CF_DBG_DISP_INST
			printf("\nSLSDB");
#endif
			break;

		case INST_SRSDAB:
#if CF_DBG_DISP_INST
			printf("\nSRSDAB");
#endif
			break;

		case INST_SLSDAB:
#if CF_DBG_DISP_INST
			printf("\nSLSDAB");
#endif
			break;

		case INST_SRSDABC:
#if CF_DBG_DISP_INST
			printf("\nSRSDABC data56:%08llX", data56);
#endif
			pending_data = 1;
			pending_data_inst = INST_SRSDABC;
			break;

		case INST_SLSDABC:
#if CF_DBG_DISP_INST
			printf("\nSLSDABC");
#endif
			pending_data = 1;
			pending_data_inst = INST_SLSDABC;
			break;

		case INST_FLLDA:
#if CF_DBG_DISP_INST
			printf("\nFLLDA:");
#endif
			break;

		case INST_FLLDB:
#if CF_DBG_DISP_INST
			printf("\nFLLDB:");
#endif
			break;

		case INST_FLLDC:
#if CF_DBG_DISP_INST
			printf("\nFLLDC:");
#endif
			break;

		case INST_FLLDAB:
#if CF_DBG_DISP_INST
			printf("\nFLLDAB:");
#endif
			break;

		case INST_FLLDABC:
#if CF_DBG_DISP_INST
			printf("\nFLLDABC:");
#endif
			break;

		case INST_FLSDC:
#if CF_DBG_DISP_INST
			printf("\nFLSDC:");
#endif
			break;

		case INST_FRSDA:
#if CF_DBG_DISP_INST
			printf("\nFRSDA:");
#endif
			break;

		case INST_FRSDB:
#if CF_DBG_DISP_INST
			printf("\nFRSDB:");
#endif
			break;

		case INST_FRSDC:
#if CF_DBG_DISP_INST
			printf("\nFRSDC:");
#endif
			break;

		case INST_FLSDA:
#if CF_DBG_DISP_INST
			printf("\nFLSDA:");
#endif
			break;
		case INST_FLSDB:
#if CF_DBG_DISP_INST
			printf("\nFLSDB:");
#endif
			break;
		case INST_FRSDAB:
#if CF_DBG_DISP_INST
			printf("\nFRSDAB:");
#endif
			break;
		case INST_FLSDAB:
#if CF_DBG_DISP_INST
			printf("\nFLSDAB:");
#endif
			break;

		case INST_FRSDABC:
#if CF_DBG_DISP_INST
			printf("\nFRSDABC data56:%016llX", data56);
#endif
			ROTATE_RIGHT(dreg_a);
			ROTATE_RIGHT(dreg_b);
			dump_dregs();
			break;

		case INST_FLSDABC:
#if CF_DBG_DISP_INST
			printf("\nFLSDABC:");
#endif
			ROTATE_LEFT(dreg_a);
			ROTATE_LEFT(dreg_b);
			dump_dregs();
			break;
		}
	}
}

void display_bus_fragment(void)
{
	int last_sync;
	int last_clk1;

	irq_set_mask_enabled(0xffffffff, false);

	last_sync = gpio_get(P_SYNC);
	last_clk1 = gpio_get(P_CLK1);

	for (int i = 0; i < NUM_FRAG; i++)
	{
		fclk1[i] = gpio_get(P_CLK1);
		fclk2[i] = gpio_get(P_CLK2);
		fsync[i] = gpio_get(P_SYNC);
		fisa[i] = gpio_get(P_ISA);
		fdata[i] = gpio_get(P_DATA);

		// If clk1 has a low edge then count it
		if ((last_clk1 == 1) && (fclk1[i] == 0))
		{
			if (i == 0)
			{
				fcycno[i] == 0;
			}
			else
			{
				fcycno[i] = fcycno[i - 1] + 1;
			}
		}
		else
		{
			if (i == 0)
			{
				fcycno[i] == 0;
			}
			else
			{
				fcycno[i] = fcycno[i - 1];
			}
		}

		if ((last_sync == 1) && (fsync[i] == 0))
		{
			// Start of cycle
			fcycno[i] = 0;
		}

		last_clk1 = fclk1[i];
		last_sync = fsync[i];
	}

	irq_set_mask_enabled(0xffffffff, true);

	// Display on USB
	for (int i = 0; i < NUM_FRAG; i++)
	{
		printf("\n%04X: clk1=%d clk2=%d sync=%d isa=%d data=%d cyc=%02d", i, fclk1[i], fclk2[i], fsync[i], fisa[i], fdata[i], fcycno[i]);
	}
	printf("\n");
}

#define NUM_TRANS      100
int trans_addr[NUM_TRANS];
int trans_data[NUM_TRANS];
int trans_instruction[NUM_TRANS];
int trans_pa[NUM_TRANS];
uint64_t trans_data56[NUM_TRANS];

#define TDB 0

void capture_bus_transactions(void)
{
  int last_sync   = 0;
  int trans_i     = 0;
  int data        = 0;
  int periph_addr = 0;
  int instruction = 0;
  int bit_no      = 0;
  int gpio_states = 0;
  int sync        = 0;
  int address     = 0;
  int  data56      = 0;
  
  gpio_states = sio_hw->gpio_in;
  last_sync = gpio_states & (1<<P_SYNC);  

  while(1)
    {
      // Wait for CLK1 to have a falling edge
      // Wait while low
      while( ((gpio_states = sio_hw->gpio_in) & (1 << P_CLK1)) == 0 )
	{
	}

      // Now high, wait for falling edge
      while( ((gpio_states = sio_hw->gpio_in) & (1 << P_CLK1)) != 0 )
	{
	}

      // Another bit, check SYNC to find bit number
      sync = gpio_states & (1<<P_SYNC);

      if( (last_sync == 0) && (sync != 0) )
	{
#if TDB	  
	  printf("\n44");
#endif
	  bit_no = 44;
	}
      else
	{
	  bit_no = (bit_no + 1) % 56;
#if TDB	  
	  printf("\n%d %08X", bit_no, gpio_states);
#endif
	}
      
      // Get data and put into data or instruction if bit no is OK for those fields
      if( (bit_no >= 44) && (bit_no <= 53) )
	{
	  instruction |= ((gpio_states & (1<<P_ISA))>>(P_ISA)) << ((bit_no - 44));
#if TDB	  
	  printf("\nI=%d (%08X)", instruction, gpio_states & (1<<P_ISA));
#endif

	}
      
      if( (bit_no >= 14) && (bit_no <= 29) )
	{
	  address |= ((gpio_states & (1<<P_ISA))>>(P_ISA)) << ((bit_no - 14));
#if TDB
	  printf("\nA=%d (%08X)", address, gpio_states & (1<<P_ISA));
#endif

	}

      if( (bit_no >= 0) && (bit_no <= 7) )
	{
	  periph_addr |= ((gpio_states & (1<<P_DATA))>>(P_DATA)) << ((bit_no - 0));
	}

      if( (bit_no >= 0) && (bit_no <= 55) )
	{
	  data56 |= ((gpio_states & (1<<P_DATA))>>(P_DATA)) << ((bit_no - 0));
	}
      
      // If bitno = 55 then we have another frame, store the transaction
      if( bit_no == 55 )
	{
	  trans_addr[trans_i]        = address;
	  trans_instruction[trans_i] = instruction;
	  trans_pa[trans_i]          = periph_addr;
	  trans_data56[trans_i]      = data56;
	  
	  address      = 0;
	  instruction  = 0;
	  data56       = 0;
	  periph_addr  = 0;
	  trans_i++;
	}
      
      if( trans_i == NUM_TRANS )
	{
	  // All done, exit
	  break;
	}
      
      last_sync = sync;
    }

  // Display what we captured
  for( int i=0; i<NUM_TRANS; i++)
    {
      printf("\n%04X: Addr:%04X %07o  Inst: %06o PeriAd:%02X Data56:%08X",
	     i,
	     trans_addr[i],
	     trans_addr[i],
	     trans_instruction[i],
	     trans_pa[i],
	     trans_data56[i]);
    }
  
  printf("\n");
}
