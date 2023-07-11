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
#define CF_DBG_DISP_ON     1
#define CF_DBG_SLCT        0
#define CF_DBG_DISP_INST   1
#define CF_USE_DISP_ON     1
#define CF_DBG_KEY         1

#define	CH9(c) 			(c&0x1FF)
#define	CH9_B03(c) 	((c&0x00F)>>0)	// Bit 0-3
#define	CH9_B47(c) 	((c&0x0F0)>>4)	// Bit 4-7
#define	CH9_B8(c) 	((c&0x100)>>8)	// Bit 9

#define REG_A	0x01
#define REG_B	0x02
#define REG_C	0x04
#define SHIFT_R	2
#define SHIFT_L	0
#define D_LONG	1
#define D_SHORT	0

const char *inst50[] = {
	"SRLDA",	"SRLDB",	"SRLDC",	"SRLDAB",
	"SRLABC",	"SLLDAB",	"SLLABC",	"SRSDA",
	"SRSDB",	"SRSDC",	"SLSDA",	"SLSDB",
	"SRSDAB",	"SLSDAB",	"SRSABC",	"SLSABC"
};
typedef struct {
	uint8_t shft;
	uint8_t reg;
	uint8_t len;
} Inst_t;
Inst_t inst50cmd[16] = {
	{ SHIFT_R|D_LONG,  REG_A, 						48 },
	{ SHIFT_R|D_LONG,  REG_B, 						48 },
	{ SHIFT_R|D_LONG,  REG_C, 						48 },
	{ SHIFT_R|D_LONG,  REG_A|REG_B, 			 8 },
	{ SHIFT_R|D_LONG,  REG_A|REG_B|REG_C, 12 },
	{ SHIFT_L|D_LONG,  REG_A|REG_B, 			 8 },
	{ SHIFT_L|D_LONG,  REG_A|REG_B|REG_C, 12 },
	{ SHIFT_R|D_SHORT, REG_A, 						 1 },
	{ SHIFT_R|D_SHORT, REG_B, 						 1 },
	{ SHIFT_R|D_SHORT, REG_C, 						 1 },
	{ SHIFT_L|D_SHORT, REG_A, 						 1 },
	{ SHIFT_L|D_SHORT, REG_B, 						 1 },
	{ SHIFT_R|D_SHORT, REG_A|REG_B, 			 1 },
	{ SHIFT_L|D_SHORT, REG_A|REG_B, 			 1 },
	{ SHIFT_R|D_SHORT, REG_A|REG_B|REG_C,  1 },
	{ SHIFT_L|D_SHORT, REG_A|REG_B|REG_C,  1 }
};

const char *inst70[] = {
	"FLLDA",	"FLLDB",	"FLLDC",	"FLLDAB",
	"FLLDABC","00570",	"FLSDC",	"FRSDA",
	"FRSDB",	"FRSDC",	"FLSDA",	"FLSDB",
	"FRSDAB",	"FLSDAB",	"FRSDABC","FLSDABC",
};
Inst_t inst70cmd[16] = {
	{ SHIFT_L, REG_A, 						 0 },
	{ SHIFT_L, REG_B, 						 0 },
	{ SHIFT_L, REG_C, 						 0 },
	{ SHIFT_L, REG_A|REG_B, 			 6 },
	{ SHIFT_L, REG_A|REG_B|REG_C,  4 },
	{ 0, 			 0, 			  				 0 },
	{ SHIFT_L, REG_C, 						 1 },
	{ SHIFT_R, REG_A, 						 1 },
	{ SHIFT_R, REG_B, 						 1 },
	{ SHIFT_R, REG_C, 						 1 },
	{ SHIFT_L, REG_A, 						 1 },
	{ SHIFT_L, REG_B, 						 1 },
	{ SHIFT_R, REG_A|REG_B, 			 1 },
	{ SHIFT_L, REG_A|REG_B, 			 1 },
	{ SHIFT_R, REG_A|REG_B|REG_C,  1 },
	{ SHIFT_L, REG_A|REG_B|REG_C,  1 }
};

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
		//return; ???????????
	}

	dreg_a &= MASK_48_BIT;
	dreg_b &= MASK_48_BIT;
	dreg_c &= REG_C_48_MASK;

#if CF_DUMP_DBG_DREGS
	printf(" A:%012llX B:%012llX C:%012llX", dreg_a, dreg_b, dreg_c);
#endif

	// Build a text form of the display
	int j = 0;

	for (i = 0; i < NR_CHARS; i++)
	{
		char cc = 0;
		char cl = 0;
		int u = 0;

		uint64_t m = 0xf;
		int b1 = ((NR_CHARS-1) - i);
		int b = 4 * b1;

		cc |= (((m << b) & dreg_a) >> b) << 0;
		cc |= (((m << b) & dreg_b) >> b) << 4;
		u = (dreg_c >> b) & 1;

		cl = (cc & 0xc0) >> 6;
		cc &= 0x3f;
		printf("(%X %X %02X)", u,  cl, cc);

		bPunct[j] = false;

		if (u) {
			// Upper ROM character
			cc |= 0x80;
		} else {
			// Convert to ASCII character
			if( cc < 0x20 )
				cc |= 0x40;
		}

		dtext[j++] = cc;

		if (cl)	{
			// Add punctuation
			bPunct[j] = true;
			dtext[j++] = " .:,"[cl];
		}
	}

	dtext[j++] = '\0';

#if CF_DISPLAY_LCD
	if (display_on)	{
		extern void UpdateLCD(char *, bool *);
		UpdateLCD(dtext, bPunct);
		printf("\n[%s]", dtext);
	} else {
		printf("\n[%s] (OFF)", dtext);
	}
#endif
}

uint64_t x;
uint64_t z;
uint64_t y;

//void updateDispReg(uint64_t data, int bShift, int bReg, int bits, const char *inst)
void updateDispReg(uint64_t data, uint8_t r)
{
	int bShift = inst50cmd[r].shft;
	int bReg = inst50cmd[r].reg;
	int bits = inst50cmd[r].len;

	uint64_t *ra = (bReg & REG_A) ? &dreg_a : NULL;
	uint64_t *rb = (bReg & REG_B) ? &dreg_b : NULL;
	uint64_t *rc = (bReg & REG_C) ? &dreg_c : NULL;

#if CF_DBG_DISP_INST
	printf("\n%s %016llX -->", inst50[r], data);
#endif

	if( (bShift & D_LONG) && bits == 48 ) {
		if( ra ) *ra = data & (MASK_48_BIT);
		if( rb ) *rb = data & (MASK_48_BIT);
		if( rc ) *rc = data & (REG_C_48_MASK);
	} else {
		int n = (bShift & D_LONG) ? 48/bits : 1;
		for(int i=0; i<n; i++) {
			uint16_t ch9 = CH9(data);
			if( bShift & SHIFT_R ) { 	// Shift RIGHT
				if( ra ) *ra = (*ra>>4) | (((uint64_t)CH9_B03(ch9)) << 44);
				if( rb ) *rb = (*rb>>4) | (((uint64_t)CH9_B47(ch9)) << 44);
				if( rc ) *rc = (*rc>>4) | (((uint64_t)CH9_B8(ch9))  << 44);
			} else {									// Shift LEFT
				if( ra ) *ra = (*ra<<4) | (CH9_B03(ch9));
				if( rb ) *rb = (*rb<<4) | (CH9_B47(ch9));
				if( rc ) *rc = (*rc<<4) | (CH9_B8(ch9));
			}
			data >>= bits; // shift 8 or 12 bits ...
		}
		if( ra ) *ra &= MASK_48_BIT;
		if( rb ) *rb &= MASK_48_BIT;
		if( rc ) *rc &= REG_C_48_MASK;
	}
	dump_dregs();
}

void rotateDispReg(uint8_t r)
{
	int bShift = inst70cmd[r].shft;
	int bReg = inst70cmd[r].reg;
	int ch = inst70cmd[r].len;

	uint64_t *ra = (bReg & REG_A) ? &dreg_a : NULL;
	uint64_t *rb = (bReg & REG_B) ? &dreg_b : NULL;
	uint64_t *rc = (bReg & REG_C) ? &dreg_c : NULL;

#if CF_DBG_DISP_INST
	printf("\n%s -->", inst70[r]);
#endif

	for(int i=0; i<ch; i++) {
		if( bShift & SHIFT_R ) { 	// Shift RIGHT
			if( ra ) ROTATE_RIGHT(*ra);
			if( rb ) ROTATE_RIGHT(*rb);
			if( rc ) ROTATE_RIGHT(*rc);
		} else {									// Shift LEFT
			if( ra ) ROTATE_LEFT(*ra);
			if( rb ) ROTATE_LEFT(*rb);
			if( rc ) ROTATE_LEFT(*rc);
		}
	}
	dump_dregs();
}

void handle_bus(int addr, int inst, int pa, uint64_t data56)
{
	printf("\nDCE:%d ADDR:%04X (%o) INST=%04X (%o)", display_ce, addr, addr, inst, inst);

	// Check for a pending instruction from the previous cycle
	if (pending_data_inst)
	{
		switch (pending_data_inst)
		{
		case INST_PRPH_SLCT:
#if CF_DBG_SLCT
			printf("\nPF AD:%02X", pa);
#endif
			display_ce = (pa == DISP_ADDR) ? 1 : 0;
			break;

		case INST_WRITE_ANNUNCIATORS:
#if CF_DBG_DISP_INST
			printf("\n%s %03llX", "WRTEN", data56 & 0xFFF);
#endif
			{
				typedef struct {
					uint8_t len;
					char ann[4];
					bool sp;
				} Annu_t;
				Annu_t annu[NR_ANNUN] = {
					{ 1, "B",  true },
					{ 2, "US", true },
					{ 1, "G",  true },
					{ 1, "R",  true },
					{ 2, "SH", true },
					{ 1, "0",  false },
					{ 1, "1",  false },
					{ 1, "2",  false },
					{ 1, "3",  false },
					{ 1, "4",  true },
					{ 1, "P",  true },
					{ 2, "AL", false }
				};
				char sBuf[24];
				extern void UpdateAnnun(char *ann);
				memset(sBuf,' ', 24);
				int pa = 0;
				for(int a=0; a<NR_ANNUN; a++) {
					if( data56 & 1<<((NR_ANNUN-1)-a) ) {
						for(int i=0; i<annu[a].len; i++)
							sBuf[pa+i] = annu[a].ann[i];
					}
					// Add space is needed ...
					pa += annu[a].len+annu[a].sp ? 1 : 0;
				}
				sBuf[pa] = 0;
				UpdateAnnun(sBuf);
			}
			break;

		case INST_SRLDA:
		case INST_SRLDB:
		case INST_SRLDC:
		case INST_SRLDAB:
		case INST_SRLABC:
		case INST_SLLDAB:
		case INST_SLLABC:
		case INST_SRSDA:
		case INST_SRSDB:
		case INST_SRSDC:
		case INST_SLSDA:
		case INST_SLSDB:
		case INST_SRSDAB:
		case INST_SLSDAB:
		case INST_SRSABC:
		case INST_SLSABC:
			updateDispReg(data56,pending_data_inst>>6);
			break;
		}
		// Clear pending flag ...
		pending_data_inst = 0;
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
		case INST_WRITE_ANNUNCIATORS:
		case INST_SRLABC:
		case INST_SRLDA:
		case INST_SRLDB:
		case INST_SRLDC:
		case INST_SRSABC:
		case INST_SLSABC:
			pending_data_inst = inst;
			break;

		case INST_SRLDAB:
		case INST_SLLABC:
		case INST_SRSDA:
		case INST_SRSDB:
		case INST_SRSDC:
		case INST_SLSDA:
		case INST_SLSDB:
		case INST_SRSDAB:
		case INST_SLSDAB:
#if CF_DBG_DISP_INST
			printf("\n%s", inst50[inst>>6]);
#endif
			break;

		case INST_FLLDA:
		case INST_FLLDB:
		case INST_FLLDC:
		case INST_FLLDAB:
		case INST_FLLDABC:
		case INST_FRSDA:
		case INST_FRSDB:
		case INST_FRSDC:
		case INST_FLSDA:
		case INST_FLSDB:
		case INST_FLSDC:
		case INST_FRSDAB:
		case INST_FLSDAB:
		case INST_FRSDABC:
		case INST_FLSDABC:
			rotateDispReg(inst>>6);
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
