#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/multicore.h"
#include "tiny41.h"
#include "core_bus.h"
#include "disasm.h"

#include "zenrom.h"
//#include "embedrom.h"
#include "ppcrom.rom"

#ifdef EMBED_ZEN_ROM
Module_t	zen = {
	LOW_EMBED_ROM2_ADDR,
	HIGH_EMBED_ROM2_ADDR,
	embed_zenrom_rom
};
#endif

#ifdef EMBED_8000_ROM
Module_t	emb = {
	LOW_EMBED_ROM_ADDR,
	HIGH_EMBED_ROM_ADDR,
	embed_rom
};
#endif

#ifdef EMBED_PPC_ROM
Module_t	ppc = {
	LOW_EMBED_PPC_ROM_ADDR,
	HIGH_EMBED_PPC_ROM_ADDR,
	embed_ppc_rom
};
#endif

Module_t *roms[16];

void addRom(Module_t *r)
{
	roms[r->start>>12] = r;
	roms[r->end>>12] = r;
	printf("Add rom @ %04X - %04X\n", r->start, r->end);
}
void initRom()
{
	memset(roms, 0, sizeof(roms));
#ifdef EMBED_PPC_ROM
	addRom(&ppc);
#endif
#ifdef EMBED_8000_ROM
	addRom(&emb);
#endif
#ifdef EMBED_ZEN_ROM
	addRom(&zen);
#endif
}

#define DRIVE_ISA

void disAsm(int inst, int addr, uint64_t data);

#define CF_DUMP_DBG_DREGS  0
#define CF_DISPLAY_LCD     1
#define CF_DISPLAY_OLED    0
#define CF_DBG_DISP_ON     1
#define CF_DBG_SLCT        0
#define CF_DBG_DISP_INST   0
#define CF_USE_DISP_ON     1
#define CF_DBG_KEY         1

#define	CH9(c) 		(c&0x1FF)
#define	CH9_B03(c) 	((c&0x00F)>>0)	// Bit 0-3
#define	CH9_B47(c) 	((c&0x0F0)>>4)	// Bit 4-7
#define	CH9_B8(c) 	((c&0x100)>>8)	// Bit 9

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

void handle_bus(volatile Bus_t *pBus);

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

#define NUM_BUS_T 3000 // 7000

int queue_overflow = 0;

volatile int data_in = 0;
volatile int data_out = 0;

volatile Bus_t bus[NUM_BUS_T];

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

int peripheral_ce = 0;
char dtext[2*NR_CHARS+1];
bool bPunct[2*NR_CHARS+1];

char cpu2buf[1024];
int nCpu2 = 0;

void core1_main_3(void)
{
	static int bIsaEn = 0;
	static int bIsa = 0;
	static uint64_t isa = 0LL;
	static uint16_t inst = 0;
	int last_data_in;
	Module_t *mp = NULL;
	volatile Bus_t *pBus = &bus[data_in];

	irq_set_mask_enabled(0xffffffff, false);

	last_sync = GPIO_PIN(P_SYNC);

	initRom();

	while (1)
	{
		// Wait for CLK2 to have a falling edge
		WAIT_FALLING(P_CLK2);

		// Bit numbers are out by one as bit_no hasn't been incremented yet.

		// Do we drive the ISA line?
		if( drive_data_flag && bit_no > 41 ) {
			// Drive the ISA line for theses data bit
			switch( bit_no ) {
			case 42:
				gpio_put(LED_PIN_B, LED_ON);
				gpio_put(P_ISA_DRV, drive_data & 1);
				break;
			case 53:
				// Don't drive data any more
				gpio_put(P_ISA_OE, 1);
				bIsaEn = 0;
				// ... so no more data after this ...
				drive_data_flag = 0;
				break;
			default:
				if( !bIsaEn ) {
					gpio_put(P_ISA_OE, 0); // Enable ISA driver
					bIsaEn = 1;
				}
				// Expose the next bit on the ISA line ...
				gpio_put(P_ISA_DRV, drive_data & 1);
				drive_data >>= 1;
				driven_isa++;
			}
		}

		// Wait for CLK1 to have a falling edge
		WAIT_FALLING(P_CLK1);

		// Another bit, check SYNC to find bit number
		sync = GPIO_PIN(P_SYNC);

		// Increment the bit number (or sync ...)
		if ( sync && !last_sync ) {
			pBus->sync = 1;
			sync_count++;
			bit_no = 44;
		} else {
			bit_no = (bit_no + 1) % 56;
		}

		// Save all bits in data56 reg ...
		if( GPIO_PIN(P_DATA) )
			data56 |= 1LL << bit_no;
		if( GPIO_PIN(P_ISA) )
			isa |= 1LL << bit_no;

		switch( bit_no ) {
		case 0:
			pBus = &bus[data_in];
			pBus->sync = 0;
			break;

		case 7:
			pBus->pa = data56 & 0xFF;
			break;

		case 29:
			// Got address. If the address is that of the embedded ROM then we flag that we have
			// to put an instruction on the bus later
			pBus->addr = address = (isa >> 14) & 0xFFFF;
			mp = roms[address >> 12];
			break;

		case 30:
			// Check if we should emulate any modules ...
			if( mp && (address >= mp->start) && (address <= mp->end) ) {
				embed_seen++;
				inst = drive_data = mp->rom[address - mp->start];
				drive_data_flag = 1;
			} else {
				if( address == 016066 ) {
					inst = drive_data = 001;
					drive_data_flag = 1;
				} else
					drive_data_flag = 0;
			}
			break;

		case 55:
			// If bitno = 55 then we have another frame, store the transaction
			// A 56 bit frame has completed, we send some information to core0 so it can update
			// the display and anything else it needs to
			pBus->cmd = inst ? inst : (isa >> 44) & 0x3FF;
			pBus->data = data56;

			isa = data56 = 0LL;
			inst = 0;

			last_data_in = data_in;

			data_in = (data_in + 1) % NUM_BUS_T;

			if (data_out == data_in)
			{
				// No space left in ring-buffer ...
				queue_overflow = 1;
				data_in = last_data_in;
				gpio_put(LED_PIN_R, LED_ON);
			}
			gpio_put(LED_PIN_B, LED_OFF);
			break;
		}
		last_sync = sync;
	}
}

void process_bus(void)
{
	// Process data coming in from the bus via core 1
	while (data_in != data_out)
	{
#if 0
	  printf("\n%d: Addr:%04X %07o  Inst: %06o PeriAd:%02X Data56:%014llX",
		 data_out,
		 bus[data_out].addr,
		 bus[data_out].addr,
		 bus[data_out].cmd,
		 bus[data_out].pa,
		 bus[data_out].data);
#else
		// Handle the bus traffic
		handle_bus(&bus[data_out]);
#endif
		data_out = (data_out + 1) % NUM_BUS_T;
	}
}

////////////////////////////////////////////////////////////////////////////////
//
// This function is passed all of the traffic on the bus. It can do
// various things with that.
//

int pending_data_inst = 0;
bool bLdi = false;
bool bFetch = false;

uint64_t dreg_a = 0, dreg_b = 0;
uint64_t dreg_c = 0;
int display_on = 0;

void dump_dregs(void)
{
	int i;

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

		int b = ((NR_CHARS-1) - i) << 2;

		cc  = (dreg_a >> b) & 0x0F;
		cc |= ((dreg_b >> b) & 0x0F) << 4;
		u = (dreg_c >> b) & 1;

		cl = (cc & 0xc0) >> 6;
		cc &= 0x3f;

		if (u) {
			// Upper ROM character
			cc |= 0x80;
		} else {
			// Convert to ASCII character
			if( cc < 0x20 )
				cc |= 0x40;
		}

		bPunct[j] = false;
		dtext[j++] = cc;

		if (cl)	{
			// Add punctuation
			bPunct[j] = true;
			dtext[j++] = " .:,"[cl];
		}
	}

	dtext[j++] = '\0';

#if CF_DISPLAY_LCD
	UpdateLCD(dtext, bPunct, display_on);
	printf("\n[%s] (%s)", dtext, display_on ? "ON":"OFF");
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

void handle_bus(volatile Bus_t *pBus)
{
	int addr = pBus->addr;
	int inst = pBus->cmd;
	int pa = pBus->pa;
	int sync = pBus->sync;
	uint64_t data56 = pBus->data;

	if (pending_data_inst == INST_PRPH_SLCT) {
		switch (pa) {
		case DISP_ADDR:
			peripheral_ce = PH_DISPLAY;
			break;
		case WAND_ADDR:
			peripheral_ce = PH_WAND;
			break;
		case TIMR_ADDR:
			peripheral_ce = PH_TIMER;
			break;
		case CRDR_ADDR:
			peripheral_ce = PH_CRDR;
			break;
		default:
			peripheral_ce = PH_NONE;
		}
	}

	//int sync = bus_sync[idx];
	//printf("\n- DCE:%d ADDR:%04X (%02o %04o) INST=%04X (%04o) PA=%02X (%03o) sync=%d DATA=%016llX", display_ce, addr, addr>>10, addr&0x3FF, inst, inst, pa, pa, sync, data56);
	if( cpu2buf[0]) {
		printf("\n[%s]", cpu2buf);
		cpu2buf[0] = 0;
	}
	printf("\n");
	switch (peripheral_ce) {
	case PH_DISPLAY:
		printf("DISP");
		break;
	case PH_WAND:
		printf("WAND");
		break;
	case PH_TIMER:
		printf("TIMR");
		break;
	case PH_CRDR:
		printf("CRDR");
		break;
	default:
		printf("    ");
	}
	//	printf("- DCE:%d ADDR:%04X (%02o %04o) INST=%04X (%04o) PA=%02X (%03o) sync=%d DATA=%016llX", display_ce, addr, addr>>10, addr&0x3FF, inst, inst, pa, pa, sync, data56);
	//	printf(" (%02o %04o) sync=%d DATA=%016llX", addr>>10, addr&0x3FF, sync, data56);
	printf(" %014llX | %04X (%X:%d %04o) %03X", data56, addr, addr>>12, (addr>>10)&0b11, addr&0x3FF, inst);

	if( bFetch ) { 
		printf("   > @%04X --> %03X (%04o)", addr, inst, inst);
		bFetch = false;
	} else {
		disAsm(inst, addr, data56);
	}
	// Check for a pending instruction from the previous cycle
	if (pending_data_inst)
	{
		switch (pending_data_inst)
		{
		case INST_LDI:
			bLdi = true;
			break;
		case INST_PRPH_SLCT:
#if CF_DBG_SLCT
			printf("\nPF AD:%02X", pa);
#endif
			break;

		case INST_WRITE_ANNUNCIATORS:
#if CF_DBG_DISP_INST
			printf("\n%s %03llX", "WRTEN", data56 & 0xFFF);
#endif
			UpdateAnnun((uint16_t)data56&0xFFF);
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
		case INST_WANDRD:
			printf("\n%s %03llX", "WAND BUF", data56 & 0xFFF);
		}
		// Clear pending flag ...
		pending_data_inst = 0;
	}

	// Check for instructions
	if (sync) {
		switch (inst) {
		case INST_DISPLAY_OFF:
#if CF_DBG_DISP_ON
			printf(" - Display off");
#endif
			display_on = 0;
			break;
		case INST_DISPLAY_TOGGLE:
			display_on = !display_on;
#if CF_DBG_DISP_ON
			printf(" (%s)", display_on ? "ON" : "OFF");
#endif
			dump_dregs();
			break;
		case INST_PRPH_SLCT:
#if CF_DBG_SLCT
			printf("\nPRPH_SLCT: PA=%02X", pa);
#endif
			pending_data_inst = inst;
			break;
		case INST_RAM_SLCT:
#if CF_DBG_SLCT
			printf("\nRAM_SLCT: PA=%02X", pa);
#endif
			pending_data_inst = inst;
			break;
		case INST_LDI:
			bLdi = true;
			pending_data_inst = inst;
			break;
		case INST_FETCH:
			bFetch = true;
			break;
		case INST_POWOFF:
			dump_dregs();
			break;
		}
	}
	// Check for display transactions
	if (peripheral_ce == PH_WAND && !bLdi)
	{
		switch (inst)
		{
		case INST_WANDRD:
			pending_data_inst = inst;
			break;
		}
	}
	// Check for display transactions
	if (peripheral_ce == PH_DISPLAY && !bLdi)
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
	bLdi = false;
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

void prtCl2Cmd(int inst)
{
	printf("%-8s %s", cmd2[(inst>>5)&0x1F], cmd2param[(inst>>2)&0x07]);
}

void cl0mod(int m, bool bA)
{
	if( bA ) {
		printf("%2d", m);
		if( m>9 )
			printf(" (%X)", m);
	} else {
		printf("%s", mcl0[m]);
	}
}

bool bLDICON = false;

void prtCl0Cmd(int inst, int data, int con)
{
	int mod = (inst>>6) & 0x0F;
	int sub = (inst>>2) & 0x0F;
	switch(sub) {
		case 0x0:
			printf(cmd00[mod]);
			break;
		case 0x1:
			if( mod == 0xF )
				 printf("ST=0");	// clears ST (CPU flags 0-7)
			else {
				printf("CF ");
				cl0mod(mod, false);
			}
			break;
		case 0x2:
			if( mod == 0xF )
				 printf("CLRKEY"); // clears the keydown flag (immediately set if key is down)
			else {
				printf("SF ");
				cl0mod(mod, false);
			}
			break;
		case 0x3:
			if( mod == 0xF )
				 printf("?KEY"); // set carry if keydown flag is set
			else {
				printf("?FS ");
				cl0mod(mod, false);
			}
			break;
		case 0x4:
			printf("LC ");
			cl0mod(mod, true);
			break;
		case 0x5:
			if( mod == 0xF )
				printf("PT=PT-1");	// decrement pointer (if PT=0 then PT=13)
			else {
				printf("?PT= ");
				cl0mod(mod, false);
			}
			break;
		case 0x6:
			printf(cmd06[mod]);
			break;
		case 0x7:
			if( mod == 0xF )
				printf("PT=PT+1");	// increment pointer (if PT=13 then PT=0)
			else {
				printf("PT= ");
				cl0mod(mod, false);
			}
			break;
		case 0x8:
			printf(cmd08[mod]);
			break;
		case 0x9:
			printf("PERTCT ");
			cl0mod(mod, true);
			break;
		case 0xA:
			printf("REG=C ");
			cl0mod(mod, true);
			break;
		case 0xB:
			printf(cmd0b[mod]);
			break;
		case 0xC:
			printf(cmd0c[mod]);
			switch( mod ) {
			case 0x4:
				bLDICON = true;
				break;
			case 0x9:
			case 0xF:
				// Don't know yet - data is pending ...
				// printf(" (PA=%02X)", con&0xFF);
				break;
			case 0xB:
				switch( peripheral_ce ) {
				case PH_DISPLAY:
					printf(" (WRTEN - writes annunciators)");
					break;
				case PH_CRDR:
					printf(" (write to cardreader)");
					break;
				}
				break;
			}
			break;
		case 0xD:
			printf("-unused-");
			break;
		case 0xE:
			if( mod == 0x0 )
				printf("READ DATA");	// copy active user memory register to C
			else {
				printf("C=REG ");
				cl0mod(mod, true);
			}
			break;
		case 0xF:
			if( mod == 0xF )
				printf("???");
			else {
				printf("RCR ");
				cl0mod(mod, false);
			}
			break;
	}
}

bool disAsmDisp(int inst)
{
	switch (inst) {
	case INST_WRITE_ANNUNCIATORS:
		printf("WRTEN");
		return true;
	case INST_SRLABC:
	case INST_SRLDA:
	case INST_SRLDB:
	case INST_SRLDC:
	case INST_SRSABC:
	case INST_SLSABC:
	case INST_SRLDAB:
	case INST_SLLDAB:
	case INST_SLLABC:
	case INST_SRSDA:
	case INST_SRSDB:
	case INST_SRSDC:
	case INST_SLSDA:
	case INST_SLSDB:
	case INST_SRSDAB:
	case INST_SLSDAB:
		printf("%s", inst50[inst>>6]);
		return true;

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
		printf("%s", inst70[inst>>6]);
		return true;
	}
	return false;
}

void disAsm(int inst, int addr, uint64_t data)
{
	static int prev;
	static bool bClass1 = false;
	static int con;
//	printf(" %04X [%02o %04o] %03X %014llX - ", addr, addr>>10, addr & 0x3FF, inst, data);
	printf(" - ");

	// Second word of LDI instruction ...
	if( bLDICON ) {
		con = inst & 0x3FF;
		printf("CON: %03X (%04o)", con, con);
		bLDICON = false;
		return;
	}

	// Second word of jump instruction ...
	if( bClass1 ) {
		prev |= (inst << 6) & 0xFF00;
		printf("%s %04X", cmd1[inst&0x03], prev);
		bClass1 = false;
		return;
	}

	// Check for peripheral instructions
	switch (peripheral_ce) {
	case PH_DISPLAY:
		if( disAsmDisp(inst) )
			return;
		break;
	case PH_WAND:
	case PH_TIMER:
	case PH_CRDR:
		break;
	}

	switch(inst & 0x03 ) {
		case 0b00:
			prtCl0Cmd(inst, data & 0x3FF, con);
			break;
		case 0b01:
			prev = (inst >> 2) & 0xFF;
			bClass1 = true;
			printf("-->");
			break;
		case 0b10:
			prtCl2Cmd(inst);
			break;
		case 0b11:
		{
			int ad, dir;
			if( inst & 0x200 ) {
				dir = '-';
				prev = 0x40 - ((inst&0x1F8) >> 3);
				ad = addr - prev;
			} else {
				dir = '+';
				prev = (inst&0x1F8) >> 3;
				ad = addr + prev;
			}
			printf("%-8s %c%02X [%04X]", inst&0x100?"JNC":"JC", dir, prev, ad);
		}
	}
}