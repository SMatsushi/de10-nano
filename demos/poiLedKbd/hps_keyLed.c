#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include "../../include/hwlib.h"
#include "../../include/socal/socal.h"
#include "../../include/socal/hps.h"
#include "../../include/socal/alt_gpio.h"

#define HW_REGS_BASE ( ALT_STM_OFST )
#define HW_REGS_SPAN ( 0x04000000 )
#define HW_REGS_MASK ( HW_REGS_SPAN - 1 )

#define USER_IO_DIR     (0x003fff00)	//  Out=GPIO[27:22]=sel[C,B,A, UD,LD], GPIO[15:8]=Seg;   IN=GPIO[3:0];
#define SEL_MASK		(0x003f0000)
#define SEL_UD			(0x00200000)	// GPIO0[21]
#define SEL_LD			(0x00100000)	// GPIO0[20]
#define SEL_138_EN		(0x00080000)	// GPIO0[19]
#define SEL_138_C		(0x00040000)	// GPIO0[18:16]: 3bit binary code to SN74HC138 
#define SEL_138_B		(0x00020000)
#define SEL_138_A		(0x00010000)
#define SEL_138_MASK	(0x00070000)
#define SEL_138_SFT		(16)			// left shift 16 bits to rw GPIO0 reg
#define SEG_MASK		(0x0000ff00)	// GPIO0[15-8]: segment drive bits
#define SEG_SFT			(8)				// left shift 8 bits to rw GPIO0 reg

#define RSTIN_MASK      (0x00000008)
#define KEYIN_MASK      (0x00000007)
#define CMDIN_MASK      (0x00000004)
#define UPN_MASK		(0x00000002)
#define LON_MASK		(0x00000001)

#define MAX_SEL		10		// scans select from 0 to MAX_SEL
#define DIG_WAIT_US	1000	// wait us between segment/key scan

void p_alt_setbits_word(void *va, uint32_t bits) {
	printf("Addr 0x%08x: setting  0x%08x\n", (uint32_t)va, bits);
	alt_setbits_word(va, bits);
}

void p_alt_clrbits_word(void *va, uint32_t bits) {
	printf("Addr 0x%08x: clearing 0x%08x\n", (uint32_t)va, bits);
	alt_clrbits_word(va, bits);
}

void *virtual_base;
uint32_t setbits, clrbits;	// set in set_sel and led_disp as side effects

void set_sel(int sel) {
	// set sel for segment/key scan selection
	if (sel >= 9) {
		clrbits |= SEL_UD;
	} else if (sel == 8) {
		clrbits |= SEL_LD;
	} else {
		clrbits |= SEL_138_EN;
		clrbits |= ((sel << SEL_138_SFT) ^ SEL_138_MASK) & SEL_138_MASK;		// mask & (1's compliments)
	}
	setbits |= SEL_MASK & (SEL_MASK ^ clrbits);
}

int led_seg[10];	// led_seg[10-6] = digit Upper[DP,3,2,1,0]	: segment with is lit
void led_disp(int sel) {
	// display LED segments at sel - 0 out to lit the segments: [D15,D14,..., D8] = [DP,G,...,(A/D1)]
	int segments;
	segments = led_seg[sel];
	clrbits |= (segments << SEG_SFT) & SEG_MASK;
	setbits |= ((segments << SEG_SFT) ^ SEG_MASK) & SEG_MASK;
	// p_alt_setbits_word( (virtual_base + ((uint32_t)( ALT_GPIO0_SWPORTA_DR_ADDR ) & (uint32_t)( HW_REGS_MASK ))), setbits);
	// p_alt_clrbits_word( (virtual_base + ((uint32_t)( ALT_GPIO0_SWPORTA_DR_ADDR ) & (uint32_t)( HW_REGS_MASK ))), clrbits);
}

unsigned char mem[65536];	// memory, zero initialized
unsigned short maddr;		// memory address register
unsigned short mdata;		// memory data register
unsigned char   mden;		// mden bit i turns on mdata digit i
unsigned char	 cmd;		// 0:Ret, ... 4:AdrSet, 5: RdIncr, 6:RdDecr, 7:WrIncr
int				reset = 0;	// 1 if reset button is pushed

unsigned char seg_tbl[20] = {0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7c, 0x07,
                             // 0,    1,    2,    3,    4,    5,    6,    7, 
                             0x7f, 0x67, 0x77, 0x7c, 0x58, 0x5e, 0x79, 0x71, 0x80};	// segments for 0, ... 9, DP
                             // 8,    9,    A,    b,    c,    d,    e,    f,   DP
#define LEDDIGS  4			// 4 digits in each LED module
#define LEDDRIVS 5			// 5 drives including DP in each LED module
#define UPPER 1
#define LOWER 0

void set_ledseg(int data, int ena, int upper) {
	// set four led segment refering 16 bit data and 4bit ena, if upper=1 set upper 4 digits 
	int i, d, ofs; 
	ofs = upper ? LEDDRIVS : 0;
	// set segments for lower 4 digits enabled by 1 in ena[3:0]
	for (i = 0; i < LEDDIGS; i++) {
		if (ena & 1) {
			d = data & 0x0f;
			led_seg[i + ofs] = seg_tbl[d];	// digit 0 to F
		} else {
			led_seg[i + ofs] = seg_tbl[0x10];	// DP
		}
		ena >>= 1;
		data >>= 4;		// next 4 bits (hex)
	}
}

void rd_mem(int maddr) {
	unsigned char md;
	md = mem[maddr];
	set_ledseg(maddr, 0xf, UPPER);	// display 4 digits for address
	set_ledseg(md, 3, LOWER);		// use lower 2 digits for data
}

void exec_cmd(int cmd) {
	// execute commands
	switch (cmd) {
		case 4:	// Adr Set
			maddr = mdata;
			rd_mem(maddr);
			break;
		case 5:	// Read Incr
		case 6: // Read Decr
			maddr += (cmd == 5) ? 1 : -1;
			rd_mem(maddr);
			break;
		case 7: // Write Incr
			mem[maddr] = mdata;
			maddr += 1;
			rd_mem(maddr);
			break;
		default:
			printf("Cmd=%d not implemented\n", cmd);
			break;
	}
}

int main(int argc, char **argv) {
	int fd;
	uint32_t  scan_input;
	int	sel;
	int key_on, key_prev;	// for chattering cancel
	int key_stb, key_stbp; // stabilized key input, and key push edge
	int key_val;	// register saving key scan result
	int i;

#ifdef NDEF
	int digit, segs;

	if (argc < 2) {
		fprintf(stderr, "Usage: %s <seg_num> <segbits (8bit hex)>\n", argv[0]);
		return (1);
	}

	digit = atoi(argv[1]);
	segs = strtol(argv[2], NULL, 16);
	if (digit > 9) {
		digit = 9;
	}
	printf("digit=%d, segments=0x%0x\n", digit, segs);
#endif //

	// map the address space for the LED registers into user space so we can interact with them.
	// we'll actually map in the entire CSR span of the HPS since we want to access various registers within that span
	if( ( fd = open( "/dev/mem", ( O_RDWR | O_SYNC ) ) ) == -1 ) {
		printf( "ERROR: could not open \"/dev/mem\"...\n" );
		return( 1 );
	}

	virtual_base = mmap( NULL, HW_REGS_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, HW_REGS_BASE );
	
	if( virtual_base == MAP_FAILED ) {
		printf( "ERROR: mmap() failed...\n" );
		close( fd );
		return( 1 );
	}
	
	// initialize the gpio controller
	printf("Setting GPIO diretion...\n");
	alt_setbits_word( (virtual_base + ((uint32_t)( ALT_GPIO0_SWPORTA_DDR_ADDR ) & (uint32_t)( HW_REGS_MASK ))), USER_IO_DIR );

	
	printf("Scanning key and LEDs\n");
	key_prev = key_on = 0;	
	while (1) {
		for(sel = key_on = 0; sel < MAX_SEL; sel++) {
			setbits = clrbits = 0;
			set_sel(sel);
			led_disp(sel);
			alt_setbits_word((virtual_base + ((uint32_t)( ALT_GPIO0_SWPORTA_DR_ADDR ) & (uint32_t)( HW_REGS_MASK ))), setbits);
			alt_clrbits_word((virtual_base + ((uint32_t)( ALT_GPIO0_SWPORTA_DR_ADDR ) & (uint32_t)( HW_REGS_MASK ))), clrbits);
			// p_alt_setbits_word((virtual_base + ((uint32_t)( ALT_GPIO0_SWPORTA_DR_ADDR ) & (uint32_t)( HW_REGS_MASK ))), setbits);
			// p_alt_clrbits_word((virtual_base + ((uint32_t)( ALT_GPIO0_SWPORTA_DR_ADDR ) & (uint32_t)( HW_REGS_MASK ))), clrbits);

			scan_input = alt_read_word((virtual_base + ((uint32_t)( ALT_GPIO0_EXT_PORTA_ADDR ) & (uint32_t)( HW_REGS_MASK ))));		
			i = scan_input & (RSTIN_MASK | KEYIN_MASK);
			if (i) {
				key_on = 1;
				key_val = (sel << 4) | i;	// [7:4, 3:0] = [sel, scan_input]
			}
			usleep(DIG_WAIT_US);	
		}
		// Check if key is pressed. 
		if (key_prev == key_on) {	// key is stabilized - chattering cancel if current key_on is the same as 16 ms ago.
			key_stb = key_on;
			reset = (key_val & RSTIN_MASK) ? 1 : 0; // keep this signal while reset is pressed
			if (key_stb != key_stbp) {	// key edge detected
				if (key_val & RSTIN_MASK) {
					printf("Reset pressed!\n");
				} else if (key_stb) { // Detect key pressing edge
					// Analyze key and execute command
					i = (key_val >> 4) & 7;		// set i = 7 bit select value
					if (key_val & CMDIN_MASK) {
						exec_cmd(i);
						// reset mdata, mden for next number imput
						mdata = mden = 0;
					} else {
						i += (key_val & UPN_MASK) ? 8 : 0;	// get tenkey value
						mdata = ((mdata << 4) | i) & 0xffff;
						mden = ((mden << 1) | 1) & 0x0f;
						set_ledseg(mdata, mden, LOWER);		// lower 4 digits
					}
				}
			}
			key_stbp = key_stb;
		}
		key_prev = key_on;
	}

	// clean up our memory mapping and exit
	if (munmap(virtual_base, HW_REGS_SPAN ) != 0) {
		printf("ERROR: munmap() failed...\n");
		close(fd);
		return(1);
	}	
	close(fd);
	return(0);
}