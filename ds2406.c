/**
 * This file implements a DS2406 with two PIOs.
 *
 * It supports basic input and output on two pins,  but alarm is
 * not fully implemented.
 * This device has only been tested with ATmega8 running at 16MHz,
 * with modification it may be usable on other CPUs too.
 *
 * The device has only been tested with OWFS, which does not implement
 * the full Channel Access command.  For this reason, this has NOT been
 * implemented or tested to fully match the spec.
 *
 * The PB1 pin is PIO.A
 * The PB2 pin is PIO.B
 *
 * It can also be built with a more advanced PWM output mode,
 * in which case the two outputs are controlled with PWM.
 * In PWM mode, PB1 happens to be OC1A, and PB2 is OC1B, hence why
 * those pins was chosen.
 *
 * The PWM outputs supports individual operation mode, with either 
 * a simple static value, or a cycling value.
 * The mode of operation is controlled by writing to the memory, which on
 * a real device is an EPROM. In this device, it is volatile, but on write
 * one can opt to write it as startup configuration into the EEPROM.
 *
 * Please see memory map below for configuration details.
 *
 * NOTE: Writing PIO.x register 0 (in OWFS, that is writing 1) will *activate* 
 * the output, which on this devices means bring it HIGH.
 * On a regular device, activate means enabling the output transistor and bringing
 * the output LOW.
 * The main reason for this is because it makes more sense.. Especially in PWM mode
 * where the output will be *active* rather than have a defined level.
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include "onewire.h"
#include "features.h"

#ifndef __AVR_ATmega8__
#error "Your AVR is not supported (or at least not tested) with this device"
#endif

#define C_READ_MEMORY		0xF0
#define C_EXTENDED_READ_MEMORY	0xA5
#define C_WRITE_MEMORY		0x0F
#define C_WRITE_STATUS		0x55
#define C_READ_STATUS		0xAA 
#define C_CHANNEL_ACCESS	0xF5

static u_char alarm;

#define STATUS_PWR	0x80
#define STATUS_PIO_B 0x40
#define STATUS_PIO_A	0x20
#define STATUS_CSS4	0x10
#define STATUS_CS33	0x08
#define STATUS_CSS2	0x04
#define STATUS_CSS1	0x02
#define STATUS_CSS0	0x01

#define CH_CTRL_ALR	0x80
#define CH_CTRL_IM	0x40
#define CH_CTRL_TOG	0x20
#define CH_CTRL_IC	0x10
#define CH_CTRL_CHS1	0x08
#define CH_CTRL_CHS0	0x04
#define CH_CTRL_CRC1	0x02
#define CH_CTRL_CRC0	0x01

#define CH_INFO_PWR				0x80
#define CH_INFO_2CH				0x40
#define CH_INFO_PIO_B_LATCH	0x20
#define CH_INFO_PIO_A_LATCH	0x10
#define CH_INFO_PIO_B_SENSED	0x08
#define CH_INFO_PIO_A_SENSED	0x04
#define CH_INFO_PIO_B_FF		0x02
#define CH_INFO_PIO_A_FF		0x01

static u_char
/* SRAM status byte:
 *    PWR, PIO-B FF, PIO-A FF, CSS4,
 *   CSS3,     CSS2,     CSS1, CSS0
 */
	status,
/* Channel info byte:
 *   Power, Num channels, PIO Latch B, PIO Latch A,
 *   Sensed B, Sensed A, FF-B, FF-A
 *
 * Latch is not implemented.
 * Sensed is only updated on Channel Access
 */
	channel_info
	
#ifdef WITH_PWM
	,

/* Internal cycle-state, bit's controlling direction
 * 	7, 6, 5,           , 4
 * 	-, -, ChB direction, ChB active
 *
 * 	3, 2, 1,           , 0
 * 	-, -, ChA direction, ChA active
 *
 * Direction means if we are going up (1) or down (0).
 */
	cycle_state,
/* Internal cycle counters, for speed control */
	cycle_cnt[2]
#endif
	;


#ifdef WITH_PWM

/* Memory, active & scratchpad */
#define MEM_LENGTH 11
static u_char active[MEM_LENGTH],
				  scratchpad[MEM_LENGTH];

/* Different actions which we want update_idle to execute */
static u_char idle_actions;
#define IDLE_ACTION_COPY_SCRATCHPAD 0x01
#define IDLE_ACTION_UPDATE_CYCLE  0x02
#define IDLE_ACTION_WRITE_EEPROM  0x04
static u_char eeprom_wr_pos;

#ifdef USE_WATCHDOG
extern uint8_t mcusr_mirror;
#endif

/**
 * Memory is layed out in two identical sections,
 * one page with active configuration, and one scratchpad. The active
 * configuration is replaced with the scratchpad contents, as soon as
 * the upper CRC byte is written and the scratchpad CRC checks out.
 * The page addressing matches the DS2406 page spec.
 *
 * The configuration consists of a set of registers which controls the
 * channels, and one common register.
 *
 * Page0, base 0x0000: active configuration
 * Page1, base 0x0020: scratchpad for configuration
 * Page2 base  0x0040: mcusr_mirror value on byte 0
 *
 * Page layout (base + offset):
 * 	Addr	Description
 * 	0000	PIO-B PWM min/set duty-cycle
 * 	0001	PIO-B PWM max duty-cycle
 * 	0002	PIO-B operation mode
 * 	0003	PIO-B cycle-mode speed
 * 	0004	PIO-A PWM min/set duty-cycle
 * 	0005	PIO-A PWM max duty-cycle
 * 	0006	PIO-A operation mode
 * 	0007	PIO-A cycle-mode speed
 * 	0008	Transfer configuration
 * 	0009	CRC16L
 * 	000A	CRC16H
 *
 * Page 0 is read only.
 * Page 1 is writable random or seqentially. When CRC16H is written, a CRC16
 * checksum operation is performed on the full scratchpad.
 * If it is valid, the content of the scratchpad will replace the active
 * configuration.
 *
 * Page 2 only has byte 0 populated, with the value of the CPUs MCUCSR from boot.
 * Any write to page 2 addr 0 will clear the register, use this to detect restarts.
 *
 *
 * The operation mode byte (for each channel) has the following meaning:
 *
 * 	Bit 7-6: Mode of operation
 * 		00: steady mode: uses MIN/SET register only.
 * 		01: cycling mode: toggle between MIN and MAX, without dimming
 * 		10: cycling mode, dimmed: dim between MIN and MAX according to
 * 			 cycle-mode speed register.
 * 		11: -
 *
 * 	Bit 5-4: What to do with OCR1x registry at the moment the
 * 				configuration goes active (cycle modes only)
 * 		00: do not touch
 * 		01: init OCR1x registry with MIN
 * 		10: init OCR1x registry with MAX
 * 		11: init OCR1x registry with MIN + (MAX-MIN)/2 (middle value)
 *
 * 	Bit 3-0: not used
 *
 * The transfer configuration configuration byte controls behaviour
 * which will take effect at any time the configuration is transfered
 * from scratchpad to active (from scratchpad, or at startup from EEPROM).
 *
 * 	Bit 7:
 * 		Enable PIO.B automatically
 * 	Bit 8:
 * 		Enable PIO.A automatically
 *
 * 	Bit 0: 
 * 		Write configuration to EEPROM, for automatic config on startup
 * 		(ignored when read from EEPROM).
 *
 * In case of invalid CRC value, the scratchpad will be cleared.
 * Reading any other memory address will give 0x00.
 * Writing any other memory address will be ignored.
 */
#define CFG_MIN_DUTY	0x0000
#define CFG_MAX_DUTY	0x0001
#define CFG_OPMODE	0x0002
#define CFG_SPEED		0x0003

#define CFG_PIOB_OFFSET		0x0000
#define CFG_PIOA_OFFSET		0x0004

#define CFG_TRX		0x0008

#define CFG_PIOB		0x0000
#define CFG_PIOA		0x0001

static void inline init_pwm_timer(void) {
	// This expects 16Mhz crystal
	// We use 244HZ PWM using timer1, with output on OC1A and OC1B

	// WGM1 3:0 = 0101 = PWM, fast 8bit(TOP=0x00ff)
	// F_PWM = Clock / ( Prescaler * (TOP+1)) 
	// 256 prescaler => PWM 244Hz @ 16Mhz
	TCCR1A = (1 << WGM10); 
	// Note that TCCR1B is used to enable/disable via sleep
}
static void inline enable_pwm_timer(void) {
	TCCR1B =(1<< WGM12) | (1<<CS12);
}
static void inline disable_pwm_timer(void) {
	TCCR1B = 0;
}
#endif /* WITH_PWM */

#ifdef USE_WATCHDOG
static void inline init_watchdog_wakeup_timer(void) {
	// If we have watchdog, we must make sure to wake up within
	// 1s from sleep, or we'll be restarted.
	// Lazy-use timer1 for this; with a prescaler of 1024 at 16Mhz, it
	// will overflow ~every 160ms and wake us up
	// Timer control is done where PWM is controlled
	TIMSK2 |= (1<<TOIE2);
}
static void inline enable_watchdog_wakeup_timer(void) {
# if defined(__AVR_ATmega8__)
	TCCR2 = (1 << CS22) | (1 << CS21) | (1 < CS20);
# elif defined(__AVR_ATmega88A__)
	TCCR2B = (1 << CS22) | (1 << CS21) | (1 < CS20);
# endif
}
static void inline disable_watchdog_wakeup_timer(void) {
# if defined(__AVR_ATmega8__)
	TCCR2 = 0;
# elif defined(__AVR_ATmega88A__)
	TCCR2B = 0;
# endif
}
#endif


void update_pio(void) ;

/* Update with a new Status register */
void update_status(u_char new_status) {
	status = new_status;

	// Clear PIO flipflop state from channel_info
	// and replace with new data from status
	channel_info =
		(channel_info & ~0x03) | 
		((status & STATUS_PIO_B) >> 5) |
		((status & STATUS_PIO_A) >> 5);

	update_pio();
}

/* Update PIOs with the current information in channel_info */
void update_pio(void) {
	/* PIO bit value 1 means set output to 1, 
	 * i.e. turn OFF output transistor.
	 * 
	 * OWFS note: when writing 1 to owfs PIO.A, owfs inverts and writes a 0 bit, which
	 * for a DS2406 means turn transistor *ON*, making the output low.
	 * This devices is inverted, read header.
	 */
	if(channel_info & CH_INFO_PIO_B_FF) {
		// B Off
#ifdef WITH_PWM
		TCCR1A &= ~(1 << COM1B1);
#else
		PORTB&=~ 0x04;
#endif
		DDRB&= ~0x04;
		status |= STATUS_PIO_B;

		channel_info |= CH_INFO_PIO_B_SENSED;
	}else{
		// B ON
#ifdef WITH_PWM
		TCCR1A |= (1 << COM1B1);
#else
		PORTB|=0x04;
#endif
		DDRB|= 0x04;
		status &= ~STATUS_PIO_B;

		channel_info &= ~CH_INFO_PIO_B_SENSED;
	}

	if(channel_info & CH_INFO_PIO_A_FF) {
		// A off
#ifdef WITH_PWM
		TCCR1A &= ~(1 << COM1A1);
#else
		PORTB&=~ 0x02;
#endif
		DDRB&= ~0x02;
		status |= STATUS_PIO_A;

		channel_info |= CH_INFO_PIO_A_SENSED;
	}else{
		// A on
#ifdef WITH_PWM
		TCCR1A |= (1 << COM1A1);
#else
		PORTB|=0x02;
#endif
		DDRB|= 0x02;
		status &= ~STATUS_PIO_A;

		channel_info &= ~CH_INFO_PIO_A_SENSED;
	}

	// Shut down timer if not in use
	if(TCCR1A & ((1<<COM1B1) | (1<<COM1A1))) {
#ifdef WITH_PWM
		enable_pwm_timer();
#endif
#ifdef USE_WATCHDOG
		disable_watchdog_wakeup_timer();
#endif
	} else {
		// Disable timer, not in use
#ifdef WITH_PWM
		disable_pwm_timer();
#endif
#ifdef USE_WATCHDOG
		enable_watchdog_wakeup_timer();
#endif
	}
}


#ifdef WITH_PWM
/**
 * Update a PWM channel in cycle mode
 */
void cycle_ch(u_char ch, volatile uint16_t* ocr_register) {
	u_char ch_offset = (ch == CFG_PIOB) ? CFG_PIOB_OFFSET : CFG_PIOA_OFFSET ;
	// Enabled for this ch?
	if((cycle_state & (1 << ch_offset)) == 0)
		return;

	// Only trigger every SPEED cycle
	++cycle_cnt[ch];
	if(cycle_cnt[ch] < active[ch_offset + CFG_SPEED])
		return;
	cycle_cnt[ch] = 0;

	if((cycle_state & (1 << (ch_offset + 1))) != 0) {
		// Going UP
		if((active[ch_offset + CFG_OPMODE] & 0xC0) == 0x40)
			// No-dim mode
			(*ocr_register) = active[ch_offset + CFG_MAX_DUTY];
		else
			(*ocr_register)++;

		if(*ocr_register >= active[ch_offset + CFG_MAX_DUTY])
			cycle_state&= ~(1 << (ch_offset + 1));
	}
	else
	{
		// Going DOWN
		if((active[ch_offset + CFG_OPMODE] & 0xC0) == 0x40)
			// No-dim mode
			(*ocr_register) = active[ch_offset + CFG_MIN_DUTY];
		else
			(*ocr_register)--;

		if(*ocr_register <= active[ch_offset + CFG_MIN_DUTY]) 
			cycle_state|= (1 << (ch_offset + 1));
	}
}

/**
 * Update PWM mode configuration for a channel
 */
void update_config_ch(u_char ch_offset, volatile uint16_t*ocr_register) {
	u_char mode = active[ch_offset + CFG_OPMODE] & 0xC0;
	if(mode == 0x40 || mode == 0x80) {
		// Cycling mode, jump between MIN and MAX, dimmed or directly.
		// Mark enabled in cycle_state
		cycle_state|= (1 << ch_offset);
	}else{
		cycle_state&= ~(1 << ch_offset);
		if(mode == 0x00) {
			// Steady mode, set MIN/SET point
			*ocr_register = active[ch_offset + CFG_MIN_DUTY];
		}
		// TODO: steady-dim mode
	}

	// OCR init-value
	mode = active[ch_offset + CFG_OPMODE] & 0x30;
	if(mode == 0x10)
		*ocr_register = active[ch_offset + CFG_MIN_DUTY];
	else if(mode == 0x20)
		*ocr_register = active[ch_offset + CFG_MAX_DUTY];
	else if(mode == 0x30)
		*ocr_register = active[ch_offset + CFG_MIN_DUTY] + 
			(active[ch_offset + CFG_MAX_DUTY] - active[ch_offset + CFG_MIN_DUTY]) / 2;

	// Set initial cycle-state direction bit
	if(*ocr_register >= active[ch_offset + CFG_MAX_DUTY])
		cycle_state&= ~(1 << (ch_offset + 1));
	else
		cycle_state|= (1 << (ch_offset + 1));
}

/* Update PWM configuration */
void update_config(void) {
	update_config_ch(CFG_PIOB_OFFSET, &OCR1B);
	update_config_ch(CFG_PIOA_OFFSET, &OCR1A);

	// Enable PIO's automatically
	if(active[CFG_TRX] & 0x80)
		channel_info&= ~CH_INFO_PIO_B_FF;

	if(active[CFG_TRX] & 0x40)
		channel_info&= ~CH_INFO_PIO_A_FF;

	if(active[CFG_TRX] & 0xC0)
		update_pio();

	// Enable/disable overflow interrupt on timer1, if 
	// cycle-mode is active on either channel
	if((cycle_state & 0x11)) {
		TIMSK1 |= (1<<TOIE1);
	} else {
		TIMSK1 &= ~(1<<TOIE1);
	}
}
#endif /* WITH_PWM */

void do_read_memory(int status_memory)
{
	u_short crc = 0;
	u_short adr;
	u_char bcrc = 1;
	u_char b;
	
	recv_byte();
	crc = crc16(crc, status_memory ? C_READ_STATUS : C_READ_MEMORY);
	b = recv_byte_in();			// address low byte
	recv_byte();
	crc = crc16(crc, b);
	adr = b;
	b = recv_byte_in();			// address high byte
	adr |= b<<8;

	/* note: bcrc is only true for the first byte to transmit */
#define XMIT(val) do { \
	xmit_byte(val); \
	if(bcrc) { crc = crc16(crc, b); bcrc = 0; } \
	crc = crc16(crc,val); \
} while(0)

	if(status_memory){
		switch(adr) {
		case 0x00: XMIT(0x00); 
		case 0x01: XMIT(0x00); 
		case 0x02: XMIT(0x00); 
		case 0x03: XMIT(0x00); 
		case 0x04: XMIT(0x00); 
		case 0x05: XMIT(0x00); 
		case 0x06: XMIT(0x00);
		case 0x07: XMIT(status);
			/* End of memory, send CRC */
			crc = ~crc;
			xmit_byte(crc);
			xmit_byte(crc >> 8);
		default:
			while(1)
				XMIT(0xFF);
		}
	}else{
#ifdef WITH_PWM
		while(adr <= 0x000A){ XMIT(active[adr]); adr++; }
		while(adr > 0x000A && adr < 0x0020) {XMIT(0x00); ++adr;}
		while(adr >= 0x0020 && adr <= 0x002A) {XMIT(scratchpad[adr - 0x20]); adr++;}
#endif
#ifdef USE_WATCHDOG
		while(adr == 0x0040) {XMIT(mcusr_mirror); ++adr;}
#endif
		while(adr <= 0x007F){XMIT(0x00); ++adr;}

		crc = ~crc;
		xmit_byte(crc);
		xmit_byte(crc >> 8);
		while(1)
			XMIT(0xFF);
	}
}

void do_write_memory(int status_memory)
{
	u_short crc = 0;
	u_short max_adr;
	u_short adr;
	u_char b;
	
	recv_byte();
	crc = crc16(crc, status_memory ? C_WRITE_STATUS : C_WRITE_MEMORY);
	b = recv_byte_in();			// address low byte
	recv_byte();
	crc = crc16(crc, b);
	adr = b;
	b = recv_byte_in();			// address high byte
	adr |= b<<8;

	recv_byte();
	crc = crc16(crc, b);

	if(status_memory)
		max_adr = 0x07;
	else
		max_adr = 0x007F;

	while(adr <= max_adr) {
		b = recv_byte_in();

		if(status_memory){
			switch(adr) {
			case 0x00: 
			case 0x01: 
			case 0x02: 
			case 0x03: 
			case 0x04: 
			case 0x05: 
			case 0x06: 
				break;
			case 0x07: 
				// New status byte;
				// always powered
				update_status(STATUS_PWR | b);
			}
		}else{
#ifdef WITH_PWM
			// Only writing to scratchpad allowed, ignore rest
			if(adr >= 0x0020 && adr <= 0x002A) scratchpad[adr - 0x20] = b;

			if(adr == 0x002A) {
				// CRC16H written, transfer to active
				idle_actions |= IDLE_ACTION_COPY_SCRATCHPAD;
			}
#endif
#ifdef USE_WATCHDOG
			if(adr == 0x0040) {mcusr_mirror = 0;}
#endif
		}

		// First pass, return crc of Cmd, Address, data.
		crc = crc16(crc, b);
		crc = ~crc;
		xmit_byte(crc & 0xFF);
		xmit_byte(crc >> 8);

		// XXX: in WRITE EPROM mode, the master will issue a strong-pullup
		// here... Tested with DS2480B and LinkUSB with no strong-pullup support!

		// Subsequent passes, return crc of address, data
		while(!rx_ready());
		// XXX : For some reason, we send an invalid CRC for some 
		// statuses, if we increment addr before send is complete!
		++adr;
		crc = crc16(0, (adr >> 8));
		crc = crc16(crc, 0x0); // upper bits always 0
		recv_byte();
	}
}

void do_channel_access(void)
{
	/* Receive 2 channel control bytes, then reply with channel info byte */
	u_short crc = 0;
	u_char b, ctrl1, wr_mode, ch_sel, ch,
			 byte_cnt, bit_cnt, crc_mode;

	recv_byte();
	crc = crc16(crc,C_CHANNEL_ACCESS);
	b = recv_byte_in();
	ctrl1 = b;

	recv_byte();
	crc = crc16(crc, b);
	b = recv_byte_in(); // ctrl2, ignored

	// Update channel info with new Sensed info
	channel_info = (channel_info & ~0x0C) |
						((PINB & ((1<<PINB2)|(1<<PINB1))) << 1);
	xmit_byte(channel_info);
	crc = crc16(crc, b);
	crc = crc16(crc, channel_info);

	// Until we get OW reset, we must now handle channel data
	// as requested in ctrl1/ctrl2
	wr_mode = (ctrl1 & CH_CTRL_IM) == 0; // Initial Mode bit, 1 = read , 0 = write
	ch_sel = (ctrl1 >> 2) & 0x03; // CHS1, CHS0: 1=A, 2=B, 3=Both

	// Active channel; if ch_sel= interleave, start on A
	ch = ch_sel == 3 ? 1 : ch_sel;

	// CRC Mode; 0 = no CRC, 1=every byte, 2 = every 8b, 3 = 32b
	crc_mode = (ctrl1 & 0x03);

	byte_cnt = 0;
	while(1) {
		bit_cnt = 8;
		if(wr_mode == 0) {
			/* Read mode. This is not real channel access read mode, but OWFS
			 * doesnt read this anyway, and I do not use it.
			 *
			 * Proper read-mode should sample at 1W low-edge.
			 * XXX: And not toggle like this... datasheet is somewhat tricky to read..
			 */
			b = 0;
			do {
				if(ch == 1) // Sensed A
					b|= (PINB >> PINB1) & 0x01;
				else if(ch == 2)  // Sensed B
					b|= (PINB >> PINB2) & 0x01;

				//xmit_bit(b & 0x01);
				b = b<<1;
			}while(--bit_cnt);
			xmit_byte(b);
			++byte_cnt;
			crc = crc16(crc, b);
		}else{
			// Write mode
			// XXX: THIS HAS NOT BEEN TESTED
			b = 0;
			do {
				recv_bit();
				b |= recv_bit_in();
				
				if(ch == 1) // FlipFlop A
					channel_info = (channel_info & ~0x01) |
						(b & 0x01);
				else if(ch == 2)  // FlipFlop B
					channel_info = (channel_info & ~0x02) |
						(b & 0x01);

				update_pio();

				b = b<<1;
			}while(--bit_cnt);
			++byte_cnt;
			crc = crc16(crc, b);
		}

		// CRC xmit time?
		if(crc_mode == 1 ||
			(crc_mode==2 && byte_cnt == 8) ||
			(crc_mode==3 && byte_cnt == 32)) {
			crc = ~crc;
			xmit_byte(crc);
			xmit_byte(crc >> 8);

			/*
			 * XXX: Should we clear?
			crc = 0;
			byte_cnt = 0;
			*/
		}

		if(ctrl1 & CH_CTRL_TOG) {
			// Toogle bit set
			// XXX: Wrong place to toggle..
			wr_mode = !wr_mode;
		}
	}
}

void do_command(u_char cmd)
{
	if(cmd == C_READ_MEMORY) {
		do_read_memory(0);
	} else if(cmd == C_READ_STATUS) {
		do_read_memory(1);
	}else if(cmd == C_WRITE_MEMORY) {
		do_write_memory(0);
	}else if(cmd == C_WRITE_STATUS) {
		do_write_memory(1);
	}else if(cmd == C_CHANNEL_ACCESS) {
		do_channel_access();
	}
	set_idle();
}

uint8_t is_alarm(void) {
	// XXX: Alarm not implemented
	return alarm;
}

#ifdef WITH_PWM
/* Check CRC of scratchpad, return 0 if OK */
u_char check_scratchpad(void) {
	int i;
	u_short crc = 0;
	for(i=0; i < MEM_LENGTH; ++i) {
		crc = crc16(crc, scratchpad[i]);
	}

	if(crc == 0xB001) {
		return 0;
	}
	return 1;
}

/* Copy scratchpad to active, and update configuration */
inline void copy_scratchpad(void) {
	int i;
	for(i=0; i < MEM_LENGTH; ++i) {
		active[i] = scratchpad[i];
	}
	update_config();
}
#endif

void update_idle(u_char bits)
{
#ifdef WITH_PWM
	int i;
	if(idle_actions & IDLE_ACTION_COPY_SCRATCHPAD) {
		if(!check_scratchpad()) {
			copy_scratchpad();
			if(active[CFG_TRX] & 0x01) {
				// Copy to EEPROM for startup
				// Do this in subsequent update_idle, so we wont block main routine for too
				// long. If a new full config is transfered before eeprom write is done,
				// we may or may not write bad data.
				idle_actions|= IDLE_ACTION_WRITE_EEPROM;
				eeprom_wr_pos = 0;
			}
		}else{
			// Bad, clear scratchpad to indicate so
			for(i=0; i < MEM_LENGTH; ++i) {
				scratchpad[i] = 0x00;
			}
			// write crc results for debug in scratcphad
			//scratchpad[0x09] = crc & 0x00ff;
			//scratchpad[0x0A] = crc >> 8;
		}

		idle_actions&= ~IDLE_ACTION_COPY_SCRATCHPAD;
	}

	if(idle_actions & IDLE_ACTION_WRITE_EEPROM) {
		if(!(EECR & (1<<EEWE))) {
			EEARL = eeprom_wr_pos + 8;
			EEDR = active[eeprom_wr_pos];
			EECR |= (1<<EEMWE);	// Prepare eeprom write 
			EECR |= (1<<EEWE);	// Start eeprom write

			++eeprom_wr_pos;
			if(eeprom_wr_pos == MEM_LENGTH) {
				// we're done
				idle_actions&= ~IDLE_ACTION_WRITE_EEPROM;
			}
		}
	}

	if(idle_actions & IDLE_ACTION_UPDATE_CYCLE) {
		cycle_ch(CFG_PIOB, &OCR1B);
		cycle_ch(CFG_PIOA, &OCR1A);

		idle_actions&= ~IDLE_ACTION_UPDATE_CYCLE;
	}
#endif

#ifdef USE_WATCHDOG
	TCNT2 = 0; // wait for next overflow
#endif

	// Go to idle sleep, saves 50% power (avg 10mA instead of 20)
	sleep_cpu();
}

#ifdef WITH_PWM
ISR (TIMER1_OVF_vect)
{
	idle_actions|= IDLE_ACTION_UPDATE_CYCLE;
}
#endif

#ifdef USE_WATCHDOG
// Do nothing; we just wake up from the sleep_cpu() instruction and do a loop
// and execute the wdr before going to sleep again.
EMPTY_INTERRUPT(TIMER2_OVF_vect);
#endif

void init_state(void)
{
#ifdef WITH_PWM
	int i;
#endif
	set_sleep_mode (SLEEP_MODE_IDLE);
	sleep_enable();

	alarm = 0;

#ifdef WITH_PWM
	init_pwm_timer();

	// Load startup configuration from EEPROM. 1W address is at 0-7,
	// we use 8-18
	while(EECR & (1<<EEWE));
	for (i=0; i < MEM_LENGTH; i++) {
		EEARL = 8 + i;				// set EPROM Address
		EECR |= (1<<EERE);	// Start eeprom read by writing EERE
		scratchpad[i] = EEDR;// Return data from data register
	}

	if(check_scratchpad()) {
		// Invalid EEPROM data, probably not set.
		// Default to Steady-mode with 100% duty-cycle
		scratchpad[0] = 0xff;
		scratchpad[1] = 0xff;
		scratchpad[2] = 0x00;
		scratchpad[3] = 0x00;

		scratchpad[4] = 0xff;
		scratchpad[5] = 0xff;
		scratchpad[6] = 0x00;
		scratchpad[7] = 0x00;

		// PIO's OFF by default
		scratchpad[8] = 0x00;

		// Valid CRC
		scratchpad[9] = 0xD1;
		scratchpad[10] = 0x0F;
	}

	cycle_state = 0;
	cycle_cnt[0] = 0;
	cycle_cnt[1] = 0;

	// Set register defaults
	channel_info = CH_INFO_PWR | CH_INFO_2CH;
#endif
	
#ifdef USE_WATCHDOG
	init_watchdog_wakeup_timer();
# ifndef WITH_WPM
	enable_watchdog_wakeup_timer();
# endif
#endif

	// Status default is all 1s (PIO's 1 == off)
	// will call update_pio
	update_status(0xFF);

#ifdef WITH_PWM
	copy_scratchpad();
	idle_actions = 0;
#endif

	// Power usage, enable pull-ups on all unused pins.
	// In idle-sleep mode, this totals on 11.5mA instead of 14.0mA
	PORTB= (1<<PB0) | (1 << PB5)| (1 << PB4)| (1 << PB3);
	PORTC=0xFF;
	PORTD=0xFF & ~0x04;
}

