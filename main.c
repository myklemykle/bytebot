// Byteseeker Jr.
// Adapted from Michael Smith's PCM Audio sketch on the Arduino Playground
//
// Adapted from https://moderndevice.com/news/build-a-hackable-bytebeat-player-at-the-ri-mini-maker-faire/ :
// Smith's code was for an atMEGA; this attiny24A port is for Alex Norman's balloonbot boards.

#include <stdint.h>
#include <stdbool.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>

// General config settings:
#define SAMPLE_RATE 8000  // hertz (each one is 1/8 ms)

#define LEDPIN PINA4
#define LED_R_PIN PINA4
#define LED_G_PIN PINA3
#define LED_B_PIN PINA2
#define BUTTON_PIN PINA7
#define BUZZER_PIN0 PINA6
#define BUZZER_PIN1 PINA5

// Use CTC mode for more accurate sample rate timer?
#define TIMER0_CTC 1

// Trim our timer to 8khz by ear ...
//#define COUNTER_TRIM 249 // works for CPU prescale /4 and timer prescale off.
#define COUNTER_TRIM 255 // works for CPU prescale /4 and timer prescale off.

// button macros:
// Button is pressed if bit BUTTON_PIN of register PINA is 1
#define BUTTON_HIGH (PINA & _BV(BUTTON_PIN))
#define BUTTON_PRESSED (! BUTTON_HIGH)

/////////////////////////
// Bytebeat stuff:
//
// Total number of bytebeat recipes; read by ISR, changed by loop()
#define SOUNDSTATES 14
#define INIT_SOUNDSTATE 6;
// The current sound state:
// (I'm making it a register 'cuz there's a sale on registers ... )
register uint8_t soundState asm ("r4");

#define BUTTON_DEBOUNCE_INTERVAL SAMPLE_RATE/100  // anything quicker than this is noise: 10 ms.
#define BUTTON_PRESS_INTERVAL SAMPLE_RATE/10 // minimum press-down time: 1/10th sec
#define BUTTON_HOLD_INTERVAL SAMPLE_RATE 		// minimum hold-down time: 1 sec

// For button mgt:
// volatile (memory) copy of t; written by ISR, used by button mgt code.
static uint16_t thisTime = 0; 

/**
 *  Our app states are:
 *  PWROFF: no electrons in the wires.  All is silent.  We can't actually detect this state. =)
 *  STARTING: just got power;  running setup, etc.
 *  PLAYING: startup finished, playing a scene.
 *  CHANGING: switching between scenes.
 *  SLEEPING: hibernating.
 *  ASLEEP: hibernating, waiting for a button press to wake us up.
 */

enum AppStates { PWROFF, STARTING, PLAYING, CHANGING, SLEEPING, ASLEEP };
static enum AppStates appState = PLAYING;


/// INTERRUPT SERVICE ROUTINES:
//
// The magic of Bytebeat is all in this interrupt handler:
// it's called at the sample rate, to generate the next byte of waveform as a function of t.
#ifdef TIMER0_CTC
ISR(TIM0_COMPA_vect) {
	// TODO: try to use timer1 for this.
#else
ISR(TIM0_OVF_vect) {
#endif
	/*
	 * TODO: this "fast" interrupt handler pushes/pops 17 registers, just because one of the 8 switchpoints
	 * below requires that many!  Jump table instead?  Or just try other -O flags?
	 */

	uint16_t value = 0;
	uint16_t t = thisTime;  // the register version, for inside the interrupt

	if (appState == SLEEPING) {
		thisTime = ++t;
		return;
	}

	switch (soundState) {
		case 0: 
			 value = ((t >> 10) & 42) * t;
			 // How it's supposed to sound: http://greggman.com/downloads/examples/html5bytebeat/html5bytebeat.html
			 break;
		case 1:
			//value = t*(t>>11&t>>8&123&t>>3);  
			value = t*(t>>11&t>>8 & 0b01100011 &t>>3);  // 0b01100011 is a little less hectic than 123
			break;
		case 2:
			//value = t*5&(t>>7)|t*3&(t*4>>10); /// very xmassy!  kind of sweet.
			value = t*5&(t>>7)|t*3&(t>>8); /// same thing ...
			 break;
		case 3:
			//value = (t>>7|t|t>>6)*10+ (4*(t*t>>13|t>>6) ); // disco techno?
			value = (t>>7|t|t>>6)*10+ ((t*t>>13|t>>6) <<2 ); // same thing ...
			 break;
		case 4:
			// value = ((t*("36364689"[t>>13&7]&15))/12&128) + (((((t>>12)^(t>>12)-2)%11*t)/4|t>>13)&127); // designed for 44khz
			//
			//value = ( ((t*("36364689"[t>>11&7]&15))/12&128)  + (( (((t>>9)^(t>>9)-2)%11*t) >>2|t>>13)&127) ) << 2; // 8khz version
			// needs optimization ...
			value = ( ((t*("36364689"[t>>11&7]&15)) &128)  + (( (((t>>9)^(t>>9)-2)%11*t) >>2|t>>13)&127) ) << 2; // 8khz version
			 break;
		case 5:
			 value = (t<<1 & 0x80); // 8khz.
			 break;
		case 6:
			 //value = t<<(t>>13 & 7) & 0x80;
			 value = (t<<1 ); // 8khz.
			 break;
		case 7:
			 value = (t<<2 ); // 8khz.
			 break;
		case 8:
			 value = (t<<3 ); // 8khz.
			 break;
		case 9:
			 value = (t<<4 ); // 8khz.
			 break;
		case 10:
			 value = (t<<5 ); // 8khz.
			 break;
		case 11:
			 value = (t<<6 ); // 8khz.
			 break;
		case 12:
			 value = (t<<7 ); // 8khz.
			 break;
		case 13:
			 value = 128; // 8khz.
			 break;
		/*case 8:*/
			 /*value = (t>>a|t|t>>(t>>16))*b+((t>>(b+1))&(a+1));   */
			 /*[>aTop = 12; aBottom = 0; bTop = 20; bBottom = 0;<]*/
			 /*break;*/
		/*case 9:*/
			 /*value = ((t*(t>>a|t>>(a+1))&b&t>>8))^(t&t>>13|t>>6);   */
			 /*[>aTop = 16; aBottom = 0; bTop = 86; bBottom = 0;<]*/
			 /*break;*/
		/*case 10:*/
			 /*value = ((t>>32)*7|(t>>a)*8|(t>>b)*7)&(t>>7);   */
			 /*[>aTop = 8; aBottom = 0; bTop = 22; bBottom = 0;<]*/
			 /*break; */
	}
	
	// send sample to PWM generator.
	OCR1AL = value & 0xff;
	thisTime = ++t;
}

	////////////////////////
	// TIMER1 (16-bit) setup:
	// Set up Timer 1 to do PWM audio on the speaker pin. 
	// The balloon board has a piezo btwn pins 7 & 8 (OC1A & OC1B)
	// instead of a pin & ground, so we need to set up two inverse PWM signals
	// on those two pins.
inline void setupTimer1(void){
	// Set fast PWM mode
	// WGM0[3:0] = 0101
	TCCR1A = (TCCR1A | _BV(WGM10)) &  ~_BV(WGM11);
	TCCR1B = (TCCR1B | _BV(WGM12)) & ~_BV(WGM13);

	// EXPERIMENT: Phase-correct PWM mode ... any audible diff? WGM* = 0001
	// TCCR1B &= ~(_BV(WGM13) | _BV(WGM12));  /// Yes, sounds crap.
	// ... that's weird.  I don't see why it would be so different, but it loses all low freqs.
	// This sounds like the bug I had before.

	// Do non-inverting PWM on pin OC1A 
	// COM1A* = 10 == clear OC1A on match, set on bottom.
	TCCR1A = (TCCR1A | _BV(COM1A1)) & ~_BV(COM1A0);

	// Do the complement on pin OC1B:
	// COM1B* = 11 == set OC1B on match, clear on bottom.
	TCCR1A |= _BV(COM1B1) | _BV(COM1B0);

	// CS1* = 001 == no prescaler.
	TCCR1B = (TCCR1B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10);

	// " The FOC1A/FOC1B bits are only active when the WGM1[3:0] bits specifies a non-PWM mode. 
	// However, for ensuring compatibility with future devices, these bits must be set to zero 
	// when TCCR1A is written when operating in a PWM mode."
	// ... okay whatever dude.
	TCCR1C = 0;

	OCR1AH = 0; // clear the top 8 bits of this register & never touch them again.
}


//////////////////
// Timer0 (8-bit) setup:
// Interrupt at 8khz 
//
// Our 8mhz system clock has a prescaler we should use first, to save batteries.
// If we set that at /4, main clock is : 2mhz.
// 
// In normal mode, we get an interrupt on overflow, aka every 256 ticks ... that yields something in the 7.5khz range.
// To speed that up to 8khz, we can use CTC mode & trigger an interrupt every ~ 250 ticks (adjust by ear)
//
inline void setupTimer0(void){
	// Set timer prescale: 
	// Prescale to 1/1024 of clock: 101
	//TCCR0B = ((TCCR0B | _BV(CS02)) & ~_BV(CS01)) | _BV(CS00);
	// Faster pussycat: /8 = 010
	// TCCR0B = ((TCCR0B & ~_BV(CS02)) | _BV(CS01)) & ~_BV(CS00);
	// speedy tweety: no prescale at all = 001
	//TCCR0B = (TCCR0B & ~(_BV(CS02) | _BV(CS01))) | _BV(CS00);

#ifdef TIMER0_CTC
	// Set compare register
	OCR0A = COUNTER_TRIM;
	// CTC mode, disconnect pins.
	TCCR0A = _BV(WGM01);
	// Clear interrupt flag (why?)
	// TIFR0 |= _BV(OCF0A);
	// Enable interrupt on counter == OCR0A:
	TIMSK0 = _BV(OCIE0A);

	// Set prescale (none) & start timer
	// Clearing cs02 & cs01, setting cs00
	TCCR0B = (TCCR0B & ~(_BV(CS02) | _BV(CS01 | _BV(WGM02)))) | _BV(CS00); 

#else
	// Normal mode, disconnect OC0A&B pins.  All zeros in this register.
	TCCR0A = 0;
	// Enable interrupt on counter overflow:
	TIMSK0 = _BV(TOIE0);
	// Set prescale & start timer
	TCCR0B = (TCCR0B & ~(_BV(CS02) | _BV(CS01))) | _BV(CS00); 
#endif
}

inline void setup(void) {
	// set CPU prescaling
	// 1: clock prescaler change enable!  (this bit on, all other bits to 0)
	CLKPR = _BV(CLKPCE);
	// 2: set clock prescaler to /4 (== 2mhz)
	CLKPR = _BV(CLKPS1); // /4
	//CLKPR = _BV(CLKPS0); // /2
	//CLKPR = 0; // /0

	setupTimer1();
	setupTimer0();
	
	// set pinMode to output (1) on these pins, and to input (0) on the rest of port A (including BUTTON_PIN):
	DDRA = _BV(LED_R_PIN) | _BV(LED_G_PIN) | _BV(LED_B_PIN)
		| _BV(BUZZER_PIN0) | _BV(BUZZER_PIN1)
		; 

	// Enable pull-up resistor on the button pin:
	PORTA = _BV(BUTTON_PIN);

	// Initialize soundState reg:
	soundState = INIT_SOUNDSTATE;

	// green means go:
	PORTA &= ~_BV(LED_G_PIN);
}

//
// Power Mgt: pause and resume
//
void pause(void) {
	cli();

	// disable timers
	TCCR1B &= ~_BV(CS10);
	TCCR0B &= ~_BV(CS10);

	// set both buzzer pins low.
	PORTA &= ~(_BV(BUZZER_PIN0) | _BV(BUZZER_PIN0));
	
	// power down some pins?
	
	// set up wake when button is pressed
	// (enable pin change interrupt on BUTTON_PIN (A7))
	GIMSK |= _BV(PCIE0);   // enable pin change interrupts generally,
	PCMSK0 |= _BV(PCINT7); // enable this pin in particular.
	
	// enter ('power down')
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	sleep_enable();		// enable sleep
	sei();

	sleep_cpu();			// sleep because we pressed the button.
}


void resume(void){

	cli();
	sleep_disable();
	
	// disable pin change interrupt
	PCMSK0 = 0;
	GIMSK &= ~_BV(PCIE0);

	// power up pins?
	// enable timers
	TCCR0B |= _BV(CS10);
	TCCR1B |= _BV(CS10);

	sei();
}


//
// Main loop.  Not interrupt-driven, not timing-critical, tho it does use
// the timer-updated thisTime value.

/**
 * The button states are:
 *  button is not pressed;  carry on.
 *  button has been pressed for less than BUTTON_PRESS_INTERVAL; carry on (waiting for debounce)
 *  button has been pressed for more than BUTTON_PRESS_INTERVAL, but less than BUTTON_HOLD_INTERVAL.  
 *  button has been held down longer than BUTTON_HOLD_INTERVAL
 */
enum ButtonStates { OFF, DEBOUNCING, PRESSED, HELD };

void loop(void) {
	static uint16_t pressTime = 0;

	static enum ButtonStates btnState = OFF;

	// Hi.  I'm a finite state machine.

	if (BUTTON_PRESSED) {
		switch(btnState) {
			case OFF: 
				// changed state!
				btnState = DEBOUNCING;
				pressTime = thisTime;
				break;
			case DEBOUNCING: 
				if (thisTime - pressTime > BUTTON_PRESS_INTERVAL) {
					// changed state!
					btnState = PRESSED;
					// Do a trick:
					switch(appState) {
						case PLAYING:
							appState = CHANGING;
							soundState = (soundState + 1) % SOUNDSTATES;
							break;
						case SLEEPING:
							// state change!
							appState = PLAYING;
							// do a trick!
							resume();
							break;
					}
					appState = PLAYING;
				} 
				break;
			case PRESSED: 
				if (thisTime - pressTime > BUTTON_HOLD_INTERVAL) {
					// changed state!
					btnState = HELD;
					// Do a trick:
					if (appState != SLEEPING) {
						appState = SLEEPING;
						pause();
					}
				}
				break;
			default: 
				// Nothing has changed;
				break;
		}
	} else { // ! BUTTON_PRESSED
		if (btnState != OFF) {
			// changed state!
			btnState = OFF;
		}
	}
}

void main(void){
	cli();
	setup();
	sei();
	for(;;){
		loop();
	}
}


///
// TODO:
//
// The sleep/wake logic is tricky.  I'm using this loop that uses timer0 to time
// time the button stuff, but I don't get timer0 while asleep.  The pin-change
// interrupt is triggered when I press & again when I release, and it's a bit
// bouncy.  
// Look into the hardware debounce stuff!  Save some time there.
// Look into using the WDT to wake instead of the pin change interrupt,
// and then timing the button down stuff.
//
// MEANWHILE: how to get better sound?  That's going to really really really matter, yo.
// Turn a box into a sprung membrane ...
