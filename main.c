// Byteseeker Jr.
// Adapted from Michael Smith's PCM Audio sketch on the Arduino Playground
//
// Evolved from & inspired by https://moderndevice.com/news/build-a-hackable-bytebeat-player-at-the-ri-mini-maker-faire/ :
// Smith's code was for an atMEGA with knobs; this attiny24A port is for Alex Norman's balloon-bot boards.

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

// LED macros:
#define RED_ON PORTA |= _BV(LED_R_PIN)
#define GREEN_ON PORTA |= _BV(LED_G_PIN)
#define BLUE_ON PORTA |= _BV(LED_B_PIN)
#define RED_OFF	PORTA &= ~_BV(LED_R_PIN)
#define GREEN_OFF	PORTA &= ~_BV(LED_G_PIN)
#define BLUE_OFF	PORTA &= ~_BV(LED_B_PIN)

/////////////////////////
// Bytebeat stuff:
//
// Total number of bytebeat recipes; read by ISR, changed by checkButton()
#define SOUNDSTATES 13
#define INIT_SOUNDSTATE 0;
// The current sound state:
// (I'm making it a register 'cuz there's a sale on registers ... )
register uint8_t soundState asm ("r4");

// volatile (memory) copy of t; incremented by ISR, used by button mgt code.
static uint16_t thisTime = 0; 

/**
 *  Our app states are:
 *  PWROFF: no electrons in the wires.  All is silent.  We can't actually detect this state.
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
// This ISR is called at timer overflow, to increment the 't' variable (time) used by bytebeat algorithms.
//
ISR(TIM1_OVF_vect) { 
	// At 8mhz clock speed we call this interrupt 4 times as often as we need to do anything,
	// so we skip doing any work 3 out of 4 times.  We "manually prescale" so that our PWM generating
	// freq is 32khz, beyond human hearing -- eliminating an unpleasant hi-frequency aliasing effect --
	// while our sample output rate remains 8khz (1/4 * 32khz), the bytebeat standard.
	//
	// For manual prescaling, we need a 2-bit counter:
	static uint8_t timeBits = 0;
	// NOTE: If we get really desperate for memory, this could maybe be done with timer0 instead.

	if (timeBits == 3) {
		timeBits = 0;
		thisTime++;
	} 
	timeBits++;

}

// The magic of Bytebeat: producing an audio sample as a (simple) function of time.
void genSample(){

	uint16_t value = 0;
	uint16_t t = thisTime;  // local register/nonvolatile version, for faster math (we hope)

	// We detect a clock tick. Do a new sample!
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
			value = t*5&(t>>7)|t*3&(t>>8); /// t*4>>10 and t>>8 should be the same thing ...
			 break;
		case 3:
			//value = (t>>7|t|t>>6)*10+ (4*(t*t>>13|t>>6) ); // disco techno?
			value = (t>>7|t|t>>6)*10+ ((t*t>>13|t>>6) <<2 ); // x << 2 and x * 4 should be the same thing ...
			 break;
		case 4:
			// value = ((t*("36364689"[t>>13&7]&15))/12&128) + (((((t>>12)^(t>>12)-2)%11*t)/4|t>>13)&127); // designed for 44khz
			value = ( ((t*("36364689"[t>>11&7]&15))/12&128)  + (( (((t>>9)^(t>>9)-2)%11*t) >>2|t>>13)&127) ) << 2; // 8khz version
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
			 /* Can't do this one without an exponent operator:
		case 13:
			 value = ( (t*(1.059 ^ (1 + (t>>12 & 11)) )<<( 1 + (t>>14 & 3))) * ( (t >> 10 & t>>13 | t >> 9) & 1) & 128) *1.99;
			 break;
			 */
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
	
	// adjust pulse width of PWM generator to generate the new sample:
	OCR1AL = value & 0xff;
}


////////////////////////
// TIMER1 (16-bit) setup:
// Set up Timer 1 to do PWM audio on the speaker pin. 
// The balloon board has a piezo btwn pins 7 & 8 (OC1A & OC1B)
// instead of a pin & ground, so we need to set up two inverse PWM signals
// on those two pins.
// Also trigger an intererupt on overflow.
inline void setupTimer1(void){
	// Set fast PWM mode
	// WGM0[3:0] = 0101
	TCCR1A = (TCCR1A | _BV(WGM10)) &  ~_BV(WGM11);
	TCCR1B = (TCCR1B | _BV(WGM12)) & ~_BV(WGM13);

	// 9-bit fast PWM mode: WGM0* = 0110
	//TCCR1A = (TCCR1A | _BV(WGM11)) &  ~_BV(WGM10);
	//TCCR1B = (TCCR1B | _BV(WGM12)) & ~_BV(WGM13);
	//
	// 10-bit fast PWM mode: WGM0* = 0111
	// TCCR1A |= (_BV(WGM11) | _BV(WGM10));
	// TCCR1B = (TCCR1B | _BV(WGM12)) & ~_BV(WGM13);

	// Do non-inverting PWM on pin OC1A:
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

	// Enable interrupt on TIMER1 overflow
	TIMSK1 |= _BV(TOIE1);
}

inline void setup(void) {
	// set CPU prescaling
	// 1: clock prescaler change enable!  (this bit on, all other bits to 0)
	CLKPR = _BV(CLKPCE);
	// 2: set clock prescaler
	//CLKPR = _BV(CLKPS1); // /4 (2mhz)
	// CLKPR = _BV(CLKPS0); // /2 (4mhz)
	CLKPR = 0; // /0 (8mhz)

	setupTimer1();
	
	// set pinMode to output (1) on these pins, and to input (0) on the rest of port A (including BUTTON_PIN):
	DDRA = _BV(LED_R_PIN) | _BV(LED_G_PIN) | _BV(LED_B_PIN)
		| _BV(BUZZER_PIN0) | _BV(BUZZER_PIN1)
		; 

	// Enable pull-up resistor on the button pin:
	PORTA = _BV(BUTTON_PIN);

	// Initialize soundState reg:
	soundState = INIT_SOUNDSTATE;

	// calibrate CPU clock. (trial and error and ear.)
	OSCCAL = 0x31;
}

///////////////////////////////
//
// Power Mgt: pause and resume
//
void pause(void) {
	cli();

	// disable timers
	TCCR1B &= ~_BV(CS10);
	// TCCR0B &= ~_BV(CS10);

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

	// zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz

	cli();
	sleep_disable();
	
	// disable pin change interrupt
	PCMSK0 = 0;
	GIMSK &= ~_BV(PCIE0);

	// power up pins?

	// enable timers
	// TCCR0B |= _BV(CS10);
	TCCR1B |= _BV(CS10);

	sei();
}


///////////////////////////////////////
// Button mgt.  
// Normal press & release advances the sound state
// Holding down for longer that BUTTON_HOLD_INTERVAL puts us to sleep,
// and the next press wakes us up again.
#define BUTTON_HOLD_INTERVAL SAMPLE_RATE 		// minimum hold-down time: 1 sec

// the button is pressed if bit BUTTON_PIN of register PINA is 1
#define BUTTON_PRESSED (PINA & _BV(BUTTON_PIN))

// states the button can be in:
enum ButtonStates { OFF, PRESSED, HELD};
static enum ButtonStates buttonState = OFF;

// How long has the button been in its current state?
static uint16_t buttonTime = 0;


///////////////////
// Debounce the button.
//
// This bit-shift debounce routine came from this enjoyable, nerdy article: 
// http://www.eng.utah.edu/~cs5780/debouncing.pdf -- A Guide To Debouncing, by Jack Ganssle
// This determines how many bits of the result we care about; we OR it to set all the other bits to 1.
#define DEBOUNCE_MASK 0b1110000000000000
// This is the value that represents a debounced "down"/"closed"/"pressed" state: twelve consecutive zeroes after a one
#define DEBOUNCE_PRESSED 0b1111000000000000
// This is the value that represents a debounced "up" state: twelve consecutive ones after a zero
#define DEBOUNCE_RELEASED 0b1110111111111111

// Called "periodically" ... To divide approx 50ms debounce time by 12 samples, 
// call every 4 ms; 'thisTime' ticks at 8khz (8 times per ms) so 
// every 4 ms == ever 32 ticks.
void sampleButton(void){
	static uint16_t State = 0;
	
	State = (State <<1) | ( BUTTON_PRESSED ? 1 : 0 ) | DEBOUNCE_MASK;

	if (State == DEBOUNCE_PRESSED) { 
		buttonState = PRESSED;
		buttonPressed();
		// green means go:
	} else if (State == DEBOUNCE_RELEASED) {
		buttonState = OFF;
		buttonReleased();
	}
}

// called once when button is pressed
void buttonPressed(void){
	buttonTime = thisTime;
	// if we were asleep, wake up!
	if (appState == SLEEPING) {
		appState = PLAYING;
	}
}

// called once when button is released
void buttonReleased(void){
	buttonTime = thisTime;
	// If we're sleeping, go back to sleep
	if (appState == SLEEPING) {
		BLUE_ON;
		pause();
	// Otherwise, advance the sound state.
	} else {
		soundState = (soundState + 1) % SOUNDSTATES;
	}
}

// called when t advances.
void twiddle(void){
	// if the button has been held down for BUTTON_HOLD_INTERVAL, go to sleep.
	if (buttonState == PRESSED) {
		
		if (thisTime - buttonTime > BUTTON_HOLD_INTERVAL) {
			GREEN_ON;
			appState = SLEEPING;
			pause();
		} else { 
			GREEN_OFF;
		}
	} else {
		RED_OFF;
		GREEN_OFF;
	}

}

void main(void){
	static uint16_t lastTime;
	uint16_t t;  // local register/nonvolatile version, for faster math (we hope)

	cli();
	setup();
	sei();
	for(;;){
		t = thisTime;

		// Check for clock ticks; if the clock has moved, generate a sample.
		if (lastTime != t) {
			genSample();
			lastTime = t;

			// Sample the button every 4 ms == every 32 ticks of T 
			if (t & 64 == 64) {
				sampleButton();
			}

		}

		// Time the button press:
		twiddle();

		// TODO: save power until the next tick?
	}
}

