////////////////////////
//
// Beatbot!  A pocket-sized byteboom beatbox!
// Rotates between different bytebeat melodies -- http://canonical.org/~kragen/bytebeat/
//
// Evolved from & inspired by https://moderndevice.com/news/build-a-hackable-bytebeat-player-at-the-ri-mini-maker-faire/ :
// Empowered by Alex Norman's generous donation of hardware.
// (Smith's code was for an atMEGA with knobs; this attiny24A port is for Alex Norman's balloon-bot boards.)
// Assisted by Jared Boone, Paul Stoffregen & others.
// Audio code adapted from Michael Smith's PCM Audio sketch on the Arduino Playground.
//
// 2015/2016 -mykle hansen-
//
// TODO:
// * further optimizations with registers (combine all flags in one reg?) might free up a bit more RAM for another song or two.
// * use a bytebeat interpreter, instead of compiling individual lines of bytebeat, to fit many more songs.
// * implement tuning, via some magic button-pressing handshake right at startup, so robots can sing in tune together.
//
// TOC:
//
// 1) Global config
// 2) Time management
// 3) Music generation
// 4) Button management
// 5) Setup & main loop


//////////////////////////////////
// 1) Global config
//
#include <stdint.h>
#include <stdbool.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

// Which pins do what?
#define LED_R_PIN PINA4
#define LED_G_PIN PINA3
#define LED_B_PIN PINA2
#define BUTTON_PIN PINA7
#define BUZZER_PIN0 PINA6
#define BUZZER_PIN1 PINA5

// RGBLED on/off macros:
#define RED_ON() PORTA |= _BV(LED_R_PIN)
#define GREEN_ON() PORTA |= _BV(LED_G_PIN)
#define BLUE_ON() PORTA |= _BV(LED_B_PIN)
#define RED_OFF()	PORTA &= ~_BV(LED_R_PIN)
#define GREEN_OFF()	PORTA &= ~_BV(LED_G_PIN)
#define BLUE_OFF()	PORTA &= ~_BV(LED_B_PIN)
#define RED_FLIP()	PORTA ^= _BV(LED_R_PIN)
#define GREEN_FLIP()	PORTA ^= _BV(LED_G_PIN)
#define BLUE_FLIP()	PORTA ^= _BV(LED_B_PIN)

// Here are some handy bit macros from http://www.avrfreaks.net/comment/26024#comment-26024 :
#define bit_get(p,m) ((p) & (m))
#define bit_set(p,m) ((p) |= (m))
#define bit_clear(p,m) ((p) &= ~(m))
#define bit_flip(p,m) ((p) ^= (m))
#define bit_write(c,p,m) (c ? bit_set(p,m) : bit_clear(p,m))

// States our application can be in:
// PLAYING: startup finished, playing a scene.
// SLEEPING: low-power hibernation, waiting to be woken by a button press.

#define PLAYING 0
#define SLEEPING 1
register uint8_t appState asm("r17");


///////////////////////////////
// 2) Time management:
//
// This is our master clock, 'time', incremented at 8khz by the ISR below.  We output one sample for every increment.
//
uint32_t masterTime = 0; 

// This ISR is called at timer1 overflow, to increment the 'time' variable used by bytebeat algorithms.
// At 8mhz clock speed we call this interrupt at 32khz -- 4 times as often as we need to increment the clock,
// so we 'manually prescale' by skipping work 3 out of every 4 times.  We do this so our PWM generating
// freq is beyond human hearing at 32khz, avoiding an unpleasant hi-frequency aliasing effect,
// while our sample output rate remains 8khz (1/4 * 32khz), the bytebeat standard.
//
// For manual prescaling, we need a 2-bit counter:
register uint8_t timeBits asm("r5");

ISR(TIM1_OVF_vect) { 
	// NOTE: If we get really desperate for memory, this could maybe be done with TIMER0 instead.

	if (timeBits == 3) {
		timeBits = 0;
		masterTime++;
	} 
	timeBits++;
}

////////////////////////////////
// Timer1 configuration at boot:
//
// Set up Timer 1 to do PWM audio on the speaker pin. 
// The balloon board has a piezo btwn pins 7 & 8 (OC1A & OC1B)
// instead of a pin & ground, so we set up two inverse PWM signals
// on those two pins.
// Also trigger an intererupt on overflow, to drive our clock.
//
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
	bit_set(TIMSK1, _BV(TOIE1));
}

/////////////////////////
// 3) Music Generation
//
// Bytebeat is a genre of simple algorthims that convert time (t) into an audio sample.
//
// How many bytebeat recipes in our tiny brain:
//
#define SOUNDSTATES 8
//
// The one we play at power-on:
//
#define INIT_SOUNDSTATE 5;
//
// The currently selected recipe:
// (I'm making it a register 'cuz there's a sale on registers ... )
//
register uint8_t soundState asm ("r4");

//
// genSample() is called at 8khz, to produce an audio sample as a (simple) function of masterTime
// using the currently selected recipe in soundState
//
register uint8_t value asm ("r6");
void genSample(){
	uint32_t t = masterTime;  // local register/nonvolatile version, for faster math (we hope)

	// memory savings: precompute some constants that appear in many different formulae:
	
	uint32_t tx3 = t + t + t;
	uint32_t trr6  = t>>6;  
	uint32_t trr7  = t>>7;  
	uint32_t trr8  = t>>8;  
	uint32_t trr11 = t>>11;  
	uint32_t trr13 = t>>13;  

	switch (soundState) {
		case 0: 
			//value = ((t >> 10) & 42) * t;
			value = ((t >> 10) & 42) * t;
			// How it's supposed to sound: http://greggman.com/downloads/examples/html5bytebeat/html5bytebeat.html
			break;
		case 1:
			//value = t*(trr11&trr8&123&t>>3);  
			value = t*(trr11&trr8 & 0b01100011 &t>>3);  // 0b01100011 is a little less hectic than 123
			break;
		case 2:
			//value = t*5&(trr7)|tx3&(t*4>>10); /// very xmassy!  kind of sweet.
			value = (t*5&(trr7)) | (tx3&(trr8)); /// t*4>>10 and trr8 should be the same thing ...
			break;
		case 3:
			//value = (trr7|t|trr6)*10+ (4*(t*t>>13|trr6) ); // disco techno?
			value = (trr7|t|trr6)*10 + ((t*trr13|trr6)<<2 ); // x << 2 and x * 4 should be the same thing ...
			break;
		case 4:
			// value = ((t*("36364689"[t>>13&7]&15))/12&128) + (((((t>>12)^(t>>12)-2)%11*t)/4|t>>13)&127); // designed for 44khz
			value = ( ((t*("36364689"[trr11&7]&15))/12&128)  + (( (((t>>9)^(t>>9)-2)%11*t) >>2|trr13)&127) ) << 2; // 8khz version
			break;
		case 5: 
			value = ((t<<2^trr6)*2 + (t<<3^trr7)) * ("11121114"[(t>>15)&7]-48);
			break;
		case 6:
			value = ((t<<1)^((t<<1)+(trr7)&t>>12))|t>>(4-(1&(t>>19)))|trr7;
			break;
		case 7:
			 value = t*6&((trr8|t<<4)) ^ t*4&((trr7|t<<3)) ^ t*2&((trr6|t<<2));
			 break;
			 /*
		case 8: /// too slow?
			value = tx3 & (t/12>>8|t%3)%24 | (trr8)&19  | (trr8)&19 >> (t&((t>>1)>>11)%63) >> (tx3&(trr7&t + t<<11));
			 break;
			 // Can't do this one without an exponent operator:
		case 13:
			 value = ( (t*(1.059 ^ (1 + (t>>12 & 11)) )<<( 1 + (t>>14 & 3))) * ( (t >> 10 & t>>13 | t >> 9) & 1) & 128) *1.99;
			 break;
			 */
	}
	
	// adjust the pulse width of the timer1 PWM generator to generate the new sample as PWM volts:
	OCR1AL = value;

	// twinkle:
	// this was my first random guess at something & looks prettier than all subsequent refinements i tried. =)
	if (t>>4 & value>>7 & 1) { RED_FLIP() ;}
	if (t>>5 & value>>6 & 1) { GREEN_FLIP() ;}
	if (trr6 & value>>5 & 1) { BLUE_FLIP() ;}
}


///////////////////////////////////////
// 4) Button mgt.  
// Normal press & release advances the sound state
// Holding down for longer that BUTTON_HOLD_INTERVAL puts us to sleep,
// and the next press wakes us up again.
//
#define BUTTON_HOLD_INTERVAL 8000 // minimum hold-down time: 1 sec

// the button is pressed if bit BUTTON_PIN of register PINA is 1
//
#define BUTTON_PRESSED (PINA & _BV(BUTTON_PIN))

// the states the button can be in:
//
enum ButtonStates { RELEASED, PRESSED };
static enum ButtonStates buttonState = RELEASED;

// How long has the button been in its current state?
//
static uint16_t buttonTime = 0;

//
// Debounce the button.
//
// This clever bit-shift debounce algorithm comes from this enjoyable, nerdy article: 
// http://www.eng.utah.edu/~cs5780/debouncing.pdf -- A Guide To Debouncing, by Jack Ganssle
// Basically we don't believe the button until it says the same thing N times in a row.  Here, N = 12.
//
// This masks how many bits of the result we care about (13); we OR it to set all the other bits to 1.
#define DEBOUNCE_MASK     0b1110000000000000

// This is the value that represents a debounced "down"/"closed"/"pressed" state: 12 consecutive zeroes after a one
#define DEBOUNCE_PRESSED  0b1111000000000000

// This is the value that represents a debounced "up" state: 12 consecutive ones after a zero
#define DEBOUNCE_RELEASED 0b1110111111111111

// sampleButton() is called periodically, adjusted to spread the 12 debouncing samples over the
// button's observed bounce time.  (You did hook your button to an oscilloscope and press
// it over and over for three hours, didn't you?  Well, Jack Ganssle did ... he says 50-100ms is good.)
//
void sampleButton(void){
	static uint16_t State = 0;
	
	State = (State <<1) | ( BUTTON_PRESSED ? 1 : 0 ) | DEBOUNCE_MASK;

	if (State == DEBOUNCE_PRESSED) { 

		buttonState = PRESSED;
		buttonTime = (masterTime & 0xffff);
		// if we were asleep, wake up!
		if (appState == SLEEPING) {
			appState = PLAYING;
		}

	} else if (buttonState == PRESSED && State == DEBOUNCE_RELEASED) {

		buttonState = RELEASED;
		buttonTime = (masterTime & 0xffff);
		soundState = (soundState + 1) % SOUNDSTATES;

	}
}

// Called when t advances.
void twiddle(void){
	// if the button has been held down for BUTTON_HOLD_INTERVAL, go to sleep.
	if (buttonState == PRESSED) {
		
		if ((masterTime & 0xffff) - buttonTime > BUTTON_HOLD_INTERVAL) {
			appState = SLEEPING;
			sleep();
		}
	}

}

///////////////////////////////
// 5) Power Mgt: 
//
// Watchdog timer macros:
//
#define WDT_ENABLE() WDTCSR |= _BV(WDIE)
#define WDT_DISABLE() WDTCSR &= ~_BV(WDIE)
#define WDT_RESET() __asm__ __volatile__ ("wdr")

// sleep() is called at sleep.  It turns off everything that we can turn off & puts us in a coma,
// to be wakened someday by a watchdog interrupt.
//
void sleep(void) {
	cli();

	// disable timer1
	TCCR1B &= ~_BV(CS10);

	// set both buzzer pins low.
	PORTA &= ~(_BV(BUZZER_PIN0) | _BV(BUZZER_PIN0));
	
	// power down LEDs.
	RED_OFF(); GREEN_OFF(); BLUE_OFF();
	
	// Enable watchdog timer interrupt mode
	WDT_ENABLE();
	
	// enter ('power down')
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	sleep_enable();		// enable sleep
	sei(); 						// Allows watchdog interrupt to happen. (?)

	// sleep with the lights on?	
	sleep_cpu();			// sleep because we pressed the button.

	// WAKEUP EXECUTION RESUMES HERE!
}

// wake() is called at wakeup, and un-does whatever sleep() did.
void wake(void){

	cli();
	sleep_disable();
	
	// disable watchdog
	WDT_DISABLE();

	// power up pins?

	// enable timer1
	TCCR1B |= _BV(CS10);

	sei();
}

// idle() is called when we're waiting for the next clock tick.  It
// just turns off the CPU until an interrupt happens.  Mild power savings.

void idle(void){
	set_sleep_mode(SLEEP_MODE_IDLE);
	sleep_enable();		
	sleep_cpu();
}

// We'll use the watchdog timer to wake us from sleep.
// Configure it at startup, but don't turn it on yet.
void setupWatchdog(void){
	uint8_t state = 0;
	// Watchdog should be disabled by default, but just to be sure: disable watchdog. (WDE == 0, WDIE == 0)
	// watchdog trigger rate: every 16ms (WDP* == 0000);
	bit_clear(state, _BV(WDE) | _BV(WDP3) | _BV(WDP2) | _BV(WDP1) | _BV(WDP0) | _BV(WDIE));
	
	// The Magic Watchdog Configuration Change Happy Dance:
	// All changes must be written with WDCE = 0
	state &= ~_BV(WDCE);
	cli();
	WDT_RESET();
	bit_clear(MCUSR, _BV(WDRF));
	bit_set(WDTCSR, _BV(WDCE) | _BV(WDE));
	// You have four cycles to complete your changes. Good luck!
	WDTCSR = state;
	sei();
}

//
// This ISR is polled by the watchdog when we're sleeping.  
// It wakes us back up once the button has been released and then re-pressed.
// It uses a simplified version of the button debounce function that
// we use when awake: it's called less often, watches fewer bits.
//
// This determines how many bits of the result we care about (13); we OR it to set all the other bits to 1.
#define WDT_DEBOUNCE_MASK     0b11100000
#define WDT_DEBOUNCE_PRESSED  0b11110000
#define WDT_DEBOUNCE_RELEASED 0b11101111

// Q: why aren't i getting these two #defines via io.h????  Some avr-gcc header bug?  or not including something?  dunno.
// At any rate, the datasheet says it's vector 4 for attiny24a, and that works!
#define WDT_vect_num  4
#define WDT_vect      _VECTOR(4)  /* Watchdog Time-out */

register uint8_t buttonPinState asm ("r5");

ISR(WDT_vect) {
	
	buttonPinState = (buttonPinState <<1) | ( BUTTON_PRESSED ? 1 : 0 ) | WDT_DEBOUNCE_MASK;

	if (buttonState == PRESSED && (buttonPinState == WDT_DEBOUNCE_RELEASED)) {
		buttonState = RELEASED;
	} else if (buttonState == RELEASED && (buttonPinState == WDT_DEBOUNCE_PRESSED)) { 
		// Wake from sleep!
		appState = PLAYING;
		// Do the rest:
		wake();
	}

}

////////////////////////
// 
// 6) Setup and main loop:
//
inline void setup(void) {

	// Conserve battery:
	// Turn off the ADC, we don't use it.
	ADCSRA &= ~_BV(ADEN);
	// Cut all power to ADC, timer0 & serial port
	PRR |= _BV(PRADC) | _BV(PRTIM0) | _BV(PRUSI); // 1 == disabled in this register
	// I believe the BOD is still turned off (off by default on new chips)
	// With the BOD, ADC and comparator are off, the internal referernce voltage should be off too.

	// Set CPU prescaling (two steps):
	// 1: clock prescaler change enable!  (this bit on, all other bits to 0)
	CLKPR = _BV(CLKPCE);
	// 2: actually change the clock prescale value
	CLKPR = 0; // /0 (8mhz)

	// configure main clock:
	setupTimer1();

	// configure watchdog for sleep/wake:
	setupWatchdog();

	// set pinMode to output (1) on these pins, and to input (0) on the rest of port A (including BUTTON_PIN):
	DDRA = _BV(LED_R_PIN) | _BV(LED_G_PIN) | _BV(LED_B_PIN)
		| _BV(BUZZER_PIN0) | _BV(BUZZER_PIN1)
		; 

	// Enable pull-up resistor on the button pin:
	PORTA = _BV(BUTTON_PIN);

	// Initialize registers: 
	// TODO: these could all be one register!  smaller code?  we'd have to try it.
	soundState = INIT_SOUNDSTATE;
	timeBits = 0;
	buttonPinState = 0;
	appState = 0;

	// calibrate CPU clock. (trial and error and ear.)
	OSCCAL = 0x31;
}


/////////////////////////////
// All together now:
// Configure hardware & then loop forever computing samples when needed.
// 
void main(void){
	static uint8_t lastTime;
	uint8_t t8;  

	// Configure hardware:
	cli();
	setup();
	sei();

	// Loop forever:
	for(;;){
		t8 = masterTime; // we only need to look at the low byte here.

		// Check for clock ticks; if the clock has moved, generate a sample.
		if (lastTime != t8) {
			genSample();
			lastTime = t8;

			// Sample the button every 4 ms == every 32 ticks of T 
			if ((t8 & 64) == 64) {
				sampleButton(); // we only need the low byte
			}

			// Time the button press events:
			twiddle();

		}
		// Save power until the next clock tick.
		idle();
	}
}

