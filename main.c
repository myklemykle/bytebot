// Byteseeker Jr.
// Adapted from Michael Smith's PCM Audio sketch on the Arduino Playground
//
// Adapted from https://moderndevice.com/news/build-a-hackable-bytebeat-player-at-the-ri-mini-maker-faire/ :
// Smith's code was for an atMEGA; this attiny24A port is for Alex Norman's balloonbot boards.

#include <stdint.h>
#include <stdbool.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>

#define SAMPLE_RATE 8000  // hertz (each one is 1/8 ms)
#define BUTTON_PRESS_INTERVAL SAMPLE_RATE/10 // minimum hold-down time: 1/10th sec
#define BUTTON_DEBOUNCE_INTERVAL SAMPLE_RATE/100  // anything quicker than this is noise: 10 ms.

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
#define COUNTER_TRIM 120 

// button macros:
// Button is pressed if bit BUTTON_PIN of register PINA is 1
#define BUTTON_HIGH (PINA & _BV(BUTTON_PIN))
#define BUTTON_PRESSED (! BUTTON_HIGH)

// les variables
// for ByteBeat:
int t=0;
volatile int a, b, c, d;
volatile int value;
uint8_t state = 1;
#define states 9

// For button mgt:
int lastTime;
int thisTime;
int bounceTime;
bool btnState = false;
bool debouncing = false;

// The magic of Bytebeat is all in this interrupt handler:
// it's called at the sample rate, to generate the next byte of waveform as a function of t.
#ifdef TIMER0_CTC
ISR(TIM0_COMPA_vect) {
#else
ISR(TIM0_OVF_vect) {
#endif

   switch (state) {
      case 1: 
         //value = ((t&((t>>a)))+(t|((t>>b))))&(t>>(a+1))|(t>>a)&(t*(t>>b));  
				 // TODO: the a, b,c, d values need defining... these were pots or something on the original byteseeker.
				 // For now, here's a state that doesn't use them.
				 // How it's supposed to sound: http://greggman.com/downloads/examples/html5bytebeat/html5bytebeat.html
				 value = ((t >> 10) & 42) * t;

         /*aTop = 10;*/
         /*aBottom =0;*/
         /*bTop = 14;*/
         /*bBottom = 0;*/
         break;
      case 2: 
         value =(t*(t>>(a/10)|t>>(b/10)))>>(t>>((b/10)*2)); 
         /*aTop = 10;*/
         /*aBottom =0;*/
         /*bTop = 16;*/
         /*bBottom = 0;*/
         break;
      case 3:
        value = t*(((t>>(a*3))|(t>>(10+a)))&(b&(t>>(a*2))));   
         /*aTop = 6;*/
         /*aBottom =0;*/
         /*bTop = 50;*/
         /*bBottom = 0;*/
        break;
      case 4:
        value = t*(((t>>a)&(t>>8))&((b+73)&(t>>3)));  
         /*aTop = 22;*/
         /*aBottom =0;*/
         /*bTop = 99;*/
         /*bBottom = 0;*/
         break;
      case 5:
        value = t*(((t>>a)|(t>>(b*2)))&(63&(t>>b)));   
         /*aTop = 24; aBottom = 0; bTop = 8; bBottom = 0;*/
         break;
      case 6:
        value = ((t>>a&t)-(t>>a)+(t>>a&t))+(t*((t>>b)&b)); 
         /*aTop = 10; aBottom = 0; bTop = 28; bBottom = 0;*/
         break;
      case 7:
        value = ((t%42)*(t>>a)|(0x577338)-(t>>a))/(t>>b)^(t|(t>>a));  
         /*aTop = 8; aBottom = 0; bTop = 32; bBottom = 0;*/
         break;
      case 8:
         value = (t>>a|t|t>>(t>>16))*b+((t>>(b+1))&(a+1));   
         /*aTop = 12; aBottom = 0; bTop = 20; bBottom = 0;*/
         break;
      case 9:
         value = ((t*(t>>a|t>>(a+1))&b&t>>8))^(t&t>>13|t>>6);   
         /*aTop = 16; aBottom = 0; bTop = 86; bBottom = 0;*/
         break;
      case 10:
         value = ((t>>32)*7|(t>>a)*8|(t>>b)*7)&(t>>7);   
         /*aTop = 8; aBottom = 0; bTop = 22; bBottom = 0;*/
         break; 

    }
    
	  // send sample to PWM generator.
    OCR1A = 0xff & value;  // truncate at 8 bits?  dunno why else we do this.
    ++t;

		// DEBUG: prove we're interrupting: turn on green LED
		PORTA |= _BV(LED_G_PIN);
}

	////////////////////////
	// TIMER1 (16-bit) setup:
	// Set up Timer 1 to do PWM audio on the speaker pin. 
	// The balloon board has a piezo btwn pins 7 & 8 (OC1A & OC1B)
	// instead of a pin & ground, so we need to set up two inverse PWM signals
	// on those two pins.
inline void setupTimer1(void){
	// Set fast PWM mode
	// WGM0[2:0] = 011
	TCCR1A |= _BV(WGM11) | _BV(WGM10);
	TCCR1B &= ~_BV(WGM12);

	// Do non-inverting PWM on pin OC1A 
	// COM1A* = 10 == clear OC1A on match, set on bottom.
	TCCR1A = (TCCR1A | _BV(COM1A1)) & ~_BV(COM1A0);

	// Do the complement on pin OC1B:
	// COM1B* = 11 == set OC1B on match, clear on bottom.
	TCCR1A |= _BV(COM1B1) | _BV(COM1B0);

	// CS1* = 001 == no prescaler.
	TCCR1B = (TCCR1B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10);

	// Set initial pulse width to the first sample.
	// OCR1A is a 16-bit register, so we have to do this with
	// interrupts disabled to be safe ... or so I'm told?
	cli();
	OCR1A = 0;
}


//////////////////
// Timer0 (8-bit) setup:
// Interrupt at 8khz 
//
// Our main system clock has a prescaler we should use first, to save batteries.
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
	TCCR0B = (TCCR0B & ~(_BV(CS02) | _BV(CS01))) | _BV(CS00);

#ifdef TIMER0_CTC
	// CTC mode, disconnect pins.
	TCCR0A = _BV(WGM01);
	TCCR0B &= ~_BV(WGM02);
	// Set compare register
	OCR0A = COUNTER_TRIM;
	// Enable interrupt on counter == OCR0A:
	TIMSK0 = _BV(OCIE0A);

	// NOT WORKING WTF?
	// 
	// Voodoo from other examples I've seen:
	// initialize timer?
	//TCNT0 = 0;
	// Force a compare (jump-starts something?)
	//TCCR0B |= _BV(FOC0A);
	
#else
	// Normal mode, disconnect OC0A&B pins.  All zeros in this register.
	TCCR0A = 0;
	// Enable interrupt on counter overflow:
	TIMSK0 = _BV(TOIE0);

#endif
}

void setup(void) {

	// balloon_pwm did this, dunno why ...
	//wdt_disable(); 

	// turn off prescaling
	// 1: clock prescaler change enable!  (this bit on, all other bits to 0)
	CLKPR = _BV(CLKPCE);
	// 2: set clock prescaler to /4 (== 2mhz)
	CLKPR = _BV(CLKPS1);

	setupTimer1();
	setupTimer0();
	
	// set pinMode to output (1) on these pins, and to input (0) on the rest of port A (including BUTTON_PIN):
	DDRA = _BV(LED_R_PIN) | _BV(LED_G_PIN) | _BV(LED_B_PIN)
		| _BV(BUZZER_PIN0) | _BV(BUZZER_PIN1)
		// | _BV(10) // Potentiometer
		// | _BV(LED_R_PIN) | _BV(LED_G_PIN) | _BV(LED_B_PIN)
		// | _BV(IGNITE_PIN) //igniter
		; 

	// Enable pull-up resistor on the button pin:
	PORTA = _BV(BUTTON_PIN);

	lastTime = t;
	thisTime = t;

	PORTA |= _BV(LED_B_PIN); // DEBUG: turn on blue led to prove we're set up.
}

void loop(void) {
	thisTime = t;
	if ((thisTime - lastTime) > BUTTON_PRESS_INTERVAL) { 
		// turn on red led to prove we're checking:
		PORTA |= _BV(LED_R_PIN);

		lastTime = thisTime;

		if (!btnState) { // button wasn't pressed before.
			// check button:
			if (BUTTON_PRESSED) { 
				PORTA &= ~_BV(LED_G_PIN); // debug: turn off the green LED to prove we red the pin
				// button is pressed! but is it debounced?
				if (! debouncing ){  // not debouncing yet ...
					debouncing = true; // start debouncing!
					bounceTime = thisTime;
				} else {  // already deboucing
					if ((thisTime - bounceTime) > BUTTON_DEBOUNCE_INTERVAL) {
						if (BUTTON_PRESSED) { 
							// button's pressed now!
							btnState = true;
							// debouncing used to be cool.
							debouncing = false;
							// skip forward one state:
							state = (state + 1) % states;
							// blink somewhat:
							// blinkNTimes(state+1);
							//
							PORTA &= ~_BV(LED_B_PIN); // debug: turn off the blue LED to prove we triggered a thing.

						}
					}
				} // debounce
			} // button is pressed

		} else { // btnstate == true
			if (! BUTTON_PRESSED ) { // if button is no longer pressed,
				btnState = false;     // clear button state.
				// debug: turn green & blue back on
				PORTA |= _BV(LED_G_PIN) | _BV(LED_B_PIN);
			}
		}
	}
}

int main(void){
	setup();
	sei();
	for(;;){
		loop();
	}
}

// not yet used:
void stopPlayback(void)
{
	// Disable playback per-sample interrupt.
	TIMSK1 &= ~_BV(OCIE1A);

	// Disable the per-sample timer completely.
	TCCR1B &= ~_BV(CS10);
	// Q: why do we need to do both of those things?

	// Disable the PWM timer.
	TCCR0B &= ~_BV(CS10);

	// set both buzzer pins low.
	PORTA &= ~(_BV(BUZZER_PIN0) | _BV(BUZZER_PIN0));
}

