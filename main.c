
// Byteseeker Jr.
// Adapted from Michael Smith's PCM Audio sketch on the Arduino Playground
//
// (From https://moderndevice.com/news/build-a-hackable-bytebeat-player-at-the-ri-mini-maker-faire/
// Ported to Alex's balloonbot boards by mykle hansen.)

#include <stdint.h>
#include <stdbool.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>

#define SAMPLE_RATE 8000  // hertz (each one is 1/8 ms)
#define BUTTON_PRESS_INTERVAL 400  // minimum hold-down time: 50 ms, expressed in samples
#define BUTTON_DEBOUNCE_INTERVAL 80  // anything quicker than this is noise: 10 ms, expressed in samples

#define LEDPIN PINA4
#define LED_R_PIN PINA4
#define LED_G_PIN PINA3
#define LED_B_PIN PINA2
#define BTN1PIN PINA7
#define BUZZER_PIN0 PINA6
#define BUZZER_PIN1 PINA5

// Button is pressed if bit BTN1PIN of register PINA is 1
#define BTN1_HIGH(reg) (reg & _BV(BTN1PIN))
#define BTN1_PRESSED(reg) (! BTN1_HIGH(reg))


int t=0;
unsigned int lastTime = 0;
unsigned int thisTime = 0;
unsigned int bounceTime = 0;
volatile int a, b, c, d;
volatile int value;
int state = 1;
#define states 9
bool btnState = false;
bool debouncing = false;

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

// This is called at 8000 Hz to load the next sample.
ISR(TIM0_OVF_vect) {

   switch (state) {
      case 1: 
         value = ((t&((t>>a)))+(t|((t>>b))))&(t>>(a+1))|(t>>a)&(t*(t>>b));  
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
    
    OCR1A = 0xff & value;  // how is '0xff & value' different from 'value'?
    ++t;

		// prove we're interrupting: turn on green LED
		PORTA |= _BV(LED_G_PIN);

}

inline void startPlayback(void)
{
    // Set up Timer 1 to do pulse width modulation on the speaker pin, 
		// and Timer 0 to trigger our sample-generation code at (approximately) 8khz.

		// The balloon board has a piezo btwn pins 7 & 8 (OC1A & OC1B)
		// instead of a pin & ground.  We need to set up two inverse PWM signals
		// on those two pins.

		/////////////
		// TIMER1 (16-bit) setup:
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
    // interrupts disabled to be safe ... I'm told?
		cli();
    OCR1A = 0;
		// 
		// done with TIMER1
		//////////////////

		//////////////////
		// Timer0 (8-bit) setup:
    // Interrupt at 8khz (1/1000 our clock)

		// Set normal mode & prescale to 1/1024 of clock.  This presumes our clock is 8mhz,
		// so the interrupt is getting us somewhere close to 1khz.  A little slower.
		// (I think one could get fancy & use 8-bit CTC mode plus a smaller prescale to tack
		// closer to exacty 8khz, but it doesn't matter for this application.)
		//
		// Normal mode: disconnect OC0A&B pins.  All zeros in this register.
    TCCR0A = 0;

		// TODO: I find this bit-flipping syntax really hard to read & write ... isn't there a better pattern?
    // Prescale to 1/1024 of clock:
    TCCR0B = ((TCCR0B | _BV(CS02)) & ~_BV(CS01)) | _BV(CS00);

		// Enable interrupt on counter overflow:
    TIMSK0 |= _BV(TOIE0);

		// Disable the other two interrupt modes on this timer:
    TIMSK0 &= ~( _BV(OCIE1A) | _BV(OCIE1B) ); 
}

/*
void blinkNTimes(int n) {
  for (int i=0; i<n; i++) {
    digitalWrite(LEDPIN, LOW);
    delay(100);
    digitalWrite(LEDPIN, HIGH);
    delay(100);
  }
}
*/

void setup(void) {

		// set pinMode to output (1) on these pins, and to input (0) on the rest of port A (including BTN1PIN):
		DDRA = _BV(LED_R_PIN) | _BV(LED_G_PIN) | _BV(LED_B_PIN)
			| _BV(BUZZER_PIN0) | _BV(BUZZER_PIN1)
			// | _BV(10) // Potentiometer
			// | _BV(LED_R_PIN) | _BV(LED_G_PIN) | _BV(LED_B_PIN)
			// | _BV(IGNITE_PIN) //igniter
			; 

		// Enable pull-up resistor on the button pin:
		PORTA = _BV(BTN1PIN);

    startPlayback();
		
		// turn on blue led:
		PORTA |= _BV(LED_B_PIN);

    
    lastTime = t;
    thisTime = t;
}

void loop(void) {
	thisTime = t;
	if ((thisTime - lastTime) > BUTTON_PRESS_INTERVAL) { 
		//updateScreen();
		lastTime = thisTime;
		//a = map(analogRead(0), 0, 1023, aBottom, aTop); 
		//b = map(analogRead(1), 0, 1023, bBottom, bTop);   

		if (!btnState) { // button wasn't pressed before.
			// check button:
			if (BTN1_PRESSED(PINA)) { 
				// button is pressed! but is it debounced?
				if (! debouncing ){  // not debouncing yet ...
					debouncing = true; // start debouncing!
					bounceTime = thisTime;
				} else {  // already deboucing
					if ((thisTime - bounceTime) > BUTTON_DEBOUNCE_INTERVAL) {
						if (BTN1_PRESSED(PINA)) { 
							// button's pressed now!
							btnState = true;
							// debouncing used to be cool.
							debouncing = false;
							// skip forward one state:
							state = (state + 1) % states;
							// blink somewhat:
							// blinkNTimes(state+1);
						}
					}
				} // debounce
			} // button is pressed

		} else { // btnstate == true
			if (! BTN1_PRESSED(PINA) ) { // if button is no longer pressed,
				btnState = false;     // clear button state.
			}
		}
	}
	// turn on red led:
	PORTA |= _BV(LED_R_PIN);

}

int main(void){
	setup();
	sei();
	for(;;){
		loop();
	}
}

