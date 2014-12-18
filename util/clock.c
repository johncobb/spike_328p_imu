/*
 * clock.c
 *
 *  Created on: Oct 20, 2014
 *      Author: jcobb
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include "defines.h"
#include "clock.h"

volatile clock_time_t clock_millis;
//volatile clock_time_t future;

//http://ucexperiment.wordpress.com/2012/03/16/examination-of-the-arduino-millis-function/

/*
Excerpt from link above:

Clock Cycles:
A 8MHz oscillator results in the microcontroller having a clock cycle once every 8-millionth of a second.
A clock cycle is roughly the time it takes for one instruction cycle (there are exceptions).
So on the micro, each clock cycle ticks off at every 1/8,000,000 second, which is:

0.000000125 seconds
0.000125 milliseconds (ms)
0.125 microseconds (µs)

Prescaler:
Technically a prescaler divides the timer, it can also be thought of as a multiplier.
For example, with an microcontroller's clock cycle occurring every 1/8,000,000 of a second, applying a 64 prescale
means Timer #0 is ticking at 64 times the base oscillator rate, or 64*1/8,000,000, which is every:

0.000008 seconds
0.008 ms
8 µs

Timer Interrupt/Overflow:

Timer #0 has an 8-bit counter register (TCNT0) which is incremented by 1 every 0.008 milliseconds.
The maximum 8-bit value the timer can count up to is 256 (hexadecimal 0xFF).
Therefore, when the timer reaches 256, it Òrolls overÓ to 0 or ÒoverflowsÓ. Thus overflow interrupt occurs each
time the timer's counter TCNT0 rolls over. Taking the above math into account,
this occurs every 1/8,000,000 (oscillator) * 64 (prescale) * 256 (roll over) = 0.002048 seconds or 2.048 microseconds.
It is important note the interrupt doesn't occur exactly each millisecond.

Fractions:
The Timer #0 interrupt handler is made slightly more complicated because of the
decimal part of the 2.048ms roll-over period. First, letÕs do some math:

clockCyclesPerMicrosecond = 8,000,000/1,000,000 = 8
MICROSECONDS_PER_TIMER0_OVERFLOW = ((64*256)*1000)/(8,000,000/1,000) = 2048
MILLIS_INC = 2048/1000 = 2.048 = 2 (integer)
FRAC_INC = (2048%1000 >> 3) = (48 >> 3) = 6
FRAC_MAX = (1000 >> 3) = 125

Example :
ISR(TIMER0_OVF_vect) {
  timer0_millis += 1;
  timer0_fract += 3;
  if (timer0_fract >= 125) {
    timer0_fract -= 125;
    timer0_millis += 1;
  }
  timer0_overflow_count++;
}


*/




volatile unsigned long timer0_overflow_count = 0;
volatile unsigned long timer0_millis = 0;
static unsigned char timer0_fract = 0;

void clock_init()
{
	// timer mode
	TCCR0A |= _BV(WGM01) | _BV(WGM00);

	// F_CPU/64/1000 = 125
	OCR0A = F_CPU/64/1000;
	// Enable timer set prescalar to 64
	TCCR0B |= _BV(CS01) | _BV(CS00);

	// Enable overflow interrupt
	TIMSK0 = _BV(TOIE0);
}

clock_time_t clock_time()
{
	clock_time_t m;
	uint8_t oldSREG = SREG;
	// disable interrupts while we read timer0_millis or we might get an
	// inconsistent value (e.g. in the middle of a write to timer0_millis)
	cli();

	m = timer0_millis;
	SREG = oldSREG;

	return m;

	//return clock_millis;
}

clock_time_t clock_time_micros()
{
	clock_time_t m;
	uint8_t oldSREG = SREG;
	uint8_t t;

	cli();

	m = timer0_overflow_count;
	t = TCNT0;

	if ((TIFR0 & _BV(TOV0)) && (t < 255))
			m++;

	SREG = oldSREG;

	return ((m << 8) + t) * (64 / clockCyclesPerMicrosecond());
}

void delay_millis(clock_time_t millis)
{
	clock_time_t future = clock_time() + millis;

	while(true)
	{
		if(clock_time() > future)
		{
			break;
		}
	}
}

ISR(TIMER0_OVF_vect)
{
	// copy these to local variables so they can be stored in registers
	// (volatile variables must be read from memory on every access)
	unsigned long m = timer0_millis;
	unsigned char f = timer0_fract;

	m += MILLIS_INC;
	f += FRACT_INC;
	if (f >= FRACT_MAX) {
		f -= FRACT_MAX;
		m += 1;
	}

	timer0_fract = f;
	timer0_millis = m;
	timer0_overflow_count++;
}

void isr_tick()
{

}



