/*
 * TSEA56_styrmodul.c
 *
 * Created: 2024-03-28 10:24:33
 * Author : aleli930
 */ 

#include <avr/io.h>


int main(void)
{
	// output
	DDRB = 0xff;
	
	// INIT PWM
	// use fast PWM
	TCCR0A = (1<<COM0A0)|(2<<COM0B0)|(3<<WGM00);  // Set when timer hits BOTTOM, clear on compare match, mode 3 fast PWM
    TCCR0B = (1<<WGM02)|(5<<CS00);  // other part of WGM0, clock select f / 1024
	
	
	OCR0A = 155; // OCR0A should be the PWM frequency
	OCR0B = (1.5 * 155) / 20; // OCR0B should be the pulse length 1.5 ms / 20 ms * TOP
	// OC0A is the port 
	
	TCCR3A = (1<<COM3A0)|(2<<COM3B0)|(3<<WGM30);  // Set when timer hits BOTTOM, clear on compare match, mode 3 fast PWM
	TCCR3B = (1<<WGM32)|(5<<CS30);  // other part of WGM0, clock select f / 1024
	
	
	OCR3A = 155; // OCR0A should be the PWM frequency
	OCR3B = (1.5 * 155) / 20; // OCR0B should be the pulse length 1.5 ms / 20 ms * TOP
	// TCCR2A = ()
    while (1) 
    {
    }
}

