/*
 * TSEA56_styrmodul.c
 *
 * Created: 2024-03-28 10:24:33
 * Author : aleli930
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>


int main(void)
{
	int i;
	
	// TWI
	TWBR |= 
	TWCR |= (1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(1<<TWEN);  // Slave receiver mode
	
	
	// output
	DDRB = 0xff;
	
	// PWM
	// use fast PWM
	
	// Gasreglage
	TCCR0A = (1<<COM0A0)|(2<<COM0B0)|(3<<WGM00);  // Set when timer hits BOTTOM, clear on compare match, mode 3 fast PWM
    TCCR0B = (1<<WGM02)|(5<<CS00);  // other part of WGM0, clock select f / 1024
	
	
	OCR0A = 155; // OCR0A should be the PWM frequency
	OCR0B = (1.6 * 155) / 20; // OCR0B should be the pulse length 1.5 ms / 20 ms * TOP
	// OC0A is the port 
	
	// Styrservo
	TCCR2A = (1<<COM2A0)|(2<<COM2B0)|(3<<WGM20);  // Set when timer hits BOTTOM, clear on compare match, mode 3 fast PWM
	TCCR2B = (1<<WGM22)|(5<<CS20);  // other part of WGM0, clock select f / 1024
	
	OCR2A = 155; // OCR0A should be the PWM frequency
	OCR2B = (1.5 * 155) / 20; // OCR0B should be the pulse length 1.5 ms / 20 ms * TOP
	
	// Test timer
	TIMSK1 = (1<<OCIE1A);
    
	// Enable interrupt
	// EICRA |= (1<<ISC10)|(1<<ISC00);
	
	sei();
	
	while (1) 
    {
		i++;
    }
}

ISR(INT0_vect)
{
	OCR0B = (2 * 155) / 20;
	OCR2B = (2 * 155) / 20;
}

ISR(INT1_vect)
{
	OCR0B = (1.5 * 155) / 20;
	OCR2B = (1.5 * 155) / 20;
}
