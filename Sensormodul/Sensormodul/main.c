/*
 * Sensormodul.c
 *
 * Created: 2024-04-08 16:10:14
 * Author : aleli930
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>

volatile uint8_t left_counter = 0;
volatile uint8_t right_counter = 0;


int main(void)
{
    cli();
	DDRA = 0xff;  // output
	DDRB = 0xff;  // output
	DDRD = (0<<PORTD2)|(0<<PORTD3);  // Set INT1:0 as input
	
	EICRA = (3<<ISC10)|(3<<ISC00);  // Interrupt trigger set
	EIMSK = (1<<INT1)|(1<<INT0);  // Enable interupt
	
	sei();
	
    while (1) 
    {
    }
}

ISR(INT0_vect)  // LEFT
{
	left_counter++;
	PORTA = left_counter;
}

ISR(INT1_vect)  // RIGHT
{
	right_counter++;
	PORTB = right_counter;
}
