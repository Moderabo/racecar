/*
 * TSEA56_styrmodul.c
 *
 * Created: 2024-03-28 10:24:33
 * Author : aleli930
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>

const int ADDR = 25;

// Slave receiver
#
#define TW_SR_SLA_ACK		0x60
#define TW_SR_GEN_ACK		0x70
#define TW_SR_DATA_ACK		0x80
#define TW_SR_DATA_NOT_ACK	0x88

void i2c_init()
{
	TWAR = (ADDR<<TWA0)|(0<<TWGCE);
	TWCR = (1<<TWINT)|(1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(1<<TWEN)|(1<<TWIE);  // Slave receiver mode
}

void pwm_init()
{
	// use fast PWM
	
	// Gasreglage
	DDRD = 0xff; // output
	
	TCCR1A = (1<<COM1A0)|(2<<COM1B0)|(3<<WGM10);
	TCCR1B = (3<<WGM12)|(2<<CS10);
	
	OCR1A = 20597; // OCR0A should be the PWM 
	OCR1B = (1.5 * 2059.7) / 2; // OCR0B should be the pulse length 1.5 ms / 20 ms * TOP
	
	// Styrservo
	DDRB = 0xff; // output
	
	TCCR3A = (1<<COM3A0)|(2<<COM3B0)|(3<<WGM30);  // Set when timer hits BOTTOM, clear on compare match, mode 3 fast PWM
	TCCR3B = (3<<WGM32)|(2<<CS30);  // other part of WGM0, clock select f / 1024
	
	OCR3A = 20597; // OCR0A should be the PWM frequency
	OCR3B = (1.5 * 2059.7) / 2; // OCR0B should be the pulse length 1.5 ms / 20 ms * TOP
}

int main(void)
{
	int i;
	
	cli();
	
	DDRA = 0xff;
	
	// TWI
	i2c_init();	
	
	// PWM
	pwm_init();

	// Test timer
	//TIMSK1 = (1<<OCIE1A);
    
	// Enable interrupt
	//EICRA |= (1<<ISC10)|(1<<ISC00);
	
	sei();
	
	while (1) 
    {
		cli();
		i++;
		sei();
    }
}

ISR(TWI_vect)
{
	uint8_t TW_STATUS = TWSR & 0xf8;
	
	switch(TW_STATUS)
	{
		case TW_SR_SLA_ACK:
			TWCR = (1<<TWIE)|(1<<TWINT)|(1<<TWEA)|(1<<TWEN);
			break;
		case TW_SR_DATA_ACK:
			PORTA = (2059.7 + TWDR * 0.0314) / 2;
			OCR1B = (2059.7 + TWDR * 0.0314) / 2;
			TWCR = (1<<TWIE)|(1<<TWINT)|(1<<TWEA)|(1<<TWEN);
			break;
		default:
			TWCR = (1<<TWIE)|(1<<TWINT)|(1<<TWEA)|(1<<TWEN);
			break;
	}
}
