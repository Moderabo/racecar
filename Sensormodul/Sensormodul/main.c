/*
 * Sensormodul.c
 *
 * Created: 2024-04-08 16:10:14
 * Author : aleli930
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>

//#define PORT_DEBUG

volatile uint16_t left_counter = 0;
volatile uint16_t right_counter = 0;
volatile uint16_t speed = 0;
volatile uint16_t speed_buffer = 0;

const int ADDR = 34;

// Slave transmitter
#define TW_ST_SLA_ACK		0xA8
#define TW_SR_GEN_ACK		0x70
#define TW_ST_DATA_ACK		0xB8
#define TW_SR_DATA_NOT_ACK	0x88
#define TW_ST_STOP			0xC8


void i2c_init()
{
	TWAR = (ADDR<<TWA0)|(0<<TWGCE);  // Set address and general call
	TWCR = (1<<TWINT)|(1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(1<<TWEN)|(1<<TWIE);  // Slave transmitter mode
}


void sensor_init()
{
	DDRD = (0<<PORTD2)|(0<<PORTD3);  // Set INT1:0 as input
	
	EICRA = (3<<ISC10)|(3<<ISC00);  // Interrupt trigger set
	EIMSK = (1<<INT1)|(1<<INT0);  // Enable interrupt
}


void timer_init()
{
	TCCR1A = (1<<COM1A0)|(0<<WGM10);			// Normal counter mode
	TCCR1B = (1<<WGM12)|(5<<CS10);	// Normal counter mode
	TIMSK1 = (1<<OCIE1A);			// Interrupt on TCTN1 = OCR1A
	
	OCR1A = 1600;						// Sample time
}


int main(void)
{
    cli();
#ifdef PORT_DEBUG
	DDRA = 0xff;  // DEBUG: output
	DDRB = 0xff;  // DEBUG: output
#endif
	i2c_init();
	
	sensor_init();

	timer_init();
	
	sei();
	
    while (1) 
    {}
}


ISR(INT0_vect)  // LEFT
{
	left_counter++;
#ifdef PORT_DEBUG
	PORTA = left_counter;  // DEBUG: 
#endif
}


ISR(INT1_vect)  // RIGHT
{
	right_counter++;
#ifdef PORT_DEBUG
	PORTB = right_counter;  // DEBUG:
#endif
}


ISR(TIMER1_COMPA_vect)
{
	speed = (left_counter+right_counter) * 63;  // Pre calculated for mm/s
	left_counter = 0;
	right_counter = 0;
}


ISR(TWI_vect)
{
	uint8_t TW_STATUS = TWSR & 0xf8;  // Extract I2C status
	
	switch(TW_STATUS)
	{
		case TW_ST_SLA_ACK:
			speed_buffer = speed;
			TWDR = speed_buffer % 0x100;
			TWCR = (1<<TWIE)|(1<<TWINT)|(1<<TWEA)|(1<<TWEN);  // Send last byte
			break;
		case TW_ST_DATA_ACK:
			TWDR = speed_buffer / 0x100;
			TWCR = (1<<TWIE)|(1<<TWINT)|(0<<TWEA)|(1<<TWEN);  // Send last byte
			break;
		case TW_ST_STOP:
			TWCR = (1<<TWIE)|(1<<TWINT)|(1<<TWEA)|(1<<TWEN);  // Return to idle
		default:
			TWCR = (1<<TWIE)|(1<<TWINT)|(1<<TWEA)|(1<<TWEN);
			break;
	}
}
