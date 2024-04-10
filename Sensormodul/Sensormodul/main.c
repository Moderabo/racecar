/*
 * Sensormodul.c
 *
 * Created: 2024-04-08 16:10:14
 * Author : aleli930
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>

//#define PORT_DEBUG

volatile uint8_t left_counter = 0;
volatile uint8_t right_counter = 0;

volatile uint8_t i2c_byte_cnt = 0;  // Tell which byte in sequence

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


int main(void)
{
    cli();
#ifdef PORT_DEBUG
	DDRA = 0xff;  // DEBUG: output
	DDRB = 0xff;  // DEBUG: output
#endif
	i2c_init();
	
	sensor_init();
	
	sei();
	
    while (1) 
    {
    }
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

ISR(TWI_vect)
{
	uint8_t TW_STATUS = TWSR & 0xf8;  // Extract I2C status
	
	switch(TW_STATUS)
	{
		case TW_ST_SLA_ACK:
			TWDR = left_counter;
			TWCR = (1<<TWIE)|(1<<TWINT)|(1<<TWEA)|(1<<TWEN);  // Send another byte
			break;
		case TW_ST_DATA_ACK:
			TWDR = right_counter;
			TWCR = (1<<TWIE)|(1<<TWINT)|(0<<TWEA)|(1<<TWEN);  // Send last byte
			break;
		case TW_ST_STOP:
			TWCR = (1<<TWIE)|(1<<TWINT)|(1<<TWEA)|(1<<TWEN);  // Return to idle
		default:
			TWCR = (1<<TWIE)|(1<<TWINT)|(1<<TWEA)|(1<<TWEN);
			break;
	}
}
