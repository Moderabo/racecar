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
#define TW_SR_STOP			0xA0

volatile uint8_t i2c_byte_cnt = 0;  // Tell which byte in sequence
volatile uint8_t control_switch = 0;  // Key to gasrelage or styr
volatile uint16_t control_signal = 0;  // 


void i2c_init()
{
	TWAR = (ADDR<<TWA0)|(0<<TWGCE);
	TWCR = (1<<TWINT)|(1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(1<<TWEN)|(1<<TWIE);  // Slave receiver mode
}

void pwm_init()
{
	// use fast PWM
	
	// Styrservo
	DDRD = (1<<4); // Output on OC1B
	
	TCCR1A = (1<<COM1A0)|(2<<COM1B0)|(3<<WGM10);
	TCCR1B = (3<<WGM12)|(2<<CS10);
	
	OCR1A = 20597; // OCR0A should be the PWM 
	OCR1B = (1.5 * 2059.7) / 2; // OCR0B should be the pulse length 1.5 ms / 20 ms * TOP
	
	// Gasreglage
	DDRB = (1<<7); // Output on OC3B
	
	TCCR3A = (1<<COM3A0)|(2<<COM3B0)|(3<<WGM30);  // Set when timer hits BOTTOM, clear on compare match, mode 3 fast PWM
	TCCR3B = (3<<WGM32)|(2<<CS30);  // other part of WGM0, clock select f / 1024
	
	OCR3A = 20597; // OCR0A should be the PWM frequency
	OCR3B = (1.5 * 2059.7) / 2; // OCR0B should be the pulse length 1.5 ms / 20 ms * TOP
}

int main(void)
{
	int i;
	
	cli();
	
	// TWI
	i2c_init();	
	
	// PWM
	pwm_init();

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
	uint8_t TW_STATUS = TWSR & 0xf8;  // Extract I2C status
	
	switch(TW_STATUS)
	{
		case TW_SR_SLA_ACK:
			TWCR = (1<<TWIE)|(1<<TWINT)|(1<<TWEA)|(1<<TWEN);  // Enable new interrupt
			break;
		case TW_SR_DATA_ACK:
			switch(i2c_byte_cnt)
			{
				case 0:
					control_switch = TWDR;
					break;
				case 1:
					control_signal |= (TWDR<<0);
					break;
				case 2:
					control_signal |= (TWDR<<8);
					break;
				default:
					break;
			}
			i2c_byte_cnt++;
			TWCR = (1<<TWIE)|(1<<TWINT)|(1<<TWEA)|(1<<TWEN);
			break;
		case TW_SR_STOP:
			switch(control_switch)
			{
				case 1:
					OCR1B = (2059.7 + control_signal * 0.0314) / 2;
					break;
				case 2:
					OCR3B = (2059.7 + control_signal * 0.0314) / 2;
					break;
				default:
					break;
			}
			i2c_byte_cnt = 0;
			control_switch = 0;
			control_signal = 0;
			TWCR = (1<<TWIE)|(1<<TWINT)|(1<<TWEA)|(1<<TWEN);
			break;
		default:
			TWCR = (1<<TWIE)|(1<<TWINT)|(1<<TWEA)|(1<<TWEN);
			break;
	}
}
