#include <iostream>
#include <wiringPiI2C.h>
#include <unistd.h>


#define CONTROL_AVR   0x19

class I2CConnection
{
	//Setup I2C communication
public: 
	I2CConnection(int avr_adr) : fd {wiringPiI2CSetup(avr_adr)}
	{
		if (fd == -1){
			std::cout << "Failed to init I2C.\n";
		}
	}

void gas(float x)
{
	int result;
	if ( x < -1 || x > 1 )
	{
		return;
	}
	uint16_t x_2byte {(0xffff * (1 + x)) / 2};
	result = wiringPiI2CWriteReg16(fd, 0x02, x_2byte);
}

void steer(float x)  // -1 left, 1 right
{
	int result;
	if ( x < -1 || x > 1 )
	{
		return;
	}
	uint16_t x_2byte {(0xffff * (1 + x)) / 2};
	result = wiringPiI2CWriteReg16(fd, 0x01, x_2byte);
}
private:
	int fd;
};


int main()
{
	I2CConnection i2c_connection {CONTROL_AVR};
	
	i2c_connection.gas(-.2);
	sleep(1);
	i2c_connection.gas(0);
	sleep(1);
	i2c_connection.gas(.2);
	sleep(1);
	i2c_connection.gas(0);
	sleep(4);
	i2c_connection.gas(-.3);

	/*
	for (int i {0}; i<5;i++)
	{
		i2c_connection.gas(.2);
		sleep(0.1);
		//i2c_connection.steer(1);
		sleep(3);
		i2c_connection.gas(0);
		sleep(.1);
		sleep(10);
		i2c_connection.gas(-.4);
		sleep(.1);
		//i2c_connection.steer(-1);
		sleep(3);
		i2c_connection.gas(0);
		sleep(.1);
		sleep(3);
	}*/
	sleep(1);
	i2c_connection.steer(0);
	sleep(1);
	i2c_connection.gas(0);
	return 0;
}
