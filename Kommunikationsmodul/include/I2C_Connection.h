#ifndef I2C_CONNECTION_H
#define I2C_CONNECTION_H

// I2C includes
#include <iostream>
#include <wiringPiI2C.h>
#include <unistd.h>

#define CONTROL_AVR   0x19
#define SENSOR_AVR    34

// define the I2C interface
class I2CConnection
{
	//Setup I2C communication
public:
	I2CConnection()
	{
		fd_CONTROL = wiringPiI2CSetup(25);
		if (fd_CONTROL == -1){
			std::cout << "Failed to init I2C.\n";
		}

		fd_SENSOR = wiringPiI2CSetup(34);
		if (fd_SENSOR  == -1){
            std::cout << "Failed to init I2C.\n";
        }
	}
    ~I2CConnection()
    {
        gas(0);
    }

    void gas(float x)
    {
        int result;
        if ( x < -1 || x > 1 )
        {
            return;
        }
        uint16_t x_2byte {static_cast<uint16_t>((0xffff * (1 + x)) / 2)};
        //std::cout << "Gas: " << x_2byte << '\n';
        result = wiringPiI2CWriteReg16(fd_CONTROL, 0x02, x_2byte);
    }


    void steer(float x)  // -1 left, 1 right
    {
        int result;
        if ( x < -1 || x > 1 )
        {
            return;
        }
        uint16_t x_2byte {static_cast<uint16_t>((0xffff * (1 + x)) / 2)};
        //std::cout << "Steer: " << x_2byte << '\n';
        result = wiringPiI2CWriteReg16(fd_CONTROL, 0x01, x_2byte);
    }


    int getSpeed()
    {
        int result;

        result = wiringPiI2CReadReg16(fd_SENSOR, 0x01);
        //std::cout << "Speed: " << result << '\n';
        return result;
    }

private:
    int fd_CONTROL;
    int fd_SENSOR;
};
#endif
