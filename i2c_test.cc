#include <iostream>
#include <wiringPiI2C.h>


#define CONTROL_AVR   0x19

int main()
{
	//Setup I2C communication
	int fd, result;
	int fd2;
	fd = wiringPiI2CSetup(CONTROL_AVR);
	//fd2 = wiringPiI2CSetup(0x12);
	if (fd == -1){
	std::cout << "Failed to init I2C.\n";
	return -1;
	}

	result =  wiringPiI2CWriteReg16(fd, 0x01 , 0x8000);
  	std::cout << "Init result: "<< fd << "\n";
	//std::cout << "Init result2: "<< fd2 << "\n";
	return 0;
}
