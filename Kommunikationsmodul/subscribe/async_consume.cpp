
#include <iostream>
#include <cstdlib>
#include <string>
#include <cstring>
#include <cctype>
#include <thread>
#include <chrono>
#include "mqtt/async_client.h"
#include <vector>
#include <sstream>

#include "MQTT_Connection.h"

// I2C includes
#include <iostream>
#include <wiringPiI2C.h>
#include <unistd.h>


#define CONTROL_AVR   0x19
#define SENSOR_AVR    34
using namespace std;

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

void gas(float x)
{
	int result;
	if ( x < -1 || x > 1 )
	{
		return;
	}
	uint16_t x_2byte {(0xffff * (1 + x)) / 2};
	cout << "Gas: " << x_2byte << '\n';
	result = wiringPiI2CWriteReg16(fd_CONTROL, 0x02, x_2byte);
}


void steer(float x)  // -1 left, 1 right
{
	int result;
	if ( x < -1 || x > 1 )
	{
		return;
	}
	uint16_t x_2byte {(0xffff * (1 + x)) / 2};
	cout << "Steer: " << x_2byte << '\n';
	result = wiringPiI2CWriteReg16(fd_CONTROL, 0x01, x_2byte);
}


int get_speed()
{
        int result;

        result = wiringPiI2CRead(fd_SENSOR);
	cout << "Speed: " << result << '\n';
	return result;
}

private:
	int fd_CONTROL;
	int fd_SENSOR;
};


uint64_t timestamp()
{
        auto now = std::chrono::system_clock::now();
        auto tse = now.time_since_epoch();
        auto msTm = std::chrono::duration_cast<std::chrono::milliseconds>(tse);
        return uint64_t(msTm.count());
}

/////////////////////////////////////////////////////////////////////////////

int main(int argc, char* argv[])
{
	uint64_t        t = timestamp(),
                        tlast = t,
                        tstart = t;

	int speed = 0;
	// Setup I2C communication
	I2CConnection i2c_connection;
	MQTT_Connection mqtt_connection;


        while (true)
        {
		std::vector<float> commands_value_list;

		commands_value_list = mqtt_connection.receiveMsg();

		if( commands_value_list.empty() ){
		break;
		}

		if(commands_value_list[2])
		{
			i2c_connection.steer(0);
			sleep(0.01);
           		i2c_connection.gas(0);
			break;
		}


		i2c_connection.steer(commands_value_list[0]);
		sleep(0.1);
		i2c_connection.gas(commands_value_list[1]);
		sleep(0.01);
		speed = i2c_connection.get_speed();

		t = timestamp();
                tlast = t - tstart;
		mqtt_connection.publishMsg( tlast, speed );

         }


 	return 0;
}

