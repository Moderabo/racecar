
// async_consume.cpp
//
// VIKTIGT!!!!
// Man  måste byta clientid varje gång innnan man kompilerar för att köra programmet mer en än gång!!!.
// Fixar detta nästa vecka då jag förhoppningsvis är pigg.


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

const string SERVER_ADDRESS	{ "10.42.0.1" };
const string CLIENT_ID		{ "CGgggga666" };
const string TOPIC 		{ "commands" };

const int  QOS = 1;

/////////////////////////////////////////////////////////////////////////////

int main(int argc, char* argv[])
{
	uint64_t        t = timestamp(),
                        tlast = t,
                        tstart = t;

	int speed = 0;
	// Setup I2C communication
	I2CConnection i2c_connection;


	mqtt::async_client cli(SERVER_ADDRESS, CLIENT_ID);

	auto connOpts = mqtt::connect_options_builder()
		.clean_session(false)
		.finalize();

	try {
		// Start consumer before connecting to make sure to not miss messages

		cli.start_consuming();

		// Connect to the server

		cout << "Connecting to the MQTT server..." << flush;
		auto tok = cli.connect(connOpts);

		// Getting the connect response will block waiting for the
		// connection to complete.
		auto rsp = tok->get_connect_response();

		// If there is no session present, then we need to subscribe, but if
		// there is a session, then the server remembers us and our
		// subscriptions.
		if (!rsp.is_session_present())
			cli.subscribe(TOPIC, QOS)->wait();

		cout << "OK" << endl;

		// Consume messages
		// This just exits if the client is disconnected.
		// (See some other examples for auto or manual reconnect)


		auto top = mqtt::topic(cli, "data", QOS);

		cout << "Waiting for messages on topic: '" << TOPIC << "'" << endl;


                while (true)
              	{
		auto msg = cli.consume_message();

                if (!msg)
                {
			break;
		}

                cout << msg->get_topic() << ": " << msg->to_string() << endl;

				// Converts MQTT message to a vector consisting off floats
				// Send three floats with a space beetween them
				// First float controls stering, second one controls gas and third stops machine

				std::stringstream test(msg->to_string());
				std::string commands;
				std::vector<float> commands_value_list;

				while(std::getline(test, commands, ' '))
				{
					 commands_value_list.push_back(std::stof(commands));
				}

				if(commands_value_list[2])
				//if(false)
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
				std::string payload = to_string(tlast)	+ " " + to_string(speed);
				top.publish(payload);

            }



		// If we're here, the client was almost certainly disconnected.
		// But we check, just to make sure.

		if (cli.is_connected()) {
			cout << "\nShutting down and disconnecting from the MQTT server..." << flush;
			//cli.unsubscribe(TOPIC)->wait();
			cli.stop_consuming();
			cli.disconnect()->wait();
			cout << "OK" << endl;
		}
		else {
			cout << "\nClient was disconnected" << endl;
		}
	}
	catch (const mqtt::exception& exc) {
		cerr << "\n  " << exc << endl;
		return 1;
	}

 	return 0;
}

