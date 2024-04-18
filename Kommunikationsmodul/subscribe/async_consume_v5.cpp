// async_consume_v5.cpp

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



using namespace std;

const string SERVER_ADDRESS	{ "10.42.0.1" };
const string CLIENT_ID		{ "submarine" };
const string TOPIC 			{ "commands" };

const int  QOS = 1;

/////////////////////////////////////////////////////////////////////////////

int main(int argc, char* argv[])
{
	//I2C setup
	I2CConnection i2c_connection {CONTROL_AVR};


	// Create a client using MQTT v5
	mqtt::create_options createOpts(MQTTVERSION_5);
	mqtt::async_client cli(SERVER_ADDRESS, CLIENT_ID, createOpts);

	auto connOpts = mqtt::connect_options_builder()
		.properties({
			{mqtt::property::SESSION_EXPIRY_INTERVAL, 604800}
	    })
		.clean_session(false)
		.finalize();

	try {
		cli.set_connection_lost_handler([](const std::string&) {
			cout << "*** Connection Lost ***" << endl;
		});

		cli.set_disconnected_handler([](const mqtt::properties&, mqtt::ReasonCode reason) {
			cout << "*** Disconnected. Reason: " << reason << " ***" << endl;
		});

		// Start consumer before connecting to make sure to not miss messages

		cli.start_consuming();

		// Connect to the server

		cout << "Connecting to the MQTT server..." << flush;
		auto tok = cli.connect(connOpts);

		// Getting the connect response will block waiting for the
		// connection to complete.
		auto rsp = tok->get_connect_response();

		// Make sure we were granted a v5 connection.
		if (rsp.get_mqtt_version() < MQTTVERSION_5) {
			cout << "Did not get an MQTT v5 connection." << endl;
			exit(1);
		}

		// If there is no session present, then we need to subscribe, but if
		// there is a session, then the server remembers us and our
		// subscriptions.
		if (!rsp.is_session_present()) {
			cout << "Session not present on broker. Subscribing." << endl;
			cli.subscribe(TOPIC, QOS)->wait();
		}

		cout << "OK" << endl;

		// Consume messages
		// This just exits if the client is disconnected.
		// (See some other examples for auto or manual reconnect)

		cout << "Waiting for messages on topic: '" << TOPIC << "'" << endl;




		while (tolower(cin.get()) != 'q') {
			auto msg = cli.consume_message();
			if (!msg) break;
			cout << msg->get_topic() << ": " << msg->to_string() << endl;

			std::stringstream test(msg->to_string());
                        std::string commands;
                        std::vector<float> commands_value_list;

                        while(std::getline(test, commands, ' '))
                        {
                        	commands_value_list.push_back(std::stof(commands));
                        }

                        if(commands_value_list[2])
                        {
                                //i2c_connection.steer(0);
                                //sleep(0.01);
                                //i2c_connection.gas(0);
                        	break;
                        }

                        //i2c_connection.steer(commands_value_list[0]);
                        //sleep(0.11);
                        //i2c_connection.gas(commands_value_list[1]);
		}

		// If we're here, the client was almost certainly disconnected.
		// But we check, just to make sure.

		if (cli.is_connected()) {
			cout << "\nShutting down and disconnecting from the MQTT server..." << flush;
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

