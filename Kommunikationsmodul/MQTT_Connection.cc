
#include "MQTT_Connection.h"


MQTT_Connection::MQTT_Connection()
{

	int QOS = 1;
	std::string COMMAND_TOPIC = "commands";
	// Create connect options

	auto connOpts = mqtt::connect_options_builder()
		.clean_session(false)
		.finalize();

	// Start consumer before connecting to make sure to not miss messages

	m_cli.start_consuming();

        // Connect to the server

	std::cout << "Connecting to the MQTT server..." << std::flush;
	auto tok = m_cli.connect(connOpts);

	// Getting the connect response will block waiting for the
	// connection to complete.
	auto rsp = tok->get_connect_response();

	// If there is no session present, then we need to subscribe, but if
	// there is a session, then the server remembers us and our
	// subscriptions.
	if (!rsp.is_session_present())
		m_cli.subscribe(COMMAND_TOPIC, QOS)->wait();

	std::cout << "OK" << std::endl;

	// Consume messages
	// This just exits if the client is disconnected.
	// (See some other examples for auto or manual reconnect)
	std::cout << "Waiting for messages on topic: '" << COMMAND_TOPIC << "'" << std::endl;

}

MQTT_Connection::~MQTT_Connection()
{
	if (m_cli.is_connected()) {
		std::cout << "\nShutting down and disconnecting from the MQTT server..." << std::flush;
		m_cli.stop_consuming();
		m_cli.disconnect()->wait();
		std::cout << "OK" << std::endl;
	}
	else {
		std::cout << "\nClient was disconnected" << std::endl;
	}

}

std::vector<float> MQTT_Connection::receiveMsg()
{

	// Takes message and returns data form the message in form a vector.

	std::string commands;
	std::vector<float> commands_value_list;

	auto msg = m_cli.consume_message();

        if (!msg)
        {
		return  commands_value_list;
	}

	std::stringstream test(msg->to_string());

	while(std::getline(test, commands, ' '))
	{
		 commands_value_list.push_back(std::stof(commands));
	}

	return commands_value_list;
}

void MQTT_Connection::pubSpeed(float time, float speed)
{
	// Publishes the speed and time.

	std::string payload = std::to_string(time) + " " + std::to_string(speed);
	m_speedTopic.publish(payload);
}

void MQTT_Connection::pubCones(std::vector<Cone> &cones)
{
        // Publishes all cones.

	std::string payload = "";

	for (auto cone = cones.begin(); cone != cones.end(); cone++)
        {
		payload = payload + std::to_string(cone.x) + " " + std::to_string(cone.y ) + " " + std::to_string(cone.r) + " " ;
	}


        m_conesTopic.publish(payload);
}

