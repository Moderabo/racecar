#include "MQTT_Connection.h"


MQTT_Connection::MQTT_Connection()
{

	int QOS = 1;
	std::string COMMAND_TOPIC = "commands";
	
	// Creates connect options
	auto connOpts = mqtt::connect_options_builder()
		.clean_session(false)
		.finalize();


	
	// Start consuming messages
	m_cli.start_consuming();

	// Initiate connection to broker with connect options
	auto tok = m_cli.connect(connOpts);

	// Check if the session is already present
	auto rsp = tok->get_connect_response();

	if (!rsp.is_session_present())
		m_cli.subscribe(COMMAND_TOPIC, QOS)->wait();

}

MQTT_Connection::~MQTT_Connection()
{
	//Disconnects client if its not already disconnected.

	if (m_cli.is_connected()) {
		m_cli.stop_consuming();
		m_cli.disconnect()->wait();
	}

}

std::vector<float> MQTT_Connection::receiveMsg()
{
	// Takes message and returns data form the message in form a vector.

	std::string commands;
	std::vector<float> commands_value_list;

	//Takes latest message from que if there is any.
	auto msg = m_cli.try_consume_message_for(std::chrono::seconds(0));
	if (!msg)
        {
                return  commands_value_list;
        }

	// Clears message que.
	while(true)
	{
	 	auto tmp_msg = m_cli.try_consume_message_for(std::chrono::seconds(0));
		if (!tmp_msg)
        	{
                	break;
       		}
		msg = tmp_msg;
	}

	//Converts message to float commands:
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

	for (auto cone : cones)
        {
		payload = payload + std::to_string(cone.x) + "," + std::to_string(cone.y ) + "," + std::to_string(cone.r) + ";" ;
	}
        m_conesTopic.publish(payload);
}

void MQTT_Connection::pubBezier(std::string msg)
{
    // Publishes Bezier points.

    m_bezierTopic.publish(msg);
}

void MQTT_Connection::pubCurve(std::string msg)
{
    // Publishes all curve points.

    m_curveTopic.publish(msg);
}

void MQTT_Connection::pubLap(std::string msg)
{
    // Publishes current lap.

    m_lapTopic.publish(msg);
}
