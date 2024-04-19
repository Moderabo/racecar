#pragma once

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

#include "Lidar.h"

class MQTT_Connection
{
public:
        MQTT_Connection();
	~MQTT_Connection();

	std::vector<float> receiveMsg();

	void pubSpeed(float time, float speed);
	void pubCones(std::vector<Cone> &cones);

private:
	mqtt::async_client m_cli{"10.42.0.1","RPi" };
	mqtt::topic m_speedTopic{m_cli,"data" ,1 };
	mqtt::topic m_conesTopic{m_cli,"cones" ,1 };

};
