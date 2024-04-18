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


class MQTT_Connection
{
public:
        MQTT_Connection();
	~MQTT_Connection();

	std::vector<float> receiveMsg();

	void publishMsg(float time, float speed);

private:
	mqtt::async_client m_cli{"10.42.0.1","RPi" };
	mqtt::topic m_top{m_cli,"data" ,1 };

};
