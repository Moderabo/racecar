async_con : async_consume.cpp
	g++ async_consume.cpp -pthread -lpaho-mqttpp3 -lpaho-mqtt3as -l wiringPi -o async_con.out
MQTT_test : MQTT_Connection.cc
	g++ async_consume.cpp MQTT_Connection.cc -pthread -lpaho-mqttpp3 -lpaho-mqtt3as -l wiringPi -o MQTT_test.out
