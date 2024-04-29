
CCFLAGS = -MMD -pthread -Ilidar_lib -IReglering

LDFLAGS = -Llidar_lib -lsl_lidar_sdk -lwiringPi -lpaho-mqttpp3 -lpaho-mqtt3as

main: main.o Lidar.o utils.o MQTT_Connection.o
	g++ $^ $(CCFLAGS) $(LDFLAGS) -o $@.out

exec: main
	./main.out

main.o: main.cc
	g++ $(CCFLAGS) $(LDFLAGS) -c $< -o $@

%.o: %.cc %.h
	g++ $(CCFLAGS) $(LDFLAGS) -c $< -o $@

-include *.d

clean:
	@ \rm -rf *.o *.out *.d

