
CCFLAGS = -pthread -Ilidar_lib -IReglering

LDFLAGS = -Llidar_lib -lsl_lidar_sdk -lwiringPi -lpaho-mqttpp3 -lpaho-mqtt3as

main: main.o Lidar.o
	g++ $^ $(CCFLAGS) $(LDFLAGS) -o $@.out

main.o: main.cc
	g++ $(CCFLAGS) $(LDFLAGS) -c $< -o $@

%.o: %.cc %.h
	g++ $(CCFLAGS) $(LDFLAGS) -c $< -o $@

clean:
	@ \rm -rf *.o *.out

