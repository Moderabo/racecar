
CCFLAGS = -pthread -Ilidar_lib

LDFLAGS = -Llidar_lib -lsl_lidar_sdk -lwiringPi

main: main.o Lidar.o
	g++ $^ $(CCFLAGS) $(LDFLAGS) -o $@.out

main.o: main.cc
	g++ $(CCFLAGS) $(LDFLAGS) -c $< -o $@

%.o: %.cc %.h
	g++ $(CCFLAGS) $(LDFLAGS) -c $< -o $@

clean:
	@ \rm -rf *.o *.out

