
SRC = src
IDIR = include
OBJDIR = objdir

# .o files required for main program
MAIN = $(addprefix $(OBJDIR), main.o Lidar.o utils.o MQTT_Connection.o \
	   						  Planner.o)

CCFLAGS = -MMD -pthread -Ilidar_lib -I$(IDIR)

LDFLAGS = -Llidar_lib -lsl_lidar_sdk -lwiringPi -lpaho-mqttpp3 -lpaho-mqtt3as

main: $(MAIN)
	g++ $^ $(CCFLAGS) $(LDFLAGS) -o $@.out

exec: main
	./main.out

$(OBJDIR)/main.o: $(SRC)/main.cc | dir
	g++ $(CCFLAGS) $(LDFLAGS) -c $< -o $@

$(OBJDIR)/%.o: $(SRC)/%.cc $(IDIR)/%.h | dir
	g++ $(CCFLAGS) $(LDFLAGS) -c $< -o $@

dir:
	@mkdir -p $(OBJDIR)

-include *.d

clean:
	@ \rm -rf *.o *.out *.d

