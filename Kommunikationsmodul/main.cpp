/*
 *  Vårt eget test
 *
 */

// I2C includes
#include <iostream>
#include <wiringPiI2C.h>
#include <unistd.h>

// lidar includes
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <vector>
#include "sl_lidar.h" 
#include "sl_lidar_driver.h"
#include <utility>
#include <algorithm>

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#define CONTROL_AVR   0x19
#define SENSOR_AVR    34

using namespace std;
using namespace sl;

// define the I2C interface
class I2CConnection
{
	//Setup I2C communication
public:
	I2CConnection()
	{
		fd_CONTROL = wiringPiI2CSetup(25);
		if (fd_CONTROL == -1){
			std::cout << "Failed to init I2C.\n";
		}

		fd_SENSOR = wiringPiI2CSetup(34);
		 if (fd_SENSOR  == -1){
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
	cout << "Gas: " << x_2byte << '\n';
	result = wiringPiI2CWriteReg16(fd_CONTROL, 0x02, x_2byte);
}


void steer(float x)  // -1 left, 1 right
{
	int result;
	if ( x < -1 || x > 1 )
	{
		return;
	}
	uint16_t x_2byte {(0xffff * (1 + x)) / 2};
	cout << "Steer: " << x_2byte << '\n';
	result = wiringPiI2CWriteReg16(fd_CONTROL, 0x01, x_2byte);
}


int get_speed()
{
        int result;

        result = wiringPiI2CRead(fd_SENSOR);
	cout << "Speed: " << result << '\n';
	return result;
}

private:
	int fd_CONTROL;
	int fd_SENSOR;
};

const string SERVER_ADDRESS	{ "10.42.0.1" };
const string CLIENT_ID		{ "CGgggga666" };
const string TOPIC 		{ "commands" };

const int  QOS = 1;

typedef std::vector<std::pair<float,float>> Cluster;
typedef std::pair<float,float> Cone;
typedef std::pair<Cone,Cone> Gate;

bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}

void filter(std::vector<std::vector<std::pair<float,float>>> &clusters)
{
	// Remove clusters which have few datapoints

	// indicies of small clusters
	std::vector<int> small_clusters {};
	
	// Find small clusters
	for (int i {0}; i < clusters.size(); i++)
	{
 		if (clusters.at(i).size() < 3)
    	{
			small_clusters.push_back(i);
	    }
   	}
	// Remove small clusters
    for (int i { static_cast<int>(small_clusters.size()) - 1 }; i >= 0; i--)
    {
    	clusters.erase(clusters.begin() + small_clusters.at(i));
    }
}

std::vector<Cone> findCones(std::vector<std::vector<std::pair<float,float>>> &clusters)
{
	// Create cones from clusters filtering out to large objects
	std::vector<Cone> cones {};

	for (auto cluster : clusters)
	{
		float theta1 = cluster.front().first;
		float theta2 = cluster.back().first;
		float r1 = cluster.front().second;
		float r2 = cluster.back().second;
		float x1 = r1 * cos(theta1);
		float x2 = r2 * cos(theta2);
		float y1 = r1 * sin(theta1);
		float y2 = r2 * sin(theta2);

		if (abs((x1-x2) * (x1-x2) + (y1-y2) * (y1-y2)) > 230*230)
		{
			continue;
		}
		float x = (x1 + x2)/2;
		float y = (y1 + y2)/2;
		std::pair<float,float> xy {x, y};
		cones.push_back(xy);
	}

	return cones;
}

bool validGate(Cone &cone1, Cone &cone2)
{
    std::pair<float,float> gate_vect;
    gate_vect.first = cone1.first - cone2.first;
    gate_vect.second = cone1.second - cone2.second;

    float gate_dist_squared {gate_vect.first*gate_vect.first + gate_vect.second*gate_vect.second};

    if ( gate_dist_squared > 700*700)
    {
        return false;
    }
    else
    {
        return true;
    }
}


std::vector<Gate> findGates(std::vector<Cone> &cones)
{
    std::vector<Gate> gates {};

    for (auto cone1 = cones.begin(); cone1 != cones.end(); cone1++)
    {
        for (auto cone2 {cone1 + 1}; cone2 != cones.end(); cone2++)
        {
            if ( validGate(*cone1, *cone2) )
            {
                Gate gate {*cone1, *cone2};
                gates.push_back(gate);
            }
        }
    }

    return gates;
}


Gate findNextGate(std::vector<Gate> &gates)
{
    for (auto &gate : gates)
    {
        if ( gate.first.first > 0 && gate.second.first > 0 )
        {
           return gate;
        }
    }
    
    Gate no_gate {};
    return no_gate;
}


void saveClusters(std::vector<std::vector<std::pair<float,float>>> &clusters)
{
	// Save clusters to file
	std::ofstream datafile;
	datafile.open("data.csv");

	// Enumerate clusters
	int object {0};

	for (auto &cluster : clusters)
	{
		for (auto &coordinate : cluster)
		{
			// write data to disc
			datafile  << coordinate.first << ',' << coordinate.second << ',' << object << '\n';
		}
		object++;
	}

	datafile.close();
}


void saveCones(std::vector<std::pair<float,float>> &cones)
{
	std::ofstream conesfile;
	conesfile.open("cones.csv");

	for (auto &cone : cones)
	{
		conesfile << cone.first << ',' << cone.second << '\n';
	}

	conesfile.close();
}


void saveGates(std::vector<Gate> &gates)
{
	std::ofstream file;
	file.open("gates.csv");

	for (auto &gate : gates)
	{
		file << gate.first.first   << ','
             << gate.first.second  << ','
             << gate.second.first  << ','
             << gate.second.second << ',' << '\n';
	}

	file.close();
}


int main(int argc, const char * argv[]) {

    signal(SIGINT, ctrlc);
	
	// Setup I2C communication
	I2CConnection i2c_connection {};

    // Define the result
    sl_result     op_result;
    
    // Create the channel instance
	Result<IChannel*> channel = createSerialPortChannel("/dev/ttyUSB0", 115200);

	// Create the driver instance
	ILidarDriver* lidar = *createLidarDriver();

	lidar->connect(*channel);

    // start scan...
    lidar->startScan(false,true);

    //lidar->setMotorSpeed(200);


    // fetch result and print it out, do it 10 times
    for(int i = 0; i < 10000000000; ++i)
    {
        sl_lidar_response_measurement_node_hq_t  nodes[8192];
        size_t   count = _countof(nodes);

        std::vector<std::vector<std::pair<float,float>>> clusters;

	    // get the lidar data
        op_result = lidar->grabScanDataHq(nodes, count);

	    // check if everythings ok
        if (SL_IS_OK(op_result)) {
            lidar->ascendScanData(nodes, count);

	        int object  = 0;

	        // initialize previous distance to something very big (100 meters)
	        float prev_distance = 100000.f;

	        // loop  through all the points
            for (int pos = 0; pos < (int)count ; ++pos) {
		        // converte to milimeters and radians
		        float angle = (nodes[pos].angle_z_q14 * 90.f) / 16384.f * M_PI / 180.f;
		        float distance = nodes[pos].dist_mm_q2/4.0f;
		        int quality = nodes[pos].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;
		        std::pair<float,float> node{angle,distance};

		        switch(quality){
			    // not ok
			    case 0:
				    break;
			    // ok
			    case 47:
				    //std::cout << "Angle: " << angle << "\tdistance " << distance << std::endl;
				    // check if this should be the start of a new object
				    if (abs( distance - prev_distance) > 200.f) {
					    std::vector<std::pair<float,float>> cluster {};
					    clusters.push_back(cluster);
				    }
				    clusters.back().push_back(node);
				    // uppdate prev_distance
				    prev_distance = distance;
				    break;
		        }
            }
        }

    	// Concatenate first and last cluster if they are actually the same (circles)
        if (abs(clusters.front().front().second - clusters.back().back().second) < 200.f)
        {
    		clusters.front().insert(clusters.front().begin(), clusters.back().begin(), clusters.back().end());
    		clusters.pop_back();
        }
    
    	// Remove small clusters
    	filter(clusters);
    
    	//saveClusters(clusters);
    
	    std::vector<std::pair<float,float>> cones;
	    cones = findCones(clusters);

	    //saveCones(cones);

        std::vector<Gate> gates;
        gates = findGates(cones);

        //saveGates(gates);

        Gate next_gate;
        next_gate = findNextGate(gates);

        std::pair<float,float> next_gate_midpoint;
        next_gate_midpoint.first = (next_gate.first.first + next_gate.second.first) / 2;
        next_gate_midpoint.second = (next_gate.first.second + next_gate.second.second) / 2;

        //std::cout << next_gate_midpoint.first << ','
        //          << next_gate_midpoint.second << std::endl;

        float angle_to_steer = atan2(next_gate_midpoint.second,next_gate_midpoint.first)*3;
        std::cout << angle_to_steer << '\n';
	
    	angle_to_steer = std::max(-1.f, std::min(angle_to_steer, 1.f));
    
        i2c_connection.steer(angle_to_steer);
    	sleep(0.1);
    	i2c_connection.gas(0.1);
    	sleep(0.1);

        if (ctrl_c_pressed){ 
            break;
        }
    }

    i2c_connection.steer(0);
    lidar->stop();

    // done!
    if(lidar) {
        delete lidar;
        lidar = NULL;
    }

    return 0;
}

