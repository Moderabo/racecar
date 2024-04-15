/*
 *  Vårt eget test
 *
 */

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
#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif


using namespace sl;

bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}

int main(int argc, const char * argv[]) {

    std::ofstream datafile;
    datafile.open ("data.csv");
    std::ofstream conesfile;
    conesfile.open ("cones.csv");

    signal(SIGINT, ctrlc);
    
    // Define the result
    sl_result     op_result;
    
    // Create the channel instance
	Result<IChannel*> channel = createSerialPortChannel("/dev/ttyUSB0", 115200);

	// Create the driver instance
	ILidarDriver* lidar = *createLidarDriver();

	lidar->connect(*channel);

    // start scan...
    lidar->startScan(false,true);

    lidar->setMotorSpeed(200);


    // fetech result and print it out, do it 10 times
    for(int i = 0; i < 1; ++i) {
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
    	if (abs(clusters.front().front().second - clusters.back().back().second) < 200.f)
    	{
		clusters.front().insert(clusters.front().begin(), clusters.back().begin(), clusters.back().end());
		clusters.pop_back();
    	}
    	std::vector<int> small_clusters {};
    	for (int i {0}; i < clusters.size(); i++)
    	{
    		if (clusters.at(i).size() < 5)
		{
			small_clusters.push_back(i);
		}
    	}
    	for (int i {small_clusters.size()-1}; i >= 0; i--)
    	{
    		clusters.erase(clusters.begin() + small_clusters.at(i));
    	}
	int object {0};
	for (auto cluster : clusters)
	{
		for (auto coordinate : cluster)
		{
			// write data to disc
			datafile  << coordinate.first << ',' << coordinate.second << ',' << object << '\n';
		}
		object++;
	}
	std::vector<std::pair<float,float>> cones {};
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
	for (auto cone : cones)
	{
		conesfile << cone.first << ',' << cone.second << '\n';
	}
        if (ctrl_c_pressed){ 
            break;
        }
    }
    lidar->stop();
    datafile.close();
    conesfile.close();
    // done!
    if(lidar) {
        delete lidar;
        lidar = NULL;
    }
    return 0;
}

