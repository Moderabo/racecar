#ifndef LIDAR
#define LIDAR

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

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

#include "utils.h"

using namespace sl;


class Lidar
{
public:
    Lidar();
    ~Lidar();

    void update();
    std::vector<Cluster> getPoints();
    std::vector<Cone> getCones();
private:
    std::vector<Cluster> clusters;
    std::vector<Cone> cones;
    
    // Define the result
    sl_result op_result;
    
    // Create the channel instance
	Result<IChannel*> channel;

	// Create the driver instance
	ILidarDriver* lidar;

    void filter();
    void findCones();
};

#endif

