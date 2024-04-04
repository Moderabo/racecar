/*
 *  Vårt eget test
 *  
 */

#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <signal.h>
#include <string.h>

#include "sl_lidar.h" 
#include "sl_lidar_driver.h"
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
    
    lidar->setMotorSpeed(600);
    
        
    // fetech result and print it out...
    while (1) {
        sl_lidar_response_measurement_node_hq_t  nodes[8192];
        size_t   count = _countof(nodes);

        op_result = lidar->grabScanDataHq(nodes, count);

        if (SL_IS_OK(op_result)) {
            lidar->ascendScanData(nodes, count);
            for (int pos = 0; pos < (int)count ; ++pos) {
                printf("%s theta: %03.2f Dist: %08.2f Q: %d \n", 
                    (nodes[pos].flag & SL_LIDAR_RESP_HQ_FLAG_SYNCBIT) ?"S ":"  ", 
                    (nodes[pos].angle_z_q14 * 90.f) / 16384.f,
                    nodes[pos].dist_mm_q2/4.0f,
                    nodes[pos].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
            }
        }
        
        if (ctrl_c_pressed){ 
            break;
        }
    }

    lidar->stop();
	
    // done!
    if(lidar) {
        delete lidar;
        lidar = NULL;
    }
    return 0;
}

