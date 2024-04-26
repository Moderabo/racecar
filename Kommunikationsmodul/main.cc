/*
 *  RPi main script
 *
 */
#include <Eigen/Dense>
#include <iostream>
#include <chrono>

#include "utils.h"
#include "I2C_Connection.h"
#include "Lidar.h"
#include "MQTT_Connection.h"
#include "Planner.h"

using namespace std;
using namespace Eigen;


uint64_t timestamp()
{
    auto now = std::chrono::system_clock::now();
    auto tse = now.time_since_epoch();
    auto msTm = std::chrono::duration_cast<std::chrono::milliseconds>(tse);
    return uint64_t(msTm.count());
}


bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}


int main(int argc, const char * argv[])
{
    signal(SIGINT, ctrlc);

    // Setup I2C communication
    I2CConnection i2c_connection {};

    // Setup MQTT communication
    MQTT_Connection mqtt_connection {};

    // Initiate LiDAR
    Lidar lidar {};

    Planner bezier{};

    uint64_t t {0};
    uint64_t tlast {0};
    int speed {0};


    // fetch result and print it out
    for(int i = 0; i < 10000; ++i)
    {
        // Fetch new LiDAR data
        lidar.update();

        //std::vector<Cluster> clusters {lidar.getPoints()};
    	//saveClusters(clusters);

        std::vector<Cone> cones {lidar.getCones()};
	    //saveCones(cones);

        // Find all valid gates
        std::vector<AltGate> gates;
        gates = findGates(cones);
        //saveGates(gates);

        // Find previous and next gate
        std::pair<AltGate,AltGate> prev_next_gate;
        prev_next_gate = findPrevNextGate(gates);

        AltGate prev_gate { prev_next_gate.first };
        AltGate next_gate { prev_next_gate.second };

        std::cout << "Previous gate: " << prev_gate.x << ", "
                                       << prev_gate.y << ", "
                                       << prev_gate.angle << '\n';
        std::cout << "Next gate: " << next_gate.x << ", "
                                   << next_gate.y << ", "
                                   << next_gate.angle << '\n';

        // Route planning and calculation of 
        bezier.Update(prev_gate.x, prev_gate.y, prev_gate.angle,
                      next_gate.x, next_gate.y ,next_gate.angle);

        float angle_to_steer = bezier.getRefAngle(0, 0, 0);

        //float angle_to_steer {3*atan2f(next_gate.y, next_gate.x)};
        std::cout << "trying angle: " << angle_to_steer << std::endl;

    	angle_to_steer = std::max(-1.f, std::min(angle_to_steer, 1.f));
    	float gas = std::max(-1.f, std::min(1.2f*bezier.getRefSpeed(), 1.f));

	    mqtt_connection.pubCones(cones);

        mqtt_connection.pubBezier(bezier.getBezier_points());
        mqtt_connection.pubCurve(bezier.getBezier_curve());

        i2c_connection.steer(angle_to_steer);
    	sleep(0.1);
    	i2c_connection.gas(1.5*gas);
    	sleep(0.1);
        speed = i2c_connection.getSpeed();

	t = timestamp();
        tlast = t - tlast;
	mqtt_connection.pubSpeed( tlast, speed );

        if (ctrl_c_pressed){
            break;
        }
    }
    return 0;
}

