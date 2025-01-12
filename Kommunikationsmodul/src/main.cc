/*
 *  RPi main script
 *
 */
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

    Planner planner{};

    float T_c {0};
    float t {0};
    uint64_t tlast {timestamp()};
    int state {0};
    std::vector<float> commands;

    i2c_connection.steer(-1);
    sleep(1);
    i2c_connection.steer(1);
    sleep(1);
    i2c_connection.steer(0);

    // fetch result and print it out
    while ( true )
    {
	    if (ctrl_c_pressed)
        {
            break;
        }

        // steer, gas, mode
        commands = mqtt_connection.receiveMsg();

        if( !commands.empty() )
        {
            state = static_cast<int>(commands.at(0));

            switch ( state )
            {
                case 0:  // Idle
                    lidar.stop();
                    i2c_connection.steer(0);
                    i2c_connection.gas(0);
                    break;
                case 1:  // Manual
                    i2c_connection.steer(commands.at(1));
                    i2c_connection.gas(commands.at(2));
                    break;
                case 2:  // Automatic
                    planner.set_maximum_scaled_speed(commands.at(1));
                    planner.set_minimum_scaled_speed(commands.at(2));
                    planner.set_min_radius(commands.at(3));
                    planner.set_max_radius(commands.at(4));
                    planner.set_K_p_angle_to_goal(commands.at(5));
                    planner.set_K_p_offset_tangent(commands.at(6));
                    lidar.start();
                    break;
                default:
                    lidar.stop();
                    i2c_connection.steer(0);
                    i2c_connection.gas(0);
                    break;
            }
        }

        // Automatic: get lidar data
        if ( state == 2 )
        {
            // Fetch new LiDAR data
            lidar.update();

            //std::vector<Cluster> clusters {lidar.getPoints()};
            //saveClusters(clusters);

            std::vector<Cone> cones {lidar.getCones()};
            //saveCones(cones);

            // Find all valid gates
            std::vector<Gate> gates;
            gates = findGates(cones);
            //saveGates(gates);

            // Find previous and next gate
            std::pair<Gate,Gate> prev_next_gate;
            prev_next_gate = findPrevNextGate(gates);

            Gate prev_gate { prev_next_gate.first };
            Gate next_gate { prev_next_gate.second };

            //Calculation of time diffrence pid.
            T_c = (timestamp() - tlast) / 1000.f;
            tlast = timestamp();

            // Route planning and calculation of based on gate positions
            planner.update(prev_gate, next_gate, T_c);

            mqtt_connection.pubCones(cones);
            mqtt_connection.pubBezier(planner.getBezier_points());
            mqtt_connection.pubCurve(planner.getBezier_curve());
            mqtt_connection.pubLap(planner.getLap());

            float angle_to_steer = planner.getRefAngle();
            angle_to_steer = std::max(-1.f, std::min(angle_to_steer, 1.f));

            i2c_connection.steer(angle_to_steer);
            i2c_connection.gas(planner.getRefSpeed());
            int speed = i2c_connection.getSpeed();
            mqtt_connection.pubSpeed( T_c, speed );
        }
        else 
        {
            planner = Planner();
        }
    }
    return 0;
}

