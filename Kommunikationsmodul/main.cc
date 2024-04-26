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

    Planner bezier{};

    uint64_t t {0};
    uint64_t tstart {timestamp()};
    int state {0};
    std::vector<float> commands;

    // fetch result and print it out
    while ( true )
    {
	    if (ctrl_c_pressed)
        {
            break;
        }

        t = timestamp() - tstart;

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
                    sleep(0.05);
                    i2c_connection.gas(0);
                    sleep(0.01);
                    break;
                case 1:  // Manual
                    i2c_connection.steer(commands.at(1));
                    sleep(0.05);
                    i2c_connection.gas(commands.at(2));
                    sleep(0.01);
                    break;
                case 2:  // Automatic
                    lidar.start();
                    break;
                default:
                    lidar.stop();
                    i2c_connection.steer(0);
                    sleep(0.05);
                    i2c_connection.gas(0);
                    sleep(0.01);
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
            std::vector<AltGate> gates;
            gates = findGates(cones);
            //saveGates(gates);

            // Find previous and next gate
            std::pair<AltGate,AltGate> prev_next_gate;
            prev_next_gate = findPrevNextGate(gates);

            AltGate prev_gate { prev_next_gate.first };
            AltGate next_gate { prev_next_gate.second };

            // Route planning and calculation of
            Planner bezier {prev_gate.x, prev_gate.y, prev_gate.angle,
                            next_gate.x, next_gate.y ,next_gate.angle};

            mqtt_connection.pubCones(cones);
            mqtt_connection.pubBezier(bezier.getBezier_points());
            mqtt_connection.pubCurve(bezier.getBezier_curve());

            // Automatic: steer and gas
            if ( state == 2 )
            {
                float angle_to_steer = bezier.getRefAngle(0, 0, 0);
                angle_to_steer = std::max(-1.f, std::min(angle_to_steer, 1.f));
                float gas = std::max(-1.f, std::min(1.2f*bezier.getRefSpeed(), 1.f));

                i2c_connection.steer(angle_to_steer);
                sleep(0.05);
                i2c_connection.gas(bezier.getRefSpeed());
                sleep(0.01);
                int speed = i2c_connection.getSpeed();
                sleep(0.01);
                mqtt_connection.pubSpeed( t, speed );
            }
        }
    }
    return 0;
}

