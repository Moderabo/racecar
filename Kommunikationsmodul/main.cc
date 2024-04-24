/*
 *  RPi main script
 *
 */
#include <Eigen/Dense>
#include <iostream>

#include "I2C_Connection.h"
#include "Lidar.h"
#include "MQTT_Connection.h"
#include "Planner.h"

using namespace std;
using namespace Eigen;

const string SERVER_ADDRESS	{ "10.42.0.1" };
const string CLIENT_ID		{ "CGgggga666" };
const string TOPIC 		{ "commands" };

const int  QOS = 1;

typedef std::pair<Cone,Cone> Gate;

struct AltGate
{
    float x;
    float y;
    float angle;
};

bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}


AltGate convertGate(Gate &gate)
{
    float x0 { gate.first.x };
    float y0 { gate.first.y };
    float x1 { gate.second.x };
    float y1 { gate.second.y };

    AltGate alt_gate;

    alt_gate.x = (x1 + x0) / 2;
    alt_gate.y = (y1 + y0) / 2;
    //alt_gate.angle = acos( abs(y1-y0) / sqrt( (x1-x0)*(x1-x0)+(y1-y0)*(y1-y0) ) );
    float theta = atan2f(y1-y0,x1-x0);

    if ( theta < -M_PI / 2 )
    {
        alt_gate.angle = M_PI / 2 + theta;
    }
    else if ( theta < 0 )
    {
        alt_gate.angle = M_PI / 2 + theta;
    }
    else if ( theta < M_PI / 2 )
    {
        alt_gate.angle = -M_PI / 2 + theta;
    }
    else
    {
        alt_gate.angle = -M_PI / 2 + theta;
    }

    return alt_gate;
}


bool validGate(Cone &cone1, Cone &cone2)
{
    std::pair<float,float> gate_vect;
    gate_vect.first = cone1.x - cone2.x;
    gate_vect.second = cone1.y - cone2.y;

    float gate_dist_squared {gate_vect.first*gate_vect.first + gate_vect.second*gate_vect.second};

    if ( gate_dist_squared > 750*750)
    {
        return false;
    }
    else
    {
        return true;
    }
}


std::vector<AltGate> findGates(std::vector<Cone> &cones)
{
    std::vector<AltGate> gates {};

    for (auto cone1 = cones.begin(); cone1 != cones.end(); cone1++)
    {
        for (auto cone2 {cone1 + 1}; cone2 != cones.end(); cone2++)
        {
            if ( validGate(*cone1, *cone2) )
            {
                Gate gate {*cone1, *cone2};
                gates.push_back(convertGate(gate));
            }
        }
    }

    return gates;
}


std::pair<AltGate,AltGate> findPrevNextGate(std::vector<AltGate> &gates)
{
    std::pair<AltGate,AltGate> prev_next_gate {};
    float closest_prev_gate { 1e30 };
    float closest_next_gate { 1e30 };

    for (auto &gate : gates)
    {
        // Previous gate
        if ( gate.x < 0 )
        {
            if ( gate.x*gate.x + gate.y*gate.y < closest_prev_gate )
            {
                prev_next_gate.first = gate;
                closest_prev_gate = gate.x*gate.x + gate.y*gate.y;
            }
        }
        // Next gate
        else if ( abs(atan2f(gate.y, gate.x)) < 60 )
        {
            if ( gate.x*gate.x + gate.y*gate.y < closest_next_gate )
            {
                prev_next_gate.second = gate;
                closest_next_gate = gate.x*gate.x + gate.y*gate.y;
            }
        }
    }
    
    return prev_next_gate;
}


void saveClusters(std::vector<Cluster> &clusters)
{
	// Save clusters to file
	std::ofstream datafile;
	datafile.open("data.csv");

	// Enumerate clusters
	int object {0};

	for (auto &cluster : clusters)
	{
		for (auto &point : cluster)
        {
            // write data to disc
            datafile  << point.x << ','
                      << point.y << ','
                      << object << '\n';
		}
		object++;
	}

	datafile.close();
}


void saveCones(std::vector<Cone> &cones)
{
	std::ofstream conesfile;
	conesfile.open("cones.csv");

	for (auto &cone : cones)
	{
		conesfile << cone.x << ','
                  << cone.y << ','
                  << cone.r << '\n';
	}

	conesfile.close();
}


void saveGates(std::vector<Gate> &gates)
{
	std::ofstream file;
	file.open("gates.csv");

	for (auto &gate : gates)
	{
		file << gate.first.x  << ','
             << gate.first.y  << ','
             << gate.first.r  << ','
             << gate.second.x << ','
             << gate.second.y << ','
             << gate.second.r << ',' << '\n';
	}

	file.close();
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

    // fetch result and print it out, do it 10 times
    for(int i = 0; i < 10000; ++i)
    {
        lidar.update();

        //std::vector<Cluster> clusters {lidar.getPoints()};
    	//saveClusters(clusters);

        std::vector<Cone> cones {lidar.getCones()};
	    //saveCones(cones);

        std::vector<AltGate> gates;
        gates = findGates(cones);
        //saveGates(gates);

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

        Planner bezier {prev_gate.x, prev_gate.y, prev_gate.angle,
                        next_gate.x, next_gate.y ,next_gate.angle};

        float angle_to_steer = bezier.getRefAngle(0, 0, 0);
        
        //float angle_to_steer {3*atan2f(next_gate.y, next_gate.x)};
        std::cout << "trying angle: " << angle_to_steer << std::endl;

    	angle_to_steer = std::max(-1.f, std::min(angle_to_steer, 1.f));

	    mqtt_connection.pubCones(cones);

        mqtt_connection.pubBezier(bezier.getBezier_points());
        mqtt_connection.pubCurve(bezier.getBezier_curve());

        i2c_connection.steer(angle_to_steer);
    	sleep(0.1);
    	i2c_connection.gas(0);
    	sleep(0.1);
        int g = i2c_connection.getSpeed();
        if (ctrl_c_pressed){
            break;
        }
    }
    return 0;
}

