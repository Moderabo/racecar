/*
 *  RPi main script
 *
 */
#include <Eigen/Dense>
#include <iostream>

#include "I2C_Connection.h"
#include "Lidar.h"
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


bool validGate(Cone &cone1, Cone &cone2)
{
    std::pair<float,float> gate_vect;
    gate_vect.first = cone1.x - cone2.x;
    gate_vect.second = cone1.y - cone2.y;

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


std::pair<Gate,Gate> findPrevNextGate(std::vector<Gate> &gates)
{
    std::pair<Gate,Gate> prev_next_gate {};

    // Previous gate
    for (auto &gate : gates)
    {
        if ( gate.first.x < 0 && gate.second.x < 0 )
        {
            prev_next_gate.first = gate;
            break;
        }
    }
    
    // Next gate
    for (auto &gate : gates)
    {
        if ( gate.first.x > 0 && gate.second.x > 0 )
        {
            prev_next_gate.second = gate;
            break;
        }
    }
    
    return prev_next_gate;
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
    alt_gate.angle = acos( (x1-x0)/ sqrt((x1-x0)*(x1-x0)+(y1-y0)*(y1-y0)) );
    
    return alt_gate;
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

    // Initiate LiDAR
    Lidar lidar {};

    // fetch result and print it out, do it 10 times
    for(int i = 0; i < 1000; ++i)
    {
        lidar.update();

        std::vector<Cluster> clusters {lidar.getPoints()};
    	saveClusters(clusters);
    
	    std::vector<Cone> cones {lidar.getCones()};
	    saveCones(cones);

        std::vector<Gate> gates;
        gates = findGates(cones);

        saveGates(gates);

        std::pair<Gate,Gate> prev_next_gate;
        prev_next_gate = findPrevNextGate(gates);

        AltGate prev_gate { convertGate(prev_next_gate.first) };
        AltGate next_gate { convertGate(prev_next_gate.second) };
        
        Planner bezier {prev_gate.x, prev_gate.y, prev_gate.angle,
                        next_gate.x, next_gate.y ,next_gate.angle};

        float angle = bezier.getRefAngle(0, 0, 0);
        
        float angle_to_steer = angle;
        std::cout << "trying angle: " << angle_to_steer << std::endl;
	
    	angle_to_steer = std::max(-1.f, std::min(angle_to_steer, 1.f));
    
        i2c_connection.steer(angle_to_steer);
    	sleep(0.1);
    	i2c_connection.gas(0.1);
    	sleep(0.1);

        if (ctrl_c_pressed){ 
            break;
        }

    }

    return 0;
}

