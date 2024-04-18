/*
 *  RPi main script
 *
 */

#include "I2C_Connection.h"
#include "Lidar.h"

using namespace std;

const string SERVER_ADDRESS	{ "10.42.0.1" };
const string CLIENT_ID		{ "CGgggga666" };
const string TOPIC 		{ "commands" };

const int  QOS = 1;

typedef std::pair<Cone,Cone> Gate;

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


Gate findNextGate(std::vector<Gate> &gates)
{
    for (auto &gate : gates)
    {
        if ( gate.first.x > 0 && gate.second.x > 0 )
        {
           return gate;
        }
    }
    
    Gate no_gate {};
    return no_gate;
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

        Gate next_gate;
        next_gate = findNextGate(gates);

        Point next_gate_midpoint;
        next_gate_midpoint.x = (next_gate.first.x + next_gate.second.x) / 2;
        next_gate_midpoint.y = (next_gate.first.y + next_gate.second.y) / 2;

        //std::cout << next_gate_midpoint.x << ','
        //          << next_gate_midpoint.y << std::endl;

        float angle_to_steer = atan2(next_gate_midpoint.y,next_gate_midpoint.x)*3;
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

    return 0;
}

