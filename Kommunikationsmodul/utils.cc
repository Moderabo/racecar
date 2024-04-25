#include <cmath>
#include <vector>
#include "utils.h"


AltGate convertGate(Gate &gate)
{
    float x0 { gate.first.x };
    float y0 { gate.first.y };
    float x1 { gate.second.x };
    float y1 { gate.second.y };
    // Vector from 0 to 1 cone
    float x01 { (x1 - x0) / sqrt((x1-x0)*(x1-x0)+(y1-y0)*(y1-y0)) };
    float y01 { (y1 - y0) / sqrt((x1-x0)*(x1-x0)+(y1-y0)*(y1-y0)) };

    AltGate alt_gate;
    // Calculate midpoint of the space between cones and not from cone midpoint
    alt_gate.x = ( x0 + x1 + x01 * (gate.first.r-gate.second.r) ) / 2;
    alt_gate.y = ( y0 + y1 + y01 * (gate.first.r-gate.second.r) ) / 2;
    
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
    float x = cone1.x - cone2.x;
    float y = cone1.y - cone2.y;
    // calculate the distance of the space between two cones 
    float gate_dist_squared { x*x + y*y - cone1.r*cone1.r - cone2.r*cone2.r };

    return gate_dist_squared < 750*750;
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
        else //if ( abs(atan2f(gate.y, gate.x)) < (60+10) * M_PI/180 )
        {
            if ( gate.x*gate.x + gate.y*gate.y < closest_next_gate )
            {
                prev_next_gate.second = gate;
                closest_next_gate = gate.x*gate.x + gate.y*gate.y;
            }
        }
    }
    // Check if a previous gate is found
    if ( closest_prev_gate > 1e29 )
    {
        // If no previous is found, add one behind of the car
        AltGate prev_gate {0, -1e3, 0};
        prev_next_gate.second = prev_gate;
    }
    // Check if a next gate is found
    if ( closest_next_gate > 1e29 )
    {
        // If no next is found, add one in front of the car
        AltGate next_gate {0, 1e3, 0};
        prev_next_gate.first = next_gate;
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
