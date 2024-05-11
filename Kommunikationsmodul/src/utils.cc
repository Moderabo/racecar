#include <cmath>
#include <vector>
#include "utils.h"

#include <iostream>

Gate convertGate(ConePair &cone_pair)
{
    float x0 { cone_pair.first.x };
    float y0 { cone_pair.first.y };
    float x1 { cone_pair.second.x };
    float y1 { cone_pair.second.y };
    // Vector from 0 to 1 cone
    float x01 { (x1 - x0) / sqrtf((x1-x0)*(x1-x0)+(y1-y0)*(y1-y0)) };
    float y01 { (y1 - y0) / sqrtf((x1-x0)*(x1-x0)+(y1-y0)*(y1-y0)) };

    Gate gate;
    // Calculate midpoint of the space between cones and not from cone midpoint
    gate.x = ( x0 + x1 + x01 * (cone_pair.first.r-cone_pair.second.r) ) / 2;
    gate.y = ( y0 + y1 + y01 * (cone_pair.first.r-cone_pair.second.r) ) / 2;
    
    float theta = atan2f(y1-y0,x1-x0);

    if ( theta < -M_PI / 2 )
    {
        gate.angle = M_PI / 2 + theta;
    }
    else if ( theta < 0 )
    {
        gate.angle = M_PI / 2 + theta;
    }
    else if ( theta < M_PI / 2 )
    {
        gate.angle = -M_PI / 2 + theta;
    }
    else
    {
        gate.angle = -M_PI / 2 + theta;
    }
    
    // Find type
    if ( cone_pair.first.r == 120.f/2 && cone_pair.second.r == 120.f/2 )
    {
        gate.type = 0;
    }
    else if ( cone_pair.first.r == 190.f/2 && cone_pair.second.r == 190.f/2 )
    {
        gate.type = 2;
    }
    else
    {
        float xgc { cone_pair.first.x - gate.x };
        float ygc { cone_pair.first.y - gate.y };
        float xg { gate.y };
        float yg { -gate.x };
        bool left_cone { xgc * xg + ygc * yg > 0 };

        //std::cout << left_cone << std::endl;
        if ( left_cone && (cone_pair.first.r == 120.f/2) )
        {
            gate.type = -1;
        }
        else
        {
            gate.type = 1;
        }
    }
    
    return gate;
}


bool validGate(Cone &cone1, Cone &cone2)
{
    float x = cone1.x - cone2.x;
    float y = cone1.y - cone2.y;
    // calculate the distance of the space between two cones 
    float gate_dist_squared { x*x + y*y - cone1.r*cone1.r - cone2.r*cone2.r };

    return gate_dist_squared < 900*900;
}


bool isInGate(Gate gate)
{
    return gate.x*gate.x+gate.y*gate.y < 350*350;  
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
                ConePair cone_pair {*cone1, *cone2};
                gates.push_back(convertGate(cone_pair));
            }
        }
    }

    return gates;
}


std::pair<Gate,Gate> findPrevNextGate(std::vector<Gate> &gates)
{
    std::pair<Gate,Gate> prev_next_gate {};
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
    if ( closest_prev_gate > 2000*2000 )
    {
        // If no previous is found, add one behind of the car
        Gate prev_gate {-1e3, 0, 0};
        prev_next_gate.first = prev_gate;
    }
    // Check if a next gate is found
    if ( closest_next_gate > 2000*2000 )
    {
        // depending on what the previous gate was we imagine a new gate at different places
        switch (prev_next_gate.first.type)
        {
        
        // if the prev gate was a right turn
        case -1:
        {
            //std::cout << "case -1" << std::endl;
            Gate next_gate {1e3, 1e3, 0.79f};
            prev_next_gate.second = next_gate;
            break;
        }

        // if the prev gate was a left turn 
        case  1:
        {
            //std::cout << "case 1" << std::endl;
            Gate next_gate {1e3, -1e3, -0.79f};
            prev_next_gate.second = next_gate;
            break;
        }

        default:
        {
            // If no next is found, add one in front of the car
            Gate next_gate {1e3, 0, 0};
            prev_next_gate.second = next_gate;
            break;
        }

        }

    }
    // Change angle if needed (boundary condition)
    float gate_diff_x = prev_next_gate.second.x - prev_next_gate.first.x;
    float gate_diff_y = prev_next_gate.second.y - prev_next_gate.first.y;

    float gate_diff_angle = atan2f(gate_diff_y, gate_diff_x);

    if ( cos( gate_diff_angle - prev_next_gate.first.angle) < 0 )
    {
        prev_next_gate.first.angle = prev_next_gate.first.angle + M_PI;
    }

    if ( cos( gate_diff_angle - prev_next_gate.second.angle) < 0 )
    {
        prev_next_gate.second.angle = prev_next_gate.second.angle + M_PI;
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


void saveGates(std::vector<ConePair> &cone_pairs)
{
	std::ofstream file;
	file.open("gates.csv");

	for (auto &cone_pair : cone_pairs)
	{
		file << cone_pair.first.x  << ','
             << cone_pair.first.y  << ','
             << cone_pair.first.r  << ','
             << cone_pair.second.x << ','
             << cone_pair.second.y << ','
             << cone_pair.second.r << ',' << '\n';
	}

	file.close();
}
