#ifndef UTILS_H
#define UTILS_H

#include <cmath>
#include <fstream>
#include <vector>


struct Point
{
    float x;
    float y;
};

struct Cone
{
    float x;
    float y;
    float r;
};

typedef std::vector<Point> Cluster;

typedef std::pair<Cone,Cone> Gate;

struct AltGate
{
    float x;
    float y;
    float angle;
};


AltGate convertGate(Gate &gate);


bool validGate(Cone &cone1, Cone &cone2);


std::vector<AltGate> findGates(std::vector<Cone> &cones);


std::pair<AltGate,AltGate> findPrevNextGate(std::vector<AltGate> &gates);


void saveClusters(std::vector<Cluster> &clusters);


void saveCones(std::vector<Cone> &cones);


void saveGates(std::vector<Gate> &gates);

#endif

