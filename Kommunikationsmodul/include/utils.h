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

typedef std::pair<Cone,Cone> ConePair;

struct Gate
{
    float x;
    float y;
    float angle;
    int type;  // 0: ordinairy, -1: right, 1:left, 2: start, -2: imaginary
};

Gate convertGate(ConePair &);

bool validGate(Cone &cone1, Cone &cone2);

bool isInGate(Gate prev);

std::vector<Gate> findGates(std::vector<Cone> &cones);


std::pair<Gate,Gate> findPrevNextGate(std::vector<Gate> &gates);


void saveClusters(std::vector<Cluster> &clusters);


void saveCones(std::vector<Cone> &cones);


void saveGates(std::vector<Gate> &);

#endif

