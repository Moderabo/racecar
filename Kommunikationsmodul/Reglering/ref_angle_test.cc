#include <iostream>
#include <Eigen/Dense>
#include "Calc_ref.h"
#include "Planner.h"
//Controller test
using Eigen::MatrixXf;
int main()
{
    int  size = 20; 
    float angle;
    MatrixXf P(size,2);
    Planner bezier {0,0,1.5707,2000,2000,0.4000};

    angle = bezier.getRefAngle(400, 1200, 1.1);
}
