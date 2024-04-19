#include <iostream>
#include <Eigen/Dense>
#include "ref_angle_test.h"
#include "planner_test.h"
//Controller test
using Eigen::MatrixXf;
int main()
{
    int  size = 20; 
    MatrixXf P(size,2);
    Planner bezier {0,0,1.5707,2000,2000,0.4000};
    P = bezier.update_P(size);

    Calc_ref angle_to_sterr(P, 2000,2000,0.4000);

    angle_to_sterr.update_ref(size,100,100, M_1_PI/5.0);
}
