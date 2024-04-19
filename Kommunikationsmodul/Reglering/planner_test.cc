#include <iostream>
#include <Eigen/Dense>
#include "planner_test.h"
//Planner test
using Eigen::MatrixXf;
int main()
{
    int  size = 20; 
    MatrixXf P(size,2);
    Planner bezier {0,0,1.5707,2000,2000,0.4000};
    P = bezier.update_P(size);
    std::cout << P << std::endl;
}