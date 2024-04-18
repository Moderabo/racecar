#include <iostream>
#include <Eigen/Dense>
#include "EKFslam.h"

using Eigen::MatrixXd; 

// This file should be used to test functionality of the EKFslamObj class

int main()
{
  MatrixXd m(2,2);
  m(0,0) = 3;
  m(1,0) = 2.5;
  m(0,1) = -1;
  m(1,1) = m(1,0) + m(0,1);
  std::cout << m << std::endl;
}