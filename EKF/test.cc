#include <iostream>
#include <Eigen/Dense>
#include "EKFslam.h"

using Eigen::MatrixXd; 

// This file should be used to test functionality of the EKFslamObj class

int main()
{
  EKFslamObj slam;

  std::cout << slam.getPosition() << '\n';
  slam.predict(1,1,1000);
  std::cout << slam.getPosition() << '\n';

}