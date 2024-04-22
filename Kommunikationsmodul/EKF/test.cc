#include <iostream>
#include <Eigen/Dense>
#include "EKFslam.h"

using Eigen::MatrixXd; 

// This file should be used to test functionality of the EKFslamObj class

int main()
{
  EKFslamObj slam;

  std::cout << slam.getPosition() << '\n';
  slam.predict(1,0,1);
  std::cout << slam.getPosition() << '\n';
  std::vector<Cone> observationer;
  Cone kon{1.f,1.f,10.f};
  observationer.push_back(kon);
  slam.correct(observationer);
	
  std::cout << '\n' << slam.StateCovariance << '\n';

}
