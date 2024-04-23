#ifndef EKFSLAM
#define EKFSLAM

// Use eigen for linear algebra
#include <Eigen/Dense>

#include "Lidar.h"

using Eigen::MatrixXf;
using Eigen::VectorXf;

//  A class that defines the entier slam instace
class EKFslamObj
{
    public:
    // Constructor
    EKFslamObj();

    // Destructor
    ~EKFslamObj() = default;

    // Call this funtion to update the current state based on input params
    int predict(float TimeStep, float Steer, float Gas);

    // Call this function with observations to update the pose and covariance
    int correct(std::vector<Cone> &observations);

    // Return the current state
    VectorXf getState();

    std::vector<Cone> getCones();

    // Return the position of the veichle
    Eigen::Vector3f getPosition();

    // The vector containing the position of the current state
    VectorXf State;
    
    // The matrix containing the covvariance for all the objects
    MatrixXf StateCovariance;

    // The lenght of the car in mm (420 from vanheden)
    float carLength;

    private:
};

#endif
