#include "EKFslam.h"
#include <cmath>

EKFslamObj::EKFslamObj()
   : carLength {420.f}
{
    State = VectorXd(3);
    StateCovarriance = MatrixXd(3,3);
    
    State << 0.f, 0.f, 0.f;

    StateCovarriance << 1.f, 0.f, 0.f,
                        0.f, 1.f, 0.f,
                        0.f, 0.f, 1.f;

    return;
}

// Do the predict step
int EKFslamObj::predict(float TimeStep, float Steer, float Gas)
{
    // Update the state vector
    float x = State(0);
    float y = State(1);
    float theta = State(2);

    float x_dot       = Gas * cos(Steer + theta);
    float y_dot       = Gas * sin(Steer + theta);;
    float theta_dot   = Gas * sin(Steer) / carLength;

    State(0) = x + x_dot * TimeStep;
    State(1) = y + y_dot * TimeStep;
    State(2) = theta + theta_dot * TimeStep; 

    // Update the covvariance matrix
    

    return 0;
}

// Do the correct step
int EKFslamObj::correct()
{
    return 0;
}

// Get the state vector
VectorXd EKFslamObj::getState()
{
    // Return the entire state vector
    return State;
}

// Get the position of car
Eigen::Vector3d EKFslamObj::getPosition()
{
    // Return the top three elements of the state vector, i.e. the position of the car
    return State.block(0,0,3,1);
}