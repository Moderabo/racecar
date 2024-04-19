#include "EKFslam.h"
#include <cmath>

EKFslamObj::EKFslamObj()
   : carLength {420.f}
{
    State = VectorXd(3);
    StateCovariance = MatrixXd(3,3);
    
    State << 0.f, 0.f, 0.f;

    StateCovariance << 1.f, 0.f, 0.f,
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

    //float x_dot       = Gas * cos(Steer + theta);
    //float y_dot       = Gas * sin(Steer + theta);;
    //float theta_dot   = Gas * sin(Steer) / carLength;

    //State(0) = x + x_dot * TimeStep;
    //State(1) = y + y_dot * TimeStep;
    //State(2) = theta + theta_dot * TimeStep; 

// New test predict
// Initiate Fx matrix
	MatrixXd Fx(3,State.rows());
	Fx(0,0) = 1;
	Fx(1,1) = 1;
	Fx(2,2) = 1;

	// Update state vector
	Eigen::Vector3d poseUpdate {{Gas * cos(Steer + theta),
						 Gas * sin(Steer + theta),
						 Gas * sin(Steer) / carLength}};

	State = State + Fx.transpose() * poseUpdate * TimeStep;

    // Update the covvariance matrix
	MatrixXd I(StateCovariance.rows(), StateCovariance.cols());
	I.setIdentity();

	MatrixXd jacob(3,3);
	jacob << 0.f, 0.f, - Gas * sin(Steer + theta) * TimeStep,
		     0.f, 0.f, Gas * cos(Steer + theta) * TimeStep,
			 0.f, 0.f, 0.f;
	MatrixXd Gt(I.rows(), I.cols());
	Gt = I + Fx.transpose() * jacob * Fx;
	
	// Covariance matrix R, choose probability of being correct
	MatrixXd R(3,3);
	R << 10, 0, 0,
		 0, 10, 0,
		 0, 0, 10;

	StateCovariance = Gt * StateCovariance * Gt.transpose() + Fx.transpose() * R * Fx;
	

    return 0;
}

// Do the correct step
int EKFslamObj::correct(std::vector<Cone> &observations)
{
	MatrixXd Qt(2,2);
	Qt << 10, 0,
		  0, 10;

	for (auto observation : observations)
	{
		// Association of previous landmarks
		// *** here ***
		if (2*j > State.rows() - 3)
		{
			State.conservativeResize(State.rows() + 2);
			StateCovariance.conservativeResize(State.rows() + 2, State.cols() + 2);

			State(State.rows() - 2) = cos(theta) * observation.x - cos(M_PI/2 - theta) * observation.y + State(0);
			State(State.rows() - 1) = cos(theta) * observation.x + sin(M_PI/2 - theta) * observation.y + State(1);

			StateCovariance(StateCovariance.rows() - 2, StateCovariance.cols() - 2) = 1000000;
			StateCovariance(StateCovariance.rows() - 1, StateCovariance.cols() - 1) = 1000000;
		}
	}
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
