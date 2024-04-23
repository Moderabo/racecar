#include "EKFslam.h"
#include <cmath>

EKFslamObj::EKFslamObj()
   : carLength {420.f}
{
    State = VectorXf(3);
    StateCovariance = MatrixXf(3,3);
    
    State << 1.f, 1.f, 0.f;

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
	MatrixXf Fx(3,State.rows());
	Fx(0,0) = 1;
	Fx(1,1) = 1;
	Fx(2,2) = 1;

	// Update state vector
	Eigen::Vector3f poseUpdate {{Gas * cos(Steer + theta),
						 Gas * sin(Steer + theta),
						 Gas * sin(Steer) / carLength}};

    std::cout << "Fx.trans\t" << Fx.transpose() << std::endl;
	State = State + Fx.transpose() * poseUpdate * TimeStep;
    std::cout << "State\t" << State(0) << "\t" << State(1) << std::endl;

    // Update the covvariance matrix
	MatrixXf I(StateCovariance.rows(), StateCovariance.cols());
	I.setIdentity();

	MatrixXf jacob(3,3);
	jacob << 0.f, 0.f, - Gas * sin(Steer + theta) * TimeStep,
		     0.f, 0.f, Gas * cos(Steer + theta) * TimeStep,
			 0.f, 0.f, 0.f;
	MatrixXf Gt(I.rows(), I.cols());
	Gt = I + Fx.transpose() * jacob * Fx;

	// Covariance matrix R, choose probability of being correct
	MatrixXf R(3,3);
	R << 10, 0, 0,
		 0, 10, 0,
		 0, 0, 10;

	StateCovariance = Gt * StateCovariance * Gt.transpose() + Fx.transpose() * R * Fx;


    return 0;
}

// Do the correct step
int EKFslamObj::correct(std::vector<Cone> &observations)
{
	float theta = State(2);

	MatrixXf Qt(2,2);
	Qt << 10, 0,
		  0, 10;


	// The vector that associate the observation i, with the row j in the state matrix
	// i.e. associate(i) = j; State(j) = "x_coordinate"; State(j+1) = "y_coordinate"
	std::vector<int> association;

	// Associate every observation with a known landmark
	// loop over all the given observations and give them a number that know landmark

    Eigen::VectorXf tempState = State;
    Eigen::MatrixXf tempStateCovariance = StateCovariance;


	for (auto observation : observations)
	{
		// calculate where the object is in the static coordinate plane
		float x = cos(theta) * observation.x - cos(M_PI/2 - theta) * observation.y + State(0);
		float y = cos(theta) * observation.x + sin(M_PI/2 - theta) * observation.y + State(1);

		// the previous minimum distace
		float min_distance = 1e10;
		// the association nr
		int j = 0;

		// loop over all the previously know landmarks and associate an object to a know landmark

		for (int i = 3; i < State.rows() ;i+=2)
		{
			// calculate the distance to the gate
			float dist = pow(pow(x-State(i),2)+pow(y-State(i+1),2),0.5f);
			// if better than previous
			if (dist <= min_distance)
			{
				// new minimum distance
				min_distance = dist;

				// the corresponding row
				j = i;
			}
		}

		// if the closest know landmark isn't within 200mm we have a new landmark
		if (min_distance > 200.f)
		{

			// increase the size of the size state representation
			tempState.conservativeResize(tempState.rows() + 2);
			tempStateCovariance.conservativeResize(tempState.rows(), tempState.rows());

			// set the position to the measured position
			tempState(tempState.rows() - 2) = x;
			tempState(tempState.rows() - 1) = y;

			// with only one measurment the covvariance is big
			tempStateCovariance(tempStateCovariance.rows() - 2, tempStateCovariance.cols() - 2) = 1e100;
			tempStateCovariance(tempStateCovariance.rows() - 1, tempStateCovariance.cols() - 1) = 1e100;


			// the number of the observation should be the new last land mark
			j = tempState.rows() - 2;
		}

		// put the new know association into the association vector
		association.push_back(j);
	}
    State = tempState;
    StateCovariance = tempStateCovariance;

	int obs_nr {0};
	// loop over all the observations and do the ekfSLAM stuff on them now
	for (auto observation : observations)
	{
		// do some stupid stuff i guess

		// distance between car and object
		Eigen::Vector2f delta;
		delta << State(association[obs_nr]) - State(0), 
				 State(association[obs_nr]+1) - State(1);
		float q = pow(delta.coeff(0),2)+ pow(delta.coeff(1),2);

		// the observation i guess
		Eigen::Vector2f z;
		z<< pow(pow(observation.x,2)+pow(observation.y,2),0.5f), atan2(observation.y,observation.x);

		Eigen::Vector2f z_hat;
		z_hat<< pow(q,0.5f), atan2(delta.coeff(1),delta.coeff(0))- State(2);

		// 5xN matrix with zeros
		MatrixXf F(5,State.rows());
		F << MatrixXf::Identity(3,3), MatrixXf::Zero(3,State.rows()-3),
		MatrixXf::Zero(2,State.rows());

		F.block(3,association[obs_nr],4,association[obs_nr]+1) << 1,0,0,1;

		MatrixXf H(2,5);

		H << -pow(q,0.5f)*delta.coeff(0), -pow(q,0.5f)*delta.coeff(1), 0, pow(q,0.5f)*delta.coeff(0), pow(q,0.5f)*delta.coeff(1),
			 delta.coeff(1), -delta.coeff(0), -q, -delta.coeff(1), delta.coeff(0);

		H = (1/q) * H * F;

		MatrixXf K(State.rows(),2);
        MatrixXf Itwo(2,2);
        Itwo.setIdentity();
        MatrixXf inv(2, 2);
        inv = (H*StateCovariance * H.transpose() + Qt).householderQr().solve(Itwo);
		//K = StateCovariance * H.transpose()*(H*StateCovariance * H.transpose() + Qt).inverse();
        K = StateCovariance * H.transpose()*inv;
		State = State + K*(z - z_hat);

		MatrixXf I(StateCovariance.rows(),StateCovariance.rows());
		I.setIdentity();
		StateCovariance = (I - K*H)*StateCovariance;

		obs_nr ++;
	}

    return 0;
}

// Get the state vector
VectorXf EKFslamObj::getState()
{
    // Return the entire state vector
    return State;
}

// Get the position of car
Eigen::Vector3f EKFslamObj::getPosition()
{
    // Return the top three elements of the state vector, i.e. the position of the car
    return State.block(0,0,3,1);
}

std::vector<Cone> EKFslamObj::getCones()
{
	std::vector<Cone> Cones;
	for(int i=3; i > State.rows(); i+=2)
	{
		Cone currentCone{State(i),State(i+1),100.f};
		Cones.push_back(currentCone);
	}

	return Cones;
}
