#include "EKFslam.h"
#include <cmath>

EKFslamObj::EKFslamObj()
   : carLength {420.f}
{
    State = VectorXd(3);
    StateCovariance = MatrixXd(3,3);
    
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
	float theta = State(2);

	MatrixXd Qt(2,2);
	Qt << 10, 0,
		  0, 10;

	
	// The vector that associate the observation i, with the row j in the state matrix
	// i.e. associate(i) = j; State(j) = "x_coordinate"; State(j+1) = "y_coordinate"
	std::vector<int> association;

	// Associate every observation with a known landmark  
	// loop over all the given observations and give them a number that know landmark
 
	for (auto observation : observations)
	{
		// calculate where the object is in the static coordinate plane
		float x = cos(theta) * observation.x - cos(M_PI/2 - theta) * observation.y + State(0);
		float y = cos(theta) * observation.x + sin(M_PI/2 - theta) * observation.y + State(1);
		std::cout << "x: " << x << "\ty: " << y << "\tobs x: " << observation.x << "\tobs y: " 
				  << observation.y << "\ttheta: " << theta << "\tstate 0: " 
				  << State(0) << "\tstate 1: " << State(1) <<std::endl;
		
		// the previous minimum distace
		float min_distance = 1e10;
		// the association nr
		int j = 0;

		// loop over all the previously know landmarks and associate an object to a know landmark

		for (int i = 3; i < State.rows() ;i+=2)
		{
			//std::cout << "loopa loopade har loopat" << std::endl;
			// calculate the distance to the gate
			float dist = pow(pow(x-State(i),2)+pow(y-State(i+1),2),0.5f);
			//std::cout << x << ' ' << y << ' ' << State(i) << ' ' << State(i+1) << ' ' << dist << std::endl;
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
			//std::cout << "New Landmark: " << min_distance << std::endl;

			// increase the size of the size state representation
			State.conservativeResize(State.rows() + 2);
			StateCovariance.conservativeResize(State.rows(), State.rows());

			// set the position to the measured position
			State(State.rows() - 2) = x;
			State(State.rows() - 1) = y;

			// with only one measurment the covvariance is big
			StateCovariance(StateCovariance.rows() - 2, StateCovariance.cols() - 2) = 1e100;
			StateCovariance(StateCovariance.rows() - 1, StateCovariance.cols() - 1) = 1e100;


			// the number of the observation should new last land mark
			j = State.rows() - 2;
		}

		// put the new know association into the association vector
		association.push_back(j);
	}


	// loop over all the observations and do the ekfSLAM stuff on them now
	for (auto observation : observations)
	{
		int obs_nr = 0;
		// do some stupid stuff i guess

		// distance between car and object
		Eigen::Vector2d delta;
		delta << State(association[obs_nr]) - State(0), 
				 State(association[obs_nr]+1) - State(1);
		float q = pow(delta(0),2)+ pow(delta(1),2);

		// the observation i guess
		Eigen::Vector2d z;
		z<< pow(pow(observation.x,2)+pow(observation.y,2),0.5f), atan2(observation.y,observation.x);

		Eigen::Vector2d z_hat;
		z_hat<< pow(q,0.5f), atan2(delta(1),delta(0))- State(2);

		// 5xN matrix with zeros
		MatrixXd F(5,State.rows());
		F(0,0) = 1.f;
		F(1,1) = 1.f;
		F(2,2) = 1.f;
		F(3,obs_nr*2+3) = 1.f;
		F(4,obs_nr*2+4) = 1.f;

		MatrixXd H(2,5);

		H << -pow(q,0.5f)*delta(0), -pow(q,0.5f)*delta(1), 0, pow(q,0.5f)*delta(0), pow(q,0.5f)*delta(1),
			 delta(1), -delta(0), -q, -delta(1), delta(0);  

		H = 1/q * H * F;
		std::cout << "H:\n" << H << std::endl;

		MatrixXd K(State.rows(),2);

		K = StateCovariance * H.transpose()*(H*StateCovariance * H.transpose() + Qt).inverse();
		std::cout << "K:\t" << K << '\n'
			  << "z:\t" << z << '\n'
			  << "zhat:\t" << z_hat << std::endl;
		State = State + K*(z - z_hat);

		MatrixXd I(StateCovariance.rows(),StateCovariance.rows());
		I.setIdentity();
		StateCovariance = (I - K*H)*StateCovariance;
		
		obs_nr ++;
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
