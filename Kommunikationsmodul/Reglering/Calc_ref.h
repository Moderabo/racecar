#ifndef PURSUIT_H_
#define PURSUIT_H_
#include "PID.h"

class Calc_ref{
    public:
    Calc_ref() = default;
    Calc_ref(Eigen::MatrixXf P, Eigen::MatrixXf K,
     float x_goal, float y_goal, float goal_angle, float K_p_angle_to_goal = 0.7,
    float K_p_offset_tangent = 0.1): 
    P{P}, K{K}, x_goal{x_goal}, y_goal{y_goal}, goal_angle{goal_angle}, 
    pid_c{0.5, {0.87154,6.84371,0,100,1,1}}, K_p_angle_to_goal{K_p_angle_to_goal},
    K_p_offset_tangent{K_p_offset_tangent}, look_ahead_dist{2}
    {
    }
    virtual ~Calc_ref()
    {}

    float update_ref(int size)
    {
        float angle_from_tangent;
        float angle_to_goal;

        Eigen::VectorXf d_vec(size);

        // here we should loop over all the points in the P vector (size + extra after gate)
        for(int n=0; n < P.rows(); n++ ){

            float  distance = pow(pow(P.coeff(n,0),2) + pow(P.coeff(n,1),2), 0.5);
            d_vec.row(n) << distance;
        }

        // calculate minimum distance to the line and get the corresponding index
        float XTE = d_vec.minCoeff(&index);

        // calculate the angle to the look ahead point
        angle_to_goal = atan2f(P.coeff(index + look_ahead_dist,1),P.coeff(index + look_ahead_dist,0));

        // calculate the angle to the tangent
        float x = P.coeff(index + 1,0) - P.coeff(index,0);
        float y = P.coeff(index + 1,1) - P.coeff(index,1);
        angle_from_tangent = atan2f(y,x);

        CTS = angle_from_tangent;

        // get the angle we should turn
        refrence_angle = K_p_angle_to_goal * angle_to_goal + K_p_offset_tangent * CTS;

        //returns something normally between pi/9 scaled to [0,1] and if angle is bigger its capped later in main.
        return refrence_angle*9/(3.14); //*pid_c.update(refrence_angle, car_angle)
    
    }

    void set_K_p_angle_to_goal(float fraction)
    {
        K_p_angle_to_goal = fraction;
    }

    void set_K_p_offset_tangent(float fraction)
    {
        K_p_offset_tangent = fraction;
    }

    float get_refrence_angle()
    {
        return refrence_angle;
    }

    float get_XTE()
    {
        return XTE;
    }

    float get_CTS()
    {
        return CTS;
    }

    float get_scaled_Speed()
    {
        return K.coeff(index);
    }

    private:
     
    float XTE;
    float CTS;
    float refrence_angle;
    float angle_to_goal;
    int look_ahead_dist;

    //Waypoints
    Eigen::MatrixXf P;
    Eigen::MatrixXf K;
    float x_goal; 
    float y_goal; 
    float goal_angle;
    PIDController pid_c; //currently  trash

    Eigen::Index index;
    float K_p_angle_to_goal;
    float K_p_offset_tangent;

};

#endif /* PURSUIT_H_ */
