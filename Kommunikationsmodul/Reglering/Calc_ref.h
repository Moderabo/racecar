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
    K_p_offset_tangent{K_p_offset_tangent}
    {
    }
    virtual ~Calc_ref()
    {}

    float update_ref(int size,float car_x, float car_y, float car_angle)
    {
        float angle_from_tangent;
        float angle_to_goal;

        Eigen::VectorXf d_vec(size);

        for(int n=0; n <= size-1; n++ ){

            float  distance = pow(pow(car_x-P.coeff(n,0),2) + pow(car_y-P.coeff(n,1),2), 0.5);
            d_vec.row(n) << distance;
        }

        float XTE = d_vec.minCoeff(&index); //minimum distance to ref line, could be optimiced..

        Eigen::MatrixXf rot_M(2,2); //inverse of a rotation matrix in cars angle. 
        //Make it independent on cordinate system.
        rot_M.row(0) << cos(-car_angle), sin(-car_angle);
        rot_M.row(1) << -sin(-car_angle), cos(-car_angle);

        Eigen::MatrixXf cords1(1,2);
        cords1.row(0) << (P.coeff(index + size/10,0) - car_x), (P.coeff(index + size/10,1) - car_y);
        cords1 = cords1 * rot_M; //1x2 matrix
        angle_to_goal = angle(cords1.coeff(0,1),cords1.coeff(0,0));

        Eigen::MatrixXf cords2(1,2);
        cords2.row(0) << (P.coeff(index + size/10,0) - P.coeff(index,0)), (P.coeff(index + size/10,1) - P.coeff(index,1));
        cords2 = cords2 * rot_M; //1x2 matrix
        angle_from_tangent = angle(cords2.coeff(0,1),cords2.coeff(0,0));

        CTS = angle_from_tangent;

        refrence_angle = K_p_angle_to_goal * angle_to_goal + K_p_offset_tangent * CTS + car_angle;

        return refrence_angle*9/(3.14); //*pid_c.update(refrence_angle, car_angle)
        //returns something normally between pi/9 scaled to [0,1] and if angle is bigger its capped later in main.
    }

    void set_K_p_angle_to_goal(float fraction)
    {
        K_p_angle_to_goal = fraction;
    }

    void set_K_p_offset_tangent(float fraction)
    {
        K_p_offset_tangent = fraction;
    }

    float angle(float y, float x){     
        return atan2f(y,x);
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
