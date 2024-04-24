#ifndef PURSUIT_H_
#define PURSUIT_H_
#include "PID.h"

class Calc_ref{
    public:
    Calc_ref() = default;
    Calc_ref(Eigen::MatrixXf P,
     float x_goal, float y_goal, float goal_angle): 
    P{P},x_goal{x_goal}, y_goal{y_goal}, goal_angle{goal_angle}, pid_c{0.5, {0.87154,6.84371,0,100,1,1}}
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

        Eigen::Index index;
        float XTE = d_vec.minCoeff(&index); //minimum distance to ref line, could be optimiced..

        Eigen::MatrixXf rot_M(2,2); //inverse of a rotation matrix in cars angle. 
        //Make it independent on cordinate system.
        rot_M.row(0) << cos(-car_angle), sin(-car_angle);
        rot_M.row(1) << -sin(-car_angle), cos(-car_angle);

        if(index + size/10 <= size){

            Eigen::MatrixXf cords1(1,2);
            cords1.row(0) << (P.coeff(index + size/10,0) - car_x), (P.coeff(index + size/10,1) - car_y);
            cords1 = cords1 * rot_M; //1x2 matrix
            angle_to_goal = angle(cords1.coeff(0,1),cords1.coeff(0,0));


            Eigen::MatrixXf cords2(1,2);
            cords2.row(0) << (P.coeff(index + size/10,0) - P.coeff(index,0)), (P.coeff(index + size/10,1) - P.coeff(index,1));
            cords2 = cords2 * rot_M; //1x2 matrix
            angle_from_tangent = angle(cords2.coeff(0,1),cords2.coeff(0,0));

        }else if (index + size/10 > size) //If car are close to the goal set new goal point further away.
        {
            Eigen::MatrixXf cords1(1,2);
            cords1.row(0) << (x_goal + 500*cos(goal_angle) - car_x), (x_goal + 500*sin(goal_angle) - car_y);
            cords1 = cords1 * rot_M; //1x2 matrix
            angle_to_goal = angle(cords1.coeff(0,1),cords1.coeff(0,0));

            angle_from_tangent = 0;
        }
        CTS = angle_from_tangent;

        refrence_angle = 0.9 * angle_to_goal + 0.1 * CTS + car_angle;

        return refrence_angle*9/(3.14); //*pid_c.update(refrence_angle, car_angle)
        //returns something normally between pi/9 and if angle is bigger its capped later in main.
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

    private:
     
    float XTE;
    float CTS;
    float refrence_angle;
    float angle_to_goal;

    //Waypoints
    Eigen::MatrixXf P;
    float x_goal; 
    float y_goal; 
    float goal_angle;


    PIDController pid_c;

    //Testing private variables!

};

#endif /* PURSUIT_H_ */
