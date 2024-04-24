#ifndef PLANNER_H_
#define PLANNER_H_

#include <memory>

#include "Calc_ref.h"

class Planner
{
public:
    Planner(float x_start, float y_start, float start_angle,
            float x_goal, float y_goal, float goal_angle,
            int size=20)
    : x_start{x_start}, y_start{y_start}, start_angle{start_angle},
      x_goal{x_goal}, y_goal{y_goal}, goal_angle{goal_angle}, 
      size {size}, P {size,2}, s {4,2}, calc_ref {} 
    {
        //Matrix for calculations
        int r = 0; //index in loop
        Eigen::MatrixXf l(size,4);
        //Eigen::MatrixXf s(4,2);
        //Position in s matrix
        s.row(0) << x_start, y_start;
        s.row(1) << (x_start + 700*cos(start_angle)), (y_start + 700*sin(start_angle));
        s.row(2) << (x_goal - 700*cos(goal_angle)), (y_goal - 700*sin(goal_angle));
        s.row(3) << x_goal, y_goal;

        Eigen::RowVectorXf distance_vec(size);
        for(float u = 1.f/size; u <=(1.f-1.f/size); u = u + 1.f/size){

            float l1 = pow((1.f-u),(3));
            float l2 = 3*pow((1.f-u),(2))*(u);
            float l3 = 3*(1.f-u)*pow((u),(2));
            float l4 = pow((u),(3));

            r = int(size*u);

            l.row(r) << l1, l2, l3, l4;

        }
        l.row(0) << 1, 0, 0, 0; //might be one pointv wring scaled in the curve..
        l.row(size-1) << 0, 0, 0, 1; //special case because its 0 index and its hard to think..

        P = l * s; //Calculates the bezier curve
        std::cout << P << std::endl;

        //Is done when initalizing..
        calc_ref = std::make_unique<Calc_ref>(P, x_goal, y_goal, goal_angle);
    } 
    virtual ~Planner() 
    {}
    Eigen::MatrixXf update_P(int size=20)
    {
        int r = 0;
        Eigen::MatrixXf l(size,4);
        //Eigen::MatrixXf s(4,2);
        s.row(0) << x_start, y_start;
        s.row(1) << (x_start + 700*cos(start_angle)), (y_start + 700*sin(start_angle));
        s.row(2) << (x_goal - 700*cos(goal_angle)), (y_goal - 700*sin(goal_angle));
        s.row(3) << x_goal, y_goal;

        Eigen::RowVectorXf distance_vec(size);

        for(float u = 1.f/size; u <=(1.f-1.f/size); u = u + 1.f/size){

            float l1 = pow((1.f-u),(3));
            float l2 = 3*pow((1.f-u),(2))*(u);
            float l3 = 3*(1.f-u)*pow((u),(2));
            float l4 = pow((u),(3));

            r = int(size*u);

            l.row(r) << l1, l2, l3, l4;

        }
        l.row(0) << 1, 0, 0, 0;
        l.row(size-1) << 0, 0, 0, 1;

        P = l * s;

        return P;
    }
    float getRefAngle(float car_x, float car_y, float car_angle)
    {
        return calc_ref->update_ref(size, car_x, car_y, car_angle);
    }

    std::string getBezier_points()
    {
        std::ostringstream ss;
        for(int i {0};i<=3; i++)
        {
            ss << s.coeff(i,0) << "," << s.coeff(i,1) << ";";
        }

        return ss.str();
    }

    std::string getBezier_curve()
    {
        std::ostringstream ss;
        for(int i {0};i<=19; i++)
        {
             ss << P.coeff(i,0) << "," << P.coeff(i,1) << ";";
        }
        ss << P.coeff(19,0) + 500*cos(goal_angle) << "," << P.coeff(19,1) + 500*sin(goal_angle) << ";";
        ss << P.coeff(19,0) + 250*cos(goal_angle) << "," << P.coeff(19,1) + 250*sin(goal_angle) << ";";

        return ss.str();

    }

private:
    float x_start; 
    float y_start; 
    float start_angle; 
    float x_goal; 
    float y_goal; 
    float goal_angle;
    int size;
    Eigen::MatrixXf P;
    std::unique_ptr<Calc_ref> calc_ref;
        //Calc_ref calc_ref_tmp {P, x_goal, y_goal, goal_angle};
        //Calc_ref calc_ref_tmp {P, x_goal, y_goal, goal_angle};

    Eigen::MatrixXf s;
};

#endif /* PLANNER_H_ */
