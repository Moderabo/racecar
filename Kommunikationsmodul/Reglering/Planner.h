#ifndef PLANNER_H_
#define PLANNER_H_

#include "Calc_ref.h"

class Planner
{
public:
    Planner(float x_start, float y_start, float start_angle,
            float x_goal, float y_goal, float goal_angle,
            int size=20)
    : x_start{x_start}, y_start{y_start}, start_angle{start_angle},
      x_goal{x_goal}, y_goal{y_goal}, goal_angle{goal_angle}, 
      size {size}, P {size,2}
    {
        int r = 0;
        Eigen::MatrixXf l(size,4);
        Eigen::MatrixXf s(4,2);
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
        //std::cout << l << std::endl;
        //std::cout << s << std::endl;

        P = l * s;

        calc_ref = Calc_ref(P, x_goal, y_goal, goal_angle);
    } 
    virtual ~Planner() 
    {}
    Eigen::MatrixXf update_P(int size=20)
    {
        int r = 0;
        Eigen::MatrixXf l(size,4);
        Eigen::MatrixXf s(4,2);
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
        //std::cout << l << std::endl;
        //std::cout << s << std::endl;

        P = l * s;

        return P;
    }
    float getRefAngle(float car_x, float car_y, float car_angle)
    {
        return calc_ref.update_ref(size, car_x, car_y, car_angle);
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
    Calc_ref calc_ref;
};

#endif /* PLANNER_H_ */
