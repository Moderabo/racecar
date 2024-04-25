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
      size {size}, P {size,2}, s {4,2}, calc_ref {}, K {size,1}
    {
        //Matrix for calculations
        int k = 0;
        Eigen::MatrixXf l(size,4);
        //Position in s matrix
        s.row(0) << x_start, y_start;
        s.row(1) << (x_start + 700*cos(start_angle)), (y_start + 700*sin(start_angle));
        s.row(2) << (x_goal - 700*cos(goal_angle)), (y_goal - 700*sin(goal_angle));
        s.row(3) << x_goal, y_goal;

        Eigen::RowVectorXf distance_vec(size);
        for(int u = 0; u < size; u++){ 

            float a = (u)/19.f;

            float l1 = pow((1.f-a),(3));
            float l2 = 3*pow((1.f-a),(2))*(a);
            float l3 = 3*(1.f-a)*pow((a),(2));
            float l4 = pow((a),(3));

            l.row(u) << l1, l2, l3, l4;

        }

        P = l * s; //Calculates the bezier curve

        for(int t = 0; t < size; t++) 
        //Small code from https://github.com/reiniscimurs/Bezier-Curve/blob/main/Bezier.py
        //Derivation and second Derivation of the Bezier curve to calculate the curvature in each point
        {
            float step = t / size;
            float x_d = 3*(pow((1-step),2))*(s.coeff(1,0) - P.coeff(0,0)) + 
                6*(1-step)*(step)*(s.coeff(2,0)-s.coeff(1,0)) +
                3*(pow(step,2))*(s.coeff(3,0) - s.coeff(2.0));

            float y_d = 3*(pow((1-step),2))*(s.coeff(1,1) - P.coeff(0,1)) + 
                6*(1-step)*(step)*(s.coeff(2,1)-s.coeff(1,1)) +
                3*(pow(step,2))*(s.coeff(3,1) - s.coeff(2,1));

            float x_dd = 6*(1-step)*(s.coeff(2,0) -2*s.coeff(1,0) + s.coeff(0,0)) +
                6*(step)*(s.coeff(3,0) -2*s.coeff(2,0) + s.coeff(1,0));

            float y_dd = 6*(1-step)*(s.coeff(2,1) -2*s.coeff(1,1) + s.coeff(0,1)) +
                6*(step)*(s.coeff(3,1) -2*s.coeff(2,1) + s.coeff(1,1));
            //absolute value... 
            float scaled_speed = 0.1 ; // max steering is 0.5
            float k = pow(pow((x_d*y_dd - y_d*x_dd)/(pow( (pow(x_d,2) + pow(y_d,2)) ,3.f/2.f)),2),0.5);
            float min_radius = 700.f;
            float max_radius = 2000.f;
            if(k <= min_radius){
                scaled_speed = 0.1; //minimum scaled speed
            }else if ( k>= max_radius)
            {
                scaled_speed = 0.3; //maximum scaled speed
            }else{
                scaled_speed = 0.1 + 0.2*(k-min_radius)/max_radius; //räta linkens ekvation
            }
 
            K.row(t) << scaled_speed;

        }

        //Is done when initalizing..
        calc_ref = std::make_unique<Calc_ref>(P,K, x_goal, y_goal, goal_angle);
    } 
    virtual ~Planner() 
    {}
    
    float getRefAngle(float car_x, float car_y, float car_angle)
    {
        return calc_ref->update_ref(size, car_x, car_y, car_angle);
    }

    float getRefSpeed()
    {
        return calc_ref->get_scaled_Speed(); //Should be a scaled 0.1 to 0.5.. Dont drive backwards autonomt...
    }

    void set_K_p_angle_to_goal(float fraction)
    {
        calc_ref->set_K_p_angle_to_goal(fraction);
    }

    void set_K_p_offset_tangent(float fraction)
    {
        calc_ref->set_K_p_offset_tangent(fraction);
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
        Eigen::MatrixXf Copy_P(size+10,2);
        Eigen::MatrixXf Add_points(10,2);
        for(int i = 1; i <= 10; i++)
        {
            Add_points.row(i-1) << (x_goal + 50*i*cos(goal_angle)), (y_goal + 50*i*sin(goal_angle));
        }

        Copy_P << P, Add_points;
            
        for(int i {0};i<= size + 10-1; i++)
        {
             ss << Copy_P.coeff(i,0) << "," << Copy_P.coeff(i,1) << ";";
        }

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
    Eigen::MatrixXf K;


    Eigen::MatrixXf s;
};

#endif /* PLANNER_H_ */
