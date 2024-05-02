#ifndef PLANNER_H_
#define PLANNER_H_

#include <Eigen/Dense>
#include <memory>

#include "Calc_ref.h"

class Planner
{
public:
    Planner() = default;

    void update(float prev_x, float prev_y, float prev_angle,float next_x, float next_y, float next_angle)
    {
        // set all the member variables
        x_start = prev_x;
        y_start = prev_y;
        start_angle = prev_angle;
        x_goal = next_x;
        y_goal = next_y;
        goal_angle = next_angle;
        s = Eigen::MatrixXf(4,2);

        
        //Matrix for calculations
        int k = 0;

        // base the number of points in the parameter curve on distance between gates
        float len = pow(pow(x_start-x_goal,2)+pow(y_start-y_goal,2),0.5f);
        size = (len)/85;
        P = Eigen::MatrixXf(size+5,2); // here we add 5 points after the last gate
        K = Eigen::MatrixXf(size+5,1); //MÅSTE FIXA MED DE EXTA 5 PUNKTERNA!!!
        R = Eigen::MatrixXf(size+5,1);
        Eigen::MatrixXf l(size,4);

        //Position in s matrix
        s.row(0) << x_start, y_start;
        s.row(1) << (x_start + 0.5f*len*cos(start_angle)), (y_start + 0.5f*len*sin(start_angle));
        s.row(2) << (x_goal - 0.5f*len*cos(goal_angle)), (y_goal - 0.5f*len*sin(goal_angle));
        s.row(3) << x_goal, y_goal;

        Eigen::RowVectorXf distance_vec(size);
        for(int u = 0; u < size; u++){ 

            float a = (u)/(size-1.f);

            float l1 = pow((1.f-a),(3));
            float l2 = 3*pow((1.f-a),(2))*(a);
            float l3 = 3*(1.f-a)*pow((a),(2));
            float l4 = pow((a),(3));

            l.row(u) << l1, l2, l3, l4;

        }
        // Here we add the extra points after the gate
        Eigen::MatrixXf Add_points(P.rows() - size,2);
        Eigen::MatrixXf Add_points1(K.rows() - size,2);
        Eigen::MatrixXf Add_points2(R.rows() - size,2); //test
        for(int i = 1; i <= P.rows()-size; i++)
        {
            Add_points.row(i-1) << (x_goal + 100*i*cos(goal_angle)), (y_goal + 100*i*sin(goal_angle));
            Add_points1.row(i-1) = minimum_scaled_speed;
            Add_points2.row(i-1) = min_radius;


        }
        P << l*s,  Add_points;
        
        for(int t = 0; t < size; t++) 
        //Derivation and second Derivation of the Bezier curve to calculate the curvature in each point
        {
            float step = t / (size*1.0f);
            float a = s.coeff(0,0);
            float b = s.coeff(1,0);
            float c = s.coeff(2,0);
            float d = s.coeff(3,0);
            float e = s.coeff(0,1);
            float f = s.coeff(1,1);
            float g = s.coeff(2,1);
            float h = s.coeff(3,1);

            float x_d = 3*((d-3*c+3*b-a)*pow(step,2) + 2*(c-2*b+a)*step + (b-a));

            float y_d = 3*((h-3*g+3*f-e)*pow(step,2) + 2*(g-2*f+e)*step + (f-e));

            float x_dd = 6*((d-3*c+3*b-a)*step + (c-2*b+a));

            float y_dd = 6*((h-3*g+3*f-e)*step + (g-2*f+e));

            //absolute value... 
            float scaled_speed = 0.1 ; // max steering is 0.5
            float k = pow(pow(      (x_d*y_dd - y_d*x_dd)/(pow(  (pow(x_d,2.f) + pow(y_d,2.f))   ,3.f/2.f))    ,2.f),0.5f);

            if(1/k <= min_radius){
                scaled_speed = minimum_scaled_speed;
            }else if ( 1/k>= max_radius)
            {
                scaled_speed = maximum_scaled_speed; 
            }else{
                scaled_speed = minimum_scaled_speed + (maximum_scaled_speed - minimum_scaled_speed)*(k-min_radius)/max_radius; //räta linkens ekvation
            }
 
            K.row(t) << scaled_speed;
            R.row(t) << 1/k;
        }

        K << Add_points1;
        R << Add_points2;

        std::cout << "R = " << R << std::endl;
        std::cout << "K = " << K << std::endl;

        //Is done when initalizing..
        calc_ref = std::make_unique<Calc_ref>(P,K, x_goal, y_goal, goal_angle);
        calc_ref->set_K_p_angle_to_goal(K_p_angle_to_goal);
        calc_ref->set_K_p_offset_tangent(K_p_offset_tangent);
    } 


    virtual ~Planner() 
    {}

    void set_min_radius(float radius)
    {
        min_radius = radius;
    }

    void set_max_radius(float radius)
    {
        max_radius = radius;
    }

    void set_maximum_scaled_speed(float scale)
    {
        maximum_scaled_speed = scale;
    }

    void set_minimum_scaled_speed(float scale)
    {
        minimum_scaled_speed =scale;
    }
    
    float getRefAngle(float car_x, float car_y, float car_angle)
    {
        return calc_ref->update_ref(size);
    }

    float getRefSpeed()
    {
        return calc_ref->get_scaled_Speed(); //Should be a scaled 0.1 to 0.5.. Dont drive backwards autonomt...
    }

    void set_K_p_angle_to_goal(float fraction)
    {
        K_p_angle_to_goal = fraction;
    }

    void set_K_p_offset_tangent(float fraction)
    {
        K_p_offset_tangent = fraction;
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

    Eigen::MatrixXf get_Bezier_mat()
    {
        return s;
    }


    std::string getBezier_curve()
    {
        std::ostringstream ss;
;            
        for(int i {0};i< P.rows(); i++)
        {
             ss << P.coeff(i,0) << "," << P.coeff(i,1) << ";";
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
    float K_p_angle_to_goal;
    float K_p_offset_tangent;
    int size = 20;
    Eigen::MatrixXf P;
    std::unique_ptr<Calc_ref> calc_ref;
    Eigen::MatrixXf K;
    Eigen::MatrixXf R;


    Eigen::MatrixXf s;
    float min_radius = 800.f;
    float max_radius = 2000.f;

    float minimum_scaled_speed = 0.15f;
    float maximum_scaled_speed = 0.45;
};

#endif /* PLANNER_H_ */
