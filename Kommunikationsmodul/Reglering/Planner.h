#ifndef PLANNER_H_
#define PLANNER_H_

#include <Eigen/Dense>
#include <memory>

#include "Calc_ref.h"

class Planner
{
public:
    Planner() = default;
    ~Planner() = default;

    // the main function should be called once every time we get new gate-data
    void update(float prev_x, float prev_y, float prev_angle,float next_x, float next_y, float next_angle);
 
    // setters for the parameters used when determining ref speed and angle
    void set_min_radius(float radius);
    void set_max_radius(float radius);
    void set_maximum_scaled_speed(float scale);
    void set_minimum_scaled_speed(float scale);
    void set_K_p_angle_to_goal(float fraction);
    void set_K_p_offset_tangent(float fraction);

    // all the acess of data should be done by getters that do not do any calculations
    float getRefSpeed();
    float getRefAngle();
    float get_XTE();
    float get_CTS();
    Eigen::MatrixXf get_Bezier_mat();

    // Get the data in a string format which can be sent over mqtt 
    std::string getBezier_points();
    std::string getBezier_curve();

private:

    // parameters for controll
    float K_p_angle_to_goal;
    float K_p_offset_tangent;

    //  contains all the points on the bezier curve
    Eigen::MatrixXf P;

    // contains curvature in all points of the bezier curve
    Eigen::MatrixXf K;

    // ?????
    Eigen::MatrixXf R;

    // the 4 points that define the bezier curve
    Eigen::MatrixXf s;

    // default values used for determining speed 
    float min_radius = 800.f;
    float max_radius = 2000.f;
    float minimum_scaled_speed = 0.15f;
    float maximum_scaled_speed = 0.45;


    // stuff used in Calc_ref.h maybe remove?
    float XTE;
    float CTS;
    float refrence_angle;
    float refrence_speed;

    // how many points ahead we look in pure pursit
    int look_ahead_dist = 3;
};

#endif /* PLANNER_H_ */
