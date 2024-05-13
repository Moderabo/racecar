#ifndef PLANNER_H_
#define PLANNER_H_

#include "utils.h"
#include <Eigen/Dense>
#include <memory>

class Planner
{
public:
    Planner() = default;
    ~Planner() = default;

    // the main function should be called once every time we get new gate-data
    void update(Gate prev_gate, Gate next_gate,float T_c);
 
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
    // Member variables
    //================================================================

    // parameters for controll
    float K_p_angle_to_goal;
    float K_p_offset_tangent;

    // Segments
    struct Segment{
        Gate gate;
        int n;
        Gate last_gate;
    };
    std::vector<Segment> segments;

    // contains all the points on the bezier curve
    Eigen::MatrixXf P;

    // contains curvature in all points of the bezier curve
    Eigen::MatrixXf K;

    // the 4 points that define the bezier curve
    Eigen::MatrixXf s;
    Eigen::MatrixXf s_next;

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

    // state variables
    enum state {calibration, stop, comp, finish};
    state current_state = calibration;
    float timer;    // Timer for sleep after calibration

    // parameters used in controlling the segments
    int segment_nr = -1;
    int lap_nr = -1;
    bool in_a_gate = false;
    bool in_goal = false;

    //time diffrence for pid controller.
    float T_c;

    // how many points ahead we look in pure pursit
    int look_ahead_dist = 4;

    // Private methods used in update, should be called in 
    // the order they are written here
    //==========================================================

    // update segment
    void update_segment(Gate& prev_gate, Gate& next_gate);
    Gate calc_next_gate(Gate&, int);
    Gate calc_prev_gate(Gate&, int);

    // calculate the parameter curve
    void calc_P(int size, float x_start, float y_start, float start_angle,
                float x_goal, float y_goal, float goal_angle);
    void calc_P_comp(int, float, float, float, float, float, float, int,
                     float, float, float);
    // calculate the curvature of the curve in each point
    void calc_K(int size);
    void calc_K_comp(int size, int);
    // calculat the all the usefull stuff, this requires P and K
    void calc_ref();
};

#endif /* PLANNER_H_ */
