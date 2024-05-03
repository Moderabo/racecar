#include "Planner.h"
#include "PID.h"

void Planner::update(float x_start, float y_start, float start_angle,float x_goal, float y_goal, float goal_angle, float T_c)
{
    // set all the member variables
    s = Eigen::MatrixXf(4,2);

    // base the number of points in the parameter curve on distance between gates
    float len = pow(pow(x_start-x_goal,2)+pow(y_start-y_goal,2),0.5f);
    int size = (len)/85;
    P = Eigen::MatrixXf(size+5,2); // here we add 5 points after the last gate
    K = Eigen::MatrixXf(size+5,1); //Eigen kan inte K << K, Addpoints1 därav K1
    K1 = Eigen::MatrixXf(size,1);
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
    Eigen::MatrixXf Add_points1(K.rows() - size,1);
    for(int i = 1; i <= P.rows()-size; i++)
    {
        Add_points.row(i-1) << (x_goal + 100*i*cos(goal_angle)), (y_goal + 100*i*sin(goal_angle));
        Add_points1.row(i-1) << minimum_scaled_speed;


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
            scaled_speed = minimum_scaled_speed + (maximum_scaled_speed - minimum_scaled_speed)*(1/k-min_radius)/max_radius; //räta linkens ekvation
        }

        K1.row(t) << scaled_speed;
    }

    K = K1 , Add_points1;

    // This was update_ref now we do what it did in the primary update function
    //=============================================================================================

    Eigen::VectorXf d_vec(P.rows());
    PIDController pid_c {T_c, {0.87154,6.84371,0,100,1,1}};


    // here we should loop over all the points in the P vector (size + extra after gate)
    for(int n=0; n < P.rows(); n++ ){

        float  distance = pow(pow(P.coeff(n,0),2) + pow(P.coeff(n,1),2), 0.5);
        d_vec.row(n) << distance;
    }

    // calculate minimum distance to the line and get the corresponding index
    Eigen::Index index;
    XTE = d_vec.minCoeff(&index);

    // calculate the angle to the look ahead point
    float angle_to_goal = atan2f(P.coeff(index + look_ahead_dist,1),P.coeff(index + look_ahead_dist,0));

    // calculate the angle to the tangent
    float x = P.coeff(index + 1,0) - P.coeff(index,0);
    float y = P.coeff(index + 1,1) - P.coeff(index,1);
    float angle_from_tangent = atan2f(y,x);

    CTS = angle_from_tangent;

    // get the angle we should turn
    float controller = pid_c.update((K_p_angle_to_goal * angle_to_goal + K_p_offset_tangent * CTS)*9/(3.14),0);
    refrence_angle = (K_p_angle_to_goal * angle_to_goal + K_p_offset_tangent * CTS)*9/(3.14)*controller;
    
    refrence_speed =  K.coeff(index);
    // the return is still found in getRefAngle, in the return pid_c is added.
}

void Planner::set_min_radius(float radius)
{
    min_radius = radius;
}

void Planner::set_max_radius(float radius)
{
    max_radius = radius;
}

void Planner::set_maximum_scaled_speed(float scale)
{
    maximum_scaled_speed = scale;
}

void Planner::set_minimum_scaled_speed(float scale)
{
    minimum_scaled_speed =scale;
}

void Planner::set_K_p_angle_to_goal(float fraction)
{
    K_p_angle_to_goal = fraction;
}

void Planner::set_K_p_offset_tangent(float fraction)
{
    K_p_offset_tangent = fraction;
}

float Planner::getRefAngle()
{
    //returns something normally between pi/9 and -pi/9 scaled to [-1,1] and if angle is bigger its capped later in main.
    return refrence_angle;
}

float Planner::getRefSpeed()
{
    return refrence_speed;
}

float Planner::get_XTE()
{
    return XTE;
}

float Planner::get_CTS()
{
    return CTS;
}

Eigen::MatrixXf Planner::get_Bezier_mat()
{
    return s;
}

std::string Planner::getBezier_curve()
{
    std::ostringstream ss;        
    for(int i {0};i< P.rows(); i++)
    {
            ss << P.coeff(i,0) << "," << P.coeff(i,1) << ";";
    }

    return ss.str();
}

std::string Planner::getBezier_points()
{
    std::ostringstream ss;
    for(int i {0};i<=3; i++)
    {
        ss << s.coeff(i,0) << "," << s.coeff(i,1) << ";";
    }

    return ss.str();
}
