#include "Planner.h"
#include "PID.h"

#include <iostream>

void Planner::update(Gate prev_gate, Gate next_gate, float T_c)
{
    // based on the state you should do different things
    switch(current_state) 
    {
    case calibration:     // calibration
    {
        update_segment(prev_gate, next_gate);

        // take out the coordinates from the gates
        float x_start = prev_gate.x;
        float y_start = prev_gate.y;
        float start_angle = prev_gate.angle;
        float x_goal = next_gate.x;
        float y_goal = next_gate.y;
        float goal_angle = next_gate.angle;

        // Determine the number of points in the parameter curve based on distance
        // between gates
        float len = sqrtf(pow(x_start-x_goal,2)+pow(y_start-y_goal,2));
        int size = len / 85;
        
        // {\HUGE OBS these functions must be called in this specific order!!}

        // now we calculate the bezier curve (matrices s and P)
        calc_P(size, x_start, y_start, start_angle,x_goal, y_goal, goal_angle);

        // calculate the curvature in every point (matrix K)
        calc_K(size);

        // now calculate all the parameters used in controlling the car
        calc_ref();

        refrence_speed = minimum_scaled_speed;

        // if we should change state
        if (lap_nr == 0 && next_gate.type == 2 && sqrtf(pow(x_goal,2)+pow(y_goal,2)) < 1000)
        {
            current_state = stop;
            timer = 5;
        }
        break;
    }

    case stop:    // stop before comp
    {
        timer = timer - T_c;
        if (timer < 0)
        {
            current_state = comp;
        }
        refrence_speed = 0;
        break;
    }

    case comp:     // time trials
    {
        if (lap_nr > 5)
            current_state = finish;

        Gate next_next_gate;
        
        if (prev_gate.type != -2 || next_gate.type != -2)
        {
            float prev_gate_dist {pow(prev_gate.x,2)+pow(prev_gate.y,2)};
            float next_gate_dist {pow(next_gate.x,2)+pow(next_gate.y,2)};
            
            if (prev_gate_dist < next_gate_dist)
            {
                next_gate = calc_next_gate(prev_gate, segment);
                next_next_gate = calc_next_gate(next_gate, segment+1 % size(segment);
            }
            else
            {
                prev_gate = calc_prev_gate(next_gate, segment);
                next_next_gate = calc_next_gate(next_gate, segment+1 % size(segment);
            }
        }
        else if (prev_gate.type != -2)
        {
            next_gate = calc_next_gate(prev_gate, segment);
            next_next_gate = calc_next_gate(next_gate, segment+1 % size(segment);
        }
        else if (next_gate.type != -2)
        {
            prev_gate = calc_prev_gate(next_gate, segment);
            next_next_gate = calc_next_gate(next_gate, segment+1 % size(segment);
        }
        else
        {
            next_gate = calc_next_gate(prev_gate, segment);
            next_next_gate = calc_next_gate(next_gate, segment+1 % size(segment);
        }

        // take out the coordinates from the gates
        float x_start = prev_gate.x;
        float y_start = prev_gate.y;
        float start_angle = prev_gate.angle;
        float x_goal = next_gate.x;
        float y_goal = next_gate.y;
        float goal_angle = next_gate.angle;
        float x_goal_next = next_next_gate.x;
        float y_goal_next = next_next_gate.y;
        float goal_angle_next = next_next_gate.angle;

        // Determine the number of points in the parameter curve based on distance
        // between gates
        float len = sqrtf(pow(x_start-x_goal,2)+pow(y_start-y_goal,2));
        int size = len / 85;
        float len_next = sqrtf(pow(x_goal-x_goal_next,2)+pow(y_goal-y_goal_next,2));
        int size_next = len_next / 85;
        
        // {\HUGE OBS these functions must be called in this specific order!!}

        // now we calculate the bezier curve (matrices s and P)
        calc_P_comp(size, x_start, y_start, start_angle,x_goal, y_goal,
                    goal_angle, size_next, );

        // calculate the curvature in every point (matrix K)
        calc_K(size, size_next);

        // now calculate all the parameters used in controlling the car
        calc_ref();

        break;
    }

    case finish: // when the race is finished do nothing!
    {
        refrence_speed = 0;
        break;
    }
    }

    // here we do some stupid stuff to check if we have passed a gate or not
    if (in_a_gate && !isInGate(prev_gate))
    {
        segment_nr += 1;
        if (in_goal)
        {
            segment_nr = 0;
            lap_nr += 1;
            std::cout << "lap: " << lap_nr << std::endl;
        }
    }
        
    if (isInGate(prev_gate))
    {
        in_a_gate = true;
        if (prev_gate.type == 2)
        {
            in_goal = true;
        }
    }
    else
    {
        in_a_gate = false;
        in_goal = false;
    }
}

void Planner::update_segment(Gate& prev_gate, Gate& next_gate)
{
    if (prev_gate.type == -2 || next_gate.type == -2 || segment_nr < 0)
    {
        return;
    }
    // Pre-rotation
    float x_pre {next_gate.x - prev_gate.x};
    float y_pre {next_gate.y - prev_gate.y};
    float angle {next_gate.angle - prev_gate.angle};
    
    float x {x_pre*cos(-prev_gate.angle)-y_pre*sin(-prev_gate.angle)};
    float y {x_pre*sin(-prev_gate.angle)+y_pre*cos(-prev_gate.angle)};
    
    if (segment_nr >= size(segments))
    {
        Gate gate {x, y, angle};
        segments.push_back(gate);
        return;
    }

    segment.at(segment_nr).n++;
    
    Gate gate = segments.at(segment_nr).gate;
    Gate last_gate = segments.at(segment_nr).last_gate;
    gate.x += (x - last_gate.x) / n;
    gate.y += (y - last_gate.y) / n;
    gate.angle += (angle - last_gate.angle) / n;

}

Gate calc_next_gate(Gate& gate, int seg_nr)
{
    // 
    float angle_pre = segment.at(seg_nr).gate.angle;
    float x_pre = segment.at(seg_nr).gate.x;
    float y_pre = segment.at(seg_nr).gate.y;

    // Rotate saved segment to prev_gate coordinate system
    float x {x_pre*cos(prev_gate.angle)-y_pre*sin(prev_gate.angle)};
    float y {x_pre*sin(prev_gate.angle)+y_pre*cos(prev_gate.angle)};

    // Calculate new position
    Gate next_gate;
    next_gate.x = gate.x + x;
    next_gate.y = gate.y + y;
    next_gate.angle = angle_pre - gate.angle;

    return next_gate;
}

Gate calc_prev_gate(Gate& gate, int seg_nr)
{
    float angle_pre = segment.at(seg_nr).gate.angle;
    float x_pre = segment.at(seg_nr).gate.x;
    float y_pre = segment.at(seg_nr).gate.y;

    float x {x_pre*cos(prev_gate.angle)-y_pre*sin(prev_gate.angle)};
    float y {x_pre*sin(prev_gate.angle)+y_pre*cos(prev_gate.angle)};

    Gate next_gate;
    next_gate.x = gate.x - x;
    next_gate.y = gate.y - y;
    next_gate.angle = angle_pre - gate.angle;

    return next_gate;
}

void Planner::calc_P(int size, float x_start, float y_start, float start_angle,
                     float x_goal, float y_goal, float goal_angle)
{    
    // set all the member variables
    P = Eigen::MatrixXf(size+10,2); // here we add 5 points after the last gate
    s = Eigen::MatrixXf(4,2);
    Eigen::MatrixXf l(size,4);

    float len = pow(pow(x_start-x_goal,2)+pow(y_start-y_goal,2),0.5f);

    //Position in s matrix
    s.row(0) << x_start, y_start;
    s.row(1) << (x_start + 0.3f*len*cos(start_angle)), (y_start + 0.3f*len*sin(start_angle));
    s.row(2) << (x_goal - 0.3f*len*cos(goal_angle)), (y_goal - 0.3f*len*sin(goal_angle));
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
    for(int i = 1; i <= P.rows()-size; i++)
    {
        Add_points.row(i-1) << (x_goal + 100*i*cos(goal_angle)), (y_goal + 100*i*sin(goal_angle));

    }
    P << l*s,  Add_points;
}

void Planner::calc_P_comp(int size, float x_start, float y_start, float start_angle,
                     float x_goal, float y_goal, float goal_angle,
                     int size_next, float x_goal_next, float y_goal_next, float goal_angle_next)
{    
    // set all the member variables
    P = Eigen::MatrixXf(size+size_next,2); // here we add 5 points after the last gate
    s = Eigen::MatrixXf(4,2);
    Eigen::MatrixXf l(size,4);

    float len = pow(pow(x_start-x_goal,2)+pow(y_start-y_goal,2),0.5f);

    //Position in s matrix
    s.row(0) << x_start, y_start;
    s.row(1) << (x_start + 0.3f*len*cos(start_angle)), (y_start + 0.3f*len*sin(start_angle));
    s.row(2) << (x_goal - 0.3f*len*cos(goal_angle)), (y_goal - 0.3f*len*sin(goal_angle));
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
    P << l*s;

    s_next = Eigen::MatrixXf(4,2);
    Eigen::MatrixXf l_next(size,4);

    len = pow(pow(x_goal-x_goal_next,2)+pow(y_goal-y_goal_next,2),0.5f);

    //Position in s matrix
    s.row(0) << x_goal, y_goal;
    s.row(1) << (x_goal + 0.3f*len*cos(goal_angle)), (y_start + 0.3f*len*sin(goal_angle));
    s.row(2) << (x_goal_next - 0.3f*len*cos(goal_angle_next)), (y_goal_next - 0.3f*len*sin(goal_angle_next));
    s.row(3) << x_goal_next, y_goal_next;

    Eigen::RowVectorXf distance_vec(size_next);
    for(int u = 0; u < size; u++){ 

        float a = (u)/(size-1.f);

        float l1 = pow((1.f-a),(3));
        float l2 = 3*pow((1.f-a),(2))*(a);
        float l3 = 3*(1.f-a)*pow((a),(2));
        float l4 = pow((a),(3));

        l.row(u) << l1, l2, l3, l4;

    }
    P << l*s_next;
}

void Planner::calc_K(int size)
{
    K = Eigen::MatrixXf(size+10,1); //Eigen kan inte K << K, Addpoints1 därav K1
    Eigen::MatrixXf K1(size,1);
    
    Eigen::MatrixXf Add_points(K.rows() - size,1);
    for(int i = 1; i <= P.rows()-size; i++)
        Add_points.row(i-1) << minimum_scaled_speed;

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
        float scaled_speed; // max steering is 0.5
        float k = sqrtf(pow( (x_d*y_dd - y_d*x_dd)/(pow( (pow(x_d,2.f) + pow(y_d,2.f)), 3.f/2.f)), 2.f));

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

    K << K1 , Add_points;
}

void Planner::calc_K_comp(int size, int size_next)
{
    K = Eigen::MatrixXf(size+size_next,1); //Eigen kan inte K << K, Addpoints1 därav K1
    Eigen::MatrixXf K1(size,1);
    
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
        float scaled_speed; // max steering is 0.5
        float k = sqrtf(pow( (x_d*y_dd - y_d*x_dd)/(pow( (pow(x_d,2.f) + pow(y_d,2.f)), 3.f/2.f)), 2.f));

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

    Eigen::MatrixXf K2(size_next,1);
    
    for(int t = 0; t < size_next; t++) 
    //Derivation and second Derivation of the Bezier curve to calculate the curvature in each point
    {
        float step = t / (size*1.0f);
        float a = s_next.coeff(0,0);
        float b = s_next.coeff(1,0);
        float c = s_next.coeff(2,0);
        float d = s_next.coeff(3,0);
        float e = s_next.coeff(0,1);
        float f = s_next.coeff(1,1);
        float g = s_next.coeff(2,1);
        float h = s_next.coeff(3,1);

        float x_d = 3*((d-3*c+3*b-a)*pow(step,2) + 2*(c-2*b+a)*step + (b-a));

        float y_d = 3*((h-3*g+3*f-e)*pow(step,2) + 2*(g-2*f+e)*step + (f-e));

        float x_dd = 6*((d-3*c+3*b-a)*step + (c-2*b+a));

        float y_dd = 6*((h-3*g+3*f-e)*step + (g-2*f+e));

        //absolute value... 
        float scaled_speed; // max steering is 0.5
        float k = sqrtf(pow( (x_d*y_dd - y_d*x_dd)/(pow( (pow(x_d,2.f) + pow(y_d,2.f)), 3.f/2.f)), 2.f));

        if(1/k <= min_radius){
            scaled_speed = minimum_scaled_speed;
        }else if ( 1/k>= max_radius)
        {
            scaled_speed = maximum_scaled_speed; 
        }else{
            scaled_speed = minimum_scaled_speed + (maximum_scaled_speed - minimum_scaled_speed)*(1/k-min_radius)/max_radius; //räta linkens ekvation
        }

        K2.row(t) << scaled_speed;
    }

    K << K1 , K2;
}

void Planner::calc_ref()
{
    Eigen::VectorXf d_vec(P.rows());

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
    float K_i = 0.005;
    float K_d = 0.5;
    float anglesgn = 1.f;
    //punish on angle diffrence and tangent offset
    float unscaled_ref_angle = K_p_angle_to_goal * angle_to_goal + K_p_offset_tangent * CTS;
    if(angle_to_goal < 0)
    {
        anglesgn = -1.f;
    }
    //Also punish for error to the refrence track and a derivitate state that depens on time
    float unscaled_controller = T_c * ( XTE*anglesgn*K_i - unscaled_ref_angle*K_d); 
    refrence_angle = (unscaled_ref_angle + unscaled_controller)*9/(3.14);
    
    refrence_speed =  K.coeff(index+3);
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
    for(int i {0}; i < P.rows(); i++)
    {
            ss << P.coeff(i,0) << "," << P.coeff(i,1) << ";";
    }

    return ss.str();
}

std::string Planner::getBezier_points()
{
    std::ostringstream ss;
    for(int i {0}; i < 4; i++)
    {
        ss << s.coeff(i,0) << "," << s.coeff(i,1) << ";";
    }

    return ss.str();
}
