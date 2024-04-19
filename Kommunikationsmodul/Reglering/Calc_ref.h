#ifndef PURSUIT_H_
#define PURSUIT_H_

class Calc_ref{
    public:
    Calc_ref() = default;
    Calc_ref(Eigen::MatrixXf P,
     float x_goal, float y_goal, float goal_angle): 
    P{P},x_goal{x_goal}, y_goal{y_goal}, goal_angle{goal_angle}
    {
    }
    virtual ~Calc_ref()
    {}

    float update_ref(int size,float car_x, float car_y, float car_angle)
    {
        Eigen::VectorXf d_vec(size);

        for(int n=0; n <= size-1; n++ ){

            float  distance = pow(pow(car_x-P(n,0),2) + pow(car_y-P(n,1),2), 0.5);
            d_vec.row(n) << distance;
        }

        Eigen::Index index;
        float XTE = d_vec.minCoeff(&index); //minimum distance to ref line
        float angle_from_tangent = 0;
        float angle_to_goal = 0;

        Eigen::MatrixXf rot_M(2,2); //inverse of a rotation matrix in cars angle.
        rot_M(0,0) = cos(-car_angle);
        rot_M(1,1) = sin(-car_angle);
        rot_M(1,0) = -sin(-car_angle);
        rot_M(1,1) = cos(-car_angle);
        //std::cout << rot_M << std::endl;

        if(index + size/5 < size){

            Eigen::MatrixXf cords1(1,2);
            cords1(0,0) = P(index + size/10,0) - car_x;
            cords1(0,1) = P(index + size/10,1) - car_y;
            cords1 = cords1 * rot_M; //1x2 matrix
            float angle_to_goal = angle(cords1(0,1),cords1(0,0));


            Eigen::MatrixXf cords2(1,2);
            cords2(0,0) = P(index + size/10,0) - P(index,0);
            cords2(0,1) = P(index + size/10,1) - P(index,1);
            cords2 = cords2 * rot_M; //1x2 matrix
            float angle_from_tangent = angle(cords2(0,1),cords2(0,0));

        }else if (index + size/5 >= size)
        {
            Eigen::MatrixXf cords1(1,2);
            cords1(0,0) = x_goal + 500*cos(goal_angle) - car_x;
            cords1(0,1) = x_goal + 500*sin(goal_angle) - car_y;
            cords1 = cords1 * rot_M; //1x2 matrix
            float angle_to_goal = angle(cords1(0,1),cords1(0,0));

            float angle_from_tangent = 0;
        }
        float CTS = angle_from_tangent;

        refrence_angle= 0.4 * angle_to_goal + 0.1 * CTS + car_angle;
        //fix some max angle like abs && angle < pi/4 or angle > -pi/4

        if (refrence_angle > M_1_PI/4.0){ //check max steering angle
            return M_1_PI/4.0;
        }else if (refrence_angle < -M_1_PI/4.0)
        {
            return -M_1_PI/4.0;
        }else{
            return refrence_angle;
        }
        
    }

    float angle(float x, float y){
        //stupid special cases with (0,-0)??       
        return atan2f(y,x);
    }


    private:
     
    float XTE;
    float CTS;
    float refrence_angle;

    //Waypoints
    Eigen::MatrixXf P;
    float x_goal; 
    float y_goal; 
    float goal_angle;




};

#endif /* PURSUIT_H_ */
