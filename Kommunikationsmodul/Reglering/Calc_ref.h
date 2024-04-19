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
        float angle_from_tangent;
        float angle_to_goal;

        Eigen::VectorXf d_vec(size);

        for(int n=0; n <= size-1; n++ ){

            float  distance = pow(pow(car_x-P.coeff(n,0),2) + pow(car_y-P.coeff(n,1),2), 0.5);
            d_vec.row(n) << distance;
        }

        Eigen::Index index;
        float XTE = d_vec.minCoeff(&index); //minimum distance to ref line
        std::cout << index << std::endl;

        Eigen::MatrixXf rot_M(2,2); //inverse of a rotation matrix in cars angle.
        rot_M.row(0) << cos(-car_angle), sin(-car_angle);
        rot_M.row(1) << -sin(-car_angle), cos(-car_angle);

        //std::cout << rot_M << std::endl;

        if(index + size/5 < size){

            Eigen::MatrixXf cords1(1,2);
            cords1.row(0) << (P.coeff(index + size/10,0) - car_x), (P.coeff(index + size/10,1) - car_y);
            //std::cout << cords1 << std::endl;
            cords1 = cords1 * rot_M; //1x2 matrix
            //std::cout << cords1 << std::endl;
            angle_to_goal = angle(cords1.coeff(0,1),cords1.coeff(0,0));
            std::cout << angle_to_goal << std::endl;


            Eigen::MatrixXf cords2(1,2);
            cords2.row(0) << (P.coeff(index + size/10,0) - P.coeff(index,0)), (P.coeff(index + size/10,1) - P.coeff(index,1));
            cords2 = cords2 * rot_M; //1x2 matrix
            angle_from_tangent = angle(cords2.coeff(0,1),cords2.coeff(0,0));
            std::cout << angle_from_tangent << std::endl;

        }else if (index + size/5 >= size)
        {
            Eigen::MatrixXf cords1(1,2);
            cords1.row(0) << (x_goal + 500*cos(goal_angle) - car_x), (x_goal + 500*sin(goal_angle) - car_y);
            cords1 = cords1 * rot_M; //1x2 matrix
            angle_to_goal = angle(cords1.coeff(0,1),cords1.coeff(0,0));

            angle_from_tangent = 0;
        }
        CTS = angle_from_tangent;

        std::cout << CTS << std::endl;
        std::cout << angle_to_goal << std::endl;
        std::cout << car_angle << std::endl;
        refrence_angle = 0.4 * angle_to_goal + 0.1 * CTS + car_angle;
        std::cout << refrence_angle << std::endl;
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

    float angle(float y, float x){
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
