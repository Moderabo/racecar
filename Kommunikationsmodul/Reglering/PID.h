#ifndef PIDCONTROLLER_H_
#define PIDCONTROLLER_H_

//This is C++ implementation of MATLAB's 2DOF discrete time PID controller
//https://gist.github.com/idt12312/6422524ff294e7aa3bddcbec74c8d269
class PIDController
{
  public:
    struct Param
    {
        float P;
        float I;
        float D;
        float N;
        float b;
        float c;
    };

    // _Ts: Control loop period
    // _param: Controller parameter
    PIDController(float _Ts, const Param &_param) : Ts(_Ts), param(_param)
    {
        reset();
    }

    virtual ~PIDController() {}

    float update(float r, float y)
    {
        const float u_d = ((param.c * r - y) * param.D - filtered_derivative_state) * param.N;
        const float u_p = (param.b * r - y) * param.P;

        const float u = u_d + u_p + integral_state;

        integral_state += Ts * (r - y) * param.I;
        filtered_derivative_state += Ts * u_d;

        return u;
    }

    void reset()
    {
        integral_state = 0.0;
        filtered_derivative_state = 0.0;
    }

  private:
    const float Ts;
    const Param param;
    float integral_state;
    float filtered_derivative_state;
};

#endif /* PIDCONTROLLER_H_ */