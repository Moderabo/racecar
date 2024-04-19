#include "PID.h"

int main()
{
    PIDController pid_c {1, {1,1,1,1,1,1}};
    pid_c.update(1,0.5);
    return 0;
}