#include "PID.h"

int main()
{
    PIDController pid_c {0.5, {0.87154,6.84371,0,100,1,1}};
    pid_c.update(1,0.5);
    return 0;
}