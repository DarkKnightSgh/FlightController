#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

class PIDController
{
public:
    PIDController(float p, float i, float d);
    float compute(float setpoint, float measurement);

    void setTunings(float p, float i, float d);

private:
    float kp, ki, kd;
    float prevError;
    float integral;
};

#endif
