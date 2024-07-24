#include "PIDController.h"

PIDController::PIDController(float p, float i, float d) 
    : kp(p), ki(i), kd(d), prevError(0), integral(0) {}

void PIDController::setTunings(float p, float i, float d)
{
    kp = p;
    ki = i;
    kd = d;
}

float PIDController::compute(float setpoint, float measurement)
{
    float error = setpoint - measurement;
    integral += error;
    float derivative = error - prevError;
    prevError = error;
    return kp * error + ki * integral + kd * derivative;
}
