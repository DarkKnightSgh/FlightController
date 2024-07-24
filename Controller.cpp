#include "Controller.h"

QuadcopterController::QuadcopterController(HAL& hal) 
    : hal(hal), rollPID(1.0, 0.0, 0.0), pitchPID(1.0, 0.0, 0.0), yawPID(1.0, 0.0, 0.0),
      roll(0), pitch(0), yaw(0), altitude(0) {}

void QuadcopterController::setup() {
    hal.initMotors();
    imu.initialize();
    barometer.initialize();
    communication.initialize();
}

void QuadcopterController::loop() {
    unsigned long loopTimer = hal.currentMicros();

    imu.readSensorData(roll, pitch, yaw);
    altitude = barometer.readAltitude();

    stabilize();

    communication.sendData(roll, pitch, yaw, altitude);

    while (hal.currentMicros() - loopTimer < 4000);
}

void QuadcopterController::stabilize() {
    float rollOutput = rollPID.compute(0, roll);
    float pitchOutput = pitchPID.compute(0, pitch);
    float yawOutput = yawPID.compute(0, yaw);

    esc.setMotorSpeed(0, 1500 + rollOutput + pitchOutput - yawOutput);
    esc.setMotorSpeed(1, 1500 - rollOutput + pitchOutput + yawOutput);
    esc.setMotorSpeed(2, 1500 - rollOutput - pitchOutput - yawOutput);
    esc.setMotorSpeed(3, 1500 + rollOutput - pitchOutput + yawOutput);
}
