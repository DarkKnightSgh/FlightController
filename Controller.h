#ifndef QUADCOPTERCONTROLLER_H
#define QUADCOPTERCONTROLLER_H

#include "IMU.h"
#include "Barometer.h"
#include "ESC.h"
#include "PIDController.h"
#include "Communication.h"
#include "HAL.h"

class QuadcopterController {
public:
    QuadcopterController(HAL& hal);
    void setup();
    void loop();

private:
    HAL& hal;
    IMU imu;
    Barometer barometer;
    ESC esc;
    PIDController rollPID;
    PIDController pitchPID;
    PIDController yawPID;
    Communication communication;

    float roll, pitch, yaw;
    float altitude;

    void stabilize();
};

#endif
