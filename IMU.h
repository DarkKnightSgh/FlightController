#ifndef IMU_H
#define IMU_H

class IMU {
public:
    void initialize();
    void readSensorData(float &roll, float &pitch, float &yaw);
};

#endif
