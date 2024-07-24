#ifndef COMMUNICATION_H
#define COMMUNICATION_H

class Communication {
public:
    void initialize();
    void sendData(float roll, float pitch, float yaw, float altitude);
};

#endif
