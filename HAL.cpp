#include "HAL.h"

void initMotors() {
    // analogWriteFrequency(1, 250);
    // analogWriteFrequency(2, 250);
    // analogWriteFrequency(3, 250);
    // analogWriteFrequency(4, 250);
    // analogWriteResolution(12);
}

void setMotorPWM(int motor, int value) {
    // analogWrite(motor, value);
}

int readReceiver(int channel) {
    // Read receiver input from specified channel
    // return pulseIn(channel, HIGH, 25000);
}

float readBatteryVoltage() {
    // Read and return battery voltage
    // return analogRead(A0) * (5.0 / 1023.0);
}

void delayMs(int ms) {
    // delay(ms);
}

unsigned long currentMicros() {
    // return micros();
}
