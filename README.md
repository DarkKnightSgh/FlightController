## Custom Quadcopter Flight Controller

This project implements a custom flight controller for a quadcopter using a microcontroller for stabilization tasks and a companion computer for high-level tasks. This separation ensures efficient real-time control and flexibility for advanced features.

### Components:

Microcontroller : Handles real-time stabilization using sensor inputs and controlling the motors.

Companion Computer : Manages high-level tasks such as navigation, communication with ground stations, and additional sensor processing. (Not Implemented)

### Key Functions:
setup(): Initializes the quadcopter's sensors, motors, and other peripherals.
loop(): Continuously runs the stabilization control loop.

### Classes:
Barometer,IMU,ESC,HAL - Hardware 
PID Control - Controller code 
main - Main Program to run
