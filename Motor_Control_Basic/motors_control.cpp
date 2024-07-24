#include <iostream>
#include <vector>
#include <cmath>

// PID Controller class
class PIDController 
{
public:
    PIDController(float kp, float ki, float kd)
        : kp(kp), ki(ki), kd(kd), previous_error(0), integral(0) {}

    float compute(float error, float dt)
    {
        integral += error * dt;
        float derivative = (error - previous_error) / dt;
        previous_error = error;
        return kp * error + ki * integral + kd * derivative;
    }

private:
    float kp, ki, kd;
    float previous_error;
    float integral;
};

// Quadcopter class
class Quadcopter {
public:
    Quadcopter(float kp_roll, float ki_roll, float kd_roll,float kp_pitch, float ki_pitch, float kd_pitch,float kp_yaw, float ki_yaw, float kd_yaw)
        : motor_speeds(4, 0), gyro_data(3, 0), accel_data(3, 0),
          thrust(0), roll(0), pitch(0), yaw(0),
          pid_roll(kp_roll, ki_roll, kd_roll),
          pid_pitch(kp_pitch, ki_pitch, kd_pitch),
          pid_yaw(kp_yaw, ki_yaw, kd_yaw) {}

    void set_motor_speeds(const std::vector<float>& speeds)
    {
        motor_speeds = speeds;
        std::cout << "Motor speeds set to: ";
        for (auto speed : motor_speeds)
        {
            std::cout << speed << " ";
        }
        std::cout << std::endl;
    }

    void update_sensor_data(const std::vector<float>& gyro, const std::vector<float>& accel) {
        gyro_data = gyro;
        accel_data = accel;
    }

    void set_controls(float thrust, float roll, float pitch, float yaw) {
        this->thrust = thrust;
        this->roll = roll;
        this->pitch = pitch;
        this->yaw = yaw;
    }

    void stabilize(float dt)
    {
        // Calculate errors based on sensor data (desired state is level flight)
        float roll_error = roll - gyro_data[0];
        float pitch_error = pitch - gyro_data[1];
        float yaw_error = yaw - gyro_data[2];

        // Calculate motor speed adjustments using PID controllers
        float roll_adjustment = pid_roll.compute(roll_error, dt);
        float pitch_adjustment = pid_pitch.compute(pitch_error, dt);
        float yaw_adjustment = pid_yaw.compute(yaw_error, dt);

        // Base thrust applied equally to all motors
        float base_thrust = thrust;

        // Adjust motor speeds to achieve desired roll, pitch, and yaw values
        set_motor_speeds
        (
            {
            base_thrust + roll_adjustment - pitch_adjustment + yaw_adjustment,  // Motor 1
            base_thrust - roll_adjustment - pitch_adjustment - yaw_adjustment,  // Motor 2
            base_thrust + roll_adjustment + pitch_adjustment - yaw_adjustment,  // Motor 3
            base_thrust - roll_adjustment + pitch_adjustment + yaw_adjustment   // Motor 4
            }
        );
    }

private:
    std::vector<float> motor_speeds;
    std::vector<float> gyro_data;
    std::vector<float> accel_data;
    float thrust;
    float roll;
    float pitch;
    float yaw;
    PIDController pid_roll;
    PIDController pid_pitch;
    PIDController pid_yaw;
};

int main()
{
    // PID parameters random set
    float kp_roll = 1.0, ki_roll = 0.0, kd_roll = 0.1;
    float kp_pitch = 1.0, ki_pitch = 0.0, kd_pitch = 0.1;
    float kp_yaw = 1.0, ki_yaw = 0.0, kd_yaw = 0.1;

    // Create Quadcopter instance
    Quadcopter quadcopter(kp_roll, ki_roll, kd_roll, kp_pitch, ki_pitch, kd_pitch, kp_yaw, ki_yaw, kd_yaw);

    //sensor data
    std::vector<float> gyro_data = {0.1, -0.2, 0.05};
    std::vector<float> accel_data = {0.0, 0.0, 9.8};

    //Update sensor data
    quadcopter.update_sensor_data(gyro_data, accel_data);

    // Desired control values
    float thrust = 50.0;
    float roll = 0.0;
    float pitch = 0.0;
    float yaw = 0.0;

    // Set controls
    quadcopter.set_controls(thrust, roll, pitch, yaw);

    // Time step
    float dt = 0.01;

    // Stabilize the quadcopter
    quadcopter.stabilize(dt);

    return 0;
}
