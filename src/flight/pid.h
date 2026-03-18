#pragma once

typedef struct PIDController {
    const float Kp;
    const float Ki;
    const float Kd;
    float integral;
    float previous_error; 
    const float out_max;
    const float out_min;
    const float integral_max;
    const float integral_min;
} PIDController;

extern PIDController pid_altitude_rate;
extern PIDController pid_alt_limiter;

extern PIDController pid_roll_rate;
extern PIDController pid_roll_angle;
extern PIDController pid_y_speed;
extern PIDController pid_y_speed_limiter;

extern PIDController pid_pitch_rate;
extern PIDController pid_pitch_angle;
extern PIDController pid_x_speed;
extern PIDController pid_x_speed_limiter;

extern PIDController pid_yaw_rate;
extern PIDController pid_yaw_limiter;

int pid(float current, float target, PIDController *ctl);