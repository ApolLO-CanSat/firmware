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

PIDController pid_altitude_rate;
PIDController pid_alt_limiter; // limiter = altitude. only P-term. might make a new func later to make this faster.

PIDController pid_roll_rate;
PIDController pid_roll_angle;
PIDController pid_y_speed; // only P and I, no D.
PIDController pid_y_speed_limiter; // here, limiter = y-position (relative to aircraft rotation). P only like above

PIDController pid_pitch_rate;
PIDController pid_pitch_angle;
PIDController pid_x_speed; // same as above but for x
PIDController pid_x_speed_limiter; // same as above but for x

PIDController pid_yaw_rate; // idk if this should have D term but lets leave it at 0 for now
PIDController pid_yaw_limiter; // limiter = angle, P only.

int pid(float current, float target, PIDController *ctl);