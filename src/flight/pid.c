#include "pid.h"

/*
===== TUNING GUIDE =====
Start with Kp, set Ki and Kd to 0. Increase Kp until you get a good response, but not too much overshoot/oscillation. (For autonomous, it can be a little bit smooth)
Then increase Ki until you get rid of any steady-state error, but not too much overshoot or oscillation.
Finally, increase Kd to reduce overshoot and improve stability, but not too much to cause noise sensitivity. Increase IN SMALL STEPS, warning below.

WARNING: Too high Kd can damage the motors! The quad can still fly with Kd=0, so if you don't know what you're doing, just leave it at 0.
After a flight, touch the motors. If you can't keep your finger on them for more than a second, LOWER KD IMMEDIATELY.
Too high Kd can cause oscillations and unnecessary heat. This will burn out the motors eventually, and can cause a crash if it happens mid-flight.
You can also try out the default, safe values.


What do the values mean?
Kp controls the PRESENT - reacts to immediate changes
Ki controls the PAST - reacts to accumulated error over time
Kd controls the FUTURE - reacts to the rate of change of the error (DANGEROUS! DO NOT TUNE IF NOT REQUIRED, AND KEEP IT AS LOW AS POSSIBLE!)
*/





// Altitude control
PIDController pid_altitude_rate = {
    .Kp = 1.2f,
    .Ki = 0.25f,
    .Kd = 0.015f,
    .integral = 0.0f,
    .previous_error = 0.0f,
    .out_max = 100.0f,
    .out_min = 0.0f,
    .integral_max = 50.0f,
    .integral_min = -50.0f
};

PIDController pid_alt_limiter = {
    .Kp = 0.8f,
    .Ki = 0.0f,
    .Kd = 0.0f,
    .integral = 0.0f,
    .previous_error = 0.0f,
    .out_max = 2.0f, // max 2m/s
    .out_min = -2.0f, // min -2m/s
    .integral_max = 0.0f, // no integral term
    .integral_min = 0.0f
};



// Pitch control
PIDController pid_pitch_rate = {
    .Kp = 0.08f,
    .Ki = 0.04f,
    .Kd = 0.0015f,
    .integral = 0.0f,
    .previous_error = 0.0f,
    .out_max = 100.0f,
    .out_min = -100.0f,
    .integral_max = 500.0f,
    .integral_min = -500.0f
};

PIDController pid_pitch_angle = {
    .Kp = 6.0f,
    .Ki = 0.0f, // TO BE TUNED!!!
    .Kd = 0.0f, // TO BE TUNED!!!
    .integral = 0.0f,
    .previous_error = 0.0f,
    .out_max = 360.0f, // max 360 degrees/s
    .out_min = -360.0f, // min -360 degrees/s
    .integral_max = 0.0f, // no integral term (FOR NOW!)
    .integral_min = 0.0f
};

PIDController pid_x_speed = {
    .Kp = 4.0f,
    .Ki = 0.8f,
    .Kd = 0.0f,
    .integral = 0.0f,
    .previous_error = 0.0f,
    .out_max = 45.0f, // max 45 degrees
    .out_min = -45.0f, // min -45 degrees
    .integral_max = 15.0f, // max integral 15 degrees/s, in case of wind or something
    .integral_min = -15.0f / 0.8f // same here :D
};

PIDController pid_x_speed_limiter = {
    .Kp = 0.5f,
    .Ki = 0.0f,
    .Kd = 0.0f,
    .integral = 0.0f,
    .previous_error = 0.0f,
    .out_max = 4.0f, // max 4m/s, this is not vertical so we can go a bit faster, as propwash is BELOW us not in front of us lmao rolf xd lol (im fubar, ignore this comment)
    .out_min = -4.0f, // min -4m/s
    .integral_max = 0.0f, // no integral term
    .integral_min = 0.0f
};

// Roll control, those are 2 axis on the same plane so values are literally the same

PIDController pid_roll_rate = {
    .Kp = 0.08f,
    .Ki = 0.04f,
    .Kd = 0.0015f,
    .integral = 0.0f,
    .previous_error = 0.0f,
    .out_max = 100.0f,
    .out_min = -100.0f,
    .integral_max = 500.0f,
    .integral_min = -500.0f
};

PIDController pid_roll_angle = {
    .Kp = 6.0f,
    .Ki = 0.0f, // TO BE TUNED!!!
    .Kd = 0.0f, // TO BE TUNED!!!
    .integral = 0.0f,
    .previous_error = 0.0f,
    .out_max = 360.0f, // max 360 degrees/s
    .out_min = -360.0f, // min -360 degrees/s
    .integral_max = 0.0f, // no integral term (FOR NOW!)
    .integral_min = 0.0f  
};

PIDController pid_y_speed = {
    .Kp = 4.0f,
    .Ki = 0.8f,
    .Kd = 0.0f,
    .integral = 0.0f,
    .previous_error = 0.0f,
    .out_max = 45.0f, // max 45 degrees
    .out_min = -45.0f, // min -45 degrees
    .integral_max = 15.0f, // max integral 15 degrees/s, in case of wind or something
    .integral_min = -15.0f / 0.8f // same here :D
};

PIDController pid_y_speed_limiter = {
    .Kp = 0.5f,
    .Ki = 0.0f,
    .Kd = 0.0f,
    .integral = 0.0f,
    .previous_error = 0.0f,
    .out_max = 4.0f, // max 4m/s, this is not vertical so we can go a bit faster, as propwash is BELOW us not in front of us lmao rolf xd lol (im fubar, ignore this comment)
    .out_min = -4.0f, // min -4m/s
    .integral_max = 0.0f, // no integral term
    .integral_min = 0.0f
};

// Now yaw!

PIDController pid_yaw_rate = {
    .Kp = 0.15f,
    .Ki = 0.10f,
    .Kd = 0.000f, // please dont add D term to yaw. SZYMALA PLEASE DONT. YOU WILL COOK THE HELL OUT OF THE MOTORS. I KNOW YOU WANT TO TUNE IT, BUT PLEASE DONT. JUST LEAVE IT AT 0.
    .integral = 0.0f,
    .previous_error = 0.0f,
    .out_max = 100.0f,
    .out_min = -100.0f,
    .integral_max = 100.0f,
    .integral_min = -100.0f
};

PIDController pid_yaw_limiter = {
    .Kp = 4.0f,
    .Ki = 0.0f,
    .Kd = 0.0f,
    .integral = 0.0f,
    .previous_error = 0.0f,
    .out_max = 180.0f, // max 180 degrees/s
    .out_min = -180.0f, // min -180 degrees/s
    .integral_max = 0.0f,
    .integral_min = 0.0f
};





int pid(float current, float target, PIDController *ctl) {
    float error = target - current; // Calculate error
    ctl->integral += error; // Update integral term
    // Clamp integral term to prevent windup
    if (ctl->integral > ctl->integral_max) {
        ctl->integral = ctl->integral_max;
    } else if (ctl->integral < ctl->integral_min) {
        ctl->integral = ctl->integral_min;
    }
    float derivative = error - ctl->previous_error; // Calculate derivative term
    float output = ctl->Kp * error + ctl->Ki * ctl->integral + ctl->Kd * derivative; // Calculate PID output
    // Clamp output to limits
    if (output > ctl->out_max) {
        output = ctl->out_max;
    } else if (output < ctl->out_min) {
        output = ctl->out_min;
    }
    ctl->previous_error = error; // Update previous error

    return (int)output;
}