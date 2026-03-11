// kinda a template, i still dont understand freertos lol

// inputs from pids are 0-100, we map 150-2047

#include "mixer.h"

// yaw +/- could get flipped, we gotta still check if we're running props in or props out
int mixer_fr(float throttle, float roll, float pitch, float yaw) {
    int output = (int)(150.0f + ((throttle - roll + pitch + yaw) * (2047.0f - 150.0f) / 100.0f));
    return output;
}
int mixer_fl(float throttle, float roll, float pitch, float yaw) {
    int output = (int)(150.0f + ((throttle + roll + pitch - yaw) * (2047.0f - 150.0f) / 100.0f));
    return output;
}
int mixer_br(float throttle, float roll, float pitch, float yaw) {
    int output = (int)(150.0f + ((throttle - roll - pitch - yaw) * (2047.0f - 150.0f) / 100.0f));
    return output;
}
int mixer_bl(float throttle, float roll, float pitch, float yaw) {
    int output = (int)(150.0f + ((throttle + roll - pitch + yaw) * (2047.0f - 150.0f) / 100.0f));
    return output;
}

