#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "semphr.h"

typedef enum {
    FM_DISARM = 0,
    FM_READY = 1,
    FM_GYRO = 2,
    FM_ANGLE = 3,
    FM_GPS_STBL = 4,
    FM_GPS_WPT = 5
} flight_mode_t;

typedef struct {
    flight_mode_t mode;
    
    // Setpoints
    float target_roll_angle;    // Target angle in degrees
    float target_pitch_angle;   // Target angle in degrees
    float target_yaw_rate;      // Target yaw rate
    float target_roll_rate;     // Calculated by Angle task for Gyro task
    float target_pitch_rate;    // Calculated by Angle task for Gyro task
    float target_throttle;
    float target_alt;
    
    // Telemetry / Current State (for other threads)
    float current_roll;
    float current_pitch;
    float current_yaw;
    float current_alt;
    float current_vertical_speed;
    
    // Motor outputs (150-2047)
    int motor_fr;
    int motor_fl;
    int motor_br;
    int motor_bl;

    // Armed status
    bool armed;

    // Mutex for state synchronization
    SemaphoreHandle_t mutex;
} autopilot_state_t;

extern autopilot_state_t autopilot_state;

void autopilot_init();
