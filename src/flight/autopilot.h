#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "queue.h"

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
    float roll_angle;
    float pitch_angle;
    float yaw_rate;
    float roll_rate;
    float pitch_rate;
    
    float throttle;
    float altitude;
    float vertical_speed;
    
    // GPS / Navigation
    double lat;
    double lon;
    float speed;
} autopilot_command_t;

typedef struct {
    flight_mode_t mode;
    
    // Setpoints (Updated by respective PID tasks)
    float target_roll_angle; 
    float target_pitch_angle;
    float target_yaw_rate;    
    float target_roll_rate;   
    float target_pitch_rate;  
    float target_throttle;
    float target_alt;
    float target_vertical_speed;
    
    // GPS Navigation targets
    double target_lat;
    double target_lon;
    float target_yaw;
    
    // Telemetry / Current State
    float current_roll;
    float current_pitch;
    float current_yaw;
    
    float current_roll_rate;
    float current_pitch_rate;
    float current_yaw_rate;

    float current_speed_x; // Relative to vehicle (m/s)
    float current_speed_y; // Relative to vehicle (m/s)
    
    // GPS position and heading
    double current_lat;
    double current_lon;
    float current_heading; // Compass heading from GPS (degrees)

    float current_alt;
    float current_vertical_speed;
    
    // GPS initialization flag
    bool gps_targets_initialized;
    
    // Motor outputs (150-2047)
    int motor_fr;
    int motor_fl;
    int motor_br;
    int motor_bl;

    // Armed status
    bool armed;
    int planner_state;
} autopilot_state_t;

typedef struct {
    int16_t accel[3];
    int16_t gyro[3];
} imu_data_t;

extern autopilot_state_t autopilot_state;

// Separate queues for each control loop to avoid interference
extern QueueHandle_t q_gyro;
extern QueueHandle_t q_angle;
extern QueueHandle_t q_altitude;
extern QueueHandle_t q_gps;
extern QueueHandle_t q_imu_data;

void autopilot_init();
void autopilot_send_command(autopilot_command_t *cmd);
