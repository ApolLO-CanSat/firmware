#include "autopilot.h"
#include "pid.h"
#include "mixer.h"
#include "../drivers/imu/imu.h"
#include "../drivers/baro/baro.h"
#include "../drivers/lt_logger/lt_logger.h"

#include "FreeRTOS.h"
#include "task.h"
#include <math.h>

autopilot_state_t autopilot_state = {
    .mode = FM_DISARM,
    .armed = false,
    .target_throttle = 0.0f
};

// Internal smoothing/filtering
static float gyro_filtered[3] = {0,0,0};
static float accel_filtered[3] = {0,0,0};

static float alpha_gyro = 0.1f; // Simple LPF alpha
static float alpha_accel = 0.05f;

// Task loop constants
#define GYRO_LOOP_HZ 1000
#define ANGLE_LOOP_HZ 250
#define ALTI_LOOP_HZ 150
#define GPS_LOOP_HZ 5

// --- GYRO TASK (1kHz) ---
void task_gyro(void *params) {
    TickType_t last_wake = xTaskGetTickCount();
    int16_t raw_accel[3], raw_gyro[3], raw_temp;
    
    while (1) {
        if (autopilot_state.mode >= FM_GYRO) {
            d_imu_read_raw(raw_accel, raw_gyro, &raw_temp);
            
            // Simple LPF for gyro
            for (int i=0; i<3; i++) {
                gyro_filtered[i] = gyro_filtered[i] * (1.0f - alpha_gyro) + (float)raw_gyro[i] * alpha_gyro;
            }

            // In FM_GYRO, we PID against target rates
            // Usually, target_roll/pitch would be converted to deg/s
            int r_out = pid(gyro_filtered[0], autopilot_state.target_roll, &pid_roll_rate);
            int p_out = pid(gyro_filtered[1], autopilot_state.target_pitch, &pid_pitch_rate);
            int y_out = pid(gyro_filtered[2], autopilot_state.target_yaw, &pid_yaw_rate);

            // Mix and set motor values
            autopilot_state.motor_fr = mixer_fr(autopilot_state.target_throttle, r_out, p_out, y_out);
            autopilot_state.motor_fl = mixer_fl(autopilot_state.target_throttle, r_out, p_out, y_out);
            autopilot_state.motor_br = mixer_br(autopilot_state.target_throttle, r_out, p_out, y_out);
            autopilot_state.motor_bl = mixer_bl(autopilot_state.target_throttle, r_out, p_out, y_out);
        } else {
            // DISARM (0) or READY (1) -> Motors off or idle
            int idle_val = (autopilot_state.mode == FM_READY) ? 150 : 0;
            autopilot_state.motor_fr = idle_val;
            autopilot_state.motor_fl = idle_val;
            autopilot_state.motor_br = idle_val;
            autopilot_state.motor_bl = idle_val;
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000 / GYRO_LOOP_HZ));
    }
}

// --- ANGLE TASK (250Hz) ---
void task_angle(void *params) {
    TickType_t last_wake = xTaskGetTickCount();
    while (1) {
        if (autopilot_state.mode >= FM_ANGLE) {
            // Placeholder: Fuse Accel + Gyro to get current_roll/pitch
            // For now, let's pretend we have a simple integrator
            // In a real drone, this would be a Complementary Filter or Kalman Filter
            
            // Update rate targets for the 1kHz loop
            autopilot_state.target_roll = pid(autopilot_state.current_roll, 0.0f /* setpoint */, &pid_roll_angle);
            autopilot_state.target_pitch = pid(autopilot_state.current_pitch, 0.0f /* setpoint */, &pid_pitch_angle);
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000 / ANGLE_LOOP_HZ));
    }
}

// --- ALTITUDE TASK (150Hz) ---
void task_altitude(void *params) {
    TickType_t last_wake = xTaskGetTickCount();
    int32_t raw_temp, raw_press;
    float last_alt = 0.0f;

    while (1) {
        if (autopilot_state.mode >= FM_ANGLE) { // Altitude often coupled with angle/stable modes
            d_baro_read(&raw_temp, &raw_press);
            // Conversion to meters (simplified)
            float pressure_hpa = raw_press / 100.0f;
            autopilot_state.current_alt = 44330.0 * (1.0 - pow(pressure_hpa / 1013.25, 1.0 / 5.255));
            
            float dt = 1.0f / ALTI_LOOP_HZ;
            autopilot_state.current_vertical_speed = (autopilot_state.current_alt - last_alt) / dt;
            last_alt = autopilot_state.current_alt;

            // Alt limiter (Position P) -> Alt rate (Velocity PID)
            float rate_target = pid(autopilot_state.current_alt, autopilot_state.target_alt, &pid_alt_limiter);
            float throttle_adjust = pid(autopilot_state.current_vertical_speed, rate_target, &pid_altitude_rate);
            
            // Adjust base throttle (this is simplified)
            // autopilot_state.target_throttle = throttle_adjust; 
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000 / ALTI_LOOP_HZ));
    }
}

// --- GPS/NAVIGATION TASK (5Hz) ---
void task_gps(void *params) {
    TickType_t last_wake = xTaskGetTickCount();
    while (1) {
        if (autopilot_state.mode >= FM_GPS_STBL) {
            // Calculate XY speeds and feed into speed limiters
            // Output targets for Pitch/Roll (Angle loop)
            // Yaw also handled here usually if heading-constrained
            
            // pid(current_x_speed, target_x_speed, &pid_x_speed_limiter);
            // ...
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000 / GPS_LOOP_HZ));
    }
}

void autopilot_init() {
    LT_I("Initializing Autopilot tasks...");
    
    xTaskCreate(task_gyro, "ap_gyro", 1024, NULL, 5, NULL);
    xTaskCreate(task_angle, "ap_angle", 1024, NULL, 4, NULL);
    xTaskCreate(task_altitude, "ap_alti", 1024, NULL, 3, NULL);
    xTaskCreate(task_gps, "ap_gps", 1024, NULL, 2, NULL);
}
