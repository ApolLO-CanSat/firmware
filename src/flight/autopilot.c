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
        // LOCK-FREE READ of mode and targets for jitter-free 1kHz performance
        // High-level tasks (Angle/Alti) update these values once per their loop
        flight_mode_t current_mode = autopilot_state.mode;
        float target_r = autopilot_state.target_roll;
        float target_p = autopilot_state.target_pitch;
        float target_y = autopilot_state.target_yaw;
        float throttle = autopilot_state.target_throttle;

        if (current_mode >= FM_READY) {
            if (current_mode >= FM_GYRO) {
                d_imu_read_raw(raw_accel, raw_gyro, &raw_temp);
                
                // Simple LPF for gyro
                for (int i=0; i<3; i++) {
                    gyro_filtered[i] = gyro_filtered[i] * (1.0f - alpha_gyro) + (float)raw_gyro[i] * alpha_gyro;
                }

                // In FM_GYRO, we PID against target rates
                int r_out = pid(gyro_filtered[0], target_r, &pid_roll_rate);
                int p_out = pid(gyro_filtered[1], target_p, &pid_pitch_rate);
                int y_out = pid(gyro_filtered[2], target_y, &pid_yaw_rate);

                // Mix and set motor values (Writing to state is fine as only this task writes motor outputs)
                autopilot_state.motor_fr = mixer_fr(throttle, r_out, p_out, y_out);
                autopilot_state.motor_fl = mixer_fl(throttle, r_out, p_out, y_out);
                autopilot_state.motor_br = mixer_br(throttle, r_out, p_out, y_out);
                autopilot_state.motor_bl = mixer_bl(throttle, r_out, p_out, y_out);
            } else {
                // READY (1) -> Motors idle
                autopilot_state.motor_fr = 150;
                autopilot_state.motor_fl = 150;
                autopilot_state.motor_br = 150;
                autopilot_state.motor_bl = 150;
            }
        } else {
            // DISARM (0) -> Force motors to 0
            autopilot_state.motor_fr = 0;
            autopilot_state.motor_fl = 0;
            autopilot_state.motor_br = 0;
            autopilot_state.motor_bl = 0;
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000 / GYRO_LOOP_HZ));
    }
}

// --- ANGLE TASK (250Hz) ---
void task_angle(void *params) {
    TickType_t last_wake = xTaskGetTickCount();
    while (1) {
        if (autopilot_state.mode >= FM_ANGLE) {
            if (xSemaphoreTake(autopilot_state.mutex, portMAX_DELAY) == pdTRUE) {
                // Re-check inside mutex to be safe against mode changes during wait
                if (autopilot_state.mode >= FM_ANGLE) {
                    autopilot_state.target_roll = pid(autopilot_state.current_roll, 0.0f, &pid_roll_angle);
                    autopilot_state.target_pitch = pid(autopilot_state.current_pitch, 0.0f, &pid_pitch_angle);
                }
                xSemaphoreGive(autopilot_state.mutex);
            }
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
        if (autopilot_state.mode >= FM_ANGLE) {
            if (xSemaphoreTake(autopilot_state.mutex, portMAX_DELAY) == pdTRUE) {
                if (autopilot_state.mode >= FM_ANGLE) {
                    d_baro_read(&raw_temp, &raw_press);
                    float pressure_hpa = raw_press / 100.0f;
                    autopilot_state.current_alt = 44330.0 * (1.0 - pow(pressure_hpa / 1013.25, 1.0 / 5.255));
                    
                    float dt = 1.0f / ALTI_LOOP_HZ;
                    autopilot_state.current_vertical_speed = (autopilot_state.current_alt - last_alt) / dt;
                    last_alt = autopilot_state.current_alt;

                    float rate_target = pid(autopilot_state.current_alt, autopilot_state.target_alt, &pid_alt_limiter);
                }
                xSemaphoreGive(autopilot_state.mutex);
            }
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000 / ALTI_LOOP_HZ));
    }
}

// --- GPS/NAVIGATION TASK (5Hz) ---
void task_gps(void *params) {
    TickType_t last_wake = xTaskGetTickCount();
    while (1) {
        if (autopilot_state.mode >= FM_GPS_STBL) {
            if (xSemaphoreTake(autopilot_state.mutex, portMAX_DELAY) == pdTRUE) {
                if (autopilot_state.mode >= FM_GPS_STBL) {
                    // GPS logic...
                }
                xSemaphoreGive(autopilot_state.mutex);
            }
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000 / GPS_LOOP_HZ));
    }
}

void autopilot_init() {
    LT_I("Initializing Autopilot tasks...");
    
    autopilot_state.mutex = xSemaphoreCreateMutex();
    if (autopilot_state.mutex == NULL) {
        LT_E("Failed to create autopilot mutex");
        return;
    }
    
    xTaskCreate(task_gyro, "ap_gyro", 1024, NULL, 5, NULL);
    xTaskCreate(task_angle, "ap_angle", 1024, NULL, 4, NULL);
    xTaskCreate(task_altitude, "ap_alti", 1024, NULL, 3, NULL);
    xTaskCreate(task_gps, "ap_gps", 1024, NULL, 2, NULL);
}
