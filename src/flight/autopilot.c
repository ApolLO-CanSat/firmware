#include "autopilot.h"
#include "pid.h"
#include "mixer.h"
#include "../drivers/imu/imu.h"
#include "../drivers/baro/baro.h"
#include "../drivers/lt_logger/lt_logger.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <math.h>

autopilot_state_t autopilot_state = {
    .mode = FM_DISARM,
    .armed = false,
    .target_throttle = 0.0f
};

QueueHandle_t q_gyro = NULL;
QueueHandle_t q_angle = NULL;
QueueHandle_t q_altitude = NULL;
QueueHandle_t q_gps = NULL;

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
    autopilot_command_t cmd;
    
    while (1) {
        // Read from gyro-specific queue
        if (xQueueReceive(q_gyro, &cmd, 0) == pdTRUE) {
            autopilot_state.mode = cmd.mode;
            if (autopilot_state.mode == FM_GYRO) {
                autopilot_state.target_roll_rate = cmd.roll_rate;
                autopilot_state.target_pitch_rate = cmd.pitch_rate;
                autopilot_state.target_yaw_rate = cmd.yaw_rate;
                autopilot_state.target_throttle = cmd.throttle;
            }
            autopilot_state.armed = (cmd.mode > FM_DISARM);
        }

        flight_mode_t current_mode = autopilot_state.mode;
        float target_r_rate = autopilot_state.target_roll_rate;
        float target_p_rate = autopilot_state.target_pitch_rate;
        float target_y_rate = autopilot_state.target_yaw_rate;
        float throttle = autopilot_state.target_throttle;

        if (current_mode >= FM_READY) {
            if (current_mode >= FM_GYRO) {
                d_imu_read_raw(raw_accel, raw_gyro, &raw_temp);
                
                for (int i=0; i<3; i++) {
                    gyro_filtered[i] = gyro_filtered[i] * (1.0f - alpha_gyro) + (float)raw_gyro[i] * alpha_gyro;
                }

                int r_out = pid(gyro_filtered[0], target_r_rate, &pid_roll_rate);
                int p_out = pid(gyro_filtered[1], target_p_rate, &pid_pitch_rate);
                int y_out = pid(gyro_filtered[2], target_y_rate, &pid_yaw_rate);

                autopilot_state.motor_fr = mixer_fr(throttle, r_out, p_out, y_out);
                autopilot_state.motor_fl = mixer_fl(throttle, r_out, p_out, y_out);
                autopilot_state.motor_br = mixer_br(throttle, r_out, p_out, y_out);
                autopilot_state.motor_bl = mixer_bl(throttle, r_out, p_out, y_out);
            } else {
                autopilot_state.motor_fr = 150;
                autopilot_state.motor_fl = 150;
                autopilot_state.motor_br = 150;
                autopilot_state.motor_bl = 150;
            }
        } else {
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
    autopilot_command_t cmd;
    while (1) {
        // Read from angle-specific queue
        if (xQueueReceive(q_angle, &cmd, 0) == pdTRUE) {
            if (autopilot_state.mode == FM_ANGLE) {
                autopilot_state.target_roll_angle = cmd.roll_angle;
                autopilot_state.target_pitch_angle = cmd.pitch_angle;
                autopilot_state.target_yaw_rate = cmd.yaw_rate;
                autopilot_state.target_throttle = cmd.throttle;
            }
        }

        if (autopilot_state.mode >= FM_ANGLE) {
            autopilot_state.target_roll_rate = pid(autopilot_state.current_roll, autopilot_state.target_roll_angle, &pid_roll_angle);
            autopilot_state.target_pitch_rate = pid(autopilot_state.current_pitch, autopilot_state.target_pitch_angle, &pid_pitch_angle);
        } else if (autopilot_state.mode == FM_GYRO) {
            // Rate targets are set directly from q_gyro in FM_GYRO
        } else {
            autopilot_state.target_roll_rate = 0.0f; 
            autopilot_state.target_pitch_rate = 0.0f;
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000 / ANGLE_LOOP_HZ));
    }
}

// --- ALTITUDE TASK (150Hz) ---
void task_altitude(void *params) {
    TickType_t last_wake = xTaskGetTickCount();
    int32_t raw_temp, raw_press;
    float last_alt = 0.0f;
    autopilot_command_t cmd;

    while (1) {
        if (xQueueReceive(q_altitude, &cmd, 0) == pdTRUE) {
             // In altitude hold modes, we take altitude setpoint
             autopilot_state.target_alt = cmd.altitude;
             // If not in GPS mode yet, we might still take roll/pitch/yaw from here if angle mode + alti
             if (autopilot_state.mode == FM_ANGLE && cmd.altitude > 0) {
                 // Hybrid mode handling could go here
             }
        }

        d_baro_read(&raw_temp, &raw_press);
        float pressure_hpa = raw_press / 100.0f;
        autopilot_state.current_alt = 44330.0 * (1.0 - pow(pressure_hpa / 1013.25, 1.0 / 5.255));
                
        float dt = 1.0f / ALTI_LOOP_HZ;
        autopilot_state.current_vertical_speed = (autopilot_state.current_alt - last_alt) / dt;
        last_alt = autopilot_state.current_alt;

        // If high level mode, update throttle based on alti PID
        if (autopilot_state.mode >= FM_GPS_STBL) {
             // autopilot_state.target_throttle = pid(autopilot_state.current_alt, autopilot_state.target_alt, &pid_alt);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000 / ALTI_LOOP_HZ));
    }
}

// --- GPS/NAVIGATION TASK (5Hz) ---
void task_gps(void *params) {
    TickType_t last_wake = xTaskGetTickCount();
    autopilot_command_t cmd;
    
    while (1) {
        if (xQueueReceive(q_gps, &cmd, 0) == pdTRUE) {
            if (autopilot_state.mode >= FM_GPS_STBL) {
                // Take GPS targets
                // autopilot_state.target_lat = cmd.lat;
                // autopilot_state.target_lon = cmd.lon;
            }
        }

        if (autopilot_state.mode >= FM_GPS_STBL) {
            // Update roll/pitch angles based on GPS position/velocity PID
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000 / GPS_LOOP_HZ));
    }
}

void autopilot_init() {
    LT_I("Initializing Autopilot tasks...");
    
    q_gyro = xQueueCreate(5, sizeof(autopilot_command_t));
    q_angle = xQueueCreate(5, sizeof(autopilot_command_t));
    q_altitude = xQueueCreate(5, sizeof(autopilot_command_t));
    q_gps = xQueueCreate(5, sizeof(autopilot_command_t));

    xTaskCreate(task_gyro, "ap_gyro", 1024, NULL, 5, NULL);
    xTaskCreate(task_angle, "ap_angle", 1024, NULL, 4, NULL);
    xTaskCreate(task_altitude, "ap_alti", 1024, NULL, 3, NULL);
    xTaskCreate(task_gps, "ap_gps", 1024, NULL, 2, NULL);
}

void autopilot_send_command(autopilot_command_t *cmd) {
    // Basic routing logic: send to all relevant queues
    // Each task will decide what to pick based on mode
    if (q_gyro) xQueueSend(q_gyro, cmd, 0);
    if (q_angle) xQueueSend(q_angle, cmd, 0);
    if (q_altitude) xQueueSend(q_altitude, cmd, 0);
    if (q_gps) xQueueSend(q_gps, cmd, 0);
}
