#include "autopilot.h"
#include "pid.h"
#include "mixer.h"
#include "dshot.h"
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
    .target_throttle = 0.0f,
    .target_lat = 0.0,
    .target_lon = 0.0,
    .target_yaw = 0.0f,
    .current_lat = 0.0,
    .current_lon = 0.0,
    .current_heading = 0.0f,
    .gps_targets_initialized = false
};

QueueHandle_t q_gyro = NULL;
QueueHandle_t q_angle = NULL;
QueueHandle_t q_altitude = NULL;
QueueHandle_t q_gps = NULL;
QueueHandle_t q_imu_data = NULL;

// Internal smoothing/filtering
static float gyro_filtered[3] = {0,0,0};
static float accel_filtered[3] = {0,0,0};

static float alpha_gyro = 0.1f; // Simple LPF alpha
static float alpha_accel = 0.05f;

// GPS navigation state
static float last_target_lat = 0.0;
static float last_target_lon = 0.0;
static TickType_t yaw_limiter_stable_ticks = 0;
static bool yaw_limiter_active = true;

// Task loop constants
#define GYRO_LOOP_HZ 1000
#define ANGLE_LOOP_HZ 250
#define ALTI_LOOP_HZ 150
#define GPS_LOOP_HZ 5

// --- GYRO TASK (1kHz) ---
void task_gyro(void *params) {
    TickType_t last_wake = xTaskGetTickCount();
    int16_t cal_accel[3], cal_gyro[3], cal_temp;
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
                d_imu_read_cal(cal_accel, cal_gyro, &cal_temp);

                // Push accel and gyro data to IMU queue for other tasks
                imu_data_t imu_msg;
                for(int i=0; i<3; i++) {
                    imu_msg.accel[i] = cal_accel[i];
                    imu_msg.gyro[i] = cal_gyro[i];
                }
                xQueueSend(q_imu_data, &imu_msg, 0);
                
                for (int i=0; i<3; i++) {
                    gyro_filtered[i] = gyro_filtered[i] * (1.0f - alpha_gyro) + (float)cal_gyro[i] * alpha_gyro;
                }

                autopilot_state.current_roll_rate = gyro_filtered[0];
                autopilot_state.current_pitch_rate = gyro_filtered[1];
                autopilot_state.current_yaw_rate = gyro_filtered[2];

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
    imu_data_t imu_data;
    
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

        // Receive IMU data from the 1kHz task
        while (xQueueReceive(q_imu_data, &imu_data, 0) == pdTRUE) {
            // Roll: atan2(ay, az)
            // Pitch: atan2(-ax, sqrt(ay^2 + az^2))
            float roll_acc = atan2f((float)imu_data.accel[1], (float)imu_data.accel[2]) * 57.29578f;
            float pitch_acc = atan2f(-(float)imu_data.accel[0], sqrtf((float)imu_data.accel[1]*(float)imu_data.accel[1] + (float)imu_data.accel[2]*(float)imu_data.accel[2])) * 57.29578f;

            // Apply LPF to smooth out acceleration noise (alpha_accel)
            autopilot_state.current_roll = autopilot_state.current_roll * (1.0f - alpha_accel) + roll_acc * alpha_accel;
            autopilot_state.current_pitch = autopilot_state.current_pitch * (1.0f - alpha_accel) + pitch_acc * alpha_accel;
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
    int32_t cal_temp, cal_press;
    float last_alt = 0.0f;
    autopilot_command_t cmd;

    while (1) {
        if (xQueueReceive(q_altitude, &cmd, 0) == pdTRUE) {
             // In altitude hold modes, we take altitude setpoint
             autopilot_state.target_alt = cmd.altitude;
        }

        d_baro_read(&cal_temp, &cal_press);
        float pressure_hpa = cal_press / 100.0f;
        // Formula: 44330.0 * (1.0 - pow(pressure_hpa / 1013.25, 1.0 / 5.255))
        float current_alt_raw = 44330.0f * (1.0f - powf(pressure_hpa / 1013.25f, 1.0f / 5.255f));
        
        // Use complementary or LPF filter for altitude
        autopilot_state.current_alt = autopilot_state.current_alt * 0.9f + current_alt_raw * 0.1f;
                
        float dt = 1.0f / ALTI_LOOP_HZ;
        float current_vs_raw = (autopilot_state.current_alt - last_alt) / dt;
        autopilot_state.current_vertical_speed = autopilot_state.current_vertical_speed * 0.9f + current_vs_raw * 0.1f;
        last_alt = autopilot_state.current_alt;

        // If high level mode, update throttle based on alti PID
        if (autopilot_state.mode >= FM_GPS_STBL) {
             // autopilot_state.target_throttle = pid(autopilot_state.current_alt, autopilot_state.target_alt, &pid_alt);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000 / ALTI_LOOP_HZ));
    }
}

// Placeholder GPS reading function
// TODO: Replace with actual GPS module driver calls
typedef struct {
    double latitude;
    double longitude;
    float horizontal_speed_x; // m/s
    float horizontal_speed_y; // m/s
    float compass_heading;    // degrees (0-360)
    bool fix_valid;
} gps_data_t;

static gps_data_t placeholder_gps_read(void) {
    gps_data_t gps = {
        .latitude = 0.0,
        .longitude = 0.0,
        .horizontal_speed_x = 0.0f,
        .horizontal_speed_y = 0.0f,
        .compass_heading = 0.0f,
        .fix_valid = false
    };
    // TODO: Implement actual GPS reading from GPS module
    // gps.latitude = read_gps_latitude();
    // gps.longitude = read_gps_longitude();
    // gps.horizontal_speed_x = read_gps_speed_x();
    // gps.horizontal_speed_y = read_gps_speed_y();
    // gps.compass_heading = read_compass_heading();
    // gps.fix_valid = check_gps_fix();
    return gps;
}

// --- GPS/NAVIGATION TASK (5Hz) ---
void task_gps(void *params) {
    TickType_t last_wake = xTaskGetTickCount();
    autopilot_command_t cmd;
    
    while (1) {
        // Read GPS data from sensor
        gps_data_t gps = placeholder_gps_read();
        
        if (gps.fix_valid) {
            // Update current GPS position and heading
            autopilot_state.current_lat = gps.latitude;
            autopilot_state.current_lon = gps.longitude;
            autopilot_state.current_heading = gps.compass_heading;
            autopilot_state.current_speed_x = gps.horizontal_speed_x;
            autopilot_state.current_speed_y = gps.horizontal_speed_y;
            
            // Initialize target lat/lon to current position if not yet set
            // This prevents drift to 0,0 if targets are never commanded
            if (!autopilot_state.gps_targets_initialized) {
                autopilot_state.target_lat = gps.latitude;
                autopilot_state.target_lon = gps.longitude;
                autopilot_state.gps_targets_initialized = true;
                last_target_lat = gps.latitude;
                last_target_lon = gps.longitude;
            }
        }
        
        // Process GPS command updates
        if (xQueueReceive(q_gps, &cmd, 0) == pdTRUE) {
            if (autopilot_state.mode >= FM_GPS_STBL) {
                // Check if waypoint changed - if so, re-enable yaw limiter
                if (cmd.lat != last_target_lat || cmd.lon != last_target_lon) {
                    yaw_limiter_active = true;
                    yaw_limiter_stable_ticks = 0;
                    last_target_lat = cmd.lat;
                    last_target_lon = cmd.lon;
                }
                // Update GPS targets from command
                autopilot_state.target_lat = cmd.lat;
                autopilot_state.target_lon = cmd.lon;
            }
        }

        if (autopilot_state.mode >= FM_GPS_STBL && gps.fix_valid) {
            // Simple position error calculation (will be more accurate with proper coordinate transform)
            double lat_error = autopilot_state.target_lat - autopilot_state.current_lat;
            double lon_error = autopilot_state.target_lon - autopilot_state.current_lon;
            
            // Scale lat/lon error to approximate meters (very simplified)
            // At equator: 1 degree = ~111,000 meters
            float lat_error_m = (float)lat_error * 111000.0f;
            float lon_error_m = (float)lon_error * 111000.0f;
            
            // X speed control: limiter first, then speed PID
            // Limiters constrain the ramp rate for position error input
            int limited_desire_speed_x = pid(autopilot_state.current_speed_x, lat_error_m, &pid_x_speed_limiter);
            // Speed PID: current speed vs limited desired speed
            int target_speed_x = pid(autopilot_state.current_speed_x, (float)limited_desire_speed_x, &pid_x_speed);
            autopilot_state.target_pitch_rate = (float)target_speed_x;
            
            // Y speed control: limiter first, then speed PID
            int limited_desire_speed_y = pid(autopilot_state.current_speed_y, lon_error_m, &pid_y_speed_limiter);
            // Speed PID: current speed vs limited desired speed
            int target_speed_y = pid(autopilot_state.current_speed_y, (float)limited_desire_speed_y, &pid_y_speed);
            autopilot_state.target_roll_rate = -(float)target_speed_y;
            
            // Yaw control: calculate desired yaw heading to target waypoint
            float lat_diff = (float)(autopilot_state.target_lat - autopilot_state.current_lat);
            float lon_diff = (float)(autopilot_state.target_lon - autopilot_state.current_lon);
            float desired_yaw = atan2f(lon_diff, lat_diff) * 57.29578f; // Convert to degrees
            
            // Normalize to 0-360
            if (desired_yaw < 0.0f) desired_yaw += 360.0f;
            if (desired_yaw >= 360.0f) desired_yaw -= 360.0f;
            
            if (yaw_limiter_active) {
                // Yaw limiter: constraints how fast we turn to the waypoint heading
                float limited_desired_yaw = (float)pid(autopilot_state.current_yaw, desired_yaw, &pid_yaw_limiter);
                
                // Yaw rate PID: current yaw rate vs limited desired yaw (as target)
                autopilot_state.target_yaw_rate = (float)pid(autopilot_state.current_yaw_rate, limited_desired_yaw, &pid_yaw_rate);
                
                // Track stability: increment counter each cycle limiter is running
                yaw_limiter_stable_ticks++;
                
                // After 3 seconds stable (15 cycles at 5Hz), disable limiter
                if (yaw_limiter_stable_ticks >= 15) {
                    yaw_limiter_active = false;
                    yaw_limiter_stable_ticks = 0;
                }
            } else {
                // Limiter disabled: directly use desired yaw as yaw rate target
                autopilot_state.target_yaw_rate = (float)pid(autopilot_state.current_yaw_rate, desired_yaw, &pid_yaw_rate);
            }
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000 / GPS_LOOP_HZ));
    }
}

void autopilot_init() {
    LT_I("Initializing Autopilot tasks...");
    
    LT_I("Performing IMU calibration...");
    d_imu_calibrate();
    
    q_gyro = xQueueCreate(5, sizeof(autopilot_command_t));
    q_angle = xQueueCreate(5, sizeof(autopilot_command_t));
    q_altitude = xQueueCreate(5, sizeof(autopilot_command_t));
    q_gps = xQueueCreate(5, sizeof(autopilot_command_t));
    q_imu_data = xQueueCreate(10, sizeof(imu_data_t));

    xTaskCreate(task_gyro, "ap_gyro", 1024, NULL, 5, NULL);
    xTaskCreate(task_angle, "ap_angle", 1024, NULL, 4, NULL);
    xTaskCreate(task_altitude, "ap_alti", 1024, NULL, 3, NULL);
    xTaskCreate(task_gps, "ap_gps", 1024, NULL, 2, NULL);
    
    // Initialize DShot motor control
    dshot_init();
}

void autopilot_send_command(autopilot_command_t *cmd) {
    // Basic routing logic: send to all relevant queues
    // Each task will decide what to pick based on mode
    if (q_gyro) xQueueSend(q_gyro, cmd, 0);
    if (q_angle) xQueueSend(q_angle, cmd, 0);
    if (q_altitude) xQueueSend(q_altitude, cmd, 0);
    if (q_gps) xQueueSend(q_gps, cmd, 0);
}
