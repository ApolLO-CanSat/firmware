#include "planner.h"
#include "autopilot.h"
#include "FreeRTOS.h"
#include "task.h"
#include "../drivers/lt_logger/lt_logger.h"
#include <stdbool.h>

static planner_state_t current_planner_state = PLANNER_STILL_ON_PARACHUTE;
static bool planner_running = false;
static TickType_t state_timer = 0;

// Configuration / Constants
#define CUTOFF_HOLD_TIME_MS 1000
#define RUNAWAY_PITCH -20.0f
#define STABLE_THROTTLE 30.0f  // Example throttle value
#define LANDING_THROTTLE_FALL_THRESHOLD 0.1f

// Predefined GPS waypoints (examples)
#define WPT_LAT 52.2297 
#define WPT_LON 21.0122

static void send_ap_cmd(flight_mode_t mode, float roll, float pitch, float roll_rate, float pitch_rate, float yaw_rate, float throttle, float alt, double lat, double lon) {
    autopilot_command_t cmd = {
        .mode = mode,
        .roll_angle = roll,
        .pitch_angle = pitch,
        .roll_rate = roll_rate,
        .pitch_rate = pitch_rate,
        .yaw_rate = yaw_rate,
        .throttle = throttle,
        .altitude = alt,
        .lat = lat,
        .lon = lon
    };
    autopilot_send_command(&cmd);
}

void planner_task(void *params) {
    TickType_t last_wake = xTaskGetTickCount();
    
    while (1) {
        if (!planner_running) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        switch (current_planner_state) {
            case PLANNER_STILL_ON_PARACHUTE:
                // State 0: Still on parachute
                send_ap_cmd(FM_READY, 0, 0, 0, 0, 0, 0, 0, 0, 0);
                break;

            case PLANNER_CUTOFF_HOLD:
                // State 1: gyro only runaway
                if (state_timer == 0) state_timer = xTaskGetTickCount();
                
                if (xTaskGetTickCount() - state_timer < pdMS_TO_TICKS(CUTOFF_HOLD_TIME_MS)) {
                    // FM_GYRO uses rates and throttle directly
                    send_ap_cmd(FM_GYRO, 0, 0, 0, RUNAWAY_PITCH, 0, STABLE_THROTTLE, 0, 0, 0);
                } else {
                    current_planner_state = PLANNER_ANGLE_RUNAWAY;
                    state_timer = 0;
                }
                break;

            case PLANNER_ANGLE_RUNAWAY:
                // State 2: angle mode runaway
                send_ap_cmd(FM_ANGLE, 0, RUNAWAY_PITCH, 0, 0, 0, STABLE_THROTTLE, 0, 0, 0);
                break;

            case PLANNER_STABILIZE:
                // State 3: stabilize
                send_ap_cmd(FM_GPS_STBL, 0, 0, 0, 0, 0, STABLE_THROTTLE, 100.0f, 0, 0);
                break;

            case PLANNER_WAYPOINT:
                // State 4: fly to predefined waypoint
                send_ap_cmd(FM_GPS_WPT, 0, 0, 0, 0, 0, STABLE_THROTTLE, 100.0f, WPT_LAT, WPT_LON);
                break;

            case PLANNER_LANDING_SPOT:
                // State 5: landing on the spot
                send_ap_cmd(FM_GPS_WPT, 0, 0, 0, 0, 0, STABLE_THROTTLE * 0.8f, 20.0f, WPT_LAT, WPT_LON);
                break;

            case PLANNER_FINAL_LANDING:
                // State 6: final landing
                send_ap_cmd(FM_GPS_WPT, 0, 0, 0, 0, 0, STABLE_THROTTLE * 0.5f, 2.0f, WPT_LAT, WPT_LON);
                break;

            case PLANNER_CHECK_LANDED:
                // State 7: check if still falling
                if (autopilot_state.current_vertical_speed > -0.5f && autopilot_state.current_vertical_speed < 0.5f) {
                    current_planner_state = PLANNER_DISARM;
                } else {
                    current_planner_state = PLANNER_LANDING_SPOT;
                }
                break;

            case PLANNER_DISARM:
                // State 8: disarm
                send_ap_cmd(FM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0, 0);
                planner_running = false;
                break;
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(50)); // 20Hz planner
    }
}

void planner_init() {
    LT_I("Initializing Planner...");
    xTaskCreate(planner_task, "planner", 1024, NULL, 2, NULL);
}

void planner_start() {
    planner_running = true;
    current_planner_state = PLANNER_STILL_ON_PARACHUTE;
    LT_I("Planner started");
}

void planner_stop() {
    planner_running = false;
    LT_I("Planner stopped");
}