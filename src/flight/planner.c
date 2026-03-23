#include "planner.h"
#include "autopilot.h"
#include "FreeRTOS.h"
#include "task.h"
#include "../drivers/lt_logger/lt_logger.h"
#include <stdbool.h>
#include <math.h>

static planner_state_t current_planner_state = PLANNER_STILL_ON_PARACHUTE;
static bool planner_running = false;
static TickType_t state_timer = 0;
static TickType_t stability_timer = 0;
static bool arrival_logged = false;

// Configuration / Constants
#define CUTOFF_HOLD_TIME_MS 1000
#define RUNAWAY_HOLD_TIME_MS 5000
#define STABILITY_TIME_MS 2000
#define WAYPOINT_STABILITY_TIME_MS 2000
#define CHECK_DELAY_MS 200

#define RUNAWAY_PITCH -20.0f
#define STABLE_THROTTLE 30.0f
#define LANDING_THROTTLE_FALL_THRESHOLD 0.1f
#define WAYPOINT_ACCURACY_RADIUS 4.0f
#define LANDING_ALT_THRESHOLD 3.0f

// Predefined GPS waypoints (examples)
#define WPT_LAT 52.2297 
#define WPT_LON 21.0122

static float get_dist_to_wpt(double lat1, double lon1) {
    // Basic flat-earth approximation for small distances
    // In a real scenario, use Haversine or projected coords
    // For now, let's assume we have current_lat/lon in autopilot_state
    // float dx = (lat1 - autopilot_state.current_lat) * 111319.0f;
    // float dy = (lon1 - autopilot_state.current_lon) * 111319.0f * cos(lat1 * M_PI / 180.0);
    // return sqrtf(dx*dx + dy*dy);
    return 0.0f; // Placeholder until GPS driver provides current pos
}

static bool is_stable() {
    return (fabsf(autopilot_state.current_roll) < 5.0f && 
            fabsf(autopilot_state.current_pitch) < 5.0f &&
            fabsf(autopilot_state.current_vertical_speed) < 0.5f);
}

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
                send_ap_cmd(FM_READY, 0, 0, 0, 0, 0, 0, 0, 0, 0);
                break;

            case PLANNER_CUTOFF_HOLD:
                if (state_timer == 0) state_timer = xTaskGetTickCount();
                if (xTaskGetTickCount() - state_timer < pdMS_TO_TICKS(CUTOFF_HOLD_TIME_MS)) {
                    send_ap_cmd(FM_GYRO, 0, 0, 0, RUNAWAY_PITCH, 0, STABLE_THROTTLE, 0, 0, 0);
                } else {
                    planner_set_state(PLANNER_ANGLE_RUNAWAY);
                }
                break;

            case PLANNER_ANGLE_RUNAWAY:
                if (state_timer == 0) state_timer = xTaskGetTickCount();
                send_ap_cmd(FM_ANGLE, 0, RUNAWAY_PITCH, 0, 0, 0, STABLE_THROTTLE, 0, 0, 0);
                if (xTaskGetTickCount() - state_timer > pdMS_TO_TICKS(RUNAWAY_HOLD_TIME_MS)) {
                    planner_set_state(PLANNER_STABILIZE);
                }
                break;

            case PLANNER_STABILIZE:
                send_ap_cmd(FM_GPS_STBL, 0, 0, 0, 0, 0, STABLE_THROTTLE, 100.0f, 0, 0);
                if (is_stable()) {
                    if (stability_timer == 0) stability_timer = xTaskGetTickCount();
                    if (xTaskGetTickCount() - stability_timer > pdMS_TO_TICKS(STABILITY_TIME_MS)) {
                        planner_set_state(PLANNER_WAYPOINT);
                    }
                } else {
                    stability_timer = 0;
                }
                break;

            case PLANNER_WAYPOINT:
                send_ap_cmd(FM_GPS_WPT, 0, 0, 0, 0, 0, STABLE_THROTTLE, 100.0f, WPT_LAT, WPT_LON);
                
                // Logic: reached radius && stable for 2 sec
                if (get_dist_to_wpt(WPT_LAT, WPT_LON) < WAYPOINT_ACCURACY_RADIUS && is_stable()) {
                    if (stability_timer == 0) stability_timer = xTaskGetTickCount();
                    if (xTaskGetTickCount() - stability_timer > pdMS_TO_TICKS(WAYPOINT_STABILITY_TIME_MS)) {
                        planner_set_state(PLANNER_LANDING_SPOT);
                    }
                } else {
                    stability_timer = 0;
                }
                break;

            case PLANNER_LANDING_SPOT:
                send_ap_cmd(FM_GPS_WPT, 0, 0, 0, 0, 0, STABLE_THROTTLE * 0.8f, 20.0f, WPT_LAT, WPT_LON);
                // Threshold of 3m from target altitude
                if (fabsf(autopilot_state.current_alt - 20.0f) < LANDING_ALT_THRESHOLD) {
                    planner_set_state(PLANNER_FINAL_LANDING);
                }
                break;

            case PLANNER_FINAL_LANDING:
                send_ap_cmd(FM_GPS_WPT, 0, 0, 0, 0, 0, STABLE_THROTTLE * 0.5f, 2.0f, WPT_LAT, WPT_LON);
                // vertical speed is very low on the negative side (approaching zero/ground)
                // If were falling, vz is negative. VZ > -0.1 means were nearly not falling
                if (autopilot_state.current_vertical_speed > -0.2f) {
                    planner_set_state(PLANNER_CHECK_LANDED);
                }
                break;

            case PLANNER_CHECK_LANDED:
                if (state_timer == 0) state_timer = xTaskGetTickCount();
                if (xTaskGetTickCount() - state_timer < pdMS_TO_TICKS(CHECK_DELAY_MS)) {
                    // Just wait
                } else {
                    if (fabsf(autopilot_state.current_vertical_speed) < 0.2f) {
                        planner_set_state(PLANNER_DISARM);
                    } else {
                        planner_set_state(PLANNER_LANDING_SPOT);
                    }
                }
                break;

            case PLANNER_DISARM:
                send_ap_cmd(FM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0, 0);
                planner_running = false;
                break;
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(50));
    }
}

void planner_init() {
    LT_I("Initializing Planner...");
    xTaskCreate(planner_task, "planner", 1024, NULL, 2, NULL);
    planner_running = false;
    planner_set_state(PLANNER_STILL_ON_PARACHUTE);
}

void planner_start() {
    planner_running = true;
    planner_set_state(PLANNER_STILL_ON_PARACHUTE);
    LT_I("Planner started");
}

void planner_stop() {
    planner_running = false;
    LT_I("Planner stopped");
}

void planner_set_state(planner_state_t state) {
    LT_I("Changing Planner State to %d", state);
    current_planner_state = state;
    autopilot_state.planner_state = (int)state;
    state_timer = 0; // Reset timers for new state
    stability_timer = 0;
}