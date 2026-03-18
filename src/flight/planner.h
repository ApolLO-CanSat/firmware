#pragma once

#include "autopilot.h"

typedef enum {
    PLANNER_STILL_ON_PARACHUTE = 0,
    PLANNER_CUTOFF_HOLD = 1,
    PLANNER_ANGLE_RUNAWAY = 2,
    PLANNER_STABILIZE = 3,
    PLANNER_WAYPOINT = 4,
    PLANNER_LANDING_SPOT = 5,
    PLANNER_FINAL_LANDING = 6,
    PLANNER_CHECK_LANDED = 7,
    PLANNER_DISARM = 8
} planner_state_t;

void planner_init();
void planner_task(void *params);
void planner_start();
void planner_stop();
void planner_set_state(planner_state_t state);