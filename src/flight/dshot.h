#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "queue.h"

typedef enum {
    DSHOT_CMD_START,        // Start pulling from autopilot throttle values
    DSHOT_CMD_STOP,         // Stop all motors (set to 0)
    DSHOT_CMD_DIRECT,       // Set direct throttle values (0-2047 for each motor)
} dshot_command_type_t;

typedef struct {
    dshot_command_type_t cmd_type;
    uint16_t motor_fr;      // For DSHOT_CMD_DIRECT
    uint16_t motor_fl;
    uint16_t motor_br;
    uint16_t motor_bl;
} dshot_command_t;

extern QueueHandle_t q_dshot;

// Initialize dshot encoders (called by autopilot_init)
void dshot_init(void);

// Send a dshot command
void dshot_send_command(dshot_command_t *cmd);

// Task for dshot (created internally)
void task_dshot(void *params);
