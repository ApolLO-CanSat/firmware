#include "dshot.h"
#include "autopilot.h"
#include "../drivers/lt_logger/lt_logger.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"

// Include the DShot PIO program header
#include "dshot_encoder.pio.h"

// Queue for dshot commands
QueueHandle_t q_dshot = NULL;

// GPIO pins for motors: Front-Right, Front-Left, Back-Right, Back-Left
#define DSHOT_PIN_FR 20
#define DSHOT_PIN_FL 21
#define DSHOT_PIN_BR 22
#define DSHOT_PIN_BL 23

// DShot output frequency (1kHz for responsive motor control)
#define DSHOT_LOOP_HZ 1000

// PIO state machine info for each motor
typedef struct {
    PIO pio;
    uint pio_offset;
    int pio_sm;
    uint gpio_pin;
} dshot_motor_t;

static dshot_motor_t motors[4] = {
    {.gpio_pin = DSHOT_PIN_FR},
    {.gpio_pin = DSHOT_PIN_FL},
    {.gpio_pin = DSHOT_PIN_BR},
    {.gpio_pin = DSHOT_PIN_BL}
};

static bool dshot_running = false;
static bool dshot_program_loaded = false;
static uint dshot_program_offset = 0;

// DShot constants
#define DSHOT_MIN_THROTTLE 48
#define DSHOT_MAX_THROTTLE 2047

// Encode a DShot command value with checksum
static uint16_t dshot_encode_command(uint16_t throttle) {
    // Clamp to valid range
    if (throttle < DSHOT_MIN_THROTTLE) throttle = DSHOT_MIN_THROTTLE;
    if (throttle > DSHOT_MAX_THROTTLE) throttle = DSHOT_MAX_THROTTLE;
    
    // Shift for telemetry bit (0)
    uint16_t c = throttle << 1;
    
    // Calculate and add checksum (4 bits)
    uint16_t checksum = (c ^ (c >> 4) ^ (c >> 8)) & 0x0F;
    c = (c << 4) | checksum;
    
    return c;
}

// Send DShot command to a motor via PIO
static void dshot_send_motor(int motor_idx, uint16_t throttle) {
    if (motor_idx >= 4) return;
    
    dshot_motor_t *motor = &motors[motor_idx];
    if (motor->pio_sm < 0) return;
    
    uint16_t encoded = dshot_encode_command(throttle);
    pio_sm_put_blocking(motor->pio, motor->pio_sm, encoded);
}

// --- DSHOT TASK (1kHz) ---
void task_dshot(void *params) {
    TickType_t last_wake = xTaskGetTickCount();
    dshot_command_t cmd;
    
    while (1) {
        // Check for dshot commands (start/stop/direct)
        if (xQueueReceive(q_dshot, &cmd, 0) == pdTRUE) {
            switch (cmd.cmd_type) {
                case DSHOT_CMD_START:
                    if (!dshot_running) {
                        dshot_running = true;
                        LT_I("DShot motors started");
                    }
                    break;
                    
                case DSHOT_CMD_STOP:
                    if (dshot_running) {
                        dshot_send_motor(0, 0);
                        dshot_send_motor(1, 0);
                        dshot_send_motor(2, 0);
                        dshot_send_motor(3, 0);
                        dshot_running = false;
                        LT_I("DShot motors stopped");
                    }
                    break;
                    
                case DSHOT_CMD_DIRECT:
                    // Send direct throttle values
                    dshot_send_motor(0, cmd.motor_fr);
                    dshot_send_motor(1, cmd.motor_fl);
                    dshot_send_motor(2, cmd.motor_br);
                    dshot_send_motor(3, cmd.motor_bl);
                    break;
                    
                default:
                    break;
            }
        }
        
        // When running and in armed mode, pull throttle from autopilot_state
        if (dshot_running && autopilot_state.armed && autopilot_state.mode >= FM_GYRO) {
            // Send motor values from autopilot state (already computed by mixer)
            uint16_t fr = (uint16_t)autopilot_state.motor_fr;
            uint16_t fl = (uint16_t)autopilot_state.motor_fl;
            uint16_t br = (uint16_t)autopilot_state.motor_br;
            uint16_t bl = (uint16_t)autopilot_state.motor_bl;
            
            dshot_send_motor(0, fr);
            dshot_send_motor(1, fl);
            dshot_send_motor(2, br);
            dshot_send_motor(3, bl);
        } else if (dshot_running && !autopilot_state.armed) {
            // Disarmed: send minimum throttle to all motors
            dshot_send_motor(0, DSHOT_MIN_THROTTLE);
            dshot_send_motor(1, DSHOT_MIN_THROTTLE);
            dshot_send_motor(2, DSHOT_MIN_THROTTLE);
            dshot_send_motor(3, DSHOT_MIN_THROTTLE);
        }
        
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000 / DSHOT_LOOP_HZ));
    }
}

void dshot_init(void) {
    LT_I("Initializing DShot motor control on pins %d,%d,%d,%d", 
         DSHOT_PIN_FR, DSHOT_PIN_FL, DSHOT_PIN_BR, DSHOT_PIN_BL);

    if (!dshot_program_loaded) {
        dshot_program_offset = pio_add_program(pio0, &dshot_encoder_program);
        dshot_program_loaded = true;
    }
    
    // Initialize each motor's PIO state machine
    for (int i = 0; i < 4; i++) {
        dshot_motor_t *motor = &motors[i];
        
        // Use PIO0 with automatic SM selection
        motor->pio = pio0;
        motor->pio_offset = dshot_program_offset;
        motor->pio_sm = pio_claim_unused_sm(motor->pio, false);
        
        if (motor->pio_sm < 0) {
            LT_E("Failed to claim PIO SM for motor %d", i);
            continue;
        }
        
        // Initialize the PIO state machine for DShot on this GPIO
        dshot_encoder_program_init(motor->pio, motor->pio_sm, motor->pio_offset, motor->gpio_pin, true);
        
        LT_I("Motor %d (GPIO%d) initialized", i, motor->gpio_pin);
    }
    
    // Create queue for dshot commands
    q_dshot = xQueueCreate(5, sizeof(dshot_command_t));
    
    // Create dshot task
    xTaskCreate(task_dshot, "dshot", 1024, NULL, 3, NULL);
    
    LT_I("DShot initialized and ready");
}

void dshot_send_command(dshot_command_t *cmd) {
    if (q_dshot) {
        xQueueSend(q_dshot, cmd, 0);
    }
}
