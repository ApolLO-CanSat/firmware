#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/binary_info.h"

#include "FreeRTOS.h"
#include "task.h"

#include "drivers/led/led.h"

void blink_task(__unused void *params) {
    bool on = false;
    printf("blink_task starts\n");
    pico_led_init();
    while (true) {
#if configNUMBER_OF_CORES > 1
        static int last_core_id = -1;
        if (portGET_CORE_ID() != last_core_id) {
            last_core_id = portGET_CORE_ID();
            printf("blink task is on core %d\n", last_core_id);
        }
#endif
        pico_set_led(on);
        on = !on;

        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

int main() {
    bi_decl(bi_program_description("ApolLO CanSat firmware"));
    stdio_init_all();
    sleep_ms(1000);

    xTaskCreate(
        blink_task,
        "blink_task",
        256,
        NULL,
        1,
        NULL
    );

    vTaskStartScheduler();
    printf("Scheduler exited\n");
}
