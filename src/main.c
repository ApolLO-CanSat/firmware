#include <stdio.h>

#include "pico/binary_info.h"
#include "pico/stdlib.h"

#include "FreeRTOS.h"
#include "task.h"

#include "drivers/imu/imu.h"
#include "drivers/led/led.h"

void blink_task(__unused void *params) {
  bool on = false;
  printf("blink_task starts\n");
  d_led_init();
  while (true) {
#if configNUMBER_OF_CORES > 1
    static int last_core_id = -1;
    if (portGET_CORE_ID() != last_core_id) {
      last_core_id = portGET_CORE_ID();
      printf("blink task is on core %d\n", last_core_id);
    }
#endif
    d_led_set(on);
    on = !on;

    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void i2c_test_task(__unused void *params) {
  printf("i2c_test_task starts\n");
  d_imu_init();
  int16_t accel[3], gyro[3], temp;
  while (true) {
    d_imu_read_raw(accel, gyro, &temp);
    printf(
      "Accel: %d %d %d, Gyro: %d %d %d, Temp: %f\n",
      accel[0],
      accel[1],
      accel[2],
      gyro[0],
      gyro[1],
      gyro[2],
      (temp / 340.0) + 36.53
    );
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

int main() {
  bi_decl(bi_program_description("ApolLO CanSat firmware"));
  stdio_init_all();

  xTaskCreate(blink_task, "blink_task", 256, NULL, 1, NULL);
  xTaskCreate(i2c_test_task, "i2c_test_task", 512, NULL, 1, NULL);

  vTaskStartScheduler();
  printf("Scheduler exited\n");
}
