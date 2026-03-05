#include <stdio.h>

#include "pico/binary_info.h"
#include "pico/stdlib.h"

#include "FreeRTOS.h"
#include "portmacro.h"
#include "projdefs.h"
#include "task.h"

#include "drivers/baro/baro.h"
#include "drivers/imu/imu.h"
#include "drivers/led/led.h"
#include "drivers/lt_logger/lt_logger.h"

void blink_task(__unused void *params) {
  bool on = false;
  LT_I("blink_task starts");
  d_led_init();
  while (true) {
#if configNUMBER_OF_CORES > 1
    static int last_core_id = -1;
    if (portGET_CORE_ID() != last_core_id) {
      last_core_id = portGET_CORE_ID();
      LT_D("blink task is on core %d", last_core_id);
    }
#endif
    d_led_set(on);
    on = !on;

    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void imu_test_task(__unused void *params) {
  LT_I("imu_test_task starts");
  d_imu_init();
  int16_t accel[3], gyro[3], temp;
  while (true) {
    d_imu_read_raw(accel, gyro, &temp);
    LT_D(
      "Accel: %d %d %d, Gyro: %d %d %d, Temp: %f",
      accel[0],
      accel[1],
      accel[2],
      gyro[0],
      gyro[1],
      gyro[2],
      (temp / 340.0) + 36.53
    );
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void baro_test_task(__unused void *params) {
  LT_I("baro_test_task starts");
  d_baro_init();
  int32_t pressure, temperature;
  while (true) {
    d_baro_read(&temperature, &pressure);
    LT_D("Pressure: %.3f kPa, Temperature: %.2f C", pressure / 1000.f, temperature / 100.f);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

int main() {
  bi_decl(bi_program_description("ApolLO CanSat firmware"));
  stdio_init_all();
  LT_I("Firmware starts");

  xTaskCreate(blink_task, "blink", 256, NULL, 1, NULL);
  xTaskCreate(imu_test_task, "imu_test", 512, NULL, 1, NULL);
  xTaskCreate(baro_test_task, "baro_test", 512, NULL, 1, NULL);

  vTaskStartScheduler();
  LT_E("Scheduler exited unexpectedly");
}
