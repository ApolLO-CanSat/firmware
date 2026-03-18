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
#include "drivers/lora/lora.h"
#include "drivers/lt_logger/lt_logger.h"
#include "drivers/i2c/i2c.h"
#include "flight/autopilot.h"

/*
void telemetry_task(__unused void *params) {
  LT_I("telemetry_task starts (waiting for friend)");
  while (true) {
    LT_D("ALT: %.2f MODE: %d MOTORS: %d %d %d %d GYRO: %.2f %.2f %.2f ACCEL: %.2f %.2f %.2f", 
      autopilot_state.current_alt,
      autopilot_state.mode,
      autopilot_state.motor_fr,
      autopilot_state.motor_fl,
      autopilot_state.motor_br,
      autopilot_state.motor_bl,
      gyro_filtered[0], gyro_filtered[1], gyro_filtered[2],
      accel_filtered[0], accel_filtered[1], accel_filtered[2]);
      // todo: more telem (my part) + send by lora (not my part)
    vTaskDelay(pdMS_TO_TICKS(200)); 
  }
}
*/

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

void lora_test_task(__unused void *params) {
  LT_I("lora_test_task starts");
  vTaskDelay(2000 / portTICK_PERIOD_MS); // temp fix, idk why it works
  bool ok = d_lora_init(435.0f, SX1278_BW_125_00_KHZ, SX1278_SF_9, SX1278_CR_4_5);
  if (!ok) {
    LT_E("LoRa init failed");
    vTaskDelete(NULL);
    return;
  }

  uint32_t counter = 0;
  while (true) {
    char msg[64];
    snprintf(msg, sizeof(msg), "ApolLO #%lu", (unsigned long)counter++);
    if (d_lora_send_string(msg)) {
      LT_D("LoRa TX: \"%s\"", msg);
    } else {
      LT_W("LoRa TX failed");
    }

    lora_rx_packet_t rx;
    while (d_lora_receive(&rx, 0)) {
      rx.data[rx.length] = '\0';
      LT_D("LoRa RX: \"%s\" (RSSI: %d, SNR: %d)", (char *)rx.data, rx.rssi, rx.snr);
    }

    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

int main() {
  bi_decl(bi_program_description("ApolLO CanSat firmware"));
  stdio_init_all();
  LT_I("Firmware starts");

  /*
  // Drivers initialization (for "new firmware" and not this test_task dogshit :D )
  d_imu_init();
  d_baro_init();
  autopilot_init();

  xTaskCreate(telemetry_task, "telem", 512, NULL, 1, NULL);
  */

  xTaskCreate(blink_task, "blink", 256, NULL, 1, NULL);
  xTaskCreate(imu_test_task, "imu_test", 512, NULL, 1, NULL);
  xTaskCreate(baro_test_task, "baro_test", 512, NULL, 1, NULL);
  xTaskCreate(lora_test_task, "lora_test", 1024, NULL, 1, NULL);

  vTaskStartScheduler();
  LT_E("Scheduler exited unexpectedly");
}
