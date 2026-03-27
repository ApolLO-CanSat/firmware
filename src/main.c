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
#include "flight/planner.h"
#include <string.h>
#include <stdlib.h>

/*void blink_task(__unused void *params) {
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
    char msg[128];
    snprintf(msg, sizeof(msg), "ALT:%.2f M:%d MOT:%d %d %d %d", 
      autopilot_state.current_alt,
      autopilot_state.mode,
      autopilot_state.motor_fr,
      autopilot_state.motor_fl,
      autopilot_state.motor_br,
      autopilot_state.motor_bl);

    if (d_lora_send_string(msg)) {
      LT_D("LoRa Telem TX: %s", msg);
    } else {
      LT_W("LoRa Telem TX failed");
    }


    lora_rx_packet_t rx;
    while (d_lora_receive(&rx, 0)) {
      rx.data[rx.length] = '\0';
      LT_D("LoRa RX: \"%s\" (RSSI: %d, SNR: %d)", (char *)rx.data, rx.rssi, rx.snr);
      // echo the packet back
      if (d_lora_send(rx.data, rx.length)) {
        LT_D("LoRa Echo TX: \"%s\"", (char *)rx.data);
      } else {
        LT_W("LoRa Echo TX failed");
      }
    }
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}






// if you insist...
xTaskCreate(blink_task, "blink", 256, NULL, 1, NULL);
xTaskCreate(imu_test_task, "imu_test", 512, NULL, 1, NULL);
xTaskCreate(baro_test_task, "baro_test", 512, NULL, 1, NULL);
xTaskCreate(lora_test_task, "lora_test", 1024, NULL, 1, NULL);
*/


void lora_receive_task(__unused void *params) {
  LT_I("lora_receive_task starts NOW!");
  lora_rx_packet_t rx;
  while (true) {
    if (d_lora_receive(&rx, portMAX_DELAY)) {
      if (rx.length < LORA_MAX_PKT_LENGTH) {
        rx.data[rx.length] = '\0';
      }
      LT_D("LoRa RX: \"%s\" (RSSI: %d, SNR: %d)", (char *)rx.data, rx.rssi, rx.snr);
      // check if AP cmd, format: "APCMD:MODE,ROLL,PITCH,YAW,THROTTLE,ALTITUDE,LAT,LON", if not then echo back
      if (strncmp((char *)rx.data, "APCMD:", 6) == 0) {
        // Process AP command
        char *token = strtok((char *)rx.data + 6, ",");
        if (token) {
          autopilot_command_t cmd;
          memset(&cmd, 0, sizeof(cmd));
          cmd.mode = (flight_mode_t)atoi(token);
          
          token = strtok(NULL, ",");
          cmd.roll_angle = token ? (float)atof(token) : 0.0f;
          token = strtok(NULL, ",");
          cmd.pitch_angle = token ? (float)atof(token) : 0.0f;
          token = strtok(NULL, ",");
          cmd.yaw_rate = token ? (float)atof(token) : 0.0f;
          token = strtok(NULL, ",");
          cmd.throttle = token ? (float)atof(token) : 0.0f;
          token = strtok(NULL, ",");
          cmd.altitude = token ? (float)atof(token) : 0.0f;
          token = strtok(NULL, ",");
          cmd.lat = token ? atof(token) : 0.0;
          token = strtok(NULL, ",");
          cmd.lon = token ? atof(token) : 0.0;

          autopilot_send_command(&cmd);
          LT_I("Processed AP Command: MODE=%d, ROLL=%.1f, PITCH=%.1f, YAW=%.1f, THROTTLE=%.1f, ALT=%.1f, LAT=%.6f, LON=%.6f",
            cmd.mode, cmd.roll_angle, cmd.pitch_angle, cmd.yaw_rate, cmd.throttle, cmd.altitude, cmd.lat, cmd.lon);
          // and transmit an ack
          char ack_msg[16] = "ACK APCMD DONE!";
          if (d_lora_send_string(ack_msg)) {
            LT_D("LoRa ACK TX: %s", ack_msg);
          } else {
            LT_W("LoRa ACK TX failed");
          }

        } else {
          LT_W("Invalid AP Command format");
        }
      } else {
        // echo back only if not a command
        if (d_lora_send(rx.data, rx.length)) {
          LT_D("LoRa Echo TX: \"%s\"", (char *)rx.data);
        } else {
          LT_W("LoRa Echo TX failed");
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(200)); // small delay to prevent tight loop if messages are coming in very fast
  }
}

void lora_telemetry_task(__unused void *params) {
  LT_I("lora_telemetry_task starts NOW!");
  vTaskDelay(pdMS_TO_TICKS(1000)); // wait for everything to initialize and stabilize
  LT_D("Starting telemetry transmission...");
  static uint32_t counter = 0;
  while (true) {
    LT_D("Preparing telemetry packet #%d", counter++);
    char msg[256] = "this is a test!";
    // RPY(deg), RPY_rate(deg/s), XY_speed(m/s), ALT(m), VS(m/s), MODE(F,P,A), MOT(4)
    /*snprintf(msg, sizeof(msg), "ANG:%.1f,%.1f,%.1f RT:%.1f,%.1f,%.1f SPD:%.2f,%.2f ALT:%.2f VS:%.2f FM:%d PM:%d ARM:%d MOT:%d,%d,%d,%d", 
      autopilot_state.current_roll,
      autopilot_state.current_pitch,
      autopilot_state.current_yaw,
      autopilot_state.current_roll_rate,
      autopilot_state.current_pitch_rate,
      autopilot_state.current_yaw_rate,
      autopilot_state.current_speed_x,
      autopilot_state.current_speed_y,
      autopilot_state.current_alt,
      autopilot_state.current_vertical_speed,
      autopilot_state.mode,
      autopilot_state.planner_state,
      autopilot_state.armed,
      autopilot_state.motor_fr,
      autopilot_state.motor_fl,
      autopilot_state.motor_br,
      autopilot_state.motor_bl);*/
    LT_D("Telemetry packet created: %s", msg);
    if (d_lora_send_string(msg)) {
      LT_D("LoRa Telem TX: %s", msg);
    } else {
      LT_W("LoRa Telem TX failed");
    }
    LT_D("Telemetry packet #%d sent, waiting for next update...", counter);
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

void hardware_init_task(void *params) {
  (void)params;
  
  LT_I("Hardware Init Task: STARTING...");
  
  LT_I("IMU init...");
  d_imu_init();
  
  LT_I("Baro init...");
  d_baro_init();
  
  LT_I("Autopilot & Planner init...");
  autopilot_init();
  planner_init();

  LT_I("LoRa init...");
  if (d_lora_init(435.0f, SX1278_BW_125_00_KHZ, SX1278_SF_9, SX1278_CR_4_5)) {
    vTaskDelay(1000);
    xTaskCreate(lora_receive_task, "lora_receive", 2048, NULL, 2, NULL);
    vTaskDelay(1000);
    xTaskCreate(lora_telemetry_task, "lora_telemetry", 2048, NULL, 1, NULL);
    LT_I("LoRa services started.");
  } else {
    LT_E("LoRa init FAILED in task.");
  }
  
  LT_I("Hardware Init Task: COMPLETED.");
  while(true) {
    vTaskDelay(portMAX_DELAY);
  }
  vTaskDelete(NULL);
}

void dummy_task(void *params) {
  (void)params;
  while (true) {
    LT_I("ALIVE! (FreeRTOS scheduler OK)");
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

int main() {
  stdio_init_all();
  
  sleep_ms(2000);

  LT_I("CORE: PRE-SCHEDULER BOOT OK.");

  xTaskCreate(dummy_task, "dummy", 1024, NULL, 1, NULL);
  xTaskCreate(hardware_init_task, "hardware_init_tester", 2048, NULL, 2, NULL);

  vTaskStartScheduler();
  
  while(true) {
    tight_loop_contents();
  }
}
