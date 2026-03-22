#include "drivers/lt_logger/lt_logger.h"
#include "pico/stdlib.h"

#include "FreeRTOS.h"
#include "task.h"

#include "drivers/i2c/i2c.h"

#include "imu.h"

#define ADDR _u(0x68)

int16_t accel_bias[3] = {0}, gyro_bias[3] = {0};

void d_imu_init() {
  LT_T("Initializing MPU6050");
  d_i2c_init();

  // Reset
  uint8_t buf[] = {0x6b, 0x80};
  d_i2c_write(ADDR, buf, sizeof(buf), false);

  vTaskDelay(100 / portTICK_PERIOD_MS);

  // Clear sleep mode
  buf[1] = 0x00;
  d_i2c_write(ADDR, buf, sizeof(buf), false);
  
  LT_T("Initialized MPU6050. Remember to calibrate.");
  
  vTaskDelay(100 / portTICK_PERIOD_MS);
}

void d_imu_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
  uint8_t buffer[6];

  // Start reading acceleration registers from register 0x3B for 6 bytes
  uint8_t val = 0x3B;
  d_i2c_write_read(ADDR, &val, sizeof(val), buffer, sizeof(buffer));
  for (int i = 0; i < 3; i++)
    accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);

  // Now gyro data from reg 0x43 for 6 bytes
  val = 0x43;
  d_i2c_write_read(ADDR, &val, sizeof(val), buffer, sizeof(buffer));
  for (int i = 0; i < 3; i++)
    gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);

  // Now temperature from reg 0x41 for 2 bytes
  val = 0x41;
  d_i2c_write_read(ADDR, &val, sizeof(val), buffer, sizeof(buffer));
  *temp = buffer[0] << 8 | buffer[1];
}

void d_imu_calibrate() {
  LT_I("Calibrating MPU6050... Keep it still!");
  const int samples = 1000;
  int32_t accel_sum[3] = {0}, gyro_sum[3] = {0};
  int16_t temp;
  
  for (int i = 0; i < samples; i++) {
    d_imu_read_raw((int16_t*)accel_sum, (int16_t*)gyro_sum, &temp);
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }

  for (int i = 0; i < 3; i++) {
    accel_bias[i] = accel_sum[i] / samples;
    gyro_bias[i] = gyro_sum[i] / samples;
  }

  LT_I("MPU6050 Calibration done. Accel bias: [%d, %d, %d], Gyro bias: [%d, %d, %d]", 
      accel_bias[0], accel_bias[1], accel_bias[2],
      gyro_bias[0], gyro_bias[1], gyro_bias[2]);
}

void d_imu_read_cal(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
  d_imu_read_raw(accel, gyro, temp);
  for (int i = 0; i < 3; i++) {
    accel[i] -= accel_bias[i];
    gyro[i] -= gyro_bias[i];
  }
}