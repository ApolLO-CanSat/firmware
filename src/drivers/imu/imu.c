#include "hardware/i2c.h"
#include "pico/stdlib.h"

#include "FreeRTOS.h"
#include "task.h"

#include "drivers/i2c/i2c.h"

static int addr = 0x68;

void d_imu_init() {
  d_i2c_init();

  // Reset
  uint8_t buf[] = {0x6b, 0x80};
  d_i2c_write(addr, buf, sizeof(buf), false);

  vTaskDelay(100 / portTICK_PERIOD_MS);

  // Clear sleep mode
  buf[1] = 0x00;
  d_i2c_write(addr, buf, sizeof(buf), false);
}

void d_imu_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
  uint8_t buffer[6];

  d_i2c_mutex_take();
  
  // Start reading acceleration registers from register 0x3B for 6 bytes
  uint8_t val = 0x3B;
  d_i2c_write_read_unsafe(addr, &val, sizeof(val), buffer, sizeof(buffer));
  for (int i = 0; i < 3; i++)
    accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);

  // Now gyro data from reg 0x43 for 6 bytes
  val = 0x43;
  d_i2c_write_read_unsafe(addr, &val, sizeof(val), buffer, sizeof(buffer));
  for (int i = 0; i < 3; i++)
    gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);

  // Now temperature from reg 0x41 for 2 bytes
  val = 0x41;
  d_i2c_write_read_unsafe(addr, &val, sizeof(val), buffer, sizeof(buffer));
  *temp = buffer[0] << 8 | buffer[1];

  d_i2c_mutex_give();
}
