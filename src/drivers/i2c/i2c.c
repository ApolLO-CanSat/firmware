#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"

#include "FreeRTOS.h"
#include "semphr.h"

#include "i2c.h"

#if !defined(i2c_default) || !defined(PICO_DEFAULT_I2C_SDA_PIN) || !defined(PICO_DEFAULT_I2C_SCL_PIN)
#error "Default I2C pins not found in the board configuration"
#endif

SemaphoreHandle_t i2c_mutex;
static bool is_i2c_initialized = false;

void d_i2c_init() {
  if (is_i2c_initialized)
    return;
  i2c_mutex = xSemaphoreCreateMutex();
  xSemaphoreTake(i2c_mutex, portMAX_DELAY);

  i2c_init(i2c_default, 400 * 1000);
  gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
  gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
  bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

  xSemaphoreGive(i2c_mutex);
  is_i2c_initialized = true;
}

// Mutex utils
void d_i2c_mutex_take() {
  xSemaphoreTake(i2c_mutex, portMAX_DELAY);
}

void d_i2c_mutex_give() {
  xSemaphoreGive(i2c_mutex);
}

// Write
int d_i2c_write_unsafe(uint8_t addr, const uint8_t *src, size_t len, bool nostop) {
  if (!is_i2c_initialized)
    return PICO_ERROR_GENERIC;
  return i2c_write_blocking(i2c_default, addr, src, len, nostop);
}

int d_i2c_write(uint8_t addr, const uint8_t *src, size_t len, bool nostop) {
  d_i2c_mutex_take();
  int result = d_i2c_write_unsafe(addr, src, len, nostop);
  d_i2c_mutex_give();
  return result;
}

// Read
int d_i2c_read_unsafe(uint8_t addr, uint8_t *dst, size_t len, bool nostop) {
  if (!is_i2c_initialized)
    return PICO_ERROR_GENERIC;
  return i2c_read_blocking(i2c_default, addr, dst, len, nostop);
}

int d_i2c_read(uint8_t addr, uint8_t *dst, size_t len, bool nostop) {
  d_i2c_mutex_take();
  int result = d_i2c_read_unsafe(addr, dst, len, nostop);
  d_i2c_mutex_give();
  return result;
}

// R/W
int d_i2c_write_read_unsafe(uint8_t addr, const uint8_t *src, size_t src_len, uint8_t *dst, size_t dst_len) {
  if (!is_i2c_initialized)
    return PICO_ERROR_GENERIC;

  int result = i2c_write_blocking(i2c_default, addr, src, src_len, true);
  if (result < 0)
    return result;
  return i2c_read_blocking(i2c_default, addr, dst, dst_len, false);
}

int d_i2c_write_read(uint8_t addr, const uint8_t *src, size_t src_len, uint8_t *dst, size_t dst_len) {
  d_i2c_mutex_take();
  int result = d_i2c_write_read_unsafe(addr, src, src_len, dst, dst_len);
  d_i2c_mutex_give();
  return result;
}
