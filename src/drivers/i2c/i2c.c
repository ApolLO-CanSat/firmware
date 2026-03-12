#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"

#include "FreeRTOS.h"
#include "portmacro.h"
#include "semphr.h"

#include "drivers/lt_logger/lt_logger.h"

#include "i2c.h"

#if !defined(i2c_default) || !defined(PICO_DEFAULT_I2C_SDA_PIN) || !defined(PICO_DEFAULT_I2C_SCL_PIN)
#error "Default I2C pins not found in the board configuration"
#endif

// Mutex utils
static SemaphoreHandle_t i2c_mutex = NULL;

void d_i2c_mutex_take() {
  xSemaphoreTake(i2c_mutex, portMAX_DELAY);
}

void d_i2c_mutex_give() {
  xSemaphoreGive(i2c_mutex);
}

// Initialization
static bool is_i2c_initialized = false;

void d_i2c_init() {
  if (!i2c_mutex)
    i2c_mutex = xSemaphoreCreateMutex();
  d_i2c_mutex_take();

  if (is_i2c_initialized) {
    d_i2c_mutex_give();
    return;
  }

  LT_T("Initializing I2C");
  
  UBaseType_t old_affinity = vTaskCoreAffinityGet(NULL);
  vTaskCoreAffinitySet(NULL, 1 << portGET_CORE_ID());
  taskENTER_CRITICAL();

  i2c_init(i2c_default, 400 * 1000);
  gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
  gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
  bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

  taskEXIT_CRITICAL();
  vTaskCoreAffinitySet(NULL, old_affinity);
  
  is_i2c_initialized = true;
  LT_T("I2C initialized");

  d_i2c_mutex_give();
}

// Write
int d_i2c_write_unsafe(uint8_t addr, const uint8_t *src, size_t len, bool nostop) {
  if (!is_i2c_initialized)
    return PICO_ERROR_GENERIC;

  UBaseType_t old_affinity = vTaskCoreAffinityGet(NULL);
  vTaskCoreAffinitySet(NULL, 1 << portGET_CORE_ID());
  taskENTER_CRITICAL();

  int res = i2c_write_blocking(i2c_default, addr, src, len, nostop);

  taskEXIT_CRITICAL();
  vTaskCoreAffinitySet(NULL, old_affinity);
  return res;
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

  UBaseType_t old_affinity = vTaskCoreAffinityGet(NULL);
  vTaskCoreAffinitySet(NULL, 1 << portGET_CORE_ID());
  taskENTER_CRITICAL();

  int res = i2c_read_blocking(i2c_default, addr, dst, len, nostop);

  taskEXIT_CRITICAL();
  vTaskCoreAffinitySet(NULL, old_affinity);
  return res;
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

  int result = d_i2c_write_unsafe(addr, src, src_len, true);
  if (result < 0)
    return result;
  return d_i2c_read_unsafe(addr, dst, dst_len, false);
}

int d_i2c_write_read(uint8_t addr, const uint8_t *src, size_t src_len, uint8_t *dst, size_t dst_len) {
  d_i2c_mutex_take();
  int result = d_i2c_write_read_unsafe(addr, src, src_len, dst, dst_len);
  d_i2c_mutex_give();
  return result;
}
