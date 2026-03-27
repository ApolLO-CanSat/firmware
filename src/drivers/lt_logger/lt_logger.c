// Copyright (c) Kuba Szczodrzyński 2025-7-10.

#include <stdio.h>

#include "pico/stdlib.h"

#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>

#include "lt_logger.h"

#define COLOR_FMT            "\e[0;30m"
#define COLOR_BLACK          0x00
#define COLOR_RED            0x01
#define COLOR_GREEN          0x02
#define COLOR_YELLOW         0x03
#define COLOR_BLUE           0x04
#define COLOR_MAGENTA        0x05
#define COLOR_CYAN           0x06
#define COLOR_WHITE          0x07
#define COLOR_BRIGHT_BLACK   0x10
#define COLOR_BRIGHT_RED     0x11
#define COLOR_BRIGHT_GREEN   0x12
#define COLOR_BRIGHT_YELLOW  0x13
#define COLOR_BRIGHT_BLUE    0x14
#define COLOR_BRIGHT_MAGENTA 0x15
#define COLOR_BRIGHT_CYAN    0x16
#define COLOR_BRIGHT_WHITE   0x17

static const char levels[] = {'V', 'D', 'I', 'W', 'E', 'F'};

#if LT_LOGGER_COLOR
static const uint8_t colors[] = {
  COLOR_BRIGHT_CYAN,
  COLOR_BRIGHT_BLUE,
  COLOR_BRIGHT_GREEN,
  COLOR_BRIGHT_YELLOW,
  COLOR_BRIGHT_RED,
  COLOR_BRIGHT_MAGENTA,
};
#endif

static SemaphoreHandle_t logger_sem = NULL;

#if LT_LOGGER_CALLER
void lt_log(const uint8_t level, const char *caller, const unsigned short line, const char *format, ...) {
#else
void lt_log(const uint8_t level, const char *format, ...) {
#endif

  if (!logger_sem)
    logger_sem = xSemaphoreCreateMutex();

#if LT_LOGGER_TIMESTAMP
  // NOLINTNEXTLINE(bugprone-integer-division)
  float seconds = xTaskGetTickCount() * portTICK_PERIOD_MS / 1000.0f;
#endif

#if LT_LOGGER_TASK
  char *task_name   = "";
  char task_colon   = '-';
  TaskHandle_t task = (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) ? NULL : xTaskGetCurrentTaskHandle();
  if (task) {
    task_name  = pcTaskGetName(task);
    task_colon = ':';
  }
#endif

#if LT_LOGGER_COLOR
  char c_bright = '0' + (colors[level] >> 4);
  char c_value  = '0' + (colors[level] & 0x7);
#endif

  bool has_semaphore = false;
  if (task && xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
    if (xSemaphoreTake(logger_sem, 0) == pdTRUE) {
      has_semaphore = true;
    }
  }

  if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
    printf(
#if LT_LOGGER_COLOR
      "\e[%c;3%cm"
#endif
      "%c [      0.000] "
#if LT_LOGGER_COLOR
      "\e[0m"
#endif
#if LT_LOGGER_TASK
      "          - "
#endif
#if LT_LOGGER_CALLER
      "%s():%hu: "
#endif
      ,
#if LT_LOGGER_COLOR
      c_bright,
      c_value,
#endif
      levels[level]
#if LT_LOGGER_CALLER
      ,
      caller,
      line
#endif
    );
  } else {
    printf(
    // format:
#if LT_LOGGER_COLOR
      "\e[%c;3%cm"
#endif
      "%c "
#if LT_LOGGER_TIMESTAMP
      "[%11.3f] "
#endif
#if LT_LOGGER_COLOR
      "\e[0m"
#endif
#if LT_LOGGER_CALLER
      "%s():%hu: "
#endif
#if LT_LOGGER_TASK
      "%10s%c "
#endif
      ,
    // arguments:
#if LT_LOGGER_COLOR
      c_bright, // whether text is bright
      c_value,  // text color
#endif
      levels[level]
#if LT_LOGGER_TIMESTAMP
      ,
      seconds // float
#endif
#if LT_LOGGER_CALLER
      ,
      caller,
      line
#endif
#if LT_LOGGER_TASK
      ,
      task_name,
      task_colon // printing outside of tasks
#endif
    );
  }

  va_list va_args;
  va_start(va_args, format);
  vprintf(format, va_args);
  va_end(va_args);
  putchar('\n');
  if (has_semaphore) {
    xSemaphoreGive(logger_sem);
  }
}
