#include <string.h>

#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "drivers/lt_logger/lt_logger.h"

#include "lora.h"

#if !defined(spi_default) || !defined(PICO_DEFAULT_SPI_SCK_PIN) || !defined(PICO_DEFAULT_SPI_TX_PIN) || \
  !defined(PICO_DEFAULT_SPI_RX_PIN) || !defined(PICO_DEFAULT_SPI_CSN_PIN)
#error "Default SPI pins not found in board configuration"
#endif

#ifndef LORA_DIO0_PIN
#warning "LORA_DIO0_PIN not defined in board configuration"
#define LORA_DIO0_PIN 26
#endif

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

#define SPI_SPEED          (2 * 1000 * 1000)
#define RSSI_OFFSET        (-164)
#define TX_QUEUE_DEPTH     8
#define RX_QUEUE_DEPTH     8
#define TX_TIMEOUT_MS      10000
#define DAEMON_STACK_WORDS 1024
#define DAEMON_PRIORITY    3

#define NOTIFY_DIO0     (1 << 0)
#define NOTIFY_TX_READY (1 << 1)
#define NOTIFY_RESUME   (1 << 2)

// ---------------------------------------------------------------------------
// Internal types
// ---------------------------------------------------------------------------

typedef struct {
  uint8_t data[LORA_MAX_PKT_LENGTH];
  uint8_t length;
} lora_tx_packet_t;

typedef enum {
  RADIO_RX,
  RADIO_TX,
} radio_state_t;

// ---------------------------------------------------------------------------
// Module state
// ---------------------------------------------------------------------------

static TaskHandle_t daemon_handle;
static SemaphoreHandle_t spi_mutex;
static QueueHandle_t tx_queue;
static QueueHandle_t rx_queue;
static radio_state_t radio_state;
static volatile bool radio_paused;

// ---------------------------------------------------------------------------
// SPI register access
// ---------------------------------------------------------------------------

static uint8_t reg_read(uint8_t reg) {
  uint8_t tx[2] = {reg & 0x7F, 0x00};
  uint8_t rx[2] = {0};
  gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 0);
  spi_write_read_blocking(spi_default, tx, rx, 2);
  gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 1);
  return rx[1];
}

static void reg_write(uint8_t reg, uint8_t value) {
  uint8_t tx[2] = {reg | 0x80, value};
  gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 0);
  spi_write_blocking(spi_default, tx, 2);
  gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 1);
}

static void reg_write_masked(uint8_t reg, uint8_t value, uint8_t msb, uint8_t lsb) {
  uint8_t current = reg_read(reg);
  uint8_t mask    = (0xFF << (msb + 1)) | (0xFF >> (8 - lsb));
  reg_write(reg, (current & mask) | value);
}

static void fifo_read_burst(uint8_t *data, uint8_t length) {
  uint8_t addr = SX1278_REG_FIFO & 0x7F;
  gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 0);
  spi_write_blocking(spi_default, &addr, 1);
  spi_read_blocking(spi_default, 0x00, data, length);
  gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 1);
}

static void fifo_write_burst(const uint8_t *data, uint8_t length) {
  uint8_t addr = SX1278_REG_FIFO | 0x80;
  gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 0);
  spi_write_blocking(spi_default, &addr, 1);
  spi_write_blocking(spi_default, data, length);
  gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 1);
}

// ---------------------------------------------------------------------------
// Radio helpers (only called from daemon task context)
// ---------------------------------------------------------------------------

static void set_opmode(uint8_t mode) {
  reg_write_masked(SX1278_REG_OP_MODE, mode, 2, 0);
}

static void enter_rx(void) {
  radio_state = RADIO_RX;
  reg_write(SX1278_REG_IRQ_FLAGS, 0xFF);
  set_opmode(SX1278_STANDBY);
  reg_write_masked(SX1278_REG_DIO_MAPPING_1, SX1278_DIO0_RX_DONE, 7, 6);
  reg_write(SX1278_REG_FIFO_RX_BASE_ADDR, SX1278_FIFO_RX_BASE_ADDR_MAX);
  reg_write(SX1278_REG_FIFO_ADDR_PTR, SX1278_FIFO_RX_BASE_ADDR_MAX);
  set_opmode(SX1278_RXCONTINUOUS);
}

static void begin_tx(const lora_tx_packet_t *pkt) {
  radio_state = RADIO_TX;
  reg_write(SX1278_REG_IRQ_FLAGS, 0xFF);
  set_opmode(SX1278_STANDBY);
  reg_write_masked(SX1278_REG_DIO_MAPPING_1, SX1278_DIO0_TX_DONE, 7, 6);
  reg_write(SX1278_REG_PAYLOAD_LENGTH, pkt->length);
  reg_write(SX1278_REG_FIFO_TX_BASE_ADDR, SX1278_FIFO_TX_BASE_ADDR_MAX);
  reg_write(SX1278_REG_FIFO_ADDR_PTR, SX1278_FIFO_TX_BASE_ADDR_MAX);
  fifo_write_burst(pkt->data, pkt->length);
  set_opmode(SX1278_TX);
}

// ---------------------------------------------------------------------------
// DIO0 ISR — minimal: just wake the daemon
// ---------------------------------------------------------------------------

static void dio0_isr(uint gpio, uint32_t events) {
  (void)gpio;
  (void)events;
  BaseType_t woken = pdFALSE;
  xTaskNotifyFromISR(daemon_handle, NOTIFY_DIO0, eSetBits, &woken);
  portYIELD_FROM_ISR(woken);
}

// ---------------------------------------------------------------------------
// Daemon task — owns all SPI access after init
// ---------------------------------------------------------------------------

static void handle_dio0(void) {
  uint8_t irq = reg_read(SX1278_REG_IRQ_FLAGS);
  reg_write(SX1278_REG_IRQ_FLAGS, 0xFF);

  if (radio_state == RADIO_TX) {
    if (!(irq & SX1278_CLEAR_IRQ_FLAG_TX_DONE))
      return;

    lora_tx_packet_t next;
    if (xQueueReceive(tx_queue, &next, 0) == pdTRUE) {
      begin_tx(&next);
    } else {
      enter_rx();
    }
    return;
  }

  // RX path
  if (!(irq & SX1278_CLEAR_IRQ_FLAG_RX_DONE))
    return;

  if (irq & SX1278_CLEAR_IRQ_FLAG_PAYLOAD_CRC_ERROR) {
    LT_W("LoRa CRC error");
    return;
  }

  lora_rx_packet_t pkt;
  pkt.length = reg_read(SX1278_REG_RX_NB_BYTES);
  reg_write(SX1278_REG_FIFO_ADDR_PTR, reg_read(SX1278_REG_FIFO_RX_CURRENT_ADDR));
  fifo_read_burst(pkt.data, pkt.length);
  pkt.rssi = RSSI_OFFSET + (int16_t)reg_read(SX1278_REG_PKT_RSSI_VALUE);
  pkt.snr  = (int8_t)reg_read(SX1278_REG_PKT_SNR_VALUE);

  if (xQueueSend(rx_queue, &pkt, 0) != pdTRUE) {
    LT_W("LoRa RX queue full, packet dropped");
  }
}

static void handle_tx_ready(void) {
  if (radio_state != RADIO_RX)
    return;

  lora_tx_packet_t pkt;
  if (xQueueReceive(tx_queue, &pkt, 0) == pdTRUE) {
    begin_tx(&pkt);
  }
}

static void lora_daemon(void *params) {
  (void)params;

  xSemaphoreTake(spi_mutex, portMAX_DELAY);
  enter_rx();
  xSemaphoreGive(spi_mutex);

  LT_I("LoRa daemon running");

  for (;;) {
    TickType_t wait = (radio_state == RADIO_TX) ? pdMS_TO_TICKS(TX_TIMEOUT_MS) : portMAX_DELAY;

    uint32_t bits = 0;
    BaseType_t got = xTaskNotifyWait(0, UINT32_MAX, &bits, wait);

    xSemaphoreTake(spi_mutex, portMAX_DELAY);

    if (radio_paused) {
      if (bits & NOTIFY_RESUME) {
        radio_paused = false;
        enter_rx();
      }
      xSemaphoreGive(spi_mutex);
      continue;
    }

    if (got == pdFALSE) {
      if (radio_state == RADIO_TX) {
        LT_W("LoRa TX timeout, resetting to RX");
        enter_rx();
      }
      xSemaphoreGive(spi_mutex);
      continue;
    }

    if (bits & NOTIFY_DIO0)
      handle_dio0();

    if (bits & NOTIFY_TX_READY)
      handle_tx_ready();

    xSemaphoreGive(spi_mutex);
  }
}

// ---------------------------------------------------------------------------
// Hardware init (called once from d_lora_init, before daemon starts)
// ---------------------------------------------------------------------------

static bool hw_init(float frequency_mhz, uint8_t bandwidth, uint8_t spreading_factor, uint8_t coding_rate) {
  spi_init(spi_default, SPI_SPEED);
  gpio_set_function(PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI);
  gpio_set_function(PICO_DEFAULT_SPI_TX_PIN, GPIO_FUNC_SPI);
  gpio_set_function(PICO_DEFAULT_SPI_RX_PIN, GPIO_FUNC_SPI);

  gpio_init(PICO_DEFAULT_SPI_CSN_PIN);
  gpio_set_dir(PICO_DEFAULT_SPI_CSN_PIN, GPIO_OUT);
  gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 1);

  vTaskDelay(pdMS_TO_TICKS(10));

  uint8_t version = reg_read(SX1278_REG_VERSION);
  if (version != 0x12) {
    LT_E("SX1278 not found (version=0x%02X)", version);
    spi_deinit(spi_default);
    return false;
  }
  LT_I("SX1278 detected (version=0x%02X)", version);

  // Sleep mode required to switch to LoRa mode
  set_opmode(SX1278_SLEEP);
  vTaskDelay(pdMS_TO_TICKS(10));
  reg_write_masked(SX1278_REG_OP_MODE, SX1278_LORA, 7, 7);

  // Carrier frequency: frf = freq / (32 MHz / 2^19)
  uint32_t frf = (uint32_t)(frequency_mhz / (32.0f / (float)(1 << 19)));
  reg_write(SX1278_REG_FRF_MSB, (frf >> 16) & 0xFF);
  reg_write(SX1278_REG_FRF_MID, (frf >> 8) & 0xFF);
  reg_write(SX1278_REG_FRF_LSB, frf & 0xFF);

  // PA: max power on PA_BOOST
  reg_write(SX1278_REG_PA_CONFIG, SX1278_PA_SELECT_BOOST | SX1278_MAX_POWER | SX1278_OUTPUT_POWER);
  reg_write_masked(SX1278_REG_OCP, SX1278_OCP_ON | SX1278_OCP_TRIM, 5, 0);
  reg_write(SX1278_REG_LNA, SX1278_LNA_GAIN_1 | SX1278_LNA_BOOST_HF_ON);
  reg_write_masked(SX1278_REG_PA_DAC, SX1278_PA_BOOST_ON, 2, 0);

  reg_write(SX1278_REG_HOP_PERIOD, SX1278_HOP_PERIOD_OFF);

  // Modem config
  reg_write_masked(
    SX1278_REG_MODEM_CONFIG_2, spreading_factor | SX1278_TX_MODE_SINGLE | SX1278_RX_CRC_MODE_ON, 7, 2
  );
  reg_write(SX1278_REG_MODEM_CONFIG_1, bandwidth | coding_rate | SX1278_HEADER_EXPL_MODE);
  reg_write_masked(SX1278_REG_MODEM_CONFIG_3, SX1278_LOW_DATA_RATE_OPT_ON | SX1278_AGC_AUTO_ON, 3, 2);

  // Detection optimization for SF7–12
  reg_write_masked(SX1278_REG_DETECT_OPTIMIZE, SX1278_DETECT_OPTIMIZE_SF_7_12, 2, 0);
  reg_write(SX1278_REG_DETECTION_THRESHOLD, SX1278_DETECTION_THRESHOLD_SF_7_12);

  // FIFO base addresses
  reg_write(SX1278_REG_FIFO_TX_BASE_ADDR, SX1278_FIFO_TX_BASE_ADDR_MAX);
  reg_write(SX1278_REG_FIFO_RX_BASE_ADDR, SX1278_FIFO_RX_BASE_ADDR_MAX);

  set_opmode(SX1278_STANDBY);
  reg_write(SX1278_REG_IRQ_FLAGS, 0xFF);

  // DIO0 interrupt
  gpio_init(LORA_DIO0_PIN);
  gpio_set_dir(LORA_DIO0_PIN, GPIO_IN);
  gpio_set_irq_enabled_with_callback(LORA_DIO0_PIN, GPIO_IRQ_EDGE_RISE, true, dio0_isr);

  return true;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

bool d_lora_init(float frequency_mhz, uint8_t bandwidth, uint8_t spreading_factor, uint8_t coding_rate) {
  if (daemon_handle != NULL)
    return true;

  if (!hw_init(frequency_mhz, bandwidth, spreading_factor, coding_rate))
    return false;

  spi_mutex = xSemaphoreCreateMutex();
  tx_queue  = xQueueCreate(TX_QUEUE_DEPTH, sizeof(lora_tx_packet_t));
  rx_queue  = xQueueCreate(RX_QUEUE_DEPTH, sizeof(lora_rx_packet_t));
  configASSERT(spi_mutex);
  configASSERT(tx_queue);
  configASSERT(rx_queue);

  BaseType_t ok = xTaskCreate(lora_daemon, "lora", DAEMON_STACK_WORDS, NULL, DAEMON_PRIORITY, &daemon_handle);
  configASSERT(ok == pdPASS);

  LT_I("LoRa initialized at %.2f MHz", (double)frequency_mhz);
  return true;
}

bool d_lora_send(const uint8_t *data, uint8_t length) {
  if (daemon_handle == NULL || length == 0)
    return false;

  lora_tx_packet_t pkt;
  pkt.length = length;
  memcpy(pkt.data, data, length);

  if (xQueueSend(tx_queue, &pkt, pdMS_TO_TICKS(100)) != pdTRUE)
    return false;

  xTaskNotify(daemon_handle, NOTIFY_TX_READY, eSetBits);
  return true;
}

bool d_lora_send_string(const char *str) {
  size_t len = strlen(str);
  if (len >= LORA_MAX_PKT_LENGTH)
    return false;
  return d_lora_send((const uint8_t *)str, (uint8_t)(len + 1));
}

bool d_lora_receive(lora_rx_packet_t *packet, TickType_t timeout) {
  if (rx_queue == NULL)
    return false;
  return xQueueReceive(rx_queue, packet, timeout) == pdTRUE;
}

uint8_t d_lora_available(void) {
  if (rx_queue == NULL)
    return 0;
  return (uint8_t)uxQueueMessagesWaiting(rx_queue);
}

void d_lora_sleep(void) {
  xSemaphoreTake(spi_mutex, portMAX_DELAY);
  radio_paused = true;
  set_opmode(SX1278_SLEEP);
  xSemaphoreGive(spi_mutex);
}

void d_lora_standby(void) {
  xSemaphoreTake(spi_mutex, portMAX_DELAY);
  radio_paused = true;
  set_opmode(SX1278_STANDBY);
  xSemaphoreGive(spi_mutex);
}

void d_lora_resume(void) {
  xTaskNotify(daemon_handle, NOTIFY_RESUME, eSetBits);
}

int d_lora_get_rssi_last(void) {
  xSemaphoreTake(spi_mutex, portMAX_DELAY);
  int rssi = RSSI_OFFSET + (int)reg_read(SX1278_REG_PKT_RSSI_VALUE);
  xSemaphoreGive(spi_mutex);
  return rssi;
}

int d_lora_get_rssi_now(void) {
  xSemaphoreTake(spi_mutex, portMAX_DELAY);
  int rssi = RSSI_OFFSET + (int)reg_read(SX1278_REG_RSSI_VALUE);
  xSemaphoreGive(spi_mutex);
  return rssi;
}

int8_t d_lora_get_snr_last(void) {
  xSemaphoreTake(spi_mutex, portMAX_DELAY);
  int8_t snr = (int8_t)reg_read(SX1278_REG_PKT_SNR_VALUE);
  xSemaphoreGive(spi_mutex);
  return snr;
}
