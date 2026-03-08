#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"

#include "defines.h"

#define LORA_MAX_PKT_LENGTH 255

typedef struct {
  uint8_t data[LORA_MAX_PKT_LENGTH];
  uint8_t length;
  int16_t rssi;
  int8_t snr;
} lora_rx_packet_t;

// Initialize the SX1278 and start the LoRa daemon task.
bool d_lora_init(float frequency_mhz, uint8_t bandwidth, uint8_t spreading_factor, uint8_t coding_rate);

// Queue a packet for transmission. Returns false if the TX queue is full.
bool d_lora_send(const uint8_t *data, uint8_t length);

// Queue a string for transmission (including null terminator).
bool d_lora_send_string(const char *str);

// Receive a packet, blocking up to `timeout` ticks. Returns false on timeout.
bool d_lora_receive(lora_rx_packet_t *packet, TickType_t timeout);

// Number of packets waiting in the RX queue.
uint8_t d_lora_available(void);

// Power management — callable from any task.
// Sleep/standby pause the daemon; resume restarts RX.
void d_lora_sleep(void);
void d_lora_standby(void);
void d_lora_resume(void);

// Signal measurements — callable from any task.
int d_lora_get_rssi_last(void);
int d_lora_get_rssi_now(void);
int8_t d_lora_get_snr_last(void);
