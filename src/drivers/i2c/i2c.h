#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

void d_i2c_mutex_take();
void d_i2c_mutex_give();

void d_i2c_init();

int d_i2c_write_unsafe(uint8_t addr, const uint8_t *src, size_t len, bool nostop);
int d_i2c_write(uint8_t addr, const uint8_t *src, size_t len, bool nostop);

int d_i2c_read_unsafe(uint8_t addr, uint8_t *dst, size_t len, bool nostop);
int d_i2c_read(uint8_t addr, uint8_t *dst, size_t len, bool nostop);

int d_i2c_write_read_unsafe(uint8_t addr, const uint8_t *src, size_t src_len, uint8_t *dst, size_t dst_len);
int d_i2c_write_read(uint8_t addr, const uint8_t *src, size_t src_len, uint8_t *dst, size_t dst_len);
