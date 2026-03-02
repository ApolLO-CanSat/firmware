#pragma once

#include <stdint.h>

void d_imu_init();
void d_imu_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp);