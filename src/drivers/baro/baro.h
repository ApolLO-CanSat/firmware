#pragma once

#include <stdint.h>

void d_baro_init();
void d_baro_read(int32_t *temp, int32_t *pressure);