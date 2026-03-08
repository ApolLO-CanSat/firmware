/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// -----------------------------------------------------
// NOTE: THIS HEADER IS ALSO INCLUDED BY ASSEMBLER SO
//       SHOULD ONLY CONSIST OF PREPROCESSOR DIRECTIVES
// -----------------------------------------------------

// This header may be included by other board headers as "boards/orpheus_pico.h"

#ifndef _BOARDS_ORPHEUS_PICO_H
#define _BOARDS_ORPHEUS_PICO_H

pico_board_cmake_set(PICO_PLATFORM, rp2040)

// For board detection
#define ORPHEUS_PICO

// --- UART ---
// (no changes)

// --- LED ---
#ifndef PICO_DEFAULT_LED_PIN
#define PICO_DEFAULT_LED_PIN 23
#endif
#ifndef PICO_DEFAULT_WS2812_PIN
#define PICO_DEFAULT_WS2812_PIN 24
#endif

// --- I2C ---
// (no changes)

// --- SPI ---
// (no changes)

// --- LoRa ---
#ifndef LORA_DIO0_PIN
#define LORA_DIO0_PIN 26
#endif

// --- FLASH ---
// (no changes)

pico_board_cmake_set_default(PICO_FLASH_SIZE_BYTES, (8 * 1024 * 1024))
#ifndef PICO_FLASH_SIZE_BYTES
#define PICO_FLASH_SIZE_BYTES (8 * 1024 * 1024)
#endif

#ifndef PICO_RP2040_B0_SUPPORTED
#define PICO_RP2040_B0_SUPPORTED 0
#endif
#ifndef PICO_RP2040_B1_SUPPORTED
#define PICO_RP2040_B1_SUPPORTED 0
#endif
#ifndef PICO_RP2040_B2_SUPPORTED
#define PICO_RP2040_B2_SUPPORTED 1
#endif

#include "boards/pico.h"

#endif
