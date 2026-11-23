/* SPDX-License-Identifier: GPL-3.0-or-later */
/* Copyright (C) 2026 Franco Venturi. Part of x6100_bb_fw (GPLv3). */
/*
 * bsp_gpio (F427 shim) -- the Bsp_Gpio surface app_radio.c expects, minus the
 * X6200 HAL. app_radio.c bakes in direct board-GPIO writes (T/R + PA relays,
 * audio-out enable/att, headphone detect, ext-mic power) through this API; this
 * shim shadows the H7 Platform/bsp_gpio.h so the clean file compiles on F427.
 *
 * GPIO_PinState / GPIO_PIN_n are HAL symbols absent from the LL-only build, so
 * they are defined here (HAL-compatible values).
 *
 * M5-0: the implementation (bsp_gpio.c) is a SAFE NO-OP -- writes touch no pin,
 * reads report the idle state -- so none of app_radio's board GPIOs are driven
 * yet. M5b backs these with the real X6100 mapping (see board_config.h).
 */
#pragma once

#include <stdint.h>
#include "stm32f4xx.h"   /* GPIO_TypeDef, GPIOA.. (CMSIS device header) */

typedef enum {
    GPIO_PIN_RESET = 0,
    GPIO_PIN_SET   = 1,
} GPIO_PinState;

/* HAL-style 16-bit pin masks (app_radio uses GPIO_PIN_13 directly). */
#define GPIO_PIN_0   ((uint16_t)0x0001)
#define GPIO_PIN_1   ((uint16_t)0x0002)
#define GPIO_PIN_2   ((uint16_t)0x0004)
#define GPIO_PIN_3   ((uint16_t)0x0008)
#define GPIO_PIN_4   ((uint16_t)0x0010)
#define GPIO_PIN_5   ((uint16_t)0x0020)
#define GPIO_PIN_6   ((uint16_t)0x0040)
#define GPIO_PIN_7   ((uint16_t)0x0080)
#define GPIO_PIN_8   ((uint16_t)0x0100)
#define GPIO_PIN_9   ((uint16_t)0x0200)
#define GPIO_PIN_10  ((uint16_t)0x0400)
#define GPIO_PIN_11  ((uint16_t)0x0800)
#define GPIO_PIN_12  ((uint16_t)0x1000)
#define GPIO_PIN_13  ((uint16_t)0x2000)
#define GPIO_PIN_14  ((uint16_t)0x4000)
#define GPIO_PIN_15  ((uint16_t)0x8000)

typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
} BspPin;

void          Bsp_Gpio_Write(BspPin pin, GPIO_PinState state);
GPIO_PinState Bsp_Gpio_Read(BspPin pin);
void          Bsp_Gpio_Toggle(BspPin pin);
