/* SPDX-License-Identifier: GPL-3.0-or-later */
/* Copyright (C) 2026 Franco Venturi. Part of x6100_bb_fw (GPLv3). */
/*
 * main.h (F427 shim) -- app_radio.c includes "main.h", the X6200 CubeMX header
 * that (besides HAL handles) defines the board GPIO pin macros app_radio.c writes
 * directly (Core/Inc/main.h on the X6200). On F427 there are no HAL handles, so
 * this shim provides ONLY those pin macros. It must NOT shadow board_config.h:
 * the real Platform/board_config.h carries BOARD_DEFAULT_FREQ_HZ / the protection
 * thresholds / BOARD_AUDIO_SAMPLE_RATE_HZ that app_radio_state.c and app_ptt.c
 * need, so the pins live here instead.
 *
 * M5-0: every macro points at a dummy-safe pin and Bsp_Gpio is a no-op
 * (bsp_gpio.c), so none of these is ever driven. The TX-only relays
 * (RF_PA_RELAY_2 / T_R_RELAY_2) are written only in TX mode, which M5-0 never
 * enters. M5b replaces these with the real X6100 mapping (T/R PE8/PG11 TX vs PE9
 * RX + 5T/5R rails; see x6100-base/docs/pa_control.md) and a real Bsp_Gpio.
 */
#pragma once

#include "bsp_gpio.h"   /* GPIO_TypeDef, GPIOx, GPIO_PIN_n, GPIO_PinState */

/* Dummy-safe placeholder (never driven while Bsp_Gpio is a no-op). */
#define M5_DUMMY_GPIO_PORT   GPIOA
#define M5_DUMMY_GPIO_PIN    GPIO_PIN_0

#define AUDIO_OUT_EN_GPIO_Port     M5_DUMMY_GPIO_PORT
#define AUDIO_OUT_EN_Pin           M5_DUMMY_GPIO_PIN
#define AUDIO_OUT_ATT_GPIO_Port    M5_DUMMY_GPIO_PORT
#define AUDIO_OUT_ATT_Pin          M5_DUMMY_GPIO_PIN
#define HEADPHONE_DETECT_GPIO_Port M5_DUMMY_GPIO_PORT
#define HEADPHONE_DETECT_Pin       M5_DUMMY_GPIO_PIN
#define RF_PA_RELAY_2_GPIO_Port    M5_DUMMY_GPIO_PORT
#define RF_PA_RELAY_2_Pin          M5_DUMMY_GPIO_PIN
#define T_R_RELAY_2_GPIO_Port      M5_DUMMY_GPIO_PORT
#define T_R_RELAY_2_Pin            M5_DUMMY_GPIO_PIN
