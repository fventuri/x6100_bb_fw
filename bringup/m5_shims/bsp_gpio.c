/* SPDX-License-Identifier: GPL-3.0-or-later */
/* Copyright (C) 2026 Franco Venturi. Part of x6100_bb_fw (GPLv3). */
/*
 * bsp_gpio (F427 shim impl) -- M5-0 SAFE NO-OP. app_radio.c's board-GPIO writes
 * (T/R + PA relays, audio-out enable/att, ext-mic power) are intercepted here and
 * do nothing, so adopting app_radio for RX cannot mis-drive any X6100 pin. Reads
 * report the idle state (headphone NOT connected -> speaker path stays selected).
 * M5b replaces these bodies with the real X6100 GPIO mapping.
 */
#include "bsp_gpio.h"

void Bsp_Gpio_Write(BspPin pin, GPIO_PinState state)
{
    (void)pin;
    (void)state;   /* M5-0: no pin driven */
}

GPIO_PinState Bsp_Gpio_Read(BspPin pin)
{
    (void)pin;
    return GPIO_PIN_SET;   /* idle-high: e.g. HEADPHONE_DETECT inactive */
}

void Bsp_Gpio_Toggle(BspPin pin)
{
    (void)pin;
}
