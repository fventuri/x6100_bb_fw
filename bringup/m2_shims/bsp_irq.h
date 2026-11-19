/* SPDX-License-Identifier: GPL-3.0-or-later */
/* M1 spike shim: HAL-free bsp_irq (the real Platform/bsp_irq.h #includes
 * stm32h7xx_hal.h; here we take the CMSIS-core intrinsics from the F4 device
 * header instead). Same critical-section semantics. */
#pragma once

#include <stdint.h>

#include "stm32f427xx.h"   /* CMSIS core intrinsics + __NVIC_PRIO_BITS (=4) */

static inline uint32_t Bsp_Irq_Save(void) {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}
static inline void Bsp_Irq_Restore(uint32_t primask) { __set_PRIMASK(primask); }

#define BSP_IRQ_LOW_BASEPRI (1u << (8u - __NVIC_PRIO_BITS))

static inline uint32_t Bsp_Irq_SaveLow(void) {
    uint32_t basepri = __get_BASEPRI();
    __set_BASEPRI(BSP_IRQ_LOW_BASEPRI);
    __DSB(); __ISB();
    return basepri;
}
static inline void Bsp_Irq_RestoreLow(uint32_t basepri) {
    __set_BASEPRI(basepri);
    __DSB(); __ISB();
}
