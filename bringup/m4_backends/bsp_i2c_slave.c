/* SPDX-License-Identifier: GPL-3.0-or-later */
/* Copyright (C) 2026 Franco Venturi. Part of x6100_bb_fw (GPLv3). */
/*
 * bsp_i2c_slave (F427, no-HAL) -- OEM-compatible I2C3 register-window slave.
 * ============================================================================
 * Implements the Platform/bsp_i2c_slave.h transaction contract on the STM32F427
 * with direct register I2C (the shared Platform/bsp_i2c_slave.c is the H7/HAL
 * variant; this one replaces it, honoring the no-HAL rule). It is the downlink
 * transport for App/app_hostlink: the stock Xiegu R2/Linux app (or
 * phase4-linux-ctl.py) drives the base-band through the OEM BASE 1.1.9 16-bit
 * register file.
 *
 * Electrically identical to the proven phase3/phase4 slave: 7-bit 0x72
 * (OAR1 = 0x00E4), PA8 = I2C3_SCL, PC9 = I2C3_SDA, AF4, open-drain, pulled up.
 *
 * Wire format (OEM, docs/protocol.md):
 *   write [0x72+W][reg-hi][reg-lo][payload LE...]
 *   read  [0x72+W][reg-hi][reg-lo][Sr][0x72+R][data...]
 * The register offset is latched from the two address bytes and PERSISTS across
 * the STOP of an offset-only write, so a write-offset / STOP / read sequence
 * reads from the previously selected window.
 *
 * Bridging byte-oriented hardware to the transaction callbacks: payload bytes of
 * a write are accumulated and dispatched to on_write(reg, buf, len) once, on the
 * terminating STOP (a register-only write -- the read offset select -- carries
 * no payload and is NOT dispatched). A read builds its whole reply by calling
 * on_read(reg, buf, cap) once at the read address-match, then clocks it out byte
 * by byte (zero padding beyond the reply, matching the HAL variant's semantics
 * app_hostlink is written against).
 *
 * The callbacks run in the I2C3 event ISR (priority 6, below the DSP ISR; the
 * slave clock-stretches), so they must not block -- app_hostlink only latches
 * flags and copies there.
 */

#include "bsp_i2c_slave.h"

#include <stdbool.h>
#include <stddef.h>

#include "bsp.h" /* LL + GPIO macros (M_AF/M_OD/PULL_UP/SPD_HI), PCLK1_HZ */

#define I2C3_ADDR8 0xE4u /* 7-bit 0x72 in the OAR1 shifted form */

/* Register address + the largest served region (calib 507 B) with headroom,
 * matching the HAL variant. */
#define BSP_I2C_SLAVE_BUF_LEN 520u

static BspI2cSlaveWriteFn s_on_write;
static BspI2cSlaveReadFn  s_on_read;

/* Latched byte offset; PERSISTS across STOP (OEM offset-write/read contract). */
static volatile uint16_t s_reg;
/* Write framing phase: 0 = expect reg-hi, 1 = expect reg-lo, 2 = payload. */
static volatile uint8_t  s_phase;

static uint8_t          s_wbuf[BSP_I2C_SLAVE_BUF_LEN];
static volatile uint16_t s_wlen;

static uint8_t          s_rbuf[BSP_I2C_SLAVE_BUF_LEN];
static volatile uint16_t s_rlen;
static volatile uint16_t s_rpos;

static void i2c3_slave_hw_init(void) {
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA |
                             LL_AHB1_GRP1_PERIPH_GPIOC);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C3);

    /* PA8 = I2C3_SCL, PC9 = I2C3_SDA: AF4, open-drain, pulled up. */
    M_AF(GPIOA, 8);
    M_OD(GPIOA, 8);
    PULL_UP(GPIOA, 8);
    SPD_HI(GPIOA, 8);
    LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_8, LL_GPIO_AF_4);
    M_AF(GPIOC, 9);
    M_OD(GPIOC, 9);
    PULL_UP(GPIOC, 9);
    SPD_HI(GPIOC, 9);
    LL_GPIO_SetAFPin_8_15(GPIOC, LL_GPIO_PIN_9, LL_GPIO_AF_4);

    I2C3->CR1 = I2C_CR1_SWRST;
    I2C3->CR1 = 0;
    I2C3->CR2 = (PCLK1_HZ / 1000000u) & I2C_CR2_FREQ; /* APB1 clock in MHz */
    I2C3->OAR1 = (1u << 14) | I2C3_ADDR8;             /* 7-bit 0x72, bit14=1 */
    I2C3->OAR2 = 0;
    I2C3->CR1 = I2C_CR1_PE;
    I2C3->CR1 |= I2C_CR1_ACK;
    I2C3->CR2 |= I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN | I2C_CR2_ITERREN;

    NVIC_SetPriority(I2C3_EV_IRQn, 6); /* below the DSP ISR; slave stretches */
    NVIC_EnableIRQ(I2C3_EV_IRQn);
    NVIC_SetPriority(I2C3_ER_IRQn, 6);
    NVIC_EnableIRQ(I2C3_ER_IRQn);
}

int Bsp_I2cSlave_Init(BspI2cSlaveWriteFn on_write, BspI2cSlaveReadFn on_read) {
    if (on_write == NULL || on_read == NULL) {
        return -1;
    }
    s_on_write = on_write;
    s_on_read  = on_read;
    s_reg      = 0u;
    s_phase    = 0u;
    s_wlen     = 0u;
    s_rlen     = 0u;
    s_rpos     = 0u;
    i2c3_slave_hw_init();
    return 0;
}

/* I2C3 event ISR: overrides the weak vector in Platform/f427/startup.c. */
void I2C3_EV_IRQHandler(void) {
    uint32_t sr1 = I2C3->SR1;

    if (sr1 & I2C_SR1_ADDR) {
        (void)I2C3->SR1;
        uint32_t sr2 = I2C3->SR2; /* reading SR1 then SR2 clears ADDR */
        if (sr2 & I2C_SR2_TRA) {
            /* Master reads from us: build the whole reply from the persisted
             * offset now, clock it out on the TXE events below. */
            size_t n = 0u;
            if (s_on_read != NULL) {
                n = s_on_read(s_reg, s_rbuf, sizeof(s_rbuf));
            }
            if (n == 0u) {
                s_rbuf[0] = 0u;
                n = 1u;
            }
            s_rlen = (uint16_t)n;
            s_rpos = 0u;
        } else {
            /* Master writes to us: the next two bytes re-select the offset. */
            s_phase = 0u;
            s_wlen  = 0u;
        }
    }

    if (sr1 & I2C_SR1_RXNE) {
        uint8_t b = (uint8_t)I2C3->DR;
        if (s_phase == 0u) {
            s_reg   = (uint16_t)b << 8;
            s_phase = 1u;
        } else if (s_phase == 1u) {
            s_reg |= b;
            s_phase = 2u;
            s_wlen  = 0u;
        } else if (s_wlen < sizeof(s_wbuf)) {
            s_wbuf[s_wlen++] = b;
        }
    }

    if ((sr1 & I2C_SR1_TXE) && !(sr1 & I2C_SR1_AF)) {
        I2C3->DR = (s_rpos < s_rlen) ? s_rbuf[s_rpos++] : 0x00u;
    }

    if (sr1 & I2C_SR1_STOPF) {
        (void)I2C3->SR1;
        I2C3->CR1 |= I2C_CR1_PE;
        /* End of a write: dispatch the accumulated payload once. A register-only
         * write (offset select for a following read) carries no payload and is
         * not dispatched; s_reg is intentionally kept across the STOP. */
        if (s_phase == 2u && s_wlen > 0u && s_on_write != NULL) {
            s_on_write(s_reg, s_wbuf, s_wlen);
        }
        s_phase = 0u;
        s_wlen  = 0u;
    }
}

/* I2C3 error ISR: AF is the normal end-of-read NACK from the master. */
void I2C3_ER_IRQHandler(void) {
    I2C3->SR1 &= ~(I2C_SR1_AF | I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_OVR);
}
