/* SPDX-License-Identifier: GPL-3.0-or-later */
/* Copyright (C) 2026 Franco Venturi. Part of x6100_bb_fw (GPLv3). */
/*
 * bsp_i2c_bus_f427 -- F427 backend for the Bsp_I2c_* bus abstraction that
 * drv_gauge (MAX17205) and drv_charger (BQ24725) use, implemented over the
 * shared blocking i2c.c master. Only BUS_1 (I2C1) and BUS_2 (I2C2) exist on the
 * X6100; the gauge/charger live on BUS_2.
 *
 * Concurrency: the M5 build is single-threaded cooperative -- the DSP runs in
 * the I2S ISRs and never touches I2C2, and every I2C2 user (codec backend +
 * the app_protect sample burst) runs in the main loop via the scheduler. So the
 * bus-claim/guard calls are no-ops here (nothing can preempt an I2C2 transfer),
 * and the timeout arguments are advisory: i2c.c already bounds every transfer
 * internally and returns -1 rather than hanging on a stuck slave.
 *
 * i2c.c shapes used: WriteRead with a 1-byte register pointer -> i2c_read()
 * (write-reg / repeated-START / read-N); raw write -> i2c_write_raw(). The
 * gauge and charger only ever issue those two shapes.
 */
#include "bsp_i2c_bus.h"

#include "i2c.h"

static I2C_TypeDef *bus_to_i2c(BspI2cBus bus) {
    switch (bus) {
    case BSP_I2C_BUS_1:
        return I2C1;
    case BSP_I2C_BUS_2:
        return I2C2;
    default:
        return 0;
    }
}

int Bsp_I2c_Write(BspI2cBus bus, uint8_t addr, const uint8_t *data, size_t len) {
    I2C_TypeDef *i2c = bus_to_i2c(bus);

    if (i2c == 0 || data == 0) {
        return -1;
    }
    return i2c_write_raw(i2c, addr, data, (uint32_t)len);
}

int Bsp_I2c_WriteRead(BspI2cBus bus, uint8_t addr, const uint8_t *tx,
                      size_t tx_len, uint8_t *rx, size_t rx_len) {
    I2C_TypeDef *i2c = bus_to_i2c(bus);

    if (i2c == 0 || tx == 0 || rx == 0 || tx_len != 1u) {
        /* The gauge/charger always write exactly one register-pointer byte. */
        return -1;
    }
    return i2c_read(i2c, addr, tx[0], rx, (uint32_t)rx_len);
}

int Bsp_I2c_Read(BspI2cBus bus, uint8_t addr, uint8_t *data, size_t len) {
    (void)bus;
    (void)addr;
    (void)data;
    (void)len;
    /* No register-less read shape is used by the gauge/charger. */
    return -1;
}

int Bsp_I2c_WriteTimeout(BspI2cBus bus, uint8_t addr, const uint8_t *data,
                         size_t len, uint32_t timeout_ms) {
    (void)timeout_ms;
    return Bsp_I2c_Write(bus, addr, data, len);
}

int Bsp_I2c_WriteReadTimeout(BspI2cBus bus, uint8_t addr, const uint8_t *tx,
                             size_t tx_len, uint8_t *rx, size_t rx_len,
                             uint32_t timeout_ms) {
    (void)timeout_ms;
    return Bsp_I2c_WriteRead(bus, addr, tx, tx_len, rx, rx_len);
}

/* Single-threaded cooperative build: nothing can preempt an I2C2 transfer, so
 * the ownership claim and guards are no-ops. */
void Bsp_I2c_Bus2Claim(void) {}
void Bsp_I2c_Bus2GuardEnter(void) {}
void Bsp_I2c_Bus2GuardExit(void) {}
