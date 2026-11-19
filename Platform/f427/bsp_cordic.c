/* SPDX-License-Identifier: GPL-3.0-or-later */
/* Copyright (C) 2026 Franco Venturi. Part of x6100_bb_fw (GPLv3). */
/*
 * bsp_cordic -- X6100 (STM32F427) replacement for the X6200's STM32H7 hardware
 * CORDIC co-processor.
 * ============================================================================
 * The F427 has no CORDIC peripheral, but it does have a single-precision FPU,
 * and the whole SDR chain is already float (CMSIS-DSP f32). So the two CORDIC
 * entry points the App layer uses (App/SDR/sdr_fm_disc.c atan2 for the FM
 * discriminator; App/SDR/sdr_nbfm.c sin/cos for the NFM TX modulator) are
 * served by CMSIS-DSP's FastMath functions, which are LUT + polynomial on the
 * FPU.
 *
 * This avoids the fixed-point <-> float conversion cost that makes the hardware
 * CORDIC a wash in practice (cf. the SimpleFOC "STM32 CORDIC co-processor"
 * thread): our data never leaves float, so there is nothing to convert.
 *
 * No STM32 HAL -- pure math over CMSIS-DSP.
 */
#include "bsp_cordic.h"

#include "arm_math.h"

/* out = atan2(y, x), matching the STM32 CORDIC's atan2(X,Y) argument order that
 * the original Bsp_Cordic_Atan2(x_arr, y_arr, ...) presented. Callers pass
 * (i, q) to get the complex phase atan2(q, i). */
void Bsp_Cordic_Atan2(const float *x_arr, const float *y_arr, float *out_atan2,
                      uint32_t size) {
    for (uint32_t n = 0; n < size; ++n) {
        float32_t r = 0.0f;
        /* arm_atan2_f32(y, x, &r): full four-quadrant, valid for all inputs;
         * returns ARM_MATH_NANINF only for (0,0), where phase is undefined and
         * 0 is the sensible value (libm atan2f(0,0) is 0 too). */
        (void)arm_atan2_f32(y_arr[n], x_arr[n], &r);
        out_atan2[n] = r;
    }
}

/* Fill out_sin[n] = sin(angles_rad[n]), out_cos[n] = cos(angles_rad[n]). */
void Bsp_Cordic_SinCos(const float *angles_rad, float *out_sin, float *out_cos,
                       uint32_t size) {
    for (uint32_t n = 0; n < size; ++n) {
        out_sin[n] = arm_sin_f32(angles_rad[n]);
        out_cos[n] = arm_cos_f32(angles_rad[n]);
    }
}
