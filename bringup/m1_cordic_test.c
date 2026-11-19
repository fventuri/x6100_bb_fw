/* SPDX-License-Identifier: GPL-3.0-or-later */
/* Copyright (C) 2026 Franco Venturi. Part of x6100_bb_fw (GPLv3). */
/*
 * M1 pre-flight: validate the software CORDIC replacement (Platform/f427/
 * bsp_cordic.c) on the F427 model, against libm, before porting any App demod.
 *
 * Sweeps the full circle and reports the worst-case error of Bsp_Cordic_Atan2()
 * vs libm atan2f and of Bsp_Cordic_SinCos() vs libm sinf/cosf. Errors are
 * printed scaled to integers (micro-radians / parts-per-million) so the banner
 * needs no float printf (newlib-nano).
 */
#include <math.h>
#include <stdint.h>

#include "bsp.h"
#include "bsp_cordic.h"

#define N 1024u

static float s_ang[N];
static float s_i[N], s_q[N];
static float s_sin[N], s_cos[N];
static float s_atan[N];

int main(void) {
    bsp_gpio_init_safe();
    bsp_clock_init_250mhz();
    bsp_systick_init_1ms();
    bsp_usart1_init();

    bsp_uprintf("\r\n=== x6100_bb_fw M1 CORDIC-replacement test ===\r\n");
    bsp_uprintf("CMSIS-DSP arm_atan2_f32 / arm_sin_f32 / arm_cos_f32 on M4F\r\n");

    /* Build a full-circle sweep of unit-ish vectors with varied magnitude. */
    for (uint32_t k = 0; k < N; ++k) {
        float t = -(float)M_PI + (2.0f * (float)M_PI) * (float)k / (float)N;
        float mag = 0.25f + 1.75f * (float)k / (float)N; /* 0.25 .. 2.0 */
        s_ang[k] = t;
        s_i[k]   = mag * cosf(t);
        s_q[k]   = mag * sinf(t);
    }

    /* atan2: out = atan2(q, i); compare to libm atan2f(q, i). */
    Bsp_Cordic_Atan2(s_i, s_q, s_atan, N);
    long atan_err_urad = 0;
    for (uint32_t k = 0; k < N; ++k) {
        float ref = atan2f(s_q[k], s_i[k]);
        float e = fabsf(s_atan[k] - ref);
        long urad = (long)(e * 1.0e6f + 0.5f);
        if (urad > atan_err_urad) atan_err_urad = urad;
    }

    /* sin/cos over the same angles; compare to libm. */
    Bsp_Cordic_SinCos(s_ang, s_sin, s_cos, N);
    long sin_err_ppm = 0, cos_err_ppm = 0;
    for (uint32_t k = 0; k < N; ++k) {
        float es = fabsf(s_sin[k] - sinf(s_ang[k]));
        float ec = fabsf(s_cos[k] - cosf(s_ang[k]));
        long ps = (long)(es * 1.0e6f + 0.5f);
        long pc = (long)(ec * 1.0e6f + 0.5f);
        if (ps > sin_err_ppm) sin_err_ppm = ps;
        if (pc > cos_err_ppm) cos_err_ppm = pc;
    }

    bsp_uprintf("samples: %u over [-pi, +pi], |vec| 0.25..2.0\r\n", (unsigned)N);
    bsp_uprintf("atan2 max err : %ld urad  (%ld.%03ld mrad)\r\n",
                atan_err_urad, atan_err_urad / 1000, atan_err_urad % 1000);
    bsp_uprintf("sin   max err : %ld ppm (of full scale)\r\n", sin_err_ppm);
    bsp_uprintf("cos   max err : %ld ppm (of full scale)\r\n", cos_err_ppm);

    bsp_uprintf("DONE\r\n");
    for (;;) { bsp_delay_ms(1000); }
}
