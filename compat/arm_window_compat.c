/* SPDX-License-Identifier: GPL-3.0-or-later */
/* Copyright (C) 2026 Franco Venturi. Part of x6100_bb_fw (GPLv3). */
/*
 * arm_hanning_f32 -- fills the CMSIS-DSP WindowFunctions gap in the pinned
 * v1.11.0 submodule (see compat/dsp/window_functions.h). Independent GPLv3
 * implementation of the documented periodic-Hann formula; not derived from the
 * Apache-2.0 CMSIS source.
 */
#include "dsp/window_functions.h"

#include <math.h>

/* Periodic Hann: pDst[i] = 0.5*(1 - cos(2*pi*i/blockSize)). k = 2/blockSize so
 * the cosine argument is PI*i*k == 2*pi*i/blockSize, matching arm_hanning_f32. */
void arm_hanning_f32(float32_t *pDst, uint32_t blockSize)
{
    const float32_t k = 2.0f / (float32_t)blockSize;
    for (uint32_t i = 0u; i < blockSize; i++) {
        pDst[i] = 0.5f * (1.0f - cosf((float32_t)M_PI * (float32_t)i * k));
    }
}
