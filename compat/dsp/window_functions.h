/* SPDX-License-Identifier: GPL-3.0-or-later */
/* Copyright (C) 2026 Franco Venturi. Part of x6100_bb_fw (GPLv3). */
/*
 * CMSIS-DSP window-function shim.
 * ============================================================================
 * The pinned CMSIS-DSP submodule is v1.11.0, which predates the WindowFunctions
 * group (no Source/WindowFunctions, no Include/dsp/window_functions.h). But
 * App/SDR/sdr_spectrum.c does `#include "dsp/window_functions.h"` and calls
 * arm_hanning_f32(). This header satisfies that include (compat/ is placed
 * ahead of the submodule Include on the -I path); the definition is the GPLv3
 * reimplementation in compat/arm_window_compat.c. Only arm_hanning_f32 is
 * declared -- the only window the port uses. Mirrors the MAX/MIN gap handled by
 * compat/cmsis_dsp_compat.h.
 */
#pragma once

#include "arm_math_types.h"   /* float32_t */

#ifdef __cplusplus
extern "C" {
#endif

/* Periodic (DFT-even) Hann window: pDst[i] = 0.5*(1 - cos(2*pi*i/blockSize)),
 * i = 0 .. blockSize-1. Matches the CMSIS-DSP arm_hanning_f32 definition. */
void arm_hanning_f32(float32_t *pDst, uint32_t blockSize);

#ifdef __cplusplus
}
#endif
