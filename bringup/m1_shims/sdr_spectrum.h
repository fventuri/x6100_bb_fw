/* SPDX-License-Identifier: GPL-3.0-or-later */
/* M1 spike shim: the three spectrum/noise accessors the AM squelch metric
 * calls. Stubbed as "no valid noise estimate" so the metric reads 0; the real
 * FFT spectrum (App/SDR/sdr_spectrum.c) is a later milestone. */
#pragma once

#include <stdbool.h>

float Sdr_Spectrum_Get_Noise_Lin(void);
bool  Sdr_Spectrum_Is_Noise_Valid(void);
float Sdr_Spectrum_Get_Demod_Noise_Est_Db(void);
