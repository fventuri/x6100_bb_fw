/* SPDX-License-Identifier: GPL-3.0-or-later */
/* Copyright (C) 2026 Franco Venturi. Part of x6100_bb_fw (GPLv3). */
/*
 * Global helper macros that upstream's App layer expects to be defined
 * everywhere. They came from a *patched* CMSIS-DSP dsp/utils.h in the X6200
 * source; the official ARM CMSIS-DSP (our pinned submodule, v1.11.0) does not
 * define MAX/MIN. This header is force-included by the build (-include) so the
 * macros are available in every translation unit, matching upstream behaviour
 * without editing the imported App files. Guarded, so a later real definition
 * wins harmlessly.
 */
#pragma once

#ifndef MIN
#define MIN(x, y) ((x) < (y) ? (x) : (y))
#endif

#ifndef MAX
#define MAX(x, y) ((x) > (y) ? (x) : (y))
#endif
