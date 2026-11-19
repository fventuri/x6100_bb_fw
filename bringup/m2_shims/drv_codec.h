/* SPDX-License-Identifier: GPL-3.0-or-later */
/* M1 spike shim: just the block-size constants the demod needs (the real
 * Devices/drv_codec.h is an AIC3204/I2S driver; not ported yet). Values match
 * upstream: 128 int32 half-words per DMA half = 64 stereo frames per block. */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#define DRV_CODEC_DMA_HALF_WORDS 128u
#define DRV_CODEC_BLOCK_FRAMES   (DRV_CODEC_DMA_HALF_WORDS / 2u)
