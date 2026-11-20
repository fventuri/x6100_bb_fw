#!/bin/sh
# SPDX-License-Identifier: GPL-3.0-or-later
# Build the M3 spectrum spike: the real App/SDR/sdr_spectrum.c (512-point
# CMSIS-DSP arm_cfft_f32 + Hann window) fed by the X6100 codec producer,
# replacing the M2 S-meter/noise stub. Adds the CMSIS-DSP transform sources and
# the GPLv3 arm_hanning_f32 compat (the pinned v1.11.0 submodule predates
# WindowFunctions). Reuses the M2 shims for the shared system layer
# (bsp_irq/bsp_log/drv_codec) and m3_shims for the spectrum-specific ones
# (bsp_memory neutralisation + scheduler stub). See bringup/m3_spectrum_spike.c.
set -e
cd "$(dirname "$0")"
. ../scripts/build_common.sh

APP=../App
SDR=../App/SDR
PLAT=../Platform/f427
COMPAT=../compat
M3SHIM=m3_shims
M2SHIM=m2_shims

GDSP=${GDSP:-../third_party/CMSIS-DSP}
DSP_INC="$GDSP/Include"
DSP_PRIV="$GDSP/PrivateInclude"
DSP_SRC="$GDSP/Source"

# CMSIS-DSP objects for the 512-point complex FFT + windowing/mult, plus the
# zoom IIR the spectrum uses.
DSP_FILES="\
  $DSP_SRC/TransformFunctions/arm_cfft_f32.c \
  $DSP_SRC/TransformFunctions/arm_cfft_radix8_f32.c \
  $DSP_SRC/TransformFunctions/arm_cfft_init_f32.c \
  $DSP_SRC/TransformFunctions/arm_bitreversal2.c \
  $DSP_SRC/CommonTables/arm_const_structs.c \
  $DSP_SRC/CommonTables/arm_common_tables.c \
  $DSP_SRC/BasicMathFunctions/arm_mult_f32.c \
  $DSP_SRC/SupportFunctions/arm_fill_f32.c \
  $DSP_SRC/FilteringFunctions/arm_biquad_cascade_df1_f32.c \
  $DSP_SRC/FilteringFunctions/arm_biquad_cascade_df1_init_f32.c"

# Real spectrum module + its sibling helpers (unmodified upstream).
SDR_FILES="\
  $SDR/sdr_spectrum.c \
  $SDR/sdr_iir_decim.c"

# -ICOMPAT ahead of the submocule Include so dsp/window_functions.h resolves to
# the shim; -IM3SHIM then -IM2SHIM for the system-layer shims; BSP_PROF_ENABLE=0
# compiles out the profiling hooks (no Bsp_Prof_* symbols needed).
build_firmware m3_spectrum_spike \
    -DBSP_PROF_ENABLE=0 \
    -I"$COMPAT" -I"$M3SHIM" -I"$M2SHIM" \
    -I"$SDR" -I"$APP" -I../Devices -I../Platform -I"$PLAT" \
    -I"$DSP_INC" -I"$DSP_PRIV" \
    m3_spectrum_spike.c \
    "$PLAT/drv_fpga_codec.c" \
    $SDR_FILES \
    "$M3SHIM/m3_stubs.c" \
    "$COMPAT/arm_window_compat.c" \
    $DSP_FILES -lm
