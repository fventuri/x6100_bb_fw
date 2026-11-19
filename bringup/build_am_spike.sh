#!/bin/sh
# SPDX-License-Identifier: GPL-3.0-or-later
# Build the M1 AM-demod spike: upstream App/SDR/sdr_am.c + sdr_iir.c (unmodified)
# on the F427 with CMSIS-DSP and HAL-free shims for the not-yet-ported layer.
set -e
cd "$(dirname "$0")"
. ../scripts/build_common.sh

APP=../App
SDR=../App/SDR
SHIM=m1_shims

# Modular CMSIS-DSP from the pinned submodule (ARM CMSIS-DSP v1.11.0). MAX/MIN
# come from compat/cmsis_dsp_compat.h (force-included by build_common.sh).
GDSP=${GDSP:-../third_party/CMSIS-DSP}
DSP_INC="$GDSP/Include"
DSP_PRIV="$GDSP/PrivateInclude"
DSP_SRC="$GDSP/Source"

# CMSIS-DSP objects the AM path needs (biquad df1 f32 + init, fill).
DSP_FILES="$DSP_SRC/FilteringFunctions/arm_biquad_cascade_df1_f32.c \
           $DSP_SRC/FilteringFunctions/arm_biquad_cascade_df1_init_f32.c \
           $DSP_SRC/SupportFunctions/arm_fill_f32.c"

# Shims shadow the real system headers (bsp_irq/bsp_log/drv_codec/app_event);
# -I$SHIM must come first. CMSIS core/device come from build_common's CFLAGS.
build_firmware m1_am_spike \
    -I"$SHIM" -I"$SDR" -I"$APP" -I../Platform -I../Platform/f427 \
    -I"$DSP_INC" -I"$DSP_PRIV" \
    m1_am_spike.c \
    "$SDR/sdr_am.c" "$SDR/sdr_iir.c" \
    "$SHIM/m1_stubs.c" \
    $DSP_FILES -lm
