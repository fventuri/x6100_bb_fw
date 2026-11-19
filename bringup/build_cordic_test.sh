#!/bin/sh
# SPDX-License-Identifier: GPL-3.0-or-later
# Build the M1 pre-flight CORDIC-replacement accuracy test for the STM32F427.
set -e
cd "$(dirname "$0")"
. ../scripts/build_common.sh

# Pinned CMSIS-DSP submodule (ARM CMSIS-DSP v1.11.0).
GDSP=${GDSP:-../third_party/CMSIS-DSP}
DSP_INC="$GDSP/Include"
DSP_PRIV="$GDSP/PrivateInclude"
FM="$GDSP/Source/FastMathFunctions"
CT="$GDSP/Source/CommonTables"
DSP_FILES="$FM/arm_atan2_f32.c $FM/arm_sin_f32.c $FM/arm_cos_f32.c \
           $CT/arm_common_tables.c"

# -lm for libm reference (atan2f/sinf/cosf/fabsf) in the test harness.
# --gc-sections drops the unused common tables so only sinTable_f32 survives.
build_firmware m1_cordic_test \
    m1_cordic_test.c ../Platform/f427/bsp_cordic.c \
    $DSP_FILES -I"$DSP_INC" -I"$DSP_PRIV" -I../Platform -lm
