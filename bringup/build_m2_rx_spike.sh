#!/bin/sh
# SPDX-License-Identifier: GPL-3.0-or-later
# Build the M2 RX transport spike: the real, unmodified upstream RX pipeline
# (App/SDR/sdr_rx.c + decimation + AM demod + audio resamplers) fed by the
# X6100 codec producer backend (Platform/f427/drv_fpga_codec.c), with HAL-free
# shims for the not-yet-ported system layer (bsp_irq/bsp_log/drv_codec/
# app_event) and stubs for the modules outside the AM datapath (spectrum,
# noise reduction). See bringup/m2_rx_spike.c.
set -e
cd "$(dirname "$0")"
. ../scripts/build_common.sh

APP=../App
SDR=../App/SDR
PLAT=../Platform/f427
SHIM=m2_shims

GDSP=${GDSP:-../third_party/CMSIS-DSP}
DSP_INC="$GDSP/Include"
DSP_PRIV="$GDSP/PrivateInclude"
DSP_SRC="$GDSP/Source"

# CMSIS-DSP objects referenced across the compiled RX set + software CORDIC.
DSP_FILES="\
  $DSP_SRC/FastMathFunctions/arm_atan2_f32.c \
  $DSP_SRC/FastMathFunctions/arm_cos_f32.c \
  $DSP_SRC/FastMathFunctions/arm_sin_f32.c \
  $DSP_SRC/FilteringFunctions/arm_biquad_cascade_df1_f32.c \
  $DSP_SRC/FilteringFunctions/arm_biquad_cascade_df1_init_f32.c \
  $DSP_SRC/FilteringFunctions/arm_fir_f32.c \
  $DSP_SRC/FilteringFunctions/arm_fir_init_f32.c \
  $DSP_SRC/FilteringFunctions/arm_fir_interpolate_f32.c \
  $DSP_SRC/FilteringFunctions/arm_fir_interpolate_init_f32.c \
  $DSP_SRC/SupportFunctions/arm_copy_f32.c \
  $DSP_SRC/SupportFunctions/arm_fill_f32.c \
  $DSP_SRC/BasicMathFunctions/arm_scale_f32.c \
  $DSP_SRC/BasicMathFunctions/arm_offset_f32.c \
  $DSP_SRC/StatisticsFunctions/arm_power_f32.c \
  $DSP_SRC/CommonTables/arm_common_tables.c"

# Real upstream RX datapath modules (unmodified).
SDR_FILES="\
  $SDR/sdr_rx.c \
  $SDR/sdr_nb.c \
  $SDR/sdr_resample.c \
  $SDR/sdr_halfband_decim.c \
  $SDR/sdr_iir_decim.c \
  $SDR/sdr_iir.c \
  $SDR/sdr_am.c \
  $SDR/sdr_ssb.c \
  $SDR/sdr_sideband.c \
  $SDR/sdr_nbfm.c \
  $SDR/sdr_fm_disc.c \
  $SDR/sdr_wfm.c \
  $SDR/sdr_agc.c \
  $SDR/sdr_notch.c \
  $SDR/sdr_squelch.c \
  $SDR/sdr_audio_resample.c"

# Shims (-I$SHIM first so app_event/bsp_*/drv_codec resolve to the shims).
build_firmware m2_rx_spike \
    -I"$SHIM" -I"$SDR" -I"$APP" -I../Devices -I../Platform -I"$PLAT" \
    -I"$DSP_INC" -I"$DSP_PRIV" \
    m2_rx_spike.c \
    "$PLAT/drv_fpga_codec.c" "$PLAT/bsp_cordic.c" \
    $SDR_FILES \
    "$SHIM/m2_stubs.c" \
    $DSP_FILES -lm
