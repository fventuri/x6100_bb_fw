#!/bin/sh
# SPDX-License-Identifier: GPL-3.0-or-later
# Build the M3 RX radio: the first flashable integrated receiver. Proven X6100
# hardware bring-up (VersaClock/codec/RF/LO, ported from x6100-bbfw phase3-rx)
# feeding the ported Georgy pipeline (sdr_rx + real sdr_spectrum + half-band
# audio out) via the i2s_rx_path strong i2s_dsp() override.
#
# Produces m3_rx_radio.elf/.bin AND m3_rx_radio.xgf (AES-encrypted Xiegu IAP
# image). Franco flashes the .xgf; this script never touches the radio.
#
# Uses the REAL Platform/bsp_memory.h (the .ram_d1_bss section is now in
# stm32f427.ld) and the GPLv3 arm_hanning_f32 compat; m2_shims supply the
# not-yet-ported system layer (app_event/bsp_irq/bsp_log/drv_codec), m3_shims
# the app-event/NR/CW stubs and the scheduler capture. See bringup/m3_rx_radio.c.
set -e
cd "$(dirname "$0")"
. ../scripts/build_common.sh

APP=../App
SDR=../App/SDR
PLAT=../Platform/f427
COMPAT=../compat
M2SHIM=m2_shims
M3SHIM=m3_shims

GDSP=${GDSP:-../third_party/CMSIS-DSP}
DSP_INC="$GDSP/Include"
DSP_PRIV="$GDSP/PrivateInclude"
DSP_SRC="$GDSP/Source"

# CMSIS-DSP objects: RX datapath + software CORDIC + the 512-pt spectrum FFT.
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
  $DSP_SRC/BasicMathFunctions/arm_mult_f32.c \
  $DSP_SRC/StatisticsFunctions/arm_power_f32.c \
  $DSP_SRC/TransformFunctions/arm_cfft_f32.c \
  $DSP_SRC/TransformFunctions/arm_cfft_radix8_f32.c \
  $DSP_SRC/TransformFunctions/arm_cfft_init_f32.c \
  $DSP_SRC/TransformFunctions/arm_bitreversal2.c \
  $DSP_SRC/CommonTables/arm_const_structs.c \
  $DSP_SRC/CommonTables/arm_common_tables.c"

# Real upstream RX datapath + spectrum (unmodified).
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
  $SDR/sdr_audio_resample.c \
  $SDR/sdr_spectrum.c"

# Platform backend under test.
PLAT_FILES="\
  $PLAT/drv_fpga_codec.c \
  $PLAT/bsp_cordic.c \
  $PLAT/i2s.c \
  $PLAT/i2s_rx_path.c"

# Real App infrastructure (event bus + cooperative scheduler), replacing the
# spike stubs so RxCfg propagates to the AGC/squelch/demods and the spectrum
# task runs on a real time base.
APP_FILES="\
  $APP/app_event.c \
  $APP/app_scheduler.c"

# -ICOMPAT ahead of the submodule Include (dsp/window_functions.h shim).
# -I$PLAT BEFORE -I../Platform so the F427 bsp_time.h wins over the H7/HAL one
# (bsp_memory.h/bsp_prof.h exist only under Platform/, so they still resolve
# real). BSP_PROF_ENABLE=0 drops profiling. m3_app_stubs.c = NR/CW only (event
# bus + scheduler + spectrum are linked real).
build_firmware m3_rx_radio \
    -DBSP_PROF_ENABLE=0 \
    -I"$COMPAT" -I"$M2SHIM" \
    -I"$SDR" -I"$APP" -I../Devices -I"$PLAT" -I../Platform \
    -I"$DSP_INC" -I"$DSP_PRIV" \
    m3_rx_radio.c \
    $PLAT_FILES \
    $APP_FILES \
    $SDR_FILES \
    "$M3SHIM/m3_app_stubs.c" \
    "$COMPAT/arm_window_compat.c" \
    $DSP_FILES -lm

xgf_encrypt m3_rx_radio
