#!/bin/sh
# SPDX-License-Identifier: GPL-3.0-or-later
# Build the M4a integrated RX host firmware: the proven X6100 hardware bring-up
# (VersaClock/codec/RF/LO/I2S/DSP, from m3_rx_radio) driving the UNMODIFIED clean
# App host-link stack (App/app_hostlink.c + app_radio_state.c) over real X6100
# backends (App_Fpga_*/App_RadioRfPath_*/App_Radio_* in m4_rx_host.c) and the
# real no-HAL OEM I2C3 register slave (m4_backends/bsp_i2c_slave.c). The stock
# Linux host (or phase4-linux-ctl.py) drives tune/mode/vol/AGC. Produces
# m4_rx_host.elf/.bin AND m4_rx_host.xgf (flashable; Franco flashes). See
# bringup/m4_rx_host.c.
set -e
cd "$(dirname "$0")"
. ../scripts/build_common.sh

APP=../App
SDR=../App/SDR
PLAT=../Platform/f427
DEV=../Devices
COMPAT=../compat
M4SHIM=m4_shims

GDSP=${GDSP:-../third_party/CMSIS-DSP}
DSP_INC="$GDSP/Include"
DSP_PRIV="$GDSP/PrivateInclude"
DSP_SRC="$GDSP/Source"

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

# Real upstream RX datapath + spectrum + the CW/NR/compressor modules the
# host-link state machine configures (m3 stubbed these; M4 links them real).
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
  $SDR/sdr_compressor.c \
  $SDR/sdr_audio_resample.c \
  $SDR/sdr_spectrum.c"

# Platform backend: RX-producer getters (drv_fpga_codec) + CORDIC for the DSP.
PLAT_FILES="\
  $PLAT/drv_fpga_codec.c \
  $PLAT/bsp_cordic.c \
  $PLAT/i2s.c \
  $PLAT/i2s_rx_path.c"

# The real no-HAL X6100 I2C3 OEM-register slave (replaces the HAL Platform one).
M4BACK=m4_backends

# The clean App host-link stack (UNMODIFIED) + the real event bus + scheduler.
APP_FILES="\
  $APP/app_hostlink.c \
  $APP/app_radio_state.c \
  $APP/app_event.c \
  $APP/app_scheduler.c"

build_firmware m4_rx_host \
    -DBSP_PROF_ENABLE=0 \
    -I"$COMPAT" -I"$M4SHIM" -I"$M4BACK" \
    -I"$SDR" -I"$APP" -I"$DEV" -I"$PLAT" -Im2_shims -I../Platform \
    -I"$DSP_INC" -I"$DSP_PRIV" \
    m4_rx_host.c \
    $PLAT_FILES \
    $APP_FILES \
    $SDR_FILES \
    "$M4SHIM/m4_stubs_common.c" "$M4BACK/bsp_i2c_slave.c" "$M4BACK/flow_uplink.c" \
    m3_shims/m3_app_stubs.c \
    "$COMPAT/arm_window_compat.c" \
    $DSP_FILES -lm

xgf_encrypt m4_rx_host
