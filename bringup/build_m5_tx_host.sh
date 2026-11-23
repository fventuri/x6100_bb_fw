#!/bin/sh
# SPDX-License-Identifier: GPL-3.0-or-later
# Build the M5-0 TX-host firmware: the proven X6100 hardware bring-up
# (VersaClock/codec/RF/LO/I2S, from m4_rx_host) now driving the UNMODIFIED clean
# App DSP ORCHESTRATOR App/app_radio.c as the single RX+TX dispatcher, over F427
# backends: the F427 Drv_Codec (in m5_tx_host.c), the i2s_rx_path_m5 bridge that
# runs app_radio's radio_block_cb each synced I2S bank, bsp_board_f427 (TX/PA
# signals safe-OFF), drv_fpga_tx (TX I/Q accepted+discarded -> no RF), the m5
# TX-side stubs, the m5_shims (bsp_gpio no-op + main.h pin macros), and the
# unchanged OEM host-link stack + I2C3 slave + flow uplink from M4. M5-0 is
# RX-only (TX hard-inhibited); it re-validates AM RX through app_radio. Produces
# m5_tx_host.elf/.bin AND m5_tx_host.xgf (flashable; Franco flashes).
set -e
cd "$(dirname "$0")"
. ../scripts/build_common.sh

APP=../App
SDR=../App/SDR
PLAT=../Platform/f427
DEV=../Devices
COMPAT=../compat
M5SHIM=m5_shims
M4SHIM=m4_shims
M5BACK=m5_backends
M4BACK=m4_backends

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
# host-link state machine configures (CW/NR DSP stays stubbed; sdr_tx stubbed).
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

# Platform backend: RX-producer getters (drv_fpga_codec) + CORDIC + the I2S
# transport. NOTE: Platform/f427/i2s_rx_path.c is DROPPED -- the m5_backends
# i2s_rx_path_m5.c bridge replaces it (both define i2s_dsp/g_rx_volume/
# I2s_Rx_Path_Init).
PLAT_FILES="\
  $PLAT/drv_fpga_codec.c \
  $PLAT/bsp_cordic.c \
  $PLAT/i2s.c"

# The clean App DSP orchestrator (app_radio.c, UNMODIFIED) + host-link stack +
# the real event bus + scheduler.
APP_FILES="\
  $APP/app_radio.c \
  $APP/app_hostlink.c \
  $APP/app_radio_state.c \
  $APP/app_event.c \
  $APP/app_scheduler.c \
  $APP/app_protect.c"

# F427 backends for the app_radio surface + the m5 TX-side stubs.
M5BACK_FILES="\
  $M5BACK/bsp_board_f427.c \
  $M5BACK/drv_fpga_tx.c \
  $M5BACK/m5_stubs.c \
  $M5BACK/i2s_rx_path_m5.c \
  $M5BACK/bsp_timer_sched.c \
  $M5BACK/bsp_i2c_bus_f427.c \
  $M5SHIM/bsp_gpio.c"

build_firmware m5_tx_host \
    -DBSP_PROF_ENABLE=0 \
    -DX6100_M5_REAL_PROTECT \
    -I"$COMPAT" -I"$M5SHIM" -I"$M4SHIM" -I"$M4BACK" \
    -I"$SDR" -I"$APP" -I"$DEV" -I"$PLAT" -Im2_shims -I../Platform \
    -I"$DSP_INC" -I"$DSP_PRIV" \
    m5_tx_host.c \
    $PLAT_FILES \
    $APP_FILES \
    $M5BACK_FILES \
    "$DEV/drv_adc_mon.c" \
    "$DEV/drv_gauge.c" \
    $SDR_FILES \
    "$M4SHIM/m4_stubs_common.c" "$M4BACK/bsp_i2c_slave.c" "$M4BACK/flow_uplink.c" \
    m3_shims/m3_app_stubs.c \
    "$COMPAT/arm_window_compat.c" \
    $DSP_FILES -lm

xgf_encrypt m5_tx_host
