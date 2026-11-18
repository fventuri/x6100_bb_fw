#!/bin/sh
# SPDX-License-Identifier: GPL-3.0-or-later
# Build the M0 bring-up image (minimal boot + USART banner) for the STM32F427.
#   ./build.sh                 # expects STM32CubeF4 at ~/st/STM32CubeF4
#   CUBE=/path/to/STM32CubeF4 ./build.sh
#   HCLK_MHZ=168 ./build.sh    # in-spec safe clock instead of the OEM 250 MHz
set -e
cd "$(dirname "$0")"
. ../scripts/build_common.sh
build_firmware m0_boot m0_boot.c
